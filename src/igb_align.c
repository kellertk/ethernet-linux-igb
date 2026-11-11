// SPDX-License-Identifier: GPL-2.0
/* Board 2049: The goal is to set the time of day in the PHC and align it with
 * the GNSS pulse. The LMK05318B provides both the 25 MHz clock and the 1PPS,
 * so adjusting the LMK's phase will lock the PHC to GNSS. Startup can leave the
 * LMK in an undefined state, necessitating alignment with the GNSS pulse.
 *
 * Two loops run from the DPLL worker on extts captures stored by
 * igb_tsync_interrupt. The PHC loop steps the PHC close to the GNSS pulse and
 * holds it there. The PPS loop measures the PPS output and pulls it into phase
 * with DPLL_REF_SYNC_PH_OFFSET (R340-R345). During a pull event, the DPLL loop
 * bandwidth is temporarily widened to allow the phase alignment to finish
 * faster. 
 *
 * The LMK has another quirk in that the DPLL integrator has a 682 ppm slew
 * limit, and at this speed it will overshoot the target and swing back. When we
 * detect such a move, we rewrite the PH_OFFSET to counteract the overshoot
 * using a model of that loop (Kp and Kp/Ki) as measured on the 2049 board.
 *
 * Before trying to adjust phase, the OCXO must be ready. Also, because a phase
 * adjustment also slews the I210 clock, we check that the PHC is counting whole
 * seconds and reseed the PHC if necessary. Also because of this, we keep the
 * PHY down until we're fully phase locked.
 */


#include "igb.h"
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/sort.h>
#include <linux/rtnetlink.h>

int phc_align = 1;
module_param(phc_align, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(phc_align, "coarse align the PHC to the GNSS on load (default 1)");

int pps_align = 1;
module_param(pps_align, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pps_align, "enable GNSS PPS alignment (default 1)");

int pull_loop_gain = 25;
module_param(pull_loop_gain, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pull_loop_gain, "DPLL main LOOP_GAIN (R275) during phase adjustment, 0 = keep the configured loop (default 25 = 0.1 Hz)");

int pull_quant = 3;
module_param(pull_quant, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pull_quant, "DPLL main QUANT (R296) during phase adjustment (default 3 = 0.1 Hz)");

static const int pull_scale = 500000; /* PH_OFFSET scaling: 500000 is 0.5 the datasheet values */

int pps_step_ppm = 550;
module_param(pps_step_ppm, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pps_step_ppm, "phase step rate limit, ppm (default 550, max 680)");

int pps_step_kp = 553;
module_param(pps_step_kp, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pps_step_kp, "Kp of the widened DPLL loop during phase adjust, permille (default 553)");

int pps_step_tau_ms = 128700;
module_param(pps_step_tau_ms, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pps_step_tau_ms, "Kp/Ki of the widened DPLL loop during phase adjust, ms (default 128700)");

int pps_step_lead_in = 10;
module_param(pps_step_lead_in, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pps_step_lead_in, "delay this many GNSS edges after widening the DPLL loop before adjusting phase (default 10)");

int pps_narrow_ns = 500;
module_param(pps_narrow_ns, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(pps_narrow_ns, "set DPLL to narrow LBW once phase error is below this value, ns (default 500)");

int pps_extts = 0;
module_param(pps_extts, int, S_IRUSR);
MODULE_PARM_DESC(pps_extts, "extts channel carrying the PPS (default 0 = SDP0)");

int gnss_extts = 1;
module_param(gnss_extts, int, S_IRUSR);
MODULE_PARM_DESC(gnss_extts, "extts channel carrying the GNSS pulse (default 1 = SDP1)");

int seed_phc = 0;
module_param(seed_phc, int, S_IRUSR);
MODULE_PARM_DESC(seed_phc, "seed the PHC from system time by default instead of the card RTC (default 0)");

int enable_user_phc_adjust = 0;
module_param(enable_user_phc_adjust, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(enable_user_phc_adjust, "accept adjtime/adjfine on the PHC from userspace (default 0)");

#define LMK_R12				0x000c
#define LMK_PH_OFFSET		0x0154	   /* R340, 45-bit two's complement, MSB first */
#define LMK_PH_OFFSET_MASK	((1ULL << 45) - 1)
#define LMK_R252			0x00fc	   /* bit 0 DPLL_LOOP_EN */
#define LMK_FILT_SCALAR		0x010e	   /* R270[1:0]:R271 DPLL_REF_FILT_SCALAR */
#define LMK_LOOP_GAIN		0x0113	   /* R275 main LOOP_GAIN (undocumented; TICS Pro field map) */
#define LMK_QUANT			0x0128 	   /* R296 main QUANT */
#define PPS_SETTLE_S		4	
#define PPS_SLEW_WAIT_S		60
#define ALIGN_SPREAD_NS		2000	   /* a window wider than this is a glitch */
#define PHC_HELD_NS			500
#define PHC_RESTEP_NS		1000
#define PHC_LAT_LEARN_NS	100000	   /* a residual beyond this is not the step latency */
#define PHC_LAT_MAX_NS		50000
#define PPS_ALIGNED_NS		1000
#define PPS_REPULL_NS		2000
#define PPS_RESYNC_NS		100000000	/* a far landing is re-synced instead of pulled */
#define PPS_RESYNC_HOLDOFF_S	600
#define PPS_STEP_NS	1000000	    /* this far off is a step: OUT0 slews for minutes, the link goes down */
#define PPS_STEP_RAMP_US	8000000	    /* a step changes its rate over this long */
#define PPS_STEP_DELAY_MS	100	        /* PH_OFFSET rewritten this long after a GNSS edge, clear of the TDC */
#define PPS_STEP_STALL_S	5
#define PPS_TAIL_SETTLE_S	10	        /* after a step, before the PPS slope is fitted */
#define PPS_WIDE_FLOCK_S	600	        /* frequency-locked this long without phase lock: narrow anyway */
#define PPS_WIDE_MAX_S		1800	    /* never leave the loop wide longer than this */
#define REF_CAP_STALE_S		3		    /* no GNSS capture this long with PRIREF valid = capture path dead */
#define REF_REARM_S			30
#define CLK_FAULT_PPM		5000	    /* the DPLL slews OUT0 at about 1000 ppm at most */
#define CLK_CHECK_MAX_S		10	        /* longer gaps restart the baseline */
#define CLK_FAULT_HOLD_S	10	        /* this long without a fault = steady again */

static const char *phc_states[] = { "wait", "acquire", "held" };
static const char *pps_states[] = { "wait", "settle", "measure", "slew", "aligned", "step", "tail" };
static const char *ref_states[] = { "acquire", "ready", "holdover", "relock" };

static void pps_step_stop(struct igb_adapter *adapter, const char *why, bool finish);

#define align_dbg(adapter, fmt, ...) do { \
	if (igb_debug_level() >= 4) \
		dev_info(&(adapter)->pdev->dev, fmt, ##__VA_ARGS__); \
} while (0)

static int lmk_rd(struct igb_adapter *adapter, u16 reg)
{
	u8 addr[2] = { reg >> 8, reg & 0xff }, val;
	struct i2c_msg msgs[2] = {
		{ .addr = adapter->i2c_lmk05318b->addr, .len = 2, .buf = addr },
		{ .addr = adapter->i2c_lmk05318b->addr, .flags = I2C_M_RD, .len = 1, .buf = &val },
	};
	int ret = i2c_transfer(adapter->i2c_lmk05318b->adapter, msgs, 2);

	if (ret == 2)
		return val;
	return ret < 0 ? ret : -EIO;
}

static int lmk_wr(struct igb_adapter *adapter, u16 reg, u8 val)
{
	return i2c_smbus_write_word_data(adapter->i2c_lmk05318b, reg >> 8, (reg & 0xff) | (val << 8));
}

static int lmk_rd_offset(struct igb_adapter *adapter, u64 *val)
{
	int i, b;

	*val = 0;
	for (i = 0; i < 6; i++) {
		b = lmk_rd(adapter, LMK_PH_OFFSET + i);
		if (b < 0)
			return b;
		*val = (*val << 8) | b;
	}
	return 0;
}

static int lmk_wr_offset(struct igb_adapter *adapter, u64 val)
{
	int i, ret;

	for (i = 0; i < 6; i++) {
		ret = lmk_wr(adapter, LMK_PH_OFFSET + i, (val >> (8 * (5 - i))) & 0xff);
		if (ret < 0)
			return ret;
	}
	adapter->pps.reg = val;
	return 0;
}

/* PH_OFFSET counts per second = 4 x VCO1 (2.5 GHz) x DPLL_REF_FILT_SCALAR
 * per the datasheet (eq. 12), so 10 x the scalar per ns before the measured
 * scale. The scalar is the register file's, so it is read from the chip. */
static int lmk_counts_per_ns(struct igb_adapter *adapter, s64 *cpn)
{
	int hi = lmk_rd(adapter, LMK_FILT_SCALAR);
	int lo = lmk_rd(adapter, LMK_FILT_SCALAR + 1);
	int scalar;

	if (hi < 0 || lo < 0)
		return hi < 0 ? hi : lo;
	scalar = ((hi & 0x03) << 8) | lo;
	if (!scalar)
		return -EINVAL;
	*cpn = 10LL * scalar;
	return 0;
}

static bool ocxo_ready(struct igb_adapter *adapter)
{
	return !adapter->i2c_tmpocxo || adapter->ocxo_ready;
}

static void pps_loop_wide(struct igb_adapter *adapter)
{
	struct igb_pps_align *pa = &adapter->pps;
	int g, q;

	if (!pull_loop_gain || pa->wide)
		return;
	g = lmk_rd(adapter, LMK_LOOP_GAIN);
	q = lmk_rd(adapter, LMK_QUANT);
	if (g < 0 || q < 0 || lmk_wr(adapter, LMK_LOOP_GAIN, pull_loop_gain) < 0 ||
	    lmk_wr(adapter, LMK_QUANT, pull_quant) < 0) {
		if (g >= 0 && q >= 0) {
			lmk_wr(adapter, LMK_LOOP_GAIN, g);
			lmk_wr(adapter, LMK_QUANT, q);
		}
		dev_warn(&adapter->pdev->dev, "PPS: DPLL loop gain write failed, stepping without adjusting LBW\n");
		return;
	}
	pa->narrow_gain = g;
	pa->narrow_quant = q;
	pa->wide = true;
	pa->wide_unlocked = false;
	pa->wide_since = jiffies;
	pa->flock_since = 0;
	dev_info(&adapter->pdev->dev, "PPS: DPLL loop widened for the step: LOOP_GAIN %d -> %d, QUANT %d -> %d\n",
		 g, pull_loop_gain, q, pull_quant);
}

static void pps_loop_narrow(struct igb_adapter *adapter, const char *why)
{
	struct igb_pps_align *pa = &adapter->pps;

	if (!pa->wide)
		return;
	if (lmk_wr(adapter, LMK_LOOP_GAIN, pa->narrow_gain) < 0 || lmk_wr(adapter, LMK_QUANT, pa->narrow_quant) < 0) {
		dev_warn(&adapter->pdev->dev, "PPS: DPLL LBW restore failed, retrying\n");
		return;
	}
	pa->wide = false;
	dev_info(&adapter->pdev->dev, "PPS: DPLL LBW narrowed (%s): LOOP_GAIN %d, QUANT %d\n",
		 why, pa->narrow_gain, pa->narrow_quant);
}

static void pps_loop_check(struct igb_adapter *adapter, u8 r14)
{
	struct igb_pps_align *pa = &adapter->pps;
	bool flock = !(r14 & 0x50);
	bool plock = flock && !(r14 & 0x80);

	if (!pa->wide)
		return;
	if (!plock)
		pa->wide_unlocked = true;
	if (plock && pa->wide_unlocked) {
		pps_loop_narrow(adapter, "phase locked");
		return;
	}
	if (!flock)
		pa->flock_since = 0;
	else if (!pa->flock_since)
		pa->flock_since = jiffies;
	if (pa->flock_since && time_after(jiffies, pa->flock_since + PPS_WIDE_FLOCK_S * HZ))
		pps_loop_narrow(adapter, "frequency locked without phase lock");
	else if (time_after(jiffies, pa->wide_since + PPS_WIDE_MAX_S * HZ))
		pps_loop_narrow(adapter, "time limit");
}

void igb_lmk_reset(struct igb_adapter *adapter)
{
	igb_ref_event(adapter, "DPLL reset");
	pps_step_stop(adapter, "DPLL reset", false);
	pps_loop_narrow(adapter, "DPLL reset");
	lmk_wr(adapter, LMK_R12, 0x9b);
	usleep_range(10000, 20000);
	lmk_wr(adapter, LMK_R12, 0x1b);
	adapter->phc.state = PHC_WAIT;
	adapter->phc.need_seed = true;
	adapter->pps.state = PPS_WAIT;
	adapter->pps.pull_pending = false;	/* a re-sync lands anew */
	adapter->pps.early = true;
	adapter->pps.last_reset = jiffies;
	adapter->pps.far_logged = false;
	dev_info(&adapter->pdev->dev, "DPLL reset\n");
}

/* Under tmreg_lock, on every GNSS rising edge. */
static void clk_check(struct igb_adapter *adapter, u64 ns)
{
	struct igb_clk_check *c = &adapter->clk;
	u64 raw = ktime_get_raw_ns();
	s64 n, dev;

	if (c->have && c->epoch == adapter->phc_epoch) {
		n = div64_u64(raw - c->raw + NSEC_PER_SEC / 2, NSEC_PER_SEC);
		dev = (s64)(ns - c->ns) - n * NSEC_PER_SEC;
		if (n >= 1 && n <= CLK_CHECK_MAX_S && abs(dev) > n * CLK_FAULT_PPM * 1000LL) {
			c->fault = true;
			c->dev = dev;
			c->n = n;
		}
	}
	c->have = true;
	c->epoch = adapter->phc_epoch;
	c->raw = raw;
	c->ns = ns;
}

/* From the extts work (MSI-X) or the interrupt (legacy vectors). The I210
 * latches both edges; the pin still high means this was the rising one. */
void igb_align_capture(struct igb_adapter *adapter, int chan, u64 ns)
{
	struct igb_align_cap *cap = &adapter->align_cap[chan];
	u32 ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
	unsigned long flags;

	if (!(ctrl & (chan ? E1000_TS_SDP1_DATA : E1000_TS_SDP0_DATA)))
		return;
	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	cap->ns = ns;
	cap->seq++;
	cap->at = jiffies;
	if (chan == gnss_extts) {
		clk_check(adapter, ns);
		if (adapter->pps.step.armed) {
			adapter->pps.step.edges++;
			schedule_delayed_work(&adapter->pps.step.work, msecs_to_jiffies(PPS_STEP_DELAY_MS));
		}
	}
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);
}

static bool cap_take(struct igb_adapter *adapter, int chan, u32 *seen, u64 *ns)
{
	struct igb_align_cap *cap = &adapter->align_cap[chan];
	unsigned long flags;
	bool fresh;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	fresh = cap->seq != *seen;
	*seen = cap->seq;
	*ns = cap->ns;
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);
	return fresh;
}

static void cap_flush(struct igb_adapter *adapter, int chan, u32 *seen)
{
	u64 ns;

	cap_take(adapter, chan, seen, &ns);
}

/* Fold to the nearest second: (-0.5 s, 0.5 s]. */
static s64 wrap_second(s64 ns)
{
	s32 rem;

	div_s64_rem(ns, NSEC_PER_SEC, &rem);
	ns = rem;
	if (ns > NSEC_PER_SEC / 2)
		ns -= NSEC_PER_SEC;
	else if (ns <= -NSEC_PER_SEC / 2)
		ns += NSEC_PER_SEC;
	return ns;
}

static int cmp_s64(const void *a, const void *b)
{
	s64 x = *(const s64 *)a, y = *(const s64 *)b;

	return x < y ? -1 : x > y;
}

/* Median of a full window, or false when the window is a glitch. */
static bool window_median(s64 *win, s64 *median)
{
	s64 sorted[ALIGN_SAMPLES];

	memcpy(sorted, win, sizeof(sorted));
	sort(sorted, ALIGN_SAMPLES, sizeof(s64), cmp_s64, NULL);
	if (sorted[ALIGN_SAMPLES - 1] - sorted[0] > ALIGN_SPREAD_NS)
		return false;
	*median = (sorted[ALIGN_SAMPLES / 2 - 1] + sorted[ALIGN_SAMPLES / 2]) / 2;
	return true;
}

static void phc_run(struct igb_adapter *adapter, bool locked)
{
	struct igb_phc_align *p = &adapter->phc;
	s64 err, median, step;
	u64 ns;

	if (!locked) {
		if (p->state != PHC_WAIT) {
			dev_warn(&adapter->pdev->dev, "PHC phase: Waiting for DPLL lock\n");
			p->need_seed = true;
		}
		p->state = PHC_WAIT;
		p->n = 0;
		p->stepped = false;
		return;
	}
	if (p->state == PHC_WAIT) {
		p->state = PHC_ACQUIRE;
		p->n = 0;
		if (!p->seeded || p->need_seed) {
			igb_phc_seed(adapter, "at DPLL lock");
			p->seeded = true;
			p->need_seed = false;
		}
		cap_flush(adapter, gnss_extts, &p->seen);
		dev_info(&adapter->pdev->dev, "PHC phase: DPLL locked, acquiring\n");
		return;
	}
	if (!cap_take(adapter, gnss_extts, &p->seen, &ns))
		return;
	err = wrap_second(ns);
	p->err = err;
	align_dbg(adapter, "PHC phase: GNSS pulse at %lld ns\n", err);

	if (p->stepped) {
		p->stepped = false;
		if (abs(err) < PHC_LAT_LEARN_NS) {
			p->lat = clamp_t(s64, p->lat - err, 0, PHC_LAT_MAX_NS);
			align_dbg(adapter, "PHC phase: Step latency now %lld ns\n", p->lat);
		}
	}

	switch (p->state) {
	case PHC_ACQUIRE:
		p->win[p->n++] = err;
		if (p->n < ALIGN_SAMPLES)
			return;
		p->n = 0;
		if (!window_median(p->win, &median)) {
			dev_warn(&adapter->pdev->dev, "PHC phase: Captures spread beyond %d ns, measuring again\n", ALIGN_SPREAD_NS);
			return;
		}
		if (abs(median) < PHC_HELD_NS) {
			p->state = PHC_HELD;
			dev_info(&adapter->pdev->dev, "PHC phase: %lld ns after %u steps, step latency %lld ns\n",
				 median, p->steps, p->lat);
			return;
		}
		step = -median + p->lat;
		igb_ptp_adjtime_internal(adapter, step);
		p->steps++;
		p->stepped = true;
		dev_info(&adapter->pdev->dev, "PHC phase: Error %lld ns, stepped %lld ns\n", median, step);
		return;
	case PHC_HELD:
		if (abs(err) <= PHC_RESTEP_NS) {
			p->bad = 0;
			return;
		}
		if (++p->bad < ALIGN_SAMPLES)
			return;
		p->bad = 0;
		p->state = PHC_ACQUIRE;
		dev_warn(&adapter->pdev->dev, "PHC phase: Error %lld ns, re-acquiring\n", err);
		return;
	}
}

static void pps_slew(struct igb_adapter *adapter, s64 err)
{
	struct igb_pps_align *pa = &adapter->pps;
	s64 adj = -err;
	u64 cur, val;
	s64 cpn, counts;

	if (lmk_counts_per_ns(adapter, &cpn) < 0) {
		dev_warn(&adapter->pdev->dev, "PPS: DPLL_REF_FILT_SCALAR read failed\n");
		return;
	}
	if (lmk_rd_offset(adapter, &cur) < 0) {
		dev_warn(&adapter->pdev->dev, "PPS: DPLL_REF_SYNC_PH_OFFSET read failed\n");
		return;
	}
	if (!pa->scale_ppm)
		pa->scale_ppm = pull_scale;
	counts = div_s64(div_s64(adj * cpn, 1000) * pa->scale_ppm, 1000);
	val = (cur + counts) & LMK_PH_OFFSET_MASK;
	if (lmk_wr_offset(adapter, val) < 0) {
		dev_warn(&adapter->pdev->dev, "PPS: DPLL_REF_SYNC_PH_OFFSET write failed\n");
		return;
	}
	pa->writes++;
	pa->pull_pending = true;
	pa->pull_err = err;
	pa->pull_ns = adj;
	dev_info(&adapter->pdev->dev, "PPS: Error %lld ns, slewing %lld ns, DPLL_REF_SYNC_PH_OFFSET 0x%llx -> 0x%llx\n",
		 err, adj, cur, val);
}

/* First measurement after a step: scale = scale x requested / moved. */
static void pps_learn_scale(struct igb_adapter *adapter, s64 median)
{
	struct igb_pps_align *pa = &adapter->pps;
	s64 moved = median - pa->pull_err;
	s64 scale;

	pa->pull_pending = false;
	if (abs(pa->pull_ns) < PPS_STEP_NS)
		return;
	if (!moved || (moved > 0) != (pa->pull_ns > 0)) {
		dev_warn(&adapter->pdev->dev, "PPS: Wanted step of %lld ns, moved %lld ns, scale kept at %u ppm\n",
			 pa->pull_ns, moved, pa->scale_ppm);
		return;
	}
	scale = div_s64((s64)pa->scale_ppm * pa->pull_ns, moved);
	if (scale < pa->scale_ppm / 4 || scale > (s64)pa->scale_ppm * 4) {
		dev_warn(&adapter->pdev->dev, "PPS: Wanted step of %lld ns, moved %lld ns (???), scale kept at %u ppm\n",
			 pa->pull_ns, moved, pa->scale_ppm);
		return;
	}
	dev_info(&adapter->pdev->dev, "PPS: Wanted step of %lld ns, moved %lld ns, scale %u -> %lld ppm\n",
		 pa->pull_ns, moved, pa->scale_ppm, scale);
	pa->scale_ppm = scale;
}

/* A step's velocity is a trapezoid up to +r, across to -r and back to zero
 * (knots in us and ps/s); p1 and p2 are its plateaus. */
static void step_knots(struct igb_pps_step *m, s64 r, s64 p1, s64 p2)
{
	s64 ramp = PPS_STEP_RAMP_US;

	m->t[0] = 0;
	m->t[1] = ramp;
	m->t[2] = ramp + p1;
	m->t[3] = 3 * ramp + p1;
	m->t[4] = 3 * ramp + p1 + p2;
	m->t[5] = 4 * ramp + p1 + p2;
	m->v[0] = 0;
	m->v[1] = r;
	m->v[2] = r;
	m->v[3] = -r;
	m->v[4] = -r;
	m->v[5] = 0;
	m->n = div64_s64(m->t[5] + USEC_PER_SEC - 1, USEC_PER_SEC);
}

/* Distance (ps) the velocity covers by t (us). */
static s64 step_pos(const struct igb_pps_step *m, s64 t)
{
	s64 p = 0, a, b, vt;
	int i;

	for (i = 0; i < 5 && t > m->t[i]; i++) {
		a = m->t[i];
		b = min(t, m->t[i + 1]);
		if (b == a)
			continue;
		vt = m->v[i] + div64_s64((m->v[i + 1] - m->v[i]) * (b - a), m->t[i + 1] - a);
		p += div64_s64((b - a) * (m->v[i] + vt), 2 * USEC_PER_SEC);
	}
	return p;
}

/* Planned output movement (ps) after TDC event k. */
static s64 step_delta(const struct igb_pps_step *m, u32 k)
{
	return step_pos(m, (s64)(k + 1) * USEC_PER_SEC) - step_pos(m, (s64)k * USEC_PER_SEC);
}

/* The widened loop per TDC event (one per GNSS edge): the output moves
 * Kp e + I, the integrator takes Kp / tau of e. */
static void step_event(struct igb_pps_step *m, s64 target)
{
	s64 e = target - m->y;

	m->y += div_s64(e * m->kp, 1000) + m->i;
	m->i += div_s64(e * m->kp, m->tau_ms);
}

/* The target that makes the model output move v at the next event. */
static s64 step_lead(const struct igb_pps_step *m, s64 v)
{
	return m->y + div_s64((v - m->i) * 1000, m->kp);
}

static s64 step_i_end(struct igb_pps_step *m)
{
	u32 k;

	m->y = 0;
	m->i = 0;
	for (k = 0; k < m->n; k++)
		step_event(m, step_lead(m, step_delta(m, k)));
	return m->i;
}

/* Plateaus for dist (ps, > 0) at up to r (ps/s, whole ppm): the model
 * integrator must be empty when the velocity is, or the slow pole creeps
 * the difference in over many minutes. */
static void step_plan(struct igb_pps_step *m, s64 dist, s64 r)
{
	s64 flat = div64_s64(dist, div64_s64(r, USEC_PER_SEC));
	s64 lo = 0, hi = 2000LL * USEC_PER_SEC, mid;

	while (hi - lo > 1) {
		mid = lo + ((hi - lo) >> 1);
		step_knots(m, r, mid + flat, mid);
		if (step_i_end(m) > 0)
			lo = mid;
		else
			hi = mid;
	}
	step_knots(m, r, lo + flat, lo);
}

static u64 step_reg(const struct igb_pps_step *m, s64 target)
{
	s64 counts = div_s64(div_s64(m->sign * target * m->cpn, 1000) * m->scale_ppm, 1000000);

	return (m->base + counts) & LMK_PH_OFFSET_MASK;
}

/* The target in effect for the next event; none during the lead-in. */
static s64 step_next(const struct igb_pps_step *m)
{
	return m->k < m->skip ? 0 : step_lead(m, step_delta(m, m->k - m->skip));
}

/* PPS_STEP_DELAY_MS after each GNSS edge: the model takes the events since
 * the last run on the target that was in effect, then the target for the
 * next event is written. The last event leaves the end point. */
static void step_work(struct work_struct *work)
{
	struct igb_pps_step *m = container_of(to_delayed_work(work), struct igb_pps_step, work);
	struct igb_adapter *adapter = container_of(m, struct igb_adapter, pps.step);
	unsigned long flags;
	u32 edges;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	edges = m->edges;
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);

	mutex_lock(&m->lock);
	if (!m->active || m->failed || !adapter->i2c_lmk05318b)
		goto out;
	m->last = jiffies;
	while (m->k < edges && m->k < m->n) {
		step_event(m, m->target);
		m->k++;
	}
	if (m->k >= m->n) {
		m->target = m->dist;
		m->active = false;
		m->done = true;
		spin_lock_irqsave(&adapter->tmreg_lock, flags);
		m->armed = false;
		spin_unlock_irqrestore(&adapter->tmreg_lock, flags);
	} else {
		m->target = step_next(m);
	}
	if (m->k >= m->skip && lmk_wr_offset(adapter, step_reg(m, m->target)) < 0) {
		m->failed = true;
		m->active = true;
	}
	align_dbg(adapter, "PPS: Step event %u of %u, edges %u, target %lld ps, model %lld ps %lld ps/s\n",
		  m->k, m->n, edges, m->target, m->y, m->i);
out:
	mutex_unlock(&m->lock);
}

/* finish leaves the end point programmed: the loop gets there on its own. */
static void pps_step_stop(struct igb_adapter *adapter, const char *why, bool finish)
{
	struct igb_pps_step *m = &adapter->pps.step;
	unsigned long flags;
	bool was;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	m->armed = false;
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);
	mutex_lock(&m->lock);
	was = m->active;
	m->active = false;
	if (was && finish && adapter->i2c_lmk05318b)
		lmk_wr_offset(adapter, step_reg(m, m->dist));
	mutex_unlock(&m->lock);
	cancel_delayed_work(&m->work);
	if (was && why)
		dev_warn(&adapter->pdev->dev, "PPS: Step stopped (%s)%s\n", why,
			 finish ? ", DPLL_REF_SYNC_PH_OFFSET set to end point" : "");
}

static void pps_step_start(struct igb_adapter *adapter, s64 err)
{
	struct igb_pps_align *pa = &adapter->pps;
	struct igb_pps_step *m = &pa->step;
	s64 adj = -err, dist = abs(adj) * 1000, cpn, r;
	unsigned long flags;
	u64 cur;
	int ret;

	if (lmk_counts_per_ns(adapter, &cpn) < 0 || lmk_rd_offset(adapter, &cur) < 0) {
		dev_warn(&adapter->pdev->dev, "PPS: DPLL register read failed, retrying\n");
		return;
	}
	igb_ref_event(adapter, "PPS step");
	pps_loop_wide(adapter);
	if (!pa->scale_ppm)
		pa->scale_ppm = pull_scale;
	mutex_lock(&m->lock);
	m->kp = clamp(pps_step_kp, 100, 1500);
	m->tau_ms = clamp(pps_step_tau_ms, 10000, 2000000);
	/* short steps run slower, so that the plateaus outlast the ramps */
	r = clamp_t(s64, div64_s64(3 * dist, (s64)m->tau_ms * 1000), 1, clamp(pps_step_ppm, 1, 680));
	step_plan(m, dist, r * 1000000);
	m->skip = clamp(pps_step_lead_in, 0, 60);
	m->n += m->skip;
	m->sign = adj < 0 ? -1 : 1;
	m->dist = dist;
	m->base = cur;
	m->cpn = cpn;
	m->scale_ppm = pa->scale_ppm;
	m->y = 0;
	m->i = 0;
	m->k = 0;
	m->done = false;
	m->failed = false;
	m->target = step_next(m);
	ret = m->target ? lmk_wr_offset(adapter, step_reg(m, m->target)) : 0;
	m->active = !ret;
	m->last = jiffies;
	mutex_unlock(&m->lock);
	if (ret < 0) {
		dev_warn(&adapter->pdev->dev, "PPS: DPLL_REF_SYNC_PH_OFFSET write failed\n");
		return;
	}
	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	m->edges = 0;
	m->armed = true;
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);
	pa->writes++;
	pa->pull_pending = true;
	pa->pull_err = err;
	pa->pull_ns = adj;
	pa->early = false;
	pa->state = PPS_STEP;
	dev_info(&adapter->pdev->dev, "PPS: Error %lld ns, stepping %lld ns over %u s, DPLL_REF_SYNC_PH_OFFSET 0x%llx -> 0x%llx\n",
		 err, adj, m->n - m->skip, cur, step_reg(m, m->dist));
}

static void pps_step_check(struct igb_adapter *adapter)
{
	struct igb_pps_align *pa = &adapter->pps;
	struct igb_pps_step *m = &pa->step;
	unsigned long last;
	bool done, failed;

	mutex_lock(&m->lock);
	done = m->done;
	failed = m->failed;
	last = m->last;
	mutex_unlock(&m->lock);
	if (failed || (!done && time_after(jiffies, last + PPS_STEP_STALL_S * HZ))) {
		pps_step_stop(adapter, failed ? "DPLL_REF_SYNC_PH_OFFSET write failed" : "no GNSS edges", true);
		pa->state = PPS_WAIT;
		return;
	}
	if (!done)
		return;
	dev_info(&adapter->pdev->dev, "PPS: DPLL_REF_SYNC_PH_OFFSET 0x%llx\n", pa->reg);
	pa->state = PPS_TAIL;
	pa->tail_n = 0;
	pa->tail_last = 0;
	pa->tail_since = jiffies;
	pa->have_pps = pa->have_gnss = false;
	cap_flush(adapter, pps_extts, &pa->seen_pps);
	cap_flush(adapter, gnss_extts, &pa->seen_gnss);
}

/* After a step the widened loop creeps the rest in on its slow pole, so the
 * PPS slope times about Kp/Ki is what is left to go. */
static void pps_tail(struct igb_adapter *adapter, u64 gnss, s64 err)
{
	struct igb_pps_align *pa = &adapter->pps;
	s64 t = div_u64(gnss, NSEC_PER_SEC), x, y, st = 0, se = 0, stt = 0, ste = 0, num, den;
	s64 n = PPS_TAIL_SAMPLES, prev;
	int i;

	if (time_before(jiffies, pa->tail_since + PPS_TAIL_SETTLE_S * HZ) || adapter->clk.bad) {
		pa->tail_n = 0;
		return;
	}
	/* A PHC step or a gap restarts the fit, and the first PPS edge after a
	 * gap is dropped: at the unmute it lands up to 2 us off the others. */
	prev = pa->tail_last;
	pa->tail_last = t;
	if (t <= prev || t > prev + 5) {
		pa->tail_n = 0;
		return;
	}
	pa->tail_t[pa->tail_n % PPS_TAIL_SAMPLES] = t;
	pa->tail_e[pa->tail_n % PPS_TAIL_SAMPLES] = err;
	if (++pa->tail_n < PPS_TAIL_SAMPLES)
		return;
	for (i = 0; i < PPS_TAIL_SAMPLES; i++) {
		x = pa->tail_t[i] - t;
		y = pa->tail_e[i] - err;
		st += x;
		se += y;
		stt += x * x;
		ste += x * y;
	}
	num = n * ste - st * se;
	den = n * stt - st * st;
	if (den <= 0 || abs(num) * pps_step_tau_ms >= (s64)pps_narrow_ns * 1000 * den)
		return;
	dev_info(&adapter->pdev->dev, "PPS: %lld ns left after the step (slope over %d s)\n",
		 div64_s64(num * pps_step_tau_ms, den * 1000), PPS_TAIL_SAMPLES);
	pps_loop_narrow(adapter, "stepped");
	pa->state = PPS_WAIT;
}

void igb_align_init(struct igb_adapter *adapter)
{
	mutex_init(&adapter->pps.step.lock);
	INIT_DELAYED_WORK(&adapter->pps.step.work, step_work);
	adapter->pps.early = true;
}

void igb_align_stop(struct igb_adapter *adapter)
{
	pps_step_stop(adapter, NULL, false);
	cancel_delayed_work_sync(&adapter->pps.step.work);
}

static void pps_run(struct igb_adapter *adapter, u8 r14, u8 r411)
{
	struct igb_pps_align *pa = &adapter->pps;
	bool ref = (r411 & 0x0c) && !(r14 & 0x10);
	bool flock = ref && !(r14 & 0x40);
	bool plock = flock && !(r14 & 0x80);
	s64 err, median;
	u64 ns;

	switch (pa->state) {
	case PPS_STEP:
		if (ref) {
			pps_step_check(adapter);
			return;
		}
		pps_step_stop(adapter, "reference lost", true);
		pa->state = PPS_WAIT;
		return;
	case PPS_TAIL:
		if (ref && pa->wide)
			break;
		if (!ref)
			dev_warn(&adapter->pdev->dev, "PPS: Reference lost after step, waiting\n");
		pa->state = PPS_WAIT;
		return;
	default:
		if (plock || (flock && pa->early && pa->state <= PPS_MEASURE))
			break;
		if (pa->state != PPS_WAIT)
			dev_warn(&adapter->pdev->dev, "PPS: DPLL unlocked, waiting\n");
		pa->state = PPS_WAIT;
		return;
	}
	switch (pa->state) {
	case PPS_WAIT:
		if (!ocxo_ready(adapter)) {
			if (!pa->ocxo_logged)
				dev_info(&adapter->pdev->dev, "PPS: DPLL locked, waiting for OCXO temperature\n");
			pa->ocxo_logged = true;
			return;
		}
		pa->ocxo_logged = false;
		pa->state = PPS_SETTLE;
		pa->lock_since = jiffies;
		dev_info(&adapter->pdev->dev, "PPS: DPLL %s locked, settling %d s\n", plock ? "phase" : "frequency",
			 PPS_SETTLE_S);
		return;
	case PPS_SLEW:
		if (time_before(jiffies, pa->wait_until))
			return;
		fallthrough;
	case PPS_SETTLE:
		if (time_before(jiffies, pa->lock_since + PPS_SETTLE_S * HZ))
			return;
		pa->state = PPS_MEASURE;
		pa->n = 0;
		pa->have_pps = pa->have_gnss = false;
		cap_flush(adapter, pps_extts, &pa->seen_pps);
		cap_flush(adapter, gnss_extts, &pa->seen_gnss);
		dev_info(&adapter->pdev->dev, "PPS: Measuring\n");
		return;
	default:
		break;
	}

	if (cap_take(adapter, pps_extts, &pa->seen_pps, &ns)) {
		pa->ns_pps = ns;
		pa->have_pps = true;
	}
	if (cap_take(adapter, gnss_extts, &pa->seen_gnss, &ns)) {
		pa->ns_gnss = ns;
		pa->have_gnss = true;
	}
	if (!pa->have_pps || !pa->have_gnss)
		return;
	pa->have_pps = pa->have_gnss = false;
	err = wrap_second((s64)(pa->ns_pps - pa->ns_gnss));
	pa->err = err;
	align_dbg(adapter, "PPS: PPS - GNSS = %lld ns\n", err);

	switch (pa->state) {
	case PPS_TAIL:
		pps_tail(adapter, pa->ns_gnss, err);
		return;
	case PPS_MEASURE:
		pa->win[pa->n++] = err;
		if (pa->n < ALIGN_SAMPLES)
			return;
		pa->n = 0;
		if (!window_median(pa->win, &median)) {
			dev_warn(&adapter->pdev->dev, "PPS: Captures spread beyond %d ns, measuring again\n", ALIGN_SPREAD_NS);
			return;
		}
		if (pa->pull_pending)
			pps_learn_scale(adapter, median);
		if (abs(median) < PPS_ALIGNED_NS && plock) {
			pa->state = PPS_ALIGNED;
			pa->early = false;
			dev_info(&adapter->pdev->dev, "PPS aligned: %lld ns, DPLL_REF_SYNC_PH_OFFSET 0x%llx after %u writes\n",
				 median, pa->reg, pa->writes);
			return;
		}
		if (!ocxo_ready(adapter)) {
			pa->state = PPS_WAIT;
			dev_warn(&adapter->pdev->dev, "PPS: OCXO not ready, not adjusting\n");
			return;
		}
		if (abs(median) > PPS_RESYNC_NS) {
			unsigned long due = pa->last_reset + PPS_RESYNC_HOLDOFF_S * HZ;

			if (time_before(jiffies, due)) {
				if (!pa->far_logged)
					dev_warn(&adapter->pdev->dev, "PPS: %lld ns offset, re-syncing in %lu s\n",
						 median, (due - jiffies) / HZ);
				pa->far_logged = true;
				return;
			}
			dev_warn(&adapter->pdev->dev, "PPS: %lld ns offset, re-syncing\n", median);
			igb_lmk_reset(adapter);
			return;
		}
		if (abs(median) >= PPS_STEP_NS) {
			pps_step_start(adapter, median);
			return;
		}
		if (!plock) {
			pa->early = false;
			pa->state = PPS_WAIT;
			dev_info(&adapter->pdev->dev, "PPS: %lld ns at frequency lock, measuring again at phase lock\n", median);
			return;
		}
		pps_slew(adapter, median);
		pa->state = PPS_SLEW;
		pa->lock_since = jiffies;
		pa->wait_until = jiffies + PPS_SLEW_WAIT_S * HZ;
		return;
	case PPS_ALIGNED:
		if (abs(err) <= PPS_REPULL_NS) {
			pa->bad = 0;
			return;
		}
		if (++pa->bad < ALIGN_SAMPLES)
			return;
		pa->bad = 0;
		pa->state = PPS_MEASURE;
		pa->n = 0;
		dev_warn(&adapter->pdev->dev, "PPS: Error %lld ns, measuring again\n", err);
		return;
	default:
		return;
	}
}

/* The link and the PHC follow one reference state. The PHY is powered from
 * the DPLL worker, so the netdev lock is only tried; a miss retries next pass. */
static void link_power(struct igb_adapter *adapter, bool up)
{
	struct net_device *netdev = adapter->netdev;

	adapter->ref.link_pending = 0;
	if (!netdev || !netif_running(netdev) || test_bit(__IGB_DOWN, adapter->state))
		return;
	if (!rtnl_trylock()) {
		adapter->ref.link_pending = up ? 1 : -1;
		return;
	}
	if (up)
		igb_power_up_link(adapter);
	else
		igb_power_down_link(adapter);
	rtnl_unlock();
	dev_info(&adapter->pdev->dev, "PHY: Link %s\n", up ? "released" : "held down");
}

bool igb_ref_link_allowed(struct igb_adapter *adapter)
{
	return !adapter->i2c_lmk05318b || !(phc_align || pps_align) || adapter->ref.latched;
}

bool igb_ref_time_valid(struct igb_adapter *adapter)
{
	return adapter->ref.valid;
}

const char *igb_ref_state(struct igb_adapter *adapter)
{
	return ref_states[adapter->ref.state];
}

/* igb_up: the PHY stays down until the reference has been ready once. */
void igb_link_gate_up(struct igb_adapter *adapter)
{
	if (!igb_ref_link_allowed(adapter)) {
		dev_info(&adapter->pdev->dev, "PHY: Time reference not ready, refusing link up\n");
		return;
	}
	igb_power_up_link(adapter);
}

/* A reset or a step slews OUT0 and the PHC for minutes: nobody listens. */
void igb_ref_event(struct igb_adapter *adapter, const char *why)
{
	struct igb_ref *r = &adapter->ref;

	if (!adapter->i2c_lmk05318b || !(phc_align || pps_align))
		return;
	if (r->latched)
		dev_warn(&adapter->pdev->dev, "Time reference lost (%s)\n", why);
	r->latched = false;
	r->valid = false;
	r->state = REF_ACQUIRE;
	link_power(adapter, false);
	igb_dpll_notify(adapter);
}

/* DPLL_LOOP_EN off: the DPLL cannot lock; the outputs, OUT0 (the I210 clock) and the PHC, are not reliable */
void igb_dpll_hold(struct igb_adapter *adapter, bool hold)
{
	struct igb_ref *r = &adapter->ref;
	int v;

	if (!adapter->i2c_lmk05318b || hold == r->dpll_held)
		return;
	v = lmk_rd(adapter, LMK_R252);
	if (v >= 0)
		v = lmk_wr(adapter, LMK_R252, hold ? v & ~0x01 : v | 0x01);
	if (v < 0) {
		dev_warn(&adapter->pdev->dev, "DPLL %s failed\n", hold ? "hold" : "release");
		return;
	}
	r->dpll_held = hold;
	dev_info(&adapter->pdev->dev, "%s\n", hold ? "DPLL waiting until OCXO ready" : "DPLL released: OCXO ready");
}

static bool gnss_fresh(struct igb_adapter *adapter)
{
	struct igb_align_cap *cap = &adapter->align_cap[gnss_extts];

	return cap->seq && time_before(jiffies, cap->at + REF_CAP_STALE_S * HZ);
}

static void ref_update(struct igb_adapter *adapter, u8 r14, u8 r411, u8 r167)
{
	struct igb_ref *r = &adapter->ref;
	bool locked = !(r14 & 0xd0) && (r411 & 0x0c);
	bool holdover = r14 & 0x10;
	bool pps_ok = !pps_align || adapter->pps.state == PPS_ALIGNED;
	bool fresh = !phc_align || gnss_fresh(adapter);
	bool phc_ok = !phc_align || adapter->phc.state == PHC_HELD;
	bool ready = locked && pps_ok && phc_ok && fresh;
	bool valid;
	int state;

	r->r14 = r14;
	r->r411 = r411;
	r->r167 = r167;
	if (phc_align && (r411 & 0x04) && !fresh && time_after_eq(jiffies, r->rearm_at)) {
		dev_warn(&adapter->pdev->dev, "GNSS captures stopped while PRIREF is valid, re-arming extts\n");
		igb_ptp_align_extts(adapter);
		r->rearm_at = jiffies + REF_REARM_S * HZ;
	}
	if (ready && !r->latched) {
		r->latched = true;
		dev_warn(&adapter->pdev->dev, "Clock reference ready: DPLL locked, PPS aligned, PHC held\n");
		link_power(adapter, true);
	} else if (r->link_pending) {
		link_power(adapter, r->link_pending > 0);
	}
	valid = r->latched && (locked ? ready : holdover);
	if (valid != r->valid) {
		r->valid = valid;
		dev_warn(&adapter->pdev->dev, "PHC reads %s\n", valid ? "permitted" : "blocked");
	}
	if (!r->latched)
		state = REF_ACQUIRE;
	else if (locked)
		state = valid ? REF_READY : REF_RELOCK;
	else
		state = holdover ? REF_HOLDOVER : REF_RELOCK;
	if (state != r->state) {
		r->state = state;
		dev_info(&adapter->pdev->dev, "Clock reference %s\n", ref_states[state]);
	}
	igb_dpll_notify(adapter);
}

/* A fault drops the reference and holds the PHC loop until the timer has
 * counted whole seconds for CLK_FAULT_HOLD_S. */
static void clk_run(struct igb_adapter *adapter)
{
	struct igb_clk_check *c = &adapter->clk;
	struct igb_phc_align *p = &adapter->phc;
	unsigned long flags;
	bool fault;
	s64 dev, n;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	fault = c->fault;
	c->fault = false;
	dev = c->dev;
	n = c->n;
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);

	if (fault) {
		if (!c->bad) {
			dev_warn(&adapter->pdev->dev, "PHC: Clock rate error of %lld ns over %lld s, re-seeding at next lock\n",
				 dev, n);
			igb_ref_event(adapter, "PHC clock fault");
		}
		c->bad = true;
		c->bad_until = jiffies + CLK_FAULT_HOLD_S * HZ;
		p->state = PHC_WAIT;
		p->n = 0;
		p->stepped = false;
		p->need_seed = true;
	} else if (c->bad && time_after(jiffies, c->bad_until)) {
		c->bad = false;
		dev_info(&adapter->pdev->dev, "PHC: Clock steady again\n");
	}
}

/* One pass per DPLL worker loop (about a second). */
void igb_align_run(struct igb_adapter *adapter, u8 r14, u8 r411, u8 r167)
{
	bool locked = !(r14 & 0xd0) && (r411 & 0x0c);

	if ((r14 & 0x10) && !(adapter->phc.last_r14 & 0x10))
		adapter->phc.need_seed = true;
	adapter->phc.last_r14 = r14;
	if (!adapter->i2c_lmk05318b || pps_extts == gnss_extts ||
	    pps_extts < 0 || pps_extts >= IGB_N_EXTTS || gnss_extts < 0 || gnss_extts >= IGB_N_EXTTS)
		return;
	clk_run(adapter);
	pps_loop_check(adapter, r14);
	if (phc_align)
		phc_run(adapter, locked && !adapter->clk.bad);
	if (pps_align)
		pps_run(adapter, r14, r411);
	ref_update(adapter, r14, r411, r167);
}

const char *igb_phc_align_state(struct igb_adapter *adapter)
{
	return phc_align ? phc_states[adapter->phc.state] : "off";
}

const char *igb_pps_align_state(struct igb_adapter *adapter)
{
	return pps_align ? pps_states[adapter->pps.state] : "off";
}
