// SPDX-License-Identifier: GPL-2.0
/* Board 2409: the LMK05318B as a kernel dpll device. Lock status follows the
 * driver's reference state, not the chip's R14 bits: locked means the PPS is
 * aligned and the PHC held; holdover only after a ready reference. */

#include "igb.h"
#include <linux/module.h>
#include <linux/etherdevice.h>

#if IS_ENABLED(CONFIG_DPLL)
#include <linux/dpll.h>

static enum dpll_lock_status igb_dpll_status(struct igb_adapter *adapter)
{
	struct igb_ref *r = &adapter->ref;
	bool locked = !(r->r14 & 0xd0) && (r->r411 & 0x0c);

	if (r->valid && locked)
		return (r->r14 & 0x20) ? DPLL_LOCK_STATUS_LOCKED_HO_ACQ : DPLL_LOCK_STATUS_LOCKED;
	if (r->latched && (r->r14 & 0x10))
		return DPLL_LOCK_STATUS_HOLDOVER;
	return DPLL_LOCK_STATUS_UNLOCKED;
}

static enum dpll_pin_state igb_dpll_pin_state(struct igb_adapter *adapter, int idx)
{
	struct igb_ref *r = &adapter->ref;

	if ((r->r167 & 0x03) == idx + 1)
		return DPLL_PIN_STATE_CONNECTED;
	if (r->r411 & (0x04 << idx))
		return DPLL_PIN_STATE_SELECTABLE;
	return DPLL_PIN_STATE_DISCONNECTED;
}

static int igb_dpll_mode_get(const struct dpll_device *dpll, void *priv,
			     enum dpll_mode *mode, struct netlink_ext_ack *extack)
{
	*mode = DPLL_MODE_MANUAL;
	return 0;
}

static int igb_dpll_lock_status_get(const struct dpll_device *dpll, void *priv,
				    enum dpll_lock_status *status,
				    enum dpll_lock_status_error *status_error,
				    struct netlink_ext_ack *extack)
{
	*status = igb_dpll_status(priv);
	*status_error = DPLL_LOCK_STATUS_ERROR_NONE;
	return 0;
}

static const struct dpll_device_ops igb_dpll_ops = {
	.mode_get = igb_dpll_mode_get,
	.lock_status_get = igb_dpll_lock_status_get,
};

static int igb_dpll_pin_state_get(const struct dpll_pin *pin, void *pin_priv,
				  const struct dpll_device *dpll, void *dpll_priv,
				  enum dpll_pin_state *state, struct netlink_ext_ack *extack)
{
	*state = igb_dpll_pin_state(dpll_priv, (long)pin_priv);
	return 0;
}

static int igb_dpll_pin_direction_get(const struct dpll_pin *pin, void *pin_priv,
				      const struct dpll_device *dpll, void *dpll_priv,
				      enum dpll_pin_direction *direction,
				      struct netlink_ext_ack *extack)
{
	*direction = DPLL_PIN_DIRECTION_INPUT;
	return 0;
}

static int igb_dpll_pin_frequency_get(const struct dpll_pin *pin, void *pin_priv,
				      const struct dpll_device *dpll, void *dpll_priv,
				      u64 *frequency, struct netlink_ext_ack *extack)
{
	*frequency = DPLL_PIN_FREQUENCY_1_HZ;
	return 0;
}

static const struct dpll_pin_ops igb_dpll_pin_ops = {
	.state_on_dpll_get = igb_dpll_pin_state_get,
	.direction_get = igb_dpll_pin_direction_get,
	.frequency_get = igb_dpll_pin_frequency_get,
};

static struct dpll_pin_frequency igb_dpll_1pps = DPLL_PIN_FREQUENCY_1PPS;
static const struct dpll_pin_properties igb_dpll_pin_prop[2] = {
	{ .board_label = "PRIREF", .type = DPLL_PIN_TYPE_GNSS, .freq_supported_num = 1, .freq_supported = &igb_dpll_1pps },
	{ .board_label = "SECREF", .type = DPLL_PIN_TYPE_EXT, .freq_supported_num = 1, .freq_supported = &igb_dpll_1pps },
};

void igb_dpll_register(struct igb_adapter *adapter)
{
	struct igb_ref *r = &adapter->ref;
	int i, err;

	if (!adapter->i2c_lmk05318b)
		return;
	r->clock_id = ether_addr_to_u64(adapter->netdev->dev_addr);
	r->dpll = dpll_device_get(r->clock_id, 0, THIS_MODULE, &r->dpll_tracker);
	if (IS_ERR(r->dpll)) {
		dev_warn(&adapter->pdev->dev, "dpll device get failed: %ld\n", PTR_ERR(r->dpll));
		r->dpll = NULL;
		return;
	}
	err = dpll_device_register(r->dpll, DPLL_TYPE_PPS, &igb_dpll_ops, adapter);
	if (err) {
		dev_warn(&adapter->pdev->dev, "dpll device register failed: %d\n", err);
		dpll_device_put(r->dpll, &r->dpll_tracker);
		r->dpll = NULL;
		return;
	}
	for (i = 0; i < 2; i++) {
		r->pin[i] = dpll_pin_get(r->clock_id, i, THIS_MODULE, &igb_dpll_pin_prop[i], &r->pin_tracker[i]);
		if (IS_ERR(r->pin[i])) {
			dev_warn(&adapter->pdev->dev, "dpll pin %d get failed: %ld\n", i, PTR_ERR(r->pin[i]));
			r->pin[i] = NULL;
			continue;
		}
		err = dpll_pin_register(r->dpll, r->pin[i], &igb_dpll_pin_ops, (void *)(long)i);
		if (err) {
			dev_warn(&adapter->pdev->dev, "dpll pin %d register failed: %d\n", i, err);
			dpll_pin_put(r->pin[i], &r->pin_tracker[i]);
			r->pin[i] = NULL;
		}
	}
	r->dpll_status = DPLL_LOCK_STATUS_UNLOCKED;
	dev_info(&adapter->pdev->dev, "dpll device registered, clock id %016llx\n", r->clock_id);
}

void igb_dpll_unregister(struct igb_adapter *adapter)
{
	struct igb_ref *r = &adapter->ref;
	int i;

	if (!r->dpll)
		return;
	for (i = 0; i < 2; i++) {
		if (!r->pin[i])
			continue;
		dpll_pin_unregister(r->dpll, r->pin[i], &igb_dpll_pin_ops, (void *)(long)i);
		dpll_pin_put(r->pin[i], &r->pin_tracker[i]);
		r->pin[i] = NULL;
	}
	dpll_device_unregister(r->dpll, &igb_dpll_ops, adapter);
	dpll_device_put(r->dpll, &r->dpll_tracker);
	r->dpll = NULL;
}

/* From the DPLL worker after each status pass. */
void igb_dpll_notify(struct igb_adapter *adapter)
{
	struct igb_ref *r = &adapter->ref;
	int i, v;

	if (!r->dpll)
		return;
	v = igb_dpll_status(adapter);
	if (v != r->dpll_status) {
		r->dpll_status = v;
		dpll_device_change_ntf(r->dpll);
	}
	for (i = 0; i < 2; i++) {
		if (!r->pin[i])
			continue;
		v = igb_dpll_pin_state(adapter, i);
		if (v != r->pin_state[i]) {
			r->pin_state[i] = v;
			dpll_pin_change_ntf(r->pin[i]);
		}
	}
}

#else /* CONFIG_DPLL */

void igb_dpll_register(struct igb_adapter *adapter)
{
	if (adapter->i2c_lmk05318b)
		dev_info(&adapter->pdev->dev, "kernel without CONFIG_DPLL: no dpll device\n");
}

void igb_dpll_unregister(struct igb_adapter *adapter) { }
void igb_dpll_notify(struct igb_adapter *adapter) { }

#endif /* CONFIG_DPLL */
