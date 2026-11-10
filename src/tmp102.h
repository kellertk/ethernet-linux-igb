/* SPDX-License-Identifier: GPL-2.0 */
/* TMP102 temperature sensors on the Octo board 2409: board (0x49) and OCXO (0x4a). */
#ifndef _TMP102_
#define _TMP102_

#include <linux/types.h>

#define TMP102_TEMP_REG                 0x00
#define TMP102_CONF_REG                 0x01
/* note: these bit definitions are byte swapped */
#define         TMP102_CONF_SD          0x0100
#define         TMP102_CONF_TM          0x0200
#define         TMP102_CONF_POL         0x0400
#define         TMP102_CONF_F0          0x0800
#define         TMP102_CONF_F1          0x1000
#define         TMP102_CONF_R0          0x2000
#define         TMP102_CONF_R1          0x4000
#define         TMP102_CONF_OS          0x8000
#define         TMP102_CONF_EM          0x0010
#define         TMP102_CONF_AL          0x0020
#define         TMP102_CONF_CR0         0x0040
#define         TMP102_CONF_CR1         0x0080
#define TMP102_TLOW_REG                 0x02
#define TMP102_THIGH_REG                0x03

#define TMP102_CONFREG_MASK     (TMP102_CONF_SD | TMP102_CONF_TM | \
                                 TMP102_CONF_POL | TMP102_CONF_F0 | \
                                 TMP102_CONF_F1 | TMP102_CONF_OS | \
                                 TMP102_CONF_EM | TMP102_CONF_AL | \
                                 TMP102_CONF_CR0 | TMP102_CONF_CR1)

#define TMP102_CONFIG_CLEAR     (TMP102_CONF_SD | TMP102_CONF_OS | \
                                 TMP102_CONF_CR0)
#define TMP102_CONFIG_SET       (TMP102_CONF_TM | TMP102_CONF_EM | \
                                 TMP102_CONF_CR1)

#define CONVERSION_TIME_MS              35      /* in milli-seconds */

/* convert left adjusted 13-bit TMP102 register value to milliCelsius */
static inline int tmp102_reg_to_mC(s16 val)
{
        return ((val & ~0x01) * 1000) / 128;
}

/* convert milliCelsius to left adjusted 13-bit TMP102 register value */
static inline u16 tmp102_mC_to_reg(int val)
{
        return (val * 128) / 1000;
}

struct i2c_client;

/* alert window in whole degrees C; returns the last write's status */
int tmp102_init(struct i2c_client *client, int lo_limit, int hi_limit);
/* temperature in milliCelsius */
long tmp102_read(struct i2c_client *client);
/* 1 while the temperature is outside the alert window */
int tmp102_read_alert(struct i2c_client *client);

#endif /* _TMP102_ */
