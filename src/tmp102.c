// SPDX-License-Identifier: GPL-2.0
/* TMP102 temperature sensors on the Octo board 2409. */
#include <linux/i2c.h>
#include <linux/swab.h>
#include "tmp102.h"

int tmp102_init(struct i2c_client *client, int lo_limit, int hi_limit)
{
	int res;

	res = i2c_smbus_write_word_data(client, TMP102_TLOW_REG, __swab16(lo_limit * 256));
	res = i2c_smbus_write_word_data(client, TMP102_THIGH_REG, __swab16(hi_limit * 256));
	return res;
}

long tmp102_read(struct i2c_client *client)
{
	__u16 regval = __swab16(i2c_smbus_read_word_data(client, TMP102_TEMP_REG));

	return tmp102_reg_to_mC(regval);
}

int tmp102_read_alert(struct i2c_client *client)
{
	__u16 regval = __swab16(i2c_smbus_read_word_data(client, TMP102_CONF_REG));

	return (regval & TMP102_CONF_AL) != TMP102_CONF_AL;
}
