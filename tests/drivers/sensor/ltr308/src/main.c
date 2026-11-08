/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include "ltr308.h"

#define DT_DRV_COMPAT liteon_ltr308

/* Register-file emulator: what the driver wrote, and data the test sets. */
static uint8_t regs[0x30];

static int ltr308_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				int addr)
{
	uint8_t reg;

	ARG_UNUSED(target);
	ARG_UNUSED(addr);

	if (num_msgs < 1 || (msgs[0].flags & I2C_MSG_READ) || msgs[0].len < 1) {
		return -EIO;
	}
	reg = msgs[0].buf[0];

	if (num_msgs == 1) {
		for (uint32_t i = 1; i < msgs[0].len; i++) {
			regs[reg + i - 1] = msgs[0].buf[i];
		}
		return 0;
	}

	if (num_msgs == 2 && (msgs[1].flags & I2C_MSG_READ)) {
		for (uint32_t i = 0; i < msgs[1].len; i++) {
			msgs[1].buf[i] = regs[reg + i];
		}
		return 0;
	}

	return -EIO;
}

static const struct i2c_emul_api ltr308_emul_api = {
	.transfer = ltr308_emul_transfer,
};

static int ltr308_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	regs[LTR308_REG_PART_ID] = LTR308_PART_ID;
	return 0;
}

EMUL_DT_INST_DEFINE(0, ltr308_emul_init, NULL, NULL, &ltr308_emul_api, NULL);

static const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(ltr308));

ZTEST(ltr308, test_init_config)
{
	zassert_true(device_is_ready(dev));
	zassert_equal(regs[LTR308_REG_MAIN_CTRL], LTR308_ALS_ENABLE);
	/* 6x gain is code 2 */
	zassert_equal(regs[LTR308_REG_GAIN], 2);
	/* 50 ms / 17 bit is code 3, 500 ms rate is code 3 */
	zassert_equal(regs[LTR308_REG_MEAS_RATE], 0x33);
}

ZTEST(ltr308, test_lux)
{
	struct sensor_value val;

	/* 0x12345 counts, 20-bit field with junk in the top nibble */
	regs[LTR308_REG_DATA_0] = 0x45;
	regs[LTR308_REG_DATA_0 + 1] = 0x23;
	regs[LTR308_REG_DATA_0 + 2] = 0xf1;

	zassert_ok(sensor_sample_fetch(dev));
	zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &val));

	/* 74565 * 0.6 / 6 * (100 / 50) = 14913 lux */
	zassert_equal(val.val1, 14913);
	zassert_equal(val.val2, 0);
	zassert_equal(sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val), -ENOTSUP);
}

ZTEST(ltr308, test_ulux_formula)
{
	/* Power-on defaults: 3x, 100 ms; one count is 0.2 lux */
	zassert_equal(ltr308_ulux(1, 3, 100), 200000);
	/* Full scale at 18x, 400 ms stays in range */
	zassert_equal(ltr308_ulux(LTR308_DATA_MASK, 18, 400), 8738125000ULL);
}

ZTEST_SUITE(ltr308, NULL, NULL, NULL, NULL, NULL);
