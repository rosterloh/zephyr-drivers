/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include <icm20948.h>
#include <icm20948_emul.h>

#define ACCEL_XOUT_H ICM20948_REG_ACCEL_XOUT_H
#define GYRO_XOUT_H  (ICM20948_REG_ACCEL_XOUT_H + 6)
#define TEMP_OUT_H   (ICM20948_REG_ACCEL_XOUT_H + 12)

struct icm20948_fixture {
	const struct device *dev;
	const struct emul *target;
};

static void *icm20948_setup(void)
{
	static struct icm20948_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(icm20948)),
		.target = EMUL_DT_GET(DT_NODELABEL(icm20948)),
	};

	zassert_not_null(fixture.dev);
	zassert_not_null(fixture.target);
	return &fixture;
}

/**
 * @brief The device came up: WHO_AM_I matched and init returned success.
 */
ZTEST_F(icm20948, test_device_ready)
{
	zexpect_true(device_is_ready(fixture->dev));
}

/**
 * @brief Init leaves the part in bank 0, where the sample registers live.
 *
 * The register map is banked and nothing errors on a wrong bank, so a fetch
 * issued while bank 2 is selected would silently read configuration bytes as
 * acceleration.
 */
ZTEST_F(icm20948, test_init_leaves_bank_0)
{
	zexpect_equal(0, icm20948_emul_current_bank(fixture->target));
}

/**
 * @brief The full-scale selects reached bank 2, in the register's bit field.
 *
 * Both default to the part's power-on range, so the expected field value is 0
 * shifted into bits 2:1 — this is the check that the driver wrote them to the
 * right bank at all.
 */
ZTEST_F(icm20948, test_full_scale_written_to_bank_2)
{
	zexpect_equal(0 << ICM20948_FS_SEL_SHIFT,
		      icm20948_emul_get_reg(fixture->target, 2, ICM20948_REG_GYRO_CONFIG_1));
	zexpect_equal(0 << ICM20948_FS_SEL_SHIFT,
		      icm20948_emul_get_reg(fixture->target, 2, ICM20948_REG_ACCEL_CONFIG));
}

/**
 * @brief Accelerometer counts convert to m/s^2 at the default +/-2 g.
 *
 * 16384 LSB/g, so a full 16384 counts is exactly one g. The negative case is
 * here because val1 and val2 must carry the same sign for the sensor API, and
 * C division truncating toward zero is what makes that work.
 */
ZTEST_F(icm20948, test_accel_conversion)
{
	struct sensor_value val[3];

	icm20948_emul_set_sample(fixture->target, ACCEL_XOUT_H, 16384);
	icm20948_emul_set_sample(fixture->target, ACCEL_XOUT_H + 2, -16384);
	icm20948_emul_set_sample(fixture->target, ACCEL_XOUT_H + 4, 0);

	zassert_ok(sensor_sample_fetch(fixture->dev));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_ACCEL_XYZ, val));

	zexpect_equal(9, val[0].val1);
	zexpect_equal(806650, val[0].val2);
	zexpect_equal(-9, val[1].val1);
	zexpect_equal(-806650, val[1].val2);
	zexpect_equal(0, val[2].val1);
	zexpect_equal(0, val[2].val2);
}

/**
 * @brief Gyroscope counts convert to rad/s at the default +/-250 dps.
 *
 * 131 LSB/dps, so 131 counts is 1 deg/s = 0.017453 rad/s. This is the reading
 * that the fractional sensitivities (65.5, 32.8, 16.4 at the wider ranges)
 * would round away if they were held as whole LSB.
 */
ZTEST_F(icm20948, test_gyro_conversion)
{
	struct sensor_value val[3];

	icm20948_emul_set_sample(fixture->target, GYRO_XOUT_H, 131);
	icm20948_emul_set_sample(fixture->target, GYRO_XOUT_H + 2, -131);
	icm20948_emul_set_sample(fixture->target, GYRO_XOUT_H + 4, 0);

	zassert_ok(sensor_sample_fetch(fixture->dev));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_GYRO_XYZ, val));

	zexpect_equal(0, val[0].val1);
	zexpect_equal(17453, val[0].val2);
	zexpect_equal(0, val[1].val1);
	zexpect_equal(-17453, val[1].val2);
	zexpect_equal(0, val[2].val2);
}

/**
 * @brief Die temperature applies the datasheet's offset, not just the scale.
 *
 * raw/333.87 + 21 degC. A driver that dropped the +21 would read a plausible
 * room temperature as 0, so the zero-count case is the one that catches it.
 */
ZTEST_F(icm20948, test_temp_conversion)
{
	struct sensor_value val;

	icm20948_emul_set_sample(fixture->target, TEMP_OUT_H, 0);

	zassert_ok(sensor_sample_fetch(fixture->dev));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_DIE_TEMP, &val));

	zexpect_equal(21, val.val1);
	zexpect_equal(0, val.val2);
}

/**
 * @brief A fetch reads all three sensors in one burst.
 *
 * The part latches the sample registers as a set, so a driver that read them
 * per channel could mix two samples. Loading all three and asking for one
 * proves the burst covered the whole block.
 */
ZTEST_F(icm20948, test_single_burst_covers_all_channels)
{
	struct sensor_value accel, gyro, temp;

	icm20948_emul_set_sample(fixture->target, ACCEL_XOUT_H + 4, 16384);
	icm20948_emul_set_sample(fixture->target, GYRO_XOUT_H, 131);
	icm20948_emul_set_sample(fixture->target, TEMP_OUT_H, 0);

	zassert_ok(sensor_sample_fetch(fixture->dev));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_ACCEL_Z, &accel));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_GYRO_X, &gyro));
	zassert_ok(sensor_channel_get(fixture->dev, SENSOR_CHAN_DIE_TEMP, &temp));

	zexpect_equal(9, accel.val1);
	zexpect_equal(17453, gyro.val2);
	zexpect_equal(21, temp.val1);
}

/**
 * @brief An unsupported channel is refused rather than answered with zeros.
 */
ZTEST_F(icm20948, test_unsupported_channel)
{
	struct sensor_value val;

	zassert_ok(sensor_sample_fetch(fixture->dev));
	zexpect_equal(-ENOTSUP, sensor_channel_get(fixture->dev, SENSOR_CHAN_MAGN_XYZ, &val));
}

ZTEST_SUITE(icm20948, NULL, icm20948_setup, NULL, NULL, NULL);
