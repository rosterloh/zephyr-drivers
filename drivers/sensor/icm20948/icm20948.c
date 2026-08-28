/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);

/* Datasheet section 3.2/3.1 sensitivity, indexed by the full-scale register
 * field. The gyro values are fractional LSB/dps, so they are held as
 * milli-LSB: 65.5 and 16.4 are not representable otherwise, and rounding them
 * costs 0.15% of every reading. */
static const uint16_t icm20948_accel_lsb_per_g[] = {16384, 8192, 4096, 2048};
static const uint32_t icm20948_gyro_lsb_milli_per_dps[] = {131000, 65500, 32800, 16400};

static int icm20948_bank_set(const struct device *dev, uint8_t bank)
{
	const struct icm20948_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_BANK_SEL,
				     bank << ICM20948_BANK_SHIFT);
}

/* Micro-units to a sensor_value. Negative values floor toward zero the way the
 * sensor API expects: val1 and val2 carry the same sign. */
static void icm20948_set_value(struct sensor_value *val, int64_t micro)
{
	val->val1 = (int32_t)(micro / 1000000);
	val->val2 = (int32_t)(micro % 1000000);
}

static void icm20948_accel_value(const struct device *dev, int16_t raw, struct sensor_value *val)
{
	const struct icm20948_config *config = dev->config;

	/* raw/LSB_per_g * 9.80665 m/s^2, in micro-units. 32767 * 9806650 is
	 * comfortably inside int64. */
	icm20948_set_value(val,
			   ((int64_t)raw * 9806650) / icm20948_accel_lsb_per_g[config->accel_fs]);
}

static void icm20948_gyro_value(const struct device *dev, int16_t raw, struct sensor_value *val)
{
	const struct icm20948_config *config = dev->config;

	/* raw/LSB_per_dps * pi/180 rad/s, in micro-units. The sensitivity is
	 * fractional (65.5, 32.8, 16.4), so it is held as milli-LSB and the
	 * 1000x cancels into the constant: 1000 * 1e6 * pi/180 = 17453293. */
	icm20948_set_value(val, ((int64_t)raw * 17453293) /
					icm20948_gyro_lsb_milli_per_dps[config->gyro_fs]);
}

static void icm20948_temp_value(int16_t raw, struct sensor_value *val)
{
	icm20948_set_value(val, ((int64_t)raw * 1000000000) / ICM20948_TEMP_SENSITIVITY_MILLI +
					ICM20948_TEMP_OFFSET_MICRO);
}

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct icm20948_config *config = dev->config;
	struct icm20948_data *data = dev->data;
	uint8_t buf[ICM20948_SAMPLE_BYTES];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ACCEL_XYZ &&
	    chan != SENSOR_CHAN_GYRO_XYZ && chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	ret = icm20948_bank_set(dev, 0);
	if (ret < 0) {
		return ret;
	}

	/* One burst for all three: the part latches the sample registers as a
	 * set, so splitting the read by channel would risk mixing two samples. */
	ret = i2c_burst_read_dt(&config->i2c, ICM20948_REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("failed to read sample registers: %d", ret);
		return ret;
	}

	for (int i = 0; i < 3; i++) {
		data->accel[i] = (int16_t)sys_get_be16(&buf[i * 2]);
		data->gyro[i] = (int16_t)sys_get_be16(&buf[6 + i * 2]);
	}
	data->temp = (int16_t)sys_get_be16(&buf[12]);

	return 0;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_accel_value(dev, data->accel[chan - SENSOR_CHAN_ACCEL_X], val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (int i = 0; i < 3; i++) {
			icm20948_accel_value(dev, data->accel[i], &val[i]);
		}
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		icm20948_gyro_value(dev, data->gyro[chan - SENSOR_CHAN_GYRO_X], val);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		for (int i = 0; i < 3; i++) {
			icm20948_gyro_value(dev, data->gyro[i], &val[i]);
		}
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20948_temp_value(data->temp, val);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icm20948_init(const struct device *dev)
{
	const struct icm20948_config *config = dev->config;
	uint8_t id;
	int ret;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("i2c bus %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	ret = icm20948_bank_set(dev, 0);
	if (ret < 0) {
		LOG_ERR("failed to select bank 0: %d", ret);
		return ret;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, ICM20948_REG_WHO_AM_I, &id);
	if (ret < 0) {
		LOG_ERR("failed to read WHO_AM_I: %d", ret);
		return ret;
	}
	if (id != ICM20948_WHO_AM_I_VAL) {
		LOG_ERR("bad WHO_AM_I 0x%02x, expected 0x%02x", id, ICM20948_WHO_AM_I_VAL);
		return -ENODEV;
	}

	ret = i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_PWR_MGMT_1,
				    ICM20948_PWR_MGMT_1_RESET);
	if (ret < 0) {
		return ret;
	}
	/* Datasheet gives no reset time; 100 ms is what the vendor driver waits.
	 * The reset also returns the bank select to 0, so it is re-asserted
	 * below rather than assumed. */
	k_msleep(100);

	ret = icm20948_bank_set(dev, 0);
	if (ret < 0) {
		return ret;
	}

	/* Out of sleep and onto the best available clock. The part comes up
	 * asleep, and a sleeping device returns stale sample registers rather
	 * than an error. */
	ret = i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_PWR_MGMT_1,
				    ICM20948_PWR_MGMT_1_CLK_AUTO);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_PWR_MGMT_2,
				    ICM20948_PWR_MGMT_2_ALL_ON);
	if (ret < 0) {
		return ret;
	}

	ret = icm20948_bank_set(dev, 2);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_GYRO_CONFIG_1,
				    config->gyro_fs << ICM20948_FS_SEL_SHIFT);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_ACCEL_CONFIG,
				    config->accel_fs << ICM20948_FS_SEL_SHIFT);
	if (ret < 0) {
		return ret;
	}

	/* Leave the device in bank 0, where the sample registers are. */
	ret = icm20948_bank_set(dev, 0);
	if (ret < 0) {
		return ret;
	}

	/* Gyro start-up is specified at 35 ms; reading before that returns
	 * whatever the register held. */
	k_msleep(35);

	return 0;
}

static DEVICE_API(sensor, icm20948_driver_api) = {
	.sample_fetch = icm20948_sample_fetch,
	.channel_get = icm20948_channel_get,
};

/*
 * The binding's enums are ordered to match the register field, so the enum
 * index IS the field value and also the sensitivity-table index. Reorder the
 * binding and every reading scales wrong with no build error, which is why the
 * ordering is called out in both places.
 */
#define ICM20948_INIT_INST(n)                                                                      \
	static struct icm20948_data icm20948_data_##n;                                             \
	static const struct icm20948_config icm20948_config_##n = {                                \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.accel_fs = DT_INST_ENUM_IDX(n, accel_fs),                                         \
		.gyro_fs = DT_INST_ENUM_IDX(n, gyro_fs),                                           \
	};                                                                                         \
	BUILD_ASSERT(DT_INST_ENUM_IDX(n, accel_fs) < ARRAY_SIZE(icm20948_accel_lsb_per_g));        \
	BUILD_ASSERT(DT_INST_ENUM_IDX(n, gyro_fs) < ARRAY_SIZE(icm20948_gyro_lsb_milli_per_dps));  \
	SENSOR_DEVICE_DT_INST_DEFINE(n, icm20948_init, NULL, &icm20948_data_##n,                   \
				     &icm20948_config_##n, POST_KERNEL,                            \
				     CONFIG_SENSOR_INIT_PRIORITY, &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_INIT_INST)
