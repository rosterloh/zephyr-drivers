/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lite-On LTR-308ALS-01 ambient light sensor.
 */

#define DT_DRV_COMPAT liteon_ltr308

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ltr308.h"

LOG_MODULE_REGISTER(ltr308, CONFIG_SENSOR_LOG_LEVEL);

struct ltr308_config {
	struct i2c_dt_spec i2c;
	uint8_t gain; /* 1, 3, 6, 9 or 18 */
	uint8_t gain_reg;
	uint16_t integration_ms;
	uint8_t meas_rate_reg;
};

struct ltr308_data {
	uint32_t counts;
};

static int ltr308_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct ltr308_config *cfg = dev->config;
	struct ltr308_data *data = dev->data;
	uint8_t buf[3];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	ret = i2c_burst_read_dt(&cfg->i2c, LTR308_REG_DATA_0, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	data->counts = sys_get_le24(buf) & LTR308_DATA_MASK;
	return 0;
}

static int ltr308_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct ltr308_config *cfg = dev->config;
	struct ltr308_data *data = dev->data;
	uint64_t ulux;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	ulux = ltr308_ulux(data->counts, cfg->gain, cfg->integration_ms);
	val->val1 = (int32_t)(ulux / 1000000U);
	val->val2 = (int32_t)(ulux % 1000000U);
	return 0;
}

static DEVICE_API(sensor, ltr308_api) = {
	.sample_fetch = ltr308_sample_fetch,
	.channel_get = ltr308_channel_get,
};

static int ltr308_init(const struct device *dev)
{
	const struct ltr308_config *cfg = dev->config;
	uint8_t id;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&cfg->i2c, LTR308_REG_PART_ID, &id);
	if (ret < 0) {
		LOG_ERR("Unable to read part ID: %d", ret);
		return ret;
	}
	if (id != LTR308_PART_ID) {
		LOG_ERR("Wrong part ID 0x%02x", id);
		return -ENODEV;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LTR308_REG_GAIN, cfg->gain_reg);
	if (ret < 0) {
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, LTR308_REG_MEAS_RATE, cfg->meas_rate_reg);
	if (ret < 0) {
		return ret;
	}

	return i2c_reg_write_byte_dt(&cfg->i2c, LTR308_REG_MAIN_CTRL, LTR308_ALS_ENABLE);
}

/* Register codes, by the order of the binding's enums */
#define LTR308_GAIN_REG(n) DT_INST_ENUM_IDX(n, gain)
#define LTR308_RES_REG(n)  DT_INST_ENUM_IDX(n, integration_time_ms)
#define LTR308_RATE_REG(n)                                                                         \
	(DT_INST_ENUM_IDX(n, measurement_rate_ms) +                                                \
	 (DT_INST_ENUM_IDX(n, measurement_rate_ms) >= 4 ? 1 : 0))

#define LTR308_DEFINE(n)                                                                           \
	BUILD_ASSERT(DT_INST_PROP(n, measurement_rate_ms) >= DT_INST_PROP(n, integration_time_ms), \
		     "measurement-rate-ms shorter than integration-time-ms");                      \
	static struct ltr308_data ltr308_data_##n;                                                 \
	static const struct ltr308_config ltr308_cfg_##n = {                                       \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.gain = DT_INST_PROP(n, gain),                                                     \
		.gain_reg = LTR308_GAIN_REG(n),                                                    \
		.integration_ms = DT_INST_PROP(n, integration_time_ms),                            \
		.meas_rate_reg = (LTR308_RES_REG(n) << 4) | LTR308_RATE_REG(n),                    \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(n, ltr308_init, NULL, &ltr308_data_##n, &ltr308_cfg_##n,      \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ltr308_api);

DT_INST_FOREACH_STATUS_OKAY(LTR308_DEFINE)
