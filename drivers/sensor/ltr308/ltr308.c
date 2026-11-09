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

/* Register code to value; rate code 4 is reserved */
static const uint8_t ltr308_gains[] = {1, 3, 6, 9, 18};
static const uint16_t ltr308_rates_ms[] = {25, 50, 100, 500, 0, 1000, 2000};

struct ltr308_config {
	struct i2c_dt_spec i2c;
	uint8_t gain_reg;
	uint8_t res_reg; /* 0..4: 20..16 bit, 400..25 ms */
	uint8_t rate_reg;
};

struct ltr308_data {
	uint32_t counts;
	uint8_t gain_reg;
	uint8_t res_reg;
	uint8_t rate_reg;
};

static uint16_t ltr308_integration_ms(uint8_t res_reg)
{
	return 400U >> res_reg;
}

static int ltr308_write_meas_rate(const struct device *dev, uint8_t res_reg, uint8_t rate_reg)
{
	const struct ltr308_config *cfg = dev->config;
	struct ltr308_data *data = dev->data;
	int ret;

	if (ltr308_rates_ms[rate_reg] < ltr308_integration_ms(res_reg)) {
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LTR308_REG_MEAS_RATE, (res_reg << 4) | rate_reg);
	if (ret < 0) {
		return ret;
	}

	data->res_reg = res_reg;
	data->rate_reg = rate_reg;
	return 0;
}

/*
 * Attributes apply to SENSOR_CHAN_LIGHT. GAIN is 1, 3, 6, 9 or 18;
 * RESOLUTION is 16..20 bits, which sets the integration time (25..400 ms);
 * SAMPLING_FREQUENCY is 40, 20, 10, 2, 1 or 0.5 Hz and must not be faster
 * than the integration time allows. The next conversion after a change
 * completes one measurement period later; a fetch before that returns the
 * previous count, converted with the new settings.
 */
static int ltr308_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct ltr308_config *cfg = dev->config;
	struct ltr308_data *data = dev->data;
	int64_t uhz;
	int ret;

	if (chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_GAIN:
		for (uint8_t i = 0U; i < ARRAY_SIZE(ltr308_gains); i++) {
			if (val->val1 == ltr308_gains[i] && val->val2 == 0) {
				ret = i2c_reg_write_byte_dt(&cfg->i2c, LTR308_REG_GAIN, i);
				if (ret == 0) {
					data->gain_reg = i;
				}
				return ret;
			}
		}
		return -EINVAL;
	case SENSOR_ATTR_RESOLUTION:
		if (val->val1 < 16 || val->val1 > 20 || val->val2 != 0) {
			return -EINVAL;
		}
		return ltr308_write_meas_rate(dev, (uint8_t)(20 - val->val1), data->rate_reg);
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		uhz = sensor_value_to_micro(val);
		for (uint8_t i = 0U; i < ARRAY_SIZE(ltr308_rates_ms); i++) {
			if (ltr308_rates_ms[i] != 0U && uhz * ltr308_rates_ms[i] == 1000000000LL) {
				return ltr308_write_meas_rate(dev, data->res_reg, i);
			}
		}
		return -EINVAL;
	default:
		return -ENOTSUP;
	}
}

static int ltr308_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val)
{
	struct ltr308_data *data = dev->data;

	if (chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_GAIN:
		val->val1 = ltr308_gains[data->gain_reg];
		val->val2 = 0;
		return 0;
	case SENSOR_ATTR_RESOLUTION:
		val->val1 = 20 - data->res_reg;
		val->val2 = 0;
		return 0;
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return sensor_value_from_micro(val, 1000000000LL / ltr308_rates_ms[data->rate_reg]);
	default:
		return -ENOTSUP;
	}
}

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
	struct ltr308_data *data = dev->data;
	uint64_t ulux;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	ulux = ltr308_ulux(data->counts, ltr308_gains[data->gain_reg],
			   ltr308_integration_ms(data->res_reg));
	val->val1 = (int32_t)(ulux / 1000000U);
	val->val2 = (int32_t)(ulux % 1000000U);
	return 0;
}

static DEVICE_API(sensor, ltr308_api) = {
	.sample_fetch = ltr308_sample_fetch,
	.channel_get = ltr308_channel_get,
	.attr_set = ltr308_attr_set,
	.attr_get = ltr308_attr_get,
};

static int ltr308_init(const struct device *dev)
{
	const struct ltr308_config *cfg = dev->config;
	struct ltr308_data *data = dev->data;
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
	data->gain_reg = cfg->gain_reg;

	ret = ltr308_write_meas_rate(dev, cfg->res_reg, cfg->rate_reg);
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
		.gain_reg = LTR308_GAIN_REG(n),                                                    \
		.res_reg = LTR308_RES_REG(n),                                                      \
		.rate_reg = LTR308_RATE_REG(n),                                                    \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(n, ltr308_init, NULL, &ltr308_data_##n, &ltr308_cfg_##n,      \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ltr308_api);

DT_INST_FOREACH_STATUS_OKAY(LTR308_DEFINE)
