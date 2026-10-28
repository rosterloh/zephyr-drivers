/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * Copyright (c) 2026 Richard Osterloh
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Out-of-tree fork of Zephyr's drivers/sensor/ti/ina3221/ina3221.c, taken to
 * add the read/decode (RTIO) sensor API: the in-tree driver only implements
 * fetch/get, so reading three rails means a fetch plus a private
 * SENSOR_ATTR_INA3221_SELECTED_CHANNEL write per rail. It binds the same
 * "ti,ina3221" compatible, so select exactly one of CONFIG_INA3221 (in-tree)
 * and CONFIG_INA3221_OOT (this file).
 *
 * The fetch/get path is unchanged apart from moving the measurement into
 * ina3221_measure(), which both paths share, under a mutex.
 */

#define DT_DRV_COMPAT ti_ina3221

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#ifdef CONFIG_SENSOR_ASYNC_API
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/rtio/work.h>
#endif

#include "ina3221.h"

LOG_MODULE_REGISTER(INA3221, CONFIG_SENSOR_LOG_LEVEL);

#define MAX_RETRIES 10

static const int32_t avg_mode_samples[8] = {1, 4, 16, 64, 128, 256, 512, 1024};
static const int32_t conv_time_us[8] = {140, 204, 332, 588, 1100, 2116, 4156, 8244};

static int reg_read(const struct device *dev, uint8_t reg_addr, uint16_t *reg_data)
{
	const struct ina3221_config *cfg = dev->config;
	uint8_t rx_buf[2];
	int rc;

	rc = i2c_write_read_dt(&cfg->bus, &reg_addr, sizeof(reg_addr), rx_buf, sizeof(rx_buf));

	*reg_data = sys_get_be16(rx_buf);

	return rc;
}

static int reg_write(const struct device *dev, uint8_t addr, uint16_t reg_data)
{
	const struct ina3221_config *cfg = dev->config;
	uint8_t tx_buf[3];

	tx_buf[0] = addr;
	sys_put_be16(reg_data, &tx_buf[1]);

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

static int ina3221_init(const struct device *dev)
{
	int ret;
	const struct ina3221_config *cfg = dev->config;
	struct ina3221_data *data = dev->data;
	uint16_t reg_data;

	__ASSERT_NO_MSG(cfg->avg_mode < 8);
	__ASSERT_NO_MSG(cfg->conv_time_bus < 8);
	__ASSERT_NO_MSG(cfg->conv_time_shunt < 8);

	k_mutex_init(&data->lock);

	/* select first enabled channel */
	for (size_t i = 0; i < 3; ++i) {
		if (cfg->enable_channel[i]) {
			data->selected_channel = i;
			break;
		}
	}

	/* check bus */
	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR_DEVICE_NOT_READY(cfg->bus.bus);
		return -ENODEV;
	}

	/* check connection */
	ret = reg_read(dev, INA3221_MANUF_ID, &reg_data);
	if (ret) {
		LOG_ERR("No answer from sensor.");
		return ret;
	}
	if (reg_data != INA3221_MANUF_ID_VALUE) {
		LOG_ERR("Unexpected manufacturer ID: 0x%04x", reg_data);
		return -EFAULT;
	}
	ret = reg_read(dev, INA3221_CHIP_ID, &reg_data);
	if (ret) {
		return ret;
	}
	if (reg_data != INA3221_CHIP_ID_VALUE) {
		LOG_ERR("Unexpected chip ID: 0x%04x", reg_data);
		return -EFAULT;
	}

	/* reset */
	ret = reg_read(dev, INA3221_CONFIG, &reg_data);
	if (ret) {
		return ret;
	}
	reg_data |= INA3221_CONFIG_RST;
	ret = reg_write(dev, INA3221_CONFIG, reg_data);
	if (ret) {
		return ret;
	}

	/* configure */
	reg_data = (cfg->conv_time_shunt << 3) | (cfg->conv_time_bus << 6) | (cfg->avg_mode << 9) |
		   (INA3221_CONFIG_CH1 * cfg->enable_channel[0]) |
		   (INA3221_CONFIG_CH2 * cfg->enable_channel[1]) |
		   (INA3221_CONFIG_CH3 * cfg->enable_channel[2]);

	ret = reg_write(dev, INA3221_CONFIG, reg_data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int start_measurement(const struct device *dev, bool bus, bool shunt)
{
	int ret;
	uint16_t reg_data;

	ret = reg_read(dev, INA3221_CONFIG, &reg_data);
	if (ret) {
		return ret;
	}

	reg_data &= ~(INA3221_CONFIG_BUS | INA3221_CONFIG_SHUNT);
	reg_data |= (INA3221_CONFIG_BUS * bus) | (INA3221_CONFIG_SHUNT * shunt);

	ret = reg_write(dev, INA3221_CONFIG, reg_data);
	if (ret) {
		return ret;
	}
	return 0;
}

static bool measurement_ready(const struct device *dev)
{
	int ret;
	uint16_t reg_data;

	ret = reg_read(dev, INA3221_MASK_ENABLE, &reg_data);
	if (ret) {
		return false;
	}
	return reg_data & INA3221_MASK_ENABLE_CONVERSION_READY;
}

static int ina3221_measure_locked(const struct device *dev, enum sensor_channel chan,
				  int16_t v_bus[INA3221_NUM_CHANNELS],
				  int16_t v_shunt[INA3221_NUM_CHANNELS])
{
	const struct ina3221_config *cfg = dev->config;
	bool measurement_successful = false;
	k_timeout_t measurement_time = K_NO_WAIT;
	uint8_t enabled_channels = 0;
	int32_t channel_conv_time_us = 0;
	int ret;

	/* Trigger measurement and wait for completion */
	if (chan == SENSOR_CHAN_VOLTAGE) {
		ret = start_measurement(dev, true, false);
		if (ret) {
			return ret;
		}
		channel_conv_time_us = conv_time_us[cfg->conv_time_bus];
	} else if (chan == SENSOR_CHAN_CURRENT) {
		ret = start_measurement(dev, false, true);
		if (ret) {
			return ret;
		}
		channel_conv_time_us = conv_time_us[cfg->conv_time_shunt];
	} else if (chan == SENSOR_CHAN_POWER || chan == SENSOR_CHAN_ALL) {
		ret = start_measurement(dev, true, true);
		if (ret) {
			return ret;
		}
		channel_conv_time_us =
			conv_time_us[cfg->conv_time_bus] + conv_time_us[cfg->conv_time_shunt];
	} else {
		return -ENOTSUP;
	}

	for (size_t i = 0; i < 3; ++i) {
		if (cfg->enable_channel[i]) {
			enabled_channels++;
		}
	}

	/* Measurements are performed sequentially for all enabled channels. */
	/* See chapter 7.3.1 in the datasheet. */
	measurement_time =
		K_USEC(enabled_channels * avg_mode_samples[cfg->avg_mode] * channel_conv_time_us);

	for (size_t i = 0; i < MAX_RETRIES; ++i) {
		k_sleep(measurement_time);
		if (measurement_ready(dev)) {
			measurement_successful = true;
			break;
		}
	}

	if (!measurement_successful) {
		LOG_ERR("Measurement timed out.");
		return -EFAULT;
	}

	for (size_t i = 0; i < 3; ++i) {
		uint16_t raw;

		if (!cfg->enable_channel[i]) {
			continue;
		}
		if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POWER ||
		    chan == SENSOR_CHAN_VOLTAGE) {
			ret = reg_read(dev, INA3221_BUS_V1 + 2 * i, &raw);
			if (ret) {
				return ret;
			}
			v_bus[i] = (int16_t)raw >> 3;
		}
		if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POWER ||
		    chan == SENSOR_CHAN_CURRENT) {
			ret = reg_read(dev, INA3221_SHUNT_V1 + 2 * i, &raw);
			if (ret) {
				return ret;
			}
			v_shunt[i] = (int16_t)raw >> 3;
		}
	}

	return ret;
}

int ina3221_measure(const struct device *dev, enum sensor_channel chan,
		    int16_t v_bus[INA3221_NUM_CHANNELS], int16_t v_shunt[INA3221_NUM_CHANNELS])
{
	struct ina3221_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = ina3221_measure_locked(dev, chan, v_bus, v_shunt);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int ina3221_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ina3221_data *data = dev->data;

	return ina3221_measure(dev, chan, data->v_bus, data->v_shunt);
}

#ifdef CONFIG_SENSOR_ASYNC_API
static void ina3221_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *read_cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = read_cfg->sensor;
	const struct ina3221_config *cfg = dev->config;
	struct ina3221_encoded_data *edata;
	uint32_t min_buf_len = sizeof(*edata);
	uint32_t buf_len;
	uint8_t *buf;
	uint64_t cycles;
	int rc;

	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	edata = (struct ina3221_encoded_data *)buf;
	memset(edata, 0, sizeof(*edata));

	/* One conversion of everything enabled, whichever channels were asked
	 * for: the decoder picks out the rails, and splitting the read would
	 * only cost extra conversions.
	 */
	rc = ina3221_measure(dev, SENSOR_CHAN_ALL, edata->v_bus, edata->v_shunt);
	if (rc != 0) {
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	edata->timestamp_ns = sensor_clock_cycles_to_ns(cycles);
	for (size_t i = 0; i < INA3221_NUM_CHANNELS; i++) {
		if (cfg->enable_channel[i]) {
			edata->channels |= BIT(i);
		}
		edata->shunt_r[i] = cfg->shunt_r[i];
	}

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

void ina3221_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();

	ARG_UNUSED(dev);

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider increasing "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, ina3221_submit_sync);
}
#endif /* CONFIG_SENSOR_ASYNC_API */

static int ina3221_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct ina3221_config *cfg = dev->config;
	struct ina3221_data *data = dev->data;
	float result;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		result = data->v_bus[data->selected_channel] * INA3221_BUS_VOLTAGE_LSB;
		break;
	case SENSOR_CHAN_CURRENT:
		result = data->v_shunt[data->selected_channel] * INA3221_SHUNT_VOLTAGE_LSB /
			 (cfg->shunt_r[data->selected_channel] / 1000.0f);
		break;
	case SENSOR_CHAN_POWER:
		result = data->v_bus[data->selected_channel] * INA3221_BUS_VOLTAGE_LSB *
			 data->v_shunt[data->selected_channel] * INA3221_SHUNT_VOLTAGE_LSB /
			 (cfg->shunt_r[data->selected_channel] / 1000.0f);
		break;
	default:
		LOG_DBG("Channel not supported by device!");
		return -ENOTSUP;
	}

	return sensor_value_from_float(val, result);
}

static int ina3221_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	ARG_UNUSED(chan);

	if (attr != SENSOR_ATTR_INA3221_SELECTED_CHANNEL) {
		return -ENOTSUP;
	}

	struct ina3221_data *data = dev->data;

	if (val->val1 < 1 || val->val1 > 3) {
		return -EINVAL;
	}

	data->selected_channel = val->val1 - 1;
	return 0;
}

static DEVICE_API(sensor, ina3221_api) = {
	.sample_fetch = ina3221_sample_fetch,
	.channel_get = ina3221_channel_get,
	.attr_set = ina3221_attr_set,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = ina3221_submit,
	.get_decoder = ina3221_get_decoder,
#endif
};

#define INST_DT_INA3221(index)                                                                     \
	static const struct ina3221_config ina3221_config_##index = {                              \
		.bus = I2C_DT_SPEC_INST_GET(index),                                                \
		.avg_mode = DT_INST_PROP(index, avg_mode),                                         \
		.conv_time_bus = DT_INST_PROP(index, conv_time_bus),                               \
		.conv_time_shunt = DT_INST_PROP(index, conv_time_shunt),                           \
		.enable_channel = DT_INST_PROP(index, enable_channel),                             \
		.shunt_r = DT_INST_PROP(index, shunt_resistors),                                   \
	};                                                                                         \
	static struct ina3221_data ina3221_data_##index;                                           \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(index, ina3221_init, NULL, &ina3221_data_##index,             \
				     &ina3221_config_##index, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &ina3221_api);

DT_INST_FOREACH_STATUS_OKAY(INST_DT_INA3221);
