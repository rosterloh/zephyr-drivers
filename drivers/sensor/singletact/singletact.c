#define DT_DRV_COMPAT pps_singletact

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "singletact.h"

LOG_MODULE_REGISTER(singletact, CONFIG_SENSOR_LOG_LEVEL);

static int singletact_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t size)
{
	const struct singletact_config *config = dev->config;
	uint8_t cmd[3] = {SINGLETACT_CMD_READ, reg, size};

	return i2c_write_read_dt(&config->i2c, cmd, sizeof(cmd), buf, size);
}

static int singletact_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct singletact_config *config = dev->config;
	uint8_t cmd[5] = {SINGLETACT_CMD_WRITE, reg, 1U, val, 0xFF};

	return i2c_write_dt(&config->i2c, cmd, sizeof(cmd));
}

static int singletact_set_address(const struct device *dev, uint8_t address)
{
	return singletact_reg_write(dev, SINGLETACT_REG_ADDRESS, address);
}

static int singletact_read_output(const struct device *dev)
{
	struct singletact_data *data = dev->data;
	uint8_t read_data[6];
	int ret;

	ret = singletact_reg_read(dev, SINGLETACT_REG_FRAME_IDX_MSB, read_data, sizeof(read_data));
	if (ret < 0) {
		return ret;
	}

	/* Get the data */
	for (int i = 0; i < sizeof(read_data) / 2; i++) {
		data->output_buffer[i] = sys_get_be16(read_data + (i * 2));
	}

	if ((data->output_buffer[0] == 0xFFFF) && (data->output_buffer[1] == 0xFFFF)) {
		LOG_ERR("Read error: Invalid data received");
		return -EIO;
	}

	if (data->output_buffer[2] > SINGLETACT_MAX_OUTPUT_VALUE) {
		LOG_ERR("Read error: Invalid sensor output value %04x", data->output_buffer[2]);
		return -EINVAL;
	}

	LOG_DBG("IDX: %04x, TS: %04x, DATA: %04x", data->output_buffer[0], data->output_buffer[1],
		data->output_buffer[2]);

	return 0;
}

static int singletact_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ALL:
		ret = singletact_read_output(dev);
		break;
	default:
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	return ret;
}

static int singletact_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	const struct singletact_config *config = dev->config;
	struct singletact_data *data = dev->data;
	float output;
	int ret;

	switch ((enum sensor_channel_singletact)chan) {
	case SENSOR_CHAN_RAW:
		val->val1 = data->output_buffer[2];
		val->val2 = data->baseline;
		break;
	case SENSOR_CHAN_FORCE:
		// Load (N) = (Digital Output (Counts) - Baseline Output (Counts) / 512 (Counts)) *
		// Sensor Rating (N)
		output = ((float)data->output_buffer[2] - (float)data->baseline) / 512;
		output *= ((float)config->sensor_rating / 1000);

		ret = sensor_value_from_double(val, output);
		if (ret < 0) {
			LOG_ERR("Failed to convert output to sensor value: %d", ret);
			return ret;
		}
		break;
	default:
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	return 0;
}

static int singletact_attr_set(const struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret;

	switch ((enum sensor_attribute_singletact)attr) {
	case SENSOR_ATTR_ADDRESS:
		ret = singletact_set_address(dev, (uint8_t)val->val1);
		break;

	default:
		LOG_ERR("Invalid sensor attribute");
		ret = -EINVAL;
	}

	return ret;
}

static int singletact_setup(const struct device *dev)
{
	const struct singletact_config *config = dev->config;
	struct singletact_data *data = dev->data;
	int ret;
	uint8_t read_data[2];
	uint16_t baseline_buffer[config->baseline_samples];

	ret = singletact_reg_read(dev, SINGLETACT_REG_SERIAL_MSB, read_data, sizeof(read_data));
	if (ret < 0) {
		LOG_ERR("Failed to read device serial number");
		return ret;
	}
	data->serial = sys_get_be16(read_data);

	ret = singletact_reg_read(dev, SINGLETACT_REG_FW_REV, &data->fw, 1U);
	if (ret < 0) {
		LOG_ERR("Failed to read firmware revision");
		return ret;
	}

	// Get a baseline (NOTE: The device should be unloaded at statup)
	for (int i = 0; i < config->baseline_samples; i++) {
		ret = singletact_read_output(dev);
		if (ret < 0) {
			LOG_ERR("Failed to set baseline for output");
			continue;
		}
		baseline_buffer[i] = data->output_buffer[2];
	}

	// Calculate the average of the baseline readings
	uint64_t baseline_sum = 0;
	for (int i = 0; i < config->baseline_samples; i++) {
		baseline_sum += baseline_buffer[i];
	}
	data->baseline = baseline_sum / config->baseline_samples;

	LOG_INF("Device serial: %04x, firmware version: %02x, baseline: %d configured",
		data->serial, data->fw, data->baseline);

	return ret;
}

static int singletact_init(const struct device *dev)
{
	const struct singletact_config *config = dev->config;
	int ret;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C dev %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	ret = singletact_setup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup device");
		return ret;
	}

	LOG_DBG("Device %s initialised", dev->name);

	return 0;
}

static const struct sensor_driver_api singletact_driver_api = {
	.attr_set = singletact_attr_set,
	.sample_fetch = singletact_sample_fetch,
	.channel_get = singletact_channel_get,
};

#define SINGLETACT_INIT_INST(n)                                                                    \
	static struct singletact_data singletact_data_##n;                                         \
	static const struct singletact_config singletact_config_##n = {                            \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.baseline_samples = DT_INST_PROP(n, baseline_samples),                             \
		.sensor_rating = DT_INST_PROP(n, sensor_rating),                                   \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(n, singletact_init, NULL, &singletact_data_##n,               \
				     &singletact_config_##n, POST_KERNEL,                          \
				     CONFIG_SENSOR_INIT_PRIORITY, &singletact_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SINGLETACT_INIT_INST)
