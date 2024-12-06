#define DT_DRV_COMPAT ams_as7341

#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include "as7341.h"

LOG_MODULE_REGISTER(as7341, CONFIG_SENSOR_LOG_LEVEL);

static int as7341_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t size)
{
	const struct as7341_config *config = dev->config;

	return i2c_write_read_dt(&config->i2c, &reg, 1U, buf, size);
}

static int as7341_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct as7341_config *config = dev->config;
	uint8_t cmd[2] = {reg, val};

	return i2c_write_dt(&config->i2c, cmd, 2U);
}

int as7341_reg_update(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t old_value, new_value;
	int ret;

	ret = as7341_reg_read(dev, reg, &old_value, 1U);
	if (ret < 0) {
		return ret;
	}

	new_value = (old_value & ~mask) | (val & mask);
	if (new_value == old_value) {
		return 0;
	}

	return as7341_reg_write(dev, reg, new_value);
}

static int as7341_power_set(const struct device *dev, bool on)
{
	struct as7341_data *data = dev->data;
	int ret = 0;
	uint8_t power_val = on ? AS7341_POWER_ON : AS7341_POWER_OFF;

	ret = as7341_reg_update(dev, AS7341_REG_ENABLE, AS7341_POWER_MASK, power_val);
	if (ret < 0) {
		LOG_ERR("Failed to %s able power for the device", on ? "en" : "dis");
		return ret;
	}

	data->powered_on = on;
	return 0;
}

static int as7341_spectral_enable(const struct device *dev, bool enabled)
{
	int ret = 0;
	uint8_t spectral_val = enabled ? AS7341_SPECTRAL_ON : AS7341_SPECTRAL_OFF;

	ret = as7341_reg_update(dev, AS7341_REG_ENABLE, AS7341_SPECTRAL_MASK, spectral_val);
	if (ret < 0) {
		LOG_ERR("Failed to %s integrating", enabled ? "start" : "stop");
	}

	return ret;
}

static int as7341_mux_set_low(const struct device *dev, bool low)
{
	int ret = 0;

	ret = as7341_spectral_enable(dev, false);
	if (ret < 0) {
		return ret;
	}

	ret = as7341_reg_update(dev, AS7341_REG_CFG6, AS7341_SMUX_CMD_MASK, AS7341_SMUX_CMD_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to write SMUX_CMD");
		return ret;
	}

	if (low) {
		// SMUX Config for F1,F2,F3,F4,NIR,Clear
		ret = as7341_reg_write(dev, 0x00, 0x30);  // F3 left set to ADC2
		ret |= as7341_reg_write(dev, 0x01, 0x01); // F1 left set to ADC0
		ret |= as7341_reg_write(dev, 0x02, 0x00); // Reserved or disabled
		ret |= as7341_reg_write(dev, 0x03, 0x00); // F8 left disabled
		ret |= as7341_reg_write(dev, 0x04, 0x00); // F6 left disabled
		ret |= as7341_reg_write(
			dev, 0x05, 0x42); // F4 left connected to ADC3/f2 left connected to ADC1
		ret |= as7341_reg_write(dev, 0x06, 0x00); // F5 left disabled
		ret |= as7341_reg_write(dev, 0x07, 0x00); // F7 left disabled
		ret |= as7341_reg_write(dev, 0x08, 0x50); // CLEAR connected to ADC4
		ret |= as7341_reg_write(dev, 0x09, 0x00); // F5 right disabled
		ret |= as7341_reg_write(dev, 0x0A, 0x00); // F7 right disabled
		ret |= as7341_reg_write(dev, 0x0B, 0x00); // Reserved or disabled
		ret |= as7341_reg_write(dev, 0x0C, 0x20); // F2 right connected to ADC1
		ret |= as7341_reg_write(dev, 0x0D, 0x04); // F4 right connected to ADC3
		ret |= as7341_reg_write(dev, 0x0E, 0x00); // F6/F8 right disabled
		ret |= as7341_reg_write(dev, 0x0F, 0x30); // F3 right connected to AD2
		ret |= as7341_reg_write(dev, 0x10, 0x01); // F1 right connected to AD0
		ret |= as7341_reg_write(dev, 0x11, 0x50); // CLEAR right connected to AD4
		ret |= as7341_reg_write(dev, 0x12, 0x00); // Reserved or disabled
		ret |= as7341_reg_write(dev, 0x13, 0x06); // NIR connected to ADC5
	} else {
		// SMUX Config for F5,F6,F7,F8,NIR,Clear
		ret = as7341_reg_write(dev, 0x00, 0x00);  // F3 left disable
		ret |= as7341_reg_write(dev, 0x01, 0x00); // F1 left disable
		ret |= as7341_reg_write(dev, 0x02, 0x00); // reserved/disable
		ret |= as7341_reg_write(dev, 0x03, 0x40); // F8 left connected to ADC3
		ret |= as7341_reg_write(dev, 0x04, 0x02); // F6 left connected to ADC1
		ret |= as7341_reg_write(dev, 0x05, 0x00); // F4/ F2 disabled
		ret |= as7341_reg_write(dev, 0x06, 0x10); // F5 left connected to ADC0
		ret |= as7341_reg_write(dev, 0x07, 0x03); // F7 left connected to ADC2
		ret |= as7341_reg_write(dev, 0x08, 0x50); // CLEAR Connected to ADC4
		ret |= as7341_reg_write(dev, 0x09, 0x10); // F5 right connected to ADC0
		ret |= as7341_reg_write(dev, 0x0A, 0x03); // F7 right connected to ADC2
		ret |= as7341_reg_write(dev, 0x0B, 0x00); // Reserved or disabled
		ret |= as7341_reg_write(dev, 0x0C, 0x00); // F2 right disabled
		ret |= as7341_reg_write(dev, 0x0D, 0x00); // F4 right disabled
		ret |= as7341_reg_write(
			dev, 0x0E, 0x24); // F8 right connected to ADC2/ F6 right connected to ADC1
		ret |= as7341_reg_write(dev, 0x0F, 0x00); // F3 right disabled
		ret |= as7341_reg_write(dev, 0x10, 0x00); // F1 right disabled
		ret |= as7341_reg_write(dev, 0x11, 0x50); // CLEAR right connected to AD4
		ret |= as7341_reg_write(dev, 0x12, 0x00); // Reserved or disabled
		ret |= as7341_reg_write(dev, 0x13, 0x06); // NIR connected to ADC5
	}

	ret = as7341_reg_update(dev, AS7341_REG_ENABLE, AS7341_SMUXEN_MASK, AS7341_SMUXEN_ON);
	if (ret < 0) {
		LOG_ERR("Failed to enable SMUX");
	}

	/* Read until cleared */
	int timeout = 1000; // If it takes 1000 milliseconds then something is wrong
	int count = 0;
	bool smux_enabled = true;
	uint8_t enable = 0;
	while (smux_enabled && count < timeout) {
		ret = as7341_reg_read(dev, AS7341_REG_ENABLE, &enable, 1U);
		if (ret < 0) {
			smux_enabled = enable & AS7341_SMUXEN_MASK;
		}
		k_busy_wait(1000);
		count++;
	}
	if (count >= timeout) {
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int as7341_delay_for_data(const struct device *dev, int wait_ms)
{
	int ret = 0;
	uint8_t status;

	if (wait_ms == 0) { // Wait forever
		bool data_ready = false;
		while (!data_ready) {
			ret = as7341_reg_read(dev, AS7341_REG_STATUS2, &status, 1U);
			if (ret < 0) {
				data_ready = status & AS7341_AVALID_ON;
			}
			k_busy_wait(1000);
		}
		return ret;
	}

	if (wait_ms > 0) { // Wait for that many milliseconds
		uint32_t elapsed_ms = 0;
		bool data_ready = false;
		while (!data_ready && elapsed_ms < (uint32_t)wait_ms) {
			ret = as7341_reg_read(dev, AS7341_REG_STATUS2, &status, 1U);
			if (ret < 0) {
				data_ready = status & AS7341_AVALID_ON;
			}
			k_busy_wait(1000);
			elapsed_ms++;
		}
		return ret;
	}

	if (wait_ms < 0) {
		ret = -ENOTSUP;
	}

	return ret;
}

static int as7341_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as7341_data *data = dev->data;
	uint8_t read_data[12];
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ALL:
		ret = as7341_mux_set_low(dev, true);
		if (ret < 0) {
			return ret;
		}

		ret = as7341_spectral_enable(dev, true);
		if (ret < 0) {
			return ret;
		}

		ret = as7341_delay_for_data(dev, 0);

		ret = as7341_reg_read(dev, AS7341_REG_CH0_DATA, read_data, sizeof(read_data));
		if (ret < 0) {
			return ret;
		}

		/* Get the data */
		for (int i = 0; i < sizeof(read_data) / 2; i++) {
			data->channel_buffer[i] = sys_get_le16(read_data + (i * 2));
		}

		ret = as7341_mux_set_low(dev, false);
		if (ret < 0) {
			return ret;
		}

		ret = as7341_spectral_enable(dev, true);
		if (ret < 0) {
			return ret;
		}

		ret = as7341_delay_for_data(dev, 0);

		ret = as7341_reg_read(dev, AS7341_REG_CH0_DATA, read_data, 12U);
		if (ret < 0) {
			return ret;
		}

		for (int i = 0; i < sizeof(read_data) / 2; i++) {
			data->channel_buffer[6 + i] = sys_get_le16(read_data + (i * 2));
		}

		break;
	default:
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	return ret;
}

static int as7341_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct as7341_data *data = dev->data;
	uint16_t channel;

	switch (chan) {
	case SENSOR_CHAN_LIGHT:
		channel = data->channel_buffer[AS7341_CHANNEL_CLEAR];
		break;
	case SENSOR_CHAN_IR:
		channel = data->channel_buffer[AS7341_CHANNEL_NIR];
	default:
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	val->val1 = channel;
	val->val2 = 0;

	return 0;
}

static int as7341_set_gain(const struct device *dev, enum sensor_gain_as7341 gain)
{
	struct as7341_data *data = dev->data;
	int ret;

	ret = as7341_reg_write(dev, AS7341_REG_CFG1, gain);
	if (ret < 0) {
		LOG_ERR("Failed to set gain mode");
	}

	data->again = gain;

	return ret;
}

/* Total integration time will be (ATIME + 1) * (ASTEP + 1) * 2.78µS */
static int as7341_set_integration(const struct device *dev, uint8_t time, uint16_t step)
{
	struct as7341_data *data = dev->data;
	int ret;

	ret = as7341_reg_write(dev, AS7341_REG_ATIME, time);
	if (ret < 0) {
		LOG_ERR("Failed to set integration steps");
		return ret;
	}

	ret = as7341_reg_write(dev, AS7341_REG_ASTEP, step);
	if (ret < 0) {
		LOG_ERR("Failed to set integration step time");
		return ret;
	}

	data->atime = time;
	data->astep = step;
	data->integration_ms = (time + 1) * (step + 1) * 2.78 / 1000;

	return 0;
}

static int as7341_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret;

	ret = as7341_power_set(dev, false);
	if (ret < 0) {
		return ret;
	}

	switch ((enum sensor_attribute_as7341)attr) {
	case SENSOR_ATTR_GAIN_MODE:
		ret = as7341_set_gain(dev, (enum sensor_gain_as7341)val->val1);
		break;
	case SENSOR_ATTR_INTEGRATION_TIME:
		ret = as7341_set_integration(dev, val->val1, val->val2);
		break;

	default:
		LOG_ERR("Invalid sensor attribute");
		ret = -EINVAL;
	}

	ret = as7341_power_set(dev, true);

	return ret;
}

static int as7341_setup(const struct device *dev)
{
	struct as7341_data *data = dev->data;
	uint8_t device_id;
	int ret;

	ret = as7341_reg_read(dev, AS7341_REG_ID, &device_id, 1U);
	if (ret < 0) {
		LOG_ERR("Failed to read device ID");
		return ret;
	}

	if ((device_id & 0xFC) != (AS7341_DEV_ID << 2)) {
		LOG_ERR("Device with ID 0x%02x is not supported", device_id);
		return -ENOTSUP;
	}

	/* Set initial values to match sensor values on reset */
	data->again = 256U;
	data->atime = 0U;
	data->astep = 999U;

	/* Set integration time to default recommended 50ms */
	ret = as7341_set_integration(dev, 29, 599);
	if (ret < 0) {
		return ret;
	}

	ret = as7341_power_set(dev, true);

	return ret;
}

static int as7341_init(const struct device *dev)
{
	const struct as7341_config *config = dev->config;
	int ret;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C dev %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	ret = as7341_setup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup device");
		return ret;
	}

	LOG_INF("Device %s initialised", dev->name);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int as7341_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct as7341_data *data = dev->data;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = as7341_power_set(dev, true);
		if (ret < 0) {
			LOG_ERR("Failed to power on device");
			return ret;
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = as7341_power_set(dev, false);
		if (ret < 0) {
			LOG_ERR("Failed to power off device");
			return ret;
		}
		break;
	default:
		LOG_ERR("Unsupported PM action");
		return -ENOTSUP;
	}

	return 0;
}
#endif

static const struct sensor_driver_api as7341_driver_api = {
	.attr_set = as7341_attr_set,
	.sample_fetch = as7341_sample_fetch,
	.channel_get = as7341_channel_get,
};

#define AS7341_INIT_INST(n)                                                                        \
	static struct as7341_data as7341_data_##n;                                                 \
	static const struct as7341_config as7341_config_##n = {.i2c = I2C_DT_SPEC_INST_GET(n)};    \
	PM_DEVICE_DT_INST_DEFINE(n, as7341_pm_action);                                             \
	SENSOR_DEVICE_DT_INST_DEFINE(n, as7341_init, PM_DEVICE_DT_INST_GET(n), &as7341_data_##n,   \
				     &as7341_config_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &as7341_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS7341_INIT_INST)
