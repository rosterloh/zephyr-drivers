#define DT_DRV_COMPAT ams_as7341

#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include "as7341.h"

LOG_MODULE_REGISTER(as7341, CONFIG_SENSOR_LOG_LEVEL);

static int as7341_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t size)
{
	const struct as7341_config *config = dev->config;
	uint8_t cmd = TSL2591_NORMAL_CMD | reg;

	return i2c_write_read_dt(&config->i2c, &cmd, 1U, buf, size);
}

static int as7341_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct as7341_config *config = dev->config;
	uint8_t cmd[2] = {TSL2591_NORMAL_CMD | reg, val};

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

static int as7341_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as7341_dev_data *dev_data = dev->data;
	const struct as7341_dev_cfg *dev_cfg = dev->config;
	uint8_t read_data[12];
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ALL:
		ret = as7341_reg_read(dev, TSL2591_REG_C0DATAL, als_data, 4U);
		if (ret < 0) {
			LOG_ERR("Failed to read ALS data");
			return ret;
		}

		data->vis_count = sys_get_le16(als_data);
		data->ir_count = sys_get_le16(als_data + 2);
		break;
	case SENSOR_CHAN_LIGHT:
		ret = tsl2591_reg_read(dev, TSL2591_REG_C0DATAL, als_data, 2U);
		if (ret < 0) {
			LOG_ERR("Failed to read ALS visible light data");
			return ret;
		}

		data->vis_count = sys_get_le16(als_data);
		break;
	case SENSOR_CHAN_IR:
		ret = tsl2591_reg_read(dev, TSL2591_REG_C1DATAL, als_data, 2U);
		if (ret < 0) {
			LOG_ERR("Failed to read ALS infrared data");
			return ret;
		}

		data->ir_count = sys_get_le16(als_data);
		break;
	default:
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	return err;
}

static int as7341_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct as7341_dev_data *dev_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ALL:
		val->val1 = ((int32_t)dev_data->position * AS5600_FULL_ANGLE) /
							AS5600_PULSES_PER_REV;

		val->val2 = (((int32_t)dev_data->position * AS5600_FULL_ANGLE) %
			     AS5600_PULSES_PER_REV) * (AS5600_MILLION_UNIT / AS5600_PULSES_PER_REV);
		break;
	case SENSOR_CHAN_LIGHT:
		break;
	case SENSOR_CHAN_IR:
		break;
	default:
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	if (chan == SENSOR_CHAN_ALL) {
		
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int as7341_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct as7341_data *data = dev->data;
	int ret;

	ret = as7341_reg_update(dev, AS7341_REG_ENABLE, AS7341_POWER_MASK, AS7341_POWER_OFF);
	if (ret < 0) {
		LOG_ERR("Unable to power down device");
		return ret;
	}

	switch ((enum sensor_attribute_tsl2591)attr) {
	case SENSOR_ATTR_GAIN_MODE:
		ret = tsl2591_set_gain(dev, (enum sensor_gain_tsl2591)val->val1);
		break;
	case SENSOR_ATTR_INTEGRATION_TIME:
		ret = tsl2591_set_integration(dev, val->val1);
		break;

#ifdef CONFIG_TSL2591_TRIGGER
	case SENSOR_ATTR_INT_PERSIST:
		ret = tsl2591_set_persist(dev, val->val1);
		break;
#endif
	default:
		LOG_ERR("Invalid sensor attribute");
		ret = -EINVAL;
		goto exit; /* So the compiler doesn't warn if triggers not enabled */
	}

exit:
	if (data->powered_on) {
		ret = tsl2591_reg_update(dev, TSL2591_REG_ENABLE, TSL2591_POWER_MASK,
					 TSL2591_POWER_ON);
	}

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
	data->atime = 278U;

	ret = tsl2591_reg_write(dev, AS7341_REG_ENABLE, AS7341_POWER_ON);
	if (ret < 0) {
		LOG_ERR("Failed to perform initial power up of device");
		return ret;
	}

	data->powered_on = true;

	return 0;
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
		ret = as7341_reg_update(dev, AS7341_REG_ENABLE, AS7341_POWER_MASK,
					AS7341_POWER_ON);
		if (ret < 0) {
			LOG_ERR("Failed to power on device");
			return ret;
		}

		data->powered_on = true;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = as7341_reg_update(dev, AS7341_REG_ENABLE, AS7341_POWER_MASK,
					AS7341_POWER_OFF);
		if (ret < 0) {
			LOG_ERR("Failed to power off device");
			return ret;
		}

		data->powered_on = false;
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

#define AS7341_INIT_INST(n)							\
	static struct as7341_data as7341_data_##n;				\
	static const struct as7341_config as7341_config_##n = {			\
		.i2c = I2C_DT_SPEC_INST_GET(n)					\
	};									\
	PM_DEVICE_DT_INST_DEFINE(n, as7341_pm_action);				\
	SENSOR_DEVICE_DT_INST_DEFINE(n, as7341_init, PM_DEVICE_DT_INST_GET(n),	\
				     &as7341_data_##n, &as7341_config_##n,	\
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
				     &as7341_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS7341_INIT_INST)
