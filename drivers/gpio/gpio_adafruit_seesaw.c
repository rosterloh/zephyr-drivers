#define DT_DRV_COMPAT adafruit_seesaw_gpio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(gpio_seesaw, CONFIG_GPIO_ADAFRUIT_SEESAW_LOG_LEVEL);

/* Mirror of register IDs (children do not share the MFD private header). */
#define SEESAW_GPIO_BASE        0x01U
#define SEESAW_GPIO_DIRSET_BULK 0x02U
#define SEESAW_GPIO_DIRCLR_BULK 0x03U
#define SEESAW_GPIO_BULK        0x04U
#define SEESAW_GPIO_BULK_SET    0x05U
#define SEESAW_GPIO_BULK_CLR    0x06U
#define SEESAW_GPIO_BULK_TOGGLE 0x07U
#define SEESAW_GPIO_INTENSET    0x08U
#define SEESAW_GPIO_INTENCLR    0x09U
#define SEESAW_GPIO_INTFLAG     0x0AU
#define SEESAW_GPIO_PULLENSET   0x0BU
#define SEESAW_GPIO_PULLENCLR   0x0CU

struct gpio_seesaw_config {
	struct gpio_driver_config common;
	const struct device *mfd;
};

struct gpio_seesaw_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	struct mfd_seesaw_int_callback mfd_int;
};

static int write_mask32(const struct device *dev, uint8_t reg, uint32_t mask)
{
	const struct gpio_seesaw_config *cfg = dev->config;
	uint8_t buf[4];

	sys_put_be32(mask, buf);
	return mfd_seesaw_write(cfg->mfd, SEESAW_GPIO_BASE, reg, buf, sizeof(buf));
}

static int read_mask32(const struct device *dev, uint8_t reg, uint32_t *out)
{
	const struct gpio_seesaw_config *cfg = dev->config;
	uint8_t buf[4];
	int ret = mfd_seesaw_read(cfg->mfd, SEESAW_GPIO_BASE, reg, buf, sizeof(buf), 0);

	if (ret == 0) {
		*out = sys_get_be32(buf);
	}
	return ret;
}

static int gpio_seesaw_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	uint32_t mask = BIT(pin);
	int ret;

	if ((flags & GPIO_OUTPUT) != 0U) {
		ret = write_mask32(dev, SEESAW_GPIO_DIRSET_BULK, mask);
		if (ret < 0) {
			return ret;
		}
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			return write_mask32(dev, SEESAW_GPIO_BULK_SET, mask);
		}
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			return write_mask32(dev, SEESAW_GPIO_BULK_CLR, mask);
		}
		return 0;
	}

	if ((flags & GPIO_INPUT) != 0U) {
		ret = write_mask32(dev, SEESAW_GPIO_DIRCLR_BULK, mask);
		if (ret < 0) {
			return ret;
		}
		if ((flags & GPIO_PULL_UP) != 0U) {
			ret = write_mask32(dev, SEESAW_GPIO_PULLENSET, mask);
			if (ret == 0) {
				ret = write_mask32(dev, SEESAW_GPIO_BULK_SET, mask);
			}
			return ret;
		}
		if ((flags & GPIO_PULL_DOWN) != 0U) {
			ret = write_mask32(dev, SEESAW_GPIO_PULLENSET, mask);
			if (ret == 0) {
				ret = write_mask32(dev, SEESAW_GPIO_BULK_CLR, mask);
			}
			return ret;
		}
		return write_mask32(dev, SEESAW_GPIO_PULLENCLR, mask);
	}

	return -ENOTSUP;
}

static int gpio_seesaw_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	uint32_t v;
	int ret = read_mask32(dev, SEESAW_GPIO_BULK, &v);

	if (ret == 0) {
		*value = v;
	}
	return ret;
}

static int gpio_seesaw_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	int ret = write_mask32(dev, SEESAW_GPIO_BULK_SET, mask & value);

	if (ret == 0) {
		ret = write_mask32(dev, SEESAW_GPIO_BULK_CLR, mask & ~value);
	}
	return ret;
}

static int gpio_seesaw_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return write_mask32(dev, SEESAW_GPIO_BULK_SET, pins);
}

static int gpio_seesaw_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return write_mask32(dev, SEESAW_GPIO_BULK_CLR, pins);
}

static int gpio_seesaw_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	return write_mask32(dev, SEESAW_GPIO_BULK_TOGGLE, pins);
}

static int gpio_seesaw_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	uint32_t mask = BIT(pin);

	ARG_UNUSED(trig);

	/* Seesaw firmware fires INTFLAG on any change; no level-trigger or
	 * direction selection. Reject level mode, accept any edge trig.
	 */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		return write_mask32(dev, SEESAW_GPIO_INTENCLR, mask);
	}
	return write_mask32(dev, SEESAW_GPIO_INTENSET, mask);
}

static int gpio_seesaw_manage_callback(const struct device *dev, struct gpio_callback *callback,
				       bool set)
{
	struct gpio_seesaw_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static void gpio_seesaw_on_mfd_int(const struct device *mfd, void *user_data)
{
	const struct device *dev = user_data;
	struct gpio_seesaw_data *data = dev->data;
	uint32_t flag;

	ARG_UNUSED(mfd);

	if (read_mask32(dev, SEESAW_GPIO_INTFLAG, &flag) == 0 && flag != 0U) {
		gpio_fire_callbacks(&data->callbacks, dev, flag);
	}
}

static DEVICE_API(gpio, gpio_seesaw_api) = {
	.pin_configure = gpio_seesaw_pin_configure,
	.port_get_raw = gpio_seesaw_port_get_raw,
	.port_set_masked_raw = gpio_seesaw_port_set_masked_raw,
	.port_set_bits_raw = gpio_seesaw_port_set_bits_raw,
	.port_clear_bits_raw = gpio_seesaw_port_clear_bits_raw,
	.port_toggle_bits = gpio_seesaw_port_toggle_bits,
	.pin_interrupt_configure = gpio_seesaw_pin_interrupt_configure,
	.manage_callback = gpio_seesaw_manage_callback,
};

static int gpio_seesaw_init(const struct device *dev)
{
	const struct gpio_seesaw_config *cfg = dev->config;
	struct gpio_seesaw_data *data = dev->data;

	if (!device_is_ready(cfg->mfd)) {
		return -ENODEV;
	}

	data->mfd_int.handler = gpio_seesaw_on_mfd_int;
	data->mfd_int.user_data = (void *)dev;
#ifdef CONFIG_MFD_ADAFRUIT_SEESAW_INT
	(void)mfd_seesaw_add_int_callback(cfg->mfd, &data->mfd_int);
#endif
	return 0;
}

#define GPIO_SEESAW_DEFINE(inst)                                                                   \
	static struct gpio_seesaw_data gpio_seesaw_data_##inst;                                    \
	static const struct gpio_seesaw_config gpio_seesaw_cfg_##inst = {                          \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst)},                \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, gpio_seesaw_init, NULL, &gpio_seesaw_data_##inst,              \
			      &gpio_seesaw_cfg_##inst, POST_KERNEL,                                \
			      CONFIG_GPIO_ADAFRUIT_SEESAW_INIT_PRIORITY, &gpio_seesaw_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_SEESAW_DEFINE)
