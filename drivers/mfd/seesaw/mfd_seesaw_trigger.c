#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>

#include "mfd_seesaw_regs.h"

LOG_MODULE_DECLARE(mfd_seesaw, CONFIG_MFD_ADAFRUIT_SEESAW_LOG_LEVEL);

/*
 * Mirror of layouts in mfd_seesaw.c. Keep in sync — if a field is added or
 * reordered there, update here. Both files share the unconditional layout
 * (no #ifdef) because this translation unit is only compiled when
 * CONFIG_MFD_ADAFRUIT_SEESAW_INT is set, so the conditional fields are
 * always present in this build.
 */
struct mfd_seesaw_config {
	struct i2c_dt_spec i2c;
	uint32_t read_delay_us;
	struct gpio_dt_spec int_gpio;
};

struct mfd_seesaw_data {
	struct k_mutex lock;
	uint8_t hw_id;
	uint16_t pid;
	uint32_t options;
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_work work;
	sys_slist_t int_callbacks;
};

static void mfd_seesaw_work_handler(struct k_work *work)
{
	struct mfd_seesaw_data *data = CONTAINER_OF(work, struct mfd_seesaw_data, work);
	const struct device *mfd = data->dev;
	struct mfd_seesaw_int_callback *cb;

	SYS_SLIST_FOR_EACH_CONTAINER(&data->int_callbacks, cb, node) {
		cb->handler(mfd, cb->user_data);
	}
}

static void mfd_seesaw_gpio_callback(const struct device *port, struct gpio_callback *gcb,
				     uint32_t pins)
{
	struct mfd_seesaw_data *data = CONTAINER_OF(gcb, struct mfd_seesaw_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	(void)k_work_submit(&data->work);
}

int mfd_seesaw_init_interrupt(const struct device *mfd)
{
	const struct mfd_seesaw_config *cfg = mfd->config;
	struct mfd_seesaw_data *data = mfd->data;
	int ret;

	data->dev = mfd;
	sys_slist_init(&data->int_callbacks);
	k_work_init(&data->work, mfd_seesaw_work_handler);

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, mfd_seesaw_gpio_callback, BIT(cfg->int_gpio.pin));
	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		return ret;
	}
	return gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

int mfd_seesaw_add_int_callback(const struct device *mfd, struct mfd_seesaw_int_callback *cb)
{
	struct mfd_seesaw_data *data = mfd->data;

	if (cb == NULL || cb->handler == NULL) {
		return -EINVAL;
	}
	sys_slist_append(&data->int_callbacks, &cb->node);
	return 0;
}

int mfd_seesaw_remove_int_callback(const struct device *mfd, struct mfd_seesaw_int_callback *cb)
{
	struct mfd_seesaw_data *data = mfd->data;

	if (cb == NULL) {
		return -EINVAL;
	}
	return sys_slist_find_and_remove(&data->int_callbacks, &cb->node) ? 0 : -ENOENT;
}
