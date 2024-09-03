#include <zephyr/kernel.h>

#include "seesaw.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(seesaw, CONFIG_SEESAW_LOG_LEVEL);

static inline void seesaw_setup_int(const struct seesaw_config *cfg, bool enable)
{
	unsigned int flags = enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, flags);
}

static void seesaw_process_int(const struct device *dev)
{
	struct seesaw_data *data = dev->data;
	const struct seesaw_config *config = dev->config;

	/* Clear the status */

	if (data->int_cb != NULL) {
		data->int_cb(dev);
	}

	seesaw_setup_int(config, true);
}

static void seesaw_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct seesaw_data *data = CONTAINER_OF(cb, struct seesaw_data, gpio_cb);
	const struct seesaw_config *config = data->dev->config;

	seesaw_setup_int(config, false);
#if defined(CONFIG_SEESAW_TRIGGER_OWN_THREAD)
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_SEESAW_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

#ifdef CONFIG_SEESAW_TRIGGER_OWN_THREAD
static void seesaw_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct seesaw_data *data = p1;

	while (42) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		seesaw_process_int(data->dev);
	}
}
#endif

#ifdef CONFIG_SEESAW_TRIGGER_GLOBAL_THREAD
static void seesaw_work_cb(struct k_work *work)
{
	struct seesaw_data *data = CONTAINER_OF(work, struct seesaw_data, work);

	seesaw_process_int(data->dev);
}
#endif

int seesaw_set_int_callback(const struct device *dev, seesaw_int_callback_t int_cb)
{
	struct seesaw_data *data = dev->data;
	const struct seesaw_config *config = dev->config;

	seesaw_setup_int(dev, false);

	data->int_cb = int_cb;

	seesaw_setup_int(dev, true);

	/* Check whether already asserted */
	int pv = gpio_pin_get_dt(&config->int_gpio);

	if (pv > 0) {
		seesaw_handle_int(dev);
	}

	return 0;
}

int seesaw_init_interrupt(const struct device *dev)
{
	struct seesaw_data *data = dev->data;
	const struct seesaw_config *config = dev->config;

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("%s: device %s is not ready", dev->name, config->int_gpio.port->name);
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | config->int_gpio.dt_flags);

	gpio_init_callback(&data->gpio_cb, seesaw_gpio_callback, BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port, &data->gpio_cb) < 0) {
		LOG_DBG("Failed to set gpio callback!");
		return -EIO;
	}

	data->dev = dev;

#if defined(CONFIG_SEESAW_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack, CONFIG_SEESAW_THREAD_STACK_SIZE,
			seesaw_thread, data, NULL, NULL, K_PRIO_COOP(CONFIG_SEESAW_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_SEESAW_TRIGGER_GLOBAL_THREAD)
	data->work.handler = seesaw_work_cb;
#endif

	return 0;
}