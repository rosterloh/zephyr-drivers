/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * OpenTherm master on two GPIOs: a k_timer ISR clocks out half-bits and GPIO
 * edge interrupts decode the reply. Logical GPIO level 1 is the first half
 * of a '1' bit; set the DT flags so that holds for the adapter.
 */

#define DT_DRV_COMPAT opentherm_gpio

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/opentherm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "opentherm_common.h"

LOG_MODULE_REGISTER(opentherm_gpio, CONFIG_OPENTHERM_LOG_LEVEL);

BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC % (USEC_PER_SEC / OT_HALF_BIT_US) == 0,
	     "OpenTherm clocks 500 us half-bits from a k_timer: set "
	     "CONFIG_SYS_CLOCK_TICKS_PER_SEC to a multiple of 2000, e.g. 10000");

struct ot_gpio_config {
	struct gpio_dt_spec out;
	struct gpio_dt_spec in;
};

struct ot_gpio_data {
	const struct device *dev;
	struct gpio_callback cb;
	struct k_timer tx_timer;
	struct k_mutex lock;
	struct k_sem done;
	struct ot_rx rx;
	uint32_t tx_frame;
	uint8_t tx_half;
	bool rx_armed;
	int64_t last_msg_ms;
};

static void ot_gpio_finish(struct ot_gpio_data *data, int err)
{
	data->rx_armed = false;
	if (err != 0) {
		data->rx.err = err;
	}
	k_sem_give(&data->done);
}

static void ot_gpio_edge(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct ot_gpio_data *data = CONTAINER_OF(cb, struct ot_gpio_data, cb);
	const struct ot_gpio_config *cfg = data->dev->config;
	uint32_t now = k_cycle_get_32();
	int level;

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	if (!data->rx_armed) {
		return;
	}

	level = gpio_pin_get_dt(&cfg->in);
	if (level >= 0 && ot_rx_edge(&data->rx, now, level)) {
		ot_gpio_finish(data, 0);
	}
}

/* Timer ISR: drives one half-bit per expiry, then hands the line to RX. */
static void ot_gpio_tx_half(struct k_timer *timer)
{
	struct ot_gpio_data *data = CONTAINER_OF(timer, struct ot_gpio_data, tx_timer);
	const struct ot_gpio_config *cfg = data->dev->config;
	uint8_t half = data->tx_half++;
	int level = 0;

	if (half < 2U * OT_FRAME_BITS) {
		level = ot_tx_level(data->tx_frame, half);
	} else {
		k_timer_stop(timer);
		data->rx_armed = true;
	}

	if (gpio_pin_set_dt(&cfg->out, level) != 0) {
		k_timer_stop(timer);
		ot_gpio_finish(data, -EIO);
	}
}

static int ot_gpio_transfer(const struct device *dev, uint32_t request, uint32_t *response)
{
	const struct ot_gpio_config *cfg = dev->config;
	struct ot_gpio_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ot_msg_gap(data->last_msg_ms);

	k_sem_reset(&data->done);
	ot_rx_reset(&data->rx);
	data->tx_frame = request;
	data->tx_half = 0U;
	k_timer_start(&data->tx_timer, K_NO_WAIT, K_USEC(OT_HALF_BIT_US));

	if (k_sem_take(&data->done, K_MSEC(OT_REPLY_MS)) != 0) {
		k_timer_stop(&data->tx_timer);
		data->rx_armed = false;
		(void)gpio_pin_set_dt(&cfg->out, 0);
		ret = -ETIMEDOUT;
	} else {
		ret = ot_rx_result(&data->rx, response);
	}

	if (ret != 0) {
		LOG_DBG("request 0x%08x: %d", request, ret);
	}
	data->last_msg_ms = k_uptime_get();
	k_mutex_unlock(&data->lock);

	return ret;
}

static DEVICE_API(opentherm, ot_gpio_api) = {
	.transfer = ot_gpio_transfer,
};

static int ot_gpio_init(const struct device *dev)
{
	const struct ot_gpio_config *cfg = dev->config;
	struct ot_gpio_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->out) || !gpio_is_ready_dt(&cfg->in)) {
		return -ENODEV;
	}

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_sem_init(&data->done, 0, 1);
	k_timer_init(&data->tx_timer, ot_gpio_tx_half, NULL);

	ret = gpio_pin_configure_dt(&cfg->out, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->in, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	gpio_init_callback(&data->cb, ot_gpio_edge, BIT(cfg->in.pin));
	ret = gpio_add_callback_dt(&cfg->in, &data->cb);
	if (ret != 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&cfg->in, GPIO_INT_EDGE_BOTH);
}

#define OT_GPIO_DEFINE(inst)                                                                       \
	static const struct ot_gpio_config ot_gpio_config_##inst = {                               \
		.out = GPIO_DT_SPEC_INST_GET(inst, out_gpios),                                     \
		.in = GPIO_DT_SPEC_INST_GET(inst, in_gpios),                                       \
	};                                                                                         \
	static struct ot_gpio_data ot_gpio_data_##inst;                                            \
	DEVICE_DT_INST_DEFINE(inst, ot_gpio_init, NULL, &ot_gpio_data_##inst,                      \
			      &ot_gpio_config_##inst, POST_KERNEL, CONFIG_OPENTHERM_INIT_PRIORITY, \
			      &ot_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(OT_GPIO_DEFINE)
