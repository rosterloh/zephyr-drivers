/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * OpenTherm master: Manchester at 1 kbit/s, written from the OpenTherm v2.2
 * timing. Logical GPIO level 1 is the first half of a '1' bit; set the DT
 * flags so that holds for the adapter.
 */

#define DT_DRV_COMPAT opentherm_gpio

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/opentherm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(opentherm, CONFIG_OPENTHERM_LOG_LEVEL);

#define OT_HALF_BIT_US   500U
#define OT_FRAME_BITS    34U /* start + 32 data + stop */
/* Edges closer than this to the last mid-bit edge are bit boundaries. */
#define OT_MID_MIN_US    750U
/* Spec allows mid-bit edges 900..1150 us apart; anything later lost an edge. */
#define OT_MID_MAX_US    1250U
/* Frame time plus the slave's 800 ms reply window, with margin. */
#define OT_REPLY_TIMEOUT K_MSEC(1100)
#define OT_MSG_GAP_MS    100

BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC % (USEC_PER_SEC / OT_HALF_BIT_US) == 0,
	     "OpenTherm clocks 500 us half-bits from a k_timer: set "
	     "CONFIG_SYS_CLOCK_TICKS_PER_SEC to a multiple of 2000, e.g. 10000");

enum ot_rx_state {
	OT_RX_OFF,
	OT_TX,
	OT_RX_WAIT,
	OT_RX_START,
	OT_RX_DATA,
};

struct ot_config {
	struct gpio_dt_spec out;
	struct gpio_dt_spec in;
};

struct ot_data {
	const struct device *dev;
	struct gpio_callback cb;
	struct k_timer tx_timer;
	struct k_mutex lock;
	struct k_sem done;
	enum ot_rx_state state;
	uint64_t tx_word;
	uint32_t t_mid;
	uint32_t frame;
	uint8_t bits;
	uint8_t tx_half;
	int rx_err;
	int64_t last_msg_ms;
};

static void rx_finish(struct ot_data *data, int err)
{
	data->state = OT_RX_OFF;
	data->rx_err = err;
	k_sem_give(&data->done);
}

static void ot_edge(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct ot_data *data = CONTAINER_OF(cb, struct ot_data, cb);
	const struct ot_config *cfg = data->dev->config;
	uint32_t now = k_cycle_get_32();
	uint32_t gap = k_cyc_to_us_floor32(now - data->t_mid);
	int level = gpio_pin_get_dt(&cfg->in);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	if (level < 0) {
		return;
	}

	switch (data->state) {
	case OT_RX_WAIT:
		if (level == 1) {
			data->state = OT_RX_START;
			data->t_mid = now;
		}
		break;
	case OT_RX_START:
		if (level == 0 && gap < OT_MID_MIN_US) {
			data->state = OT_RX_DATA;
			data->t_mid = now;
			data->frame = 0U;
			data->bits = 0U;
		} else {
			/* Glitch, not a start bit: wait for the next rising edge. */
			data->state = (level == 1) ? OT_RX_START : OT_RX_WAIT;
			data->t_mid = now;
		}
		break;
	case OT_RX_DATA:
		if (gap < OT_MID_MIN_US) {
			break;
		}
		if (gap > OT_MID_MAX_US) {
			rx_finish(data, -EBADMSG);
			break;
		}
		data->t_mid = now;
		/* '1' is active then idle, so the line is idle after its mid-bit edge. */
		if (data->bits < 32U) {
			data->frame = (data->frame << 1) | (level == 0 ? 1U : 0U);
			data->bits++;
		} else {
			rx_finish(data, level == 0 ? 0 : -EBADMSG);
		}
		break;
	default:
		break;
	}
}

/* Timer ISR: drives one half-bit per expiry, then hands the line to RX. */
static void ot_tx_half(struct k_timer *timer)
{
	struct ot_data *data = CONTAINER_OF(timer, struct ot_data, tx_timer);
	const struct ot_config *cfg = data->dev->config;
	uint8_t half = data->tx_half++;
	int level = 0;

	if (half < 2U * OT_FRAME_BITS) {
		bool one = ((data->tx_word >> (OT_FRAME_BITS - 1U - half / 2U)) & 1U) != 0U;

		level = ((half & 1U) == 0U) == one;
	} else {
		k_timer_stop(timer);
		data->state = OT_RX_WAIT;
	}

	if (gpio_pin_set_dt(&cfg->out, level) != 0) {
		k_timer_stop(timer);
		rx_finish(data, -EIO);
	}
}

int opentherm_transfer(const struct device *dev, uint32_t request, uint32_t *response)
{
	const struct ot_config *cfg = dev->config;
	struct ot_data *data = dev->data;
	int64_t gap;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	gap = k_uptime_get() - data->last_msg_ms;
	if (gap < OT_MSG_GAP_MS) {
		k_msleep((int32_t)(OT_MSG_GAP_MS - gap));
	}

	k_sem_reset(&data->done);
	/* Start and stop bits are both '1'. */
	data->tx_word = BIT64(33) | ((uint64_t)request << 1) | 1U;
	data->tx_half = 0U;
	data->state = OT_TX;
	k_timer_start(&data->tx_timer, K_NO_WAIT, K_USEC(OT_HALF_BIT_US));

	ret = k_sem_take(&data->done, OT_REPLY_TIMEOUT);
	if (ret != 0) {
		k_timer_stop(&data->tx_timer);
		data->state = OT_RX_OFF;
		(void)gpio_pin_set_dt(&cfg->out, 0);
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = data->rx_err;
	if (ret == 0 && (__builtin_popcount(data->frame) & 1) != 0) {
		ret = -EBADMSG;
	}
	if (ret == 0) {
		*response = data->frame;
	}

out:
	if (ret != 0) {
		LOG_DBG("request 0x%08x: %d", request, ret);
	}
	data->last_msg_ms = k_uptime_get();
	k_mutex_unlock(&data->lock);

	return ret;
}

static int ot_init(const struct device *dev)
{
	const struct ot_config *cfg = dev->config;
	struct ot_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->out) || !gpio_is_ready_dt(&cfg->in)) {
		return -ENODEV;
	}

	data->dev = dev;
	data->state = OT_RX_OFF;
	k_mutex_init(&data->lock);
	k_sem_init(&data->done, 0, 1);
	k_timer_init(&data->tx_timer, ot_tx_half, NULL);

	ret = gpio_pin_configure_dt(&cfg->out, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->in, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	gpio_init_callback(&data->cb, ot_edge, BIT(cfg->in.pin));
	ret = gpio_add_callback_dt(&cfg->in, &data->cb);
	if (ret != 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&cfg->in, GPIO_INT_EDGE_BOTH);
}

#define OT_DEFINE(inst)                                                                            \
	static const struct ot_config ot_config_##inst = {                                         \
		.out = GPIO_DT_SPEC_INST_GET(inst, out_gpios),                                     \
		.in = GPIO_DT_SPEC_INST_GET(inst, in_gpios),                                       \
	};                                                                                         \
	static struct ot_data ot_data_##inst;                                                      \
	DEVICE_DT_INST_DEFINE(inst, ot_init, NULL, &ot_data_##inst, &ot_config_##inst,             \
			      POST_KERNEL, CONFIG_OPENTHERM_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(OT_DEFINE)
