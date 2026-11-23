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
#define OT_REPLY_TIMEOUT K_MSEC(1000)
#define OT_MSG_GAP_MS    100

enum ot_rx_state {
	OT_RX_OFF,
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
	struct k_mutex lock;
	struct k_sem done;
	enum ot_rx_state state;
	uint32_t t_mid;
	uint32_t frame;
	uint8_t bits;
	int rx_err;
	int64_t last_msg_ms;
};

static uint32_t us_since(uint32_t cyc)
{
	return k_cyc_to_us_floor32(k_cycle_get_32() - cyc);
}

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

static void wait_until(uint32_t start, uint32_t us)
{
	uint32_t elapsed;

	while ((elapsed = us_since(start)) < us) {
		k_busy_wait(us - elapsed);
	}
}

static int tx_frame(const struct ot_config *cfg, uint32_t frame)
{
	/* Start and stop bits are both '1'. */
	uint64_t word = BIT64(33) | ((uint64_t)frame << 1) | 1U;
	uint32_t start;
	int ret = 0;

	/*
	 * shortcut: busy-waits ~34 ms with the scheduler locked (ISRs still run);
	 * move to a hardware timer or RMT if that starves other threads.
	 */
	k_sched_lock();
	start = k_cycle_get_32();
	for (uint32_t half = 0U; half < 2U * OT_FRAME_BITS && ret == 0; half++) {
		bool one = ((word >> (OT_FRAME_BITS - 1U - half / 2U)) & 1U) != 0U;

		ret = gpio_pin_set_dt(&cfg->out, ((half & 1U) == 0U) == one);
		wait_until(start, (half + 1U) * OT_HALF_BIT_US);
	}
	k_sched_unlock();

	return ret;
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
	ret = tx_frame(cfg, request);
	if (ret != 0) {
		gpio_pin_set_dt(&cfg->out, 0);
		goto out;
	}

	data->state = OT_RX_WAIT;
	ret = k_sem_take(&data->done, OT_REPLY_TIMEOUT);
	if (ret != 0) {
		data->state = OT_RX_OFF;
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
