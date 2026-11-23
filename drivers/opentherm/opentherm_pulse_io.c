/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * OpenTherm master on two pulse_io channels (e.g. ESP32 RMT): the hardware
 * times the request, captures the reply, and the shared decoder walks the
 * captured symbols.
 */

#define DT_DRV_COMPAT opentherm_pulse_io

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/opentherm.h>
#include <zephyr/drivers/pulse_io.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "opentherm_common.h"

LOG_MODULE_REGISTER(opentherm_pulse_io, CONFIG_OPENTHERM_LOG_LEVEL);

#define OT_PIO_RES_HZ  1000000U /* 1 tick = 1 us */
/* No edge for 2 ms ends the reply; the longest run in a frame is ~1.15 ms. */
#define OT_PIO_IDLE_US 2000U
#define OT_PIO_HALVES  (2U * OT_FRAME_BITS)
/* Room for a reply plus some glitches. */
#define OT_PIO_RX_SYMS (OT_PIO_HALVES + 16U)

struct ot_pio_config {
	const struct device *pio;
	uint8_t tx_ch;
	uint8_t rx_ch;
	bool tx_idle_high;
	bool rx_idle_high;
};

struct ot_pio_data {
	struct k_mutex lock;
	struct pulse_io_channel *tx;
	struct pulse_io_channel *rx;
	struct pulse_symbol syms[OT_PIO_RX_SYMS];
	int64_t last_msg_ms;
};

static int ot_pio_send(const struct device *dev, uint32_t request)
{
	const struct ot_pio_config *cfg = dev->config;
	struct ot_pio_data *data = dev->data;
	const struct pulse_io_config pcfg = {
		.resolution_hz = OT_PIO_RES_HZ,
		.mode = PULSE_IO_MODE_SYMBOL,
		.dir = PULSE_IO_DIR_TX,
		.idle_high = cfg->tx_idle_high,
	};
	const struct pulse_io_tx_req req = {
		.symbols = data->syms,
		.count = OT_PIO_HALVES,
	};
	int ret;

	for (uint32_t half = 0U; half < OT_PIO_HALVES; half++) {
		data->syms[half] = (struct pulse_symbol){
			.duration = OT_HALF_BIT_US,
			.level = (uint32_t)(ot_tx_level(request, half) ^
					    (cfg->tx_idle_high ? 1 : 0)),
		};
	}

	ret = pulse_io_channel_configure(cfg->pio, data->tx, &pcfg);
	if (ret == 0) {
		ret = pulse_io_transmit_sync(cfg->pio, data->tx, &req, K_MSEC(OT_REPLY_MS));
	}
	return ret;
}

static int ot_pio_receive(const struct device *dev, uint32_t *response)
{
	const struct ot_pio_config *cfg = dev->config;
	struct ot_pio_data *data = dev->data;
	const struct pulse_io_config pcfg = {
		.resolution_hz = OT_PIO_RES_HZ,
		.rx_idle_threshold_ticks = OT_PIO_IDLE_US,
		.mode = PULSE_IO_MODE_SYMBOL,
		.dir = PULSE_IO_DIR_RX,
		.idle_high = cfg->rx_idle_high,
	};
	const struct pulse_io_rx_req req = {
		.symbols = data->syms,
		.capacity = ARRAY_SIZE(data->syms),
	};
	struct ot_rx rx = {0};
	uint32_t t = 0U;
	int prev = 0;
	size_t count;
	int ret;

	ret = pulse_io_channel_configure(cfg->pio, data->rx, &pcfg);
	if (ret == 0) {
		ret = pulse_io_receive_sync(cfg->pio, data->rx, &req, &count, K_MSEC(OT_REPLY_MS));
	}
	if (ret != 0) {
		return ret;
	}

	/* Replay the captured runs as edges; the line idles at logical 0. */
	ot_rx_reset(&rx);
	for (size_t i = 0U; i < count && rx.state != OT_RX_DONE; i++) {
		int level = (int)data->syms[i].level ^ (cfg->rx_idle_high ? 1 : 0);

		if (level != prev) {
			(void)ot_rx_edge(&rx, t, level);
			prev = level;
		}
		t += k_us_to_cyc_floor32(data->syms[i].duration);
	}
	if (prev != 0 && rx.state != OT_RX_DONE) {
		(void)ot_rx_edge(&rx, t, 0);
	}
	if (rx.state != OT_RX_DONE) {
		return -EBADMSG; /* truncated */
	}

	return ot_rx_result(&rx, response);
}

static int ot_pio_transfer(const struct device *dev, uint32_t request, uint32_t *response)
{
	struct ot_pio_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ot_msg_gap(data->last_msg_ms);

	ret = ot_pio_send(dev, request);
	if (ret == 0) {
		ret = ot_pio_receive(dev, response);
	} else {
		ret = -EIO;
	}

	if (ret != 0) {
		LOG_DBG("request 0x%08x: %d", request, ret);
	}
	data->last_msg_ms = k_uptime_get();
	k_mutex_unlock(&data->lock);

	return ret;
}

static DEVICE_API(opentherm, ot_pio_api) = {
	.transfer = ot_pio_transfer,
};

static int ot_pio_init(const struct device *dev)
{
	const struct ot_pio_config *cfg = dev->config;
	struct ot_pio_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->pio)) {
		return -ENODEV;
	}

	k_mutex_init(&data->lock);

	ret = pulse_io_channel_get(cfg->pio, cfg->tx_ch, &data->tx);
	if (ret != 0) {
		return ret;
	}

	/* One channel may do both directions (loopback); it is reconfigured per use. */
	if (cfg->rx_ch == cfg->tx_ch) {
		data->rx = data->tx;
		return 0;
	}

	return pulse_io_channel_get(cfg->pio, cfg->rx_ch, &data->rx);
}

#define OT_PIO_DEFINE(inst)                                                                        \
	static const struct ot_pio_config ot_pio_config_##inst = {                                 \
		.pio = DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, pulse_ios, 0)),                  \
		.tx_ch = DT_INST_PHA_BY_IDX(inst, pulse_ios, 0, channel),                          \
		.rx_ch = DT_INST_PHA_BY_IDX(inst, pulse_ios, 1, channel),                          \
		.tx_idle_high = DT_INST_PROP(inst, tx_idle_high),                                  \
		.rx_idle_high = DT_INST_PROP(inst, rx_idle_high),                                  \
	};                                                                                         \
	BUILD_ASSERT(DT_SAME_NODE(DT_INST_PHANDLE_BY_IDX(inst, pulse_ios, 0),                      \
				  DT_INST_PHANDLE_BY_IDX(inst, pulse_ios, 1)),                     \
		     "TX and RX must be on the same pulse_io controller");                         \
	static struct ot_pio_data ot_pio_data_##inst;                                              \
	DEVICE_DT_INST_DEFINE(inst, ot_pio_init, NULL, &ot_pio_data_##inst, &ot_pio_config_##inst, \
			      POST_KERNEL, CONFIG_OPENTHERM_INIT_PRIORITY, &ot_pio_api);

DT_INST_FOREACH_STATUS_OKAY(OT_PIO_DEFINE)
