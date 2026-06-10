/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Multi-unit encoder driver for the ESP32 PCNT block. Each enabled unit
 * child node of the rosterloh,esp32-pcnt controller is its own sensor
 * device. The 16-bit hardware counter is extended to 64 bits by
 * accumulating high/low-limit events in an ISR (the hardware clears the
 * counter to zero whenever a limit is hit).
 */

#define DT_DRV_COMPAT rosterloh_esp32_pcnt

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <hal/pcnt_ll.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(esp32_pcnt_encoder, CONFIG_ESP32_PCNT_ENCODER_LOG_LEVEL);

#define PCNT_ENC_LIMIT 30000

/* Raw unit status bits (pcnt_ll_get_unit_status) */
#define PCNT_ENC_STATUS_LOW_LIMIT  BIT(4)
#define PCNT_ENC_STATUS_HIGH_LIMIT BIT(5)

#define PCNT_ENC_MAX_UNITS 8

struct pcnt_enc_unit_data {
	int64_t acc;   /* limit-event accumulator, ISR-updated */
	int64_t total; /* snapshot taken by sample_fetch */
};

struct pcnt_enc_unit_config {
	uint8_t idx;
	uint16_t filter;
	uint32_t counts_per_rev;
	uint8_t sig_pos_mode;
	uint8_t sig_neg_mode;
	uint8_t ctrl_h_mode;
	uint8_t ctrl_l_mode;
};

/* Controller-level state shared by all unit devices (single PCNT block). */
static struct {
	pcnt_dev_t *dev;
	bool initialized;
	struct pcnt_enc_unit_data *units[PCNT_ENC_MAX_UNITS];
} pcnt_enc_shared = {
	.dev = (pcnt_dev_t *)DT_INST_REG_ADDR(0),
};

PINCTRL_DT_INST_DEFINE(0);
static const struct pinctrl_dev_config *pcnt_enc_pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0);

static void IRAM_ATTR pcnt_enc_isr(void *arg)
{
	ARG_UNUSED(arg);
	pcnt_dev_t *hw = pcnt_enc_shared.dev;
	uint32_t intr_status = pcnt_ll_get_intr_status(hw);

	pcnt_ll_clear_intr_status(hw, intr_status);

	for (int i = 0; i < PCNT_ENC_MAX_UNITS; i++) {
		if (!(intr_status & BIT(i)) || pcnt_enc_shared.units[i] == NULL) {
			continue;
		}
		uint32_t st = pcnt_ll_get_unit_status(hw, i);

		if (st & PCNT_ENC_STATUS_HIGH_LIMIT) {
			pcnt_enc_shared.units[i]->acc += PCNT_ENC_LIMIT;
		} else if (st & PCNT_ENC_STATUS_LOW_LIMIT) {
			pcnt_enc_shared.units[i]->acc -= PCNT_ENC_LIMIT;
		}
	}
}

static int pcnt_enc_global_init(void)
{
	int err;
	const struct device *clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));

	err = clock_control_on(clock_dev, (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, offset));
	if (err < 0) {
		LOG_ERR("clock enable failed (%d)", err);
		return err;
	}

	err = pinctrl_apply_state(pcnt_enc_pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("pinctrl apply failed (%d)", err);
		return err;
	}

	err = esp_intr_alloc(DT_INST_IRQ_BY_IDX(0, 0, irq),
			     ESP_PRIO_TO_FLAGS(DT_INST_IRQ_BY_IDX(0, 0, priority)) |
				     ESP_INT_FLAGS_CHECK(DT_INST_IRQ_BY_IDX(0, 0, flags)) |
				     ESP_INTR_FLAG_IRAM,
			     pcnt_enc_isr, NULL, NULL);
	if (err != 0) {
		LOG_ERR("isr alloc failed (%d)", err);
		return err;
	}

	pcnt_enc_shared.initialized = true;
	return 0;
}

static int pcnt_enc_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct pcnt_enc_unit_config *cfg = dev->config;
	struct pcnt_enc_unit_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	unsigned int key = irq_lock();

	data->total = data->acc + pcnt_ll_get_count(pcnt_enc_shared.dev, cfg->idx);
	irq_unlock(key);

	return 0;
}

static int pcnt_enc_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	const struct pcnt_enc_unit_config *cfg = dev->config;
	struct pcnt_enc_unit_data *data = dev->data;

	if (chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	/* counts -> micro-degrees, then split into sensor_value */
	int64_t udeg = data->total * 360LL * 1000000LL / cfg->counts_per_rev;

	val->val1 = (int32_t)(udeg / 1000000LL);
	val->val2 = (int32_t)(udeg % 1000000LL);
	return 0;
}

static int pcnt_enc_unit_init(const struct device *dev)
{
	const struct pcnt_enc_unit_config *cfg = dev->config;
	struct pcnt_enc_unit_data *data = dev->data;
	pcnt_dev_t *hw;
	int err;

	if (!pcnt_enc_shared.initialized) {
		err = pcnt_enc_global_init();
		if (err < 0) {
			return err;
		}
	}
	hw = pcnt_enc_shared.dev;

	pcnt_enc_shared.units[cfg->idx] = data;

	pcnt_ll_stop_count(hw, cfg->idx);
	pcnt_ll_disable_all_events(hw, cfg->idx);

	/* Channel 0 carries the encoder; channel 1 must not count. */
	pcnt_ll_set_edge_action(hw, cfg->idx, 0, cfg->sig_pos_mode, cfg->sig_neg_mode);
	pcnt_ll_set_level_action(hw, cfg->idx, 0, cfg->ctrl_h_mode, cfg->ctrl_l_mode);
	pcnt_ll_set_edge_action(hw, cfg->idx, 1, PCNT_CHANNEL_EDGE_ACTION_HOLD,
				PCNT_CHANNEL_EDGE_ACTION_HOLD);
	pcnt_ll_set_level_action(hw, cfg->idx, 1, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
				 PCNT_CHANNEL_LEVEL_ACTION_KEEP);

	pcnt_ll_set_high_limit_value(hw, cfg->idx, PCNT_ENC_LIMIT);
	pcnt_ll_set_low_limit_value(hw, cfg->idx, -PCNT_ENC_LIMIT);
	pcnt_ll_enable_high_limit_event(hw, cfg->idx, true);
	pcnt_ll_enable_low_limit_event(hw, cfg->idx, true);

	pcnt_ll_set_glitch_filter_thres(hw, cfg->idx, cfg->filter);
	pcnt_ll_enable_glitch_filter(hw, cfg->idx, cfg->filter != 0);

	pcnt_ll_clear_count(hw, cfg->idx);
	pcnt_ll_enable_intr(hw, BIT(cfg->idx), true);
	pcnt_ll_start_count(hw, cfg->idx);

	return 0;
}

static DEVICE_API(sensor, pcnt_enc_api) = {
	.sample_fetch = pcnt_enc_sample_fetch,
	.channel_get = pcnt_enc_channel_get,
};

#define PCNT_ENC_UNIT_DEFINE(node_id)                                                              \
	static struct pcnt_enc_unit_data pcnt_enc_data_##node_id;                                  \
	static const struct pcnt_enc_unit_config pcnt_enc_cfg_##node_id = {                        \
		.idx = DT_REG_ADDR(node_id),                                                       \
		.filter = MIN(DT_PROP(node_id, filter), PCNT_LL_MAX_GLITCH_WIDTH),                 \
		.counts_per_rev = DT_PROP(node_id, counts_per_revolution),                         \
		.sig_pos_mode = DT_PROP(node_id, sig_pos_mode),                                    \
		.sig_neg_mode = DT_PROP(node_id, sig_neg_mode),                                    \
		.ctrl_h_mode = DT_PROP(node_id, ctrl_h_mode),                                      \
		.ctrl_l_mode = DT_PROP(node_id, ctrl_l_mode),                                      \
	};                                                                                         \
	SENSOR_DEVICE_DT_DEFINE(node_id, pcnt_enc_unit_init, NULL, &pcnt_enc_data_##node_id,       \
				&pcnt_enc_cfg_##node_id, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				&pcnt_enc_api);

DT_INST_FOREACH_CHILD_STATUS_OKAY(0, PCNT_ENC_UNIT_DEFINE)
