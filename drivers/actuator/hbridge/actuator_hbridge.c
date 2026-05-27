/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rosterloh_actuator_hbridge

#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
#include <zephyr/drivers/sensor.h>
#endif
#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
#include <zephyr/drivers/adc.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>
#include <zephyr/actuator/actuator.h>
#include "../actuator_internal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

LOG_MODULE_REGISTER(actuator_hbridge, CONFIG_ACTUATOR_LOG_LEVEL);

#define HB_CB_POOL CONFIG_ACTUATOR_MAX_CALLBACKS_PER_DEVICE

struct hbridge_data {
	struct actuator_common_data common;
	struct actuator_cb_storage cb_storage;
	struct actuator_cb_node cb_pool[HB_CB_POOL];
	const struct device *self;
	struct k_work_delayable feedback_work;
	float last_position_rad;
	uint64_t last_timestamp_us;
	bool position_valid;
};

struct hbridge_config {
	struct actuator_cb_offsets cb_offsets; /* must be first */
	struct pwm_dt_spec pwm;
	struct gpio_dt_spec in1;
	struct gpio_dt_spec in2; /* port == NULL when absent */
	bool has_in2;
	struct gpio_dt_spec stby; /* port == NULL when absent */
	bool has_stby;
#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
	const struct device *encoder; /* may be NULL */
#endif
#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
	struct adc_dt_spec adc;
	uint32_t current_sense_mohms;
	int32_t max_current_ma;
	int32_t torque_const_mnm_per_a;
	bool has_current_sense;
#endif
	uint32_t pwm_period_ns;
	uint32_t update_period_ms;
	enum actuator_mode default_mode;
	uint32_t caps;
};

static int hbridge_set_pwm(const struct hbridge_config *cfg, float duty)
{
	if (duty > 1.0f) {
		duty = 1.0f;
	}
	if (duty < -1.0f) {
		duty = -1.0f;
	}
	int fwd = (duty >= 0.0f) ? 1 : 0;
	uint32_t pulse_ns = (uint32_t)(fabsf(duty) * (float)cfg->pwm_period_ns);
	int err = gpio_pin_set_dt(&cfg->in1, fwd);

	if (err) {
		return err;
	}
	if (cfg->has_in2) {
		err = gpio_pin_set_dt(&cfg->in2, !fwd);
		if (err) {
			return err;
		}
	}
	return pwm_set_dt(&cfg->pwm, cfg->pwm_period_ns, pulse_ns);
}

static int hb_read_feedback(const struct device *dev, struct actuator_feedback *out)
{
	const struct hbridge_config *cfg = dev->config;

	ARG_UNUSED(cfg);

	*out = (struct actuator_feedback){
		.valid_mask = 0,
		.timestamp_us = (uint64_t)k_uptime_get() * 1000,
	};

#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
	if (cfg->encoder != NULL) {
		struct hbridge_data *d = dev->data;
		struct sensor_value rot;
		int err = sensor_sample_fetch(cfg->encoder);

		if (err == 0) {
			err = sensor_channel_get(cfg->encoder, SENSOR_CHAN_ROTATION, &rot);
		}
		if (err == 0) {
			float deg = (float)rot.val1 + (float)rot.val2 * 1e-6f;
			float rad = deg * (float)M_PI / 180.0f;

			out->position = rad;
			out->valid_mask |= ACTUATOR_FB_POSITION;

			if (d->position_valid) {
				uint64_t dt_us = out->timestamp_us - d->last_timestamp_us;

				if (dt_us > 0) {
					out->velocity = (rad - d->last_position_rad) /
							((float)dt_us * 1e-6f);
					out->valid_mask |= ACTUATOR_FB_VELOCITY;
				}
			}
			d->last_position_rad = rad;
			d->last_timestamp_us = out->timestamp_us;
			d->position_valid = true;
		}
	}
#endif

#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
	if (cfg->has_current_sense) {
		int16_t raw = 0;
		struct adc_sequence seq = {
			.buffer = &raw,
			.buffer_size = sizeof(raw),
		};
		int err = adc_sequence_init_dt(&cfg->adc, &seq);

		if (err == 0) {
			err = adc_read(cfg->adc.dev, &seq);
		}
		int32_t mv = raw;

		if (err == 0) {
			err = adc_raw_to_millivolts_dt(&cfg->adc, &mv);
		}
		if (err == 0) {
			int32_t i_ma = (mv * 1000) / (int32_t)cfg->current_sense_mohms;

			if (cfg->torque_const_mnm_per_a > 0) {
				float amps = (float)i_ma / 1000.0f;

				out->effort = amps * (float)cfg->torque_const_mnm_per_a / 1000.0f;
				out->valid_mask |= ACTUATOR_FB_EFFORT;
			}
			if (cfg->max_current_ma > 0 && abs(i_ma) > cfg->max_current_ma) {
				out->fault_flags |= ACTUATOR_FAULT_OVERCURRENT;
			}
		}
	}
#endif
	return 0;
}

static void hb_feedback_work(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct hbridge_data *d = CONTAINER_OF(dw, struct hbridge_data, feedback_work);
	const struct device *dev = d->self;
	const struct hbridge_config *cfg = dev->config;

	struct actuator_feedback fb;

	if (hb_read_feedback(dev, &fb) == 0 && fb.valid_mask != 0) {
		actuator_report_feedback(dev, &fb);
	}
	if (d->common.state != ACTUATOR_STATE_DISABLED) {
		k_work_schedule(&d->feedback_work, K_MSEC(cfg->update_period_ms));
	}
}

static int hb_enable(const struct device *dev)
{
	struct hbridge_data *d = dev->data;
	const struct hbridge_config *cfg = dev->config;

	bool need_worker = false;

#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
	if (cfg->encoder != NULL) {
		need_worker = true;
	}
#endif
#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
	if (cfg->has_current_sense) {
		need_worker = true;
	}
#endif
	if (cfg->has_stby) {
		int err = gpio_pin_set_dt(&cfg->stby, 1);

		if (err) {
			return err;
		}
	}
	if (need_worker) {
		k_work_schedule(&d->feedback_work, K_MSEC(cfg->update_period_ms));
	}
	return 0;
}

static int hb_disable(const struct device *dev)
{
	struct hbridge_data *d = dev->data;
	const struct hbridge_config *cfg = dev->config;

	(void)pwm_set_dt(&cfg->pwm, cfg->pwm_period_ns, 0);
	k_work_cancel_delayable(&d->feedback_work);
	d->position_valid = false;
	if (cfg->has_stby) {
		(void)gpio_pin_set_dt(&cfg->stby, 0);
	}
	return 0;
}

static int hb_set_setpoint(const struct device *dev, enum actuator_mode mode, float value)
{
	const struct hbridge_config *cfg = dev->config;

	if (mode != ACTUATOR_MODE_VELOCITY && mode != ACTUATOR_MODE_EFFORT) {
		return -ENOTSUP;
	}
	return hbridge_set_pwm(cfg, value);
}

static int hb_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	const struct hbridge_config *cfg = dev->config;

	if (!cfg->has_in2) {
		/* Single-GPIO (PWM+DIR) variant: cannot independently command
		 * brake or coast; the silicon decides what PWM=0 means. The
		 * subsystem should already have rejected this via the cap, but
		 * guard anyway. */
		return -ENOTSUP;
	}

	int err = pwm_set_dt(&cfg->pwm, cfg->pwm_period_ns, 0);

	if (err) {
		return err;
	}

	int in1_val = 0, in2_val = 0;

	switch (mode) {
	case ACTUATOR_DRIVE_MODE_NORMAL:
	case ACTUATOR_DRIVE_MODE_COAST:
		/* IN1=0, IN2=0 → outputs high-Z. Next set_setpoint reasserts. */
		in1_val = 0;
		in2_val = 0;
		break;
	case ACTUATOR_DRIVE_MODE_BRAKE:
		/* IN1=1, IN2=1 → both low-side FETs on, motor windings shorted. */
		in1_val = 1;
		in2_val = 1;
		break;
	default:
		return -EINVAL;
	}
	err = gpio_pin_set_dt(&cfg->in1, in1_val);
	if (err) {
		return err;
	}
	return gpio_pin_set_dt(&cfg->in2, in2_val);
}

static const struct actuator_driver_api hb_api = {
	.enable = hb_enable,
	.disable = hb_disable,
	.set_setpoint = hb_set_setpoint,
	.read_feedback = hb_read_feedback,
	.set_drive_mode = hb_set_drive_mode,
};

static int hb_init(const struct device *dev)
{
	struct hbridge_data *d = dev->data;
	const struct hbridge_config *cfg = dev->config;

	if (!device_is_ready(cfg->pwm.dev) || !device_is_ready(cfg->in1.port)) {
		return -ENODEV;
	}
#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
	if (cfg->encoder != NULL && !device_is_ready(cfg->encoder)) {
		LOG_ERR("encoder for %s not ready", dev->name);
		return -ENODEV;
	}
#endif
	int err = gpio_pin_configure_dt(&cfg->in1, GPIO_OUTPUT_INACTIVE);

	if (err) {
		return err;
	}
	if (cfg->has_in2) {
		if (!device_is_ready(cfg->in2.port)) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&cfg->in2, GPIO_OUTPUT_INACTIVE);
		if (err) {
			return err;
		}
	}
	if (cfg->has_stby) {
		if (!device_is_ready(cfg->stby.port)) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&cfg->stby, GPIO_OUTPUT_INACTIVE);
		if (err) {
			return err;
		}
	}
#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
	if (cfg->has_current_sense) {
		if (!adc_is_ready_dt(&cfg->adc)) {
			LOG_ERR("ADC for %s not ready", dev->name);
			return -ENODEV;
		}
		err = adc_channel_setup_dt(&cfg->adc);
		if (err) {
			return err;
		}
	}
#endif

	d->self = dev;
	d->common.state = ACTUATOR_STATE_DISABLED;
	d->common.caps = cfg->caps;
	d->cb_storage.pool = d->cb_pool;
	d->cb_storage.pool_n = HB_CB_POOL;
	atomic_set(&d->cb_storage.used, 0);
	sys_slist_init(&d->cb_storage.list);
	k_work_init_delayable(&d->feedback_work, hb_feedback_work);
	return 0;
}

#define HB_HAS_IN2(inst)           DT_INST_NODE_HAS_PROP(inst, in2_gpios)
#define HB_HAS_STBY(inst)          DT_INST_NODE_HAS_PROP(inst, stby_gpios)
#define HB_HAS_ENCODER(inst)       DT_INST_NODE_HAS_PROP(inst, encoder)
#define HB_HAS_CURRENT_SENSE(inst) DT_INST_NODE_HAS_PROP(inst, io_channels)

#define HB_CAPS(inst)                                                                              \
	(ACTUATOR_CAP_VELOCITY |                                                                   \
	 ((HB_HAS_CURRENT_SENSE(inst) && DT_INST_PROP_OR(inst, torque_constant_mnm_per_a, 0) > 0)  \
		  ? ACTUATOR_CAP_EFFORT                                                            \
		  : 0) |                                                                           \
	 (HB_HAS_ENCODER(inst) ? ACTUATOR_CAP_POSITION : 0) |                                      \
	 (HB_HAS_IN2(inst) ? ACTUATOR_CAP_DRIVE_MODE : 0))

#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
#define HB_ENCODER_INIT(inst)                                                                      \
	.encoder = COND_CODE_1(HB_HAS_ENCODER(inst),                                               \
			       (DEVICE_DT_GET(DT_INST_PHANDLE(inst, encoder))), (NULL)),
#else
#define HB_ENCODER_INIT(inst)
#endif

#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
#define HB_CURRENT_SENSE_INIT(inst)                                                                \
	.adc = COND_CODE_1(HB_HAS_CURRENT_SENSE(inst), (ADC_DT_SPEC_INST_GET(inst)), ({0})),       \
	.current_sense_mohms = DT_INST_PROP_OR(inst, current_sense_mohms, 1),                      \
	.max_current_ma = DT_INST_PROP_OR(inst, max_current_ma, 0),                                \
	.torque_const_mnm_per_a = DT_INST_PROP_OR(inst, torque_constant_mnm_per_a, 0),             \
	.has_current_sense = HB_HAS_CURRENT_SENSE(inst),
#else
#define HB_CURRENT_SENSE_INIT(inst)
#endif

#define HB_DEFINE(inst)                                                                            \
	static struct hbridge_data hb_data_##inst;                                                 \
	static const struct hbridge_config hb_config_##inst = {                                    \
		.cb_offsets =                                                                      \
			{                                                                          \
				.storage_offset = offsetof(struct hbridge_data, cb_storage),       \
			},                                                                         \
		.pwm = PWM_DT_SPEC_INST_GET(inst),                                                 \
		.in1 = GPIO_DT_SPEC_INST_GET(inst, in1_gpios),                                     \
		.in2 = COND_CODE_1(HB_HAS_IN2(inst), (GPIO_DT_SPEC_INST_GET(inst, in2_gpios)),     \
				   ({.port = NULL})),                                              \
		.has_in2 = HB_HAS_IN2(inst),                                                       \
		.stby = COND_CODE_1(HB_HAS_STBY(inst), (GPIO_DT_SPEC_INST_GET(inst, stby_gpios)),  \
				    ({.port = NULL})),                                             \
		.has_stby = HB_HAS_STBY(inst),                                                     \
		HB_ENCODER_INIT(inst) HB_CURRENT_SENSE_INIT(inst).pwm_period_ns =                  \
			DT_INST_PROP(inst, pwm_period_ns),                                         \
		.update_period_ms = DT_INST_PROP(inst, update_period_ms),                          \
		.default_mode = ACTUATOR_MODE_VELOCITY,                                            \
		.caps = HB_CAPS(inst),                                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, hb_init, NULL, &hb_data_##inst, &hb_config_##inst,             \
			      POST_KERNEL, CONFIG_ACTUATOR_INIT_PRIORITY, &hb_api);

DT_INST_FOREACH_STATUS_OKAY(HB_DEFINE)
