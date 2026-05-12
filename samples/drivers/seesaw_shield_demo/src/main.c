/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Adafruit Seesaw shield demo.
 *
 * Selects per-child demo code at compile time based on which seesaw
 * children the active shield enables. Build with --shield
 * adafruit_neoslider / adafruit_rotary_encoder / adafruit_neokey_1x4.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(seesaw_shield_demo, LOG_LEVEL_INF);

#define POLL_INTERVAL K_MSEC(200)

/* ---------- ADC (NeoSlider) ---------- */

#if DT_HAS_COMPAT_STATUS_OKAY(adafruit_seesaw_adc)

#define ADC_NODE     DT_COMPAT_GET_ANY_STATUS_OKAY(adafruit_seesaw_adc)
#define ADC_CHANNELS 4

static const struct device *const adc_dev = DEVICE_DT_GET(ADC_NODE);

static int adc_demo_init(void)
{
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
		struct adc_channel_cfg cfg = {
			.gain = ADC_GAIN_1,
			.reference = ADC_REF_INTERNAL,
			.acquisition_time = ADC_ACQ_TIME_DEFAULT,
			.channel_id = ch,
			.differential = 0,
		};

		int err = adc_channel_setup(adc_dev, &cfg);

		if (err) {
			LOG_ERR("adc_channel_setup(%u) failed: %d", ch, err);
			return err;
		}
	}

	LOG_INF("ADC demo ready (%d channels)", ADC_CHANNELS);
	return 0;
}

static void adc_demo_tick(void)
{
	int16_t samples[ADC_CHANNELS];
	struct adc_sequence seq = {
		.channels = BIT_MASK(ADC_CHANNELS),
		.buffer = samples,
		.buffer_size = sizeof(samples),
		.resolution = 10,
	};

	int err = adc_read(adc_dev, &seq);

	if (err) {
		LOG_ERR("adc_read failed: %d", err);
		return;
	}

	LOG_INF("ADC: ch0=%d ch1=%d ch2=%d ch3=%d", samples[0], samples[1], samples[2], samples[3]);
}

#else
static int adc_demo_init(void)
{
	return 0;
}
static void adc_demo_tick(void)
{
}
#endif

/* ---------- Encoder ---------- */

#if DT_HAS_COMPAT_STATUS_OKAY(adafruit_seesaw_encoder)

#define ENCODER_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(adafruit_seesaw_encoder)

static const struct device *const encoder_dev = DEVICE_DT_GET(ENCODER_NODE);
static int32_t encoder_last;

static int encoder_demo_init(void)
{
	if (!device_is_ready(encoder_dev)) {
		LOG_ERR("Encoder device not ready");
		return -ENODEV;
	}
	encoder_last = 0;
	LOG_INF("Encoder demo ready");
	return 0;
}

static void encoder_demo_tick(void)
{
	struct sensor_value val;
	int err = sensor_sample_fetch(encoder_dev);

	if (err) {
		LOG_ERR("encoder fetch failed: %d", err);
		return;
	}

	err = sensor_channel_get(encoder_dev, SENSOR_CHAN_ROTATION, &val);
	if (err) {
		LOG_ERR("encoder get failed: %d", err);
		return;
	}

	if (val.val1 != encoder_last) {
		LOG_INF("Encoder: pos=%d (delta=%d)", val.val1, val.val1 - encoder_last);
		encoder_last = val.val1;
	}
}

#else
static int encoder_demo_init(void)
{
	return 0;
}
static void encoder_demo_tick(void)
{
}
#endif

/* ---------- GPIO keypad (NeoKey 1x4) ---------- */

#if DT_HAS_COMPAT_STATUS_OKAY(adafruit_seesaw_gpio)

#define GPIO_NODE   DT_COMPAT_GET_ANY_STATUS_OKAY(adafruit_seesaw_gpio)
#define KEYPAD_PINS 4
static const uint8_t keypad_pin_map[KEYPAD_PINS] = {4, 5, 6, 7};

static const struct device *const keypad_dev = DEVICE_DT_GET(GPIO_NODE);
static uint8_t keypad_last_state[KEYPAD_PINS];

static int keypad_demo_init(void)
{
	if (!device_is_ready(keypad_dev)) {
		LOG_ERR("Keypad GPIO not ready");
		return -ENODEV;
	}

	for (size_t i = 0; i < KEYPAD_PINS; i++) {
		int err = gpio_pin_configure(keypad_dev, keypad_pin_map[i],
					     GPIO_INPUT | GPIO_PULL_UP);

		if (err) {
			LOG_ERR("keypad configure pin %u: %d", keypad_pin_map[i], err);
			return err;
		}
		keypad_last_state[i] = 1; /* pull-up, idle high */
	}

	LOG_INF("Keypad demo ready (%d keys)", KEYPAD_PINS);
	return 0;
}

static void keypad_demo_tick(void)
{
	for (size_t i = 0; i < KEYPAD_PINS; i++) {
		int level = gpio_pin_get(keypad_dev, keypad_pin_map[i]);

		if (level < 0) {
			LOG_ERR("keypad get pin %u: %d", keypad_pin_map[i], level);
			continue;
		}

		if ((uint8_t)level != keypad_last_state[i]) {
			LOG_INF("Key %u %s", (unsigned int)i, level == 0 ? "DOWN" : "UP");
			keypad_last_state[i] = (uint8_t)level;
		}
	}
}

#else
static int keypad_demo_init(void)
{
	return 0;
}
static void keypad_demo_tick(void)
{
}
#endif

/* ---------- NeoPixel cycle ---------- */

#if DT_HAS_COMPAT_STATUS_OKAY(adafruit_seesaw_neopixel)

#define STRIP_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(adafruit_seesaw_neopixel)
#define STRIP_LEN  DT_PROP(STRIP_NODE, chain_length)

static const struct device *const strip_dev = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb strip_buf[STRIP_LEN];
static uint8_t strip_phase;

static const struct led_rgb strip_colors[3] = {
	{.r = 32, .g = 0, .b = 0},
	{.r = 0, .g = 32, .b = 0},
	{.r = 0, .g = 0, .b = 32},
};

static int strip_demo_init(void)
{
	if (!device_is_ready(strip_dev)) {
		LOG_ERR("LED strip not ready");
		return -ENODEV;
	}
	LOG_INF("NeoPixel demo ready (%d pixels)", STRIP_LEN);
	return 0;
}

static void strip_demo_tick(void)
{
	const struct led_rgb color = strip_colors[strip_phase];

	for (size_t i = 0; i < STRIP_LEN; i++) {
		strip_buf[i] = color;
	}

	int err = led_strip_update_rgb(strip_dev, strip_buf, STRIP_LEN);

	if (err) {
		LOG_ERR("led_strip_update_rgb failed: %d", err);
		return;
	}

	strip_phase = (strip_phase + 1) % ARRAY_SIZE(strip_colors);
}

#else
static int strip_demo_init(void)
{
	return 0;
}
static void strip_demo_tick(void)
{
}
#endif

int main(void)
{
	LOG_INF("seesaw_shield_demo starting");

	(void)adc_demo_init();
	(void)encoder_demo_init();
	(void)keypad_demo_init();
	(void)strip_demo_init();

	while (1) {
		adc_demo_tick();
		encoder_demo_tick();
		keypad_demo_tick();
		strip_demo_tick();
		k_sleep(POLL_INTERVAL);
	}

	return 0;
}
