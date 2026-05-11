#ifdef CONFIG_LED_STRIP_ADAFRUIT_SEESAW

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/ztest.h>

#include "seesaw_emul.h"

#define LED_NODE DT_NODELABEL(led_child)

static const struct device *const led = DEVICE_DT_GET(LED_NODE);

ZTEST(led_strip_seesaw, test_ready)
{
	zassert_true(device_is_ready(led));
}

ZTEST(led_strip_seesaw, test_length)
{
	zassert_equal(led_strip_length(led), 4);
}

ZTEST(led_strip_seesaw, test_init_programmed_speed_pin_and_length)
{
	const struct emul *target = EMUL_DT_GET(DT_PARENT(LED_NODE));
	uint32_t reg_val = 0;

	zassert_not_null(target);

	/* SPEED: 1 for 800 kHz. */
	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x0E << 8) | 0x02 /* NEOPIXEL_BASE, SPEED */, &reg_val));
	zassert_equal(reg_val, 1U, "expected speed=1 (800kHz), got %u", reg_val);

	/* PIN: from overlay, pin 14. */
	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x0E << 8) | 0x01 /* NEOPIXEL_BASE, PIN */, &reg_val));
	zassert_equal(reg_val, 14U, "expected pin=14, got %u", reg_val);

	/* BUF_LENGTH: chain_length * num_colors = 4 * 3 = 12. */
	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x0E << 8) | 0x03 /* NEOPIXEL_BASE, BUF_LENGTH */, &reg_val));
	zassert_equal(reg_val, 12U, "expected buf_length=12, got %u", reg_val);
}

ZTEST(led_strip_seesaw, test_update_rgb_short_chain)
{
	struct led_rgb px[4] = {
		{.r = 0xFFU},
		{.g = 0xFFU},
		{.b = 0xFFU},
		{.r = 0x10U, .g = 0x20U, .b = 0x30U},
	};

	zassert_ok(led_strip_update_rgb(led, px, ARRAY_SIZE(px)));
}

ZTEST_SUITE(led_strip_seesaw, NULL, NULL, NULL, NULL, NULL);

#endif /* CONFIG_LED_STRIP_ADAFRUIT_SEESAW */
