#ifdef CONFIG_PWM_ADAFRUIT_SEESAW

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/ztest.h>

#include "seesaw_emul.h"

#define PWM_NODE DT_NODELABEL(pwm_child)

static const struct device *const pwm = DEVICE_DT_GET(PWM_NODE);

ZTEST(pwm_seesaw, test_ready)
{
	zassert_true(device_is_ready(pwm));
}

ZTEST(pwm_seesaw, test_cycles_per_sec)
{
	uint64_t cps = 0;

	zassert_ok(pwm_get_cycles_per_sec(pwm, 0, &cps));
	zassert_equal(cps, 1000000ULL);
}

ZTEST(pwm_seesaw, test_set_cycles_writes_freq_and_duty)
{
	const struct emul *target = EMUL_DT_GET(DT_PARENT(PWM_NODE));
	uint32_t reg_val = 0;

	zassert_not_null(target);

	/* 1 ms period, 500 us pulse = 50% duty at 1 kHz. */
	zassert_ok(pwm_set_cycles(pwm, 4, 1000, 500, 0));

	/* The driver writes a 3-byte payload: [channel, freq_be16].
	 * The emul stores the 2-byte value portion (bytes[3..4] after the
	 * module+function prefix). Verify TIMER_FREQ was written correctly.
	 */
	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x08 << 8) | 0x02 /* TIMER_BASE, TIMER_FREQ */, &reg_val));
	zassert_equal(reg_val, 1000U, "expected freq=1000 Hz, got %u", reg_val);

	zassert_ok(mfd_seesaw_mock_get_register(
		target->data, (0x08 << 8) | 0x01 /* TIMER_BASE, TIMER_PWM */, &reg_val));
	/* 500/1000 * 0xFFFF = 0x7FFF (rounded down). */
	zassert_equal(reg_val, 0x7FFFU, "expected duty=0x7FFF, got 0x%x", reg_val);
}

ZTEST(pwm_seesaw, test_zero_period_rejected)
{
	zassert_equal(pwm_set_cycles(pwm, 4, 0, 0, 0), -EINVAL);
}

ZTEST(pwm_seesaw, test_short_period_rejected)
{
	/* period=1 µs → hz=1_000_000 > UINT16_MAX → -EINVAL */
	zassert_equal(pwm_set_cycles(pwm, 4, 1, 0, 0), -EINVAL);
}

ZTEST(pwm_seesaw, test_inverted_polarity_rejected)
{
	zassert_equal(pwm_set_cycles(pwm, 4, 1000, 500, PWM_POLARITY_INVERTED), -ENOTSUP);
}

ZTEST_SUITE(pwm_seesaw, NULL, NULL, NULL, NULL, NULL);

#endif /* CONFIG_PWM_ADAFRUIT_SEESAW */
