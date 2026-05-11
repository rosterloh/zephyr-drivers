#define DT_DRV_COMPAT adafruit_seesaw_pwm

#include <zephyr/device.h>
#include <zephyr/drivers/mfd/seesaw.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(pwm_seesaw, CONFIG_PWM_ADAFRUIT_SEESAW_LOG_LEVEL);

/* Local mirror of MFD register IDs. */
#define SEESAW_TIMER_BASE 0x08U
#define SEESAW_TIMER_PWM  0x01U
#define SEESAW_TIMER_FREQ 0x02U

/* The seesaw firmware accepts a frequency value in Hz (16-bit) per pin, and a
 * separate 16-bit duty value. Expose a virtual 1 µs tick to Zephyr so that
 * pwm_set_cycles(channel, period, pulse) takes period and pulse in
 * microseconds.
 *
 * Effective period range: 16 µs (≈62.5 kHz) to 1 s (1 Hz). Shorter periods
 * exceed the firmware's 16-bit Hz register and are rejected with -EINVAL.
 */
#define SEESAW_PWM_CYCLES_PER_SEC 1000000U

struct pwm_seesaw_config {
	const struct device *mfd;
};

static int pwm_seesaw_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_seesaw_config *cfg = dev->config;
	uint8_t buf[3];
	uint64_t hz;
	uint64_t duty;
	int ret;

	if ((flags & PWM_POLARITY_INVERTED) != 0U) {
		return -ENOTSUP;
	}

	if (channel > 0xFFU) {
		return -EINVAL;
	}
	if (period_cycles == 0U) {
		return -EINVAL;
	}

	/* period (µs) → frequency (Hz). */
	hz = ((uint64_t)SEESAW_PWM_CYCLES_PER_SEC) / period_cycles;
	if (hz == 0U || hz > UINT16_MAX) {
		return -EINVAL;
	}

	/* Clamp to full duty if pulse exceeds period (PWM spec leaves this undefined). */
	duty = ((uint64_t)pulse_cycles * 0xFFFFU) / period_cycles;
	if (duty > 0xFFFFU) {
		duty = 0xFFFFU;
	}

	buf[0] = (uint8_t)channel;
	sys_put_be16((uint16_t)hz, &buf[1]);
	ret = mfd_seesaw_write(cfg->mfd, SEESAW_TIMER_BASE, SEESAW_TIMER_FREQ, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	buf[0] = (uint8_t)channel;
	sys_put_be16((uint16_t)duty, &buf[1]);
	return mfd_seesaw_write(cfg->mfd, SEESAW_TIMER_BASE, SEESAW_TIMER_PWM, buf, sizeof(buf));
}

static int pwm_seesaw_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					 uint64_t *cycles)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	*cycles = SEESAW_PWM_CYCLES_PER_SEC;
	return 0;
}

static DEVICE_API(pwm, pwm_seesaw_api) = {
	.set_cycles = pwm_seesaw_set_cycles,
	.get_cycles_per_sec = pwm_seesaw_get_cycles_per_sec,
};

static int pwm_seesaw_init(const struct device *dev)
{
	const struct pwm_seesaw_config *cfg = dev->config;

	if (!device_is_ready(cfg->mfd)) {
		return -ENODEV;
	}
	return 0;
}

#define PWM_SEESAW_DEFINE(inst)                                                                    \
	static const struct pwm_seesaw_config pwm_seesaw_cfg_##inst = {                            \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pwm_seesaw_init, NULL, NULL, &pwm_seesaw_cfg_##inst,           \
			      POST_KERNEL, CONFIG_PWM_ADAFRUIT_SEESAW_INIT_PRIORITY,               \
			      &pwm_seesaw_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SEESAW_DEFINE)
