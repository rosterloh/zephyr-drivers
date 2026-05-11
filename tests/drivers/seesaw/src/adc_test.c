#ifdef CONFIG_ADC_ADAFRUIT_SEESAW

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/ztest.h>

#include "seesaw_emul.h"

#define ADC_NODE DT_NODELABEL(adc_child)

static const struct device *const adc_dev = DEVICE_DT_GET(ADC_NODE);

ZTEST(adc_seesaw, test_ready)
{
	zassert_true(device_is_ready(adc_dev));
}

ZTEST(adc_seesaw, test_channel_setup_accepts_in_range)
{
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,
	};

	zassert_ok(adc_channel_setup(adc_dev, &cfg));
	cfg.channel_id = 18;
	zassert_ok(adc_channel_setup(adc_dev, &cfg));
	/* channel_id is a 5-bit field (0..31); verify the maximum fits */
	cfg.channel_id = 31;
	zassert_ok(adc_channel_setup(adc_dev, &cfg));
}

ZTEST(adc_seesaw, test_channel_setup_rejects_differential)
{
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,
		.differential = 1,
	};

	zassert_equal(adc_channel_setup(adc_dev, &cfg), -ENOTSUP);
}

ZTEST(adc_seesaw, test_read_returns_seeded_value)
{
	const struct emul *target = EMUL_DT_GET(DT_PARENT(ADC_NODE));
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 3,
	};

	zassert_not_null(target);
	zassert_ok(adc_channel_setup(adc_dev, &cfg));

	/* SAMD09: logical channel 3 -> pin 5 -> reg = ADC_BASE(0x09) | (0x07+5) = 0x090C. */
	zassert_ok(mfd_seesaw_mock_set_register(target->data, (0x09 << 8) | (0x07 + 5), 0x02AB));

	uint16_t sample = 0;
	const struct adc_sequence seq = {
		.channels = BIT(3),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = 10,
	};

	zassert_ok(adc_read(adc_dev, &seq));
	zassert_equal(sample, 0x02AB, "expected 0x02AB, got 0x%04x", sample);
}

ZTEST(adc_seesaw, test_high_channel_id_not_truncated)
{
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 18,
	};

	zassert_ok(adc_channel_setup(adc_dev, &cfg));

	uint16_t sample = 0;
	const struct adc_sequence seq = {
		.channels = BIT(18),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = 10,
	};
	/* On the SAMD09 emul, channel 18 has no pin mapping, so the underlying
	 * read returns -EINVAL. With the uint32_t fix, this error propagates
	 * back to the caller. With the old uint8_t bug, BIT(18) was truncated
	 * to 0 and adc_read returned 0 with sample == 0.
	 */
	int ret = adc_read(adc_dev, &seq);

	zassert_equal(ret, -EINVAL,
		      "expected -EINVAL from chan 18 on SAMD09, got %d (sample=0x%04x)", ret,
		      sample);
}

ZTEST_SUITE(adc_seesaw, NULL, NULL, NULL, NULL, NULL);

#endif /* CONFIG_ADC_ADAFRUIT_SEESAW */
