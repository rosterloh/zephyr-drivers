#ifdef CONFIG_ADAFRUIT_SEESAW_ENCODER

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include "seesaw_emul.h"

#define ENC_NODE DT_NODELABEL(encoder_child)

static const struct device *const enc = DEVICE_DT_GET(ENC_NODE);

ZTEST(seesaw_encoder, test_ready)
{
	zassert_true(device_is_ready(enc));
}

ZTEST(seesaw_encoder, test_fetch_and_get_seeded_position)
{
	const struct emul *target = EMUL_DT_GET(DT_PARENT(ENC_NODE));
	struct sensor_value v;

	zassert_not_null(target);

	/* Seed the encoder position register with a signed value (e.g. -42). */
	zassert_ok(mfd_seesaw_mock_set_register(
		target->data, (0x11 << 8) | 0x04 /* ENCODER_BASE, POSITION */, (uint32_t)-42));

	zassert_ok(sensor_sample_fetch(enc));
	zassert_ok(sensor_channel_get(enc, SENSOR_CHAN_ROTATION, &v));
	zassert_equal(v.val1, -42, "expected position -42, got %d", v.val1);
	zassert_equal(v.val2, 0);
}

ZTEST(seesaw_encoder, test_fetch_and_get_positive_position)
{
	const struct emul *target = EMUL_DT_GET(DT_PARENT(ENC_NODE));
	struct sensor_value v;

	zassert_not_null(target);

	zassert_ok(mfd_seesaw_mock_set_register(
		target->data, (0x11 << 8) | 0x04 /* ENCODER_BASE, POSITION */, 1234U));

	zassert_ok(sensor_sample_fetch(enc));
	zassert_ok(sensor_channel_get(enc, SENSOR_CHAN_ROTATION, &v));
	zassert_equal(v.val1, 1234, "expected position 1234, got %d", v.val1);
	zassert_equal(v.val2, 0);
}

ZTEST(seesaw_encoder, test_sample_fetch_rejects_unsupported_chan)
{
	zassert_equal(sensor_sample_fetch_chan(enc, SENSOR_CHAN_AMBIENT_TEMP), -ENOTSUP);
}

ZTEST(seesaw_encoder, test_channel_get_rejects_other_channels)
{
	struct sensor_value v;
	zassert_ok(sensor_sample_fetch(enc));
	zassert_equal(sensor_channel_get(enc, SENSOR_CHAN_AMBIENT_TEMP, &v), -ENOTSUP);
}

ZTEST_SUITE(seesaw_encoder, NULL, NULL, NULL, NULL, NULL);

#endif /* CONFIG_ADAFRUIT_SEESAW_ENCODER */
