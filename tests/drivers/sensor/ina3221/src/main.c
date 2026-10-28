/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/ztest.h>

#include "ina3221.h"
#include "ina3221_emul.h"

static const struct device *const ina = DEVICE_DT_GET(DT_NODELABEL(ina3221));

/* Raw readings as the driver sees them after its >> 3, in register LSBs. */
#define CH1_BUS   625  /* 5.000 V */
#define CH1_SHUNT 800  /* 32 mV over 10 mOhm = 3.2 A, 16 W */
#define CH2_BUS   413  /* 3.304 V */
#define CH2_SHUNT -400 /* -16 mV over 20 mOhm = -0.8 A, -2.6432 W */

static void set_rail(int ch, int16_t bus, int16_t shunt)
{
	ina3221_emul_set_reg(INA3221_BUS_V1 + 2 * ch, (uint16_t)(bus * 8));
	ina3221_emul_set_reg(INA3221_SHUNT_V1 + 2 * ch, (uint16_t)(shunt * 8));
}

static void ina3221_before(void *fixture)
{
	ARG_UNUSED(fixture);

	set_rail(0, CH1_BUS, CH1_SHUNT);
	set_rail(1, CH2_BUS, CH2_SHUNT);
	/* Channel 3 is disabled in devicetree; nothing should ever read this. */
	set_rail(2, 1000, 1000);
}

static int64_t sensor_value_to_micro_int(const struct sensor_value *val)
{
	return (int64_t)val->val1 * 1000000 + val->val2;
}

ZTEST(ina3221, test_device_is_ready)
{
	zassert_true(device_is_ready(ina));
}

/* The fetch/get path is kept for existing users, private attribute and all. */
ZTEST(ina3221, test_fetch_get_selected_channel)
{
	struct sensor_value sel = {.val1 = 2};
	struct sensor_value val;

	zassert_ok(sensor_sample_fetch(ina));
	zassert_ok(sensor_attr_set(ina, SENSOR_CHAN_ALL,
				   (enum sensor_attribute)SENSOR_ATTR_INA3221_SELECTED_CHANNEL,
				   &sel));

	zassert_ok(sensor_channel_get(ina, SENSOR_CHAN_VOLTAGE, &val));
	zassert_within(sensor_value_to_micro_int(&val), 3304000, 10);

	zassert_ok(sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &val));
	zassert_within(sensor_value_to_micro_int(&val), -800000, 10);
}

#ifdef CONFIG_SENSOR_ASYNC_API

SENSOR_DT_READ_IODEV(ina3221_iodev, DT_NODELABEL(ina3221), {SENSOR_CHAN_ALL, 0});
RTIO_DEFINE(ina3221_rtio, 1, 1);

static uint8_t read_buf[sizeof(struct ina3221_encoded_data)];

static void read_once(void)
{
	zassert_ok(sensor_read(&ina3221_iodev, &ina3221_rtio, read_buf, sizeof(read_buf)));
}

static int decode(enum sensor_channel type, uint16_t idx, struct sensor_q31_data *out)
{
	const struct sensor_decoder_api *decoder;
	uint32_t fit = 0;

	zassert_ok(sensor_get_decoder(ina, &decoder));

	return decoder->decode(read_buf, (struct sensor_chan_spec){type, idx}, &fit, 1, out);
}

/* Decoded value in micro-units (uV, uA, uW). */
static int64_t decode_micro(enum sensor_channel type, uint16_t idx)
{
	struct sensor_q31_data out;

	zassert_equal(decode(type, idx, &out), 1, "chan %d idx %u did not decode", type, idx);
	zassert_equal(out.header.reading_count, 1);

	return ((int64_t)out.readings[0].value * 1000000 * (1LL << out.shift)) >> 31;
}

ZTEST(ina3221, test_one_read_decodes_every_rail)
{
	uint32_t before = ina3221_emul_conversions();

	read_once();

	/* All three quantities on both rails come from a single conversion. */
	zassert_equal(ina3221_emul_conversions(), before + 1);

	zassert_within(decode_micro(SENSOR_CHAN_VOLTAGE, 0), 5000000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_CURRENT, 0), 3200000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_POWER, 0), 16000000, 100);

	zassert_within(decode_micro(SENSOR_CHAN_VOLTAGE, 1), 3304000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_CURRENT, 1), -800000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_POWER, 1), -2643200, 100);
}

ZTEST(ina3221, test_disabled_or_missing_rail_is_refused)
{
	const struct sensor_decoder_api *decoder;
	struct sensor_q31_data out;
	uint16_t frames;

	read_once();
	zassert_ok(sensor_get_decoder(ina, &decoder));

	zassert_ok(decoder->get_frame_count(
		read_buf, (struct sensor_chan_spec){SENSOR_CHAN_VOLTAGE, 1}, &frames));
	zassert_equal(frames, 1);

	/* Channel 3 (index 2) is disabled, and there is no index 3. */
	zassert_equal(decoder->get_frame_count(
			      read_buf, (struct sensor_chan_spec){SENSOR_CHAN_VOLTAGE, 2}, &frames),
		      -ENOTSUP);
	zassert_equal(decode(SENSOR_CHAN_CURRENT, 2, &out), -EINVAL);
	zassert_equal(decode(SENSOR_CHAN_VOLTAGE, 3, &out), -EINVAL);
	zassert_equal(decode(SENSOR_CHAN_AMBIENT_TEMP, 0, &out), -EINVAL);
}

ZTEST(ina3221, test_full_scale_does_not_overflow)
{
	/* 13-bit full scale on both registers: 32.76 V, 163.8 mV over 10 mOhm. */
	set_rail(0, 4095, 4095);
	set_rail(1, 4095, -4096);
	read_once();

	zassert_within(decode_micro(SENSOR_CHAN_VOLTAGE, 0), 32760000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_CURRENT, 0), 16380000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_POWER, 0), 536608800, 1000);

	/* -163.84 mV over 20 mOhm */
	zassert_within(decode_micro(SENSOR_CHAN_CURRENT, 1), -8192000, 100);
	zassert_within(decode_micro(SENSOR_CHAN_POWER, 1), -268369920, 1000);
}

ZTEST(ina3221, test_timestamp_comes_from_the_read)
{
	struct sensor_q31_data out;
	uint64_t first;

	read_once();
	zassert_equal(decode(SENSOR_CHAN_VOLTAGE, 0, &out), 1);
	first = out.header.base_timestamp_ns;

	k_msleep(5);
	read_once();
	zassert_equal(decode(SENSOR_CHAN_VOLTAGE, 0, &out), 1);

	zassert_true(out.header.base_timestamp_ns >= first + 5000000ULL,
		     "timestamps %llu then %llu", first, out.header.base_timestamp_ns);
}

#endif /* CONFIG_SENSOR_ASYNC_API */

ZTEST_SUITE(ina3221, NULL, NULL, ina3221_before, NULL, NULL);
