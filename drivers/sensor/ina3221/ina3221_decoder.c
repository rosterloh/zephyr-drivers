/*
 * Copyright (c) 2026 Richard Osterloh
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Decoder for the INA3221 read/decode path. Each of the three rails is a
 * channel index (0-2) of SENSOR_CHAN_VOLTAGE, SENSOR_CHAN_CURRENT and
 * SENSOR_CHAN_POWER, decoded to Q31 volts, amps and watts.
 */

#define DT_DRV_COMPAT ti_ina3221

#include <errno.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

#include "ina3221.h"

/*
 * Fixed Q31 shifts, wide enough for the whole register range:
 * - bus voltage is 13 bits of 8 mV, at most 32.76 V, so 2^6 V;
 * - shunt voltage is 13 bits of 40 uV, at most 163.8 mV, which is 163.8 A
 *   through the smallest (1 mOhm) shunt, so 2^8 A;
 * - power is at most 32.76 V x 163.8 A = 5.4 kW, so 2^13 W.
 */
#define INA3221_VOLTAGE_SHIFT 6
#define INA3221_CURRENT_SHIFT 8
#define INA3221_POWER_SHIFT   13

/* Value of 1.0 in Q31 at a given shift, i.e. 2^(31 - shift). */
#define Q31_ONE(shift) (INT64_C(1) << (31 - (shift)))

static bool ina3221_decoder_has_channel(const struct ina3221_encoded_data *edata,
					struct sensor_chan_spec chan_spec)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_VOLTAGE:
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_POWER:
		break;
	default:
		return false;
	}

	if (chan_spec.chan_idx >= INA3221_NUM_CHANNELS) {
		return false;
	}

	/* No shunt value means no current, and so no power, for that rail. */
	if (chan_spec.chan_type != SENSOR_CHAN_VOLTAGE && edata->shunt_r[chan_spec.chan_idx] == 0) {
		return false;
	}

	return (edata->channels & BIT(chan_spec.chan_idx)) != 0;
}

static int ina3221_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					   uint16_t *frame_count)
{
	const struct ina3221_encoded_data *edata = (const struct ina3221_encoded_data *)buffer;

	if (!ina3221_decoder_has_channel(edata, chan_spec)) {
		return -ENOTSUP;
	}

	/* No FIFO: one reading per read. */
	*frame_count = 1;

	return 0;
}

static int ina3221_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					 size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_VOLTAGE:
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_POWER:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int ina3221_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				  uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct ina3221_encoded_data *edata = (const struct ina3221_encoded_data *)buffer;
	struct sensor_q31_data *out = data_out;
	uint16_t idx = chan_spec.chan_idx;
	int64_t q;

	if (*fit != 0 || max_count == 0) {
		return 0;
	}

	if (!ina3221_decoder_has_channel(edata, chan_spec)) {
		return -EINVAL;
	}

	/*
	 * In SI units:
	 *   V = bus * 8 mV
	 *   I = shunt * 40 uV / (R mOhm)    = shunt * 0.04 / R  A
	 *   P = V * I                        = bus * shunt * 0.00032 / R  W
	 * scaled by 2^(31 - shift). Integer arithmetic throughout: the worst
	 * case product, 4096 * 4096 * 2^26, is well inside int64_t.
	 */
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_VOLTAGE:
		q = (int64_t)edata->v_bus[idx] * 8 * Q31_ONE(INA3221_VOLTAGE_SHIFT) / 1000;
		out->shift = INA3221_VOLTAGE_SHIFT;
		break;
	case SENSOR_CHAN_CURRENT:
		q = (int64_t)edata->v_shunt[idx] * 4 * Q31_ONE(INA3221_CURRENT_SHIFT) /
		    (100 * (int64_t)edata->shunt_r[idx]);
		out->shift = INA3221_CURRENT_SHIFT;
		break;
	case SENSOR_CHAN_POWER:
		q = (int64_t)edata->v_bus[idx] * edata->v_shunt[idx] * 32 *
		    Q31_ONE(INA3221_POWER_SHIFT) / (100000 * (int64_t)edata->shunt_r[idx]);
		out->shift = INA3221_POWER_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	out->header.base_timestamp_ns = edata->timestamp_ns;
	out->header.reading_count = 1;
	out->readings[0].timestamp_delta = 0;
	out->readings[0].value = (q31_t)CLAMP(q, INT32_MIN, INT32_MAX);

	*fit = 1;

	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = ina3221_decoder_get_frame_count,
	.get_size_info = ina3221_decoder_get_size_info,
	.decode = ina3221_decoder_decode,
};

int ina3221_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);

	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
