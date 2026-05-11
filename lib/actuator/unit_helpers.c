/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/actuator/internal/unit_helpers.h>

float actuator_clamp_nan(float value, float min, float max)
{
	if (!isnan(min) && value < min) {
		return min;
	}
	if (!isnan(max) && value > max) {
		return max;
	}
	return value;
}

int32_t actuator_scale_linear(float value, float src_lo, float src_hi, int32_t dst_lo,
			      int32_t dst_hi)
{
	if (isnan(value) || src_hi == src_lo) {
		return dst_lo;
	}
	if (value <= src_lo) {
		return dst_lo;
	}
	if (value >= src_hi) {
		return dst_hi;
	}
	float t = (value - src_lo) / (src_hi - src_lo);
	float scaled = (float)dst_lo + t * (float)(dst_hi - dst_lo);
	return (int32_t)scaled;
}
