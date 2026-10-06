/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_UNIT_HELPERS_H_
#define ZEPHYR_INCLUDE_ACTUATOR_INTERNAL_UNIT_HELPERS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * NaN-safe clamp: NaN min or max is treated as "unbounded on that side".
 */
float actuator_clamp_nan(float value, float min, float max);

/**
 * Linear scale a float in [src_lo, src_hi] to an int32 in [dst_lo, dst_hi].
 * Out-of-range inputs are clamped. NaN input maps to dst_lo.
 */
int32_t actuator_scale_linear(float value, float src_lo, float src_hi, int32_t dst_lo,
			      int32_t dst_hi);

/**
 * Advance @p current one tick of @p dt_s toward @p target. Growing magnitude
 * moves at @p up_per_s, shrinking magnitude at @p down_per_s; a sign change
 * first decelerates to 0. A rate <= 0 means unlimited.
 */
float actuator_ramp_step(float current, float target, float up_per_s, float down_per_s, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
