/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>
#include <zephyr/actuator/actuator_types.h>
#include <zephyr/actuator/internal/unit_helpers.h>

ZTEST_SUITE(actuator_units, NULL, NULL, NULL, NULL, NULL);

ZTEST(actuator_units, test_clamp_with_nan_limits_passthrough)
{
	float v = 5.0f;
	zassert_within(actuator_clamp_nan(v, NAN, NAN), 5.0f, 1e-6f);
}

ZTEST(actuator_units, test_clamp_with_min_only)
{
	zassert_within(actuator_clamp_nan(-1.0f, 0.0f, NAN), 0.0f, 1e-6f);
	zassert_within(actuator_clamp_nan(2.0f, 0.0f, NAN), 2.0f, 1e-6f);
}

ZTEST(actuator_units, test_clamp_with_max_only)
{
	zassert_within(actuator_clamp_nan(2.0f, NAN, 1.0f), 1.0f, 1e-6f);
}

ZTEST(actuator_units, test_clamp_with_both)
{
	zassert_within(actuator_clamp_nan(0.5f, 0.0f, 1.0f), 0.5f, 1e-6f);
	zassert_within(actuator_clamp_nan(-1.0f, 0.0f, 1.0f), 0.0f, 1e-6f);
	zassert_within(actuator_clamp_nan(2.0f, 0.0f, 1.0f), 1.0f, 1e-6f);
}

ZTEST(actuator_units, test_scale_linear_basic)
{
	/* domain [-1.0, +1.0] mapped to [0, 4095] (12-bit DAC-ish) */
	zassert_equal(actuator_scale_linear(-1.0f, -1.0f, 1.0f, 0, 4095), 0);
	zassert_equal(actuator_scale_linear(0.0f, -1.0f, 1.0f, 0, 4095), 2047);
	zassert_equal(actuator_scale_linear(1.0f, -1.0f, 1.0f, 0, 4095), 4095);
}

ZTEST(actuator_units, test_scale_linear_clamps_out_of_range)
{
	zassert_equal(actuator_scale_linear(-2.0f, -1.0f, 1.0f, 0, 4095), 0);
	zassert_equal(actuator_scale_linear(5.0f, -1.0f, 1.0f, 0, 4095), 4095);
}

ZTEST(actuator_units, test_ramp_accelerates_at_up_rate)
{
	/* 2/s over 0.1 s = 0.2 per tick */
	zassert_within(actuator_ramp_step(0.0f, 1.0f, 2.0f, 4.0f, 0.1f), 0.2f, 1e-6f);
	zassert_within(actuator_ramp_step(0.0f, -1.0f, 2.0f, 4.0f, 0.1f), -0.2f, 1e-6f);
}

ZTEST(actuator_units, test_ramp_decelerates_at_down_rate)
{
	zassert_within(actuator_ramp_step(1.0f, 0.0f, 2.0f, 4.0f, 0.1f), 0.6f, 1e-6f);
	zassert_within(actuator_ramp_step(-1.0f, -0.5f, 2.0f, 4.0f, 0.1f), -0.6f, 1e-6f);
}

ZTEST(actuator_units, test_ramp_reversal_stops_at_zero_first)
{
	zassert_within(actuator_ramp_step(0.3f, -1.0f, 2.0f, 4.0f, 0.1f), 0.0f, 1e-6f);
	zassert_within(actuator_ramp_step(0.0f, -1.0f, 2.0f, 4.0f, 0.1f), -0.2f, 1e-6f);
}

ZTEST(actuator_units, test_ramp_lands_exactly_on_target)
{
	zassert_equal(actuator_ramp_step(0.9f, 1.0f, 2.0f, 4.0f, 0.1f), 1.0f);
}

ZTEST(actuator_units, test_ramp_zero_rate_is_unlimited)
{
	zassert_equal(actuator_ramp_step(0.0f, 1.0f, 0.0f, 0.0f, 0.1f), 1.0f);
	/* unlimited down still decelerates to 0 before reversing */
	zassert_equal(actuator_ramp_step(1.0f, -1.0f, 2.0f, 0.0f, 0.1f), 0.0f);
}
