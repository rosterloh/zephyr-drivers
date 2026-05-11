/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/ztest.h>
#include <zephyr/actuator/actuator_types.h>
#include <zephyr/actuator/internal/capabilities.h>

ZTEST_SUITE(actuator_caps, NULL, NULL, NULL, NULL, NULL);

ZTEST(actuator_caps, test_mode_supported)
{
	const uint32_t caps = ACTUATOR_CAP_POSITION | ACTUATOR_CAP_VELOCITY;
	zassert_equal(actuator_cap_check_mode(caps, ACTUATOR_MODE_POSITION), 0);
	zassert_equal(actuator_cap_check_mode(caps, ACTUATOR_MODE_VELOCITY), 0);
	zassert_equal(actuator_cap_check_mode(caps, ACTUATOR_MODE_EFFORT), -ENOTSUP);
	zassert_equal(actuator_cap_check_mode(caps, ACTUATOR_MODE_DISABLED), -EINVAL);
}

ZTEST(actuator_caps, test_all_modes_caps)
{
	const uint32_t caps = ACTUATOR_CAP_POSITION | ACTUATOR_CAP_VELOCITY | ACTUATOR_CAP_EFFORT;
	zassert_equal(actuator_cap_check_mode(caps, ACTUATOR_MODE_EFFORT), 0);
}
