/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/ztest.h>
#include <drivers/dynamixel.h>

ZTEST(dynamixel_motors, test_motor_count)
{
	zassert_equal(dxl_motor_count(), 2, "two motors in overlay");
}

ZTEST(dynamixel_motors, test_lookup_by_label)
{
	const struct dxl_motor *m = dxl_motor_get_by_label("ALPHA");

	zassert_not_null(m, "ALPHA must be found");
	zassert_equal(m->id, 1, "ALPHA id");

	m = dxl_motor_get_by_label("BETA");
	zassert_not_null(m, "BETA must be found");
	zassert_equal(m->id, 2, "BETA id");
}

ZTEST(dynamixel_motors, test_lookup_missing_label)
{
	zassert_is_null(dxl_motor_get_by_label("nope"), "missing label returns NULL");
}

ZTEST(dynamixel_motors, test_iface_consistent)
{
	const struct dxl_motor *m0 = dxl_motor_get(0);
	const struct dxl_motor *m1 = dxl_motor_get(1);

	zassert_not_null(m0, "motor 0");
	zassert_not_null(m1, "motor 1");
	zassert_equal(m0->iface, m1->iface, "both motors on same iface");
}
