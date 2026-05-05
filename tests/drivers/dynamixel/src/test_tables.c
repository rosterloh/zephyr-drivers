/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/ztest.h>
#include <drivers/dynamixel.h>
#include "dynamixel_internal.h"

ZTEST(dynamixel_tables, test_known_register_addresses)
{
	uint16_t addr;
	uint8_t  length;

	zassert_ok(dxl_table_lookup(MODEL_NUMBER, &addr, &length), "MODEL_NUMBER lookup");
	zassert_equal(addr, 0); zassert_equal(length, 2);

	zassert_ok(dxl_table_lookup(ID, &addr, &length), "ID lookup");
	zassert_equal(addr, 7); zassert_equal(length, 1);

	zassert_ok(dxl_table_lookup(OPERATING_MODE, &addr, &length), "OPERATING_MODE lookup");
	zassert_equal(addr, 11); zassert_equal(length, 1);

	zassert_ok(dxl_table_lookup(TORQUE_ENABLE, &addr, &length), "TORQUE_ENABLE lookup");
	zassert_equal(addr, 64); zassert_equal(length, 1);

	zassert_ok(dxl_table_lookup(GOAL_POSITION, &addr, &length), "GOAL_POSITION lookup");
	zassert_equal(addr, 116); zassert_equal(length, 4);

	zassert_ok(dxl_table_lookup(PRESENT_POSITION, &addr, &length), "PRESENT_POSITION lookup");
	zassert_equal(addr, 132); zassert_equal(length, 4);

	zassert_ok(dxl_table_lookup(PRESENT_TEMPERATURE, &addr, &length), "PRESENT_TEMPERATURE lookup");
	zassert_equal(addr, 146); zassert_equal(length, 1);
}

ZTEST(dynamixel_tables, test_lookup_known_register)
{
	uint16_t addr;
	uint8_t  length;

	zassert_ok(dxl_table_lookup(GOAL_POSITION, &addr, &length), "lookup ok");
	zassert_equal(addr,   116, "GOAL_POSITION addr");
	zassert_equal(length, 4,   "GOAL_POSITION len");
}

ZTEST(dynamixel_tables, test_lookup_out_of_range)
{
	zassert_equal(dxl_table_lookup((enum dxl_control)9999, NULL, NULL),
		      -EINVAL, "out-of-range returns -EINVAL");
}

ZTEST(dynamixel_tables, test_lookup_null_outparams)
{
	zassert_ok(dxl_table_lookup(MODEL_NUMBER, NULL, NULL),
		   "NULL outparams allowed");
}
