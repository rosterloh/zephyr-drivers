/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <drivers/dynamixel.h>

/* These tests use the header-published control_table[] today; phase 2
 * replaces that with dxl_table_lookup() and the assertions below
 * stay valid because the values are the same.
 */

ZTEST(dynamixel_tables, test_known_register_addresses)
{
	zassert_equal(control_table[MODEL_NUMBER].address,     0,   "MODEL_NUMBER addr");
	zassert_equal(control_table[MODEL_NUMBER].length,      2,   "MODEL_NUMBER len");
	zassert_equal(control_table[ID].address,               7,   "ID addr");
	zassert_equal(control_table[ID].length,                1,   "ID len");
	zassert_equal(control_table[OPERATING_MODE].address,   11,  "OPERATING_MODE addr");
	zassert_equal(control_table[OPERATING_MODE].length,    1,   "OPERATING_MODE len");
	zassert_equal(control_table[TORQUE_ENABLE].address,    64,  "TORQUE_ENABLE addr");
	zassert_equal(control_table[TORQUE_ENABLE].length,     1,   "TORQUE_ENABLE len");
	zassert_equal(control_table[GOAL_POSITION].address,    116, "GOAL_POSITION addr");
	zassert_equal(control_table[GOAL_POSITION].length,     4,   "GOAL_POSITION len");
	zassert_equal(control_table[PRESENT_POSITION].address, 132, "PRESENT_POSITION addr");
	zassert_equal(control_table[PRESENT_POSITION].length,  4,   "PRESENT_POSITION len");
	zassert_equal(control_table[PRESENT_TEMPERATURE].address, 146, "PRESENT_TEMPERATURE addr");
	zassert_equal(control_table[PRESENT_TEMPERATURE].length,  1, "PRESENT_TEMPERATURE len");
}
