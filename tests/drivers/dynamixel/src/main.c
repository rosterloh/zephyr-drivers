/*
 * Copyright (c) 2024 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_dynamixel.h"

ZTEST(modbus, test_setup)
{
	test_setup_iface();
	test_disable();
}

ZTEST_SUITE(dynamixel, NULL, NULL, NULL, NULL, NULL);