/*
 * Copyright (c) 2024 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_DYNAMIXEL_H__
#define __TEST_DYNAMIXEL_H__

#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>
#include <drivers/dynamixel.h>

#define DXL_TEST_BAUDRATE_LOW 57600
#define DXL_TEST_BAUDRATE_HIGH 115200
#define DXL_TEST_PARITY UART_CFG_PARITY_NONE
#define DXL_TEST_RESPONSE_TO 5000

/*
 * Integration platform for this test is MKR Zero.
 * The board must be prepared accordingly:
 * SERCOM5(PB23)-RX <-> SERCOM4(PB10)-TX pins and
 * SERCOM5(PB22)-TX <-> SERCOM4(PB11)-RX pins have to be connected.
 */

uint8_t test_get_dxl_iface(void);

void test_setup_iface(void);
void test_disable(void);

#endif /* __TEST_DYNAMIXEL_H__ */