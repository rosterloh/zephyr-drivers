/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_FAKE_SERVO_H_
#define TEST_FAKE_SERVO_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>

#define FAKE_SERVO_RAM_SIZE 256
#define FAKE_SERVO_LAST_TX_SIZE 64
/* Status reply uses params_len (<= RAM size) + 11 bytes of framing. */
#define FAKE_SERVO_STATUS_BUF_SIZE (FAKE_SERVO_RAM_SIZE + 11)

struct fake_servo {
	uint8_t  id;
	uint8_t  control_ram[FAKE_SERVO_RAM_SIZE];

	/* Behaviour knobs */
	bool     drop_response;
	bool     corrupt_crc;
	uint8_t  error_byte;

	/* Capture of the most recent request */
	uint8_t  last_tx[FAKE_SERVO_LAST_TX_SIZE];
	size_t   last_tx_len;
	uint8_t  last_instruction;
	uint16_t last_addr;
	uint16_t last_length;

	/* Internal: bound UART device for response injection */
	const struct device *uart;
};

void fake_servo_init  (struct fake_servo *s, uint8_t id);
void fake_servo_attach(struct fake_servo *s, const struct device *uart);

/* Pre-load a value into the fake servo's RAM at a given address.
 * Used to set up READ test cases.
 */
void fake_servo_set_u8 (struct fake_servo *s, uint16_t addr, uint8_t  value);
void fake_servo_set_u16(struct fake_servo *s, uint16_t addr, uint16_t value);
void fake_servo_set_u32(struct fake_servo *s, uint16_t addr, uint32_t value);

uint8_t  fake_servo_get_u8 (struct fake_servo *s, uint16_t addr);
uint16_t fake_servo_get_u16(struct fake_servo *s, uint16_t addr);
uint32_t fake_servo_get_u32(struct fake_servo *s, uint16_t addr);

#endif /* TEST_FAKE_SERVO_H_ */
