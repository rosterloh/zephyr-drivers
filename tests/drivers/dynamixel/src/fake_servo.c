/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fake_servo.h"

#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/drivers/serial/uart_emul.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fake_servo, LOG_LEVEL_INF);

#define DXL_HDR0 0xFF
#define DXL_HDR1 0xFF
#define DXL_HDR2 0xFD
#define DXL_RSV  0x00

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REBOOT     0x08
#define INST_STATUS     0x55

#define CRC_POLY 0x8005
#define CRC_SEED 0x0000

static void send_status(struct fake_servo *s,
			uint8_t error,
			const uint8_t *params, uint16_t params_len)
{
	uint8_t buf[FAKE_SERVO_STATUS_BUF_SIZE];
	uint16_t length = params_len + 1 /* error */ + 2 /* crc */ + 1 /* instruction */;
	uint16_t total  = length + 7;
	uint16_t crc;

	if (s->drop_response) {
		return;
	}

	buf[0] = DXL_HDR0;
	buf[1] = DXL_HDR1;
	buf[2] = DXL_HDR2;
	buf[3] = DXL_RSV;
	buf[4] = s->id;
	sys_put_le16(length, &buf[5]);
	buf[7] = INST_STATUS;
	buf[8] = error;
	if (params_len > 0) {
		memcpy(&buf[9], params, params_len);
	}
	crc = crc16(CRC_POLY, CRC_SEED, buf, total - 2);
	if (s->corrupt_crc) {
		crc ^= 0x0001;
	}
	sys_put_le16(crc, &buf[total - 2]);

	uart_emul_put_rx_data(s->uart, buf, total);
}

static void handle_packet(struct fake_servo *s, const uint8_t *pkt, size_t len)
{
	uint8_t  id;
	uint16_t length;
	uint8_t  inst;

	if (len < 10) {
		return;
	}
	if (pkt[0] != DXL_HDR0 || pkt[1] != DXL_HDR1 || pkt[2] != DXL_HDR2) {
		return;
	}
	id     = pkt[4];
	length = sys_get_le16(&pkt[5]);
	inst   = pkt[7];

	/* Capture for assertions */
	s->last_tx_len = MIN(len, sizeof(s->last_tx));
	memcpy(s->last_tx, pkt, s->last_tx_len);
	s->last_instruction = inst;

	if (id != s->id) {
		return; /* not addressed to this servo */
	}

	switch (inst) {
	case INST_PING:
		send_status(s, s->error_byte, NULL, 0);
		break;
	case INST_READ: {
		if (len < 14) {
			return; /* malformed: READ needs 4 param bytes */
		}
		uint16_t addr = sys_get_le16(&pkt[8]);
		uint16_t rlen = sys_get_le16(&pkt[10]);
		s->last_addr   = addr;
		s->last_length = rlen;
		if (addr + rlen <= FAKE_SERVO_RAM_SIZE) {
			send_status(s, s->error_byte, &s->control_ram[addr], rlen);
		} else {
			send_status(s, s->error_byte, NULL, 0);
		}
		break;
	}
	case INST_WRITE: {
		if (len < 12) {
			return; /* malformed: WRITE needs at least 2 addr bytes */
		}
		uint16_t addr   = sys_get_le16(&pkt[8]);
		uint16_t params = length - 5; /* length covers ic + 2 addr + params + 2 crc */
		s->last_addr   = addr;
		s->last_length = params;
		if (addr + params <= FAKE_SERVO_RAM_SIZE) {
			memcpy(&s->control_ram[addr], &pkt[10], params);
		}
		send_status(s, s->error_byte, NULL, 0);
		break;
	}
	case INST_REBOOT:
		send_status(s, s->error_byte, NULL, 0);
		break;
	default:
		break;
	}
}

static void tx_data_ready_cb(const struct device *dev, size_t size, void *user_data)
{
	struct fake_servo *s = user_data;
	uint8_t buf[FAKE_SERVO_LAST_TX_SIZE];
	uint32_t got;

	if (size > sizeof(buf)) {
		size = sizeof(buf);
	}
	got = uart_emul_get_tx_data(dev, buf, size);
	if (got == 0) {
		return;
	}
	handle_packet(s, buf, got);
}

void fake_servo_init(struct fake_servo *s, uint8_t id)
{
	memset(s, 0, sizeof(*s));
	s->id = id;
}

void fake_servo_attach(struct fake_servo *s, const struct device *uart)
{
	s->uart = uart;
	uart_emul_callback_tx_data_ready_set(uart, tx_data_ready_cb, s);
}

void fake_servo_set_u8(struct fake_servo *s, uint16_t addr, uint8_t value)
{
	s->control_ram[addr] = value;
}

void fake_servo_set_u16(struct fake_servo *s, uint16_t addr, uint16_t value)
{
	sys_put_le16(value, &s->control_ram[addr]);
}

void fake_servo_set_u32(struct fake_servo *s, uint16_t addr, uint32_t value)
{
	sys_put_le32(value, &s->control_ram[addr]);
}

uint8_t fake_servo_get_u8(struct fake_servo *s, uint16_t addr)
{
	return s->control_ram[addr];
}

uint16_t fake_servo_get_u16(struct fake_servo *s, uint16_t addr)
{
	return sys_get_le16(&s->control_ram[addr]);
}

uint32_t fake_servo_get_u32(struct fake_servo *s, uint16_t addr)
{
	return sys_get_le32(&s->control_ram[addr]);
}
