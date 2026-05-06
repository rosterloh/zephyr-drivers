/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fake_bus.h"

#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/serial/uart_emul.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fake_bus, LOG_LEVEL_INF);

#define DXL_BROADCAST_ID 0xFE
#define INST_SYNC_WRITE  0x83
#define INST_BULK_WRITE  0x93

/* Forward decl — implemented later in the SYNC/BULK tasks. */
static void inject_work_fn(struct k_work *w);

struct fake_servo *fake_bus_get(struct fake_bus *bus, uint8_t id)
{
	for (size_t i = 0; i < bus->n; i++) {
		if (bus->srv[i].id == id) {
			return &bus->srv[i];
		}
	}
	return NULL;
}

/* SYNC_WRITE param layout (after instruction byte):
 *   addr_le16 ‖ data_len_le16 ‖ {id, data[L]} × N
 * pkt[8..9]   = addr,
 * pkt[10..11] = data_len,
 * pkt[12..]   = repeating (id, data[L]).
 */
static void dispatch_sync_write(struct fake_bus *bus, const uint8_t *pkt, size_t len)
{
	if (len < 14) {
		return; /* need at least addr + len + crc */
	}
	uint16_t addr = sys_get_le16(&pkt[8]);
	uint16_t data_len = sys_get_le16(&pkt[10]);
	const uint8_t *p = &pkt[12];
	const uint8_t *end = &pkt[len - 2]; /* before CRC */

	while (p + 1 + data_len <= end) {
		uint8_t id = p[0];
		struct fake_servo *s = fake_bus_get(bus, id);
		if (s != NULL && addr + data_len <= FAKE_SERVO_RAM_SIZE) {
			memcpy(&s->control_ram[addr], &p[1], data_len);
			s->last_addr = addr;
			s->last_length = data_len;
		}
		p += 1 + data_len;
	}
}

/* BULK_WRITE param layout (after instruction byte):
 *   {id, addr_le16, len_le16, data[L]} × N
 * pkt[8..]    = repeating per-entry blocks.
 */
static void dispatch_bulk_write(struct fake_bus *bus, const uint8_t *pkt, size_t len)
{
	if (len < 10) {
		return;
	}
	const uint8_t *p = &pkt[8];
	const uint8_t *end = &pkt[len - 2]; /* before CRC */

	while (p + 5 <= end) {
		uint8_t id = p[0];
		uint16_t addr = sys_get_le16(&p[1]);
		uint16_t data_len = sys_get_le16(&p[3]);
		if (p + 5 + data_len > end) {
			break;
		}
		struct fake_servo *s = fake_bus_get(bus, id);
		if (s != NULL && addr + data_len <= FAKE_SERVO_RAM_SIZE) {
			memcpy(&s->control_ram[addr], &p[5], data_len);
			s->last_addr = addr;
			s->last_length = data_len;
		}
		p += 5 + data_len;
	}
}

static void dispatch_single_target(struct fake_bus *bus, const uint8_t *pkt, size_t len)
{
	uint8_t target_id = pkt[4];

	if (target_id == DXL_BROADCAST_ID) {
		/* Broadcast PING/READ/WRITE — out of scope for Task 4; later tasks handle
		 * SYNC/BULK broadcasts which carry their own targeting in params.
		 */
		return;
	}

	struct fake_servo *s = fake_bus_get(bus, target_id);
	if (s != NULL) {
		fake_servo_handle_packet(s, pkt, len);
	}
}

static void tx_data_ready_cb(const struct device *dev, size_t size, void *user_data)
{
	struct fake_bus *bus = user_data;
	uint8_t buf[FAKE_SERVO_LAST_TX_SIZE * 2]; /* large enough for any single-shot frame */
	uint32_t got;

	if (size > sizeof(buf)) {
		size = sizeof(buf);
	}
	got = uart_emul_get_tx_data(dev, buf, size);
	if (got < 10) {
		return; /* malformed: too short for header+id+len+inst+crc */
	}

	/* Capture in every fake's last_tx/last_instruction so test code can assert
	 * on them regardless of which servo was addressed.
	 */
	for (size_t i = 0; i < bus->n; i++) {
		struct fake_servo *s = &bus->srv[i];
		s->last_tx_len = MIN(got, sizeof(s->last_tx));
		memcpy(s->last_tx, buf, s->last_tx_len);
		s->last_instruction = buf[7];
	}

	uint8_t inst = buf[7];

	switch (inst) {
	case INST_SYNC_WRITE:
		dispatch_sync_write(bus, buf, got);
		break;
	case INST_BULK_WRITE:
		dispatch_bulk_write(bus, buf, got);
		break;
	default:
		dispatch_single_target(bus, buf, got);
		break;
	}
}

void fake_bus_init(struct fake_bus *bus, const uint8_t ids[], size_t n)
{
	memset(bus, 0, sizeof(*bus));
	bus->n = MIN(n, (size_t)FAKE_BUS_MAX_SERVOS);
	for (size_t i = 0; i < bus->n; i++) {
		fake_servo_init(&bus->srv[i], ids[i]);
	}
	k_work_init_delayable(&bus->inject_work, inject_work_fn);
}

void fake_bus_attach(struct fake_bus *bus, const struct device *uart)
{
	bus->uart = uart;
	for (size_t i = 0; i < bus->n; i++) {
		bus->srv[i].uart = uart;
	}
	uart_emul_callback_tx_data_ready_set(uart, tx_data_ready_cb, bus);
}

void fake_bus_set_u8(struct fake_bus *bus, uint8_t id, uint16_t addr, uint8_t value)
{
	struct fake_servo *s = fake_bus_get(bus, id);
	if (s != NULL) {
		fake_servo_set_u8(s, addr, value);
	}
}

void fake_bus_set_u16(struct fake_bus *bus, uint8_t id, uint16_t addr, uint16_t value)
{
	struct fake_servo *s = fake_bus_get(bus, id);
	if (s != NULL) {
		fake_servo_set_u16(s, addr, value);
	}
}

void fake_bus_set_u32(struct fake_bus *bus, uint8_t id, uint16_t addr, uint32_t value)
{
	struct fake_servo *s = fake_bus_get(bus, id);
	if (s != NULL) {
		fake_servo_set_u32(s, addr, value);
	}
}

static void inject_work_fn(struct k_work *w)
{
	/* Filled in by Tasks 9 / 11 / 12 (SYNC_READ / BULK_READ injection). */
	ARG_UNUSED(w);
}
