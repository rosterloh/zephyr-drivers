/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_FAKE_BUS_H_
#define TEST_FAKE_BUS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/dynamixel.h>

#include "fake_servo.h"

#define FAKE_BUS_MAX_SERVOS 4

/* Inter-packet timing for sync/bulk read injection.
 *
 * SLOT_GAP_US: gap between consecutive injected status packets in the
 *   happy-path case. Must exceed the driver's inter-frame packet_timeout
 *   (~1003 us at 115200 baud) so the packet_timer fires between frames.
 *
 * DROP_GAP_US: gap when the previous slot was a drop. Must satisfy
 *   (rxwait_to - SLOT_GAP) < DROP_GAP < (2*rxwait_to - packet_timer):
 *   - lower bound prevents the next inject arriving while the dropped slot
 *     k_sem_take is still active (would corrupt expected_id matching);
 *   - upper bound ensures the next inject arrives inside the next slot's
 *     k_sem_take window with margin for the inter-frame packet_timer.
 *   With rxwait_to = 10 ms, the safe range is roughly 7..14 ms; 12 ms gives
 *   ~3 ms margin on both sides.
 */
#define FAKE_BUS_SLOT_GAP_US 5000U
#define FAKE_BUS_DROP_GAP_US 12000U

struct fake_bus {
	struct fake_servo srv[FAKE_BUS_MAX_SERVOS];
	size_t n;
	const struct device *uart;

	/* Pending response queue (set by tx dispatcher when SYNC/BULK_READ seen). */
	bool pending_active;
	bool pending_is_bulk;
	uint8_t pending_ids[FAKE_BUS_MAX_SERVOS];
	uint16_t pending_addr_uniform;               /* sync-read only */
	uint16_t pending_len_uniform;                /* sync-read only */
	uint16_t pending_addrs[FAKE_BUS_MAX_SERVOS]; /* bulk-read only */
	uint16_t pending_lens[FAKE_BUS_MAX_SERVOS];  /* bulk-read only */
	size_t pending_n;
	size_t pending_idx;
	bool prev_dropped;

	struct k_work_delayable inject_work;
};

void fake_bus_init(struct fake_bus *bus, const uint8_t ids[], size_t n);
void fake_bus_attach(struct fake_bus *bus, const struct device *uart);

struct fake_servo *fake_bus_get(struct fake_bus *bus, uint8_t id);

/* Convenience wrappers for pre-loading register state per servo by id. */
void fake_bus_set_u8(struct fake_bus *bus, uint8_t id, uint16_t addr, uint8_t value);
void fake_bus_set_u16(struct fake_bus *bus, uint8_t id, uint16_t addr, uint16_t value);
void fake_bus_set_u32(struct fake_bus *bus, uint8_t id, uint16_t addr, uint32_t value);

#endif /* TEST_FAKE_BUS_H_ */
