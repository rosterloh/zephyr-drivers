# Dynamixel SYNC / BULK Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Protocol-2 group instructions (SYNC_READ 0x82, SYNC_WRITE 0x83, BULK_READ 0x92, BULK_WRITE 0x93) to the Dynamixel driver, with stateless C-array API.

**Architecture:** New `dynamixel_group.c` file holds all four entry points and packet builders. Multi-status reads loop the existing single-frame primitive (`dxl_tx_wait_rx`) under one mutex; no changes to `dynamixel_serial.c`. Test harness gains a `fake_bus` that owns up to 4 fake servos sharing the emulated UART, with a delayed-work item that injects per-servo status packets at deterministic gaps.

**Tech Stack:** Zephyr RTOS, Protocol 2 (CRC-16/CCITT 0x8005), ZTest, `zephyr,uart-emul`, `k_work_delayable`, `k_mutex`, `k_sem`.

**Spec:** [`docs/superpowers/specs/2026-05-05-dynamixel-sync-bulk-design.md`](../specs/2026-05-05-dynamixel-sync-bulk-design.md)

**Branch:** `feature/dynamixel-sync-bulk` (already created, contains the spec commit).

---

## File Structure

| File | Disposition | Responsibility |
|---|---|---|
| `drivers/dynamixel/dynamixel_internal.h` | Modify | Declare `parse_status_payload` |
| `drivers/dynamixel/dynamixel_protocol.c` | Modify | Refactor `dxl_read_n` to use the new helper |
| `drivers/dynamixel/dynamixel_group.c` | Create | All four sync/bulk entry points + builders |
| `drivers/dynamixel/CMakeLists.txt` | Modify | Add `dynamixel_group.c` to library |
| `drivers/dynamixel/Kconfig` | Modify | Bump `DYNAMIXEL_BUFFER_SIZE` range, document formula |
| `include/drivers/dynamixel.h` | Modify | Public sync/bulk prototypes + entry structs |
| `tests/drivers/dynamixel/src/fake_bus.h` | Create | Multi-servo dispatcher header |
| `tests/drivers/dynamixel/src/fake_bus.c` | Create | Multi-servo dispatcher impl + delayed-work injector |
| `tests/drivers/dynamixel/src/fake_servo.c` | Modify | Expose `fake_servo_handle_packet` for bus to call |
| `tests/drivers/dynamixel/src/fake_servo.h` | Modify | Declare `fake_servo_handle_packet` |
| `tests/drivers/dynamixel/src/test_sync.c` | Create | SYNC_READ + SYNC_WRITE tests |
| `tests/drivers/dynamixel/src/test_bulk.c` | Create | BULK_READ + BULK_WRITE tests |
| `tests/drivers/dynamixel/CMakeLists.txt` | Modify | Add `fake_bus.c`, `test_sync.c`, `test_bulk.c` |
| `tests/drivers/dynamixel/prj.conf` | Modify | (none expected — `CONFIG_EMUL`, `CONFIG_UART_EMUL`, `CONFIG_DYNAMIXEL` already on) |

**Test fixture timing constants** (used by `fake_bus`):

| Constant | Value | Reason |
|---|---|---|
| `FAKE_BUS_RXWAIT_TO_US` | 10000 | 10 ms — sync/bulk tests use a shorter timeout than `test_protocol`'s 50 ms so drop scenarios complete fast in simulated time. |
| `FAKE_BUS_SLOT_GAP_US` | 5000 | 5 ms — gap between consecutive injected status packets. Must exceed the driver's inter-frame `packet_timeout` (~1003 µs at 115200 baud) by a clear margin. |
| `FAKE_BUS_DROP_GAP_US` | 15000 | `RXWAIT_TO + SLOT_GAP` — gap when the previous slot was a drop, so the driver finishes timing out before the next packet arrives. |

---

## Phase 1: Foundation (encapsulation + Kconfig)

### Task 1: Extract `parse_status_payload`

**Why:** Both single-shot reads and the new sync/bulk reads need to parse a status frame `(data, width) → (dev_err | value)`. Today the logic lives inline in `dxl_read_n`. Extracting it now keeps the rest of the plan free of duplicated parsing code, and is a behaviour-preserving refactor that the existing 22 tests cover.

**Files:**
- Modify: `drivers/dynamixel/dynamixel_internal.h`
- Modify: `drivers/dynamixel/dynamixel_protocol.c`

- [ ] **Step 1: Verify existing tests pass before any change**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All existing tests PASS.

- [ ] **Step 2: Declare `parse_status_payload` in the internal header**

Add at the bottom of `drivers/dynamixel/dynamixel_internal.h`, just before the closing `#endif`:

```c
/**
 * @brief Parse a status-packet payload into a uint32_t value.
 *
 * @param data   Pointer to the status packet's data area, where data[0] is
 *               the device-error byte and data[1..] is the value payload.
 * @param width  Register width in bytes. Must be 1, 2, or 4.
 * @param out    On success, receives the value as a uint32_t (LE-decoded).
 *
 * @retval 0       Device replied success and *out is valid.
 * @retval >0      Device-error byte (see enum dxl_error). *out untouched.
 * @retval -EINVAL Unsupported width.
 */
int parse_status_payload(const uint8_t *data, uint8_t width, uint32_t *out);
```

- [ ] **Step 3: Add the implementation in `dynamixel_protocol.c`**

Insert at the top of `dynamixel_protocol.c`, right after the includes and `LOG_MODULE_DECLARE` line, before the existing `dxl_ping`:

```c
int parse_status_payload(const uint8_t *data, uint8_t width, uint32_t *out)
{
	uint8_t dev_err = data[0];

	if (dev_err != 0) {
		return (int)dev_err;
	}
	switch (width) {
	case 1:
		*out = data[1];
		return 0;
	case 2:
		*out = sys_get_le16(&data[1]);
		return 0;
	case 4:
		*out = sys_get_le32(&data[1]);
		return 0;
	default:
		return -EINVAL;
	}
}
```

- [ ] **Step 4: Replumb `dxl_read_n` to call the helper**

Replace the success-path block in `dxl_read_n` (currently a `switch (length)` reading from `ctx->rx_frame.data[0..]`):

```c
	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		err = parse_status_payload(ctx->rx_frame.data, length, out);
	}
```

Drop the local `dev_err`, the inner `switch`, and the local `default: err = -EINVAL` — all collapse into the helper.

- [ ] **Step 5: Run tests to confirm zero regression**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All 22 existing tests still PASS. No new tests yet.

- [ ] **Step 6: Commit**

```bash
git add drivers/dynamixel/dynamixel_internal.h drivers/dynamixel/dynamixel_protocol.c
git commit -m "drivers: dynamixel: extract parse_status_payload helper"
```

---

### Task 2: Bump `DYNAMIXEL_BUFFER_SIZE` Kconfig range

**Why:** BULK_WRITE with N×L > 256 bytes can exceed today's max. The spec calls for `range 64 1024` plus a sizing-formula help block.

**Files:**
- Modify: `drivers/dynamixel/Kconfig`

- [ ] **Step 1: Edit `drivers/dynamixel/Kconfig`**

Replace lines 13–18 (the existing `config DYNAMIXEL_BUFFER_SIZE` block) with:

```kconfig
config DYNAMIXEL_BUFFER_SIZE
	int "Dynamixel buffer size"
	default 256
	range 64 1024
	help
	  Maximum frame size (bytes) for the Dynamixel TX/RX buffers.
	  Must be large enough for the largest packet you intend to send.

	  SYNC_WRITE  ~= 14 + N * (1 + L)
	  BULK_WRITE  ~= 10 + N * (5 + L)

	  where N is the number of servos addressed and L is the register
	  width in bytes. The default of 256 covers ~12 servos x 4-byte
	  writes; raise it if you need more.
```

- [ ] **Step 2: Confirm test build still works at the default**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All tests PASS, default 256-byte buffer unchanged.

- [ ] **Step 3: Commit**

```bash
git add drivers/dynamixel/Kconfig
git commit -m "drivers: dynamixel: raise BUFFER_SIZE max to 1024, document formula"
```

---

## Phase 2: Test harness — `fake_bus`

The harness needs to own up to 4 `fake_servo` instances sharing one emulated UART, dispatch incoming TX bytes by ID + instruction, and inject SYNC_READ / BULK_READ status packets one at a time with deterministic spacing.

### Task 3: Expose `fake_servo_handle_packet` for the bus

**Why:** The bus must call into per-fake packet handling. Today `handle_packet` is `static` inside `fake_servo.c`. Lift it to a public function so `fake_bus.c` can dispatch.

**Files:**
- Modify: `tests/drivers/dynamixel/src/fake_servo.h`
- Modify: `tests/drivers/dynamixel/src/fake_servo.c`

- [ ] **Step 1: Declare in header**

Add to `tests/drivers/dynamixel/src/fake_servo.h`, after the existing `fake_servo_get_*` declarations and before `#endif`:

```c
/* Process one received instruction packet. The fake reacts (sends status if
 * applicable) and updates last_tx / last_instruction / last_addr / last_length.
 * Used by fake_bus to dispatch packets to the right servo.
 */
void fake_servo_handle_packet(struct fake_servo *s, const uint8_t *pkt, size_t len);
```

- [ ] **Step 2: Rename and unstaticise in implementation**

In `tests/drivers/dynamixel/src/fake_servo.c`:

Change the function signature:

```c
void fake_servo_handle_packet(struct fake_servo *s, const uint8_t *pkt, size_t len)
```

(was `static void handle_packet(struct fake_servo *s, ...)`).

Update its single internal caller `tx_data_ready_cb` to use the new name:

```c
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
	fake_servo_handle_packet(s, buf, got);
}
```

- [ ] **Step 3: Build and run existing tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All 22 tests PASS — the rename is mechanical, behaviour unchanged.

- [ ] **Step 4: Commit**

```bash
git add tests/drivers/dynamixel/src/fake_servo.h tests/drivers/dynamixel/src/fake_servo.c
git commit -m "test: dynamixel: expose fake_servo_handle_packet for bus dispatch"
```

---

### Task 4: `fake_bus` skeleton — multi-fake PING / READ / WRITE dispatch

**Why:** Establish the bus container, the UART callback, and the simplest dispatch path (single-target instructions). Reusing the existing tests' assertions, we'll verify the bus behaves identically to `fake_servo_attach` for n=1 cases.

**Files:**
- Create: `tests/drivers/dynamixel/src/fake_bus.h`
- Create: `tests/drivers/dynamixel/src/fake_bus.c`
- Modify: `tests/drivers/dynamixel/CMakeLists.txt`

- [ ] **Step 1: Add `fake_bus.c` to the test build**

In `tests/drivers/dynamixel/CMakeLists.txt`, edit the `target_sources(app PRIVATE …)` block to insert `src/fake_bus.c`:

```cmake
target_sources(app PRIVATE
    src/main.c
    src/fake_servo.c
    src/fake_bus.c
    src/test_protocol.c
    src/test_tables.c
    src/test_motors.c
)
```

- [ ] **Step 2: Create the header**

Write `tests/drivers/dynamixel/src/fake_bus.h`:

```c
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

#define FAKE_BUS_MAX_SERVOS  4

/* Inter-packet timing for sync/bulk read injection. See plan/spec for rationale. */
#define FAKE_BUS_SLOT_GAP_US  5000U
#define FAKE_BUS_DROP_GAP_US  15000U

struct fake_bus {
	struct fake_servo srv[FAKE_BUS_MAX_SERVOS];
	size_t n;
	const struct device *uart;

	/* Pending response queue (set by tx dispatcher when SYNC/BULK_READ seen). */
	bool pending_active;
	bool pending_is_bulk;
	uint8_t pending_ids[FAKE_BUS_MAX_SERVOS];
	uint16_t pending_addr_uniform;     /* sync-read only */
	uint16_t pending_len_uniform;      /* sync-read only */
	uint16_t pending_addrs[FAKE_BUS_MAX_SERVOS];   /* bulk-read only */
	uint16_t pending_lens[FAKE_BUS_MAX_SERVOS];    /* bulk-read only */
	size_t pending_n;
	size_t pending_idx;
	bool prev_dropped;

	struct k_work_delayable inject_work;
};

void fake_bus_init  (struct fake_bus *bus, const uint8_t ids[], size_t n);
void fake_bus_attach(struct fake_bus *bus, const struct device *uart);

struct fake_servo *fake_bus_get(struct fake_bus *bus, uint8_t id);

/* Convenience wrappers for pre-loading register state per servo by id. */
void fake_bus_set_u8 (struct fake_bus *bus, uint8_t id, uint16_t addr, uint8_t  value);
void fake_bus_set_u16(struct fake_bus *bus, uint8_t id, uint16_t addr, uint16_t value);
void fake_bus_set_u32(struct fake_bus *bus, uint8_t id, uint16_t addr, uint32_t value);

#endif /* TEST_FAKE_BUS_H_ */
```

- [ ] **Step 3: Create the implementation skeleton**

Write `tests/drivers/dynamixel/src/fake_bus.c`. This first cut handles only single-target packets (PING, READ, WRITE) and mirrors the existing single-fake behaviour for those instructions:

```c
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

	dispatch_single_target(bus, buf, got);
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
```

- [ ] **Step 4: Build and run existing tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All 22 existing tests PASS — `fake_bus.c` compiles in but isn't used yet by any test.

- [ ] **Step 5: Commit**

```bash
git add tests/drivers/dynamixel/src/fake_bus.h tests/drivers/dynamixel/src/fake_bus.c \
        tests/drivers/dynamixel/CMakeLists.txt
git commit -m "test: dynamixel: add fake_bus multi-servo dispatch skeleton"
```

---

### Task 5: `fake_bus` — SYNC_WRITE / BULK_WRITE broadcast handling

**Why:** Both write group instructions are broadcast (target ID `0xFE`) and produce no status replies. Each targeted fake must apply its slice to its own register file. This is the simplest broadcast case; Task 9 will build on the same dispatcher for SYNC_READ injection.

**Files:**
- Modify: `tests/drivers/dynamixel/src/fake_bus.c`

- [ ] **Step 1: Add helper to apply SYNC_WRITE params**

Insert above `dispatch_single_target` in `fake_bus.c`:

```c
#define INST_SYNC_WRITE  0x83
#define INST_BULK_WRITE  0x93

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
```

- [ ] **Step 2: Wire broadcast cases into `tx_data_ready_cb`**

Replace the `dispatch_single_target(bus, buf, got);` line in `tx_data_ready_cb` with a switch on instruction:

```c
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
```

- [ ] **Step 3: Build (no new test runs yet)**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All 22 existing tests PASS — write-broadcast handlers are unreached so far.

- [ ] **Step 4: Commit**

```bash
git add tests/drivers/dynamixel/src/fake_bus.c
git commit -m "test: dynamixel: dispatch SYNC_WRITE/BULK_WRITE broadcasts in fake_bus"
```

---

## Phase 3: SYNC_WRITE

### Task 6: SYNC_WRITE API stubs and a failing happy-path test

**Why:** TDD step — write the test first against stubs that compile but return `-ENOSYS`.

**Files:**
- Modify: `include/drivers/dynamixel.h`
- Create: `drivers/dynamixel/dynamixel_group.c`
- Modify: `drivers/dynamixel/CMakeLists.txt`
- Create: `tests/drivers/dynamixel/src/test_sync.c`
- Modify: `tests/drivers/dynamixel/CMakeLists.txt`

- [ ] **Step 1: Add SYNC public prototypes to `include/drivers/dynamixel.h`**

Insert after the `dxl_write_u32` declaration (around line 217), before `dxl_iface_get_by_name`:

```c
/**
 * @brief Write the same register on multiple servos in one transaction.
 *
 * SYNC_WRITE (Protocol-2 0x83) is broadcast and produces no status replies.
 * Per-servo failure (e.g. servo unplugged, register read-only) is not
 * detectable without a follow-up read.
 *
 * @param iface Dynamixel interface index.
 * @param item  Control register identifier (must match the typed width).
 * @param ids   Array of N servo bus IDs.
 * @param vals  Array of N values to write.
 * @param n     Number of servos (must be > 0).
 *
 * @retval 0       All bytes written to the bus.
 * @retval -EINVAL n=0, NULL pointers, or item width mismatch.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_sync_write_u8 (int iface, enum dxl_control item,
		       const uint8_t ids[], const uint8_t  vals[], size_t n);
int dxl_sync_write_u16(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint16_t vals[], size_t n);
int dxl_sync_write_u32(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint32_t vals[], size_t n);

/**
 * @brief Read the same register from multiple servos in one transaction.
 *
 * SYNC_READ (Protocol-2 0x82) sends a broadcast instruction; each addressed
 * servo replies with its own status packet in ID-list order.
 *
 * @param iface Dynamixel interface index.
 * @param item  Control register identifier (must match the typed width).
 * @param ids   Array of N servo bus IDs.
 * @param vals  On per-slot success, receives that servo's value.
 * @param errs  Optional. If non-NULL, receives per-slot result:
 *              0 = ok, >0 = device-error byte, <0 = transport error.
 *              Pass NULL to opt out of per-slot detail; failed slots leave
 *              vals[i] untouched.
 * @param n     Number of servos (must be > 0).
 *
 * @retval 0       All servos succeeded.
 * @retval -EIO    At least one servo failed; check errs[] if non-NULL.
 * @retval -EINVAL n=0, NULL ids/vals, or item width mismatch.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_sync_read_u8 (int iface, enum dxl_control item,
		      const uint8_t ids[], uint8_t  vals[],
		      int errs[], size_t n);
int dxl_sync_read_u16(int iface, enum dxl_control item,
		      const uint8_t ids[], uint16_t vals[],
		      int errs[], size_t n);
int dxl_sync_read_u32(int iface, enum dxl_control item,
		      const uint8_t ids[], uint32_t vals[],
		      int errs[], size_t n);
```

- [ ] **Step 2: Add `dynamixel_group.c` with stub implementations**

Create `drivers/dynamixel/dynamixel_group.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

int dxl_sync_write_u8(int iface, enum dxl_control item,
		      const uint8_t ids[], const uint8_t vals[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_write_u16(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint16_t vals[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_write_u32(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint32_t vals[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_read_u8(int iface, enum dxl_control item,
		     const uint8_t ids[], uint8_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_read_u16(int iface, enum dxl_control item,
		      const uint8_t ids[], uint16_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_sync_read_u32(int iface, enum dxl_control item,
		      const uint8_t ids[], uint32_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(item);
	ARG_UNUSED(ids);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}
```

- [ ] **Step 3: Add `dynamixel_group.c` to the driver build**

Edit `drivers/dynamixel/CMakeLists.txt`:

```cmake
zephyr_library()

zephyr_library_sources(
  dynamixel.c
  dynamixel_serial.c
  dynamixel_protocol.c
  dynamixel_tables.c
  dynamixel_group.c
)
```

- [ ] **Step 4: Add `test_sync.c` with the first failing test**

Create `tests/drivers/dynamixel/src/test_sync.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/dynamixel.h>

#include "fake_bus.h"

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE  DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define FAKE_BUS_RXWAIT_TO_US 10000U

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_bus bus;
static int iface;

static void sync_before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	const uint8_t ids[] = {1, 2, 3, 4};
	struct dxl_iface_param p = {
		.rx_timeout = FAKE_BUS_RXWAIT_TO_US,
		.serial = {.baud = 115200, .parity = UART_CFG_PARITY_NONE},
	};

	fake_bus_init(&bus, ids, ARRAY_SIZE(ids));
	fake_bus_attach(&bus, uart_dev);

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(dxl_init(iface, p), "dxl_init failed");
}

static void sync_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dynamixel_sync, NULL, NULL, sync_before_each, sync_after_each, NULL);

ZTEST(dynamixel_sync, test_sync_write_u32_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	const uint32_t vals[] = {0x100, 0x200, 0x300, 0x400};

	zassert_ok(dxl_sync_write_u32(iface, GOAL_POSITION, ids, vals, ARRAY_SIZE(ids)),
		   "sync_write failed");

	/* GOAL_POSITION is at addr 116 (4 bytes). Each servo should have its slice. */
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 1), 116), 0x100, "id 1");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 2), 116), 0x200, "id 2");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 3), 116), 0x300, "id 3");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 4), 116), 0x400, "id 4");
}
```

- [ ] **Step 5: Add `test_sync.c` to the test build**

Edit `tests/drivers/dynamixel/CMakeLists.txt`:

```cmake
target_sources(app PRIVATE
    src/main.c
    src/fake_servo.c
    src/fake_bus.c
    src/test_protocol.c
    src/test_tables.c
    src/test_motors.c
    src/test_sync.c
)
```

- [ ] **Step 6: Run the new test — expect FAIL**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All 22 existing tests PASS. New `test_sync_write_u32_happy` FAILS at the first `zassert_ok` because `dxl_sync_write_u32` returns `-ENOSYS`.

- [ ] **Step 7: Commit**

```bash
git add include/drivers/dynamixel.h drivers/dynamixel/dynamixel_group.c \
        drivers/dynamixel/CMakeLists.txt \
        tests/drivers/dynamixel/src/test_sync.c \
        tests/drivers/dynamixel/CMakeLists.txt
git commit -m "drivers/test: dynamixel: stubs and failing test for sync_write_u32"
```

---

### Task 7: Implement SYNC_WRITE builder + happy path

**Why:** Make the failing test from Task 6 pass. SYNC_WRITE has no status replies, so the implementation is the simplest of the four — build the broadcast packet, hand to `dxl_serial_tx`, no wait.

**Files:**
- Modify: `drivers/dynamixel/dynamixel_group.c`

- [ ] **Step 1: Add internal helpers**

At the top of `dynamixel_group.c`, after the includes and `LOG_MODULE_DECLARE`:

```c
#define DXL_INST_SYNC_READ   0x82
#define DXL_INST_SYNC_WRITE  0x83
#define DXL_INST_BULK_READ   0x92
#define DXL_INST_BULK_WRITE  0x93

/* Driver-side cap on per-call entries for BULK paths. Bounds the
 * stack-allocated `addrs[]` and `widths[]` working arrays. Sync paths do not
 * need this cap because they don't allocate per-entry stack arrays.
 */
#define DXL_BULK_MAX_ENTRIES 256

/* Pack a value of `width` bytes (1/2/4) into dst in little-endian. */
static inline void pack_le(uint8_t *dst, uint8_t width, uint32_t v)
{
	switch (width) {
	case 1: dst[0] = (uint8_t)v; break;
	case 2: sys_put_le16((uint16_t)v, dst); break;
	case 4: sys_put_le32(v, dst); break;
	default: break;
	}
}
```

- [ ] **Step 2: Add the shared sync_write entry point**

Append to `dynamixel_group.c`:

```c
static int sync_write_n(int iface, enum dxl_control item, uint8_t expected_width,
			const uint8_t ids[], const void *vals, size_t n)
{
	if (iface < 0 || n == 0 || ids == NULL || vals == NULL) {
		return -EINVAL;
	}

	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	uint16_t addr;
	uint8_t width;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup(item, &addr, &width) != 0) {
		return -EINVAL;
	}
	if (width != expected_width) {
		return -EINVAL;
	}

	/* params = addr_le16 (2) + data_len_le16 (2) + N * (1 + width).
	 * Use size_t through the math so huge n cannot wrap a uint16_t.
	 */
	size_t params_len = 4U + n * (1U + width);
	size_t length_field = 1U /* inst */ + params_len + 2U /* crc */;

	if (length_field + 7U > CONFIG_DYNAMIXEL_BUFFER_SIZE) {
		return -ENOSPC;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = 0xFE; /* broadcast */
	ctx->tx_frame.length = (uint16_t)length_field;
	ctx->tx_frame.ic = DXL_INST_SYNC_WRITE;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	sys_put_le16(width, &ctx->tx_frame.data[2]);

	uint8_t *p = &ctx->tx_frame.data[4];
	const uint8_t *vbytes = vals; /* used for u8 path */
	const uint16_t *v16 = vals;
	const uint32_t *v32 = vals;

	for (size_t i = 0; i < n; i++) {
		*p++ = ids[i];
		switch (width) {
		case 1: pack_le(p, 1, vbytes[i]); break;
		case 2: pack_le(p, 2, v16[i]);    break;
		case 4: pack_le(p, 4, v32[i]);    break;
		}
		p += width;
	}

	dxl_serial_tx(ctx);
	/* SYNC_WRITE is broadcast; no status. tx_frame is held under the mutex
	 * until tx completion drives RX back on, but we have nothing to wait for.
	 */

	k_mutex_unlock(&ctx->iface_lock);
	return 0;
}

int dxl_sync_write_u8(int iface, enum dxl_control item,
		      const uint8_t ids[], const uint8_t vals[], size_t n)
{
	return sync_write_n(iface, item, 1, ids, vals, n);
}

int dxl_sync_write_u16(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint16_t vals[], size_t n)
{
	return sync_write_n(iface, item, 2, ids, vals, n);
}

int dxl_sync_write_u32(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint32_t vals[], size_t n)
{
	return sync_write_n(iface, item, 4, ids, vals, n);
}
```

Delete the three earlier `-ENOSYS` stubs for `dxl_sync_write_*`; the new bodies above replace them.

- [ ] **Step 3: Run the new test — expect PASS**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: `test_sync_write_u32_happy` PASSES. All other tests (22 existing) still PASS.

- [ ] **Step 4: Commit**

```bash
git add drivers/dynamixel/dynamixel_group.c
git commit -m "drivers: dynamixel: implement SYNC_WRITE u8/u16/u32"
```

---

### Task 8: SYNC_WRITE — width variants and width-mismatch coverage

**Why:** Lock down the typed-width contract: each `_u*` variant must reject items whose table width disagrees, and u8/u16 happy paths must work.

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_sync.c`

- [ ] **Step 1: Append three new tests after `test_sync_write_u32_happy`**

Insert at end of `test_sync.c`, before the file's closing newline:

```c
ZTEST(dynamixel_sync, test_sync_write_u8_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	const uint8_t vals[] = {1, 0, 1, 0};

	zassert_ok(dxl_sync_write_u8(iface, TORQUE_ENABLE, ids, vals, ARRAY_SIZE(ids)),
		   "sync_write_u8 failed");

	/* TORQUE_ENABLE is at addr 64 (1 byte). */
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 1), 64), 1, "id 1");
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 2), 64), 0, "id 2");
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 3), 64), 1, "id 3");
	zassert_equal(fake_servo_get_u8(fake_bus_get(&bus, 4), 64), 0, "id 4");
}

ZTEST(dynamixel_sync, test_sync_write_u16_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	const uint16_t vals[] = {0x0AAA, 0x0BBB, 0x0CCC, 0x0DDD};

	zassert_ok(dxl_sync_write_u16(iface, GOAL_PWM, ids, vals, ARRAY_SIZE(ids)),
		   "sync_write_u16 failed");

	/* GOAL_PWM is at addr 100 (2 bytes). */
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 1), 100), 0x0AAA, "id 1");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 2), 100), 0x0BBB, "id 2");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 3), 100), 0x0CCC, "id 3");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 4), 100), 0x0DDD, "id 4");
}

ZTEST(dynamixel_sync, test_sync_write_width_mismatch_returns_einval)
{
	const uint8_t ids[] = {1, 2};
	const uint8_t vals_u8[] = {0, 0};
	const uint16_t vals_u16[] = {0, 0};

	/* u8 helper called on a 4-byte register (GOAL_POSITION). */
	zassert_equal(dxl_sync_write_u8(iface, GOAL_POSITION, ids, vals_u8, 2),
		      -EINVAL, "u8 on 4-byte register");

	/* u16 helper called on a 1-byte register (TORQUE_ENABLE). */
	zassert_equal(dxl_sync_write_u16(iface, TORQUE_ENABLE, ids, vals_u16, 2),
		      -EINVAL, "u16 on 1-byte register");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All three new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_sync.c
git commit -m "test: dynamixel: cover sync_write u8/u16/width-mismatch"
```

---

### Task 9: SYNC_WRITE — validation and -ENOSPC

**Why:** Pin down the boundary cases listed in the spec error contract: bad inputs and oversized requests must be detected before touching the bus.

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_sync.c`

- [ ] **Step 1: Append the validation tests**

Insert at end of `test_sync.c`:

```c
ZTEST(dynamixel_sync, test_sync_write_validation_einval)
{
	const uint8_t ids[] = {1, 2};
	const uint32_t vals[] = {0, 0};

	/* n = 0 is invalid. */
	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, ids, vals, 0),
		      -EINVAL, "n=0");

	/* NULL ids. */
	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, NULL, vals, 2),
		      -EINVAL, "NULL ids");

	/* NULL vals. */
	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, ids, NULL, 2),
		      -EINVAL, "NULL vals");

	/* Out-of-range item. */
	zassert_equal(dxl_sync_write_u32(iface, (enum dxl_control)9999, ids, vals, 2),
		      -EINVAL, "bad item");
}

ZTEST(dynamixel_sync, test_sync_write_enospc_when_oversized)
{
	/* CONFIG_DYNAMIXEL_BUFFER_SIZE defaults to 256. SYNC_WRITE u32 with N
	 * servos needs 14 + N*5 bytes. N=49 -> 259 > 256. Use a fabricated
	 * over-large id list (the dispatcher won't actually fire — call returns
	 * -ENOSPC before TX).
	 */
	uint8_t ids[64];
	uint32_t vals[64];
	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		ids[i] = (uint8_t)(i + 1);
		vals[i] = 0;
	}

	zassert_equal(dxl_sync_write_u32(iface, GOAL_POSITION, ids, vals, 64),
		      -ENOSPC, "oversized request must return -ENOSPC");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: Both new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_sync.c
git commit -m "test: dynamixel: cover sync_write validation and -ENOSPC"
```

---

## Phase 4: SYNC_READ

### Task 10: `fake_bus` — SYNC_READ inject_work plumbing

**Why:** SYNC_READ requires the fake to emit one status packet per requested ID, **separated by gaps** so the driver's inter-frame `packet_timer` fires between them. Implementing this in the bus's delayed-work item lets us drive timing deterministically in native_sim.

**Files:**
- Modify: `tests/drivers/dynamixel/src/fake_bus.c`

- [ ] **Step 1: Add SYNC_READ dispatch + inject_work logic**

In `fake_bus.c`, near the existing `dispatch_sync_write` function, add:

```c
#define INST_SYNC_READ   0x82
#define INST_BULK_READ   0x92

/* Helper: directly emit a status reply for one fake. Mirrors fake_servo's
 * own send_status path but is reachable from outside the per-fake module.
 *
 * Implementation: reuses fake_servo_handle_packet by feeding it a synthetic
 * READ instruction packet for that servo. This way fake_servo's existing
 * drop_response / corrupt_crc / error_byte knobs apply automatically.
 */
static void emit_status_via_synthetic_read(struct fake_servo *s,
					   uint16_t addr, uint16_t length)
{
	uint8_t synth[14];
	uint16_t length_field = 1 /* inst */ + 4 /* params */ + 2 /* crc */;

	synth[0] = 0xFF;
	synth[1] = 0xFF;
	synth[2] = 0xFD;
	synth[3] = 0x00;
	synth[4] = s->id;
	sys_put_le16(length_field, &synth[5]);
	synth[7] = 0x02; /* INST_READ */
	sys_put_le16(addr, &synth[8]);
	sys_put_le16(length, &synth[10]);
	/* CRC bytes left zero — fake_servo_handle_packet doesn't validate the
	 * inbound CRC, only header + instruction. The bytes at &synth[12] are
	 * "CRC", but handle_packet treats pkt[8..11] as params and ignores
	 * trailing bytes for READ.
	 */

	fake_servo_handle_packet(s, synth, sizeof(synth));
}

/* SYNC_READ param layout (after instruction byte):
 *   addr_le16 ‖ data_len_le16 ‖ id[0..n-1]
 */
static void dispatch_sync_read(struct fake_bus *bus, const uint8_t *pkt, size_t len)
{
	if (len < 14) {
		return;
	}
	uint16_t addr = sys_get_le16(&pkt[8]);
	uint16_t data_len = sys_get_le16(&pkt[10]);
	const uint8_t *id_list = &pkt[12];
	const uint8_t *end = &pkt[len - 2]; /* before CRC */
	size_t n = (size_t)(end - id_list);

	if (n == 0 || n > FAKE_BUS_MAX_SERVOS) {
		return;
	}

	bus->pending_active = true;
	bus->pending_is_bulk = false;
	bus->pending_addr_uniform = addr;
	bus->pending_len_uniform = data_len;
	bus->pending_n = n;
	bus->pending_idx = 0;
	bus->prev_dropped = false;
	for (size_t i = 0; i < n; i++) {
		bus->pending_ids[i] = id_list[i];
	}

	/* Kick off the first injection at K_NO_WAIT so it runs after the test
	 * thread has returned from dxl_sync_read_*'s TX call and entered the
	 * first k_sem_take. K_NO_WAIT is fine because the work item runs on the
	 * system workqueue which can't pre-empt the test thread mid-sem-take.
	 */
	k_work_reschedule(&bus->inject_work, K_NO_WAIT);
}
```

- [ ] **Step 2: Replace the `inject_work_fn` stub**

Find the placeholder added in Task 4:

```c
static void inject_work_fn(struct k_work *w)
{
	/* Filled in by Tasks 9 / 11 / 12 (SYNC_READ / BULK_READ injection). */
	ARG_UNUSED(w);
}
```

Replace with:

```c
static void inject_work_fn(struct k_work *w)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(w);
	struct fake_bus *bus = CONTAINER_OF(dw, struct fake_bus, inject_work);

	if (!bus->pending_active || bus->pending_idx >= bus->pending_n) {
		bus->pending_active = false;
		return;
	}

	uint8_t id = bus->pending_ids[bus->pending_idx];
	uint16_t addr = bus->pending_is_bulk ? bus->pending_addrs[bus->pending_idx]
					     : bus->pending_addr_uniform;
	uint16_t length = bus->pending_is_bulk ? bus->pending_lens[bus->pending_idx]
					       : bus->pending_len_uniform;

	struct fake_servo *s = fake_bus_get(bus, id);
	bool dropped = (s == NULL) || s->drop_response;

	if (s != NULL) {
		emit_status_via_synthetic_read(s, addr, length);
	}

	bus->pending_idx++;
	bus->prev_dropped = dropped;

	if (bus->pending_idx < bus->pending_n) {
		uint32_t gap = bus->prev_dropped ? FAKE_BUS_DROP_GAP_US
						 : FAKE_BUS_SLOT_GAP_US;
		k_work_reschedule(&bus->inject_work, K_USEC(gap));
	} else {
		bus->pending_active = false;
	}
}
```

- [ ] **Step 3: Wire SYNC_READ into the dispatcher switch**

In `tx_data_ready_cb`, extend the switch added in Task 5:

```c
	switch (inst) {
	case INST_SYNC_READ:
		dispatch_sync_read(bus, buf, got);
		break;
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
```

- [ ] **Step 4: Build, no behaviour change yet**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All existing tests still PASS — sync_read code path is unreached from the driver.

- [ ] **Step 5: Commit**

```bash
git add tests/drivers/dynamixel/src/fake_bus.c
git commit -m "test: dynamixel: SYNC_READ dispatch + delayed status injection in fake_bus"
```

---

### Task 11: SYNC_READ — happy path

**Why:** Implement the multi-status loop and prove it works for the simplest case.

**Files:**
- Modify: `drivers/dynamixel/dynamixel_group.c`
- Modify: `tests/drivers/dynamixel/src/test_sync.c`
- Modify: `drivers/dynamixel/dynamixel_internal.h`

- [ ] **Step 1: Expose `dxl_serial_rx_enable` so the protocol layer can re-arm RX between iterations**

Already declared in `dynamixel_internal.h:106` (`void dxl_serial_rx_enable(struct dxl_context *ctx);`). No change needed — just confirm visibility from `dynamixel_group.c` via the `#include "dynamixel_internal.h"` already present.

- [ ] **Step 2: Add the shared sync_read entry point in `dynamixel_group.c`**

Append to `dynamixel_group.c`:

```c
static void unpack_to_typed(void *vals, size_t i, uint8_t width, uint32_t v)
{
	switch (width) {
	case 1: ((uint8_t  *)vals)[i] = (uint8_t)v;  break;
	case 2: ((uint16_t *)vals)[i] = (uint16_t)v; break;
	case 4: ((uint32_t *)vals)[i] = v;           break;
	}
}

static int sync_read_n(int iface, enum dxl_control item, uint8_t expected_width,
		       const uint8_t ids[], void *vals, int errs[], size_t n)
{
	if (iface < 0 || n == 0 || ids == NULL || vals == NULL) {
		return -EINVAL;
	}

	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	uint16_t addr;
	uint8_t width;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup(item, &addr, &width) != 0) {
		return -EINVAL;
	}
	if (width != expected_width) {
		return -EINVAL;
	}

	/* params = addr_le16 (2) + data_len_le16 (2) + N (1 byte per id).
	 * size_t arithmetic prevents wraparound for large n.
	 */
	size_t params_len = 4U + n;
	size_t length_field = 1U + params_len + 2U;

	if (length_field + 7U > CONFIG_DYNAMIXEL_BUFFER_SIZE) {
		return -ENOSPC;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = 0xFE;
	ctx->tx_frame.length = (uint16_t)length_field;
	ctx->tx_frame.ic = DXL_INST_SYNC_READ;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	sys_put_le16(width, &ctx->tx_frame.data[2]);
	for (size_t i = 0; i < n; i++) {
		ctx->tx_frame.data[4 + i] = ids[i];
	}

	int summary = 0;

	/* For the first slot the existing path drives TX + waits on the
	 * semaphore. For each subsequent slot we set expected_id, re-enable RX,
	 * and wait on the semaphore again — same single-frame primitive.
	 */
	for (size_t i = 0; i < n; i++) {
		ctx->expected_id = ids[i];

		int err;
		if (i == 0) {
			err = dxl_tx_wait_rx(ctx);
		} else {
			dxl_serial_rx_enable(ctx);
			if (k_sem_take(&ctx->wait_sem,
				       K_USEC(ctx->rxwait_to)) != 0) {
				err = -ETIMEDOUT;
			} else {
				err = ctx->rx_frame_err;
			}
		}

		int slot_rc;
		if (err == 0) {
			uint32_t v = 0;
			slot_rc = parse_status_payload(ctx->rx_frame.data, width, &v);
			if (slot_rc == 0) {
				unpack_to_typed(vals, i, width, v);
			}
		} else {
			slot_rc = err; /* -ETIMEDOUT, -EIO, -EBADMSG */
		}

		if (errs != NULL) {
			errs[i] = slot_rc;
		}
		if (slot_rc != 0) {
			summary = -EIO;
		}
	}

	k_mutex_unlock(&ctx->iface_lock);
	return summary;
}

int dxl_sync_read_u8(int iface, enum dxl_control item,
		     const uint8_t ids[], uint8_t vals[], int errs[], size_t n)
{
	return sync_read_n(iface, item, 1, ids, vals, errs, n);
}

int dxl_sync_read_u16(int iface, enum dxl_control item,
		      const uint8_t ids[], uint16_t vals[], int errs[], size_t n)
{
	return sync_read_n(iface, item, 2, ids, vals, errs, n);
}

int dxl_sync_read_u32(int iface, enum dxl_control item,
		      const uint8_t ids[], uint32_t vals[], int errs[], size_t n)
{
	return sync_read_n(iface, item, 4, ids, vals, errs, n);
}
```

Delete the three earlier `-ENOSYS` stubs for `dxl_sync_read_*`.

- [ ] **Step 3: Add the happy-path test**

Append to `test_sync.c`:

```c
ZTEST(dynamixel_sync, test_sync_read_u32_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	/* PRESENT_POSITION at addr 132 (4 bytes). */
	fake_bus_set_u32(&bus, 1, 132, 0x11111111);
	fake_bus_set_u32(&bus, 2, 132, 0x22222222);
	fake_bus_set_u32(&bus, 3, 132, 0x33333333);
	fake_bus_set_u32(&bus, 4, 132, 0x44444444);

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));
	zassert_ok(rc, "sync_read returned %d", rc);

	zassert_equal(vals[0], 0x11111111, "vals[0]");
	zassert_equal(vals[1], 0x22222222, "vals[1]");
	zassert_equal(vals[2], 0x33333333, "vals[2]");
	zassert_equal(vals[3], 0x44444444, "vals[3]");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], 0, "errs[2]");
	zassert_equal(errs[3], 0, "errs[3]");
}
```

- [ ] **Step 4: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: `test_sync_read_u32_happy` PASSES. All previous tests still PASS.

- [ ] **Step 5: Commit**

```bash
git add drivers/dynamixel/dynamixel_group.c tests/drivers/dynamixel/src/test_sync.c
git commit -m "drivers/test: dynamixel: implement SYNC_READ multi-status loop"
```

---

### Task 12: SYNC_READ — width variants

**Why:** Same shape as `test_sync_write_u8/u16_happy` but for the read path.

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_sync.c`

- [ ] **Step 1: Append two more tests**

```c
ZTEST(dynamixel_sync, test_sync_read_u8_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint8_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	/* TORQUE_ENABLE at addr 64 (1 byte). */
	fake_bus_set_u8(&bus, 1, 64, 1);
	fake_bus_set_u8(&bus, 2, 64, 0);
	fake_bus_set_u8(&bus, 3, 64, 1);
	fake_bus_set_u8(&bus, 4, 64, 0);

	zassert_ok(dxl_sync_read_u8(iface, TORQUE_ENABLE, ids, vals, errs,
				    ARRAY_SIZE(ids)),
		   "sync_read_u8 failed");
	zassert_equal(vals[0], 1, "");
	zassert_equal(vals[1], 0, "");
	zassert_equal(vals[2], 1, "");
	zassert_equal(vals[3], 0, "");
}

ZTEST(dynamixel_sync, test_sync_read_u16_happy)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint16_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	/* GOAL_PWM at addr 100 (2 bytes). */
	fake_bus_set_u16(&bus, 1, 100, 0x1111);
	fake_bus_set_u16(&bus, 2, 100, 0x2222);
	fake_bus_set_u16(&bus, 3, 100, 0x3333);
	fake_bus_set_u16(&bus, 4, 100, 0x4444);

	zassert_ok(dxl_sync_read_u16(iface, GOAL_PWM, ids, vals, errs,
				     ARRAY_SIZE(ids)),
		   "sync_read_u16 failed");
	zassert_equal(vals[0], 0x1111, "");
	zassert_equal(vals[1], 0x2222, "");
	zassert_equal(vals[2], 0x3333, "");
	zassert_equal(vals[3], 0x4444, "");
}
```

- [ ] **Step 2: Run tests, expect PASS**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: Both new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_sync.c
git commit -m "test: dynamixel: cover sync_read u8 and u16 happy paths"
```

---

### Task 13: SYNC_READ — partial fail (drop_response)

**Why:** Verify the loop's "always run all N" semantics: a single dropped servo must not poison later slots; per-slot error must be reported as `-ETIMEDOUT` while remaining slots succeed.

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_sync.c`

- [ ] **Step 1: Append the test**

```c
ZTEST(dynamixel_sync, test_sync_read_partial_drop)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED};
	int errs[4] = {0};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	/* Servo 3 silently fails to respond. Servos 1/2/4 reply normally. */
	fake_bus_get(&bus, 3)->drop_response = true;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO, "summary must be -EIO when any slot fails, got %d", rc);
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], -ETIMEDOUT, "errs[2] expected -ETIMEDOUT, got %d", errs[2]);
	zassert_equal(errs[3], 0, "errs[3]");

	zassert_equal(vals[0], 0xAAAA0001, "vals[0]");
	zassert_equal(vals[1], 0xAAAA0002, "vals[1]");
	zassert_equal(vals[2], 0xFEEDFEED, "vals[2] must be untouched on drop");
	zassert_equal(vals[3], 0xAAAA0004, "vals[3]");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: `test_sync_read_partial_drop` PASSES.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_sync.c
git commit -m "test: dynamixel: cover sync_read partial drop and -EIO summary"
```

---

### Task 14: SYNC_READ — device error byte and wrong-ID reply

**Why:** Cover the two non-timeout failure modes: a status with a non-zero error byte (positive return in errs[]) and an unexpected ID (reported as -EBADMSG, surfaced as -ETIMEDOUT in the wait — see existing `test_phase3_wrong_id_response_rejected` for prior art).

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_sync.c`

- [ ] **Step 1: Append both tests**

```c
ZTEST(dynamixel_sync, test_sync_read_device_error_byte)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0};
	int errs[4] = {0};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	/* Servo 2 returns a device-error byte. The driver treats it as a
	 * positive return code in errs[1]; vals[1] is left untouched.
	 */
	fake_bus_get(&bus, 2)->error_byte = DXL_ERR_DATA_RANGE;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO, "summary must be -EIO");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], DXL_ERR_DATA_RANGE,
		      "errs[1] must be device-error byte, got %d", errs[1]);
	zassert_equal(errs[2], 0, "errs[2]");
	zassert_equal(errs[3], 0, "errs[3]");
	zassert_equal(vals[1], 0, "vals[1] must be untouched on dev err");
	zassert_equal(vals[0], 0xAAAA0001, "vals[0]");
	zassert_equal(vals[2], 0xAAAA0003, "vals[2]");
	zassert_equal(vals[3], 0xAAAA0004, "vals[3]");
}

ZTEST(dynamixel_sync, test_sync_read_wrong_id_reply_is_timeout)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0};
	int errs[4] = {0};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	/* For slot 2 (id=3 expected), make servo 3 reply with a different id (5).
	 * The driver discards the wrong-id frame in dxl_rx_handler (-EBADMSG)
	 * and silently re-enables RX. With no other reply incoming, the slot
	 * times out as -ETIMEDOUT. Subsequent slots still proceed.
	 */
	struct fake_servo *s3 = fake_bus_get(&bus, 3);
	s3->id = 5; /* respond as id 5 even though id 3 was requested */
	s3->answer_any_id = true;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, errs,
				   ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO, "summary must be -EIO");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], -ETIMEDOUT,
		      "errs[2] expected -ETIMEDOUT for wrong-id, got %d", errs[2]);
	zassert_equal(errs[3], 0, "errs[3]");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: Both new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_sync.c
git commit -m "test: dynamixel: cover sync_read device-error byte and wrong-id reply"
```

---

### Task 15: SYNC_READ — `errs == NULL` and validation/-ENOSPC

**Why:** Lock down the `errs == NULL` opt-out (function still returns `-EIO` on any failure) and the bad-input / oversized cases.

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_sync.c`

- [ ] **Step 1: Append the tests**

```c
ZTEST(dynamixel_sync, test_sync_read_errs_null_collapses_to_eio)
{
	const uint8_t ids[] = {1, 2, 3, 4};
	uint32_t vals[4] = {0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED, 0xFEEDFEED};

	fake_bus_set_u32(&bus, 1, 132, 0xAAAA0001);
	fake_bus_set_u32(&bus, 2, 132, 0xAAAA0002);
	fake_bus_set_u32(&bus, 3, 132, 0xAAAA0003);
	fake_bus_set_u32(&bus, 4, 132, 0xAAAA0004);

	fake_bus_get(&bus, 3)->drop_response = true;

	int rc = dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals,
				   /*errs=*/NULL, ARRAY_SIZE(ids));

	zassert_equal(rc, -EIO,
		      "errs=NULL must still collapse failures to -EIO, got %d", rc);
	zassert_equal(vals[0], 0xAAAA0001, "vals[0]");
	zassert_equal(vals[1], 0xAAAA0002, "vals[1]");
	zassert_equal(vals[2], 0xFEEDFEED,
		      "vals[2] must be untouched even when errs is NULL");
	zassert_equal(vals[3], 0xAAAA0004, "vals[3]");
}

ZTEST(dynamixel_sync, test_sync_read_validation_einval)
{
	const uint8_t ids[] = {1, 2};
	uint32_t vals[2] = {0};

	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, NULL, 0),
		      -EINVAL, "n=0");
	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, NULL, vals, NULL, 2),
		      -EINVAL, "NULL ids");
	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, ids, NULL, NULL, 2),
		      -EINVAL, "NULL vals");
	zassert_equal(dxl_sync_read_u32(iface, (enum dxl_control)9999, ids, vals, NULL, 2),
		      -EINVAL, "bad item");

	uint8_t vals_u8[2] = {0};
	zassert_equal(dxl_sync_read_u8(iface, PRESENT_POSITION, ids, vals_u8, NULL, 2),
		      -EINVAL, "u8 on 4-byte register");
}

ZTEST(dynamixel_sync, test_sync_read_enospc_when_oversized)
{
	uint8_t ids[256];
	uint32_t vals[256];
	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		ids[i] = (uint8_t)((i % 250) + 1);
		vals[i] = 0;
	}

	/* SYNC_READ packet is 14 + N bytes. For N=256 -> 270 > 256 default. */
	zassert_equal(dxl_sync_read_u32(iface, PRESENT_POSITION, ids, vals, NULL, 256),
		      -ENOSPC, "oversized SYNC_READ must return -ENOSPC");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All three new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_sync.c
git commit -m "test: dynamixel: cover sync_read errs=NULL, validation, -ENOSPC"
```

---

## Phase 5: BULK_READ

### Task 16: BULK_READ — public API + `fake_bus` dispatch

**Why:** BULK_READ shares all the loop machinery with SYNC_READ, but each entry has its own `(addr, length)`. Add the entry struct, stub, and bus dispatch in one task; the implementation pass follows in Task 17.

**Files:**
- Modify: `include/drivers/dynamixel.h`
- Modify: `drivers/dynamixel/dynamixel_group.c`
- Modify: `tests/drivers/dynamixel/src/fake_bus.c`
- Create: `tests/drivers/dynamixel/src/test_bulk.c`
- Modify: `tests/drivers/dynamixel/CMakeLists.txt`

- [ ] **Step 1: Add public BULK structs and prototypes to `include/drivers/dynamixel.h`**

Insert immediately after the SYNC prototypes added in Task 6:

```c
/**
 * @brief Per-entry record for BULK_READ.
 */
struct dxl_bulk_read_entry {
	uint8_t id;
	enum dxl_control item;
};

/**
 * @brief Per-entry record for BULK_WRITE.
 *
 * @c value is interpreted at the width returned by @c dxl_table_lookup(item).
 */
struct dxl_bulk_write_entry {
	uint8_t id;
	enum dxl_control item;
	uint32_t value;
};

/**
 * @brief Read different registers from multiple servos in one transaction.
 *
 * BULK_READ (Protocol-2 0x92) sends a broadcast instruction; each addressed
 * servo replies with its own status packet in entry order.
 *
 * @param iface Dynamixel interface index.
 * @param req   Array of N {id, item} entries.
 * @param vals  On per-slot success, receives that entry's value as uint32_t.
 *              All current control-table items fit in 32 bits.
 * @param errs  Optional. Same semantics as dxl_sync_read_*.
 * @param n     Number of entries (must be > 0).
 *
 * @retval 0       All entries succeeded.
 * @retval -EIO    At least one entry failed; check errs[] if non-NULL.
 * @retval -EINVAL n=0, NULL pointers, or any req[i].item out of range.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_bulk_read(int iface, const struct dxl_bulk_read_entry req[],
		  uint32_t vals[], int errs[], size_t n);

/**
 * @brief Write different registers to multiple servos in one transaction.
 *
 * BULK_WRITE (Protocol-2 0x93) is broadcast and produces no status replies.
 *
 * @param iface Dynamixel interface index.
 * @param req   Array of N {id, item, value} entries.
 * @param n     Number of entries (must be > 0).
 *
 * @retval 0       All bytes written to the bus.
 * @retval -EINVAL n=0, NULL req, or any req[i].item out of range.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_bulk_write(int iface, const struct dxl_bulk_write_entry req[], size_t n);
```

- [ ] **Step 2: Add stub implementations in `dynamixel_group.c`**

Append:

```c
int dxl_bulk_read(int iface, const struct dxl_bulk_read_entry req[],
		  uint32_t vals[], int errs[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(req);
	ARG_UNUSED(vals);
	ARG_UNUSED(errs);
	ARG_UNUSED(n);
	return -ENOSYS;
}

int dxl_bulk_write(int iface, const struct dxl_bulk_write_entry req[], size_t n)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(req);
	ARG_UNUSED(n);
	return -ENOSYS;
}
```

- [ ] **Step 3: Add BULK_READ dispatch to `fake_bus.c`**

Insert near the other dispatchers in `fake_bus.c`:

```c
/* BULK_READ param layout (after instruction byte):
 *   {id, addr_le16, len_le16} × N
 */
static void dispatch_bulk_read(struct fake_bus *bus, const uint8_t *pkt, size_t len)
{
	if (len < 10) {
		return;
	}
	const uint8_t *p = &pkt[8];
	const uint8_t *end = &pkt[len - 2];
	size_t entries = 0;

	bus->pending_active = true;
	bus->pending_is_bulk = true;
	bus->pending_idx = 0;
	bus->prev_dropped = false;

	while (p + 5 <= end && entries < FAKE_BUS_MAX_SERVOS) {
		bus->pending_ids[entries]   = p[0];
		bus->pending_addrs[entries] = sys_get_le16(&p[1]);
		bus->pending_lens[entries]  = sys_get_le16(&p[3]);
		entries++;
		p += 5;
	}

	if (entries == 0) {
		bus->pending_active = false;
		return;
	}

	bus->pending_n = entries;
	k_work_reschedule(&bus->inject_work, K_NO_WAIT);
}
```

Wire it into the dispatcher switch in `tx_data_ready_cb`:

```c
	switch (inst) {
	case INST_SYNC_READ:
		dispatch_sync_read(bus, buf, got);
		break;
	case INST_SYNC_WRITE:
		dispatch_sync_write(bus, buf, got);
		break;
	case INST_BULK_READ:
		dispatch_bulk_read(bus, buf, got);
		break;
	case INST_BULK_WRITE:
		dispatch_bulk_write(bus, buf, got);
		break;
	default:
		dispatch_single_target(bus, buf, got);
		break;
	}
```

- [ ] **Step 4: Create `test_bulk.c` with the failing happy-path test**

Create `tests/drivers/dynamixel/src/test_bulk.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/dynamixel.h>

#include "fake_bus.h"

#define DXL_BUS_NODE   DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE  DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME DEVICE_DT_NAME(DXL_BUS_NODE)

#define FAKE_BUS_RXWAIT_TO_US 10000U

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_bus bus;
static int iface;

static void bulk_before_each(void *fixture)
{
	ARG_UNUSED(fixture);

	const uint8_t ids[] = {1, 2, 3, 4};
	struct dxl_iface_param p = {
		.rx_timeout = FAKE_BUS_RXWAIT_TO_US,
		.serial = {.baud = 115200, .parity = UART_CFG_PARITY_NONE},
	};

	fake_bus_init(&bus, ids, ARRAY_SIZE(ids));
	fake_bus_attach(&bus, uart_dev);

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(dxl_init(iface, p), "dxl_init failed");
}

static void bulk_after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	dxl_disable(iface);
}

ZTEST_SUITE(dynamixel_bulk, NULL, NULL, bulk_before_each, bulk_after_each, NULL);

ZTEST(dynamixel_bulk, test_bulk_read_happy_mixed_widths)
{
	const struct dxl_bulk_read_entry req[] = {
		{ .id = 1, .item = TORQUE_ENABLE    }, /* width 1, addr 64 */
		{ .id = 2, .item = GOAL_PWM         }, /* width 2, addr 100 */
		{ .id = 3, .item = PRESENT_POSITION }, /* width 4, addr 132 */
		{ .id = 4, .item = TORQUE_ENABLE    }, /* width 1 */
	};
	uint32_t vals[4] = {0};
	int errs[4] = {-1, -1, -1, -1};

	fake_bus_set_u8 (&bus, 1, 64,  1);
	fake_bus_set_u16(&bus, 2, 100, 0xCAFE);
	fake_bus_set_u32(&bus, 3, 132, 0xDEADBEEF);
	fake_bus_set_u8 (&bus, 4, 64,  0);

	zassert_ok(dxl_bulk_read(iface, req, vals, errs, ARRAY_SIZE(req)),
		   "bulk_read failed");

	zassert_equal(vals[0], 1, "vals[0]");
	zassert_equal(vals[1], 0xCAFE, "vals[1]");
	zassert_equal(vals[2], 0xDEADBEEF, "vals[2]");
	zassert_equal(vals[3], 0, "vals[3]");
	zassert_equal(errs[0], 0, "errs[0]");
	zassert_equal(errs[1], 0, "errs[1]");
	zassert_equal(errs[2], 0, "errs[2]");
	zassert_equal(errs[3], 0, "errs[3]");
}
```

- [ ] **Step 5: Add to test build**

Edit `tests/drivers/dynamixel/CMakeLists.txt`:

```cmake
target_sources(app PRIVATE
    src/main.c
    src/fake_servo.c
    src/fake_bus.c
    src/test_protocol.c
    src/test_tables.c
    src/test_motors.c
    src/test_sync.c
    src/test_bulk.c
)
```

- [ ] **Step 6: Run — expect new test FAILS, others PASS**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: `test_bulk_read_happy_mixed_widths` FAILS at `zassert_ok` because `dxl_bulk_read` is `-ENOSYS`. All other tests still PASS.

- [ ] **Step 7: Commit**

```bash
git add include/drivers/dynamixel.h drivers/dynamixel/dynamixel_group.c \
        tests/drivers/dynamixel/src/fake_bus.c \
        tests/drivers/dynamixel/src/test_bulk.c \
        tests/drivers/dynamixel/CMakeLists.txt
git commit -m "drivers/test: dynamixel: BULK_READ stubs, fake_bus dispatch, failing test"
```

---

### Task 17: Implement BULK_READ

**Why:** Make Task 16's failing test pass.

**Files:**
- Modify: `drivers/dynamixel/dynamixel_group.c`

- [ ] **Step 1: Replace the `dxl_bulk_read` stub**

Replace the stub from Task 16 with:

```c
int dxl_bulk_read(int iface, const struct dxl_bulk_read_entry req[],
		  uint32_t vals[], int errs[], size_t n)
{
	if (iface < 0 || n == 0 || req == NULL || vals == NULL) {
		return -EINVAL;
	}

	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	if (ctx == NULL) {
		return -ENODEV;
	}

	if (n > DXL_BULK_MAX_ENTRIES) {
		return -EINVAL;
	}

	uint16_t addrs[DXL_BULK_MAX_ENTRIES];
	uint8_t  widths[DXL_BULK_MAX_ENTRIES];
	size_t total_params = 0;

	for (size_t i = 0; i < n; i++) {
		uint16_t a;
		uint8_t  w;
		if (dxl_table_lookup(req[i].item, &a, &w) != 0) {
			return -EINVAL;
		}
		addrs[i] = a;
		widths[i] = w;
		total_params += 5U; /* id + addr_le16 + len_le16 */
	}

	size_t length_field = 1U + total_params + 2U;
	if (length_field + 7U > CONFIG_DYNAMIXEL_BUFFER_SIZE) {
		return -ENOSPC;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = 0xFE;
	ctx->tx_frame.length = (uint16_t)length_field;
	ctx->tx_frame.ic = DXL_INST_BULK_READ;
	uint8_t *p = &ctx->tx_frame.data[0];
	for (size_t i = 0; i < n; i++) {
		*p++ = req[i].id;
		sys_put_le16(addrs[i], p);  p += 2;
		sys_put_le16(widths[i], p); p += 2;
	}

	int summary = 0;
	for (size_t i = 0; i < n; i++) {
		ctx->expected_id = req[i].id;

		int err;
		if (i == 0) {
			err = dxl_tx_wait_rx(ctx);
		} else {
			dxl_serial_rx_enable(ctx);
			if (k_sem_take(&ctx->wait_sem,
				       K_USEC(ctx->rxwait_to)) != 0) {
				err = -ETIMEDOUT;
			} else {
				err = ctx->rx_frame_err;
			}
		}

		int slot_rc;
		if (err == 0) {
			uint32_t v = 0;
			slot_rc = parse_status_payload(ctx->rx_frame.data,
						       widths[i], &v);
			if (slot_rc == 0) {
				vals[i] = v;
			}
		} else {
			slot_rc = err;
		}

		if (errs != NULL) {
			errs[i] = slot_rc;
		}
		if (slot_rc != 0) {
			summary = -EIO;
		}
	}

	k_mutex_unlock(&ctx->iface_lock);
	return summary;
}
```

(`DXL_BULK_MAX_ENTRIES` is already defined in Task 7's helper block at the top of `dynamixel_group.c`.)

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: `test_bulk_read_happy_mixed_widths` PASSES.

- [ ] **Step 3: Commit**

```bash
git add drivers/dynamixel/dynamixel_group.c
git commit -m "drivers: dynamixel: implement BULK_READ multi-status loop"
```

---

### Task 18: BULK_READ — validation and -ENOSPC

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_bulk.c`

- [ ] **Step 1: Append the tests**

```c
ZTEST(dynamixel_bulk, test_bulk_read_validation_einval)
{
	const struct dxl_bulk_read_entry req[] = {
		{ .id = 1, .item = PRESENT_POSITION },
		{ .id = 2, .item = PRESENT_POSITION },
	};
	const struct dxl_bulk_read_entry bad_req[] = {
		{ .id = 1, .item = (enum dxl_control)9999 },
	};
	uint32_t vals[2] = {0};

	zassert_equal(dxl_bulk_read(iface, req, vals, NULL, 0),
		      -EINVAL, "n=0");
	zassert_equal(dxl_bulk_read(iface, NULL, vals, NULL, 2),
		      -EINVAL, "NULL req");
	zassert_equal(dxl_bulk_read(iface, req, NULL, NULL, 2),
		      -EINVAL, "NULL vals");
	zassert_equal(dxl_bulk_read(iface, bad_req, vals, NULL, 1),
		      -EINVAL, "bad item");
}

ZTEST(dynamixel_bulk, test_bulk_read_enospc_when_oversized)
{
	struct dxl_bulk_read_entry req[64];
	uint32_t vals[64];
	for (size_t i = 0; i < ARRAY_SIZE(req); i++) {
		req[i].id = (uint8_t)((i % 250) + 1);
		req[i].item = PRESENT_POSITION;
		vals[i] = 0;
	}

	/* BULK_READ packet is 10 + N*5 bytes. For N=64 -> 330 > 256 default. */
	zassert_equal(dxl_bulk_read(iface, req, vals, NULL, 64),
		      -ENOSPC, "oversized BULK_READ must return -ENOSPC");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: Both new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_bulk.c
git commit -m "test: dynamixel: cover bulk_read validation and -ENOSPC"
```

---

## Phase 6: BULK_WRITE

### Task 19: Implement BULK_WRITE + happy path

**Files:**
- Modify: `drivers/dynamixel/dynamixel_group.c`
- Modify: `tests/drivers/dynamixel/src/test_bulk.c`

- [ ] **Step 1: Replace the `dxl_bulk_write` stub**

Replace the stub:

```c
int dxl_bulk_write(int iface, const struct dxl_bulk_write_entry req[], size_t n)
{
	if (iface < 0 || n == 0 || req == NULL) {
		return -EINVAL;
	}

	struct dxl_context *ctx = dxl_get_context((uint8_t)iface);
	if (ctx == NULL) {
		return -ENODEV;
	}

	if (n > DXL_BULK_MAX_ENTRIES) {
		return -EINVAL;
	}

	uint16_t addrs[DXL_BULK_MAX_ENTRIES];
	uint8_t  widths[DXL_BULK_MAX_ENTRIES];
	size_t total_params = 0;

	for (size_t i = 0; i < n; i++) {
		uint16_t a;
		uint8_t  w;
		if (dxl_table_lookup(req[i].item, &a, &w) != 0) {
			return -EINVAL;
		}
		addrs[i] = a;
		widths[i] = w;
		total_params += 5U + w; /* id + addr + len + data */
	}

	size_t length_field = 1U + total_params + 2U;
	if (length_field + 7U > CONFIG_DYNAMIXEL_BUFFER_SIZE) {
		return -ENOSPC;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = 0xFE;
	ctx->tx_frame.length = (uint16_t)length_field;
	ctx->tx_frame.ic = DXL_INST_BULK_WRITE;
	uint8_t *p = &ctx->tx_frame.data[0];
	for (size_t i = 0; i < n; i++) {
		*p++ = req[i].id;
		sys_put_le16(addrs[i], p);  p += 2;
		sys_put_le16(widths[i], p); p += 2;
		pack_le(p, widths[i], req[i].value);
		p += widths[i];
	}

	dxl_serial_tx(ctx);
	k_mutex_unlock(&ctx->iface_lock);
	return 0;
}
```

- [ ] **Step 2: Add the happy-path test**

Append to `test_bulk.c`:

```c
ZTEST(dynamixel_bulk, test_bulk_write_happy_mixed_widths)
{
	const struct dxl_bulk_write_entry req[] = {
		{ .id = 1, .item = TORQUE_ENABLE,    .value = 1          },
		{ .id = 2, .item = GOAL_PWM,         .value = 0x0BEE     },
		{ .id = 3, .item = GOAL_POSITION,    .value = 0xDEADBEEF },
		{ .id = 4, .item = TORQUE_ENABLE,    .value = 0          },
	};

	zassert_ok(dxl_bulk_write(iface, req, ARRAY_SIZE(req)),
		   "bulk_write failed");

	zassert_equal(fake_servo_get_u8 (fake_bus_get(&bus, 1), 64),  1,          "id 1");
	zassert_equal(fake_servo_get_u16(fake_bus_get(&bus, 2), 100), 0x0BEE,     "id 2");
	zassert_equal(fake_servo_get_u32(fake_bus_get(&bus, 3), 116), 0xDEADBEEF, "id 3");
	zassert_equal(fake_servo_get_u8 (fake_bus_get(&bus, 4), 64),  0,          "id 4");
}
```

- [ ] **Step 3: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: New test PASSES; all 22 prior + sync + bulk_read tests PASS.

- [ ] **Step 4: Commit**

```bash
git add drivers/dynamixel/dynamixel_group.c tests/drivers/dynamixel/src/test_bulk.c
git commit -m "drivers/test: dynamixel: implement BULK_WRITE with mixed-width happy path"
```

---

### Task 20: BULK_WRITE — validation and -ENOSPC

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_bulk.c`

- [ ] **Step 1: Append the tests**

```c
ZTEST(dynamixel_bulk, test_bulk_write_validation_einval)
{
	const struct dxl_bulk_write_entry req[] = {
		{ .id = 1, .item = GOAL_POSITION, .value = 0 },
	};
	const struct dxl_bulk_write_entry bad_req[] = {
		{ .id = 1, .item = (enum dxl_control)9999, .value = 0 },
	};

	zassert_equal(dxl_bulk_write(iface, req, 0),
		      -EINVAL, "n=0");
	zassert_equal(dxl_bulk_write(iface, NULL, 1),
		      -EINVAL, "NULL req");
	zassert_equal(dxl_bulk_write(iface, bad_req, 1),
		      -EINVAL, "bad item");
}

ZTEST(dynamixel_bulk, test_bulk_write_enospc_when_oversized)
{
	struct dxl_bulk_write_entry req[32];
	for (size_t i = 0; i < ARRAY_SIZE(req); i++) {
		req[i].id = (uint8_t)((i % 250) + 1);
		req[i].item = GOAL_POSITION; /* width 4 */
		req[i].value = 0;
	}

	/* BULK_WRITE packet is 10 + N*(5+L). For N=32, L=4 -> 298 > 256 default. */
	zassert_equal(dxl_bulk_write(iface, req, 32),
		      -ENOSPC, "oversized BULK_WRITE must return -ENOSPC");
}
```

- [ ] **Step 2: Run tests**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: Both new tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_bulk.c
git commit -m "test: dynamixel: cover bulk_write validation and -ENOSPC"
```

---

## Phase 7: Final regression & merge

### Task 21: Full-suite regression + clang-format check

**Why:** Catch any cumulative regressions in the existing 22 tests and fail the format gate locally before pushing (CI will run the same).

- [ ] **Step 1: Run full twister suite**

Run: `cd deps/modules/lib/rosterloh-drivers && west twister -T tests/drivers/dynamixel -T samples/drivers/dynamixel -p native_sim -v --inline-logs --integration`
Expected: All tests PASS, including the 22 existing protocol/tables/motors tests, ~14 new sync/bulk tests, and the read_write build-only sample.

- [ ] **Step 2: clang-format dry-run on changed files**

Run from the drivers repo root:

```bash
git diff --name-only main...HEAD -- '*.c' '*.h' | xargs clang-format --dry-run --Werror
```

Expected: No diagnostics. If clang-format complains, run without `--dry-run` to fix in place, then commit the result with message `style: clang-format`.

- [ ] **Step 3: Push branch and open PR**

```bash
git push -u rosterloh feature/dynamixel-sync-bulk
gh pr create --repo rosterloh/zephyr-drivers \
  --title "drivers: dynamixel: SYNC/BULK group instructions" \
  --body "$(cat <<'EOF'
## Summary

- Adds Protocol-2 SYNC_READ (0x82), SYNC_WRITE (0x83), BULK_READ (0x92), BULK_WRITE (0x93).
- Stateless C-array API; group handles deferred (see spec).
- Multi-status reads loop the existing single-frame primitive under one mutex (no changes to dynamixel_serial.c).
- DYNAMIXEL_BUFFER_SIZE max raised to 1024; -ENOSPC for oversized requests.
- New fake_bus test harness with delayed-work status injection; ~14 new tests.

Spec: docs/superpowers/specs/2026-05-05-dynamixel-sync-bulk-design.md
Plan: docs/superpowers/plans/2026-05-05-dynamixel-sync-bulk.md

## Test plan
- [ ] CI green (twister tests + samples + clang-format)
- [ ] Manual: existing motor_controller app still compiles with this branch in west.yml
EOF
)"
```

Expected: PR URL printed.

---

## Self-Review Notes

This section is for the plan author; do not modify during execution.

**Spec coverage:**
- §API surface: covered by Tasks 6 (sync_write stubs), 11 (sync_read stubs), 16 (bulk struct + stubs).
- §Implementation / RX path loop: Task 11 (sync_read) and Task 17 (bulk_read).
- §Implementation / encapsulation `parse_status_payload`: Task 1.
- §Implementation / TX builders: Tasks 7 (sync_write), 11 (sync_read), 17 (bulk_read), 19 (bulk_write).
- §Implementation / validation + ENOSPC: Tasks 9, 15, 18, 20.
- §Implementation / Kconfig: Task 2.
- §Implementation / file layout: matches §"File Structure" in this plan.
- §Test plan / fake_bus harness: Tasks 3, 4, 5, 10, 16 (incremental construction).
- §Test plan / 14 new test cases:
  - sync_write happy: Task 6.
  - sync_write widths (u8, u16): Task 8.
  - sync_write width-mismatch: Task 8.
  - sync_write -ENOSPC: Task 9.
  - sync_write validation: Task 9.
  - sync_read happy: Task 11.
  - sync_read u8/u16 happy: Task 12.
  - sync_read partial drop: Task 13.
  - sync_read device error: Task 14.
  - sync_read wrong-id: Task 14.
  - sync_read errs=NULL: Task 15.
  - sync_read validation + -ENOSPC: Task 15.
  - bulk_read happy: Task 16.
  - bulk_read validation + -ENOSPC: Task 18.
  - bulk_write happy: Task 19.
  - bulk_write validation + -ENOSPC: Task 20.
  Total: 16 new tests (slightly more than the 14 budgeted; each is small).
- §Risks: acknowledged in spec, no mitigation work in plan beyond timing constants.
- §Open questions: none in spec.

**Type / signature consistency:**
- `parse_status_payload(const uint8_t *data, uint8_t width, uint32_t *out)` — declared in Task 1, used in Tasks 11, 17.
- `dxl_sync_write_u32(int iface, enum dxl_control item, const uint8_t ids[], const uint32_t vals[], size_t n)` — declared in Task 6, implemented in Task 7, tested in Tasks 6, 8, 9.
- `dxl_sync_read_u32(int iface, enum dxl_control item, const uint8_t ids[], uint32_t vals[], int errs[], size_t n)` — declared in Task 6, implemented in Task 11, tested in Tasks 11, 13, 14, 15.
- `struct dxl_bulk_read_entry { uint8_t id; enum dxl_control item; }` — declared in Task 16, used in Tasks 16–18.
- `struct dxl_bulk_write_entry { uint8_t id; enum dxl_control item; uint32_t value; }` — declared in Task 16, used in Tasks 19–20.
- `fake_bus_init`, `fake_bus_attach`, `fake_bus_get`, `fake_bus_set_u{8,16,32}` — declared in Task 4, used in Tasks 6, 11–20.
- `fake_servo_handle_packet(struct fake_servo *, const uint8_t *, size_t)` — declared in Task 3, used by `fake_bus.c` from Task 4 onwards.
- `DXL_BULK_MAX_ENTRIES` — defined in Task 7's helper block (top of `dynamixel_group.c`), reused in Tasks 17 and 19.
