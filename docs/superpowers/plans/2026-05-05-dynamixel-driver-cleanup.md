# Dynamixel Driver Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the existing Dynamixel driver's loose, hardware-only API and stub test with a typed API, a `native_sim` test harness backed by a fake servo, and a functional DT motor metadata layer.

**Architecture:** Four sequential phases. Phase 1 builds the test harness against current code with bug-pinning tests (no behavior change). Phase 2 makes internal cleanups (no public API change). Phase 3 redesigns the public API and flips bug-pinning tests to assert correct behavior. Phase 4 adds the DT motor metadata layer and prunes the now-obsolete `protocol-version` binding.

**Tech Stack:** Zephyr RTOS, ZTest, `zephyr,uart-emul` emulated UART backend, native_sim platform, west/twister build system. Repo: `rosterloh/zephyr-drivers` (vendored at `deps/modules/lib/rosterloh-drivers` in the `zephyr-applications` workspace).

**Spec:** `docs/superpowers/specs/2026-05-05-dynamixel-driver-cleanup-design.md`

**Working directory:** All paths in this plan are relative to the repository root of `rosterloh-drivers` unless noted otherwise. When running commands, `cd` to the rosterloh-drivers repo root first.

**Test command (standard):** `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`

---

## Phase 1 — Test scaffolding (no behavior change)

End of phase: a `native_sim` ztest suite that builds and passes against the current (buggy) driver. Bug-pinning tests document existing bugs as red markers — they assert the buggy behavior so they are *currently green*; phase 3 inverts them.

### Task 1: Switch testcase target to native_sim and rewrite prj.conf

**Files:**
- Modify: `tests/drivers/dynamixel/testcase.yaml`
- Modify: `tests/drivers/dynamixel/prj.conf`
- Delete: `tests/drivers/dynamixel/boards/arduino_mkrzero.overlay`

- [ ] **Step 1: Replace testcase.yaml**

```yaml
# tests/drivers/dynamixel/testcase.yaml
tests:
  drivers.dynamixel:
    tags:
      - drivers
      - dynamixel
    harness: ztest
    platform_allow:
      - native_sim
  drivers.dynamixel.build_only:
    build_only: true
    tags:
      - drivers
      - dynamixel
    integration_platforms:
      - native_sim
```

- [ ] **Step 2: Replace prj.conf**

```conf
# tests/drivers/dynamixel/prj.conf
CONFIG_LOG=y
CONFIG_ZTEST=y
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_EMUL=y
CONFIG_UART_EMUL=y
CONFIG_DYNAMIXEL=y
CONFIG_DYNAMIXEL_LOG_LEVEL_DBG=y
```

- [ ] **Step 3: Delete the hardware overlay**

```bash
git rm tests/drivers/dynamixel/boards/arduino_mkrzero.overlay
rmdir tests/drivers/dynamixel/boards 2>/dev/null || true
```

- [ ] **Step 4: Verify nothing builds yet (no overlay, missing src)**

The native_sim build is expected to fail at this point because there is no `boards/native_sim.overlay` and the test sources still reference the deleted hardware fixture. That gets fixed in subsequent tasks. Skip running twister until task 5.

- [ ] **Step 5: Commit**

```bash
git add tests/drivers/dynamixel/testcase.yaml tests/drivers/dynamixel/prj.conf
git commit -m "tests: dynamixel: target native_sim, drop arduino_mkrzero loopback"
```

### Task 2: Add native_sim devicetree overlay

**Files:**
- Create: `tests/drivers/dynamixel/boards/native_sim.overlay`

- [ ] **Step 1: Create the overlay**

```dts
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/ {
	dxl_uart: uart-emul {
		compatible = "zephyr,uart-emul";
		current-speed = <115200>;
		tx-fifo-size = <128>;
		rx-fifo-size = <128>;
		latch-buffer-size = <128>;
		status = "okay";

		dxl_bus: dxl-bus {
			status = "okay";
			compatible = "robotis,dynamixel";

			alpha: alpha {
				id = <1>;
				label = "ALPHA";
			};

			beta: beta {
				id = <2>;
				label = "BETA";
			};
		};
	};
};
```

- [ ] **Step 2: Commit**

```bash
git add tests/drivers/dynamixel/boards/native_sim.overlay
git commit -m "tests: dynamixel: add native_sim overlay with two motor children"
```

### Task 3: Add fake_servo backend

**Files:**
- Create: `tests/drivers/dynamixel/src/fake_servo.h`
- Create: `tests/drivers/dynamixel/src/fake_servo.c`

- [ ] **Step 1: Create fake_servo.h**

```c
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
```

- [ ] **Step 2: Create fake_servo.c**

```c
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
```

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/fake_servo.h tests/drivers/dynamixel/src/fake_servo.c
git commit -m "tests: dynamixel: add fake_servo backend over uart_emul"
```

### Task 4: Replace test sources with new harness skeleton

**Files:**
- Delete: `tests/drivers/dynamixel/src/test_dynamixel.c`
- Delete: `tests/drivers/dynamixel/src/test_dynamixel.h`
- Modify: `tests/drivers/dynamixel/src/main.c`
- Modify: `tests/drivers/dynamixel/CMakeLists.txt`

- [ ] **Step 1: Delete the old stub files**

```bash
git rm tests/drivers/dynamixel/src/test_dynamixel.c tests/drivers/dynamixel/src/test_dynamixel.h
```

- [ ] **Step 2: Replace src/main.c**

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

ZTEST_SUITE(dynamixel_protocol, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(dynamixel_tables,   NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(dynamixel_motors,   NULL, NULL, NULL, NULL, NULL);
```

- [ ] **Step 3: Replace CMakeLists.txt**

```cmake
# Copyright (c) 2024 Richard Osterloh <richard.osterloh@gmail.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_dynamixel)

target_sources(app PRIVATE
    src/main.c
    src/fake_servo.c
    src/test_protocol.c
    src/test_tables.c
    src/test_motors.c
)

zephyr_include_directories(./src)
zephyr_include_directories(${ZEPHYR_DRIVERS_REPO_ROOT}/drivers/dynamixel)
```

Adding the driver directory to the include path lets test files later `#include "dynamixel_internal.h"` directly without brittle relative paths.

- [ ] **Step 4: Create empty test sources so CMake target resolves**

Create `tests/drivers/dynamixel/src/test_protocol.c`:

```c
/* Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0 */
/* Tests added in subsequent tasks. */
```

Create `tests/drivers/dynamixel/src/test_tables.c`:

```c
/* Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0 */
/* Tests added in subsequent tasks. */
```

Create `tests/drivers/dynamixel/src/test_motors.c`:

```c
/* Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0 */
/* Tests added in subsequent tasks. */
```

- [ ] **Step 5: Build and run the (empty) suite**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: build succeeds, three test suites register with zero tests, twister reports success.

**Known build hurdle:** `zephyr,uart-emul`'s `DEFINE_UART_EMUL` macro iterates every `status = "okay"` child via `DT_FOREACH_CHILD_STATUS_OKAY` and calls `DEVICE_DT_GET(node_id)` on each. The Dynamixel driver historically did not register a Zephyr device, so `DEVICE_DT_GET` for the dxl-bus child node resolved to an undefined `__device_dts_ord_N` symbol → linker error. Fix landed as a follow-up commit on top of task 4: append a no-op `DEVICE_DT_INST_DEFINE` to `drivers/dynamixel/dynamixel.c`:

```c
#define DXL_DEVICE_DEFINE(inst)                                                                    \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, NULL, NULL, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(DXL_DEVICE_DEFINE)
```

The driver still doesn't use the device API (clients keep calling `dxl_iface_get_by_name`); the device object is purely a linker artifact so `DEVICE_DT_GET` resolves. This change is binding-and-overlay-neutral, so the consumer's overlay (`applications/motor_controller/boards/robotis_openrb_150.overlay`) keeps working.

- [ ] **Step 6: Commit**

```bash
git add tests/drivers/dynamixel/src/main.c tests/drivers/dynamixel/CMakeLists.txt \
        tests/drivers/dynamixel/src/test_protocol.c \
        tests/drivers/dynamixel/src/test_tables.c \
        tests/drivers/dynamixel/src/test_motors.c
git commit -m "tests: dynamixel: replace stub harness with native_sim suites"
```

### Task 5: Bug-pinning protocol tests (against current driver)

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_protocol.c`

These tests document current (buggy) behavior. They are *currently green*. Phase 3 flips them to assert correct behavior — those flips are the visible proof that the rewrite worked.

- [ ] **Step 1: Write the test file**

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phase 1 BUG-PINNING TESTS. These intentionally assert current
 * (buggy) behavior so they are green now. Phase 3 of the cleanup
 * flips them.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/dynamixel.h>

#include "fake_servo.h"

#define DXL_BUS_NODE      DT_NODELABEL(dxl_bus)
#define DXL_UART_NODE     DT_PARENT(DXL_BUS_NODE)
#define DXL_IFACE_NAME    DEVICE_DT_NAME(DXL_BUS_NODE)

static const struct device *uart_dev = DEVICE_DT_GET(DXL_UART_NODE);

static struct fake_servo srv;
static int iface;

static void bring_up(uint8_t id)
{
	struct dxl_iface_param p = {
		.rx_timeout = 50000,
		.serial = { .baud = 115200, .parity = UART_CFG_PARITY_NONE },
	};

	fake_servo_init(&srv, id);
	fake_servo_attach(&srv, uart_dev);

	iface = dxl_iface_get_by_name(DXL_IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(dxl_init(iface, p), "dxl_init failed");
}

static void tear_down(void)
{
	dxl_disable(iface);
}

ZTEST(dynamixel_protocol, test_phase1_ping_request_bytes)
{
	bring_up(1);

	zassert_ok(dxl_ping(iface, 1), "ping failed");

	zassert_equal(srv.last_tx[0], 0xFF, "header byte 0");
	zassert_equal(srv.last_tx[1], 0xFF, "header byte 1");
	zassert_equal(srv.last_tx[2], 0xFD, "header byte 2");
	zassert_equal(srv.last_tx[3], 0x00, "reserved");
	zassert_equal(srv.last_tx[4], 1,    "id");
	zassert_equal(srv.last_tx[7], 0x01, "ping instruction");
	zassert_equal(srv.last_instruction, 0x01, "captured instruction");

	tear_down();
}

ZTEST(dynamixel_protocol, test_phase1_read_u32_truncated)
{
	uint32_t val32 = 0;

	bring_up(1);
	fake_servo_set_u32(&srv, 132 /* PRESENT_POSITION addr */, 0x12345678);

	(void)dxl_read(iface, 1, PRESENT_POSITION, &val32);

	/* BUG: existing code uses sys_get_le16 even for 4-byte regs. */
	zassert_equal(val32, 0x00005678,
		      "phase 1 expects truncated low 16 bits, got 0x%08x", val32);

	tear_down();
}

ZTEST(dynamixel_protocol, test_phase1_timeout_overwrites_err)
{
	uint32_t val32 = 0;
	int rc;

	bring_up(1);
	srv.drop_response = true;

	rc = dxl_read(iface, 1, PRESENT_POSITION, &val32);

	/* BUG: dxl_read overwrites the timeout return code with rx_frame.data[0],
	 * which on timeout is uninitialised / leftover. We assert that rc is NOT
	 * -ETIMEDOUT — the right value the API ought to return — to make the bug
	 * visible. Phase 3 inverts this: rc MUST be -ETIMEDOUT.
	 */
	zassert_not_equal(rc, -ETIMEDOUT,
			  "phase 1 expects err to be overwritten, not -ETIMEDOUT");

	tear_down();
}

ZTEST(dynamixel_protocol, test_phase1_write_u8_round_trip)
{
	bring_up(1);

	zassert_ok(dxl_write(iface, 1, TORQUE_ENABLE, 1), "write failed");

	zassert_equal(srv.last_instruction, 0x03, "write instruction");
	zassert_equal(srv.last_addr,        64,   "TORQUE_ENABLE addr");
	zassert_equal(srv.last_length,      1,    "1-byte param");
	zassert_equal(fake_servo_get_u8(&srv, 64), 1, "RAM updated");

	tear_down();
}
```

- [ ] **Step 2: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS — all four tests green against current driver.

If `test_phase1_read_u32_truncated` fails (i.e. the driver actually returns the full value), that means the bug has already been fixed elsewhere; in that case, strip this test and add a phase-3 green test instead. Same applies to `test_phase1_timeout_overwrites_err`: if it fails, the error-overwrite bug is gone — drop the test and proceed.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_protocol.c
git commit -m "tests: dynamixel: pin existing protocol bugs with red-marker tests"
```

### Task 6: Tables sanity tests (against current header-based table)

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_tables.c`

- [ ] **Step 1: Write the test file**

```c
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
```

- [ ] **Step 2: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_tables.c
git commit -m "tests: dynamixel: assert known control-table register layout"
```

### Task 7: Motor metadata tests (skeleton — assert nothing is wired yet)

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_motors.c`

- [ ] **Step 1: Write the placeholder test**

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phase 4 wires up dxl_motor_count / dxl_motor_get_by_label and
 * replaces the body of this test. For now it just asserts the suite
 * runs.
 */

#include <zephyr/ztest.h>

ZTEST(dynamixel_motors, test_phase1_suite_runs)
{
	zassert_true(true, "placeholder until phase 4");
}
```

- [ ] **Step 2: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS — all three suites green.

- [ ] **Step 3: Commit**

```bash
git add tests/drivers/dynamixel/src/test_motors.c
git commit -m "tests: dynamixel: placeholder motor suite"
```

End-of-Phase-1 gate: All tests green on `native_sim`.

---

## Phase 2 — Internal cleanups (no public API change)

End of phase: tables live in their own translation unit, internal cleanups done, public header is internally tidier but functions still take `void *` / `uint32_t`. Tests still green.

### Task 8: Add dynamixel_tables.c with internal lookup; keep header table

**Files:**
- Create: `drivers/dynamixel/dynamixel_tables.c`
- Modify: `drivers/dynamixel/dynamixel_internal.h`
- Modify: `drivers/dynamixel/CMakeLists.txt`

- [ ] **Step 1: Create the tables source**

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/util.h>

#include "dynamixel_internal.h"

/* Single source of truth for the X-series control table.
 * Addresses and lengths from the XL330 / XC330 e-Manual.
 */
static const struct dxl_control_info table[] = {
	[MODEL_NUMBER]            = { 0,   2 },
	[MODEL_INFORMATION]       = { 2,   4 },
	[FIRMWARE_VERSION]        = { 6,   1 },
	[ID]                      = { 7,   1 },
	[BAUD_RATE]               = { 8,   1 },
	[RETURN_DELAY_TIME]       = { 9,   1 },
	[DRIVE_MODE]              = { 10,  1 },
	[OPERATING_MODE]          = { 11,  1 },
	[SECONDARY_ID]            = { 12,  1 },
	[PROTOCOL_VERSION]        = { 13,  1 },
	[HOMING_OFFSET]           = { 20,  4 },
	[MOVING_THRESHOLD]        = { 24,  4 },
	[TEMPERATURE_LIMIT]       = { 31,  1 },
	[MAX_VOLTAGE_LIMIT]       = { 32,  2 },
	[MIN_VOLTAGE_LIMIT]       = { 34,  2 },
	[PWM_LIMIT]               = { 36,  2 },
	[CURRENT_LIMIT]           = { 38,  2 },
	[VELOCITY_LIMIT]          = { 44,  4 },
	[MAX_POSITION_LIMIT]      = { 48,  4 },
	[MIN_POSITION_LIMIT]      = { 52,  4 },
	[STARTUP_CONFIGURATION]   = { 60,  1 },
	[PWM_SLOPE]               = { 62,  1 },
	[SHUTDOWN]                = { 63,  1 },

	[TORQUE_ENABLE]           = { 64,  1 },
	[LED]                     = { 65,  1 },
	[STATUS_RETURN_LEVEL]     = { 68,  1 },
	[REGISTERED_INSTRUCTION]  = { 69,  1 },
	[HARDWARE_ERROR_STATUS]   = { 70,  1 },
	[VELOCITY_I_GAIN]         = { 76,  2 },
	[VELOCITY_P_GAIN]         = { 78,  2 },
	[POSITION_D_GAIN]         = { 80,  2 },
	[POSITION_I_GAIN]         = { 82,  2 },
	[POSITION_P_GAIN]         = { 84,  2 },
	[FEEDFORWARD_2ND_GAIN]    = { 88,  2 },
	[FEEDFORWARD_1ST_GAIN]    = { 90,  2 },
	[BUS_WATCHDOG]            = { 98,  1 },
	[GOAL_PWM]                = { 100, 2 },
	[GOAL_CURRENT]            = { 102, 2 },
	[GOAL_VELOCITY]           = { 104, 4 },
	[PROFILE_ACCELERATION]    = { 108, 4 },
	[PROFILE_VELOCITY]        = { 112, 4 },
	[GOAL_POSITION]           = { 116, 4 },
	[REALTIME_TICK]           = { 120, 2 },
	[MOVING]                  = { 122, 1 },
	[MOVING_STATUS]           = { 123, 1 },
	[PRESENT_PWM]             = { 124, 2 },
	[PRESENT_CURRENT]         = { 126, 2 },
	[PRESENT_VELOCITY]        = { 128, 4 },
	[PRESENT_POSITION]        = { 132, 4 },
	[VELOCITY_TRAJECTORY]     = { 136, 4 },
	[POSITION_TRAJECTORY]     = { 140, 4 },
	[PRESENT_INPUT_VOLTAGE]   = { 144, 2 },
	[PRESENT_TEMPERATURE]     = { 146, 1 },
};

int dxl_table_lookup(enum dxl_control item, uint16_t *addr, uint8_t *length)
{
	if ((unsigned)item >= ARRAY_SIZE(table)) {
		return -EINVAL;
	}
	if (table[item].length == 0) {
		return -EINVAL;
	}
	if (addr) {
		*addr = table[item].address;
	}
	if (length) {
		*length = table[item].length;
	}
	return 0;
}

const struct dxl_model_info dxl_info_x330 = {
	0.229f, 0, 2048, 4096, -3.14159265f, 3.14159265f
};
```

- [ ] **Step 2: Add the prototype to dynamixel_internal.h**

Append before the final `#endif` of `drivers/dynamixel/dynamixel_internal.h`:

```c
/**
 * @brief Look up a control register's address and length.
 *
 * @param item    Control register identifier.
 * @param addr    Optional out: register address.
 * @param length  Optional out: register length in bytes.
 *
 * @retval 0 on success
 * @retval -EINVAL if item is out of range or has no entry
 */
int dxl_table_lookup(enum dxl_control item, uint16_t *addr, uint8_t *length);

/* Model info (currently unused by driver code; available for unit conversions). */
extern const struct dxl_model_info dxl_info_x330;
```

- [ ] **Step 3: Update CMakeLists.txt**

Replace `drivers/dynamixel/CMakeLists.txt` with:

```cmake
zephyr_library()

zephyr_library_sources(
  dynamixel.c
  dynamixel_serial.c
  dynamixel_protocol.c
  dynamixel_tables.c
)
```

- [ ] **Step 4: Run tests — both header table and new lookup must coexist**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS. The header still publishes the original `control_table[]`; the new file adds a parallel internal table and lookup function. Both compile (no naming clash — the file-static is named `table`, not `control_table`).

- [ ] **Step 5: Commit**

```bash
git add drivers/dynamixel/dynamixel_tables.c drivers/dynamixel/dynamixel_internal.h \
        drivers/dynamixel/CMakeLists.txt
git commit -m "drivers: dynamixel: add internal table-lookup module"
```

### Task 9: Add dxl_table_lookup test, then route protocol.c through it

**Files:**
- Modify: `tests/drivers/dynamixel/src/test_tables.c`
- Modify: `drivers/dynamixel/dynamixel_protocol.c`
- Modify: `include/drivers/dynamixel.h`

- [ ] **Step 1: Add a test that exercises dxl_table_lookup directly**

Append to `tests/drivers/dynamixel/src/test_tables.c`:

```c
#include <errno.h>
#include "dynamixel_internal.h"

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
```

`#include "dynamixel_internal.h"` resolves via the `zephyr_include_directories(${ZEPHYR_DRIVERS_REPO_ROOT}/drivers/dynamixel)` line added to the test CMakeLists.txt in task 4.

- [ ] **Step 2: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS for new lookup tests, all phase-1 tests still green.

- [ ] **Step 3: Switch dynamixel_protocol.c to use the lookup**

Replace the body of `drivers/dynamixel/dynamixel_protocol.c` with:

```c
#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#ping-0x01 */
int dxl_ping(const int iface, const uint8_t id)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 3;
	ctx->tx_frame.ic = DXL_INST_PING;

	err = dxl_tx_wait_rx(ctx);

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#read-0x02 */
int dxl_read(const int iface, const uint8_t id, uint8_t item_idx, void *data)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	uint16_t addr;
	uint8_t  length;
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup((enum dxl_control)item_idx, &addr, &length) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 7;
	ctx->tx_frame.ic = DXL_INST_READ;
	sys_put_le16(addr,   &ctx->tx_frame.data[0]);
	sys_put_le16(length, &ctx->tx_frame.data[2]);

	err = dxl_tx_wait_rx(ctx);

	/* PRESERVE the existing buggy behavior for now: phase 3 fixes this. */
	err = ctx->rx_frame.data[0];
	if (length == 1) {
		*(uint8_t *)data = ctx->rx_frame.data[1];
	} else if (length == 2) {
		*(uint16_t *)data = sys_get_le16(&ctx->rx_frame.data[1]);
	} else {
		*(uint32_t *)data = sys_get_le16(&ctx->rx_frame.data[1]);
	}

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#write-0x03 */
int dxl_write(const int iface, const uint8_t id, uint8_t item_idx, uint32_t data)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	uint16_t addr;
	uint8_t  length;
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup((enum dxl_control)item_idx, &addr, &length) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 5 + length;
	ctx->tx_frame.ic = DXL_INST_WRITE;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	if (length == 1) {
		ctx->tx_frame.data[2] = data & 0xFF;
	} else if (length == 2) {
		sys_put_le16(data & 0xFFFF, &ctx->tx_frame.data[2]);
	} else {
		sys_put_le32(data, &ctx->tx_frame.data[2]);
	}

	err = dxl_tx_wait_rx(ctx);

	/* PRESERVE the existing buggy behavior for now: phase 3 fixes this. */
	err = ctx->rx_frame.data[0];

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#reboot-0x08 */
int dxl_reboot(const int iface, const uint8_t id)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->tx_frame.id = id;
	ctx->tx_frame.length = 3;
	ctx->tx_frame.ic = DXL_INST_REBOOT;

	err = dxl_tx_wait_rx(ctx);

	k_mutex_unlock(&ctx->iface_lock);

	return err;
}
```

This routes everything through `dxl_table_lookup`. The buggy `sys_get_le16` for 4-byte values stays — it's still phase 1 / 2 and we're not changing behavior yet. The 1-byte write goes to a plain `data[2] = data & 0xFF` (a "smaller things" item, also no behavior change).

- [ ] **Step 4: Build and run all tests**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS for everything, including the bug-pinning tests (which still see the same buggy behavior).

- [ ] **Step 5: Commit**

```bash
git add drivers/dynamixel/dynamixel_protocol.c tests/drivers/dynamixel/src/test_tables.c
git commit -m "drivers: dynamixel: route protocol.c through dxl_table_lookup"
```

### Task 10: Drop static const arrays from public header

**Files:**
- Modify: `include/drivers/dynamixel.h`

- [ ] **Step 1: Edit the header**

In `include/drivers/dynamixel.h`:
- Delete the `static const struct dxl_control_info control_table[] = { ... };` block.
- Delete the `static const struct dxl_model_info info_x330 = { ... };` line.
- Keep the `struct dxl_control_info` and `struct dxl_model_info` type definitions where they are (the test file and `dynamixel_tables.c` both need the struct definitions).

The test_tables.c assertions written in task 6 reference `control_table[...]`. Those need to flip to `dxl_table_lookup` calls now.

- [ ] **Step 2: Update tests/drivers/dynamixel/src/test_tables.c**

Replace `test_known_register_addresses` with the lookup-based version:

```c
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
```

- [ ] **Step 3: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add include/drivers/dynamixel.h tests/drivers/dynamixel/src/test_tables.c
git commit -m "drivers: dynamixel: drop static const arrays from public header"
```

### Task 11: Drop dead struct dxl_frame::header field

**Files:**
- Modify: `include/drivers/dynamixel.h`

- [ ] **Step 1: Confirm no consumer reads header**

```bash
rg -n 'dxl_frame.*header|tx_frame\.header|rx_frame\.header' include drivers tests
```

Expected: no results (the header bytes are written from a literal in `tx_frame()`).

- [ ] **Step 2: Delete the field**

In `include/drivers/dynamixel.h`, in `struct dxl_frame`, delete:

```c
	/** Packet header */
	uint32_t header;
```

- [ ] **Step 3: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add include/drivers/dynamixel.h
git commit -m "drivers: dynamixel: drop dead dxl_frame::header field"
```

### Task 12: Normalize dxl_disable signature to int iface

**Files:**
- Modify: `include/drivers/dynamixel.h`
- Modify: `drivers/dynamixel/dynamixel.c`

- [ ] **Step 1: Update the prototype**

In `include/drivers/dynamixel.h`, change:

```c
int dxl_disable(const uint8_t iface);
```

to:

```c
int dxl_disable(int iface);
```

- [ ] **Step 2: Update the definition**

In `drivers/dynamixel/dynamixel.c`, change:

```c
int dxl_disable(const uint8_t iface)
```

to:

```c
int dxl_disable(int iface)
```

The body uses `iface` only as `dxl_get_context(iface)`, which already takes a `uint8_t`. Add an early `if (iface < 0) return -EINVAL;` guard at the top of `dxl_disable`:

```c
int dxl_disable(int iface)
{
	struct dxl_context *ctx;
	struct k_work_sync work_sync;

	if (iface < 0) {
		return -EINVAL;
	}

	ctx = dxl_get_context((uint8_t)iface);
	/* ...rest unchanged... */
}
```

- [ ] **Step 3: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add include/drivers/dynamixel.h drivers/dynamixel/dynamixel.c
git commit -m "drivers: dynamixel: normalize dxl_disable signature to int iface"
```

End-of-Phase-2 gate: All tests green. Header is internally tidier. Public function shapes for `dxl_read` / `dxl_write` are unchanged.

---

## Phase 3 — API redesign (breaking change)

End of phase: typed `dxl_read_u*` / `dxl_write_u*` functions exist, return-code contract is the new `<0 / 0 / >0` shape, response-ID match is enforced, the bug-pinning tests are flipped to assert correct behavior, and `motor_controller/main.c` in the consumer repo has been updated.

### Task 13: Add response-ID match check (no API change)

**Files:**
- Modify: `drivers/dynamixel/dynamixel_internal.h`
- Modify: `drivers/dynamixel/dynamixel_protocol.c`
- Modify: `tests/drivers/dynamixel/src/test_protocol.c`

- [ ] **Step 1: Write a failing test**

Append to `tests/drivers/dynamixel/src/test_protocol.c`:

```c
ZTEST(dynamixel_protocol, test_phase3_wrong_id_response_rejected)
{
	int rc;

	bring_up(2);              /* fake servo answers as id=2 */

	rc = dxl_ping(iface, 1);  /* but we ping id=1 */

	zassert_equal(rc, -ETIMEDOUT, "wrong-id reply should look like a timeout, got %d", rc);

	tear_down();
}
```

- [ ] **Step 2: Run — should FAIL**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: `test_phase3_wrong_id_response_rejected` FAILS, because the driver currently accepts any response.

- [ ] **Step 3: Add the requested-ID field and check**

In `drivers/dynamixel/dynamixel_internal.h`, add to `struct dxl_context`:

```c
	/* ID of the device the current request is addressed to */
	uint8_t expected_id;
```

In `drivers/dynamixel/dynamixel_protocol.c`, before each `dxl_tx_wait_rx(ctx)` call set `ctx->expected_id = id;` (in `dxl_ping`, `dxl_read`, `dxl_write`, `dxl_reboot`).

In `drivers/dynamixel/dynamixel_serial.c`, in `rx_frame()`, after parsing the ID and before the CRC check, add:

```c
	if (ctx->rx_frame.id != ctx->expected_id) {
		LOG_WRN("Response ID %u != expected %u",
			ctx->rx_frame.id, ctx->expected_id);
		return -EIO;
	}
```

This makes mismatched-ID responses surface as `-EIO` from `dxl_serial_rx`, which becomes the return value of `dxl_tx_wait_rx`, which we'll soon stop overwriting (next task). For *this* task, with the existing code that overwrites `err = ctx->rx_frame.data[0]`, the result will instead be a timeout (the wrong-ID packet is dropped, no further packets arrive, the wait_sem semaphore times out). That matches the test's `-ETIMEDOUT` expectation.

- [ ] **Step 4: Run — should PASS**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS, including the new wrong-id test.

- [ ] **Step 5: Commit**

```bash
git add drivers/dynamixel/dynamixel_internal.h drivers/dynamixel/dynamixel_protocol.c \
        drivers/dynamixel/dynamixel_serial.c tests/drivers/dynamixel/src/test_protocol.c
git commit -m "drivers: dynamixel: reject wrong-id responses in rx path"
```

### Task 14: Fix error-overwrite, then flip phase-1 timeout test

**Files:**
- Modify: `drivers/dynamixel/dynamixel_protocol.c`
- Modify: `tests/drivers/dynamixel/src/test_protocol.c`

- [ ] **Step 1: Flip the timeout test to assert correct behavior**

In `tests/drivers/dynamixel/src/test_protocol.c`, replace `test_phase1_timeout_overwrites_err` with:

```c
ZTEST(dynamixel_protocol, test_timeout_returns_etimedout)
{
	uint32_t val32 = 0;
	int rc;

	bring_up(1);
	srv.drop_response = true;

	rc = dxl_read(iface, 1, PRESENT_POSITION, &val32);

	zassert_equal(rc, -ETIMEDOUT,
		      "timeout must return -ETIMEDOUT, got %d", rc);

	tear_down();
}
```

- [ ] **Step 2: Run — should FAIL**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: `test_timeout_returns_etimedout` FAILS (because of the `err = ctx->rx_frame.data[0]` overwrite).

- [ ] **Step 3: Fix dxl_read and dxl_write**

In `drivers/dynamixel/dynamixel_protocol.c`, change `dxl_read`'s body so the device-error byte is only consulted on success:

```c
	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		err = ctx->rx_frame.data[0];
		if (length == 1) {
			*(uint8_t *)data = ctx->rx_frame.data[1];
		} else if (length == 2) {
			*(uint16_t *)data = sys_get_le16(&ctx->rx_frame.data[1]);
		} else {
			*(uint32_t *)data = sys_get_le16(&ctx->rx_frame.data[1]);
		}
	}
```

And the same for `dxl_write`:

```c
	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		err = ctx->rx_frame.data[0];
	}
```

The 32-bit truncation is *still there* (deliberate — phase 1's `test_phase1_read_u32_truncated` is still asserting truncation). That gets fixed in the typed-API task.

- [ ] **Step 4: Run — should PASS**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS, including the flipped timeout test.

- [ ] **Step 5: Commit**

```bash
git add drivers/dynamixel/dynamixel_protocol.c tests/drivers/dynamixel/src/test_protocol.c
git commit -m "drivers: dynamixel: stop overwriting transport err with device byte"
```

### Task 15: Add typed read/write API and width validation

**Files:**
- Modify: `include/drivers/dynamixel.h`
- Modify: `drivers/dynamixel/dynamixel_protocol.c`

- [ ] **Step 1: Write tests for the new API (before it exists)**

Append to `tests/drivers/dynamixel/src/test_protocol.c`:

```c
ZTEST(dynamixel_protocol, test_read_u32_full_value)
{
	uint32_t val = 0;

	bring_up(1);
	fake_servo_set_u32(&srv, 132 /* PRESENT_POSITION addr */, 0x12345678);

	zassert_ok(dxl_read_u32(iface, 1, PRESENT_POSITION, &val), "read failed");
	zassert_equal(val, 0x12345678, "got 0x%08x", val);

	tear_down();
}

ZTEST(dynamixel_protocol, test_read_u16_full_value)
{
	uint16_t val = 0;

	bring_up(1);
	fake_servo_set_u16(&srv, 100 /* GOAL_PWM addr */, 0xCAFE);

	zassert_ok(dxl_read_u16(iface, 1, GOAL_PWM, &val), "read failed");
	zassert_equal(val, 0xCAFE, "got 0x%04x", val);

	tear_down();
}

ZTEST(dynamixel_protocol, test_read_u8_full_value)
{
	uint8_t val = 0;

	bring_up(1);
	fake_servo_set_u8(&srv, 64 /* TORQUE_ENABLE addr */, 0xA5);

	zassert_ok(dxl_read_u8(iface, 1, TORQUE_ENABLE, &val), "read failed");
	zassert_equal(val, 0xA5, "got 0x%02x", val);

	tear_down();
}

ZTEST(dynamixel_protocol, test_write_u32_round_trip)
{
	bring_up(1);

	zassert_ok(dxl_write_u32(iface, 1, GOAL_POSITION, 0xDEADBEEF), "write failed");

	zassert_equal(srv.last_addr,   116, "GOAL_POSITION addr");
	zassert_equal(srv.last_length, 4,   "4-byte param");
	zassert_equal(fake_servo_get_u32(&srv, 116), 0xDEADBEEF, "RAM");

	tear_down();
}

ZTEST(dynamixel_protocol, test_width_mismatch_returns_einval)
{
	uint8_t val8;

	bring_up(1);

	zassert_equal(dxl_read_u8(iface, 1, PRESENT_POSITION, &val8),
		      -EINVAL, "u8 read of 4-byte register");

	zassert_equal(dxl_write_u16(iface, 1, TORQUE_ENABLE, 0),
		      -EINVAL, "u16 write of 1-byte register");

	tear_down();
}

ZTEST(dynamixel_protocol, test_invalid_register_returns_einval)
{
	uint32_t val32;

	bring_up(1);

	zassert_equal(dxl_read_u32(iface, 1, (enum dxl_control)9999, &val32),
		      -EINVAL, "out-of-range register");

	tear_down();
}

ZTEST(dynamixel_protocol, test_device_error_byte_returned_positive)
{
	uint32_t val = 0;
	int rc;

	bring_up(1);
	srv.error_byte = DXL_ERR_DATA_RANGE;
	fake_servo_set_u32(&srv, 132, 0x11);

	rc = dxl_read_u32(iface, 1, PRESENT_POSITION, &val);

	zassert_equal(rc, DXL_ERR_DATA_RANGE,
		      "device error must be returned as positive enum, got %d", rc);

	tear_down();
}
```

- [ ] **Step 2: Run — should FAIL (functions undefined)**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: build FAILS with "implicit declaration of function 'dxl_read_u32'" etc.

- [ ] **Step 3: Add the typed prototypes to the public header**

In `include/drivers/dynamixel.h`, after the existing `dxl_write` declaration, add:

```c
/**
 * @brief Read an 8-bit register.
 *
 * @param iface Dynamixel interface index.
 * @param id    Bus ID of the device.
 * @param item  Control register identifier.
 * @param out   Output pointer.
 *
 * @retval 0 on success, with device-success.
 * @retval >0 enum dxl_error from the device's status packet.
 * @retval <0 errno: -ETIMEDOUT, -EIO, -ENODEV, -EINVAL.
 */
int dxl_read_u8 (int iface, uint8_t id, enum dxl_control item, uint8_t  *out);
int dxl_read_u16(int iface, uint8_t id, enum dxl_control item, uint16_t *out);
int dxl_read_u32(int iface, uint8_t id, enum dxl_control item, uint32_t *out);

int dxl_write_u8 (int iface, uint8_t id, enum dxl_control item, uint8_t  val);
int dxl_write_u16(int iface, uint8_t id, enum dxl_control item, uint16_t val);
int dxl_write_u32(int iface, uint8_t id, enum dxl_control item, uint32_t val);
```

- [ ] **Step 4: Implement the typed functions in dynamixel_protocol.c**

Add to the bottom of `drivers/dynamixel/dynamixel_protocol.c`:

```c
static int dxl_read_n(int iface, uint8_t id, enum dxl_control item,
		      uint8_t expected_width, uint32_t *out)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	uint16_t addr;
	uint8_t  length;
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (out == NULL) {
		return -EINVAL;
	}
	if (dxl_table_lookup(item, &addr, &length) != 0) {
		return -EINVAL;
	}
	if (length != expected_width) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->expected_id     = id;
	ctx->tx_frame.id     = id;
	ctx->tx_frame.length = 7;
	ctx->tx_frame.ic     = DXL_INST_READ;
	sys_put_le16(addr,   &ctx->tx_frame.data[0]);
	sys_put_le16(length, &ctx->tx_frame.data[2]);

	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		uint8_t dev_err = ctx->rx_frame.data[0];
		if (dev_err == 0) {
			switch (length) {
			case 1: *out = ctx->rx_frame.data[1]; break;
			case 2: *out = sys_get_le16(&ctx->rx_frame.data[1]); break;
			case 4: *out = sys_get_le32(&ctx->rx_frame.data[1]); break;
			default:
				err = -EINVAL;
				break;
			}
		} else {
			err = (int)dev_err;
		}
	}

	k_mutex_unlock(&ctx->iface_lock);
	return err;
}

int dxl_read_u8(int iface, uint8_t id, enum dxl_control item, uint8_t *out)
{
	uint32_t v = 0;
	int rc = dxl_read_n(iface, id, item, 1, &v);
	if (rc == 0) {
		*out = (uint8_t)v;
	}
	return rc;
}

int dxl_read_u16(int iface, uint8_t id, enum dxl_control item, uint16_t *out)
{
	uint32_t v = 0;
	int rc = dxl_read_n(iface, id, item, 2, &v);
	if (rc == 0) {
		*out = (uint16_t)v;
	}
	return rc;
}

int dxl_read_u32(int iface, uint8_t id, enum dxl_control item, uint32_t *out)
{
	return dxl_read_n(iface, id, item, 4, out);
}

static int dxl_write_n(int iface, uint8_t id, enum dxl_control item,
		       uint8_t expected_width, uint32_t value)
{
	struct dxl_context *ctx = dxl_get_context(iface);
	uint16_t addr;
	uint8_t  length;
	int err;

	if (ctx == NULL) {
		return -ENODEV;
	}
	if (dxl_table_lookup(item, &addr, &length) != 0) {
		return -EINVAL;
	}
	if (length != expected_width) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);

	ctx->expected_id     = id;
	ctx->tx_frame.id     = id;
	ctx->tx_frame.length = 5 + length;
	ctx->tx_frame.ic     = DXL_INST_WRITE;
	sys_put_le16(addr, &ctx->tx_frame.data[0]);
	switch (length) {
	case 1:
		ctx->tx_frame.data[2] = (uint8_t)value;
		break;
	case 2:
		sys_put_le16((uint16_t)value, &ctx->tx_frame.data[2]);
		break;
	case 4:
		sys_put_le32(value, &ctx->tx_frame.data[2]);
		break;
	default:
		k_mutex_unlock(&ctx->iface_lock);
		return -EINVAL;
	}

	err = dxl_tx_wait_rx(ctx);
	if (err == 0) {
		uint8_t dev_err = ctx->rx_frame.data[0];
		err = (dev_err == 0) ? 0 : (int)dev_err;
	}

	k_mutex_unlock(&ctx->iface_lock);
	return err;
}

int dxl_write_u8 (int iface, uint8_t id, enum dxl_control item, uint8_t val)
{ return dxl_write_n(iface, id, item, 1, val); }

int dxl_write_u16(int iface, uint8_t id, enum dxl_control item, uint16_t val)
{ return dxl_write_n(iface, id, item, 2, val); }

int dxl_write_u32(int iface, uint8_t id, enum dxl_control item, uint32_t val)
{ return dxl_write_n(iface, id, item, 4, val); }
```

- [ ] **Step 5: Run — should PASS**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS for all new typed-API tests. The phase-1 truncation test (`test_phase1_read_u32_truncated`) is still green because it still uses the old `dxl_read` which still has the bug.

- [ ] **Step 6: Commit**

```bash
git add include/drivers/dynamixel.h drivers/dynamixel/dynamixel_protocol.c \
        tests/drivers/dynamixel/src/test_protocol.c
git commit -m "drivers: dynamixel: add typed read_u8/16/32 and write_u8/16/32 API"
```

### Task 16: Remove old dxl_read / dxl_write API

**Files:**
- Modify: `include/drivers/dynamixel.h`
- Modify: `drivers/dynamixel/dynamixel_protocol.c`
- Modify: `tests/drivers/dynamixel/src/test_protocol.c`

- [ ] **Step 1: Delete the phase-1 bug-pinning tests that depend on old API**

In `tests/drivers/dynamixel/src/test_protocol.c`, delete:
- `test_phase1_read_u32_truncated`
- `test_phase1_write_u8_round_trip` (its assertions are subsumed by `test_write_u32_round_trip` and similar; if you want a u8 round-trip, add one against the new API).

Replace `test_phase1_write_u8_round_trip` with the typed-API equivalent:

```c
ZTEST(dynamixel_protocol, test_write_u8_round_trip)
{
	bring_up(1);

	zassert_ok(dxl_write_u8(iface, 1, TORQUE_ENABLE, 1), "write failed");

	zassert_equal(srv.last_instruction, 0x03, "write instruction");
	zassert_equal(srv.last_addr,        64,   "TORQUE_ENABLE addr");
	zassert_equal(srv.last_length,      1,    "1-byte param");
	zassert_equal(fake_servo_get_u8(&srv, 64), 1, "RAM updated");

	tear_down();
}
```

Keep `test_phase1_ping_request_bytes` — `dxl_ping` is unchanged.

- [ ] **Step 2: Remove old declarations from the public header**

In `include/drivers/dynamixel.h`, delete:

```c
int dxl_read(const int iface, const uint8_t id, uint8_t item_idx, void *data);
int dxl_write(const int iface, const uint8_t id, uint8_t item_idx, uint32_t data);
```

Also delete the `typedef int (*dxl_raw_cb_t)(...)` and the `struct dxl_motor_config` declaration (the latter is replaced by `struct dxl_motor` in phase 4 — confirm via grep that no current code uses `dxl_motor_config`):

```bash
rg -n 'dxl_motor_config|dxl_raw_cb_t' include drivers tests
```

Expected: no results outside of the header itself.

- [ ] **Step 3: Remove old definitions from dynamixel_protocol.c**

Delete the bodies of `dxl_read` and `dxl_write` (the original ones, not `dxl_read_n` / `dxl_write_n`).

- [ ] **Step 4: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS. If anything fails to compile because old `dxl_read` / `dxl_write` is referenced from test code, fix the test references (none should remain at this point).

- [ ] **Step 5: Commit**

```bash
git add include/drivers/dynamixel.h drivers/dynamixel/dynamixel_protocol.c \
        tests/drivers/dynamixel/src/test_protocol.c
git commit -m "drivers: dynamixel: remove untyped dxl_read/dxl_write and unused decls"
```

### Task 17: Update motor_controller consumer (cross-repo)

**Files:**
- Modify: `applications/motor_controller/src/main.c` (in the `zephyr-applications` repo, not this one)

This task happens in a *different* git repo: `zephyr-applications`, located at `/home/rio/src/github/rosterloh/zephyr-applications` from this workspace. Coordinate via west manifest revision bump after this drivers branch lands.

- [ ] **Step 1: cd to the consumer repo**

```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
```

- [ ] **Step 2: Edit applications/motor_controller/src/main.c**

Three call-sites change. Find:

```c
err = dxl_write(iface, motor_id, GOAL_POSITION, slide_val * 4);
```

Replace with:

```c
err = dxl_write_u32(iface, motor_id, GOAL_POSITION, slide_val * 4);
```

Find:

```c
ret = dxl_write(iface, motor_id, TORQUE_ENABLE, 0);
ret |= dxl_write(iface, motor_id, OPERATING_MODE, DXL_OP_POSITION);
ret |= dxl_write(iface, motor_id, TORQUE_ENABLE, 1);
```

Replace with:

```c
ret = dxl_write_u8(iface, motor_id, TORQUE_ENABLE, 0);
ret |= dxl_write_u8(iface, motor_id, OPERATING_MODE, DXL_OP_POSITION);
ret |= dxl_write_u8(iface, motor_id, TORQUE_ENABLE, 1);
```

No other call-sites in the consumer use the old API (verified by `rg`).

- [ ] **Step 3: Build the application**

```bash
west build -p auto -b <motor_controller_target_board> applications/motor_controller
```

Expected: clean build. If you don't have a target board flashed handy, settle for a build-only check on the same target the existing `west.yml` resolves to. Hardware verification is left to the user.

- [ ] **Step 4: Commit (in the zephyr-applications repo)**

```bash
git add applications/motor_controller/src/main.c
git commit -m "motor_controller: switch to typed dynamixel read/write API"
```

- [ ] **Step 5: Manifest bump (later, after drivers PR merges)**

After the drivers PR merges to `rosterloh-drivers/main`, bump the manifest revision in `zephyr-applications`'s `west.yml` to point to the new SHA, push the consumer branch, merge.

- [ ] **Step 6: Return to drivers repo**

```bash
cd -
```

End-of-Phase-3 gate: All tests on drivers repo green. Public API is typed. Consumer repo compiles cleanly against the new API (PR pending coordination).

---

## Phase 4 — DT motor metadata

End of phase: `dxl_motor_count`, `dxl_motor_get`, `dxl_motor_get_by_label` work. `protocol-version` is removed from the binding.

### Task 18: Drop protocol-version from the binding

**Files:**
- Modify: `dts/bindings/robotis,dynamixel.yaml`

- [ ] **Step 1: Edit the YAML**

Replace `dts/bindings/robotis,dynamixel.yaml` with:

```yaml
description: Dynamixel over serial line device

compatible: "robotis,dynamixel"

include: uart-device.yaml

properties:
    tx-en-gpios:
      type: phandle-array
      required: false
      description: Transmit enable pin (TX_EN) of the transceiver.

child-binding:
    description: Dynamixel motor on this bus.
    properties:
      label:
        type: string
        required: false
        description: Human readable string describing the device.

      id:
        type: int
        required: true
        description: The address on the bus of this device.
```

- [ ] **Step 2: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add dts/bindings/robotis,dynamixel.yaml
git commit -m "dts: dynamixel: drop unused protocol-version property"
```

### Task 19: Add motor metadata API + tests

**Files:**
- Modify: `include/drivers/dynamixel.h`
- Modify: `drivers/dynamixel/dynamixel.c`
- Modify: `tests/drivers/dynamixel/src/test_motors.c`

- [ ] **Step 1: Write motor tests first**

Replace `tests/drivers/dynamixel/src/test_motors.c` with:

```c
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
```

Delete the placeholder `test_phase1_suite_runs`.

- [ ] **Step 2: Run — should FAIL (functions undefined)**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: build FAILS.

- [ ] **Step 3: Add types and prototypes to the public header**

In `include/drivers/dynamixel.h`, near the bottom of the API block, add:

```c
/**
 * @brief Motor metadata derived from devicetree child nodes.
 */
struct dxl_motor {
	/** Optional label string from DT. May be NULL. */
	const char *label;
	/** Parent Dynamixel interface index. */
	int         iface;
	/** Bus ID of the motor. */
	uint8_t     id;
};

/**
 * @brief Number of motors discovered in devicetree.
 */
size_t dxl_motor_count(void);

/**
 * @brief Get the motor entry at @a idx, or NULL if out of range.
 */
const struct dxl_motor *dxl_motor_get(size_t idx);

/**
 * @brief Find a motor by its DT @c label, or NULL if not found.
 *
 * The comparison is by @c strcmp; both arguments must be non-NULL.
 */
const struct dxl_motor *dxl_motor_get_by_label(const char *label);
```

- [ ] **Step 4: Implement the metadata table in dynamixel.c**

In `drivers/dynamixel/dynamixel.c`, after the existing `dxl_ctx_tbl[]` definition, add:

```c
#define DXL_MOTOR_ENTRY(motor_node, parent_inst)                       \
	{                                                              \
		.label = DT_PROP_OR(motor_node, label, NULL),          \
		.iface = parent_inst,                                  \
		.id    = DT_PROP(motor_node, id),                      \
	},

#define DXL_IFACE_MOTORS(inst)                                         \
	DT_INST_FOREACH_CHILD_VARGS(inst, DXL_MOTOR_ENTRY, inst)

static const struct dxl_motor dxl_motors[] = {
	DT_INST_FOREACH_STATUS_OKAY(DXL_IFACE_MOTORS)
};

size_t dxl_motor_count(void)
{
	return ARRAY_SIZE(dxl_motors);
}

const struct dxl_motor *dxl_motor_get(size_t idx)
{
	if (idx >= ARRAY_SIZE(dxl_motors)) {
		return NULL;
	}
	return &dxl_motors[idx];
}

const struct dxl_motor *dxl_motor_get_by_label(const char *label)
{
	if (label == NULL) {
		return NULL;
	}
	for (size_t i = 0; i < ARRAY_SIZE(dxl_motors); i++) {
		if (dxl_motors[i].label == NULL) {
			continue;
		}
		if (strcmp(dxl_motors[i].label, label) == 0) {
			return &dxl_motors[i];
		}
	}
	return NULL;
}
```

`<string.h>` is already included near the top of the file.

- [ ] **Step 5: Build and run**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS for all four motor tests.

- [ ] **Step 6: Commit**

```bash
git add include/drivers/dynamixel.h drivers/dynamixel/dynamixel.c \
        tests/drivers/dynamixel/src/test_motors.c
git commit -m "drivers: dynamixel: add DT-driven motor metadata lookup"
```

### Task 20: Final cleanup pass

**Files:** various

- [ ] **Step 1: Verify nothing references removed types**

```bash
rg -n 'dxl_motor_config|dxl_raw_cb_t|info_x330\W' include drivers tests
```

Expected: only the moved declaration in `dynamixel_internal.h` for `dxl_info_x330`. If `info_x330` (without the `dxl_` prefix) appears anywhere else, it's a leftover and needs deletion.

- [ ] **Step 2: Verify the public header is self-contained**

```bash
rg -n 'static const' include/drivers/dynamixel.h
```

Expected: no results.

- [ ] **Step 3: Run the full suite one more time**

Run: `west twister -p native_sim -T tests/drivers/dynamixel --inline-logs`
Expected: PASS, all suites green.

- [ ] **Step 4: Run twister in integration mode (matches CI)**

Run: `west twister -T tests -v --inline-logs --integration`
Expected: PASS (this is what `.github/workflows/build.yml` runs).

- [ ] **Step 5: No commit if no changes** — just confirm and proceed.

End-of-Phase-4 gate: All four suites green, public header has no `static const` arrays, motor metadata layer functional, binding honest about what the driver implements.

---

## Spec coverage check

| Spec section / requirement                                | Plan task(s)             |
|-----------------------------------------------------------|--------------------------|
| §3 file layout                                            | 2, 3, 4, 8               |
| §4 typed read/write API                                   | 15, 16                   |
| §4.1 width validation                                     | 15                       |
| §4.2 return-code contract (<0 / 0 / >0)                   | 13, 14, 15               |
| §4.3 removed declarations                                 | 11, 16                   |
| §5 dynamixel_tables.c with designated initializers        | 8 (designated init included from the start) |
| §6 binding YAML cleanup                                   | 18                       |
| §7 motor metadata generation                              | 19                       |
| §8 native_sim + uart_emul + fake servo                    | 1, 2, 3, 4               |
| §8.3 protocol tests                                       | 5 (pinning), 13–16 (final) |
| §8.3 tables tests                                         | 6, 9                     |
| §8.3 motor tests                                          | 7 (placeholder), 19      |
| §9 phased sequencing                                      | overall                  |
| §10 risks                                                 | task 4 step 5 + cross-repo coordination in task 17 |
| §11 acceptance criteria                                   | task 20                  |
