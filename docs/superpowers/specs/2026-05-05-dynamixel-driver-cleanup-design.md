# Dynamixel driver cleanup — design

**Date:** 2026-05-05
**Repo:** rosterloh/zephyr-drivers (vendored at `deps/modules/lib/rosterloh-drivers` in the zephyr-applications workspace)
**Scope:** All "Design / API smells" and "Smaller things" items from the prior code review of the Dynamixel driver.
**Out of scope:** Critical bugs from the same review (32-bit-read truncation, error-byte overwrite, missing `item_idx` bounds check, missing response-ID match). Those are bugs and will be fixed by this work as a side effect of the API redesign and test harness, but they were called out separately and are not the headline of this design.
**Also out of scope:** SYNC_READ / SYNC_WRITE / BULK_READ / BULK_WRITE, async API, protocol-1 support. Each is its own follow-up project.

## 1. Goals

1. Eliminate footgun-prone API shapes (`void *` typed reads, `uint32_t` writes that silently truncate, error codes that get overwritten by device error bytes).
2. Stop publishing static-storage arrays (`control_table[]`, `info_x330`) from a public header.
3. Make the unused DT motor child-binding actually do something — or document that it doesn't. (Decided: make it functional as a metadata layer.)
4. Stop advertising features in the binding that the driver does not implement (protocol 1).
5. Replace the hardware-loopback-only test stub with a self-contained `native_sim` suite that exercises the protocol end-to-end through a programmable fake servo.

## 2. Non-goals

- No backwards compatibility with the existing public API. The motor_controller application is the only consumer and will be updated in lockstep. Old function names (`dxl_read`, `dxl_write` taking `void *` / `uint32_t`) are removed.
- No new register support, no new protocol features, no performance work.
- No hardware test fixture maintenance. The existing `arduino_mkrzero` overlay and the loopback fixture documented in `tests/drivers/dynamixel/testcase.yaml` are removed. A future hardware smoke test is an additive change, not a regression.

## 3. File layout

```
include/drivers/dynamixel.h        ← public API only (no static const arrays)
drivers/dynamixel/
    dynamixel.c                    ← interface lifecycle (init, disable, get_by_name)
    dynamixel_protocol.c           ← ping/read/write/reboot, packet assembly
    dynamixel_serial.c             ← UART IRQ + TX/RX state machine
    dynamixel_tables.c   (NEW)     ← control_table[], info_x330, accessors
    dynamixel_internal.h           ← role unchanged; gains dxl_table_lookup proto
    Kconfig                        ← unchanged
    CMakeLists.txt                 ← adds dynamixel_tables.c
tests/drivers/dynamixel/
    boards/                        ← deleted (hardware overlay removed)
    boards/native_sim.overlay (NEW)
    src/
        main.c                     ← ZTEST_SUITE registration
        test_protocol.c   (NEW)    ← ping/read/write/reboot wire-format + error paths
        test_tables.c     (NEW)    ← control_table sanity (offsets, lengths)
        test_motors.c     (NEW)    ← DT motor metadata lookup
        fake_servo.c      (NEW)    ← mock servo backend
        fake_servo.h      (NEW)
    prj.conf                       ← updated for native_sim + uart_emul
    testcase.yaml                  ← platform_allow: native_sim
```

The existing `test_dynamixel.c` and `test_dynamixel.h` stub files are deleted (their content was a setup/disable smoke test with no real coverage).

## 4. Public API

```c
/* Lifecycle — int iface everywhere */
int  dxl_init(int iface, struct dxl_iface_param param);
int  dxl_disable(int iface);
int  dxl_iface_get_by_name(const char *iface_name);

/* Instructions — protocol-level */
int  dxl_ping(int iface, uint8_t id);
int  dxl_reboot(int iface, uint8_t id);

/* Typed read — width-correct, no void* */
int  dxl_read_u8 (int iface, uint8_t id, enum dxl_control item, uint8_t  *out);
int  dxl_read_u16(int iface, uint8_t id, enum dxl_control item, uint16_t *out);
int  dxl_read_u32(int iface, uint8_t id, enum dxl_control item, uint32_t *out);

/* Typed write */
int  dxl_write_u8 (int iface, uint8_t id, enum dxl_control item, uint8_t  val);
int  dxl_write_u16(int iface, uint8_t id, enum dxl_control item, uint16_t val);
int  dxl_write_u32(int iface, uint8_t id, enum dxl_control item, uint32_t val);

/* DT motor metadata */
struct dxl_motor {
    const char *label;     /* may be NULL */
    int         iface;     /* parent interface index */
    uint8_t     id;        /* bus ID */
};
const struct dxl_motor *dxl_motor_get_by_label(const char *label);
size_t                  dxl_motor_count(void);
const struct dxl_motor *dxl_motor_get(size_t idx);
```

### 4.1 Width validation

Each `dxl_read_u*` / `dxl_write_u*` consults `dxl_table_lookup(item, ...)` and returns `-EINVAL` if the function's width does not match the register's declared length. Calling `dxl_read_u8(.., PRESENT_POSITION, ..)` (a 4-byte register) is therefore a runtime error, not a silent truncation.

`item` out of range also returns `-EINVAL` from the same lookup.

### 4.2 Return code contract

Every API function returns one of three classes:

| Range  | Meaning                                                  |
|--------|----------------------------------------------------------|
| `< 0`  | Transport / setup failure: `-ETIMEDOUT`, `-EIO` (CRC), `-ENODEV` (interface not configured), `-EINVAL` (bad arg / width mismatch / out-of-range register). |
| `0`    | Success.                                                 |
| `> 0`  | Device returned an error byte; value is one of `enum dxl_error` (e.g. `DXL_ERR_DATA_RANGE`). |

Callers use the standard pattern:
```c
int rc = dxl_write_u32(iface, id, GOAL_POSITION, value);
if (rc < 0)         { /* couldn't reach the device */ }
else if (rc > 0)    { /* device rejected the command, see enum dxl_error */ }
```

This fixes the existing bug where `dxl_tx_wait_rx`'s return value was overwritten by `ctx->rx_frame.data[0]`.

### 4.3 Removed from the public header

- `dxl_read`, `dxl_write` (replaced by typed forms).
- `dxl_raw_cb_t` (unused).
- `struct dxl_motor_config` (replaced by `struct dxl_motor`).
- `static const struct dxl_control_info control_table[]` (moved to `dynamixel_tables.c`).
- `static const struct dxl_model_info info_x330` (moved to `dynamixel_tables.c`).
- `struct dxl_frame::header` field (always written from a literal in `tx_frame()`; never read).

### 4.4 Kept

- `enum dxl_instruction`, `enum dxl_mode`, `enum dxl_error`, `enum dxl_control`.
- `struct dxl_iface_param`, `struct dxl_serial_param`.
- The `XL330_*` / `XC330_*` model-number constants.
- The `DXL_BROADCAST_ID` and hardware-error-status bit definitions.

## 5. Tables module (`dynamixel_tables.c`)

The control table moves to a file-static array with **designated initializers**, decoupling array order from `enum dxl_control` order:

```c
static const struct dxl_control_info control_table[] = {
    [MODEL_NUMBER]      = { 0,  2 },
    [FIRMWARE_VERSION]  = { 6,  1 },
    [ID]                = { 7,  1 },
    /* ... */
    [PRESENT_POSITION]  = { 132, 4 },
    [PRESENT_TEMPERATURE] = { 146, 1 },
};
```

Internal accessor:

```c
/* dynamixel_internal.h */
int dxl_table_lookup(enum dxl_control item,
                     uint16_t *addr, uint8_t *length);
```

Returns `0` on success, `-EINVAL` if `item` is out of range or has a zero-length entry (which signals a missing initializer, caught at test time by `test_tables.c`).

`info_x330` and `struct dxl_model_info` are moved into the same file. They are currently unreferenced anywhere in the driver or its consumers; they remain available for future unit-conversion helpers but are not part of the public API right now.

## 6. DT bindings

```yaml
# dts/bindings/robotis,dynamixel.yaml
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
    id:
      type: int
      required: true
```

Differences vs current binding: `protocol-version` removed entirely (the driver only implements protocol 2; advertising 1 is a silent lie).

## 7. Motor metadata generation

`dynamixel.c` builds a compile-time array of motors discovered under all `status = "okay"` Dynamixel buses:

```c
#define DXL_MOTOR_ENTRY(motor_node, parent_inst)                       \
    {                                                                  \
        .label = DT_PROP_OR(motor_node, label, NULL),                  \
        .iface = parent_inst,                                          \
        .id    = DT_PROP(motor_node, id),                              \
    },

#define DXL_IFACE_MOTORS(inst)                                         \
    DT_INST_FOREACH_CHILD_VARGS(inst, DXL_MOTOR_ENTRY, inst)

static const struct dxl_motor dxl_motors[] = {
    DT_INST_FOREACH_STATUS_OKAY(DXL_IFACE_MOTORS)
};
```

`dxl_motor_get_by_label`, `dxl_motor_count`, `dxl_motor_get` are linear scans over this array. Zero RAM cost beyond the table; expected fleet size is < 100, so O(N) lookup is fine.

Apps continue to call `dxl_write_u32(iface, id, …)` directly. The motor table is purely a lookup convenience for "I have a label, give me an `(iface, id)` pair." Per-motor bound methods (e.g. `dxl_motor_set_goal_position(motor, val)`) are deliberately deferred to a future project — they make more sense once SYNC_WRITE exists.

## 8. Test harness

### 8.1 Backend

`tests/drivers/dynamixel/boards/native_sim.overlay` declares two UART nodes using Zephyr's built-in `zephyr,uart-emul` compatible. The Dynamixel device sits on one; the test holds the other end as a raw `const struct device *` and uses `uart_emul_*` APIs to inject RX bytes / capture TX bytes.

`prj.conf` selects `CONFIG_UART_EMUL=y`, `CONFIG_DYNAMIXEL=y`, `CONFIG_ZTEST=y`, `CONFIG_LOG=y`.

`testcase.yaml` runs on `native_sim` only; the `arduino_mkrzero` build-only entry is removed. A new `drivers.dynamixel.build_only` entry against `native_sim` and one ARM target keeps a build smoke test.

### 8.2 Fake servo (`fake_servo.[ch]`)

```c
struct fake_servo {
    uint8_t id;
    uint8_t control_ram[256];     /* mirrors a subset of the X-series table */
    bool    drop_response;        /* simulate timeout */
    bool    corrupt_crc;          /* simulate CRC error */
    uint8_t error_byte;           /* device error to return, 0 = none */
    uint8_t last_tx[64];
    size_t  last_tx_len;
    uint8_t last_instruction;
};

void fake_servo_init  (struct fake_servo *s, uint8_t id);
void fake_servo_attach(struct fake_servo *s, const struct device *uart);
```

`attach()` registers a UART RX callback on the emulated peer that:

1. Buffers bytes into a packet using the same framing the driver uses.
2. Validates CRC; if `corrupt_crc`, mutates one bit of the response CRC before sending.
3. Decodes instruction (`PING` / `READ` / `WRITE` / `REBOOT`).
4. Updates `control_ram[]` for `WRITE`, or composes a status packet from `control_ram[]` for `READ` / `PING`.
5. If `drop_response` is set, sends nothing.
6. Otherwise injects the status packet via `uart_emul_put_rx_data()`.

The fake is reusable across test files (`test_motors.c` may instantiate two of them on a single bus to exercise wrong-ID-response rejection).

### 8.3 Test files

**`test_protocol.c`** — covers the existing bug-pinning cases first, then green-path cases:

- PING request bytes match Robotis Protocol 2 spec exactly (header `FF FF FD 00`, ID, length LE, instruction `0x01`, CRC).
- READ of every length (1 / 2 / 4) round-trips a known value through the fake servo.
- READ of a 4-byte register returns full 32 bits (regression test for the `sys_get_le16` truncation bug).
- WRITE of every length serialises exactly the parameter bytes — no garbage past the parameter.
- WRITE with a value that has bits set in higher bytes than the register holds is rejected by width validation (`dxl_write_u8` with `0x100` returns `-EINVAL`).
- Timeout path: `drop_response = true` ⇒ `dxl_*` returns `-ETIMEDOUT`, not the stale RX byte.
- CRC error path: `corrupt_crc = true` ⇒ returns `-EIO`, not the stale RX byte.
- Device error byte: `error_byte = DXL_ERR_DATA_RANGE` ⇒ returns the positive error value, not zero.
- Wrong-ID response is rejected: a fake servo with a different ID answers an addressed READ; driver discards the packet and returns `-ETIMEDOUT` (the legitimate addressee never replied).
- Width mismatch (`dxl_read_u8(.., PRESENT_POSITION, ..)`) returns `-EINVAL` without sending bytes on the wire.
- Out-of-range `enum dxl_control` value returns `-EINVAL` without sending bytes on the wire.

**`test_tables.c`** — pure unit tests, no UART:

- Every `enum dxl_control` value resolves to a non-zero-length entry via `dxl_table_lookup` (catches missing designated initializers).
- Spot-check critical registers against the X-series datasheet: `PRESENT_POSITION = {132, 4}`, `GOAL_POSITION = {116, 4}`, `TORQUE_ENABLE = {64, 1}`, `OPERATING_MODE = {11, 1}`, `MODEL_NUMBER = {0, 2}`.
- Out-of-range index returns `-EINVAL`.

**`test_motors.c`** — overlay defines two motors under one bus:

- `dxl_motor_count() == 2`.
- `dxl_motor_get_by_label("alpha")` and `("beta")` return distinct entries with the right IDs.
- `dxl_motor_get_by_label("missing")` returns `NULL`.
- All entries point at `iface == 0`.

### 8.4 What the harness does NOT cover

- Real UART timing (the inter-frame `packet_timer`). The emulated UART delivers bytes synchronously; the timer fires synthetically when the test releases bytes.
- GPIO TX_EN edge timing (no GPIO node on `native_sim`).
- Real wire-level CRC of arbitrary bit errors (only the one bit-flip the fake injects).

These belong to a future hardware integration test, not this work.

## 9. Sequencing and TDD flow

The work proceeds in four phases. Each phase is independently mergeable; commits within a phase are independently bisectable.

### Phase 1 — Test scaffolding (no behavior change)

1. Add `dynamixel_tables.c` as a copy of the existing in-header arrays, behind a temporary internal accessor that the rest of the driver does not yet use. Header keeps the static const arrays for now (intentional duplication during the transition).
2. Add `native_sim.overlay`, `prj.conf` updates, `testcase.yaml` switch.
3. Add `fake_servo.[ch]`.
4. Add `test_protocol.c` with **bug-pinning tests** that assert *current* (buggy) behavior — e.g. "32-bit read returns only the low 16 bits", "timeout path returns whatever was in `rx_frame.data[0]`". These document the bugs and will be flipped in phase 3.
5. Add `test_tables.c`, `test_motors.c`.
6. Delete `arduino_mkrzero.overlay` and old `test_dynamixel.[ch]`. Keep `main.c` registering the new suites.

End-of-phase: CI runs the new suite green against current driver code.

### Phase 2 — Internal cleanups (no public API change)

Each of the following is a separate commit, each accompanied by a test or test update:

7. Move `control_table[]` and `info_x330` from header to `dynamixel_tables.c`. Switch `dynamixel_protocol.c` from direct array access to `dxl_table_lookup`. Header no longer publishes the arrays. (`test_tables.c` already covers this.)
8. Convert `control_table[]` to designated initializers. (`test_tables.c` regression-guards.)
9. Drop `struct dxl_frame::header` field; verify nothing reads it.
10. Normalize `dxl_disable` signature to `int iface`.

End-of-phase: internal tidying done, public API still old shape, all tests still green.

### Phase 3 — API redesign (breaking change)

11. Add `dxl_read_u8/u16/u32` and `dxl_write_u8/u16/u32` to header and `dynamixel_protocol.c`. Implement width validation via `dxl_table_lookup`.
12. Implement the new return-code contract (`< 0` transport / `0` ok / `> 0` device error). Fix the error-overwrite bug as a side effect.
13. Add response-ID match check in the RX path; mismatch returns `-EIO`.
14. **Flip the bug-pinning tests** in `test_protocol.c` to assert correct behavior. The diff that flips them is the proof that the rewrite is real.
15. Remove old `dxl_read` / `dxl_write` declarations and the old conditional `sys_put_le32` block from `dynamixel_protocol.c`.
16. Update `applications/motor_controller/src/main.c` in the **zephyr-applications** tree to call the new typed API. **The change is mechanical and minimal** — the only purpose is keeping the existing consumer compiling. Existing call sites translate one-for-one: `dxl_write(iface, id, GOAL_POSITION, slide_val * 4)` becomes `dxl_write_u32(iface, id, GOAL_POSITION, slide_val * 4)`, `dxl_write(iface, id, TORQUE_ENABLE, 0)` becomes `dxl_write_u8`, etc. No app-side refactoring or behavior change. This is a cross-repo change; coordinate PR ordering: drivers PR merged first, then app PR with the manifest revision bump.

End-of-phase: public API is the new shape, motor_controller builds and runs.

### Phase 4 — DT motor metadata

17. Add `struct dxl_motor` and lookup helpers to header.
18. Add `DT_INST_FOREACH_CHILD_VARGS`-based motor table generation in `dynamixel.c`.
19. Update binding YAML: drop `protocol-version`, keep child binding.
20. `test_motors.c` flips from "compile-only stub" to functional.
21. Remove `struct dxl_motor_config` and `dxl_raw_cb_t` from header.

End-of-phase: motor metadata layer functional, DT binding honest.

## 10. Risks

- **`zephyr,uart-emul` availability.** Depends on the Zephyr revision pinned by west. If unavailable, fall back to a plain emul UART driver written for this test or to a custom mock backend implementing `struct uart_driver_api`. Verified during phase 1 scaffolding.
- **Cross-repo coordination.** Phase 3 step 16 spans the drivers repo and the zephyr-applications consumer. Drivers PR must merge first, then the manifest bump in zephyr-applications. If the drivers PR is reverted, the manifest bump must be reverted too.
- **Test coverage of timing-sensitive paths.** The fake UART delivers bytes synchronously; the inter-frame packet timer fires synthetically. Real-UART regressions (e.g. baud rate misconfig) are not caught. Acceptable risk — those are integration concerns and the existing code did not test them either.

## 11. Acceptance criteria

- All four phases land. `west twister -p native_sim -T tests/drivers/dynamixel` is green.
- The public header contains no `static const` arrays.
- `dxl_read_u8(.., PRESENT_POSITION, ..)` returns `-EINVAL` (covered by test).
- `dxl_read_u32(.., PRESENT_POSITION, ..)` returns the full 32-bit value through the fake (covered by test).
- A simulated transport timeout returns `-ETIMEDOUT`, not a stale RX byte (covered by test).
- A simulated device error byte returns positive `enum dxl_error`, not zero (covered by test).
- `dxl_motor_count()` reflects the DT overlay's child motor count (covered by test).
- `applications/motor_controller` compiles cleanly against the new driver API (runtime verification on hardware is left to the consumer; not a CI gate).
- The DT binding YAML no longer mentions `protocol-version`.
