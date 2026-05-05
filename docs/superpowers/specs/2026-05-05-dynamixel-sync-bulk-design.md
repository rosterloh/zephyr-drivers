# Dynamixel SYNC / BULK Support — Design

## Goal

Add Protocol-2 group instructions to the Dynamixel driver so callers can address multiple servos in a single bus transaction:

- `SYNC_READ` (0x82) — read the **same** register from N servos.
- `SYNC_WRITE` (0x83) — write the **same** register to N servos.
- `BULK_READ` (0x92) — read **different** registers from N servos.
- `BULK_WRITE` (0x93) — write **different** registers to N servos.

Today the driver only supports per-servo `READ` / `WRITE`, so a 4-leg robot pays 4× the bus turnarounds for goal-position updates and position telemetry. Group instructions collapse N round-trips to one TX (writes) or one TX + N status replies (reads).

## Non-goals (deferred)

- **Group handles.** No `DXL_SYNC_WRITE_GROUP_DEFINE` macro, no opaque builder objects. Stateless arrays only. Revisit if a real consumer needs the per-call setup cost gone.
- **Async / callback API.** Calls block until the transaction completes (or times out). A separate future project may add an async path; it would replace the RX *sink*, not the protocol surface designed here.
- **Protocol-1 group support.** Protocol-2 only.

## API Surface

Public, declared in `include/drivers/dynamixel.h`.

### SYNC variants (uniform register)

```c
int dxl_sync_read_u8 (int iface, enum dxl_control item,
                      const uint8_t ids[], uint8_t  vals[],
                      int errs[], size_t n);
int dxl_sync_read_u16(int iface, enum dxl_control item,
                      const uint8_t ids[], uint16_t vals[],
                      int errs[], size_t n);
int dxl_sync_read_u32(int iface, enum dxl_control item,
                      const uint8_t ids[], uint32_t vals[],
                      int errs[], size_t n);

int dxl_sync_write_u8 (int iface, enum dxl_control item,
                       const uint8_t ids[], const uint8_t  vals[], size_t n);
int dxl_sync_write_u16(int iface, enum dxl_control item,
                       const uint8_t ids[], const uint16_t vals[], size_t n);
int dxl_sync_write_u32(int iface, enum dxl_control item,
                       const uint8_t ids[], const uint32_t vals[], size_t n);
```

The `_u8 / _u16 / _u32` suffix matches the existing single-shot API. The driver looks up the register width from `item` and returns `-EINVAL` if it does not match the typed function chosen.

### BULK variants (heterogeneous registers)

```c
struct dxl_bulk_read_entry  { uint8_t id; enum dxl_control item; };
struct dxl_bulk_write_entry { uint8_t id; enum dxl_control item; uint32_t value; };

int dxl_bulk_read (int iface, const struct dxl_bulk_read_entry  req[],
                   uint32_t vals[], int errs[], size_t n);
int dxl_bulk_write(int iface, const struct dxl_bulk_write_entry req[], size_t n);
```

Bulk uses one entry-point per direction because widths vary per entry. Read out is `uint32_t[]` because all current control-table items fit in 32 bits; the driver looks up the actual width from `req[i].item` and stores the appropriately-sized value in the slot.

### Error contract

**Function return:**
- `0` — every targeted servo succeeded.
- `<0` — transport / setup failure: `-ENODEV`, `-EINVAL` (bad item, width mismatch, NULL pointer with `n>0`, `n==0`), `-ENOSPC` (computed packet > `CONFIG_DYNAMIXEL_BUFFER_SIZE`).
- `-EIO` — at least one servo failed; details in `errs[]` if non-NULL.

**Per-slot `errs[i]` (read variants only):**
- `0` — that slot's `vals[i]` is valid.
- `>0` — device error byte (see `enum dxl_error`); `vals[i]` is left untouched.
- `<0` — transport error for that slot: `-ETIMEDOUT`, `-EIO` (CRC), `-EBADMSG` (status from a different ID).

**`errs == NULL` is allowed.** The caller has opted out of per-slot detail. The driver still runs all N iterations; failed slots leave `vals[i]` untouched; the function return collapses to `0` or `-EIO`.

**Write variants have no `errs[]`.** SYNC/BULK_WRITE are sent to the broadcast ID and produce no status replies. Per-servo failure is undetectable without a follow-up read — a property of the Dynamixel protocol, documented in the API help.

## Implementation

### File layout

| File | Role |
|---|---|
| `drivers/dynamixel/dynamixel_protocol.c` | Existing single-shot `ping`, `reboot`, `read_n`, `write_n`. Refactored to call the new internal helper described below. |
| `drivers/dynamixel/dynamixel_group.c` | **New.** All four sync/bulk entry points, packet builders, the multi-status loop. |
| `drivers/dynamixel/dynamixel_internal.h` | **Updated.** Exposes the new `parse_status_payload` helper. |
| `include/drivers/dynamixel.h` | **Updated.** Public sync/bulk prototypes plus the two `dxl_bulk_*_entry` structs. |
| `drivers/dynamixel/Kconfig` | **Updated.** `DYNAMIXEL_BUFFER_SIZE` range bumped from `64 256` to `64 1024`. Help text gains the sizing formula. |

### RX path: loop the existing primitive

`dynamixel_serial.c` and `dxl_tx_wait_rx()` are unchanged. SYNC_READ / BULK_READ:

1. Take `ctx->iface_lock`.
2. Build the broadcast instruction packet in `ctx->tx_frame`.
3. `dxl_serial_tx()` once.
4. **Loop** for `i = 0..n-1`:
   - `ctx->expected_id = ids[i]` (or `req[i].id` for bulk).
   - Wait on `ctx->wait_sem` with `ctx->rxwait_to`.
   - Call `parse_status_payload(ctx->rx_frame.data, length, &val_u32)`.
   - Translate `val_u32` to the typed slot in `vals[]` (sync) or per-entry width (bulk).
   - Record `errs[i]`.
5. Release the mutex.

This is exactly how the single-shot read works today — N times, under one lock. No changes to the RX path's shape, signalling, or timer.

### Encapsulation: `parse_status_payload`

Extracted from the current `dxl_read_n`:

```c
/* dynamixel_internal.h. `width` is the register width in bytes (1/2/4). */
int parse_status_payload(const uint8_t *data, uint8_t width, uint32_t *out);
```

Implementation: read `data[0]` as the device-error byte (return it as a positive value if non-zero), then read the payload starting at `data[1]` — `data[1]` for u8, `sys_get_le16(&data[1])` for u16, `sys_get_le32(&data[1])` for u32 (any other `width` returns `-EINVAL`). Both single-shot read and the sync/bulk loop call this. The protocol layer no longer reads `ctx->rx_frame` directly for the value path; a future async-RX redesign can deliver `(data, width)` from a callback without touching the protocol layer.

### TX builders

Internal to `dynamixel_group.c`. All compute total packet size first; if it would exceed `CONFIG_DYNAMIXEL_BUFFER_SIZE` they return `-ENOSPC` before locking the bus. Each builder writes into `ctx->tx_frame.data[]` and sets `tx_frame.length = ic-byte + crc + params`.

| Helper | Param layout (after instruction byte) |
|---|---|
| `build_sync_read(addr, len, ids, n)` | `addr_le16 ‖ len_le16 ‖ id[0..n-1]` |
| `build_sync_write(addr, len, ids, vals_packed, n)` | `addr_le16 ‖ len_le16 ‖ {id, data[len]} × n` |
| `build_bulk_read(reqs, n)` | `{id, addr_le16, len_le16} × n` |
| `build_bulk_write(reqs, n)` | `{id, addr_le16, len_le16, data[len]} × n` |

`tx_frame.id = DXL_BROADCAST_ID` (0xFE) for all four. SYNC_READ / BULK_READ then update `expected_id` per loop iteration before waiting; SYNC_WRITE / BULK_WRITE skip the wait entirely.

### Validation (pre-flight, before taking the mutex)

- `iface < 0`, `n == 0`, `ids == NULL`, `vals == NULL` → `-EINVAL`.
- For SYNC: `dxl_table_lookup(item)` failure or width mismatch → `-EINVAL`.
- For BULK: each `req[i].item` lookup failure → `-EINVAL`. Per-entry width determines packet size.
- Computed packet size > `CONFIG_DYNAMIXEL_BUFFER_SIZE` → `-ENOSPC`.

### Locking

`ctx->iface_lock` covers the entire transaction (build → tx → optional N-iteration wait → parse). No new locks. Single-shot callers continue to serialise against sync/bulk callers on the same mutex, exactly as today.

### Kconfig

```kconfig
config DYNAMIXEL_BUFFER_SIZE
    int "Dynamixel buffer size"
    default 256
    range 64 1024
    help
      Maximum frame size (bytes) for the Dynamixel TX/RX buffers.
      Must be large enough for the largest packet you intend to send.

      SYNC_WRITE  ≈ 14 + N * (1 + L)
      BULK_WRITE  ≈ 10 + N * (5 + L)

      where N is the number of servos addressed and L is the register
      width in bytes. The default of 256 covers ~12 servos × 4-byte
      writes; raise it if you need more.
```

The bump from `range 64 256` to `range 64 1024` is backwards-compatible — existing values stay valid.

## Test Plan

### Harness extension: multi-instance fake bus

Today's `fake_servo` is a single emulated servo on an emulated UART. For sync/bulk we add a thin "bus" that owns up to `CONFIG_TEST_FAKE_SERVO_MAX` fakes and dispatches incoming packets:

```c
struct fake_bus {
    struct fake_servo srv[CONFIG_TEST_FAKE_SERVO_MAX];
    size_t n;
};

void fake_bus_init     (struct fake_bus *bus, const uint8_t ids[], size_t n);
void fake_bus_set_value(struct fake_bus *,  uint8_t id, enum dxl_control, uint32_t);
struct fake_servo *fake_bus_get(struct fake_bus *, uint8_t id);
```

Dispatch rules:
- **PING / READ / WRITE** (existing single-shot): only the matching fake responds.
- **SYNC_WRITE / BULK_WRITE:** every targeted fake applies its slice to its register file. No replies.
- **SYNC_READ / BULK_READ:** for each requested ID **in request order**, the matching fake emits a status packet. The driver picks them up one at a time; the inter-frame timer fires synthetically per packet (same mechanism used by current single-shot tests).

The existing single-fake API stays valid for the 22 existing tests (`n=1` case).

### New cases (≈14)

| Group | Case |
|---|---|
| sync_write happy | 4 servos × u32 GOAL_POSITION → all register files updated, return 0 |
| sync_write widths | _u8 / _u16 / _u32 happy paths |
| sync_write width-mismatch | `_u8` on a u32 register → `-EINVAL` |
| sync_write -ENOSPC | `N × L` exceeds buffer → returns before TX, no register state changes |
| sync_write validation | `n=0`, `ids=NULL`, bad item → `-EINVAL` |
| sync_read happy | 4 servos preloaded with distinct values → `vals[]` populated, `errs[]` all 0 |
| sync_read partial fail | 1 of 4 has `drop_response=true` → `vals[good]` correct, `errs[3] == -ETIMEDOUT`, return `-EIO` |
| sync_read device error | 1 fake returns error byte → `errs[i] > 0`, `vals[i]` untouched, return `-EIO` |
| sync_read wrong-ID reply | `answer_any_id=true` on slot 1 → `errs[1] == -EBADMSG`, others fine |
| sync_read errs=NULL | same flaky setup, no `errs[]` → return `-EIO`, good slots populated |
| bulk_read happy | mixed `(id, item)` entries → `vals[]` match per-entry register |
| bulk_write happy | mixed `(id, item, value)` entries → each fake's register file matches |
| bulk validation | bad item / NULL / `n=0` → `-EINVAL` |
| bulk -ENOSPC | oversized request → returns before TX |

### Regression coverage

The existing 22 single-shot tests must still pass after the `parse_status_payload` extraction. No behavioural change to single-shot paths.

### Test fixtures

`before_each` constructs a `fake_bus` with a default fleet of 4 servos. Tests poke per-servo knobs (`drop_response`, `corrupt_crc`, `error_byte`, `answer_any_id`, register pre-loads) before calling the driver. `after_each` resets the bus.

## Risks (acknowledged, accepted)

- **`fake_bus` dispatcher complexity.** Multi-instance harness adds ≈150 lines of test code. Mitigation: dispatch is by ID and instruction byte only; per-instruction handlers already exist on the single fake.
- **Timeout amplification.** N missing servos in a SYNC_READ → up to `N × rxwait_to` worst case. For 32 motors × 50 ms that is 1.6 s. Property of the chosen "always run all N" semantics; documented in the API help. Real X-series respond in <2 ms, so production callers tune `rxwait_to` accordingly.
- **BULK_WRITE error invisibility.** Writes are broadcast → no status replies → silent per-servo failures (servo unplugged, register read-only, etc.) cannot be detected without a follow-up read. Property of the Dynamixel protocol, not the design.
- **Real-bus arbitration timing.** Tests use synchronous emulated UART; real servos respond with bus-turnaround delays. Same caveat that already applies to the existing test suite — accepted as integration risk.

## Open questions

None — all five design questions resolved during brainstorming:

1. Scope: all four group instructions.
2. API shape: stateless arrays, no handles.
3. RX path: loop the existing single-frame primitive + small `parse_status_payload` extraction.
4. Buffer sizing: single buffer, raise Kconfig range, validate and surface `-ENOSPC`.
5. Partial failure: per-slot `errs[]` with `NULL` allowed, function returns `-EIO` on any failure.
