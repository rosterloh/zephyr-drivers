# Actuator drive-mode capability

**Date:** 2026-05-27
**Scope:** `rosterloh-drivers` module — actuator subsystem + H-bridge backend
**Motivation:** Integrate TB6612FNG-based H-bridge drives (rasprover application) which expose IN1/IN2 inputs and therefore can perform active brake / coast under software control.

---

## Goals

1. Add a software-controllable drive-mode primitive (Normal / Brake / Coast) to the generic actuator API.
2. Extend the H-bridge driver to drive TB6612FNG-style chips that use PWM + IN1 + IN2 (+ optional STBY) signalling.
3. Preserve the existing PWM + single-direction-GPIO path for simpler H-bridge ICs.

## Non-goals

- Regenerative braking with controlled current ramp.
- A new `BRAKING` state in the actuator state machine.
- Per-mode capability bits (e.g. distinct `CAP_BRAKE` vs `CAP_COAST`).
- Smart-servo backends (Dynamixel): no drive-mode support; they will report `-ENOTSUP`.

---

## Public API

### New type and capability bit

In `include/zephyr/actuator/actuator_types.h`:

```c
enum actuator_drive_mode {
    ACTUATOR_DRIVE_MODE_NORMAL = 0, /* PWM-controlled output (default) */
    ACTUATOR_DRIVE_MODE_BRAKE,      /* short motor windings to GND     */
    ACTUATOR_DRIVE_MODE_COAST,      /* high-Z, free-wheel              */
};

#define ACTUATOR_CAP_DRIVE_MODE BIT(6)
```

### New syscall

In `include/zephyr/actuator/actuator.h`:

```c
/**
 * Set the output policy of the actuator's power stage.
 *
 * @retval 0         Mode applied.
 * @retval -ENOTSUP  Backend does not advertise ACTUATOR_CAP_DRIVE_MODE.
 * @retval -EPERM    Actuator is in DISABLED or FAULT state.
 */
__syscall int actuator_set_drive_mode(const struct device *dev,
                                      enum actuator_drive_mode mode);
```

The drive mode is **orthogonal** to the existing state machine. Calling
`actuator_set_drive_mode(dev, BRAKE)` while in `READY` or `ACTIVE` does not
change the state; it only changes the power-stage output policy. Setting any
position/velocity/effort setpoint implicitly returns the stage to
`ACTUATOR_DRIVE_MODE_NORMAL` before applying the setpoint.

## Backend vtable change

In `drivers/actuator/actuator_internal.h`:

```c
struct actuator_driver_api {
    /* ...existing fields... */
    int (*set_drive_mode)(const struct device *dev, enum actuator_drive_mode mode);
};
```

A backend that does not implement the op leaves the pointer NULL and does not
advertise `ACTUATOR_CAP_DRIVE_MODE`. The subsystem rejects the syscall with
`-ENOTSUP` when the cap is missing, so the vtable entry is not dereferenced.

## Subsystem behaviour

`z_impl_actuator_set_drive_mode` (in `subsys/actuator/actuator.c`):

1. Look up `cd->caps`; return `-ENOTSUP` if `ACTUATOR_CAP_DRIVE_MODE` not set.
2. Under the device spinlock, read `cd->state`; return `-EPERM` if it is
   `ACTUATOR_STATE_DISABLED` or `ACTUATOR_STATE_FAULT`.
3. Call `api(dev)->set_drive_mode(dev, mode)` outside the lock.
4. On failure, treat as a driver-reported fault
   (`actuator_report_state(dev, FAULT, ACTUATOR_FAULT_DRIVER(0))`).

`set_setpoint_typed` (existing) is extended: after `sm_step_locked` succeeds
and *before* the backend `set_setpoint` call, the subsystem calls
`api(dev)->set_drive_mode(dev, NORMAL)` if and only if:
- the backend has the op,
- the actuator has `ACTUATOR_CAP_DRIVE_MODE`.

This makes "set a setpoint" implicitly return to normal output, regardless of
prior brake/coast state. If the implicit `set_drive_mode(NORMAL)` call fails,
the subsystem reports a driver fault (same as a failed `set_setpoint`) and
returns the errno; it does not attempt to apply the setpoint.

## Devicetree binding change

`dts/bindings/actuator/rosterloh,actuator-hbridge.yaml`:

| Property | Type | Required | Notes |
| --- | --- | --- | --- |
| `pwms` | phandle-array | yes | unchanged |
| `in1-gpios` | phandle-array | yes | **renamed** from `dir-gpios` |
| `in2-gpios` | phandle-array | no | when present → IN1/IN2 signalling |
| `stby-gpios` | phandle-array | no | optional; may be shared across instances by the application |
| `io-channels` / `encoder` / current-sense props | — | — | unchanged |

`dir-gpios` is removed. The migration is a rename: any overlay using
`dir-gpios = <...>` becomes `in1-gpios = <...>`. The only in-tree user today
is `tests/drivers/actuator/hbridge/boards/native_sim.overlay`.

## H-bridge driver behaviour

Signalling is selected at init by inspecting the presence of `in2-gpios`:

| Configuration | `in1-gpios` only (PWM + DIR) | `in1-gpios` + `in2-gpios` (PWM + IN1/IN2) |
| --- | --- | --- |
| Forward | PWM duty = abs(d), IN1 = 1 | PWM duty = abs(d), IN1 = 1, IN2 = 0 |
| Reverse | PWM duty = abs(d), IN1 = 0 | PWM duty = abs(d), IN1 = 0, IN2 = 1 |
| `NORMAL` | (default) | (default) |
| `BRAKE` | PWM = 0 (chip-dependent: brake on TB67H, etc.) | IN1 = 1, IN2 = 1, PWM = 0 |
| `COAST` | PWM = 0 (chip-dependent: coast on DRV8871) | IN1 = 0, IN2 = 0, PWM = 0 |
| Cap advertised | `ACTUATOR_CAP_DRIVE_MODE` **not** advertised | `ACTUATOR_CAP_DRIVE_MODE` advertised |

The PWM+DIR variant does **not** implement `set_drive_mode`. Apps that need
deterministic brake/coast must use the IN1/IN2 variant. This is documented in
the binding YAML.

STBY (when present) is asserted in `enable()` and deasserted in `disable()`.
It is asserted high (active-high standby-release per TB6612 datasheet) — the
DT spec's `GPIO_ACTIVE_LOW` flag handles inverted-polarity wiring.

The TB6612 truth table that justifies the IN1/IN2 row in the table above is
documented in a single comment block at the top of `actuator_hbridge.c`.

## Other backends

- **`actuator_dxl`**: unchanged. Does not implement `set_drive_mode`, does not
  advertise `ACTUATOR_CAP_DRIVE_MODE`. Syscall returns `-ENOTSUP`.
- **`actuator_fake`**: implements `set_drive_mode` to store the most recently
  requested mode in test-visible per-instance state. Advertises
  `ACTUATOR_CAP_DRIVE_MODE` so that subsystem-level ztests can exercise the
  full path (cap check, state gating, auto-clear on setpoint).

## Shell

`subsys/actuator/actuator_shell.c` gains:

```
actuator mode <dev> normal|brake|coast
```

Wired to `actuator_set_drive_mode`. Returns the syscall's errno on failure.

## Tests

- `tests/subsys/actuator/` — add cases:
  - `mode_rejected_without_cap` (uses a fake instance with cap masked off)
  - `mode_rejected_when_disabled`
  - `mode_rejected_when_faulted`
  - `mode_applied_in_ready` (fake stores it, state unchanged)
  - `setpoint_auto_resumes_normal_after_brake`
- `tests/drivers/actuator/` — add a fake-backend driver test confirming the
  vtable plumbing reaches the driver and the stored mode is read-back
  correctly.
- Hbridge driver-level tests of the brake/coast GPIO levels are out of scope
  here (no native_sim PWM mocking today). The existing native_sim hbridge
  test is updated for the binding rename and remains a smoke test only.

## Out-of-scope / future

- A separate `phase-gpios` name for the 1-GPIO case (more semantically
  accurate than `in1-gpios` when only one is present). Skipped to keep one
  property name across both topologies; revisit if it causes confusion.
- Power-stage current limiting via the existing ADC current-sense path
  reacting to drive-mode (e.g. dynamic-brake current trip). Existing
  overcurrent fault path already covers this.

## Migration checklist

1. Rename `dir-gpios` → `in1-gpios` in
   `tests/drivers/actuator/hbridge/boards/native_sim.overlay`.
2. Communicate the rename in the next module bump (changelog / commit body).
   No production overlay in `zephyr-applications` declares an hbridge
   instance yet, so the migration touches one file.
