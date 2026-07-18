# Actuator subsystem

A generic actuator API that lets application code command a motor in physical
units — "drive this joint to 1.2 rad" — without caring whether the hardware
behind it is a DC motor on an H-bridge, a Dynamixel smart servo, or a Waveshare
serial bus servo.

## Why it exists

Actuators that do the same job expose wildly different interfaces. An H-bridge
needs PWM duty cycles and direction GPIOs; a Dynamixel servo speaks a register
protocol over a half-duplex serial bus; a bus servo has yet another framing.
Without a common layer, every application re-implements per-device control and
the kinematics/control code is welded to one motor type.

The subsystem hides that behind one API with **typed SI setpoints**
(position in rad, velocity in rad/s, effort in N·m) so the same control code
runs against any backend. See `include/zephyr/actuator/actuator.h`.

## Core ideas

### Capabilities

Not every actuator can do everything — a cheap servo may only do position, an
FOC controller may need a startup alignment phase. Each backend advertises what
it supports as a capability bitmask (`ACTUATOR_CAP_POSITION`,
`ACTUATOR_CAP_VELOCITY`, `ACTUATOR_CAP_EFFORT`, `ACTUATOR_CAP_NEEDS_ALIGN`,
`ACTUATOR_CAP_FAULT_LATCHING`, `ACTUATOR_CAP_DRIVE_MODE`,
`ACTUATOR_CAP_GROUP_NATIVE`). The subsystem checks the requested operation
against the bitmask and returns `-ENOTSUP` rather than dispatching something the
hardware can't do. Query at runtime with `actuator_get_capabilities()`.

### State machine

The subsystem owns a small state machine — `DISABLED → READY → ALIGNING →
ACTIVE → FAULT` — and drives it from events the backend reports (enable,
disable, setpoint, aligned, fault, clear-fault). The transition logic is a
**pure function** (`actuator_sm_step()`), which is why it lives in
`lib/actuator` and is unit-tested in isolation. Backends never invent their own
state semantics; they report what happened and the subsystem decides the
transition. Setting any setpoint implicitly walks an actuator up from DISABLED
through READY into ACTIVE.

### Drive mode is orthogonal

A motor's power-stage policy — `NORMAL`, `BRAKE` (short the windings), `COAST`
(high-Z free-wheel) — is independent of the state machine. Setting a drive mode
doesn't change the actuator's state, and issuing any setpoint returns the stage
to `NORMAL`. This keeps "what is the actuator trying to do" separate from "how
is the power stage configured."

### Feedback

Feedback is a snapshot struct (`struct actuator_feedback`) with a `valid_mask`
saying which fields the backend actually filled (position/velocity/effort/
temperature), plus fault flags and a timestamp. Two read paths:

- `actuator_read_feedback()` — synchronous, forces a backend transaction.
- `actuator_get_feedback()` — returns the last sample cached by the driver's
  internal worker (cheap, non-blocking).

Applications can also register state-change and feedback callbacks instead of
polling.

### Groups

Robots move many joints together. An actuator group
(`ACTUATOR_GROUP_DEFINE`) commands N actuators with one call
(`actuator_group_set_position()` etc.) and coordinates fault handling with a
policy:

- `ISOLATE` — only the faulted actuator stops (default).
- `DISABLE_ALL` — a fault in any member disables the whole group.
- `ESTOP` — emergency stop semantics for the group.

When every member shares a backend that has a native multi-actuator transaction
(e.g. a Dynamixel sync/bulk write, advertised via `ACTUATOR_CAP_GROUP_NATIVE`),
the group uses that fast path; otherwise it falls back to a per-device loop.

## Layering

The responsibilities are split so each piece is small and testable:

- `subsys/actuator/` — the generic API, state machine wiring, callbacks, group
  logic, and an optional `actuator` shell command for inspection.
- `lib/actuator/` — pure helper logic with no `struct device` dependency
  (state-machine transitions, capability checks, unit conversion), unit-tested
  on its own.
- `drivers/actuator/<backend>/` — the backends. Each implements
  `struct actuator_driver_api` (the internal contract in
  `include/zephyr/actuator/actuator.h`) and embeds the subsystem's common
  per-device data as the first field of its own data struct. The backend does
  hardware I/O; the subsystem owns state, limits, locking, and callbacks.

Current backends: H-bridge DC motor (e.g. TB6612, with optional encoder and
current sense), Dynamixel, Waveshare bus servo, and a fake backend used by the
tests.

## Adding a backend

1. Add a devicetree binding (see `dts/bindings/actuator/`, which include
   `actuator-common.yaml`).
2. Implement `struct actuator_driver_api`. Leave optional ops (`set_drive_mode`,
   the `group_*` fast paths) `NULL` if unsupported — and don't advertise the
   matching capability bit.
3. Embed `struct actuator_common_data` as the first member of your data struct.
4. Advertise the capability bitmask that matches what you implemented.
