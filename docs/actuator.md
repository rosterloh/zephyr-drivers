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
runs against any backend. A backend with no closed loop (e.g. the H-bridge)
offers open-loop `duty` instead (-1.0..1.0 of supply) rather than pretending a
duty cycle is a velocity. See `include/zephyr/actuator/actuator.h`.

## Core ideas

### Capabilities

Not every actuator can do everything — a cheap servo may only do position, an
FOC controller may need a startup alignment phase. Each backend advertises what
it supports as a capability bitmask (`ACTUATOR_CAP_POSITION`,
`ACTUATOR_CAP_VELOCITY`, `ACTUATOR_CAP_EFFORT`, `ACTUATOR_CAP_DUTY`,
`ACTUATOR_CAP_NEEDS_ALIGN`, `ACTUATOR_CAP_FAULT_LATCHING`,
`ACTUATOR_CAP_DRIVE_MODE`, `ACTUATOR_CAP_GROUP_NATIVE`). The subsystem checks the requested operation
against the bitmask and returns `-ENOTSUP` rather than dispatching something the
hardware can't do. Query at runtime with `actuator_get_capabilities()`.

### State machine

The subsystem owns a small state machine — `DISABLED`, `READY`, `ALIGNING`,
`ACTIVE`, `FAULT`, `STOPPED` — and drives it from events the backend reports
(enable, disable, setpoint, aligned, fault, clear-fault, stop, clear-stop,
reference). The transition logic is a **pure function** (`actuator_sm_step()`),
which is why it lives in `lib/actuator` and is unit-tested in isolation.
Backends never invent their own state semantics; they report what happened and
the subsystem decides the transition.

| event \ state | DISABLED | READY | ALIGNING | ACTIVE | FAULT | STOPPED |
| --- | --- | --- | --- | --- | --- | --- |
| enable | READY¹ | READY¹ | READY¹ | READY¹ | -EPERM | -EPERM |
| setpoint | -EPERM | ACTIVE | -EAGAIN | ACTIVE | -EPERM | -EPERM |
| disable | DISABLED | DISABLED | DISABLED | DISABLED | DISABLED | STOPPED |
| fault | FAULT | FAULT | FAULT | FAULT | FAULT | STOPPED |
| clear-fault | -EINVAL | -EINVAL | -EINVAL | -EINVAL | DISABLED | -EINVAL |
| stop | STOPPED | STOPPED | STOPPED | STOPPED | STOPPED | STOPPED |
| clear-stop | -EINVAL | -EINVAL | -EINVAL | -EINVAL | -EINVAL | DISABLED² |
| reference | -EPERM | ALIGNING | -EBUSY | ALIGNING | -EPERM | -EPERM |

¹ `ALIGNING` for a backend with `ACTUATOR_CAP_NEEDS_ALIGN`.
² `FAULT` if a fault was outstanding when stopped or reported while stopped.

**Enabling is explicit.** A setpoint never enables an actuator: call
`actuator_enable()` first. (Before STOPPED existed a setpoint walked a DISABLED
actuator up to ACTIVE, which made "disabled" no guarantee that nothing moves.)

### Stopping

`actuator_stop()` latches `STOPPED` under the device lock **before** any I/O,
then runs the backend's own controlled stop (`disable` for a backend without
one). From the latch on, every enable and setpoint is refused, and a fault or a
disable does not release it: only `actuator_clear_stop()` does. A fault
outstanding when stopped, or reported while stopped, is kept and surfaces as `FAULT` on the clear, so
`actuator_clear_fault()` still resets the drive. `actuator_stop_latch()` takes
the latch with no I/O, for latching several actuators before any of their
blocking stops, or holding one between paced retries of a failing stop; the
caller still owes an `actuator_stop()`. `actuator_stop()` then runs the
member's group fault policy, as a fault does; the latch alone does not, because
`DISABLE_ALL` and `ESTOP` disable the peers. A stop or disable that lands while
`actuator_enable()`'s backend op is running is redone after it, and the enable
returns `-EPERM`.

### Referencing

A backend whose position means nothing until homed advertises
`ACTUATOR_CAP_NEEDS_REFERENCE`. Until `actuator_reference()` (or
`actuator_group_reference()`) succeeds, its position setpoints get `-EACCES`.
Referencing blocks, moves the axis, and runs in `ALIGNING`, so setpoints get
`-EAGAIN` meanwhile. A failure leaves `FAULT` with `ACTUATOR_FAULT_REFERENCE`; a
stop leaves `STOPPED`; either way the reference is gone.
`actuator_invalidate_reference()` drops it for reasons only the application can
see, such as a coupled load found racked.

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

**Comms health** is read from the feedback, not a separate call: a sample older
than a few update periods (`timestamp_us`) means the backend has stopped
hearing from the hardware, and a backend that knows the link is down reports
`ACTUATOR_FAULT_COMM`.

### Limits

`actuator_set_limits()` takes a `struct actuator_limits`; start from
`ACTUATOR_LIMITS_NONE`, as `{0}` means a limit of zero. A backend with a
`set_limits` op applies them itself (it may refuse what its hardware cannot
express with `-ENOTSUP`). Without one, the subsystem keeps position, velocity
and effort limits and refuses setpoints outside them with `-ERANGE`, and
refuses acceleration limits it cannot enforce with `-ENOTSUP`.

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
Either way, every member first passes the per-device gates (state, reference,
limits, capability) or nothing is written: a coupled group is never half
written because one member was refused. A native path must also hold the same guarantees as the loop: it re-checks each
member's state before each write, so a member stopped mid-group is not written
after it.

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
current sense), Dynamixel, Waveshare bus servo, StepperOnline/Leadshine iCL-RS
integrated closed-loop steppers over Modbus RTU (`icl57`, with drive-side
homing and an `icl57` shell), and a fake backend used by the tests.

## Adding a backend

Contracts every backend keeps:

- **Enable holds position.** Enabling, or enabling after a stop or fault clear,
  must not resume a target set before it. A backend that cannot guarantee
  that refuses the enable.
- **A stop lands after the transaction in progress**, never between the halves
  of one.
- **Referencing** reports each device it moves with
  `actuator_report_reference_begin()` and `actuator_report_referenced()`.

1. Add a devicetree binding (see `dts/bindings/actuator/`, which include
   `actuator-common.yaml`).
2. Implement `struct actuator_driver_api`. Leave optional ops (`set_drive_mode`,
   the `group_*` fast paths) `NULL` if unsupported — and don't advertise the
   matching capability bit.
3. Embed `struct actuator_common_data` as the first member of your data struct.
4. Advertise the capability bitmask that matches what you implemented.
