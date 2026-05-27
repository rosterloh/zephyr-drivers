# Actuator drive-mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `NORMAL`/`BRAKE`/`COAST` drive-mode primitive to the actuator API, and rework the H-bridge backend to drive TB6612FNG-class chips (PWM + IN1/IN2 + optional STBY) while keeping the simpler PWM + single-GPIO path for chips that only expose DIR.

**Architecture:** Drive mode is exposed as a new `__syscall` on the public actuator API, gated by a new capability bit `ACTUATOR_CAP_DRIVE_MODE`. The subsystem checks the cap and the state machine, then calls a new optional `set_drive_mode` vtable op on the backend. Drive mode is orthogonal to the actuator state machine (no new state); setting a position/velocity/effort setpoint implicitly returns the stage to `NORMAL` before applying the setpoint. The H-bridge binding renames `dir-gpios` → `in1-gpios`, adds optional `in2-gpios` (presence selects IN1/IN2 signalling and unlocks the cap) and optional `stby-gpios`. The fake backend gains a tiny implementation so the subsystem tests exercise the full path.

**Tech Stack:** Zephyr RTOS 4.x, C99, `ztest` (twister), `clang-format`, devicetree YAML bindings.

**Spec:** [`docs/superpowers/specs/2026-05-27-actuator-drive-mode-design.md`](../specs/2026-05-27-actuator-drive-mode-design.md)

---

## Working directory and commands

Everything below is run from the **workspace root** at
`/home/rio/src/github/rosterloh/zephyr-applications`, not the rosterloh-drivers
checkout. Use `uv run` for every Python tool. Paths in the plan are repeated
from the workspace root where they affect commands (`deps/modules/lib/rosterloh-drivers/...`).

Recurring commands:

- Run a focused twister suite:
  `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
- Run **all** actuator-related tests (subsys + lib + driver smoke):
  `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests -p native_sim --inline-logs`
- Build the app that consumes the actuator API on hardware:
  `uv run poe agent-build motor_controller`
- Format check a file:
  `uv run clang-format --dry-run --Werror <path>`

## File map

| File | Action | Responsibility |
| --- | --- | --- |
| `include/zephyr/actuator/actuator_types.h` | edit | New `enum actuator_drive_mode`, `ACTUATOR_CAP_DRIVE_MODE` bit |
| `include/zephyr/actuator/actuator.h` | edit | New `__syscall actuator_set_drive_mode` declaration |
| `drivers/actuator/actuator_internal.h` | edit | New `set_drive_mode` vtable entry |
| `subsys/actuator/actuator.c` | edit | `z_impl_actuator_set_drive_mode`; auto-NORMAL inside `set_setpoint_typed` |
| `subsys/actuator/actuator_shell.c` | edit | `actuator mode` subcommand; `DRIVE_MODE` in caps print |
| `drivers/actuator/fake/actuator_fake.c` | edit | Implement `set_drive_mode`; advertise cap; expose `fake_get_drive_mode` for tests |
| `dts/bindings/actuator/rosterloh,actuator-fake.yaml` | edit | Bump default `capabilities` to include cap bit |
| `dts/bindings/actuator/rosterloh,actuator-hbridge.yaml` | edit | Rename `dir-gpios` → `in1-gpios`; add `in2-gpios`, `stby-gpios` |
| `drivers/actuator/hbridge/actuator_hbridge.c` | edit | Read in1/in2/stby from DT; IN1/IN2 signalling; `set_drive_mode` op; STBY in enable/disable |
| `tests/subsys/actuator/src/main.c` | edit | New ZTESTs for cap check / state gating / success / auto-NORMAL |
| `tests/subsys/actuator/boards/native_sim.overlay` | edit | Add a third fake instance without the cap, bump cap on existing two |
| `tests/drivers/actuator/hbridge/boards/native_sim.overlay` | edit | Rename `dir-gpios` → `in1-gpios`; add a second instance using in1+in2+stby |

---

## Task 1: Add enum and capability bit

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/include/zephyr/actuator/actuator_types.h`

The change is mechanical: add the new enum and the new cap bit. No test on its own — covered by Task 3's compile.

- [ ] **Step 1: Add the enum and capability bit**

Open `include/zephyr/actuator/actuator_types.h`. After the existing
`#define ACTUATOR_CAP_FAULT_LATCHING BIT(5)` line, add:

```c
#define ACTUATOR_CAP_DRIVE_MODE     BIT(6) /**< supports actuator_set_drive_mode  */
```

After the existing `enum actuator_state { ... }` block, add:

```c
/** Output policy of the power stage. Orthogonal to actuator_state. */
enum actuator_drive_mode {
	ACTUATOR_DRIVE_MODE_NORMAL = 0, /**< PWM-controlled output (default) */
	ACTUATOR_DRIVE_MODE_BRAKE,      /**< short motor windings to GND     */
	ACTUATOR_DRIVE_MODE_COAST,      /**< high-Z, free-wheel              */
};
```

- [ ] **Step 2: Verify nothing breaks**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests -p native_sim --inline-logs`
Expected: all suites still PASS.

- [ ] **Step 3: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add include/zephyr/actuator/actuator_types.h
git commit -m "actuator: add drive_mode enum and ACTUATOR_CAP_DRIVE_MODE"
```

---

## Task 2: Add syscall declaration and vtable entry

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/include/zephyr/actuator/actuator.h`
- Modify: `deps/modules/lib/rosterloh-drivers/drivers/actuator/actuator_internal.h`
- Modify: `deps/modules/lib/rosterloh-drivers/subsys/actuator/actuator.c` (stub impl returning -ENOTSUP)

- [ ] **Step 1: Add the syscall declaration**

In `include/zephyr/actuator/actuator.h`, after the existing
`__syscall int actuator_set_effort(...);` line, add:

```c
/**
 * Set the output policy of the actuator's power stage.
 *
 * Orthogonal to the actuator state machine: setting a drive mode does not
 * change actuator_get_state(). Setting any setpoint (position/velocity/effort)
 * implicitly returns the stage to ACTUATOR_DRIVE_MODE_NORMAL.
 *
 * @retval 0         Mode applied.
 * @retval -ENOTSUP  Backend does not advertise ACTUATOR_CAP_DRIVE_MODE.
 * @retval -EPERM    Actuator is in DISABLED or FAULT state.
 */
__syscall int actuator_set_drive_mode(const struct device *dev,
				      enum actuator_drive_mode mode);
```

- [ ] **Step 2: Add the vtable entry**

In `drivers/actuator/actuator_internal.h`, inside `struct actuator_driver_api`,
after the existing `int (*set_limits)(...);` line, add:

```c
	/** Optional. NULL = unsupported; backend must also not advertise
	 *  ACTUATOR_CAP_DRIVE_MODE if NULL. */
	int (*set_drive_mode)(const struct device *dev, enum actuator_drive_mode mode);
```

- [ ] **Step 3: Add a stub subsys implementation that always returns -ENOTSUP**

In `subsys/actuator/actuator.c`, after the existing
`int z_impl_actuator_set_limits(...) { ... }` function, add:

```c
int z_impl_actuator_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);
	return -ENOTSUP;
}
```

- [ ] **Step 4: Verify compile**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests -p native_sim --inline-logs`
Expected: all suites still PASS. (The new syscall exists but no caller yet.)

- [ ] **Step 5: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add include/zephyr/actuator/actuator.h \
        drivers/actuator/actuator_internal.h \
        subsys/actuator/actuator.c
git commit -m "actuator: declare actuator_set_drive_mode syscall and vtable op"
```

---

## Task 3: Subsystem cap check (TDD)

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/tests/subsys/actuator/src/main.c`
- Modify: `deps/modules/lib/rosterloh-drivers/subsys/actuator/actuator.c`

The current fake has caps `<7>` = POSITION|VELOCITY|EFFORT, **without**
`ACTUATOR_CAP_DRIVE_MODE`. So we can write the cap-rejection test before
touching the fake.

- [ ] **Step 1: Write the failing test**

In `tests/subsys/actuator/src/main.c`, at the end of the file, add:

```c
ZTEST(actuator_subsys, test_set_drive_mode_rejected_without_cap)
{
	/* fake0 currently has caps = 7 (POSITION|VELOCITY|EFFORT), no DRIVE_MODE. */
	zassert_ok(actuator_enable(FAKE0));
	zassert_equal(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE), -ENOTSUP);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
Expected: the new test passes already (subsys stub from Task 2 returns -ENOTSUP for everyone). That's fine — the test still locks the contract in. Move on.

If for some reason it fails with a different errno, fix the stub before continuing.

- [ ] **Step 3: Replace the stub with a real cap check**

In `subsys/actuator/actuator.c`, replace the stub `z_impl_actuator_set_drive_mode`
with:

```c
int z_impl_actuator_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	struct actuator_common_data *cd = common(dev);

	if ((cd->caps & ACTUATOR_CAP_DRIVE_MODE) == 0) {
		return -ENOTSUP;
	}
	if (api(dev)->set_drive_mode == NULL) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&cd->lock);
	enum actuator_state s = cd->state;
	k_spin_unlock(&cd->lock, key);

	if (s == ACTUATOR_STATE_DISABLED || s == ACTUATOR_STATE_FAULT) {
		return -EPERM;
	}

	int err = api(dev)->set_drive_mode(dev, mode);
	if (err != 0) {
		actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT, ACTUATOR_FAULT_DRIVER(0));
	}
	return err;
}
```

- [ ] **Step 4: Run the test, verify still passes**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add tests/subsys/actuator/src/main.c subsys/actuator/actuator.c
git commit -m "actuator: implement set_drive_mode subsys cap/state gating"
```

---

## Task 4: Fake backend implements drive_mode + test getter

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/drivers/actuator/fake/actuator_fake.c`
- Modify: `deps/modules/lib/rosterloh-drivers/tests/subsys/actuator/boards/native_sim.overlay`
- Modify: `deps/modules/lib/rosterloh-drivers/tests/subsys/actuator/src/main.c`

Plan: the fake stores the last requested mode in a per-instance field and
exposes it via `fake_get_drive_mode(dev)`, mirroring the existing
`fake_force_fault(dev)` pattern. The test overlay opts fake0 and fake1 into the
new cap (bit 6 → 0x40, combined with 0x07 = 0x47). A third fake instance
without the cap is added so the cap-rejection test stays meaningful even after
fake0 gains the cap.

- [ ] **Step 1: Write the failing tests**

In `tests/subsys/actuator/src/main.c`:

a) Replace the `FAKE0`/`FAKE1` macros block with this expanded version (adds a
third instance):

```c
#define FAKE0     DEVICE_DT_GET(DT_NODELABEL(fake0))
#define FAKE1     DEVICE_DT_GET(DT_NODELABEL(fake1))
#define FAKE_NODM DEVICE_DT_GET(DT_NODELABEL(fake_nodm))
```

b) Extend `before_each` so the new instance is also reset between tests.
Replace the existing function with:

```c
static void before_each(void *data)
{
	ARG_UNUSED(data);
	/* Ensure each test starts from a known DISABLED state. */
	actuator_disable(FAKE0);
	actuator_disable(FAKE1);
	actuator_disable(FAKE_NODM);
}
```

c) Update the existing rejection test to use FAKE_NODM:

```c
ZTEST(actuator_subsys, test_set_drive_mode_rejected_without_cap)
{
	zassert_ok(actuator_enable(FAKE_NODM));
	zassert_equal(actuator_set_drive_mode(FAKE_NODM, ACTUATOR_DRIVE_MODE_BRAKE),
		      -ENOTSUP);
}
```

d) At the end of the file, add the success-path tests and the extern:

```c
extern enum actuator_drive_mode fake_get_drive_mode(const struct device *dev);

ZTEST(actuator_subsys, test_set_drive_mode_brake_in_ready)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_READY,
		      "drive mode must not change actuator state");
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_BRAKE);
}

ZTEST(actuator_subsys, test_set_drive_mode_coast_in_active)
{
	zassert_ok(actuator_set_position(FAKE0, 1.0f)); /* promotes to ACTIVE */
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_ACTIVE);
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_COAST));
	zassert_equal(actuator_get_state(FAKE0), ACTUATOR_STATE_ACTIVE,
		      "drive mode must not change actuator state");
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_COAST);
}

ZTEST(actuator_subsys, test_set_drive_mode_rejected_when_disabled)
{
	zassert_ok(actuator_disable(FAKE0));
	zassert_equal(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE),
		      -EPERM);
}
```

- [ ] **Step 2: Update the test overlay**

Replace the entire contents of
`tests/subsys/actuator/boards/native_sim.overlay` with:

```dts
/ {
	fake0: actuator0 {
		compatible = "rosterloh,actuator-fake";
		status = "okay";
		label = "fake0";
		default-mode = "position";
		capabilities = <0x47>; /* POSITION | VELOCITY | EFFORT | DRIVE_MODE */
	};
	fake1: actuator1 {
		compatible = "rosterloh,actuator-fake";
		status = "okay";
		label = "fake1";
		default-mode = "position";
		capabilities = <0x47>;
	};
	fake_nodm: actuator2 {
		compatible = "rosterloh,actuator-fake";
		status = "okay";
		label = "fake_nodm";
		default-mode = "position";
		capabilities = <7>; /* POSITION | VELOCITY | EFFORT; NO drive_mode */
	};
};
```

- [ ] **Step 3: Run the tests, verify they fail**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
Expected: build FAIL — `fake_get_drive_mode` is undefined.

- [ ] **Step 4: Implement set_drive_mode in the fake**

In `drivers/actuator/fake/actuator_fake.c`:

a) Add a `last_drive_mode` field to `struct fake_data`:

```c
struct fake_data {
	struct actuator_common_data common;
	struct actuator_cb_storage cb_storage;
	struct actuator_cb_node cb_pool[FAKE_CB_POOL];
	float last_setpoint;
	enum actuator_mode last_mode;
	enum actuator_drive_mode last_drive_mode;
};
```

b) After `fake_set_setpoint`, add the op implementation:

```c
static int fake_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	struct fake_data *d = dev->data;

	d->last_drive_mode = mode;
	return 0;
}
```

c) Add the op to the vtable:

```c
static const struct actuator_driver_api fake_api = {
	.enable = fake_enable,
	.disable = fake_disable,
	.clear_fault = fake_clear_fault,
	.set_setpoint = fake_set_setpoint,
	.read_feedback = fake_read_feedback,
	.set_drive_mode = fake_set_drive_mode,
};
```

d) At the very bottom of the file, after the existing `fake_force_fault`
function, add the test-facing getter:

```c
enum actuator_drive_mode fake_get_drive_mode(const struct device *dev)
{
	const struct fake_data *d = dev->data;

	return d->last_drive_mode;
}
```

- [ ] **Step 5: Run the tests, verify they pass**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
Expected: PASS for all `actuator_subsys` tests including the new ones.

- [ ] **Step 6: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add drivers/actuator/fake/actuator_fake.c \
        tests/subsys/actuator/boards/native_sim.overlay \
        tests/subsys/actuator/src/main.c
git commit -m "actuator: fake backend implements set_drive_mode + test getter"
```

---

## Task 5: Auto-NORMAL on setpoint (TDD)

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/tests/subsys/actuator/src/main.c`
- Modify: `deps/modules/lib/rosterloh-drivers/subsys/actuator/actuator.c`

- [ ] **Step 1: Write the failing test**

In `tests/subsys/actuator/src/main.c`, at the end of the file, add:

```c
ZTEST(actuator_subsys, test_setpoint_auto_clears_drive_mode)
{
	zassert_ok(actuator_enable(FAKE0));
	zassert_ok(actuator_set_drive_mode(FAKE0, ACTUATOR_DRIVE_MODE_BRAKE));
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_BRAKE);

	zassert_ok(actuator_set_position(FAKE0, 0.5f));
	zassert_equal(fake_get_drive_mode(FAKE0), ACTUATOR_DRIVE_MODE_NORMAL,
		      "setpoint must implicitly return to NORMAL");
}
```

- [ ] **Step 2: Run the test, verify it fails**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
Expected: FAIL — `fake_get_drive_mode` still reports BRAKE after the setpoint call.

- [ ] **Step 3: Add the auto-NORMAL hook in set_setpoint_typed**

In `subsys/actuator/actuator.c`, in `set_setpoint_typed`, locate the block
right before the call to `api(dev)->set_setpoint(dev, mode, value)`:

```c
	if (rc < 0) {
		return rc;
	}

	err = api(dev)->set_setpoint(dev, mode, value);
```

Insert between the early-return and the api call so the final block reads:

```c
	if (rc < 0) {
		return rc;
	}

	if ((cd->caps & ACTUATOR_CAP_DRIVE_MODE) && api(dev)->set_drive_mode != NULL) {
		err = api(dev)->set_drive_mode(dev, ACTUATOR_DRIVE_MODE_NORMAL);
		if (err != 0) {
			actuator_report_state(dev, ACTUATOR_SM_EVT_FAULT,
					      ACTUATOR_FAULT_DRIVER(0));
			return err;
		}
	}

	err = api(dev)->set_setpoint(dev, mode, value);
```

- [ ] **Step 4: Run the tests, verify they pass**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/subsys/actuator -p native_sim --inline-logs`
Expected: PASS, all `actuator_subsys` cases including the new one.

- [ ] **Step 5: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add tests/subsys/actuator/src/main.c subsys/actuator/actuator.c
git commit -m "actuator: implicit NORMAL on setpoint when backend supports drive_mode"
```

---

## Task 6: Rename hbridge `dir-gpios` → `in1-gpios`; add `in2-gpios`, `stby-gpios`

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/dts/bindings/actuator/rosterloh,actuator-hbridge.yaml`
- Modify: `deps/modules/lib/rosterloh-drivers/drivers/actuator/hbridge/actuator_hbridge.c` (rename internal field only — no behavior change yet)
- Modify: `deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge/boards/native_sim.overlay`

This task is mechanical rename only. New props are declared in the YAML so
future tasks can use them; driver still reads only `in1-gpios` for now.

- [ ] **Step 1: Update the binding**

Replace the entire contents of
`dts/bindings/actuator/rosterloh,actuator-hbridge.yaml` with:

```yaml
description: |
  H-bridge DC motor with optional encoder and current sense.

  Signalling is selected by the presence of in2-gpios:
    - Only in1-gpios present  → PWM + DIR signalling (one direction GPIO).
      The driver does not advertise ACTUATOR_CAP_DRIVE_MODE in this mode.
    - in1-gpios and in2-gpios → PWM + IN1/IN2 signalling (TB6612FNG-class).
      The driver advertises ACTUATOR_CAP_DRIVE_MODE: BRAKE drives both
      inputs high (short brake), COAST drives both low (high-Z).

  stby-gpios is optional in either signalling. When wired, the driver
  asserts it on enable() and deasserts it on disable(). Use the
  GPIO_ACTIVE_LOW dt flag if the silicon's standby pin is active-low
  (e.g. TB6612 STBY).

compatible: "rosterloh,actuator-hbridge"

include: [actuator-common.yaml]

properties:
  pwms:
    type: phandle-array
    required: true
    description: PWM channel driving the H-bridge enable/PWM input.

  in1-gpios:
    type: phandle-array
    required: true
    description: |
      Bridge input 1. With in2-gpios present, this is the TB6612-style IN1.
      With in2-gpios absent, this is the single DIR pin.

  in2-gpios:
    type: phandle-array
    description: |
      Bridge input 2 (TB6612-style IN2). When present, the driver uses
      PWM + IN1/IN2 signalling and advertises ACTUATOR_CAP_DRIVE_MODE.

  stby-gpios:
    type: phandle-array
    description: |
      Optional standby/enable line. Asserted on actuator_enable, deasserted
      on actuator_disable. May be shared across instances by the application.

  io-channels:
    type: phandle-array
    description: |
      Optional ADC channel for current sense. Enables CAP_EFFORT and
      ACTUATOR_FAULT_OVERCURRENT detection.

  encoder:
    type: phandle
    description: |
      Optional phandle to a qdec sensor node. Enables CAP_POSITION and
      CAP_VELOCITY.

  torque-constant-mnm-per-a:
    type: int
  max-current-ma:
    type: int

  current-sense-mohms:
    type: int
    description: |
      Shunt resistor value in milliohms used by the ADC current-sense path.
      Required when io-channels is set and CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
      is enabled. For an INA-style amplifier, set this to (R_shunt_mohms /
      amp_gain).

  pwm-period-ns:
    type: int
    default: 50000  # 20 kHz
```

- [ ] **Step 2: Update the driver field name from `dir` to `in1`**

In `drivers/actuator/hbridge/actuator_hbridge.c`:

a) In `struct hbridge_config`, replace:

```c
	struct gpio_dt_spec dir;
```

with:

```c
	struct gpio_dt_spec in1;
```

b) In `hbridge_set_pwm`, replace:

```c
	int dir = (duty >= 0.0f) ? 1 : 0;
	uint32_t pulse_ns = (uint32_t)(fabsf(duty) * (float)cfg->pwm_period_ns);
	int err = gpio_pin_set_dt(&cfg->dir, dir);
```

with:

```c
	int dir = (duty >= 0.0f) ? 1 : 0;
	uint32_t pulse_ns = (uint32_t)(fabsf(duty) * (float)cfg->pwm_period_ns);
	int err = gpio_pin_set_dt(&cfg->in1, dir);
```

c) In `hb_init`, replace:

```c
	if (!device_is_ready(cfg->pwm.dev) || !device_is_ready(cfg->dir.port)) {
		return -ENODEV;
	}
```

with:

```c
	if (!device_is_ready(cfg->pwm.dev) || !device_is_ready(cfg->in1.port)) {
		return -ENODEV;
	}
```

and:

```c
	int err = gpio_pin_configure_dt(&cfg->dir, GPIO_OUTPUT_INACTIVE);
```

with:

```c
	int err = gpio_pin_configure_dt(&cfg->in1, GPIO_OUTPUT_INACTIVE);
```

d) In the `HB_DEFINE(inst)` macro, replace:

```c
		.dir = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),                                     \
```

with:

```c
		.in1 = GPIO_DT_SPEC_INST_GET(inst, in1_gpios),                                     \
```

- [ ] **Step 3: Update the test overlay**

Replace the entire contents of
`tests/drivers/actuator/hbridge/boards/native_sim.overlay` with:

```dts
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/ {
	pwm_fake: pwm-fake {
		compatible = "zephyr,fake-pwm";
		#pwm-cells = <2>;
		frequency = <20000>;
		status = "okay";
	};
	gpio_emul: gpio_emul {
		compatible = "zephyr,gpio-emul";
		gpio-controller;
		#gpio-cells = <2>;
		ngpios = <8>;
		status = "okay";
	};
	motor: motor {
		compatible = "rosterloh,actuator-hbridge";
		pwms = <&pwm_fake 0 50000>;
		in1-gpios = <&gpio_emul 0 GPIO_ACTIVE_HIGH>;
	};
};
```

- [ ] **Step 4: Verify hbridge build still works**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge -p native_sim --inline-logs`
Expected: PASS (suite is a build smoke test; the existing `test_smoke` skips).

- [ ] **Step 5: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add dts/bindings/actuator/rosterloh,actuator-hbridge.yaml \
        drivers/actuator/hbridge/actuator_hbridge.c \
        tests/drivers/actuator/hbridge/boards/native_sim.overlay
git commit -m "drivers: hbridge: rename dir-gpios to in1-gpios; declare in2/stby"
```

---

## Task 7: Hbridge IN1/IN2 signalling + cap advertisement

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/drivers/actuator/hbridge/actuator_hbridge.c`
- Modify: `deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge/boards/native_sim.overlay` (add second instance using in1+in2)

The driver now needs to detect IN1/IN2 mode at init and drive both pins
accordingly. The cap is advertised only when in2 is present.

- [ ] **Step 1: Add the second hbridge instance to the test overlay**

In `tests/drivers/actuator/hbridge/boards/native_sim.overlay`, inside the
device tree (after the existing `motor: motor { ... };` block, before the
closing `};`), add:

```dts
	motor_in12: motor_in12 {
		compatible = "rosterloh,actuator-hbridge";
		pwms = <&pwm_fake 0 50000>;
		in1-gpios = <&gpio_emul 1 GPIO_ACTIVE_HIGH>;
		in2-gpios = <&gpio_emul 2 GPIO_ACTIVE_HIGH>;
	};
```

This forces the IN1/IN2 build path to compile in CI.

- [ ] **Step 2: Add the in2 field and the has_in2 flag**

In `drivers/actuator/hbridge/actuator_hbridge.c`, expand `struct hbridge_config`
(after the existing `in1` field) to:

```c
struct hbridge_config {
	struct actuator_cb_offsets cb_offsets; /* must be first */
	struct pwm_dt_spec pwm;
	struct gpio_dt_spec in1;
	struct gpio_dt_spec in2; /* port == NULL when absent */
	bool has_in2;
#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
	const struct device *encoder; /* may be NULL */
#endif
#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
	struct adc_dt_spec adc;
	uint32_t current_sense_mohms;
	int32_t max_current_ma;
	int32_t torque_const_mnm_per_a;
	bool has_current_sense;
#endif
	uint32_t pwm_period_ns;
	uint32_t update_period_ms;
	enum actuator_mode default_mode;
	uint32_t caps;
};
```

- [ ] **Step 3: Update hbridge_set_pwm to drive IN1/IN2 when present**

Replace `hbridge_set_pwm` with:

```c
static int hbridge_set_pwm(const struct hbridge_config *cfg, float duty)
{
	if (duty > 1.0f) {
		duty = 1.0f;
	}
	if (duty < -1.0f) {
		duty = -1.0f;
	}
	int fwd = (duty >= 0.0f) ? 1 : 0;
	uint32_t pulse_ns = (uint32_t)(fabsf(duty) * (float)cfg->pwm_period_ns);
	int err = gpio_pin_set_dt(&cfg->in1, fwd);

	if (err) {
		return err;
	}
	if (cfg->has_in2) {
		err = gpio_pin_set_dt(&cfg->in2, !fwd);
		if (err) {
			return err;
		}
	}
	return pwm_set_dt(&cfg->pwm, cfg->pwm_period_ns, pulse_ns);
}
```

Rationale for the IN1/IN2 values: with TB6612, `(IN1=1, IN2=0)` = CW PWM and
`(IN1=0, IN2=1)` = CCW PWM. The PWM duty modulates the high-side that the IN
pair selects.

- [ ] **Step 4: Configure in2 in hb_init**

In `hb_init`, after the `gpio_pin_configure_dt(&cfg->in1, GPIO_OUTPUT_INACTIVE)`
block, add:

```c
	if (cfg->has_in2) {
		if (!device_is_ready(cfg->in2.port)) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&cfg->in2, GPIO_OUTPUT_INACTIVE);
		if (err) {
			return err;
		}
	}
```

- [ ] **Step 5: Wire in2 + cap into the device-tree macros**

In `drivers/actuator/hbridge/actuator_hbridge.c`, just above the existing
`#define HB_HAS_ENCODER(inst)` macro, add:

```c
#define HB_HAS_IN2(inst) DT_INST_NODE_HAS_PROP(inst, in2_gpios)
```

Replace the existing `HB_CAPS(inst)` macro with:

```c
#define HB_CAPS(inst)                                                                              \
	(ACTUATOR_CAP_VELOCITY |                                                                   \
	 ((HB_HAS_CURRENT_SENSE(inst) && DT_INST_PROP_OR(inst, torque_constant_mnm_per_a, 0) > 0)  \
		  ? ACTUATOR_CAP_EFFORT                                                            \
		  : 0) |                                                                           \
	 (HB_HAS_ENCODER(inst) ? ACTUATOR_CAP_POSITION : 0) |                                      \
	 (HB_HAS_IN2(inst) ? ACTUATOR_CAP_DRIVE_MODE : 0))
```

In the `HB_DEFINE(inst)` macro, just after the `.in1 = ...` line, add:

```c
		.in2 = COND_CODE_1(HB_HAS_IN2(inst),                                               \
				   (GPIO_DT_SPEC_INST_GET(inst, in2_gpios)),                       \
				   ({.port = NULL})),                                              \
		.has_in2 = HB_HAS_IN2(inst),                                                       \
```

- [ ] **Step 6: Verify the build**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge -p native_sim --inline-logs`
Expected: PASS. Both `motor` and `motor_in12` instances must compile.

- [ ] **Step 7: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add drivers/actuator/hbridge/actuator_hbridge.c \
        tests/drivers/actuator/hbridge/boards/native_sim.overlay
git commit -m "drivers: hbridge: PWM+IN1/IN2 signalling and DRIVE_MODE cap"
```

---

## Task 8: Hbridge `set_drive_mode` op

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/drivers/actuator/hbridge/actuator_hbridge.c`

- [ ] **Step 1: Add the op implementation**

In `drivers/actuator/hbridge/actuator_hbridge.c`, immediately after
`hb_set_setpoint` and before the `static const struct actuator_driver_api hb_api`
line, add:

```c
static int hb_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	const struct hbridge_config *cfg = dev->config;

	if (!cfg->has_in2) {
		/* Single-GPIO (PWM+DIR) variant: cannot independently command
		 * brake or coast; the silicon decides what PWM=0 means. The
		 * subsystem should already have rejected this via the cap, but
		 * guard anyway. */
		return -ENOTSUP;
	}

	int err = pwm_set_dt(&cfg->pwm, cfg->pwm_period_ns, 0);

	if (err) {
		return err;
	}

	int in1_val = 0, in2_val = 0;

	switch (mode) {
	case ACTUATOR_DRIVE_MODE_NORMAL:
	case ACTUATOR_DRIVE_MODE_COAST:
		/* IN1=0, IN2=0 → outputs high-Z. Next set_setpoint reasserts. */
		in1_val = 0;
		in2_val = 0;
		break;
	case ACTUATOR_DRIVE_MODE_BRAKE:
		/* IN1=1, IN2=1 → both low-side FETs on, motor windings shorted. */
		in1_val = 1;
		in2_val = 1;
		break;
	default:
		return -EINVAL;
	}
	err = gpio_pin_set_dt(&cfg->in1, in1_val);
	if (err) {
		return err;
	}
	return gpio_pin_set_dt(&cfg->in2, in2_val);
}
```

- [ ] **Step 2: Add the op to the vtable**

Replace the existing `hb_api` block with:

```c
static const struct actuator_driver_api hb_api = {
	.enable = hb_enable,
	.disable = hb_disable,
	.set_setpoint = hb_set_setpoint,
	.read_feedback = hb_read_feedback,
	.set_drive_mode = hb_set_drive_mode,
};
```

- [ ] **Step 3: Verify the build**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge -p native_sim --inline-logs`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add drivers/actuator/hbridge/actuator_hbridge.c
git commit -m "drivers: hbridge: implement set_drive_mode for IN1/IN2 variant"
```

---

## Task 9: Hbridge STBY GPIO support

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/drivers/actuator/hbridge/actuator_hbridge.c`
- Modify: `deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge/boards/native_sim.overlay` (add stby on motor_in12)

- [ ] **Step 1: Add stby field and has_stby flag**

In `drivers/actuator/hbridge/actuator_hbridge.c`, expand `struct hbridge_config`
to include after `bool has_in2;`:

```c
	struct gpio_dt_spec stby; /* port == NULL when absent */
	bool has_stby;
```

- [ ] **Step 2: Configure stby in hb_init**

In `hb_init`, after the in2 configuration block, add:

```c
	if (cfg->has_stby) {
		if (!device_is_ready(cfg->stby.port)) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&cfg->stby, GPIO_OUTPUT_INACTIVE);
		if (err) {
			return err;
		}
	}
```

- [ ] **Step 3: Assert/deassert stby in enable/disable**

Replace `hb_enable` with:

```c
static int hb_enable(const struct device *dev)
{
	struct hbridge_data *d = dev->data;
	const struct hbridge_config *cfg = dev->config;

	bool need_worker = false;

#ifdef CONFIG_ACTUATOR_HBRIDGE_ENCODER
	if (cfg->encoder != NULL) {
		need_worker = true;
	}
#endif
#ifdef CONFIG_ACTUATOR_HBRIDGE_CURRENT_SENSE
	if (cfg->has_current_sense) {
		need_worker = true;
	}
#endif
	if (cfg->has_stby) {
		int err = gpio_pin_set_dt(&cfg->stby, 1);

		if (err) {
			return err;
		}
	}
	if (need_worker) {
		k_work_schedule(&d->feedback_work, K_MSEC(cfg->update_period_ms));
	}
	return 0;
}
```

Replace `hb_disable` with:

```c
static int hb_disable(const struct device *dev)
{
	struct hbridge_data *d = dev->data;
	const struct hbridge_config *cfg = dev->config;

	(void)pwm_set_dt(&cfg->pwm, cfg->pwm_period_ns, 0);
	k_work_cancel_delayable(&d->feedback_work);
	d->position_valid = false;
	if (cfg->has_stby) {
		(void)gpio_pin_set_dt(&cfg->stby, 0);
	}
	return 0;
}
```

- [ ] **Step 4: Wire stby into the device-tree macros**

In `drivers/actuator/hbridge/actuator_hbridge.c`, just below the existing
`#define HB_HAS_IN2(inst) ...`, add:

```c
#define HB_HAS_STBY(inst) DT_INST_NODE_HAS_PROP(inst, stby_gpios)
```

In `HB_DEFINE(inst)`, just after the `.has_in2 = HB_HAS_IN2(inst),` line, add:

```c
		.stby = COND_CODE_1(HB_HAS_STBY(inst),                                             \
				    (GPIO_DT_SPEC_INST_GET(inst, stby_gpios)),                     \
				    ({.port = NULL})),                                             \
		.has_stby = HB_HAS_STBY(inst),                                                     \
```

- [ ] **Step 5: Add stby to the test overlay's motor_in12 instance**

In `tests/drivers/actuator/hbridge/boards/native_sim.overlay`, update the
`motor_in12` block:

```dts
	motor_in12: motor_in12 {
		compatible = "rosterloh,actuator-hbridge";
		pwms = <&pwm_fake 0 50000>;
		in1-gpios = <&gpio_emul 1 GPIO_ACTIVE_HIGH>;
		in2-gpios = <&gpio_emul 2 GPIO_ACTIVE_HIGH>;
		stby-gpios = <&gpio_emul 3 GPIO_ACTIVE_HIGH>;
	};
```

- [ ] **Step 6: Verify the build**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/actuator/hbridge -p native_sim --inline-logs`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add drivers/actuator/hbridge/actuator_hbridge.c \
        tests/drivers/actuator/hbridge/boards/native_sim.overlay
git commit -m "drivers: hbridge: optional stby-gpios asserted on enable"
```

---

## Task 10: Shell command and caps printer

**Files:**
- Modify: `deps/modules/lib/rosterloh-drivers/subsys/actuator/actuator_shell.c`

- [ ] **Step 1: Add the DRIVE_MODE bit to the caps printer**

In `subsys/actuator/actuator_shell.c`, replace the `cmd_get_caps`
`shell_print` call with:

```c
	shell_print(sh, "caps=0x%02x%s%s%s%s%s%s%s", c,
		    (c & ACTUATOR_CAP_POSITION) ? " POSITION" : "",
		    (c & ACTUATOR_CAP_VELOCITY) ? " VELOCITY" : "",
		    (c & ACTUATOR_CAP_EFFORT) ? " EFFORT" : "",
		    (c & ACTUATOR_CAP_NEEDS_ALIGN) ? " NEEDS_ALIGN" : "",
		    (c & ACTUATOR_CAP_GROUP_NATIVE) ? " GROUP_NATIVE" : "",
		    (c & ACTUATOR_CAP_FAULT_LATCHING) ? " FAULT_LATCHING" : "",
		    (c & ACTUATOR_CAP_DRIVE_MODE) ? " DRIVE_MODE" : "");
```

- [ ] **Step 2: Add the `mode` subcommand handler**

Above `SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds, ...)`, add:

```c
static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "usage: actuator mode <name> <normal|brake|coast>");
		return -EINVAL;
	}
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	enum actuator_drive_mode mode;

	if (strcmp(argv[2], "normal") == 0) {
		mode = ACTUATOR_DRIVE_MODE_NORMAL;
	} else if (strcmp(argv[2], "brake") == 0) {
		mode = ACTUATOR_DRIVE_MODE_BRAKE;
	} else if (strcmp(argv[2], "coast") == 0) {
		mode = ACTUATOR_DRIVE_MODE_COAST;
	} else {
		shell_error(sh, "unknown mode: %s", argv[2]);
		return -EINVAL;
	}
	int err = actuator_set_drive_mode(dev, mode);

	shell_print(sh, "mode: %d", err);
	return err;
}
```

- [ ] **Step 3: Register the subcommand**

Replace the existing `actuator_subcmds` block with:

```c
SHELL_STATIC_SUBCMD_SET_CREATE(
	actuator_subcmds, SHELL_CMD_ARG(enable, NULL, "<name> enable", cmd_enable, 2, 0),
	SHELL_CMD_ARG(disable, NULL, "<name> disable", cmd_disable, 2, 0),
	SHELL_CMD_ARG(clear_fault, NULL, "<name> clear-fault", cmd_clear_fault, 2, 0),
	SHELL_CMD_ARG(set, NULL, "<name> set <mode> <val>", cmd_set, 4, 0),
	SHELL_CMD_ARG(mode, NULL, "<name> mode <normal|brake|coast>", cmd_mode, 3, 0),
	SHELL_CMD(get, &get_subcmds, "<name> get state|feedback|caps", NULL), SHELL_SUBCMD_SET_END);
```

- [ ] **Step 4: Verify the shell still compiles**

Build any consumer app that has CONFIG_ACTUATOR_SHELL=y. The
`motor_controller` app builds the actuator subsys and shell:

Run: `uv run poe agent-build motor_controller`
Expected: build succeeds; tail prints summary of build outputs.

- [ ] **Step 5: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add subsys/actuator/actuator_shell.c
git commit -m "actuator: shell: actuator mode <dev> normal|brake|coast"
```

---

## Task 11: End-to-end verification

**Files:** none modified — this task only runs the existing test/build/format
gates to confirm the whole stack still works.

- [ ] **Step 1: Run the entire actuator test set**

Run: `uv run west twister -T deps/modules/lib/rosterloh-drivers/tests -p native_sim --inline-logs`
Expected: every suite PASS, including the new `actuator_subsys` tests added
across Tasks 3-5, the hbridge smoke build with two instances, and the
pre-existing dxl/state_machine/capabilities/unit_helpers suites.

- [ ] **Step 2: Build the motor_controller app**

Run: `uv run poe agent-build motor_controller`
Expected: build succeeds. This confirms the binding rename did not break the
hardware path that consumes the actuator subsys.

- [ ] **Step 3: clang-format check the touched C files**

Run from the workspace root:

```bash
uv run clang-format --dry-run --Werror \
  deps/modules/lib/rosterloh-drivers/include/zephyr/actuator/actuator_types.h \
  deps/modules/lib/rosterloh-drivers/include/zephyr/actuator/actuator.h \
  deps/modules/lib/rosterloh-drivers/drivers/actuator/actuator_internal.h \
  deps/modules/lib/rosterloh-drivers/subsys/actuator/actuator.c \
  deps/modules/lib/rosterloh-drivers/subsys/actuator/actuator_shell.c \
  deps/modules/lib/rosterloh-drivers/drivers/actuator/fake/actuator_fake.c \
  deps/modules/lib/rosterloh-drivers/drivers/actuator/hbridge/actuator_hbridge.c \
  deps/modules/lib/rosterloh-drivers/tests/subsys/actuator/src/main.c
```

Expected: no output, exit 0. If anything fails, fix the formatting and amend
the relevant commit.

- [ ] **Step 4: Verify git log is tidy**

Run: `cd deps/modules/lib/rosterloh-drivers && git log --oneline -12`
Expected: a clean sequence of focused commits matching the task list:

```
actuator: shell: actuator mode <dev> normal|brake|coast
drivers: hbridge: optional stby-gpios asserted on enable
drivers: hbridge: implement set_drive_mode for IN1/IN2 variant
drivers: hbridge: PWM+IN1/IN2 signalling and DRIVE_MODE cap
drivers: hbridge: rename dir-gpios to in1-gpios; declare in2/stby
actuator: implicit NORMAL on setpoint when backend supports drive_mode
actuator: fake backend implements set_drive_mode + test getter
actuator: implement set_drive_mode subsys cap/state gating
actuator: declare actuator_set_drive_mode syscall and vtable op
actuator: add drive_mode enum and ACTUATOR_CAP_DRIVE_MODE
docs: actuator drive-mode spec
```

No follow-up commit is needed if everything passes — Task 11 is purely
verification.
