# Kinematics Joint-Descriptor Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a Zephyr device declare its own URDF-style joint(s) in devicetree and serialize them to a transport-agnostic CBOR payload so a remote controller can assemble a kinematic tree.

**Architecture:** A lightweight library (`lib/kinematics/`). A `rosterloh,joint` devicetree binding describes each joint; a `DT_FOREACH_STATUS_OKAY` macro builds a `const` descriptor table at compile time (scaled-int DT values → SI floats); a query API exposes the table; a zcbor encoder serializes the whole table to CBOR. No device instances, no syscalls — the data is static and read-only.

**Tech Stack:** Zephyr, devicetree, zcbor 0.9.1 (ships with Zephyr), ztest, native_sim.

## Global Constraints

- Every new file starts with the repo header: `/* Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com> */` and `/* SPDX-License-Identifier: Apache-2.0 */` (two-line block as in existing files).
- C code is formatted with the in-tree `.clang-format`. Verify with `uv run clang-format --dry-run --Werror <files>` from the workspace root.
- All `west`/`twister` commands run through `uv run` from the workspace root `/home/rio/src/github/rosterloh/zephyr-applications` (never bare `west`, never `cd deps/zephyr`).
- Tests target `native_sim`.
- API and CBOR units are **SI** (m, rad, kg, kg·m², N·m, rad/s). Scaled integers exist **only** in devicetree.
- zcbor API is version 0.9.1: `zcbor_tstr_put_term(state, str, maxlen)`, `zcbor_tstr_put_lit(state, "lit")`, `zcbor_uint32_put`, `zcbor_float32_put`, `zcbor_{list,map}_start_encode(state, max)`, `zcbor_{list,map}_end_encode(state, max)`, `ZCBOR_STATE_E(name, backups, buf, len, elems)`. Decode mirrors with `_decode`/`ZCBOR_STATE_D(..., n_flags)`.
- Schema version constant: `KINEMATICS_SCHEMA_VERSION` = `1`.

**Build/test command** (used throughout):
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west twister -p native_sim --clobber-output -v \
  -T deps/modules/lib/rosterloh-drivers/tests/lib/kinematics
```

---

## File Structure

- `dts/bindings/kinematics/rosterloh,joint.yaml` — DT binding (Task 1)
- `include/zephyr/kinematics/joint_types.h` — `struct joint_descriptor`, enums, mask bits, version (Task 1)
- `include/zephyr/kinematics/joint.h` — query + encode API (Task 1 declares query, Task 2 declares encode)
- `lib/kinematics/Kconfig` — `CONFIG_KINEMATICS` (Task 1)
- `lib/kinematics/CMakeLists.txt` — library sources (Task 1, Task 2)
- `lib/kinematics/joint.c` — generated table + query API (Task 1)
- `lib/kinematics/cbor.c` — `kinematics_encode_cbor` (Task 2)
- `Kconfig` (top) — `rsource lib/kinematics/Kconfig` (Task 1)
- `CMakeLists.txt` (top) — `add_subdirectory_ifdef(CONFIG_KINEMATICS lib/kinematics)` (Task 1)
- `tests/lib/kinematics/{CMakeLists.txt,prj.conf,testcase.yaml,app.overlay,src/main.c}` — one ztest binary, table suite (Task 1) + cbor suite (Task 2)
- `samples/kinematics/{CMakeLists.txt,prj.conf,sample.yaml,app.overlay,src/main.c}` — runnable demo (Task 3)

---

## Task 1: DT binding, descriptor table, and query API

**Files:**
- Create: `dts/bindings/kinematics/rosterloh,joint.yaml`
- Create: `include/zephyr/kinematics/joint_types.h`
- Create: `include/zephyr/kinematics/joint.h`
- Create: `lib/kinematics/Kconfig`
- Create: `lib/kinematics/CMakeLists.txt`
- Create: `lib/kinematics/joint.c`
- Modify: `Kconfig` (append one line)
- Modify: `CMakeLists.txt` (append one line)
- Create test: `tests/lib/kinematics/CMakeLists.txt`, `prj.conf`, `testcase.yaml`, `app.overlay`, `src/main.c`

**Interfaces:**
- Produces: `struct joint_descriptor`, `enum joint_type`, `JOINT_HAS_LIMITS/DYNAMICS/ACTUATOR`, `KINEMATICS_SCHEMA_VERSION`, `size_t kinematics_joint_count(void)`, `const struct joint_descriptor *kinematics_joint_get(size_t idx)`.

- [ ] **Step 1: Write the failing test**

Create `tests/lib/kinematics/app.overlay` — fixture joints. Order matters: `joints[0]` is the mandatory-only joint used by the CBOR round-trip test in Task 2.

```dts
/ {
	joint_base: joint-base {
		compatible = "rosterloh,joint";
		link-parent = "world";
		link-child = "base_link";
		joint-type = "fixed";
		axis = <0 0 1000>;
		origin-xyz = <0 0 0>;
		origin-rpy = <0 0 0>;
	};

	joint_shoulder: joint-shoulder {
		compatible = "rosterloh,joint";
		link-parent = "base_link";
		link-child = "upper_arm";
		joint-type = "revolute";
		axis = <0 0 1000>;
		origin-xyz = <50000 0 100000>;          /* 0.05, 0, 0.1 m */
		origin-rpy = <0 0 1570796>;              /* ~pi/2 rad */
		limit-lower = <(-1570796)>;              /* -pi/2 rad */
		limit-upper = <1570796>;                 /*  pi/2 rad */
		limit-velocity = <3141592>;              /* ~pi rad/s */
		limit-effort = <2500>;                   /* 2.5 N*m */
		mass-mg = <250000>;                      /* 0.25 kg */
		com-xyz = <0 0 30000>;                   /* 0, 0, 0.03 m */
		inertia = <1000000 1000000 500000 0 0 0>;/* ixx=iyy=1e-3, izz=5e-4 kg*m^2 */
		actuator = <&fake_act>;
	};

	fake_act: fake-act {
		compatible = "rosterloh,actuator-fake";
		status = "okay";
	};
};
```

Create `tests/lib/kinematics/src/main.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>
#include <zephyr/kinematics/joint.h>

#define EPS 1e-4f

ZTEST_SUITE(kinematics_table, NULL, NULL, NULL, NULL, NULL);

ZTEST(kinematics_table, test_count)
{
	zassert_equal(kinematics_joint_count(), 2, "expected 2 joints");
}

ZTEST(kinematics_table, test_fixed_joint_mandatory)
{
	const struct joint_descriptor *j = kinematics_joint_get(0);

	zassert_not_null(j);
	zassert_str_equal(j->parent_link, "world");
	zassert_str_equal(j->child_link, "base_link");
	zassert_equal(j->type, JOINT_TYPE_FIXED);
	zassert_within(j->axis[2], 1.0f, EPS);
	zassert_within(j->origin_xyz[0], 0.0f, EPS);
	zassert_equal(j->valid_mask, 0, "fixed joint has no optional blocks");
}

ZTEST(kinematics_table, test_revolute_conversions)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_equal(j->type, JOINT_TYPE_REVOLUTE);
	zassert_within(j->origin_xyz[0], 0.05f, EPS);
	zassert_within(j->origin_xyz[2], 0.1f, EPS);
	zassert_within(j->origin_rpy[2], 1.570796f, EPS);
}

ZTEST(kinematics_table, test_revolute_limits_and_mask)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_true(j->valid_mask & JOINT_HAS_LIMITS);
	zassert_within(j->limits.lower, -1.570796f, EPS);
	zassert_within(j->limits.upper, 1.570796f, EPS);
	zassert_within(j->limits.velocity, 3.141592f, EPS);
	zassert_within(j->limits.effort, 2.5f, EPS);
}

ZTEST(kinematics_table, test_revolute_dynamics)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_true(j->valid_mask & JOINT_HAS_DYNAMICS);
	zassert_within(j->dynamics.mass, 0.25f, EPS);
	zassert_within(j->dynamics.com[2], 0.03f, EPS);
	zassert_within(j->dynamics.inertia[0], 1e-3f, EPS);
	zassert_within(j->dynamics.inertia[2], 5e-4f, EPS);
}

ZTEST(kinematics_table, test_actuator_link)
{
	const struct joint_descriptor *j = kinematics_joint_get(1);

	zassert_true(j->valid_mask & JOINT_HAS_ACTUATOR);
	zassert_not_null(j->actuator_name);
	zassert_str_equal(j->actuator_name, "fake-act");
}
```

Create `tests/lib/kinematics/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kinematics_lib)
target_sources(app PRIVATE src/main.c)
```

Create `tests/lib/kinematics/prj.conf`:

```
CONFIG_ZTEST=y
CONFIG_KINEMATICS=y
```

> The `&fake_act` phandle target only needs to *exist* in devicetree (the table reads
> its node name, it does not drive it), so the actuator subsystem is not enabled here.

Create `tests/lib/kinematics/testcase.yaml`:

```yaml
common:
  tags: [kinematics, lib]
tests:
  lib.kinematics:
    platform_allow: [native_sim]
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west twister -p native_sim --clobber-output -v \
  -T deps/modules/lib/rosterloh-drivers/tests/lib/kinematics
```
Expected: build ERROR — `zephyr/kinematics/joint.h: No such file or directory`.

- [ ] **Step 3: Create the types header**

Create `include/zephyr/kinematics/joint_types.h`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_KINEMATICS_JOINT_TYPES_H_
#define ZEPHYR_INCLUDE_KINEMATICS_JOINT_TYPES_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CBOR wire-format schema version. */
#define KINEMATICS_SCHEMA_VERSION 1

/** URDF joint types. Order matches the binding's joint-type enum. */
enum joint_type {
	JOINT_TYPE_FIXED = 0,
	JOINT_TYPE_REVOLUTE,
	JOINT_TYPE_CONTINUOUS,
	JOINT_TYPE_PRISMATIC,
};

/** Bits set in joint_descriptor.valid_mask for optional blocks. */
#define JOINT_HAS_LIMITS   BIT(0)
#define JOINT_HAS_DYNAMICS BIT(1)
#define JOINT_HAS_ACTUATOR BIT(2)

/**
 * One joint exposed by this device. All values are SI: metres, radians,
 * kg, kg*m^2, N*m, rad/s. Optional blocks are valid only when the matching
 * valid_mask bit is set.
 */
struct joint_descriptor {
	const char *parent_link;
	const char *child_link;
	enum joint_type type;
	float axis[3];         /**< unit vector            */
	float origin_xyz[3];   /**< m                      */
	float origin_rpy[3];   /**< rad                    */
	struct {
		float lower;   /**< rad or m               */
		float upper;
		float velocity; /**< rad/s or m/s          */
		float effort;  /**< N*m or N               */
	} limits;
	struct {
		float mass;        /**< kg                 */
		float com[3];      /**< m                  */
		float inertia[6];  /**< kg*m^2 [ixx iyy izz ixy ixz iyz] */
	} dynamics;
	const char *actuator_name; /**< NULL unless JOINT_HAS_ACTUATOR */
	uint8_t valid_mask;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KINEMATICS_JOINT_TYPES_H_ */
```

- [ ] **Step 4: Create the query API header**

Create `include/zephyr/kinematics/joint.h`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_KINEMATICS_JOINT_H_
#define ZEPHYR_INCLUDE_KINEMATICS_JOINT_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kinematics/joint_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup kinematics Kinematics joint descriptors
 * @{
 */

/** Number of joints this device declares. */
size_t kinematics_joint_count(void);

/** Get joint @p idx, or NULL if out of range. */
const struct joint_descriptor *kinematics_joint_get(size_t idx);

/**
 * Serialize all joints to a CBOR payload (see docs CDDL).
 *
 * @param buf      Output buffer.
 * @param len      Buffer size in bytes.
 * @param out_len  Set to the encoded length on success.
 * @retval 0       Encoded.
 * @retval -ENOMEM Buffer too small.
 */
int kinematics_encode_cbor(uint8_t *buf, size_t len, size_t *out_len);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KINEMATICS_JOINT_H_ */
```

- [ ] **Step 5: Create the DT binding**

Create `dts/bindings/kinematics/rosterloh,joint.yaml`:

```yaml
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

description: |
  A single mechanical joint exposed by this device (URDF-style). The kinematics
  library collects all enabled joint nodes into a descriptor table and can
  serialize them to CBOR for a remote controller to assemble a kinematic tree.

  Devicetree has no float type, so fractional values are scaled integers; the
  library converts them to SI floats. Units are noted per property.

compatible: "rosterloh,joint"

properties:
  link-parent:
    type: string
    required: true
    description: Parent link name (URDF parent).
  link-child:
    type: string
    required: true
    description: Child link name (URDF child).
  joint-type:
    type: string
    required: true
    enum: ["fixed", "revolute", "continuous", "prismatic"]
    description: URDF joint type.
  axis:
    type: array
    required: true
    description: Joint axis as a unit vector scaled x1000, e.g. <0 0 1000> for +Z.
  origin-xyz:
    type: array
    required: true
    description: Origin translation from parent, micrometres [x y z].
  origin-rpy:
    type: array
    required: true
    description: Origin rotation from parent, microradians [r p y].
  limit-lower:
    type: int
    description: Lower position limit, microradians (revolute) or micrometres (prismatic).
  limit-upper:
    type: int
    description: Upper position limit, microradians or micrometres.
  limit-velocity:
    type: int
    description: Velocity limit, microradians/s or micrometres/s.
  limit-effort:
    type: int
    description: Effort limit, milli-N*m (revolute) or milli-N (prismatic).
  mass-mg:
    type: int
    description: Link mass in milligrams. Presence enables the dynamics block.
  com-xyz:
    type: array
    description: Centre-of-mass offset, micrometres [x y z].
  inertia:
    type: array
    description: Inertia tensor [ixx iyy izz ixy ixz iyz] in g*mm^2.
  actuator:
    type: phandle
    description: Optional actuator device that drives this joint.
```

- [ ] **Step 6: Create the generated table + query API**

Create `lib/kinematics/joint.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/kinematics/joint.h>

/* Scale factors: devicetree (scaled int) -> SI float. */
#define KIN_AXIS(node, i)    (DT_PROP_BY_IDX(node, axis, i) / 1000.0f)
#define KIN_XYZ(node, p, i)  (DT_PROP_BY_IDX(node, p, i) / 1000000.0f)   /* um  -> m   */
#define KIN_RPY(node, i)     (DT_PROP_BY_IDX(node, origin_rpy, i) / 1000000.0f) /* urad -> rad */
#define KIN_ANG(node, p)     (DT_PROP_OR(node, p, 0) / 1000000.0f)        /* urad-> rad */
#define KIN_EFFORT(node)     (DT_PROP_OR(node, limit_effort, 0) / 1000.0f)/* mNm -> Nm  */
#define KIN_MASS(node)       (DT_PROP(node, mass_mg) / 1000000.0f)        /* mg  -> kg  */
#define KIN_INERTIA(node, i) (DT_PROP_BY_IDX(node, inertia, i) / 1000000000.0f) /* g*mm^2 -> kg*m^2 */

#define KIN_HAS_LIMITS(node)                                                                        \
	(DT_NODE_HAS_PROP(node, limit_lower) || DT_NODE_HAS_PROP(node, limit_upper) ||              \
	 DT_NODE_HAS_PROP(node, limit_velocity) || DT_NODE_HAS_PROP(node, limit_effort))

#define KIN_DYNAMICS(node)                                                                          \
	COND_CODE_1(DT_NODE_HAS_PROP(node, mass_mg),                                                \
		    ({.mass = KIN_MASS(node),                                                       \
		      .com = {KIN_XYZ(node, com_xyz, 0), KIN_XYZ(node, com_xyz, 1),                 \
			      KIN_XYZ(node, com_xyz, 2)},                                          \
		      .inertia = {KIN_INERTIA(node, 0), KIN_INERTIA(node, 1), KIN_INERTIA(node, 2), \
				  KIN_INERTIA(node, 3), KIN_INERTIA(node, 4),                      \
				  KIN_INERTIA(node, 5)}}),                                         \
		    ({0}))

#define KIN_ACTUATOR_NAME(node)                                                                     \
	COND_CODE_1(DT_NODE_HAS_PROP(node, actuator), (DT_NODE_FULL_NAME(DT_PHANDLE(node, actuator))),\
		    (NULL))

#define JOINT_ENTRY(node)                                                                           \
	{                                                                                           \
		.parent_link = DT_PROP(node, link_parent),                                          \
		.child_link = DT_PROP(node, link_child),                                            \
		.type = (enum joint_type)DT_ENUM_IDX(node, joint_type),                             \
		.axis = {KIN_AXIS(node, 0), KIN_AXIS(node, 1), KIN_AXIS(node, 2)},                  \
		.origin_xyz = {KIN_XYZ(node, origin_xyz, 0), KIN_XYZ(node, origin_xyz, 1),          \
			       KIN_XYZ(node, origin_xyz, 2)},                                       \
		.origin_rpy = {KIN_RPY(node, 0), KIN_RPY(node, 1), KIN_RPY(node, 2)},               \
		.limits = {.lower = KIN_ANG(node, limit_lower),                                     \
			   .upper = KIN_ANG(node, limit_upper),                                     \
			   .velocity = KIN_ANG(node, limit_velocity),                               \
			   .effort = KIN_EFFORT(node)},                                             \
		.dynamics = KIN_DYNAMICS(node),                                                     \
		.actuator_name = KIN_ACTUATOR_NAME(node),                                           \
		.valid_mask = (KIN_HAS_LIMITS(node) ? JOINT_HAS_LIMITS : 0) |                       \
			      (DT_NODE_HAS_PROP(node, mass_mg) ? JOINT_HAS_DYNAMICS : 0) |          \
			      (DT_NODE_HAS_PROP(node, actuator) ? JOINT_HAS_ACTUATOR : 0),          \
	},

static const struct joint_descriptor joints[] = {
	DT_FOREACH_STATUS_OKAY(rosterloh_joint, JOINT_ENTRY)};

size_t kinematics_joint_count(void)
{
	return ARRAY_SIZE(joints);
}

const struct joint_descriptor *kinematics_joint_get(size_t idx)
{
	if (idx >= ARRAY_SIZE(joints)) {
		return NULL;
	}
	return &joints[idx];
}
```

- [ ] **Step 7: Create the library build files**

Create `lib/kinematics/Kconfig`:

```
config KINEMATICS
	bool "Kinematics joint-descriptor library"
	select ZCBOR
	help
	  Expose this device's URDF-style joint descriptors (from devicetree)
	  and serialize them to CBOR for a remote controller to assemble a
	  kinematic tree.
```

Create `lib/kinematics/CMakeLists.txt`:

```cmake
zephyr_library_named(kinematics)
zephyr_library_sources(joint.c)
```

- [ ] **Step 8: Wire into the top-level build**

Modify `Kconfig` (append after the existing `rsource` lines):

```
rsource "lib/kinematics/Kconfig"
```

Modify `CMakeLists.txt` (append after the existing `add_subdirectory_ifdef` lines):

```cmake
add_subdirectory_ifdef(CONFIG_KINEMATICS lib/kinematics)
```

- [ ] **Step 9: Run the test to verify it passes**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west twister -p native_sim --clobber-output -v \
  -T deps/modules/lib/rosterloh-drivers/tests/lib/kinematics
```
Expected: `lib.kinematics` PASSES (6 `kinematics_table` cases pass).

- [ ] **Step 10: Format check**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
uv run clang-format --dry-run --Werror lib/kinematics/joint.c \
  include/zephyr/kinematics/joint.h include/zephyr/kinematics/joint_types.h \
  tests/lib/kinematics/src/main.c
```
Expected: no output (clean). Fix any diffs reported.

- [ ] **Step 11: Commit**

```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
git add dts/bindings/kinematics include/zephyr/kinematics lib/kinematics \
  tests/lib/kinematics Kconfig CMakeLists.txt
git commit -m "kinematics: add joint-descriptor binding, table, and query API

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: CBOR encoder

**Files:**
- Create: `lib/kinematics/cbor.c`
- Modify: `lib/kinematics/CMakeLists.txt:2` (add `cbor.c` to sources)
- Modify: `tests/lib/kinematics/src/main.c` (add `kinematics_cbor` suite)

**Interfaces:**
- Consumes: `kinematics_joint_count()`, `kinematics_joint_get()`, `struct joint_descriptor`, `KINEMATICS_SCHEMA_VERSION` (Task 1).
- Produces: `int kinematics_encode_cbor(uint8_t *buf, size_t len, size_t *out_len)` (already declared in `joint.h`).

CBOR layout (CDDL, from the spec):
```cddl
response = { "v": uint, "joints": [* joint] }
joint = {
    "parent": tstr, "child": tstr, "type": uint,
    "axis": [3*3 float], "origin": [6*6 float],
    ? "limits": [4*4 float], ? "dyn": [10*10 float], ? "actuator": tstr
}
```

- [ ] **Step 1: Write the failing test**

Append to `tests/lib/kinematics/src/main.c` (add the includes at the top alongside the existing ones, then the suite at the end):

```c
#include <zcbor_decode.h>
#include <zcbor_common.h>

ZTEST_SUITE(kinematics_cbor, NULL, NULL, NULL, NULL, NULL);

ZTEST(kinematics_cbor, test_too_small_buffer)
{
	uint8_t buf[4];
	size_t out_len = 0;

	zassert_equal(kinematics_encode_cbor(buf, sizeof(buf), &out_len), -ENOMEM);
}

ZTEST(kinematics_cbor, test_roundtrip_first_joint)
{
	uint8_t buf[512];
	size_t out_len = 0;

	zassert_equal(kinematics_encode_cbor(buf, sizeof(buf), &out_len), 0);
	zassert_true(out_len > 0);

	ZCBOR_STATE_D(ds, 4, buf, out_len, 1, 0);

	uint32_t version = 0;
	zassert_true(zcbor_map_start_decode(ds));
	zassert_true(zcbor_tstr_expect_lit(ds, "v"));
	zassert_true(zcbor_uint32_decode(ds, &version));
	zassert_equal(version, KINEMATICS_SCHEMA_VERSION);

	zassert_true(zcbor_tstr_expect_lit(ds, "joints"));
	zassert_true(zcbor_list_start_decode(ds));

	/* First joint: mandatory-only (the fixed "world"->"base_link" joint). */
	struct zcbor_string s;
	uint32_t type = 0;
	float axis[3], origin[6];

	zassert_true(zcbor_map_start_decode(ds));

	zassert_true(zcbor_tstr_expect_lit(ds, "parent"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_equal(s.len, strlen("world"));
	zassert_mem_equal(s.value, "world", s.len);

	zassert_true(zcbor_tstr_expect_lit(ds, "child"));
	zassert_true(zcbor_tstr_decode(ds, &s));
	zassert_mem_equal(s.value, "base_link", s.len);

	zassert_true(zcbor_tstr_expect_lit(ds, "type"));
	zassert_true(zcbor_uint32_decode(ds, &type));
	zassert_equal(type, JOINT_TYPE_FIXED);

	zassert_true(zcbor_tstr_expect_lit(ds, "axis"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 3; i++) {
		zassert_true(zcbor_float32_decode(ds, &axis[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_within(axis[2], 1.0f, EPS);

	zassert_true(zcbor_tstr_expect_lit(ds, "origin"));
	zassert_true(zcbor_list_start_decode(ds));
	for (int i = 0; i < 6; i++) {
		zassert_true(zcbor_float32_decode(ds, &origin[i]));
	}
	zassert_true(zcbor_list_end_decode(ds));
	zassert_within(origin[0], 0.0f, EPS);

	/* joints[0] has no optional blocks, so the joint map ends here. */
	zassert_true(zcbor_map_end_decode(ds));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west twister -p native_sim --clobber-output -v \
  -T deps/modules/lib/rosterloh-drivers/tests/lib/kinematics
```
Expected: link ERROR — `undefined reference to kinematics_encode_cbor`.

- [ ] **Step 3: Implement the encoder**

Create `lib/kinematics/cbor.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zcbor_common.h>
#include <zcbor_encode.h>
#include <zephyr/kinematics/joint.h>

#define KIN_STR_MAX   64 /* max link/actuator name length to scan */
#define KIN_JOINT_MAP 8  /* upper bound on key-value pairs per joint map */

static bool put_floats(zcbor_state_t *st, const float *v, size_t n)
{
	bool ok = zcbor_list_start_encode(st, n);

	for (size_t i = 0; i < n && ok; i++) {
		ok = ok && zcbor_float32_put(st, v[i]);
	}
	return ok && zcbor_list_end_encode(st, n);
}

static bool encode_joint(zcbor_state_t *st, const struct joint_descriptor *j)
{
	float origin[6] = {j->origin_xyz[0], j->origin_xyz[1], j->origin_xyz[2],
			   j->origin_rpy[0], j->origin_rpy[1], j->origin_rpy[2]};
	bool ok = zcbor_map_start_encode(st, KIN_JOINT_MAP);

	ok = ok && zcbor_tstr_put_lit(st, "parent");
	ok = ok && zcbor_tstr_put_term(st, j->parent_link, KIN_STR_MAX);
	ok = ok && zcbor_tstr_put_lit(st, "child");
	ok = ok && zcbor_tstr_put_term(st, j->child_link, KIN_STR_MAX);
	ok = ok && zcbor_tstr_put_lit(st, "type");
	ok = ok && zcbor_uint32_put(st, (uint32_t)j->type);
	ok = ok && zcbor_tstr_put_lit(st, "axis");
	ok = ok && put_floats(st, j->axis, 3);
	ok = ok && zcbor_tstr_put_lit(st, "origin");
	ok = ok && put_floats(st, origin, 6);

	if (ok && (j->valid_mask & JOINT_HAS_LIMITS)) {
		float limits[4] = {j->limits.lower, j->limits.upper, j->limits.velocity,
				   j->limits.effort};
		ok = ok && zcbor_tstr_put_lit(st, "limits");
		ok = ok && put_floats(st, limits, 4);
	}
	if (ok && (j->valid_mask & JOINT_HAS_DYNAMICS)) {
		float dyn[10] = {j->dynamics.mass,       j->dynamics.com[0],
				 j->dynamics.com[1],      j->dynamics.com[2],
				 j->dynamics.inertia[0],  j->dynamics.inertia[1],
				 j->dynamics.inertia[2],  j->dynamics.inertia[3],
				 j->dynamics.inertia[4],  j->dynamics.inertia[5]};
		ok = ok && zcbor_tstr_put_lit(st, "dyn");
		ok = ok && put_floats(st, dyn, 10);
	}
	if (ok && (j->valid_mask & JOINT_HAS_ACTUATOR)) {
		ok = ok && zcbor_tstr_put_lit(st, "actuator");
		ok = ok && zcbor_tstr_put_term(st, j->actuator_name, KIN_STR_MAX);
	}

	return ok && zcbor_map_end_encode(st, KIN_JOINT_MAP);
}

int kinematics_encode_cbor(uint8_t *buf, size_t len, size_t *out_len)
{
	ZCBOR_STATE_E(st, 4, buf, len, 1);
	const size_t n = kinematics_joint_count();
	bool ok = zcbor_map_start_encode(st, 2);

	ok = ok && zcbor_tstr_put_lit(st, "v");
	ok = ok && zcbor_uint32_put(st, KINEMATICS_SCHEMA_VERSION);
	ok = ok && zcbor_tstr_put_lit(st, "joints");
	ok = ok && zcbor_list_start_encode(st, n);
	for (size_t i = 0; i < n && ok; i++) {
		ok = ok && encode_joint(st, kinematics_joint_get(i));
	}
	ok = ok && zcbor_list_end_encode(st, n);
	ok = ok && zcbor_map_end_encode(st, 2);

	if (!ok) {
		return -ENOMEM;
	}
	*out_len = (size_t)(st->payload - buf);
	return 0;
}
```

- [ ] **Step 4: Add cbor.c to the library**

Modify `lib/kinematics/CMakeLists.txt`:

```cmake
zephyr_library_named(kinematics)
zephyr_library_sources(joint.c cbor.c)
```

- [ ] **Step 5: Run the test to verify it passes**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west twister -p native_sim --clobber-output -v \
  -T deps/modules/lib/rosterloh-drivers/tests/lib/kinematics
```
Expected: `lib.kinematics` PASSES (both `kinematics_table` and `kinematics_cbor` suites).

- [ ] **Step 6: Format check**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
uv run clang-format --dry-run --Werror lib/kinematics/cbor.c tests/lib/kinematics/src/main.c
```
Expected: no output. Fix any diffs.

- [ ] **Step 7: Commit**

```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
git add lib/kinematics/cbor.c lib/kinematics/CMakeLists.txt tests/lib/kinematics/src/main.c
git commit -m "kinematics: add CBOR encoder for joint descriptors

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Sample demo

**Files:**
- Create: `samples/kinematics/CMakeLists.txt`, `prj.conf`, `sample.yaml`, `app.overlay`, `src/main.c`

**Interfaces:**
- Consumes: `kinematics_joint_count()`, `kinematics_joint_get()`, `kinematics_encode_cbor()` (Tasks 1–2).

The sample is build-verified by twister (it has a `sample.yaml`); it encodes the table, prints the CBOR as hex, and prints each joint, demonstrating the device side end-to-end.

- [ ] **Step 1: Create the sample overlay**

Create `samples/kinematics/app.overlay`:

```dts
/ {
	joint_base: joint-base {
		compatible = "rosterloh,joint";
		link-parent = "world";
		link-child = "base_link";
		joint-type = "fixed";
		axis = <0 0 1000>;
		origin-xyz = <0 0 0>;
		origin-rpy = <0 0 0>;
	};

	joint_shoulder: joint-shoulder {
		compatible = "rosterloh,joint";
		link-parent = "base_link";
		link-child = "upper_arm";
		joint-type = "revolute";
		axis = <0 0 1000>;
		origin-xyz = <0 0 100000>;       /* 0.1 m */
		origin-rpy = <0 0 0>;
		limit-lower = <(-1570796)>;
		limit-upper = <1570796>;
		limit-velocity = <3141592>;
		limit-effort = <2500>;
	};
};
```

- [ ] **Step 2: Create the sample source**

Create `samples/kinematics/src/main.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kinematics/joint.h>

static const char *type_str(enum joint_type t)
{
	switch (t) {
	case JOINT_TYPE_FIXED:
		return "fixed";
	case JOINT_TYPE_REVOLUTE:
		return "revolute";
	case JOINT_TYPE_CONTINUOUS:
		return "continuous";
	case JOINT_TYPE_PRISMATIC:
		return "prismatic";
	default:
		return "?";
	}
}

int main(void)
{
	const size_t n = kinematics_joint_count();

	printk("kinematics: %zu joint(s)\n", n);
	for (size_t i = 0; i < n; i++) {
		const struct joint_descriptor *j = kinematics_joint_get(i);

		printk("  [%zu] %s -> %s (%s) origin=(%.3f %.3f %.3f) actuated=%d\n", i,
		       j->parent_link, j->child_link, type_str(j->type), (double)j->origin_xyz[0],
		       (double)j->origin_xyz[1], (double)j->origin_xyz[2],
		       (j->valid_mask & JOINT_HAS_ACTUATOR) ? 1 : 0);
	}

	uint8_t buf[512];
	size_t out_len = 0;
	int rc = kinematics_encode_cbor(buf, sizeof(buf), &out_len);

	if (rc != 0) {
		printk("encode failed: %d\n", rc);
		return 0;
	}

	printk("CBOR (%zu bytes):", out_len);
	for (size_t i = 0; i < out_len; i++) {
		printk(" %02x", buf[i]);
	}
	printk("\n");
	return 0;
}
```

- [ ] **Step 3: Create the sample build files**

Create `samples/kinematics/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kinematics_sample)
target_sources(app PRIVATE src/main.c)
```

Create `samples/kinematics/prj.conf`:

```
CONFIG_KINEMATICS=y
CONFIG_CBPRINTF_FP_SUPPORT=y
```

Create `samples/kinematics/sample.yaml`:

```yaml
sample:
  name: Kinematics joint descriptors
common:
  tags: [kinematics, sample]
  harness: console
  harness_config:
    type: one_line
    regex:
      - "kinematics: 2 joint\\(s\\)"
tests:
  sample.kinematics:
    platform_allow: [native_sim]
```

- [ ] **Step 4: Build and run the sample**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west twister -p native_sim --clobber-output -v \
  -T deps/modules/lib/rosterloh-drivers/samples/kinematics
```
Expected: `sample.kinematics` PASSES (console harness matches `kinematics: 2 joint(s)`).

- [ ] **Step 5: Format check**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
uv run clang-format --dry-run --Werror samples/kinematics/src/main.c
```
Expected: no output. Fix any diffs.

- [ ] **Step 6: Commit**

```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
git add samples/kinematics
git commit -m "kinematics: add joint-descriptor sample

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Done criteria

- `lib.kinematics` ztest passes on native_sim (table conversions, valid_mask, CBOR round-trip, -ENOMEM).
- `sample.kinematics` builds and runs on native_sim, printing joints + CBOR hex.
- All new C files pass `clang-format --dry-run --Werror`.
- Three commits on `feat/kinematics-joint-descriptor`.
