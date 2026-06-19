# Kinematics joint-descriptor library — design

**Date:** 2026-06-19
**Status:** Approved (design)

## Problem

A device that is part of a wider mechanism (e.g. a motor driving one joint) should
be able to expose its own physical/kinematic properties. A remote controller can
then interrogate every connected device on startup and assemble a kinematic tree
dynamically — URDF-style, where each joint links a parent link to a child link.

Each device is **self-describing about its own joint(s) only**. It does not know the
global tree. The controller assembles the tree by matching link names: device A's
`child_link` == device B's `parent_link` (URDF-style named links).

## Scope

- **In:** device-side declaration of joint descriptors (devicetree binding), a
  device-side query API, and a transport-agnostic CBOR wire format for serving them.
- **Out:** controller-side tree assembly, geometry/meshes, runtime-mutable
  descriptors, `floating`/`planar` joint types. Add when a real consumer needs them.

The controller is remote and the protocol is transport-agnostic. This repo is
firmware-side, so we deliver the **device side + wire format** only.

## Approach

The joint descriptor is **static physical data** — compile-time, read-only, never
changes at runtime. So this is a **lightweight library** (`lib/kinematics/`), not a
driver subsystem: a `DT_FOREACH`-generated `const` table plus a zcbor encoder. No
device instances, no syscalls, no state machine — there is no runtime state to
manage. (Rejected: a full `subsys/kinematics/` with syscalls mirroring
`subsys/actuator` — machinery for a frozen `const` struct. Can be promoted later if
joints ever gain runtime-mutable state.)

## Layout

- `dts/bindings/kinematics/rosterloh,joint.yaml` — DT binding
- `include/zephyr/kinematics/joint_types.h` — descriptor struct, enums, valid-mask bits
- `include/zephyr/kinematics/joint.h` — query + encode API
- `lib/kinematics/` — generated table + zcbor encoder + `CMakeLists.txt` + `Kconfig`
- `samples/kinematics/` — overlay with example joints; prints and decodes the CBOR
- `tests/lib/kinematics/` — ztest
- Top-level `Kconfig`: `CONFIG_KINEMATICS` (selects `CONFIG_ZCBOR`)

## Units: SI on the API and wire; scaled integers only in devicetree

The C descriptor and the CBOR wire format are **pure SI** — meters, radians, kg,
kg·m², N·m, rad/s — matching URDF 1:1 (URDF is also SI, stored as ASCII floats in
XML). The controller never sees a scaled integer.

Scaled integers appear **only in the devicetree authoring layer**, forced by a hard
Zephyr constraint: devicetree has no float type. DT cells are 32-bit integers (or
strings), so `origin-xyz = <0.05 0 0.1>` is rejected by `dtc`. Encoding fractional
values as micro-/milli-scaled integers and converting to SI floats once at compile
time is the idiomatic Zephyr pattern (cf. regulator `microvolt`, PWM `nanoseconds`,
sensor bindings). The alternative — strings parsed at runtime — ships a float parser
and pays runtime cost for static data, so it is rejected.

## DT binding: `rosterloh,joint`

One node per joint. Per the units note above, fractional values are **scaled
integers** in devicetree, converted to SI floats in the generated table.

| Property         | Type            | Units / encoding                          | Required |
|------------------|-----------------|-------------------------------------------|----------|
| `link-parent`    | string          | parent link name                          | yes      |
| `link-child`     | string          | child link name                           | yes      |
| `joint-type`     | string enum     | `fixed`\|`revolute`\|`continuous`\|`prismatic` | yes |
| `axis`           | array (3 ints)  | unit vector ×1000 (`<0 0 1000>` = Z)      | yes      |
| `origin-xyz`     | array (3 ints)  | µm                                        | yes      |
| `origin-rpy`     | array (3 ints)  | µrad                                      | yes      |
| `limit-lower`    | int             | µrad (revolute/continuous) or µm (prismatic) | no (group) |
| `limit-upper`    | int             | µrad or µm                                | no (group) |
| `limit-velocity` | int             | µrad/s or µm/s                            | no (group) |
| `limit-effort`   | int             | mN·m (or mN for prismatic)                | no (group) |
| `mass-mg`        | int             | milligrams                                | no (group) |
| `com-xyz`        | array (3 ints)  | µm                                        | no (group) |
| `inertia`        | array (6 ints)  | g·mm² `[ixx iyy izz ixy ixz iyz]`         | no (group) |
| `actuator`       | phandle         | reference to an actuator device           | no       |

Optional groups set valid-mask bits: any `limit-*` present → `JOINT_HAS_LIMITS`;
`mass-mg`+`com-xyz`+`inertia` present → `JOINT_HAS_DYNAMICS`; `actuator` present →
`JOINT_HAS_ACTUATOR`.

`// ponytail: milli-int axis is exact for principal axes; bump scale if arbitrary axes ever need precision.`

## C descriptor + API

```c
/* joint_types.h */
enum joint_type {
    JOINT_TYPE_FIXED = 0,
    JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_CONTINUOUS,
    JOINT_TYPE_PRISMATIC,
};

#define JOINT_HAS_LIMITS   BIT(0)
#define JOINT_HAS_DYNAMICS BIT(1)
#define JOINT_HAS_ACTUATOR BIT(2)

struct joint_descriptor {
    const char *parent_link, *child_link;
    enum joint_type type;
    float axis[3];
    float origin_xyz[3];   /* m   */
    float origin_rpy[3];   /* rad */
    struct { float lower, upper, velocity, effort; } limits;
    struct { float mass; float com[3]; float inertia[6]; } dynamics;
    const char *actuator_name;   /* NULL unless JOINT_HAS_ACTUATOR */
    uint8_t valid_mask;
};

/* joint.h */
size_t                         kinematics_joint_count(void);
const struct joint_descriptor *kinematics_joint_get(size_t idx);
/* Serialize all joints to CBOR. Returns 0 and sets *out_len, or -ENOMEM. */
int kinematics_encode_cbor(uint8_t *buf, size_t len, size_t *out_len);
```

The table is built with `DT_FOREACH_STATUS_OKAY(rosterloh_joint, ...)`, applying the
scale→SI conversions at compile time. `actuator_name` resolves to the DT node name of
the phandle target (the controller matches by link names; the actuator name is
informational so the controller knows which joints are actuated).

## CBOR wire format (CDDL)

Drives zcbor codegen and documents the format for the (out-of-tree) controller.
String keys for cross-language clarity. Optional keys present only when the matching
valid-mask bit is set.

```cddl
response = { "v": uint, "joints": [* joint] }
joint = {
    "parent": tstr, "child": tstr,
    "type": uint,             ; 0 fixed, 1 revolute, 2 continuous, 3 prismatic
    "axis":   [3*3 float],
    "origin": [6*6 float],    ; xyz(m) rpy(rad)
    ? "limits": [4*4 float],   ; lower upper velocity effort
    ? "dyn":    [10*10 float], ; mass com[3] inertia[6]
    ? "actuator": tstr
}
```

`"v"` is a schema version integer; bump on incompatible changes.

## Request semantics

Transport-agnostic, so we define only the **response**. The contract: *on a "describe"
request, the device calls `kinematics_encode_cbor()` and sends the bytes.* How the
request arrives (zenoh query, UART opcode, CAN message, …) is the transport's concern
and out of scope. The sample demonstrates one wiring.

## Testing

`ztest` in `tests/lib/kinematics/` with a fixture overlay declaring three joints:
a `fixed` joint, a `revolute` joint with limits, and an actuated joint with a
dynamics block and an `actuator` phandle. Assertions:

1. `kinematics_joint_count()` == 3 and ordering is stable.
2. Scaled-int → SI conversion is correct (axis, origin, limits, dynamics).
3. `valid_mask` bits match which optional groups each node declared.
4. `kinematics_encode_cbor()` round-trips: decode the buffer with zcbor and compare
   every field against the descriptor table; assert `-ENOMEM` on an undersized buffer.

The `samples/kinematics/` app doubles as a runnable demo: it encodes the table,
prints the CBOR as hex, decodes it back, and prints the reconstructed joints.
