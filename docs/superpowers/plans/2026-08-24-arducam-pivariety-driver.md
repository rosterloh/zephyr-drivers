# Arducam Pivariety Camera Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a generic Zephyr driver for Arducam Pivariety cameras, wire the Arducam ToF camera onto `esp32p4_nano` via a shield, and use it as an independent MIPI transmitter to advance the stalled CSI-2 receiver investigation.

**Architecture:** Pivariety modules carry an on-board MCU exposing a self-describing register interface at I2C `0x0c` (16-bit big-endian address, 32-bit big-endian data, transferred whole). Formats, resolutions and controls are enumerated at runtime by writing an *index* register and reading an *attribute* register until the sentinel `0xFFFFFFFE`. The driver enumerates once at init into a fixed-size `video_format_cap[]` sized by Kconfig, then serves `get_caps`/`set_fmt`/`set_stream` from that.

**Tech Stack:** Zephyr 4.4.99, `video` subsystem, raw `i2c_write_read_dt`/`i2c_write_dt` framing, ztest + `CONFIG_I2C_EMUL` for unit tests, ESP32-P4 MIPI CSI-2 receiver (`video_esp32_csi.c`).

**Spec:** `docs/superpowers/specs/2026-08-24-arducam-pivariety-driver-design.md`

## Global Constraints

- Two repos are in play. Tasks 1–6 are in the **drivers module** at `deps/modules/lib/rosterloh-drivers/` (branch `feat/arducam-pivariety`, already created, carrying in-progress CSI diagnostic changes to `drivers/video/video_esp32_csi.c` — leave them alone). Task 7 is in the **workspace repo** at `/Users/richard.osterloh/workspace/zephyr-applications`. Commit each in its own repo.
- Every C file: `clang-format` using the drivers repo's in-tree `.clang-format`, verified with `mise x -- clang-format --dry-run --Werror <files>` from the workspace root.
- Every new file starts with `/* Copyright (c) 2026 Richard Osterloh` / ` * SPDX-License-Identifier: Apache-2.0` (C) or `# ` equivalents (Kconfig/YAML/CMake).
- All builds go through `mise run`. Never bare `west`/`python`. Build the app with `mise run agent-build --sysbuild data_collection`, which writes the full log to `logs/data_collection-build.log`.
- **Run the ztest suites on `qemu_cortex_m3`, not `native_sim`.** `native_sim` is Linux-only and will not build on this Mac, but QEMU does: `arm-zephyr-eabi` is in the SDK and Homebrew supplies `qemu-system-arm`, so `ztest` + `CONFIG_I2C_EMUL` runs locally. This was verified before the plan was written — a minimal i2c-emul suite builds and reports `PROJECT EXECUTION SUCCESSFUL` under `qemu_cortex_m3`. `native_sim` stays in `platform_allow` so CI (which runs on Linux) covers it too, but every red/green step below uses QEMU.
- `qemu_cortex_m3` has no I2C controller of its own, so its overlay must *declare* the `zephyr,i2c-emul-controller` bus rather than reference an existing `&i2c0` the way the `native_sim` overlay does.
- **Do not use Zephyr's CCI register helpers for this device.** `video_read_cci_reg()`/`video_write_cci_reg()` implement true CCI semantics — a 32-bit value spread over four consecutive byte-addressed registers, read one byte per transaction with the address incrementing. Pivariety instead transfers one 32-bit value per 16-bit address in a single transaction, so the driver frames transfers itself with `i2c_write_read_dt()`/`i2c_write_dt()` (see Task 1). Register constants are plain `uint16_t`, no flag macro.
- Pivariety register constants, verbatim from `drivers/media/i2c/arducam-pivariety.h` in the Raspberry Pi kernel:
  `DEVICE_REG_BASE 0x0100`, `PIXFORMAT_REG_BASE 0x0200`, `FORMAT_REG_BASE 0x0300`, `CTRL_REG_BASE 0x0400`, `NO_DATA_AVAILABLE 0xFFFFFFFE`, `ARDUCAM_MODE_STANDBY 0x00`, `ARDUCAM_MODE_STREAMING 0x01`.
- Arducam ToF camera DT values, from the RPi `arducam-pivariety-overlay.dts`: I2C address `0x0c`, `data-lanes = <1 2>`, `clock-noncontinuous`, link frequency 493500000 Hz.
- Two schema details of this Zephyr checkout, confirmed against `dts/bindings/video/video-interfaces.yaml`: `link-frequencies` is `type: array` (32-bit cells), so write `<493500000>` and **not** `/bits/ 64 <493500000>` -- the latter fails dtlib. And `remote-endpoint-label` is `required: true`, so every endpoint node needs one; use `""` for an endpoint with no real peer, as Zephyr's own `tests/drivers/build_all/video` overlays do.

## File Structure

**Drivers module** (`deps/modules/lib/rosterloh-drivers/`):

| Path | Responsibility |
|---|---|
| `dts/bindings/video/arducam,pivariety.yaml` | Binding: i2c-device + video-interfaces port/endpoint |
| `drivers/video/arducam_pivariety.c` | The whole driver — register layer, enumeration, format/stream/controls, shell command |
| `drivers/video/Kconfig` | `CONFIG_VIDEO_ARDUCAM_PIVARIETY`, `_MAX_FORMATS` |
| `drivers/video/CMakeLists.txt` | Conditional source entry |
| `drivers/video/video_esp32_csi.c` | `csi_pixfmt_info()` gains greyscale + remaining Bayer fourccs |
| `boards/shields/arducam_tof_camera/` | `shield.yml`, `Kconfig.shield`, `arducam_tof_camera.overlay` |
| `tests/drivers/arducam_pivariety/` | ztest suite + I2C emulator backend |

The driver is a single file. It is expected to land around 550 lines including the shell command, which is in line with `imx219.c` (700 lines) in the same directory. Do not split it.

**Workspace repo:**

| Path | Responsibility |
|---|---|
| `applications/data_collection/src/cam_mgmt.c` | Capture path becomes format-agnostic |
| `applications/data_collection/CMakeLists.txt` | `SHIELD` becomes overridable |
| `applications/data_collection/README.md` | Document the camera switch |

---

### Task 1: Binding, Kconfig, and probe

**Files:**
- Create: `dts/bindings/video/arducam,pivariety.yaml`
- Create: `drivers/video/arducam_pivariety.c`
- Modify: `drivers/video/Kconfig`
- Modify: `drivers/video/CMakeLists.txt`
- Create: `tests/drivers/arducam_pivariety/CMakeLists.txt`
- Create: `tests/drivers/arducam_pivariety/prj.conf`
- Create: `tests/drivers/arducam_pivariety/testcase.yaml`
- Create: `tests/drivers/arducam_pivariety/boards/native_sim.overlay`
- Create: `tests/drivers/arducam_pivariety/boards/qemu_cortex_m3.overlay`
- Create: `tests/drivers/arducam_pivariety/src/pivariety_emul.c`
- Test: `tests/drivers/arducam_pivariety/src/main.c`

**Interfaces:**
- Consumes: nothing (first task).
- Produces: `struct pivariety_config { struct i2c_dt_spec i2c; int64_t link_freq_hz; }`, `struct pivariety_data`, `static int piv_wait_idle(const struct device *dev)`, `static int piv_read(const struct device *dev, uint16_t reg, uint32_t *val)`, `static int piv_write(const struct device *dev, uint16_t reg, uint32_t val)`. Later tasks call all three. Register constants are plain `uint16_t` addresses with no wrapper macro.

- [ ] **Step 1: Write the binding**

Create `dts/bindings/video/arducam,pivariety.yaml`:

```yaml
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

description: |
  Arducam Pivariety camera module.

  Pivariety modules are not bare sensors: an on-module MCU exposes a small
  self-describing register interface, so one driver serves the whole family.
  Formats, resolutions and controls are enumerated at runtime, which is why
  this binding carries no sensor-specific properties.

  The module reports no link frequency of its own, so link-frequencies on the
  endpoint is required -- the CSI-2 receiver needs it to configure its D-PHY.

compatible: "arducam,pivariety"

include: [i2c-device.yaml]

child-binding:
  child-binding:
    include: video-interfaces.yaml

properties:
  port:
    type: phandle
    description: Output port node.
```

- [ ] **Step 2: Add Kconfig**

Append to `drivers/video/Kconfig`:

```kconfig
config VIDEO_ARDUCAM_PIVARIETY
	bool "Arducam Pivariety camera"
	select I2C
	depends on DT_HAS_ARDUCAM_PIVARIETY_ENABLED
	help
	  Enable the Arducam Pivariety camera driver. Pivariety modules carry an
	  on-board MCU that describes the module's formats, resolutions and
	  controls over I2C, so this one driver serves every camera in the family
	  (including the Arducam ToF depth camera) without per-sensor register
	  tables.

	  Frames are delivered raw. Any depth or Bayer processing is the
	  application's problem.

config VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS
	int "Maximum enumerated (format, resolution) pairs"
	default 16
	depends on VIDEO_ARDUCAM_PIVARIETY
	help
	  Upper bound on the video_format_cap array the driver builds at init.
	  One entry is needed per (pixel format, resolution) pair, not per pixel
	  format, since each entry carries its own width/height bounds.

	  A module reporting more pairs than this is truncated with a warning
	  rather than rejected: a partially usable camera beats a dead one.
```

- [ ] **Step 3: Add the CMakeLists entry**

In `drivers/video/CMakeLists.txt`, immediately after the `CONFIG_VIDEO_IMX219_OOT` line:

```cmake
zephyr_library_sources_ifdef(CONFIG_VIDEO_ARDUCAM_PIVARIETY arducam_pivariety.c)
```

- [ ] **Step 4: Write the failing test**

Create `tests/drivers/arducam_pivariety/src/main.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/ztest.h>

#include "pivariety_emul.h"

static const struct device *const cam = DEVICE_DT_GET(DT_NODELABEL(pivariety));

ZTEST(pivariety, test_device_is_ready)
{
	zassert_true(device_is_ready(cam), "probe should succeed against the emulator");
}

ZTEST(pivariety, test_probe_reads_identity_registers)
{
	/* Probe must read all three identity registers, or a wrong/absent module
	 * would bind silently.
	 */
	zassert_true(pivariety_emul_was_read(DEVICE_ID_REG), "DEVICE_ID_REG not read");
	zassert_true(pivariety_emul_was_read(SENSOR_ID_REG), "SENSOR_ID_REG not read");
	zassert_true(pivariety_emul_was_read(DEVICE_VERSION_REG), "DEVICE_VERSION_REG not read");
}

ZTEST_SUITE(pivariety, NULL, NULL, NULL, NULL, NULL);
```

- [ ] **Step 5: Write the emulator**

Create `tests/drivers/arducam_pivariety/src/pivariety_emul.h`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PIVARIETY_EMUL_H_
#define PIVARIETY_EMUL_H_

#include <stdbool.h>
#include <stdint.h>

#define DEVICE_REG_BASE     0x0100
#define PIXFORMAT_REG_BASE  0x0200
#define FORMAT_REG_BASE     0x0300
#define CTRL_REG_BASE       0x0400

#define MODE_SELECT_REG     (DEVICE_REG_BASE | 0x0000)
#define DEVICE_VERSION_REG  (DEVICE_REG_BASE | 0x0001)
#define SENSOR_ID_REG       (DEVICE_REG_BASE | 0x0002)
#define DEVICE_ID_REG       (DEVICE_REG_BASE | 0x0003)
#define SYSTEM_IDLE_REG     (DEVICE_REG_BASE | 0x0007)

#define PIXFORMAT_INDEX_REG  (PIXFORMAT_REG_BASE | 0x0000)
#define PIXFORMAT_TYPE_REG   (PIXFORMAT_REG_BASE | 0x0001)
#define PIXFORMAT_ORDER_REG  (PIXFORMAT_REG_BASE | 0x0002)
#define MIPI_LANES_REG       (PIXFORMAT_REG_BASE | 0x0003)

#define RESOLUTION_INDEX_REG (FORMAT_REG_BASE | 0x0000)
#define FORMAT_WIDTH_REG     (FORMAT_REG_BASE | 0x0001)
#define FORMAT_HEIGHT_REG    (FORMAT_REG_BASE | 0x0002)

#define NO_DATA_AVAILABLE 0xFFFFFFFEU

/** True if the driver has read this register since boot. */
bool pivariety_emul_was_read(uint16_t reg);

/** Last value the driver wrote to this register, or NO_DATA_AVAILABLE. */
uint32_t pivariety_emul_last_write(uint16_t reg);

#endif /* PIVARIETY_EMUL_H_ */
```

Create `tests/drivers/arducam_pivariety/src/pivariety_emul.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C emulator for an Arducam Pivariety module.
 *
 * Models the index/attribute enumeration protocol rather than a flat register
 * file: writing PIXFORMAT_INDEX_REG or RESOLUTION_INDEX_REG changes what the
 * corresponding attribute registers read back, and out-of-range indices return
 * NO_DATA_AVAILABLE. That behaviour is the thing worth testing; a flat register
 * file would let a broken enumeration loop pass.
 *
 * Advertised module: one greyscale RAW10 format (the shape the ToF camera is
 * expected to report) at two resolutions, plus one Bayer RAW8 format, so the
 * fourcc mapping is exercised on both paths.
 */

#define DT_DRV_COMPAT arducam_pivariety

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "pivariety_emul.h"

#define EMUL_MAX_TRACKED_REG 0x0500

struct piv_emul_fmt {
	uint32_t data_type;
	uint32_t order;
	uint32_t lanes;
	uint32_t res[2][2]; /* {width, height} */
	uint32_t num_res;
};

static const struct piv_emul_fmt emul_fmts[] = {
	{.data_type = 0x2b, .order = 4, .lanes = 2, .res = {{240, 180}, {120, 90}}, .num_res = 2},
	{.data_type = 0x2a, .order = 0, .lanes = 2, .res = {{640, 480}, {0, 0}}, .num_res = 1},
};

static uint32_t emul_pixfmt_index;
static uint32_t emul_res_index;
static uint32_t emul_mode_select;
static bool emul_read_seen[EMUL_MAX_TRACKED_REG];
static uint32_t emul_last_write[EMUL_MAX_TRACKED_REG];

bool pivariety_emul_was_read(uint16_t reg)
{
	return reg < EMUL_MAX_TRACKED_REG && emul_read_seen[reg];
}

uint32_t pivariety_emul_last_write(uint16_t reg)
{
	return reg < EMUL_MAX_TRACKED_REG ? emul_last_write[reg] : NO_DATA_AVAILABLE;
}

static uint32_t emul_read_reg(uint16_t reg)
{
	const struct piv_emul_fmt *fmt = NULL;

	if (reg < EMUL_MAX_TRACKED_REG) {
		emul_read_seen[reg] = true;
	}

	if (emul_pixfmt_index < ARRAY_SIZE(emul_fmts)) {
		fmt = &emul_fmts[emul_pixfmt_index];
	}

	switch (reg) {
	case SYSTEM_IDLE_REG:
		return 1; /* always idle */
	case DEVICE_VERSION_REG:
		return 0x0001;
	case SENSOR_ID_REG:
		return 0x1234;
	case DEVICE_ID_REG:
		return 0x0001;
	case MODE_SELECT_REG:
		return emul_mode_select;
	case PIXFORMAT_TYPE_REG:
		return fmt ? fmt->data_type : NO_DATA_AVAILABLE;
	case PIXFORMAT_ORDER_REG:
		return fmt ? fmt->order : NO_DATA_AVAILABLE;
	case MIPI_LANES_REG:
		return fmt ? fmt->lanes : NO_DATA_AVAILABLE;
	case FORMAT_WIDTH_REG:
		if (fmt == NULL || emul_res_index >= fmt->num_res) {
			return NO_DATA_AVAILABLE;
		}
		return fmt->res[emul_res_index][0];
	case FORMAT_HEIGHT_REG:
		if (fmt == NULL || emul_res_index >= fmt->num_res) {
			return NO_DATA_AVAILABLE;
		}
		return fmt->res[emul_res_index][1];
	default:
		return NO_DATA_AVAILABLE;
	}
}

static void emul_write_reg(uint16_t reg, uint32_t val)
{
	if (reg < EMUL_MAX_TRACKED_REG) {
		emul_last_write[reg] = val;
	}

	switch (reg) {
	case PIXFORMAT_INDEX_REG:
		emul_pixfmt_index = val;
		break;
	case RESOLUTION_INDEX_REG:
		emul_res_index = val;
		break;
	case MODE_SELECT_REG:
		emul_mode_select = val;
		break;
	default:
		break;
	}
}

static int piv_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
			     int addr)
{
	ARG_UNUSED(target);
	ARG_UNUSED(addr);

	if (num_msgs == 1 && msgs[0].len == 6 && !(msgs[0].flags & I2C_MSG_READ)) {
		/* Write: 2-byte BE address followed by 4-byte BE value. */
		emul_write_reg(sys_get_be16(&msgs[0].buf[0]), sys_get_be32(&msgs[0].buf[2]));
		return 0;
	}

	if (num_msgs == 2 && msgs[0].len == 2 && !(msgs[0].flags & I2C_MSG_READ) &&
	    msgs[1].len == 4 && (msgs[1].flags & I2C_MSG_READ)) {
		sys_put_be32(emul_read_reg(sys_get_be16(&msgs[0].buf[0])), msgs[1].buf);
		return 0;
	}

	return -EIO;
}

static const struct i2c_emul_api piv_emul_api = {
	.transfer = piv_emul_transfer,
};

static int piv_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);
	return 0;
}

#define PIV_EMUL_DEFINE(n)                                                                         \
	EMUL_DT_INST_DEFINE(n, piv_emul_init, NULL, NULL, &piv_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(PIV_EMUL_DEFINE)
```

- [ ] **Step 6: Write the test harness files**

`tests/drivers/arducam_pivariety/CMakeLists.txt`:

```cmake
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(arducam_pivariety_test)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
```

`tests/drivers/arducam_pivariety/prj.conf`:

```kconfig
CONFIG_ZTEST=y
CONFIG_EMUL=y

CONFIG_I2C=y
CONFIG_I2C_EMUL=y

CONFIG_VIDEO=y
CONFIG_VIDEO_ARDUCAM_PIVARIETY=y
CONFIG_VIDEO_LOG_LEVEL_DBG=y
```

`tests/drivers/arducam_pivariety/testcase.yaml`:

```yaml
common:
  tags: drivers
  platform_allow:
    - qemu_cortex_m3
    - native_sim

tests:
  drivers.video.arducam_pivariety: {}
```

`tests/drivers/arducam_pivariety/boards/qemu_cortex_m3.overlay` — this is the one used for local red/green. `qemu_cortex_m3` has no I2C controller, so the emulated bus is declared rather than referenced:

```dts
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/video/video-interfaces.h>

/ {
	fake_i2c_bus: i2c@100 {
		status = "okay";
		compatible = "zephyr,i2c-emul-controller";
		clock-frequency = <I2C_BITRATE_STANDARD>;
		#address-cells = <1>;
		#size-cells = <0>;
		reg = <0x100 4>;

		pivariety: pivariety@c {
			compatible = "arducam,pivariety";
			reg = <0x0c>;

			port {
				pivariety_ep_out: endpoint {
					bus-type = <VIDEO_BUS_TYPE_CSI2_DPHY>;
					data-lanes = <1 2>;
					link-frequencies = <493500000>;
				};
			};
		};
	};
};
```

`tests/drivers/arducam_pivariety/boards/native_sim.overlay` — same node, but hung off the bus `native_sim` already provides, so CI covers that platform too:

```dts
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/dt-bindings/video/video-interfaces.h>

&i2c0 {
	status = "okay";

	pivariety: pivariety@c {
		compatible = "arducam,pivariety";
		reg = <0x0c>;

		port {
			pivariety_ep_out: endpoint {
				bus-type = <VIDEO_BUS_TYPE_CSI2_DPHY>;
				data-lanes = <1 2>;
				link-frequencies = <493500000>;
			};
		};
	};
};
```

- [ ] **Step 7: Run the test to verify it fails**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: FAIL at build with `arducam_pivariety.c: No such file or directory` — the driver does not exist yet.

- [ ] **Step 8: Write the driver skeleton and probe**

Create `drivers/video/arducam_pivariety.c`:

```c
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Arducam Pivariety camera driver.
 *
 * Pivariety modules are not bare sensors. An on-module MCU exposes a small
 * self-describing register interface, so one driver serves the whole family
 * without per-sensor register tables: formats, resolutions and controls are
 * enumerated at init by writing an index register and reading back attribute
 * registers until the NO_DATA_AVAILABLE sentinel.
 *
 * Reference: drivers/media/i2c/arducam-pivariety.{c,h} in the Raspberry Pi
 * kernel. Frames are delivered raw; depth or Bayer processing is the
 * application's problem.
 */

#define DT_DRV_COMPAT arducam_pivariety

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/video/video.h>

#include "video_common.h"

LOG_MODULE_REGISTER(arducam_pivariety, CONFIG_VIDEO_LOG_LEVEL);

#define DEVICE_REG_BASE    0x0100
#define PIXFORMAT_REG_BASE 0x0200
#define FORMAT_REG_BASE    0x0300
#define CTRL_REG_BASE      0x0400

#define MODE_SELECT_REG    (DEVICE_REG_BASE | 0x0000)
#define DEVICE_VERSION_REG (DEVICE_REG_BASE | 0x0001)
#define SENSOR_ID_REG      (DEVICE_REG_BASE | 0x0002)
#define DEVICE_ID_REG      (DEVICE_REG_BASE | 0x0003)
#define SYSTEM_IDLE_REG    (DEVICE_REG_BASE | 0x0007)

#define PIXFORMAT_INDEX_REG (PIXFORMAT_REG_BASE | 0x0000)
#define PIXFORMAT_TYPE_REG  (PIXFORMAT_REG_BASE | 0x0001)
#define PIXFORMAT_ORDER_REG (PIXFORMAT_REG_BASE | 0x0002)
#define MIPI_LANES_REG      (PIXFORMAT_REG_BASE | 0x0003)

#define RESOLUTION_INDEX_REG (FORMAT_REG_BASE | 0x0000)
#define FORMAT_WIDTH_REG     (FORMAT_REG_BASE | 0x0001)
#define FORMAT_HEIGHT_REG    (FORMAT_REG_BASE | 0x0002)

#define NO_DATA_AVAILABLE 0xFFFFFFFEU

#define PIV_MODE_STANDBY   0x00
#define PIV_MODE_STREAMING 0x01

/* The kernel driver spins on SYSTEM_IDLE_REG without a bound. A wedged module
 * must return an error here rather than hanging whichever thread called us.
 */
#define PIV_IDLE_TIMEOUT_MS 100

struct pivariety_config {
	struct i2c_dt_spec i2c;
	int64_t link_freq_hz;
};

struct pivariety_data {
	struct video_format fmt;
};

/*
 * Pivariety is not a CCI device. One 16-bit address selects one 32-bit value,
 * transferred whole: a 6-byte write, or a 2-byte address write followed by a
 * 4-byte read.
 *
 * Zephyr's video_{read,write}_cci_reg() cannot express this. They implement
 * genuine CCI semantics, where a 32-bit value occupies four consecutive
 * byte-addressed registers, so reading SYSTEM_IDLE_REG through them would issue
 * four transactions against 0x0107..0x010a. Frame the transfer directly
 * instead, mirroring pivariety_{read,write}_reg() in the kernel driver.
 */
static int piv_read(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct pivariety_config *cfg = dev->config;
	uint8_t addr[2];
	uint8_t buf[4];
	int ret;

	sys_put_be16(reg, addr);

	ret = i2c_write_read_dt(&cfg->i2c, addr, sizeof(addr), buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	*val = sys_get_be32(buf);

	return 0;
}

static int piv_write(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct pivariety_config *cfg = dev->config;
	uint8_t buf[6];

	sys_put_be16(reg, &buf[0]);
	sys_put_be32(val, &buf[2]);

	return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

/* The module is single-threaded internally: it must report idle before the next
 * command, or indices and attributes get out of step.
 */
static int piv_wait_idle(const struct device *dev)
{
	int64_t deadline = k_uptime_get() + PIV_IDLE_TIMEOUT_MS;
	uint32_t idle = 0;

	do {
		int ret = piv_read(dev, SYSTEM_IDLE_REG, &idle);

		if (ret < 0) {
			return ret;
		}
		if (idle != 0 && idle != NO_DATA_AVAILABLE) {
			return 0;
		}
		k_msleep(1);
	} while (k_uptime_get() < deadline);

	LOG_ERR("Module never reported idle");
	return -ETIMEDOUT;
}

static int pivariety_init(const struct device *dev)
{
	const struct pivariety_config *cfg = dev->config;
	uint32_t device_id = 0, sensor_id = 0, version = 0;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	ret = piv_wait_idle(dev);
	if (ret < 0) {
		return ret;
	}

	ret = piv_read(dev, DEVICE_ID_REG, &device_id);
	ret |= piv_read(dev, SENSOR_ID_REG, &sensor_id);
	ret |= piv_read(dev, DEVICE_VERSION_REG, &version);
	if (ret < 0) {
		LOG_ERR("Failed to read identity registers (%d)", ret);
		return -ENODEV;
	}

	if (device_id == NO_DATA_AVAILABLE) {
		LOG_ERR("No Pivariety module responding at 0x%02x", cfg->i2c.addr);
		return -ENODEV;
	}

	LOG_INF("Pivariety device 0x%04x sensor 0x%04x version 0x%04x, link %lld Hz", device_id,
		sensor_id, version, cfg->link_freq_hz);

	return 0;
}

static DEVICE_API(video, pivariety_driver_api) = {};

#define PIV_ENDPOINT(n) DT_INST_ENDPOINT_BY_ID(n, 0, 0)

#define PIVARIETY_INIT(n)                                                                          \
	static struct pivariety_data pivariety_data_##n;                                           \
                                                                                                   \
	static const struct pivariety_config pivariety_cfg_##n = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.link_freq_hz = DT_PROP_BY_IDX(PIV_ENDPOINT(n), link_frequencies, 0),              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &pivariety_init, NULL, &pivariety_data_##n,                       \
			      &pivariety_cfg_##n, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,          \
			      &pivariety_driver_api);                                              \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(pivariety_##n, DEVICE_DT_INST_GET(n), NULL);

DT_INST_FOREACH_STATUS_OKAY(PIVARIETY_INIT)
```

- [ ] **Step 9: Run the test to verify it passes**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: 2/2 PASS.

- [ ] **Step 10: Verify formatting**

Run: `mise x -- clang-format --dry-run --Werror deps/modules/lib/rosterloh-drivers/drivers/video/arducam_pivariety.c deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety/src/*.c deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety/src/*.h`

Expected: no output.

- [ ] **Step 11: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add dts/bindings/video/arducam,pivariety.yaml drivers/video/arducam_pivariety.c \
        drivers/video/Kconfig drivers/video/CMakeLists.txt tests/drivers/arducam_pivariety \
        docs/superpowers/specs/2026-08-24-arducam-pivariety-driver-design.md \
        docs/superpowers/plans/2026-08-24-arducam-pivariety-driver.md
git commit -m "video: arducam_pivariety: binding, Kconfig and probe"
```

---

### Task 2: Format enumeration and get_caps

**Files:**
- Modify: `drivers/video/arducam_pivariety.c`
- Test: `tests/drivers/arducam_pivariety/src/main.c`

**Interfaces:**
- Consumes: `piv_read`, `piv_write`, `piv_wait_idle`, `struct pivariety_data` from Task 1.
- Produces: `struct pivariety_data` gains `struct video_format_cap caps[CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS + 1]`, `uint8_t num_caps`, and `struct piv_cap_index { uint8_t fmt_idx; uint8_t res_idx; } cap_index[...]`. Also `static uint32_t piv_fourcc(uint32_t data_type, uint32_t order)` and `static int pivariety_get_caps(const struct device *dev, struct video_caps *caps)`. Task 3 uses `cap_index` to map a chosen format back to module indices.

- [ ] **Step 1: Write the failing test**

Append to `tests/drivers/arducam_pivariety/src/main.c`:

```c
ZTEST(pivariety, test_enumerates_all_format_resolution_pairs)
{
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
	int n = 0;

	zassert_ok(video_get_caps(cam, &caps));

	while (caps.format_caps[n].pixelformat != 0) {
		n++;
	}

	/* The emulator advertises RAW10-grey at 2 resolutions and RAW8-BGGR at 1. */
	zassert_equal(n, 3, "expected 3 (format, resolution) pairs, got %d", n);
}

ZTEST(pivariety, test_maps_greyscale_and_bayer_data_types)
{
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};

	zassert_ok(video_get_caps(cam, &caps));

	/* RAW10 (0x2b) + order 4 (GRAY) -> Y10P, not a Bayer format. */
	zassert_equal(caps.format_caps[0].pixelformat, VIDEO_PIX_FMT_Y10P);
	zassert_equal(caps.format_caps[0].width_min, 240);
	zassert_equal(caps.format_caps[0].height_min, 180);
	zassert_equal(caps.format_caps[1].pixelformat, VIDEO_PIX_FMT_Y10P);
	zassert_equal(caps.format_caps[1].width_min, 120);

	/* RAW8 (0x2a) + order 0 (BGGR) -> SBGGR8. */
	zassert_equal(caps.format_caps[2].pixelformat, VIDEO_PIX_FMT_SBGGR8);
	zassert_equal(caps.format_caps[2].width_min, 640);
}
```

Add `#include <zephyr/video/formats.h>` to the test's includes.

- [ ] **Step 2: Run the test to verify it fails**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: FAIL — `video_get_caps` returns `-ENOSYS` because `pivariety_driver_api` is empty.

- [ ] **Step 3: Add the fourcc mapping**

Insert into `arducam_pivariety.c` above `pivariety_init`:

```c
/* PIXFORMAT_TYPE_REG returns the MIPI CSI-2 data type directly, so the fourcc
 * follows from the (type, order) pair. Order 4 means greyscale -- which is what
 * a ToF module reports, since its raw phase data is not Bayer at all.
 */
#define PIV_DT_RAW8  0x2a
#define PIV_DT_RAW10 0x2b
#define PIV_DT_RAW12 0x2c

#define PIV_ORDER_BGGR 0
#define PIV_ORDER_GBRG 1
#define PIV_ORDER_GRBG 2
#define PIV_ORDER_RGGB 3
#define PIV_ORDER_GRAY 4

static uint32_t piv_fourcc(uint32_t data_type, uint32_t order)
{
	static const uint32_t bayer[3][4] = {
		{VIDEO_PIX_FMT_SBGGR8, VIDEO_PIX_FMT_SGBRG8, VIDEO_PIX_FMT_SGRBG8,
		 VIDEO_PIX_FMT_SRGGB8},
		{VIDEO_PIX_FMT_SBGGR10P, VIDEO_PIX_FMT_SGBRG10P, VIDEO_PIX_FMT_SGRBG10P,
		 VIDEO_PIX_FMT_SRGGB10P},
		{VIDEO_PIX_FMT_SBGGR12P, VIDEO_PIX_FMT_SGBRG12P, VIDEO_PIX_FMT_SGRBG12P,
		 VIDEO_PIX_FMT_SRGGB12P},
	};
	static const uint32_t grey[3] = {VIDEO_PIX_FMT_GREY, VIDEO_PIX_FMT_Y10P,
					 VIDEO_PIX_FMT_Y12P};
	uint32_t depth;

	switch (data_type) {
	case PIV_DT_RAW8:
		depth = 0;
		break;
	case PIV_DT_RAW10:
		depth = 1;
		break;
	case PIV_DT_RAW12:
		depth = 2;
		break;
	default:
		return 0;
	}

	if (order == PIV_ORDER_GRAY) {
		return grey[depth];
	}
	if (order > PIV_ORDER_RGGB) {
		return 0;
	}

	return bayer[depth][order];
}
```

- [ ] **Step 4: Add enumeration and get_caps**

Extend `struct pivariety_data`:

```c
struct piv_cap_index {
	uint8_t fmt_idx;
	uint8_t res_idx;
};

struct pivariety_data {
	struct video_format fmt;
	struct video_format_cap caps[CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS + 1];
	struct piv_cap_index cap_index[CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS];
	uint8_t num_caps;
};
```

Insert above `pivariety_init`:

```c
/* Walk formats, and each format's resolutions, into a flat caps array. One
 * video_format_cap entry is needed per (format, resolution) pair because each
 * entry carries its own width/height bounds; cap_index remembers which module
 * indices produced each entry so set_fmt can select it again.
 */
static int piv_enumerate(const struct device *dev)
{
	struct pivariety_data *data = dev->data;
	int ret;

	for (uint32_t f = 0; f < UINT8_MAX; f++) {
		uint32_t type = 0, order = 0, lanes = 0, fourcc;

		ret = piv_wait_idle(dev);
		ret |= piv_write(dev, PIXFORMAT_INDEX_REG, f);
		ret |= piv_read(dev, PIXFORMAT_TYPE_REG, &type);
		if (ret < 0) {
			return ret;
		}
		if (type == NO_DATA_AVAILABLE) {
			break;
		}

		ret = piv_read(dev, PIXFORMAT_ORDER_REG, &order);
		ret |= piv_read(dev, MIPI_LANES_REG, &lanes);
		if (ret < 0) {
			return ret;
		}

		fourcc = piv_fourcc(type, order);
		if (fourcc == 0) {
			LOG_DBG("Skipping unsupported format %u (dt 0x%02x order %u)", f, type,
				order);
			continue;
		}

		for (uint32_t r = 0; r < UINT8_MAX; r++) {
			uint32_t width = 0, height = 0;
			struct video_format_cap *cap;

			ret = piv_wait_idle(dev);
			ret |= piv_write(dev, RESOLUTION_INDEX_REG, r);
			ret |= piv_read(dev, FORMAT_WIDTH_REG, &width);
			if (ret < 0) {
				return ret;
			}
			if (width == NO_DATA_AVAILABLE) {
				break;
			}

			ret = piv_read(dev, FORMAT_HEIGHT_REG, &height);
			if (ret < 0) {
				return ret;
			}

			if (data->num_caps >= CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS) {
				LOG_WRN("Module reports more than %d (format, resolution) "
					"pairs; truncating. Raise "
					"CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS.",
					CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS);
				return 0;
			}

			cap = &data->caps[data->num_caps];
			cap->pixelformat = fourcc;
			cap->width_min = width;
			cap->width_max = width;
			cap->height_min = height;
			cap->height_max = height;
			cap->width_step = 0;
			cap->height_step = 0;

			data->cap_index[data->num_caps].fmt_idx = f;
			data->cap_index[data->num_caps].res_idx = r;
			data->num_caps++;

			LOG_DBG("cap %u: %ux%u fourcc 0x%08x lanes %u", data->num_caps - 1, width,
				height, fourcc, lanes);
		}
	}

	return 0;
}

static int pivariety_get_caps(const struct device *dev, struct video_caps *caps)
{
	struct pivariety_data *data = dev->data;

	if (caps->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	caps->format_caps = data->caps;
	caps->min_vbuf_count = 1;

	return 0;
}
```

Call it at the end of `pivariety_init`, before `return 0`:

```c
	ret = piv_enumerate(dev);
	if (ret < 0) {
		LOG_ERR("Format enumeration failed (%d)", ret);
		return ret;
	}

	if (data->num_caps == 0) {
		LOG_ERR("Module advertised no supported formats");
		return -ENOTSUP;
	}

	LOG_INF("Enumerated %u (format, resolution) pairs", data->num_caps);
```

`pivariety_init` needs `struct pivariety_data *data = dev->data;` at the top. Register the callback:

```c
static DEVICE_API(video, pivariety_driver_api) = {
	.get_caps = pivariety_get_caps,
};
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: 4/4 PASS.

- [ ] **Step 6: Add the diagnostic shell command**

Append to `arducam_pivariety.c`:

```c
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>

/* The analogue of imx219_regs: prove the I2C protocol works and show exactly
 * what the module advertised, before blaming the CSI receiver for anything.
 */
static int cmd_pivariety_regs(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct pivariety_config *cfg = dev->config;
	struct pivariety_data *data = dev->data;
	static const struct {
		const char *name;
		uint32_t reg;
	} regs[] = {
		{"DEVICE_ID", DEVICE_ID_REG},
		{"SENSOR_ID", SENSOR_ID_REG},
		{"DEVICE_VERSION", DEVICE_VERSION_REG},
		{"SYSTEM_IDLE", SYSTEM_IDLE_REG},
		{"MODE_SELECT", MODE_SELECT_REG},
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		uint32_t val = 0;
		int ret = piv_read(dev, regs[i].reg, &val);

		if (ret < 0) {
			shell_error(sh, "%-16s read failed (%d)", regs[i].name, ret);
		} else {
			shell_print(sh, "%-16s = 0x%08x (%u)", regs[i].name, val, val);
		}
	}

	shell_print(sh, "link frequency  = %lld Hz", cfg->link_freq_hz);
	shell_print(sh, "enumerated caps = %u", data->num_caps);

	for (uint8_t i = 0; i < data->num_caps; i++) {
		shell_print(sh, "  [%u] %ux%u fourcc 0x%08x (module fmt %u res %u)", i,
			    data->caps[i].width_min, data->caps[i].height_min,
			    data->caps[i].pixelformat, data->cap_index[i].fmt_idx,
			    data->cap_index[i].res_idx);
	}

	return 0;
}
SHELL_CMD_REGISTER(pivariety_regs, NULL, "Dump Arducam Pivariety identity and enumerated formats",
		   cmd_pivariety_regs);
#endif /* CONFIG_SHELL */
```

- [ ] **Step 7: Verify formatting and commit**

```bash
cd /Users/richard.osterloh/workspace/zephyr-applications
mise x -- clang-format --dry-run --Werror \
  deps/modules/lib/rosterloh-drivers/drivers/video/arducam_pivariety.c \
  deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety/src/main.c
cd deps/modules/lib/rosterloh-drivers
git add drivers/video/arducam_pivariety.c tests/drivers/arducam_pivariety/src/main.c
git commit -m "video: arducam_pivariety: enumerate formats and resolutions"
```

---

### Task 3: set_fmt, get_fmt and set_stream

**Files:**
- Modify: `drivers/video/arducam_pivariety.c`
- Test: `tests/drivers/arducam_pivariety/src/main.c`

**Interfaces:**
- Consumes: `piv_write`, `piv_wait_idle`, `data->caps`, `data->cap_index`, `data->num_caps` from Task 2.
- Produces: `pivariety_set_fmt`, `pivariety_get_fmt`, `pivariety_set_stream` registered in `pivariety_driver_api`. After this task the driver is functionally complete for capture.

- [ ] **Step 1: Write the failing test**

Append to `tests/drivers/arducam_pivariety/src/main.c`:

```c
ZTEST(pivariety, test_set_fmt_selects_module_indices)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_SBGGR8,
		.width = 640,
		.height = 480,
	};

	zassert_ok(video_set_format(cam, &fmt));

	/* SBGGR8 640x480 is the emulator's format 1, resolution 0. */
	zassert_equal(pivariety_emul_last_write(PIXFORMAT_INDEX_REG), 1);
	zassert_equal(pivariety_emul_last_write(RESOLUTION_INDEX_REG), 0);
}

ZTEST(pivariety, test_set_fmt_rejects_unadvertised_format)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width = 640,
		.height = 480,
	};

	zassert_equal(video_set_format(cam, &fmt), -ENOTSUP);
}

ZTEST(pivariety, test_get_fmt_returns_what_was_set)
{
	struct video_format set = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_Y10P,
		.width = 240,
		.height = 180,
	};
	struct video_format got = {.type = VIDEO_BUF_TYPE_OUTPUT};

	zassert_ok(video_set_format(cam, &set));
	zassert_ok(video_get_format(cam, &got));

	zassert_equal(got.pixelformat, VIDEO_PIX_FMT_Y10P);
	zassert_equal(got.width, 240);
	zassert_equal(got.height, 180);
}

ZTEST(pivariety, test_stream_start_stop_writes_mode_select)
{
	zassert_ok(video_stream_start(cam, VIDEO_BUF_TYPE_OUTPUT));
	zassert_equal(pivariety_emul_last_write(MODE_SELECT_REG), 1);

	zassert_ok(video_stream_stop(cam, VIDEO_BUF_TYPE_OUTPUT));
	zassert_equal(pivariety_emul_last_write(MODE_SELECT_REG), 0);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: FAIL — `video_set_format` returns `-ENOSYS`.

- [ ] **Step 3: Implement the three callbacks**

Insert above the shell block in `arducam_pivariety.c`:

```c
static int pivariety_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct pivariety_data *data = dev->data;
	size_t idx;
	int ret;

	if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	/* The caps array is built in the same order as cap_index, so the index the
	 * subsystem's matcher returns is exactly the one that remembers which
	 * module format/resolution indices produced this entry.
	 */
	ret = video_format_caps_index(data->caps, fmt, &idx);
	if (ret < 0) {
		LOG_ERR("Format '%s' %ux%u not advertised by this module",
			VIDEO_FOURCC_TO_STR(fmt->pixelformat), fmt->width, fmt->height);
		return -ENOTSUP;
	}

	ret = piv_wait_idle(dev);
	ret |= piv_write(dev, PIXFORMAT_INDEX_REG, data->cap_index[idx].fmt_idx);
	ret |= piv_wait_idle(dev);
	ret |= piv_write(dev, RESOLUTION_INDEX_REG, data->cap_index[idx].res_idx);
	if (ret < 0) {
		LOG_ERR("Failed to select format %zu (%d)", idx, ret);
		return ret;
	}

	/* The module has no register reporting pitch or frame size, so derive them
	 * from the fourcc.
	 */
	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	data->fmt = *fmt;

	return 0;
}

static int pivariety_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct pivariety_data *data = dev->data;

	if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	*fmt = data->fmt;

	return 0;
}

static int pivariety_set_stream(const struct device *dev, bool on, enum video_buf_type type)
{
	int ret;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	ret = piv_wait_idle(dev);
	if (ret < 0) {
		return ret;
	}

	return piv_write(dev, MODE_SELECT_REG, on ? PIV_MODE_STREAMING : PIV_MODE_STANDBY);
}
```

Register them:

```c
static DEVICE_API(video, pivariety_driver_api) = {
	.get_caps = pivariety_get_caps,
	.set_format = pivariety_set_fmt,
	.get_format = pivariety_get_fmt,
	.set_stream = pivariety_set_stream,
};
```

At the end of `pivariety_init`, after the `num_caps == 0` check, set the default format so `get_fmt` is meaningful before any `set_fmt`:

```c
	data->fmt.type = VIDEO_BUF_TYPE_OUTPUT;
	data->fmt.pixelformat = data->caps[0].pixelformat;
	data->fmt.width = data->caps[0].width_min;
	data->fmt.height = data->caps[0].height_min;

	ret = pivariety_set_fmt(dev, &data->fmt);
	if (ret < 0) {
		return ret;
	}
```

`pivariety_set_fmt` must be declared above `pivariety_init` or defined before it.

- [ ] **Step 4: Run the test to verify it passes**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: 8/8 PASS.

- [ ] **Step 5: Verify formatting and commit**

```bash
cd /Users/richard.osterloh/workspace/zephyr-applications
mise x -- clang-format --dry-run --Werror \
  deps/modules/lib/rosterloh-drivers/drivers/video/arducam_pivariety.c \
  deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety/src/main.c
cd deps/modules/lib/rosterloh-drivers
git add drivers/video/arducam_pivariety.c tests/drivers/arducam_pivariety/src/main.c
git commit -m "video: arducam_pivariety: format selection and streaming"
```

---

### Task 4: Controls and link frequency

**Files:**
- Modify: `drivers/video/arducam_pivariety.c`
- Test: `tests/drivers/arducam_pivariety/src/main.c`
- Modify: `tests/drivers/arducam_pivariety/src/pivariety_emul.c`

**Interfaces:**
- Consumes: everything from Tasks 1–3.
- Produces: `struct pivariety_ctrls { struct video_ctrl linkfreq; struct video_ctrl generic[8]; }` inside `pivariety_data`, `pivariety_set_ctrl` in the API. This is the last driver task; `video_get_csi_link_freq()` works from here on, which Task 6 depends on.

- [ ] **Step 1: Add control registers to the emulator**

In `pivariety_emul.h`, add below the resolution defines:

```c
#define CTRL_INDEX_REG (CTRL_REG_BASE | 0x0000)
#define CTRL_ID_REG    (CTRL_REG_BASE | 0x0001)
#define CTRL_MIN_REG   (CTRL_REG_BASE | 0x0002)
#define CTRL_MAX_REG   (CTRL_REG_BASE | 0x0003)
#define CTRL_STEP_REG  (CTRL_REG_BASE | 0x0004)
#define CTRL_DEF_REG   (CTRL_REG_BASE | 0x0005)
#define CTRL_VALUE_REG (CTRL_REG_BASE | 0x0006)
```

In `pivariety_emul.c`, add above `emul_read_reg`:

```c
/* One mappable control (exposure) and one with no Zephyr equivalent, so the
 * skip path is exercised too. 0x00980911 is V4L2_CID_EXPOSURE.
 */
struct piv_emul_ctrl {
	uint32_t id, min, max, step, def;
};

static const struct piv_emul_ctrl emul_ctrls[] = {
	{.id = 0x00980911, .min = 0, .max = 65535, .step = 1, .def = 100},
	{.id = 0x009a0999, .min = 0, .max = 1, .step = 1, .def = 0},
};

static uint32_t emul_ctrl_index;
static uint32_t emul_ctrl_value;
```

Add these cases to `emul_read_reg`'s switch, before `default`:

```c
	case CTRL_ID_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls)
			       ? emul_ctrls[emul_ctrl_index].id
			       : NO_DATA_AVAILABLE;
	case CTRL_MIN_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls)
			       ? emul_ctrls[emul_ctrl_index].min
			       : NO_DATA_AVAILABLE;
	case CTRL_MAX_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls)
			       ? emul_ctrls[emul_ctrl_index].max
			       : NO_DATA_AVAILABLE;
	case CTRL_STEP_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls)
			       ? emul_ctrls[emul_ctrl_index].step
			       : NO_DATA_AVAILABLE;
	case CTRL_DEF_REG:
		return emul_ctrl_index < ARRAY_SIZE(emul_ctrls)
			       ? emul_ctrls[emul_ctrl_index].def
			       : NO_DATA_AVAILABLE;
	case CTRL_VALUE_REG:
		return emul_ctrl_value;
```

And to `emul_write_reg`'s switch:

```c
	case CTRL_INDEX_REG:
		emul_ctrl_index = val;
		break;
	case CTRL_VALUE_REG:
		emul_ctrl_value = val;
		break;
```

- [ ] **Step 2: Write the failing test**

Append to `tests/drivers/arducam_pivariety/src/main.c`:

```c
ZTEST(pivariety, test_link_frequency_comes_from_devicetree)
{
	int64_t freq = video_get_csi_link_freq(cam, 10, 2);

	/* Pivariety has no link-frequency register; it must come from the
	 * endpoint's link-frequencies property.
	 */
	zassert_equal(freq, 493500000, "got %lld", freq);
}

ZTEST(pivariety, test_maps_known_control_and_skips_unknown)
{
	struct video_ctrl_query cq = {.dev = cam, .id = VIDEO_CID_EXPOSURE};

	zassert_ok(video_query_ctrl(&cq), "exposure should be registered");
	zassert_equal(cq.range.min, 0);
	zassert_equal(cq.range.max, 65535);
	zassert_equal(cq.range.def, 100);

	/* The vendor control has no Zephyr CID and must not have been registered. */
	cq.id = 0x009a0999;
	zassert_not_equal(video_query_ctrl(&cq), 0, "vendor control should be skipped");
}

ZTEST(pivariety, test_set_ctrl_writes_id_then_value)
{
	struct video_control ctrl = {.id = VIDEO_CID_EXPOSURE, .val = 500};

	zassert_ok(video_set_ctrl(cam, &ctrl));
	zassert_equal(pivariety_emul_last_write(CTRL_ID_REG), 0x00980911);
	zassert_equal(pivariety_emul_last_write(CTRL_VALUE_REG), 500);
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: FAIL — `video_get_csi_link_freq` returns an error and `video_query_ctrl` finds no exposure control.

- [ ] **Step 4: Implement controls**

Add the control register defines next to the others in `arducam_pivariety.c`:

```c
#define CTRL_INDEX_REG (CTRL_REG_BASE | 0x0000)
#define CTRL_ID_REG    (CTRL_REG_BASE | 0x0001)
#define CTRL_MIN_REG   (CTRL_REG_BASE | 0x0002)
#define CTRL_MAX_REG   (CTRL_REG_BASE | 0x0003)
#define CTRL_STEP_REG  (CTRL_REG_BASE | 0x0004)
#define CTRL_DEF_REG   (CTRL_REG_BASE | 0x0005)
#define CTRL_VALUE_REG (CTRL_REG_BASE | 0x0006)

#define PIV_MAX_GENERIC_CTRLS 8
```

Extend `pivariety_data`:

```c
	struct video_ctrl linkfreq;
	struct video_ctrl generic[PIV_MAX_GENERIC_CTRLS];
	uint32_t generic_id[PIV_MAX_GENERIC_CTRLS];
	uint8_t num_generic;
	int64_t link_freq_menu[1];
```

Insert above `pivariety_init`:

```c
/* The module reports raw V4L2 control IDs. Register the ones Zephyr also
 * defines and skip the rest -- including Arducam's vendor range -- rather than
 * failing probe over a control nobody asked for.
 */
static bool piv_ctrl_is_supported(uint32_t id)
{
	switch (id) {
	case VIDEO_CID_EXPOSURE:
	case VIDEO_CID_GAIN:
	case VIDEO_CID_ANALOGUE_GAIN:
	case VIDEO_CID_BRIGHTNESS:
	case VIDEO_CID_CONTRAST:
	case VIDEO_CID_SATURATION:
	case VIDEO_CID_HFLIP:
	case VIDEO_CID_VFLIP:
		return true;
	default:
		return false;
	}
}

static int piv_init_ctrls(const struct device *dev)
{
	const struct pivariety_config *cfg = dev->config;
	struct pivariety_data *data = dev->data;
	int ret;

	data->link_freq_menu[0] = cfg->link_freq_hz;

	ret = video_init_int_menu_ctrl(&data->linkfreq, dev, VIDEO_CID_LINK_FREQ, 0,
				       data->link_freq_menu, ARRAY_SIZE(data->link_freq_menu));
	if (ret < 0) {
		return ret;
	}
	data->linkfreq.flags |= VIDEO_CTRL_FLAG_READ_ONLY;

	for (uint32_t i = 0; i < UINT8_MAX; i++) {
		uint32_t id = 0, min = 0, max = 0, step = 0, def = 0;

		ret = piv_wait_idle(dev);
		ret |= piv_write(dev, CTRL_INDEX_REG, i);
		ret |= piv_read(dev, CTRL_ID_REG, &id);
		if (ret < 0) {
			return ret;
		}
		if (id == NO_DATA_AVAILABLE) {
			break;
		}

		if (!piv_ctrl_is_supported(id)) {
			LOG_DBG("Skipping control 0x%08x with no Zephyr equivalent", id);
			continue;
		}

		if (data->num_generic >= PIV_MAX_GENERIC_CTRLS) {
			LOG_WRN("More than %d mappable controls; ignoring the rest",
				PIV_MAX_GENERIC_CTRLS);
			break;
		}

		ret = piv_read(dev, CTRL_MIN_REG, &min);
		ret |= piv_read(dev, CTRL_MAX_REG, &max);
		ret |= piv_read(dev, CTRL_STEP_REG, &step);
		ret |= piv_read(dev, CTRL_DEF_REG, &def);
		if (ret < 0) {
			return ret;
		}

		ret = video_init_ctrl(&data->generic[data->num_generic], dev, id,
				      (struct video_ctrl_range){.min = min,
								.max = max,
								.step = step,
								.def = def});
		if (ret < 0) {
			return ret;
		}

		data->generic_id[data->num_generic] = id;
		data->num_generic++;
	}

	return 0;
}

static int pivariety_set_ctrl(const struct device *dev, unsigned int cid)
{
	struct pivariety_data *data = dev->data;
	int ret;

	for (uint8_t i = 0; i < data->num_generic; i++) {
		if (data->generic_id[i] != cid) {
			continue;
		}

		ret = piv_wait_idle(dev);
		ret |= piv_write(dev, CTRL_ID_REG, cid);
		ret |= piv_write(dev, CTRL_VALUE_REG, data->generic[i].val);

		return ret < 0 ? ret : 0;
	}

	return -ENOTSUP;
}
```

Call `piv_init_ctrls(dev)` as the last thing in `pivariety_init` (returning its result), and add `.set_ctrl = pivariety_set_ctrl,` to the API.

- [ ] **Step 5: Run the test to verify it passes**

Run: `mise x -- west twister -T deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety -p qemu_cortex_m3 --inline-logs`

Expected: 11/11 PASS.

- [ ] **Step 6: Verify formatting and commit**

```bash
cd /Users/richard.osterloh/workspace/zephyr-applications
mise x -- clang-format --dry-run --Werror \
  deps/modules/lib/rosterloh-drivers/drivers/video/arducam_pivariety.c \
  deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety/src/*.c \
  deps/modules/lib/rosterloh-drivers/tests/drivers/arducam_pivariety/src/*.h
cd deps/modules/lib/rosterloh-drivers
git add drivers/video/arducam_pivariety.c tests/drivers/arducam_pivariety/src/
git commit -m "video: arducam_pivariety: controls and DT link frequency"
```

---

### Task 5: Extend the CSI receiver's pixel format table

**Files:**
- Modify: `drivers/video/video_esp32_csi.c` (`csi_pixfmt_info()`)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `csi_pixfmt_info()` accepting greyscale and all four Bayer orders at 8/10/12 bits. Task 6's build depends on this, since the ToF camera is expected to report greyscale.

There is no unit test here — the function is a pure switch compiled only for the ESP32-P4, which neither QEMU nor `native_sim` can build. The build in Task 6 is the check.

- [ ] **Step 1: Replace the format switch**

In `drivers/video/video_esp32_csi.c`, replace the body of `csi_pixfmt_info()`:

```c
/* Map a pixel format to its bits-per-pixel and MIPI CSI-2 data type. Greyscale
 * and Bayer share a data type at each depth -- the CSI-2 payload is identical,
 * only the interpretation differs -- so ToF modules reporting Y10P land on the
 * same RAW10 path as a Bayer sensor.
 */
static int csi_pixfmt_info(uint32_t pixelformat, uint8_t *bpp, uint16_t *data_type)
{
	switch (pixelformat) {
	case VIDEO_PIX_FMT_GREY:
	case VIDEO_PIX_FMT_SBGGR8:
	case VIDEO_PIX_FMT_SGBRG8:
	case VIDEO_PIX_FMT_SGRBG8:
	case VIDEO_PIX_FMT_SRGGB8:
		*bpp = 8;
		*data_type = CSI_DT_RAW8;
		return 0;
	case VIDEO_PIX_FMT_Y10P:
	case VIDEO_PIX_FMT_SBGGR10P:
	case VIDEO_PIX_FMT_SGBRG10P:
	case VIDEO_PIX_FMT_SGRBG10P:
	case VIDEO_PIX_FMT_SRGGB10P:
		*bpp = 10;
		*data_type = CSI_DT_RAW10;
		return 0;
	case VIDEO_PIX_FMT_Y12P:
	case VIDEO_PIX_FMT_SBGGR12P:
	case VIDEO_PIX_FMT_SGBRG12P:
	case VIDEO_PIX_FMT_SGRBG12P:
	case VIDEO_PIX_FMT_SRGGB12P:
		*bpp = 12;
		*data_type = CSI_DT_RAW12;
		return 0;
	default:
		return -ENOTSUP;
	}
}
```

- [ ] **Step 2: Add the RAW12 data type constant**

Next to the existing `CSI_DT_RAW8`/`CSI_DT_RAW10` defines:

```c
#define CSI_DT_RAW12 0x2c
```

- [ ] **Step 3: Verify the IMX219 build still passes**

Run: `mise run agent-build --sysbuild data_collection`

Expected: `Build succeeded`. This proves the switch still compiles and the existing camera path is untouched.

- [ ] **Step 4: Verify formatting and commit**

```bash
cd /Users/richard.osterloh/workspace/zephyr-applications
mise x -- clang-format --dry-run --Werror \
  deps/modules/lib/rosterloh-drivers/drivers/video/video_esp32_csi.c
cd deps/modules/lib/rosterloh-drivers
git add drivers/video/video_esp32_csi.c
git commit -m "video: esp32_csi: accept greyscale and all Bayer orders"
```

Note: this file also carries uncommitted CSI diagnostic scaffolding from earlier work. `git add` will stage it too. That is intentional for this branch — the scaffolding is being kept for ToF bring-up and cleaned up later.

---

### Task 6: Arducam ToF camera shield

**Files:**
- Create: `boards/shields/arducam_tof_camera/shield.yml`
- Create: `boards/shields/arducam_tof_camera/Kconfig.shield`
- Create: `boards/shields/arducam_tof_camera/arducam_tof_camera.overlay`

**Interfaces:**
- Consumes: the `arducam,pivariety` binding from Task 1, `csi_pixfmt_info` from Task 5.
- Produces: a `--shield arducam_tof_camera` build target. Task 7 depends on it.

- [ ] **Step 1: Write shield.yml**

```yaml
shield:
  name: arducam_tof_camera
  full_name: Arducam ToF Depth Camera (Pivariety, 240x180)
  vendor: arducam
  supported_features:
    - video
```

- [ ] **Step 2: Write Kconfig.shield**

```kconfig
# Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
# SPDX-License-Identifier: Apache-2.0

config SHIELD_ARDUCAM_TOF_CAMERA
	def_bool $(shields_list_contains,arducam_tof_camera)
```

- [ ] **Step 3: Write the overlay**

`boards/shields/arducam_tof_camera/arducam_tof_camera.overlay`:

```dts
/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Arducam ToF depth camera on a Raspberry Pi 15-pin CSI connector.
 *
 * Values are taken from the Raspberry Pi arducam-pivariety-overlay.dts. Note
 * clock-noncontinuous: unlike the IMX219, this module drops its clock lane out
 * of HS between bursts.
 *
 * link-frequencies is mandatory. Pivariety modules report no link frequency of
 * their own, and the CSI-2 receiver needs it to configure its D-PHY.
 */

#include <zephyr/dt-bindings/video/video-interfaces.h>

/ {
	chosen {
		zephyr,camera = &csi_capture_port;
	};
};

&csi_interface {
	status = "okay";
};

&csi_ep_in {
	remote-endpoint-label = "arducam_tof_ep_out";
	bus-type = <VIDEO_BUS_TYPE_CSI2_DPHY>;
	data-lanes = <1 2>;
};

&csi_i2c {
	arducam_tof: arducam_pivariety@c {
		compatible = "arducam,pivariety";
		reg = <0x0c>;

		port {
			arducam_tof_ep_out: endpoint {
				remote-endpoint-label = "csi_ep_in";
				bus-type = <VIDEO_BUS_TYPE_CSI2_DPHY>;
				data-lanes = <1 2>;
				link-frequencies = <493500000>;
			};
		};
	};
};
```

- [ ] **Step 4: Verify it builds**

Run: `mise x -- west build -b esp32p4_nano/esp32p4/hpcore -p always --sysbuild --shield arducam_tof_camera --build-dir builds/data_collection applications/data_collection`

Expected: `Build succeeded`. If it fails with "shield not found", the drivers module's `board_root: .` is not picking up `boards/shields/` — check `zephyr/module.yml`.

Note: this bypasses `mise run app` because that task hardcodes the shield. Task 7 fixes that.

- [ ] **Step 5: Commit**

```bash
cd deps/modules/lib/rosterloh-drivers
git add boards/shields/arducam_tof_camera
git commit -m "boards: shields: add arducam_tof_camera"
```

---

### Task 7: Format-agnostic capture in data_collection

**Files:**
- Modify: `/Users/richard.osterloh/workspace/zephyr-applications/applications/data_collection/src/cam_mgmt.c`
- Modify: `/Users/richard.osterloh/workspace/zephyr-applications/applications/data_collection/CMakeLists.txt`
- Modify: `/Users/richard.osterloh/workspace/zephyr-applications/applications/data_collection/README.md`

**Interfaces:**
- Consumes: the shield from Task 6.
- Produces: a `data_collection` build that works with either camera. This task is in the **workspace repo**, not the drivers module.

- [ ] **Step 1: Make SHIELD overridable**

In `applications/data_collection/CMakeLists.txt`, replace the unconditional `set(SHIELD ...)` with:

```cmake
# Default to the Raspberry Pi Camera v2, but let -DSHIELD=... or
# `west build --shield` pick a different camera (e.g. arducam_tof_camera).
if(NOT DEFINED SHIELD)
  set(SHIELD raspberry_pi_camera_module_2)
endif()
```

This must appear before `find_package(Zephyr ...)`.

- [ ] **Step 2: Replace the hardcoded capture format**

In `src/cam_mgmt.c`, delete the `CAPTURE_WIDTH`, `CAPTURE_HEIGHT` and `CAPTURE_FORMAT` defines and their comment. Keep `CAPTURE_NBUFS`. Add the app's memory ceiling and a cached negotiated format:

```c
/* The app's PSRAM budget, expressed as a resolution ceiling rather than a
 * format. A sensor advertising a stepwise range -- the IMX219 goes to
 * 3280x2464 -- would otherwise have us ask for an 8 MB frame when the pool
 * cannot hold two of them.
 *
 * ponytail: assumes the ceiling lands on the sensor's width/height step (it
 * does for the IMX219's step of 4). If a sensor with a coarser step appears,
 * round the clamped value down to cap->width_min + n * cap->width_step.
 */
#define CAPTURE_MAX_WIDTH  1640
#define CAPTURE_MAX_HEIGHT 1232

/* The negotiated format, filled in by the first capture. INFO and CAPTURE
 * report it so a client sees what the camera actually agreed to rather than a
 * compile-time guess -- the two differ as soon as the shield changes.
 */
static struct video_format frame_fmt;
```

Add this helper above `cam_mgmt_capture()`:

```c
/* Choose the advertised format with the highest bit depth that fits the
 * ceiling. Depth first because this is a data-collection app: a sensor offering
 * both RAW8 and RAW10 should give us RAW10. Sizes are clamped rather than
 * rejected, so a stepwise sensor lands on the largest frame we can hold while a
 * sensor with one fixed small size (the ToF module's 240x180) is taken as-is.
 */
static int cam_pick_format(const struct device *cam, struct video_format *fmt)
{
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
	unsigned int best_bpp = 0;
	int ret;

	ret = video_get_caps(cam, &caps);
	if (ret < 0) {
		LOG_ERR("Failed to get caps (%d)", ret);
		return ret;
	}

	for (int i = 0; caps.format_caps[i].pixelformat != 0; i++) {
		const struct video_format_cap *cap = &caps.format_caps[i];
		unsigned int bpp = video_bits_per_pixel(cap->pixelformat);

		if (bpp <= best_bpp) {
			continue;
		}

		fmt->pixelformat = cap->pixelformat;
		fmt->width = MIN(cap->width_max, CAPTURE_MAX_WIDTH);
		fmt->height = MIN(cap->height_max, CAPTURE_MAX_HEIGHT);
		best_bpp = bpp;
	}

	if (best_bpp == 0) {
		LOG_ERR("Camera advertises no usable format");
		return -ENOTSUP;
	}

	fmt->type = VIDEO_BUF_TYPE_OUTPUT;

	return 0;
}
```

Then replace the top of `cam_mgmt_capture()` (from `struct video_format fmt = {...}` through the `video_set_format` call) with:

```c
int cam_mgmt_capture(const struct device *cam)
{
	struct video_format fmt;
	struct video_buffer *vbuf = NULL;
	struct video_buffer *drained;
	int ret;

	k_mutex_lock(&frame_lock, K_FOREVER);

	if (frame_vbuf != NULL) {
		video_buffer_release(frame_vbuf);
		frame_vbuf = NULL;
	}

	ret = cam_pick_format(cam, &fmt);
	if (ret < 0) {
		goto out;
	}

	ret = video_set_format(cam, &fmt);
	if (ret < 0) {
		LOG_ERR("Failed to set format (%d)", ret);
		goto out;
	}
```

The existing `video_get_format` call and everything after it stays as-is, except add after the `LOG_INF("Capturing ...")` line:

```c
	frame_fmt = fmt;
```

Add `#include <zephyr/video/formats.h>` for `video_bits_per_pixel()` if it is not already pulled in.

- [ ] **Step 3: Report the real format over SMP**

In `cam_h_info()`, replace the `CAPTURE_FORMAT`/`CAPTURE_WIDTH`/`CAPTURE_HEIGHT` uses:

```c
	     zcbor_tstr_put_lit(zse, "fmt") && zcbor_uint32_put(zse, frame_fmt.pixelformat) &&
	     zcbor_tstr_put_lit(zse, "w") && zcbor_uint32_put(zse, frame_fmt.width) &&
	     zcbor_tstr_put_lit(zse, "h") && zcbor_uint32_put(zse, frame_fmt.height) &&
```

Make the same three substitutions in `cam_h_capture()`.

- [ ] **Step 4: Update the README**

In `applications/data_collection/README.md`, replace the "Camera" bullet with:

```markdown
- **Camera** — a MIPI CSI-2 module selected at build time by shield. Defaults to
  the IMX219 (Raspberry Pi Camera v2) via Zephyr's in-tree
  `raspberry_pi_camera_module_2`; pass `--shield arducam_tof_camera` for the
  Arducam ToF depth camera. The capture path takes whatever format the camera
  advertises first, so nothing in the app is pinned to one sensor. Frames are
  drawn from PSRAM through the shared multi-heap.
```

In the "SMP camera group" section, replace the `INFO` bullet about planning values with:

```markdown
- `INFO`'s `fmt`/`w`/`h` are the format the camera negotiated on the most recent
  capture, and are zero before the first one. Still size buffers from
  `CAPTURE`'s `size`, which is the authoritative frame length.
```

- [ ] **Step 5: Verify both cameras build**

```bash
cd /Users/richard.osterloh/workspace/zephyr-applications
mise run agent-build --sysbuild data_collection
mise x -- west build -b esp32p4_nano/esp32p4/hpcore -p always --sysbuild \
  --shield arducam_tof_camera --build-dir builds/data_collection applications/data_collection
```

Expected: both print `Build succeeded`.

- [ ] **Step 6: Verify formatting and commit**

```bash
mise x -- clang-format --dry-run --Werror applications/data_collection/src/cam_mgmt.c
git add applications/data_collection/
git commit -m "data_collection: take the camera's advertised format, not a hardcoded one"
```

---

### Task 8: Hardware bring-up

**Files:** none — verification only.

**Interfaces:**
- Consumes: everything.
- Produces: the answer to the CSI question.

- [ ] **Step 1: Flash the ToF build**

```bash
cd /Users/richard.osterloh/workspace/zephyr-applications
mise run flash data_collection
```

- [ ] **Step 2: Confirm the module enumerates**

Run (opening the CH343 port resets the board, so the boot log comes for free; `main()` runs ~30 s in because `CONFIG_NET_CONFIG_INIT_TIMEOUT=30`):

```bash
mise x -- python3 /tmp/shcmd.py /dev/cu.usbmodem5B610412861 --boot 40 --wait 3 \
  'device list' 'pivariety_regs'
```

Expected: `arducam_pivariety@c (READY)`, a plausible `DEVICE_ID`/`SENSOR_ID`, and at least one enumerated cap. If `DEVICE_ID` reads `0xfffffffe` the module is not answering — check power before touching software. Arducam documents a 5 V 2 A requirement for this camera, which the CSI connector may not supply.

- [ ] **Step 3: Check what format it actually reports**

Expected from `pivariety_regs`: a 240x180 entry. The fourcc tells you the data type — `0x50303159` (`Y10P`) means RAW10, `0x59455247` (`GREY`) means RAW8. Record it; the spec's assumption that this is a greyscale module is unverified until now.

- [ ] **Step 4: Read the CSI register dump**

The boot capture drives the dump added during the IMX219 investigation. Compare against the IMX219 baseline:

```
phy_rx    idle=0x00010000 hs=0x00030000
stopstate idle=0x00000000 hs_or=0x00010003 hs_and=0x00000000
brg int_raw=0 dma trans=0
18/38 hs_freq_sel candidates produced activity, none above 0x17
```

The three outcomes and what each means:

| Observation | Conclusion |
|---|---|
| Frame captured | The receiver works. The IMX219 driver is at fault — go back to it with that knowledge. |
| Same signature: HS clock, no packets, activity only at low `hs_freq_sel` | Two independent transmitters fail identically. The fault is the board or the P4 receiver, not either sensor driver. |
| No HS clock at all (`rxclkactivehs=0`) | The module is not transmitting. Check power and the `MODE_SELECT` readback before concluding anything about the receiver. |

- [ ] **Step 5: Record the result**

Append the findings to the spec's "Risks and open questions" section, then commit:

```bash
cd deps/modules/lib/rosterloh-drivers
git add docs/superpowers/specs/2026-08-24-arducam-pivariety-driver-design.md
git commit -m "docs: record Arducam ToF CSI bring-up findings"
```
