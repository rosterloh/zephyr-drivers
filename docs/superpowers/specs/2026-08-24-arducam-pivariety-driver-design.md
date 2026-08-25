# Arducam Pivariety camera driver — design

**Date:** 2026-08-24
**Status:** Approved (design)

## Problem

The workspace has one MIPI CSI-2 camera driver (the forked `imx219`) and one
CSI-2 receiver driver (`video_esp32_csi`, ESP32-P4). Receiver bring-up on
`esp32p4_nano` is stuck: the D-PHY link is up and error-free at the line level,
but the host decodes no packets, and there is no way to tell whether the fault
is in the IMX219 driver, the module, or the P4 receiver.

The Arducam ToF camera is an independent MIPI transmitter — its own PLL, its own
MIPI stack, a different data type and non-continuous clock — reachable through a
much smaller driver than a bare sensor would need. It serves two goals at once:

1. A reusable Zephyr driver for Arducam's Pivariety camera family.
2. A discriminator for the CSI receiver investigation. If the ToF camera fails
   the same way the IMX219 does, the fault is the board or the P4 receiver, not
   the sensor driver.

## Background: what Pivariety is

Arducam Pivariety modules are not bare sensors. An on-module MCU exposes a
small, **self-describing** register interface, so one driver serves every module
in the family. Upstream reference: `drivers/media/i2c/arducam-pivariety.{c,h}`
in the Raspberry Pi kernel.

- I2C address `0x0c`.
- Register access is **16-bit big-endian address, 32-bit big-endian data**,
  transferred whole: a 6-byte write, or a 2-byte address write followed by a
  4-byte read.

  **This is not CCI, and Zephyr's CCI helpers cannot express it.** An early
  draft of this spec assumed `VIDEO_REG_ADDR16_DATA32_BE` +
  `video_read_cci_reg()` would work. It does not: that flag means a 32-bit
  value occupying *four consecutive byte-addressed registers*, and
  `video_read_cci_reg()` (`video_common.c:308`) loops one byte at a time
  incrementing the address, so reading `SYSTEM_IDLE_REG` would issue four
  transactions against 0x0107..0x010a. `video_read_reg_retry()` does take a
  length, but it is `static`. There is no burst variant in this tree.

  The driver therefore frames transfers itself with `i2c_write_read_dt()` /
  `i2c_write_dt()`, mirroring `pivariety_{read,write}_reg()` in the kernel
  driver. Register constants are plain `uint16_t` addresses with no flag.
- Capabilities are enumerated at runtime by writing an *index* register and
  reading back an *attribute* register, terminated by the sentinel
  `NO_DATA_AVAILABLE` (`0xFFFFFFFE`).
- Operations are separated by polling `SYSTEM_IDLE_REG`.

Register map (from the kernel header):

| Register | Address | Purpose |
|---|---|---|
| `MODE_SELECT_REG` | `0x0100` | 0 = standby, 1 = streaming |
| `DEVICE_VERSION_REG` | `0x0101` | probe |
| `SENSOR_ID_REG` | `0x0102` | probe |
| `DEVICE_ID_REG` | `0x0103` | probe |
| `SYSTEM_IDLE_REG` | `0x0107` | busy/idle handshake |
| `PIXFORMAT_INDEX_REG` | `0x0200` | format selector |
| `PIXFORMAT_TYPE_REG` | `0x0201` | MIPI CSI-2 data type (`0x2a/0x2b/0x2c`) |
| `PIXFORMAT_ORDER_REG` | `0x0202` | Bayer order, or `4` = greyscale |
| `MIPI_LANES_REG` | `0x0203` | lane count for this format |
| `RESOLUTION_INDEX_REG` | `0x0300` | resolution selector |
| `FORMAT_WIDTH_REG` | `0x0301` | width |
| `FORMAT_HEIGHT_REG` | `0x0302` | height |
| `CTRL_INDEX_REG` … `CTRL_VALUE_REG` | `0x0400`–`0x0406` | control enumeration |

The ToF camera specifically (from the RPi overlay): 240x180, 2 data lanes,
**`clock-noncontinuous`**, `link-frequencies = 493500000` (987 Mbps/lane).
Depth decoding happens entirely in Arducam's userspace SDK — this driver
delivers raw frames and nothing more.

## Scope

In scope:

- `dts/bindings/video/arducam,pivariety.yaml`
- `drivers/video/arducam_pivariety.c` (+ Kconfig, CMakeLists)
- `boards/shields/arducam_tof_camera/`
- Extending `csi_pixfmt_info()` in `video_esp32_csi.c`
- Making `data_collection`'s capture path format-agnostic (in the
  `zephyr-applications` repo, not this one)

Out of scope:

- Depth decoding, point clouds, any ToF maths. Raw frames only.
- Frame-interval control. The Pivariety register map has no frame-rate
  registers; add it only if a module turns out to expose it as a control.
- Fixing the CSI receiver. This driver *informs* that investigation; it is not
  expected to fix it.

## Architecture

### Format capability array

Zephyr's `video_get_caps()` returns a pointer to a zero-terminated
`struct video_format_cap[]`, and every in-tree sensor declares that array
`static const` at compile time. Pivariety only knows its list at runtime.

**Decision: fixed-size array in driver data, capped by Kconfig.** Enumerate at
init into a non-const `struct video_format_cap[CONFIG_VIDEO_ARDUCAM_PIVARIETY_MAX_FORMATS + 1]`,
zero-terminate it, and hand out a pointer. No heap dependency, bounded RAM, and
callers see exactly the same contract as any other Zephyr video driver. If a
module reports more entries than the cap, log a warning and truncate rather than
failing — a partially usable camera beats a dead one.

Rejected: heap allocation mirroring the kernel driver's `devm_kzalloc` (pulls a
heap dependency into a driver that otherwise needs none, for an allocation that
lives forever anyway); and hardcoding the ToF format (defeats the point of a
generic driver).

Note that one `video_format_cap` entry is needed per *(format, resolution)*
pair, not per format, since each entry carries its own min/max width and height.
The Kconfig default should account for that; 16 is a sane starting point.

### Pixel format mapping

`PIXFORMAT_TYPE_REG` returns the MIPI CSI-2 data type directly, so the fourcc
follows from the (type, order) pair:

| Type | `ORDER = 4` (grey) | `ORDER = 0..3` (Bayer) |
|---|---|---|
| `0x2a` RAW8 | `VIDEO_PIX_FMT_GREY` | `VIDEO_PIX_FMT_S{BGGR,GBRG,GRBG,RGGB}8` |
| `0x2b` RAW10 | `VIDEO_PIX_FMT_Y10P` | `…10P` |
| `0x2c` RAW12 | `VIDEO_PIX_FMT_Y12P` | `…12P` |

Anything else enumerates as unsupported and is skipped with a `LOG_DBG`, so a
module advertising YUV or RGB formats does not break probe.

### Link frequency

Pivariety has **no link-frequency register**. The RPi overlay hardcodes it in
devicetree, and this driver does the same: read `link-frequencies` from the
endpoint node and expose it as a read-only `VIDEO_CID_LINK_FREQ` menu control.
`video_esp32_csi` calls `video_get_csi_link_freq()` to compute the D-PHY lane
rate, so without this the receiver cannot configure `hs_freq_sel`.

### Controls

Enumerate `CTRL_INDEX_REG` → `CTRL_ID_REG`/`MIN`/`MAX`/`STEP`/`DEF`, and
register each control whose V4L2 CID has a Zephyr equivalent. Controls with no
Zephyr counterpart (including Arducam's vendor `V4L2_CID_ARDUCAM_*` range) are
skipped with a `LOG_DBG` rather than failing probe. `set_ctrl` writes
`CTRL_ID_REG` then `CTRL_VALUE_REG`.

This is in the first pass. It is the same index/attribute pattern as format
enumeration — roughly 40 lines — and it is what makes the driver generic rather
than ToF-specific.

### Driver flow

```
init
 ├─ i2c bus ready?
 ├─ piv_wait_idle()
 ├─ read DEVICE_ID / SENSOR_ID / DEVICE_VERSION      → log, fail if unreadable
 ├─ enumerate formats × resolutions → caps array     → fail if zero usable
 ├─ enumerate controls → video_init_ctrl()
 ├─ init VIDEO_CID_LINK_FREQ from DT
 └─ set default format (index 0,0)

set_fmt   → match caps → write PIXFORMAT_INDEX_REG, RESOLUTION_INDEX_REG
get_fmt   → cached current format
get_caps  → pointer to the enumerated array
set_stream→ write MODE_SELECT_REG 0/1
```

Every register operation is preceded by `piv_wait_idle()`, which polls
`SYSTEM_IDLE_REG` with a **timeout** — unlike the kernel driver, which spins
untimed. A wedged module must produce `-ETIMEDOUT`, not hang the caller.

### Shield

`boards/shields/arducam_tof_camera/` follows the pattern already established by
`adafruit_neokey_1x4` etc. in this repo (`shield.yml`, `Kconfig.shield`,
`<name>.overlay`), and mirrors Zephyr's in-tree
`raspberry_pi_camera_module_2` shield for content: set `zephyr,camera` to the
CSI capture port, enable `csi_interface`, declare the endpoint with
`data-lanes = <1 2>` and `link-frequencies = 493500000`, and add
`arducam_pivariety@0c` on `csi_i2c`.

Only one camera shield can be active at a time — both set `zephyr,camera` — so
switching cameras is a build-time `--shield` choice. `data_collection`'s
`CMakeLists.txt` currently pins `SHIELD` unconditionally and must become
overridable.

### Application changes (`zephyr-applications`)

`cam_mgmt.c` hardcodes `CAPTURE_WIDTH`/`CAPTURE_HEIGHT`/`CAPTURE_FORMAT`
(1640x1232 `SBGGR10P`). Replace with: query `video_get_caps()`, take the first
advertised format, `video_set_format()` it, then `video_get_format()` for the
real geometry — which the code already does for `pitch`. The SMP `CAPTURE`
reply already returns the true `size`; `INFO` and `CAPTURE` should report the
negotiated `w`/`h`/`fmt` rather than the compile-time constants, which removes
the caveat currently documented in the app README.

## Testing

Build-level:

- Builds for `esp32p4_nano/esp32p4/hpcore` with `--shield arducam_tof_camera`.
- The existing IMX219 build still succeeds unchanged.
- `clang-format --dry-run --Werror` clean.

On hardware, in order:

1. `device list` — `arducam_pivariety@0c` reports `READY`.
2. A `pivariety_regs` shell command dumps device/sensor ID, version, and the
   enumerated format/resolution table. This is the analogue of `imx219_regs` and
   is the fastest way to see whether the I2C protocol works at all.
3. `video format <csi-dev> out` — shows the formats the module advertised.
4. Boot capture drives the existing CSI register dump.

Success criteria, stated honestly:

- **Driver:** the module enumerates, a format can be set, and `MODE_SELECT`
  takes effect — observable as the D-PHY line state changing in the CSI dump.
- **Validation:** the dump answers whether an independent transmitter behaves
  like the IMX219. A frame is the hoped-for outcome, not the success criterion.

## Risks and open questions

- **987 Mbps/lane is above the ~650 Mbps ceiling** where the receiver currently
  shows any life. "No frame" is a plausible outcome. It is still a useful
  result — it would point at the board or the P4 receiver rather than the
  IMX219 driver — but it is not first light.
- **Non-continuous clock mode.** The IMX219 runs a continuous clock; this module
  does not. The receiver may need `csi_enableclk` handled differently. Treated
  as something to *observe* in the existing dump first, not to pre-emptively
  code for.
- **The ToF camera's actual advertised format is unverified.** It is expected to
  report greyscale RAW8 or RAW10 at 240x180, but that comes from reasoning about
  the hardware, not from a register read. The `pivariety_regs` command exists
  precisely to check this early; the format mapping table covers all the
  plausible answers.
- **Power.** Arducam's documentation notes the ToF camera needs a 5 V 2 A
  supply. Whether the P4-Nano's CSI connector can source that is unconfirmed and
  should be checked before blaming software for a dead module.

## Outcome (2026-08-25)

### The driver works

Against a real Arducam ToF module the driver probes, enumerates and negotiates
a format: `device 0x0030 sensor 0x2311 version 0x10004`, one (format,
resolution) pair at **240x180, pitch 360 — `Y12P`, 12 bits per pixel**.

That last part corrected a design assumption. The spec expected greyscale RAW8
or RAW10; the module reports RAW12. The `csi_pixfmt_info()` extension covering
`Y12P` → `CSI_DT_RAW12` is what made it expressible at all.

One bug was found only by hardware: **`SYSTEM_IDLE_REG` reads 0 when the module
is free**, not non-zero. The register name reads like a flag but the polarity is
a busy count, matching `wait_for_free()` in the kernel driver, which breaks on
`!value`. All 11 unit tests passed against the wrong polarity because the
emulator was written from the same misunderstanding as the driver — it returned
1 for that register. An emulator authored alongside the code it tests can prove
internal consistency but never external correctness.

### The receiver does not, and it is not our fault

The diagnostic purpose of this work is answered. Capture fails, with this
signature:

```
phy_rx=0x00030000   stopstate cycling   n_lanes=1 (2 lanes)
resetn=1 shutdownz=1 rstz=1
phy_fatal=0 pkt_fatal=0 phy=0 ecc=0 crc=0
brg int_raw=0   csi_en=1   frame_cfg correct for the negotiated geometry
0 frames
```

HS clock detected, data lanes demonstrably bursting, bridge enabled and
correctly configured, **zero packets decoded, zero errors of any kind**.

That signature is now reproduced across:

| Camera | Rate | Driver stack | Result |
|---|---|---|---|
| Sony IMX219 | 912 Mbps/lane | this repo (Zephyr) | 0 frames |
| Arducam ToF (Pivariety) | 987 Mbps/lane | this repo (Zephyr) | 0 frames |
| OmniVision OV5647 | **200 Mbps/lane** | **ESP-IDF v5.5** | 0 frames |
| OmniVision OV5647 | 200 Mbps/lane | **ESP-IDF v5.4** | 0 frames |
| **A second OV5647** on a **second, different board** | 200 Mbps/lane | ESP-IDF v5.4 | 0 frames |

The third row is the decisive one. Espressif's own `esp_cam_ctlr_csi`, their own
`esp_cam_sensor` OV5647 driver, their own reference configuration, at roughly a
fifth of the bit rate — and the register dump is bit-for-bit identical to ours.

**The fault is the board or the silicon, not driver software.** Two hypotheses
that looked strong are refuted by this:

- *Rate dependence.* An `hs_freq_sel` sweep had found receiver activity only at
  candidates below ~650 Mbps and silence above. 200 Mbps fails identically, so
  the ceiling was a red herring.
- *Clock continuity.* The ToF module runs non-continuous clock where the IMX219
  runs continuous. Same failure.

### This is a rev 1.3 board, and that increasingly matters

Two independent signals from ESP-IDF v5.5 point at the silicon revision:

1. It **defaults to requiring P4 revision >= 3.1** (`ESP32P4_REV_MIN_301`) and
   refuses to flash a v1.3 chip without explicitly setting
   `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`. That option's help text reads:
   *"Revisions higher than 3.0 (included) and revisions less than 3.0 have huge
   hardware difference."*
2. `esp_cam_new_csi_ctlr()` **rejects the reference example's own configuration**
   on this chip — RAW8 -> RGB565 fails with "failed to configure format
   conversion", because `mipi_csi_brg_ll_enable_color_conversion()` and friends
   are no-op stubs below `HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300`. Espressif's
   current CSI example cannot run as written on pre-v3 silicon.

The v5.4 row rules out the last software explanation. Revision gating only
entered `mipi_csi_brg_ll.h` in v5.5; v5.3 and v5.4 contain no
`CHIP_SUPPORT_MIN_REV` there at all, because rev 3.x did not exist when they
were written. v5.4's CSI driver therefore targets *this* silicon exclusively --
and it fails identically, needing no revision override to build or flash.

Also worth recording: the [ESP32-P4 errata](https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32p4/)
documents **no** MIPI CSI, CSI bridge, camera, DW-GDMA or D-PHY issue, for any
revision. This is not a published silicon defect.

**Conclusion: the fault is this board.** Three cameras, three independent driver
stacks, bit rates from 200 to 987 Mbps, two ESP-IDF releases either side of the
silicon revision split -- all produce a link that comes up and carries nothing.

The final row eliminates "this particular board is faulty". It was run on a
second, physically different ESP32-P4 (MAC `e8:f6:0a:e0:ce:cc` vs
`80:f1:b2:d2:cf:1d`) on a different product -- an ESP32-P4-ETH-POE rather than
the P4-Nano -- and produced a bit-identical dump. The sensor was detected and
configured on that board (`Detected Camera sensor PID=0x5647`), so the run is
valid rather than a silent no-op: GPIO7/8 is the correct SCCB pinout on both.

Both chips are **revision v1.3**.

**Conclusion: rev 1.3 silicon.** Every other candidate has been eliminated by
measurement:

| Hypothesis | Eliminated by |
|---|---|
| This Zephyr driver | ESP-IDF v5.5 and v5.4 fail identically |
| IMX219 driver, module or PLL | ToF and OV5647 fail identically |
| Cable | Swapped; different camera and cable on the second board |
| Link rate | 200 Mbps fails exactly as 987 Mbps does |
| Clock continuity | ToF non-continuous, IMX219 continuous, same failure |
| IDF's pre-v3 path being unmaintained | v5.4 targets v1.x exclusively, same failure |
| One faulty board | Second board, different design, same failure |
| One faulty camera module | Four distinct modules, all fail |

That final run used a **different OV5647 module** as well as a different board,
so it carries no residual "maybe the camera" caveat. Four physically distinct
camera modules have now been tried -- IMX219, Arducam ToF, and two separate
OV5647 boards -- across two P4 boards and three driver stacks. Every one
produces the same dump.

The remaining unknown is whether rev 3.x silicon works. Nobody has tested it
here. Given rev 1.0/1.3 are officially "not recommended for new designs", pin 54
becomes a new power rail on v3.0+, and 29 peripherals carry revision-gated HAL
code, obtaining a rev 3.x board is the obvious next step for anyone continuing
this.
Both remain consistent with every observation. Separating them needs a second
P4 board, ideally rev 3.x, which would also allow running the reference example
unmodified.

### Reproducing

A capture-only ESP-IDF test app lives outside this repo at `~/esp/csi_test`
(~150 lines). It is `examples/peripherals/camera/mipi_isp_dsi` with the DSI
display half removed, and it dumps the same CSI host and bridge registers this
driver does so the two stacks can be compared directly. It needs
`CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`, `CONFIG_ESP32P4_REV_MIN_100=y` and
`CONFIG_CAMERA_OV5647=y`, and must use RAW8 passthrough rather than the
example's RAW8 -> RGB565.
