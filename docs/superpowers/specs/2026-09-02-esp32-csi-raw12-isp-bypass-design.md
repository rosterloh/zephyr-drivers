# ESP32-P4 CSI: RAW12 capture, and why it took two fixes

Date: 2026-09-02
Status: **resolved.** The Arducam ToF camera captures 240x180 Y12P frames,
64800 bytes each, on `esp32p4_wifi6_poe_eth/esp32p4/hpcore`.

## Problem

`video_esp32_csi.c` captured RAW8 reliably (OV5647, ~35 fps) but RAW10 and
RAW12 had never produced a frame. That blocked the Arducam ToF camera
entirely: it advertises exactly one format, **240x180, pitch 360, `Y12P`**,
i.e. RAW12.

The failure gave nothing to work with. The sensor drove the D-PHY, every host
error latch read zero, and the bridge received zero packets:

```
phy: rx=0x00030000 (rxclkactivehs=1)  stopstate cycling on data and clock lanes
host: int_main=0 phy_fatal=0 pkt_fatal=0 bndry=0 seq=0 crc=0 pld_crc=0
isp:  int_raw=0x00000000     <- not even a frame header
brg:  int_raw=0x00000000
```

## Root cause: two independent bugs

Neither fix alone changes anything observable. That is why this took so long,
and why several correct hypotheses were discarded after being "disproven".

### 1. The link frequency was a guess, and it was wrong by 3.3x

`boards/shields/arducam_tof_camera` carried `link-frequencies = <493500000>` -
987 Mbps/lane - copied from the IMX219 shield rather than from anything the
module reports. The real rate is **300 Mbps/lane**.

`mipi_csi_hal_init()` turns the lane bit rate into the only D-PHY register it
writes (`0x44 = hs_freq_sel << 1`, looked up in `soc_mipi_csi_phy_pll_ranges`).
987 Mbps selects the 950-1000 Mbps band for a transmitter running at 300, so
the PHY never locked. A PHY that cannot lock produces no packets and no
errors, which is indistinguishable from every other failure in this stack.

The correct value is not a better guess - it is derived. The module reports
`VIDEO_CID_PIXEL_RATE = 50000000`, and both Zephyr's
`video_get_csi_link_freq()` and the Raspberry Pi kernel's
`v4l2_get_link_freq_ctrl()` use the same expression:

```
link_freq = pixel_rate * bpp / (2 * lanes) = 50000000 * 12 / 4 = 150000000
```

The driver had been enumerating that control and discarding it as "unmapped".
It now registers it, `link-frequencies` became an optional override, and the
shield sets nothing.

### 2. Bypass means the ISP is *disabled*, not configured differently

`csi_isp_start()` ended with an unconditional `isp_ll_enable(&ISP, true)`.
ESP-IDF never does that on the bypass path - `esp_isp_enable()` refuses
outright:

```c
ESP_RETURN_ON_FALSE(proc->bypass_isp == false, ESP_ERR_INVALID_STATE, TAG,
                    "processor is configured to be bypassed");
```

In bypass, `cntl.isp_en` stays 0 and `cntl.mipi_data_en` alone - set by
`isp_ll_set_input_data_source()`, independent of `isp_en` - holds the
CSI -> bridge path open. An *enabled* ISP configured for bypass reads
`frame_cfg.hadr_num` (32-bit words) as a pixel count.

The two bypass settings are not independent. That is what made an earlier
attempt look like "bypass breaks RAW8 too": applying the word-based Hsize
while leaving the ISP enabled is broken at any depth, and was read as evidence
against bypass rather than against half-applying it.

### A third bug found on the way

`csi_hw_start()` did `return (int)link_freq` when `link_freq <= 0`. Zero is a
failure - the source advertised nothing usable - but it was returned as
success, `mipi_csi_hal_init()` never ran, and the first bridge access faulted
on a NULL `hal.bridge_dev`. Found because fix 1 briefly produced a zero pixel
rate; it would have bitten any sensor that failed to advertise a rate.

## How it was actually settled

By testing the module on a Raspberry Pi 5 with the reference
`arducam-pivariety` driver, which captured 10 frames immediately:

```
arducam-pivariety 10-000c: firmware version: 0x10004
sensor subdev pad0: fmt:Y12_1X12/240x180
rp1-cfe: Using a link rate of 300 Mbps
/tmp/tof.raw = 662400 bytes (10 frames)
```

Three things came from that single test: the module was healthy, our format
handling matched the reference exactly, and - decisively - the kernel printed
the link rate it derived. Nothing on the ESP32-P4 side could have produced
that number, because the wrong one was hard-coded in our own devicetree.

**Lesson worth keeping: a guessed constant that silently mis-programs hardware
is worse than a missing one.** A missing value fails loudly at probe; a wrong
one fails as inexplicable silence three layers downstream, and invalidates
every experiment run against it.

## On the earlier elimination work

An earlier revision of this document carried two large tables of "eliminated"
candidates - ISP configurations, line-sync short packets, the bridge data-type
filter, the D-PHY rate band swept across all 38 PLL ranges, control ordering.

**Those results were void and have been removed.** Every one of them was
measured with the D-PHY programmed for 987 Mbps against a 300 Mbps
transmitter, so no configuration downstream could have worked. The 38-band
sweep visited the correct band but ran with temporary scaffolding that had its
own ISP/bridge mismatch, so it too proved nothing.

A negative hardware result only refutes a hypothesis if every other variable
is known-good. Here one variable was silently wrong for the whole
investigation, and the honest conclusion is that the elimination work
established nothing except the two facts that survived independent
confirmation: the receiver works (IMX219 produced ISP frame headers on the
same code path), and the 2.5 V D-PHY supply is correctly programmed (PMU
readback).

## Verification

- Arducam ToF on `esp32p4_wifi6_poe_eth/esp32p4/hpcore`: `Frame 1 captured:
  64800 bytes`, repeated across four reflashes with live-varying pixel data.
- `240 * 180 * 12 / 8 = 64800`, matching the negotiated format exactly.
- Capture completes in ~67 ms from `set_stream` to dequeue.
- `pivariety_regs` reports `link frequency = derived from pixel rate 50000000
  Hz`, i.e. no devicetree override in play.
- Builds clean for both boards, with and without the shield, plain and
  `--sysbuild`. `clang-format --Werror` clean.

Regression check, run with an IMX219 on the same board:

- **RAW10 now captures**, and never had before: 1640x1232, pitch 2050,
  2525600 bytes, all 1232 lines. The same `isp_en` fix covers RAW10 and RAW12
  alike, so the ToF work fixed the Raspberry Pi Camera v2 path as a side
  effect.

  **Single frames only.** An earlier revision of this document claimed "37 fps
  continuous"; that was wrong. The figure comes from the shell's per-buffer
  rate calculation on the *first* buffer, and only one buffer ever arrives -
  see the open issue below. The application path (`cam_mgmt`, which does a full
  start/stop per frame) works; continuous streaming does not.
- **RAW8 on the IMX219 does not capture** (`BA81` at either 1640x1232 or
  640x480: the capture starts and no buffer ever completes). This is **not a
  regression** - it was reproduced against the pre-fix driver, checked out at
  `530533b~1`, where it fails identically. It also cannot be caused by that
  commit: for 8 bpp, `isp_ll_enable(&ISP, bpp == 8)` evaluates to `true`,
  which is bit-identical to the unconditional call it replaced.

  RAW8 itself is fine - the ISP commit verified an OV5647 streaming RAW8 at
  ~35 fps - so this is specific to the IMX219's RAW8 mode and belongs to the
  forked IMX219 driver, not the CSI receiver. Tracked separately; the
  application never selects it, because `cam_pick_format()` takes the highest
  depth the sensor advertises.

## Open issues

### Continuous streaming stops after the first frame

`video capture <dev> N` delivers buffer 1 and nothing further, at any depth or
resolution tried. `cam_mgmt`'s start/stop-per-frame path is unaffected, which
is why the application works.

Eliminated, each with instrumentation on hardware:

| Candidate | How |
|---|---|
| Buffer starvation | The ISR's `fifo_in` empty branch (`active_vbuf = NULL`) was counted and logged. **Never taken.** Note it is still a latent dead-end - `video_esp32_csi_enqueue()` only does `k_fifo_put()` and cannot re-arm a stopped DMA - but it is not what happens here. |
| Line length not a multiple of the 8-byte DMA width | RAW10 at 1640 px gives pitch 2050 (256.25 words). Retested at 1600 px, pitch 2000 (250 words, exact): stalls identically. |
| DW-GDMA interrupt storm | ISR entries counted; a guard disabled interrupt generation at 2000 entries and logged. **Never triggered**, so the ISR is not being re-entered. |

The significant observation: a `k_work_delayable` scheduled at 1 Hz for the
duration of the stream **produces no output at all** after the first frame.
Nothing is running - not the ISR, not the system workqueue, not the log
processor. That is not a stalled DMA, it is the whole system going quiet, and
it points at memory corruption rather than a flow-control problem.

That reading is supported by the RAW8 result below, where capture *does*
continue and then faults in `sys_heap_free()` from `video_buffer_release()`.
Both are consistent with the DMA writing outside the buffer it was given.
`csi_cache_len()`'s comment already describes this class of bug; the arithmetic
does not obviously fit (2020480 is an exact multiple of the 64-byte line), so
the overrun is something else. Next step is to bound the DMA destination and
check what the bridge actually writes per line versus `fmt.pitch`.

### RAW8 works through bypass, but exposes the corruption

Unifying `csi_isp_start()` on bypass for every depth - deleting the `bpp == 8`
branch entirely - makes IMX219 RAW8 capture, which it never did:

```
Buffer 1/5 .. 5/5, Bytes 0-2020480/2020480, Lines 0-1232/1232, 40 FPS average
Capture of 5 buffers in 122 ms in total
```

RAW8 only kept the ISP processing path because bypass "never produced a frame",
which was the `isp_en` bug. **Not committed**: the run ends in the heap fault
above, and a fix that corrupts the heap is worse than the bug it replaces.
Land it once the overrun is understood; it removes a branch rather than adding
one.

## Related

- `docs/superpowers/specs/2026-08-24-arducam-pivariety-driver-design.md` - the
  Pivariety driver and its bring-up record. Its "Outcome (2026-08-25)" section
  concludes "the fault is the board or the silicon, not driver software". That
  is now fully retired: the fault was three software bugs, two of ours and one
  of omission in the shield.
