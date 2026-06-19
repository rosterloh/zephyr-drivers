# Waveshare ESP32-P4-Nano board support — design

**Date:** 2026-06-19
**Status:** Approved (design)

## Problem

Add an out-of-tree Zephyr board definition for the
[Waveshare ESP32-P4-Nano](https://www.waveshare.com/wiki/ESP32-P4-Nano-StartPage)
to this drivers module, so applications in the wider workspace can target it.

## Prerequisite (resolved)

ESP32-P4 SoC support did not exist in the workspace's `deps/zephyr` checkout
(stale upstream-main commit). A `west update` advanced `deps/zephyr` to a
revision that includes `soc/espressif/esp32p4`, the `esp32p4_*` devicetree
includes, and the in-tree `esp32p4_function_ev_board`. That board is near-
identical hardware and is the template for this one.

## Scope

**In:** a bring-up board with Ethernet — boots, logs over the USB-UART console,
and exposes the common peripherals. HP core (primary) + LP core targets.

**Out (documented as unpopulated in Zephyr):** WiFi/BT via the on-board
ESP32-C6, MIPI DSI/CSI, audio codec + mic/speaker, USB-OTG, GPIO-header SPI,
RTC battery.

### Why WiFi is out

The board's WiFi/BT radio is an on-board **ESP32-C6** reached from the P4 over
**SDIO** (confirmed from schematic: Reset GPIO54, CMD 19, CLK 18, D0–D3 14–17).
Zephyr's `esp_hosted` WiFi driver is **SPI-only** (`compatible
"espressif,esp-hosted"` includes `spi-device.yaml` and requires `reset-gpios`,
`dataready-gpios`, `handshake-gpios`). There is no SDIO esp-hosted transport in
Zephyr. Therefore:

- WiFi cannot be brought up by a board definition alone; it needs a new
  SDIO esp-hosted driver (upstream-grade effort) plus esp-hosted **slave**
  firmware flashed to the C6.
- A literal `espressif,esp-hosted` node — even `status = "disabled"` — would be
  **invalid and misleading** here (it would need an SPI parent and handshake
  lines that do not exist for the SDIO link). So the C6 wiring is recorded as a
  **commented devicetree block plus a `doc/index.rst` section**, not a node.

## Approach

Mirror the structure of upstream `boards/espressif/esp32p4_function_ev_board`
(HP/LP core split, same SoC family, same SDHC/RMII/LDO patterns) and adjust
identity and pins for the Nano. The alternative — authoring from scratch — is
strictly more work for an identical result, so it is rejected.

## Board layout

```
boards/waveshare/esp32p4_nano/
  board.yml
  board.cmake
  Kconfig.esp32p4_nano
  Kconfig.defconfig
  Kconfig.sysbuild
  esp32p4_nano-pinctrl.dtsi
  esp32p4_nano_hpcore.dts
  esp32p4_nano_hpcore.yaml
  esp32p4_nano_hpcore_defconfig
  esp32p4_nano_lpcore.dts
  esp32p4_nano_lpcore.yaml
  esp32p4_nano_lpcore_defconfig
  doc/index.rst
```

- `board.yml`: `name: esp32p4_nano`, `vendor: waveshare`, `socs: [{name: esp32p4}]`.
- Board identifiers: `esp32p4_nano/esp32p4/hpcore` and `esp32p4_nano/esp32p4/lpcore`.

## SoC base and memory

HP core dts includes `<espressif/esp32p4/esp32p4_n16r8.dtsi>` (16 MB flash,
8 MB PSRAM) and `<espressif/partitions_0x2000_default_16M.dtsi>`; LP core dts
includes the matching `esp32p4_lpcore*` include, following the function-EV
board.

The Nano advertises 32 MB PSRAM, but only the `n16r8` (8 MB) variant include
exists upstream. Using `n16r8` is safe for bring-up (under-reporting PSRAM is
harmless). Mark with:
`// ponytail: n16r8 (8 MB PSRAM) is the largest upstream P4 variant; bump if an n16r32 dtsi lands.`

## Peripherals and pins

Pins sourced from the ESP32-P4-Nano schematic
(`files.waveshare.com/wiki/ESP32-P4-NANO/ESP32-P4-NANO-schematic.pdf`).
**SD, BOOT, Ethernet, and the C6 SDIO link were read directly off the schematic
and are confirmed.** UART0 (GPIO37/38) and I2C0 (SCL 8 / SDA 9) follow the P4
SoC defaults and the schematic interface block; implementation must cross-check
them against the schematic before relying on them.

| Function | Detail |
|---|---|
| Console | `uart0` via on-board USB-UART bridge — TX **GPIO37**, RX **GPIO38**, 115200 8N1. `chosen { zephyr,console / zephyr,shell-uart = &uart0; }` |
| Ethernet | `&eth` + `&mdio` with IP101 `ethernet-phy@1`. RMII pinctrl: MDC **GPIO31**, MDIO **GPIO52**, ref-clk **GPIO50**, PHY power-enable **GPIO51**. `NET_L2_ETHERNET` defaulted on for hpcore. |
| MicroSD | `&sdhc` → `sdhc0`, 4-bit, `zephyr,sdmmc-disk`. CLK **43**, CMD **44**, D0 **39**, D1 **40**, D2 **41**, D3 **42**. VDD is power-gated by an AO3401 PMOS driven by **GPIO45** (assert to power the card). Alias `sdhc0`. |
| BOOT button | **GPIO35**, `GPIO_PULL_UP | GPIO_ACTIVE_LOW`, `gpio-keys` → alias `sw0`, code `INPUT_KEY_0`. |
| I2C0 | SCL **GPIO8**, SDA **GPIO9**, `I2C_BITRATE_FAST`. |
| Regulators | `&ldo` rails as on the function-EV board (3V3 / 1V8 boot-on, always-on). |
| Misc | `&wdt0` (alias `watchdog0`), `&trng0`, `&coretemp`, `&dma`, `&dma_axi`, `&gpio0` enabled. |

No user-controllable LED — the on-board LED is a 5 V power indicator — so no
`led0` alias.

### Chosen / aliases (hpcore)

- `chosen`: `zephyr,sram = &sramhp`, `zephyr,console = &uart0`,
  `zephyr,shell-uart = &uart0`, `zephyr,flash = &flash0`,
  `zephyr,code-partition = &slot0_partition`.
- `aliases`: `sw0 = &boot_button`, `watchdog0 = &wdt0`, `sdhc0 = &sdhc0`.

### Commented C6 (WiFi) block

Include a clearly commented block in the hpcore dts recording the SDIO link
(Reset 54, CMD 19, CLK 18, D0–D3 14–17), with a one-line pointer to the
`doc/index.rst` WiFi section explaining the SDIO-esp-hosted gap.

## Kconfig

- `Kconfig.esp32p4_nano`: `config BOARD_ESP32P4_NANO` selecting `SOC_ESP32P4`
  and the hpcore/lpcore cluster symbols, following the function-EV board.
- `Kconfig.defconfig`: default `NET_L2_ETHERNET` on for the hpcore target.
- `hpcore_defconfig`: `CONSOLE`, `SERIAL`, `UART_CONSOLE`, `GPIO`, `REGULATOR`.
- `Kconfig.sysbuild`, `board.cmake`: mirror the function-EV board.

## Testing / verification

- `twister` builds both targets for this board:
  `uv run west twister -p esp32p4_nano/esp32p4/hpcore -p esp32p4_nano/esp32p4/lpcore -T <a trivial app>`
  Use `samples/hello_world` (hpcore) as the build smoke test; the board has no
  custom C to unit-test, so the gate is "the board configures and links."
- Ethernet, SD, I2C presence is verified by the board building with those nodes
  enabled (devicetree + driver compile). Functional bring-up on real hardware
  (flash, observe console, ping over Ethernet, mount SD) is a manual step the
  maintainer performs; CI cannot run on the SoC.
- `doc/index.rst` documents supported features, the WiFi gap, and flashing.

## Out of scope (recorded for future work)

WiFi/BT (needs SDIO esp-hosted driver + C6 firmware), MIPI DSI/CSI, audio codec
+ mic/speaker, USB-OTG device/host, GPIO-header SPI bus, RTC battery backup.
