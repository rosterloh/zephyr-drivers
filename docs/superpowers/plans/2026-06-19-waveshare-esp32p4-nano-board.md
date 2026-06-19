# Waveshare ESP32-P4-Nano Board Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an out-of-tree Zephyr board definition for the Waveshare ESP32-P4-Nano (HP + LP core targets) to this drivers module so workspace apps can target it.

**Architecture:** The Nano is hardware-near-identical to upstream's `esp32p4_function_ev_board`. We mirror that board's structure (HP/LP core split, same SoC family, same SDHC/RMII/LDO patterns) under `boards/waveshare/esp32p4_nano/`, adjusting identity, the I2C pinmux (Nano-specific), and documentation. No C source — the deliverable is devicetree + Kconfig + YAML + docs.

**Tech Stack:** Zephyr, devicetree, Espressif ESP32-P4 SoC (RISC-V), twister, the Zephyr SDK RISC-V toolchain + Espressif HAL blobs.

## Global Constraints

- All new files carry the repo header. C-style for `.dts`/`.dtsi`:
  `/* Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com> */` then
  `/* SPDX-License-Identifier: Apache-2.0 */`. Hash-style for `.yml`/`Kconfig*`/`board.cmake`/`_defconfig`:
  `# Copyright (c) 2026 Richard Osterloh` then `# SPDX-License-Identifier: Apache-2.0`.
- All `west`/`twister` commands run through `uv run` from the workspace root
  `/home/rio/src/github/rosterloh/zephyr-applications` (never bare `west`, never `cd deps/zephyr`).
  Building requires the Espressif HAL blobs, already fetched by the workspace `poe setup`.
- Board files live in the drivers module at
  `deps/modules/lib/rosterloh-drivers/boards/waveshare/esp32p4_nano/`. The module's
  `zephyr/module.yml` already sets `board_root: .`, so the board is auto-discoverable.
- Board name `esp32p4_nano`, vendor `waveshare`, SoC `esp32p4`. Identifiers:
  `esp32p4_nano/esp32p4/hpcore` and `esp32p4_nano/esp32p4/lpcore`. Kconfig symbols:
  `BOARD_ESP32P4_NANO`, `BOARD_ESP32P4_NANO_ESP32P4_HPCORE`, `BOARD_ESP32P4_NANO_ESP32P4_LPCORE`.
- **Validated pins** (schematic + vendor demo `ESP32-P4-NANO_Demo`):
  - UART0: TX GPIO37, RX GPIO38 (console via on-board USB-UART bridge).
  - I2C0: SCL **GPIO8**, SDA **GPIO7**.
  - SDHC (4-bit): CLK 43, CMD 44, D0 39, D1 40, D2 41, D3 42.
  - Ethernet RMII: CLK 50, TX_EN 49, TXD0 34, TXD1 35, CRS_DV 28, RXD0 29, RXD1 30; MDC 31, MDIO 52; IP101 PHY at addr 1.
  - BOOT button: `&gpio1 3` (= GPIO35; strap pin reused as RMII_TXD1, exactly as function-EV).
  - SD VDD enable GPIO45 and PHY power GPIO51 are default-on in hardware (vendor demos never drive them) — documented, not driven.
  - C6 (WiFi, documented only): SDIO Reset 54, CLK 18, CMD 19, D0–D3 14–17.
- No `.c`/`.h` files are added, so no clang-format step applies.

## File Structure

```
boards/waveshare/esp32p4_nano/
  board.yml                       # identity (Task 1)
  board.cmake                     # flash/debug runners (Task 1)
  Kconfig                         # heap sizing (Task 1)
  Kconfig.esp32p4_nano            # SoC cluster select (Task 1)
  Kconfig.defconfig               # NET_L2_ETHERNET default on hpcore (Task 1)
  Kconfig.sysbuild                # MCUboot defaults (Task 1)
  esp32p4_nano-pinctrl.dtsi       # uart0/i2c0/mdio/rmii pin groups (Task 1)
  esp32p4_nano_hpcore.dts         # HP core board (Task 1)
  esp32p4_nano_hpcore.yaml        # HP core twister metadata (Task 1)
  esp32p4_nano_hpcore_defconfig   # HP core Kconfig defaults (Task 1)
  doc/index.rst                   # board doc incl. WiFi gap (Task 1)
  esp32p4_nano_lpcore.dts         # LP core board (Task 2)
  esp32p4_nano_lpcore.yaml        # LP core twister metadata (Task 2)
  esp32p4_nano_lpcore_defconfig   # LP core Kconfig defaults (Task 2)
```

---

## Task 1: HP core board target + documentation

**Files:** Create all of:
`boards/waveshare/esp32p4_nano/{board.yml,board.cmake,Kconfig,Kconfig.esp32p4_nano,Kconfig.defconfig,Kconfig.sysbuild,esp32p4_nano-pinctrl.dtsi,esp32p4_nano_hpcore.dts,esp32p4_nano_hpcore.yaml,esp32p4_nano_hpcore_defconfig,doc/index.rst}`

**Interfaces:**
- Produces: a board target `esp32p4_nano/esp32p4/hpcore` and the Kconfig symbols
  `BOARD_ESP32P4_NANO`, `BOARD_ESP32P4_NANO_ESP32P4_HPCORE`, `BOARD_ESP32P4_NANO_ESP32P4_LPCORE`
  (the LPCORE symbol is referenced here and used by Task 2).

- [ ] **Step 1: Write the failing test — confirm the board does not yet exist**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west boards 2>/dev/null | grep esp32p4_nano || echo "NOT FOUND (expected before implementation)"
```
Expected: `NOT FOUND (expected before implementation)`.

- [ ] **Step 2: Create `board.yml`**

```yaml
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

board:
  name: esp32p4_nano
  full_name: ESP32-P4-Nano
  vendor: waveshare
  socs:
  - name: esp32p4
```

- [ ] **Step 3: Create `Kconfig.esp32p4_nano`**

```
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

config BOARD_ESP32P4_NANO
	select SOC_ESP32P4_HPCORE if BOARD_ESP32P4_NANO_ESP32P4_HPCORE
	select SOC_ESP32P4_LPCORE if BOARD_ESP32P4_NANO_ESP32P4_LPCORE
```

- [ ] **Step 4: Create `Kconfig`** (heap sizing)

```
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

config HEAP_MEM_POOL_ADD_SIZE_BOARD
	int
	default 4096 if BOARD_ESP32P4_NANO_ESP32P4_HPCORE
	default 256 if BOARD_ESP32P4_NANO_ESP32P4_LPCORE
```

- [ ] **Step 5: Create `Kconfig.defconfig`**

```
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

if BOARD_ESP32P4_NANO_ESP32P4_HPCORE

configdefault NET_L2_ETHERNET
	default y

endif # BOARD_ESP32P4_NANO_ESP32P4_HPCORE
```

- [ ] **Step 6: Create `Kconfig.sysbuild`**

```
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

choice BOOTLOADER
	default BOOTLOADER_MCUBOOT
endchoice

choice BOOT_SIGNATURE_TYPE
	default BOOT_SIGNATURE_TYPE_NONE
endchoice
```

- [ ] **Step 7: Create `board.cmake`**

```cmake
# SPDX-License-Identifier: Apache-2.0

if(NOT "${OPENOCD}" MATCHES "^${ESPRESSIF_TOOLCHAIN_PATH}/.*")
  set(OPENOCD OPENOCD-NOTFOUND)
endif()
find_program(OPENOCD openocd PATHS ${ESPRESSIF_TOOLCHAIN_PATH}/openocd-esp32/bin NO_DEFAULT_PATH)

include(${ZEPHYR_BASE}/boards/common/esp32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
```

- [ ] **Step 8: Create `esp32p4_nano-pinctrl.dtsi`**

Note vs. function-EV: `uart0_default`, `mdio_default`, `rmii_default` are identical
(same pins); `i2c0_default` uses the Nano's **SCL GPIO8 / SDA GPIO7**; no SPI group.

```dts
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/dt-bindings/pinctrl/esp-pinctrl-common.h>
#include <zephyr/dt-bindings/pinctrl/esp32p4-pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/esp32p4-gpio-sigmap.h>

&pinctrl {
	uart0_default: uart0_default {
		group1 {
			pinmux = <UART0_TX_GPIO37>;
			output-high;
		};

		group2 {
			pinmux = <UART0_RX_GPIO38>;
			bias-pull-up;
		};
	};

	i2c0_default: i2c0_default {
		group1 {
			pinmux = <I2C0_SDA_GPIO7>,
				 <I2C0_SCL_GPIO8>;
			bias-pull-up;
			drive-open-drain;
			output-high;
		};
	};

	mdio_default: mdio_default {
		group1 {
			pinmux = <SMI_MDC_GPIO31>,
				 <SMI_MDIO_GPIO52>;
		};
	};

	rmii_default: rmii_default {
		group1 {
			pinmux = <RMII_CLK_GPIO50>,
				 <RMII_TX_EN_GPIO49>,
				 <RMII_TXD0_GPIO34>,
				 <RMII_TXD1_GPIO35>,
				 <RMII_CRS_DV_GPIO28>,
				 <RMII_RXD0_GPIO29>,
				 <RMII_RXD1_GPIO30>;
		};
	};
};
```

- [ ] **Step 9: Create `esp32p4_nano_hpcore.dts`**

```dts
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/dts-v1/;

#include <espressif/esp32p4/esp32p4_n16r8.dtsi>
#include "esp32p4_nano-pinctrl.dtsi"
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <espressif/partitions_0x2000_default_16M.dtsi>

/*
 * ponytail: esp32p4_n16r8 (16 MB flash / 8 MB PSRAM) is the largest upstream P4
 * variant include. The Nano populates 32 MB PSRAM; bump the include if an
 * n16r32 dtsi lands upstream.
 *
 * WiFi/BT (out of scope): an on-board ESP32-C6 provides the radio over SDIO
 * (Reset GPIO54, CLK GPIO18, CMD GPIO19, D0-D3 GPIO14-17). Zephyr's esp_hosted
 * driver is SPI-only, so no node is defined here; see doc/index.rst.
 *
 * Power rails MicroSD VDD (GPIO45) and Ethernet PHY (GPIO51) are default-on in
 * hardware (the vendor demos never drive them), so they are not modelled.
 */

/ {
	model = "Waveshare ESP32-P4-Nano HP Core";
	compatible = "espressif,esp32p4";

	chosen {
		zephyr,sram = &sramhp;
		zephyr,console = &uart0;
		zephyr,shell-uart = &uart0;
		zephyr,flash = &flash0;
		zephyr,code-partition = &slot0_partition;
	};

	aliases {
		sw0 = &boot_button;
		watchdog0 = &wdt0;
		sdhc0 = &sdhc0;
	};

	gpio_keys {
		compatible = "gpio-keys";

		boot_button: button {
			label = "Boot Button";
			gpios = <&gpio1 3 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
			zephyr,code = <INPUT_KEY_0>;
		};
	};
};

&uart0 {
	status = "okay";
	current-speed = <115200>;
	pinctrl-0 = <&uart0_default>;
	pinctrl-names = "default";
};

&trng0 {
	status = "okay";
};

&coretemp {
	status = "okay";
};

&i2c0 {
	status = "okay";
	clock-frequency = <I2C_BITRATE_FAST>;
	pinctrl-0 = <&i2c0_default>;
	pinctrl-names = "default";
};

&usb_serial {
	status = "okay";
};

&gpio0 {
	status = "okay";
};

&gpio1 {
	status = "okay";
};

&wdt0 {
	status = "okay";
};

&dma {
	status = "okay";
};

&dma_axi {
	status = "okay";
};

&ldo {
	status = "okay";

	ldo1@1 {
		regulator-init-microvolt = <3300000>;
		regulator-boot-on;
		regulator-always-on;
	};

	ldo2@2 {
		regulator-init-microvolt = <1800000>;
		regulator-boot-on;
		regulator-always-on;
	};

	ldo4@4 {
		regulator-init-microvolt = <3300000>;
		regulator-boot-on;
		regulator-always-on;
	};
};

&sdhc {
	sdhc0: sdhc@0 {
		status = "okay";
		bus-width = <4>;
		max-bus-freq = <40000000>;
		clk-pin = <43>;
		cmd-pin = <44>;
		d0-pin = <39>;
		d1-pin = <40>;
		d2-pin = <41>;
		d3-pin = <42>;

		mmc {
			compatible = "zephyr,sdmmc-disk";
			disk-name = "SD";
			status = "okay";
		};
	};
};

&mdio {
	pinctrl-0 = <&mdio_default>;
	pinctrl-names = "default";
	status = "okay";

	phy: ethernet-phy@1 {
		compatible = "ethernet-phy";
		reg = <1>;
	};
};

&eth {
	status = "okay";
	phy-handle = <&phy>;
	pinctrl-0 = <&rmii_default>;
	pinctrl-names = "default";
};
```

> The `gpio1` node is enabled because the BOOT button lives on it (`&gpio1 3`);
> `gpio0` mirrors function-EV. `&usb_serial` stays enabled for flashing/JTAG even
> though the console is `uart0`.

- [ ] **Step 10: Create `esp32p4_nano_hpcore_defconfig`**

```
# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

CONFIG_CONSOLE=y
CONFIG_SERIAL=y
CONFIG_UART_CONSOLE=y
CONFIG_GPIO=y
CONFIG_REGULATOR=y
```

- [ ] **Step 11: Create `esp32p4_nano_hpcore.yaml`**

```yaml
identifier: esp32p4_nano/esp32p4/hpcore
name: ESP32-P4-Nano HP Core
vendor: waveshare
type: mcu
arch: riscv
toolchain:
  - zephyr
supported:
  - counter
  - entropy
  - gpio
  - hwinfo
  - i2c
  - sdhc
  - uart
  - watchdog
```

- [ ] **Step 12: Create `doc/index.rst`**

```rst
.. zephyr:board:: esp32p4_nano

Overview
********

The Waveshare ESP32-P4-Nano is a development board based on the Espressif
ESP32-P4 dual-core RISC-V SoC (plus an LP RISC-V core), with 16 MB flash and
on-board PSRAM. WiFi 6 / Bluetooth LE are provided by an on-board ESP32-C6.

Supported Features
******************

The ``esp32p4_nano/esp32p4/hpcore`` target supports:

- UART console (via the on-board USB-UART bridge, GPIO37/38)
- GPIO
- I2C0 (SCL GPIO8, SDA GPIO7; drives the on-board ES8311 audio codec)
- MicroSD card (SDHC, 4-bit: CLK 43, CMD 44, D0-D3 39-42)
- 100M Ethernet (IP101 PHY over RMII)
- Watchdog, TRNG (entropy), core temperature, DMA, LDO regulators

The ``esp32p4_nano/esp32p4/lpcore`` target runs minimal firmware on the LP core.

Not Yet Supported
*****************

- **WiFi / Bluetooth.** The radio is an on-board ESP32-C6 connected to the P4
  over **SDIO** (Reset GPIO54, CLK GPIO18, CMD GPIO19, D0-D3 GPIO14-17). Zephyr's
  ``esp_hosted`` WiFi driver is **SPI-only**, so it cannot drive this link.
  Enabling WiFi requires an SDIO esp-hosted transport (not yet in Zephyr) plus
  esp-hosted slave firmware on the C6.
- MIPI DSI display / CSI camera, the ES8311 audio path (mic/speaker), USB-OTG,
  and the RTC battery are not modelled.

The MicroSD VDD enable (GPIO45) and Ethernet PHY power (GPIO51) lines are
default-on in hardware and are not driven by the board.

Programming and Debugging
*************************

Build and flash with the standard Zephyr ESP32 flow, e.g.:

.. code-block:: console

   west build -b esp32p4_nano/esp32p4/hpcore samples/hello_world
   west flash
```

- [ ] **Step 13: Verify the board is discovered and the HP core builds**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west boards 2>/dev/null | grep esp32p4_nano
uv run west build -p always -b esp32p4_nano/esp32p4/hpcore \
  deps/zephyr/samples/hello_world --build-dir builds/p4nano_hp 2>&1 | tail -25
```
Expected: `west boards` prints `esp32p4_nano`; the build ends with a memory-usage
report and `Generated zephyr.bin` / `[100%] Built target` (no CMake or devicetree
errors). The full board devicetree (uart/i2c/sdhc/eth/ldo nodes) is parsed during
this build, so a malformed node or bad pinmux would fail here.

- [ ] **Step 14: Commit**

```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
git add boards/waveshare/esp32p4_nano
git commit -m "boards: waveshare: add ESP32-P4-Nano HP core target

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: LP core target

**Files:** Create
`boards/waveshare/esp32p4_nano/{esp32p4_nano_lpcore.dts,esp32p4_nano_lpcore.yaml,esp32p4_nano_lpcore_defconfig}`

**Interfaces:**
- Consumes: `BOARD_ESP32P4_NANO_ESP32P4_LPCORE` (declared in Task 1's `Kconfig.esp32p4_nano`),
  and the heap default in Task 1's `Kconfig`.

- [ ] **Step 1: Verify the LP core target does not build yet**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west build -p always -b esp32p4_nano/esp32p4/lpcore \
  deps/zephyr/samples/hello_world --build-dir builds/p4nano_lp 2>&1 | tail -5
```
Expected: failure — the board target `esp32p4_nano/esp32p4/lpcore` has no `.dts`/`.yaml`
yet (`Board esp32p4_nano/esp32p4/lpcore not found` or a no-DTS configure error).

- [ ] **Step 2: Create `esp32p4_nano_lpcore.dts`**

```dts
/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/dts-v1/;

#include <espressif/esp32p4/esp32p4_lpcore_n16.dtsi>
#include <espressif/partitions_0x2000_default_16M.dtsi>

/ {
	model = "Waveshare ESP32-P4-Nano LP Core";
	compatible = "espressif,esp32p4";

	chosen {
		zephyr,sram = &sramlp;
		zephyr,console = &lp_uart;
		zephyr,shell-uart = &lp_uart;
		zephyr,flash = &flash0;
		zephyr,code-partition = &slot0_lpcore_partition;
	};
};

&lp_uart {
	status = "okay";
	current-speed = <115200>;
};
```

- [ ] **Step 3: Create `esp32p4_nano_lpcore_defconfig`**

```
# SPDX-License-Identifier: Apache-2.0

CONFIG_THREAD_STACK_INFO=n
CONFIG_THREAD_CUSTOM_DATA=n
CONFIG_BOOT_BANNER=n
CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=n
CONFIG_CBPRINTF_NANO=y
CONFIG_SIZE_OPTIMIZATIONS=y
CONFIG_BUSYWAIT_CPU_LOOPS_PER_USEC=4
```

- [ ] **Step 4: Create `esp32p4_nano_lpcore.yaml`**

```yaml
identifier: esp32p4_nano/esp32p4/lpcore
name: ESP32-P4-Nano LP Core
type: mcu
arch: riscv
toolchain:
  - zephyr
supported:
  - cpu
  - uart
  - serial
testing:
  only_tags:
    - introduction
  ignore_tags:
    - kernel
    - posix
    - chre
    - cpp
vendor: waveshare
```

- [ ] **Step 5: Verify the LP core builds**

Run:
```bash
cd /home/rio/src/github/rosterloh/zephyr-applications
uv run west build -p always -b esp32p4_nano/esp32p4/lpcore \
  deps/zephyr/samples/hello_world --build-dir builds/p4nano_lp 2>&1 | tail -20
```
Expected: build completes with a memory-usage report and `[100%] Built target`,
no errors.

- [ ] **Step 6: Commit**

```bash
cd /home/rio/src/github/rosterloh/zephyr-applications/deps/modules/lib/rosterloh-drivers
git add boards/waveshare/esp32p4_nano
git commit -m "boards: waveshare: add ESP32-P4-Nano LP core target

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Done criteria

- `uv run west boards` lists `esp32p4_nano`.
- `samples/hello_world` builds for both `esp32p4_nano/esp32p4/hpcore` and
  `esp32p4_nano/esp32p4/lpcore` with no errors.
- The full HP-core devicetree (uart0/i2c0/sdhc/eth/mdio/ldo) configures and links.
- `doc/index.rst` documents supported features, the WiFi/C6 SDIO gap, and the
  default-on power rails.
- Two commits on `feat/esp32p4-nano-board`.
- Functional bring-up on hardware (flash, console, SD mount, Ethernet link) is a
  manual maintainer step; CI cannot run on the SoC.
```
