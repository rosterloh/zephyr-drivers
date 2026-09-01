.. zephyr:board:: esp32p4_wifi6_poe_eth

Overview
********

The Waveshare ESP32-P4-WIFI6-POE-ETH is a multimedia development board built
around the ESP32-P4 SoC (an ESP32-P4NRW32 module with 32 MB of stacked PSRAM)
paired with a 32 MB external NOR flash. Wired networking is a 100 Mbps RJ45
port behind an IC+ IP101GRI PHY with a PoE module header; wireless comes from
an on-board ESP32-C6-MINI-1 reached over SDIO. The board also carries MIPI CSI
and DSI FPC connectors, a microSD socket, a USB 2.0 OTG HS host port, an ES8311
audio codec with microphone and speaker amplifier, and a 40-pin GPIO header.

This board definition provides both high-performance (HP) core and low-power
(LP) core targets. The Zephyr console is routed to UART0 (GPIO37/38), the port
wired to the Type-C CH343P USB-to-UART bridge.

Hardware
********

- ESP32-P4 SoC (silicon revision v1.3) with 32 MB stacked PSRAM
- 32 MB external NOR flash (GigaDevice)
- Type-C USB-to-UART port (CH343P) for power, flashing and serial console
- USB 2.0 OTG HS host port (USB-A)
- 100 Mbps Ethernet RJ45 with an IC+ IP101GRI PHY and a PoE module header
- ESP32-C6-MINI-1 radio co-processor over SDIO (Wi-Fi 6, Bluetooth 5 LE)
- microSD card slot (4-bit SDHC at 40 MHz)
- 2-lane MIPI CSI camera connector (Raspberry Pi 15-pin, J3)
- 2-lane MIPI DSI display connector (J1)
- ES8311 audio codec, NS4150B speaker amplifier, on-board microphone
- BOOT and RST buttons, power LED
- 2x20 GPIO header

Pinout
======

All values below were read off the board schematic.

.. list-table::
   :header-rows: 1

   * - Function
     - Pins
   * - Console UART0
     - TX GPIO37, RX GPIO38
   * - I2C0 (codec, CSI SCCB, DSI touch)
     - SDA GPIO7, SCL GPIO8
   * - Ethernet RMII
     - CLK GPIO50, TX_EN GPIO49, TXD0 GPIO34, TXD1 GPIO35,
       CRS_DV GPIO28, RXD0 GPIO29, RXD1 GPIO30
   * - Ethernet SMI
     - MDC GPIO31, MDIO GPIO52; PHY reset GPIO51, PHY address 1
   * - microSD (slot 0)
     - CLK GPIO43, CMD GPIO44, D0-D3 GPIO39-42; VDD enable GPIO45 (active low)
   * - ESP32-C6 SDIO (slot 1)
     - CLK GPIO18, CMD GPIO19, D0-D3 GPIO14-17; CHIP_PU GPIO54
   * - I2S0 (ES8311)
     - MCLK GPIO13, SCLK GPIO12, LRCK GPIO10, DSDIN GPIO9, ASDOUT GPIO11
   * - Speaker amplifier enable
     - GPIO53
   * - BOOT button
     - GPIO35 (shared with RMII TXD1)

Flash layout
============

The board carries 32 MB of NOR flash but uses the **16 MB** default partition
table (``partitions_0x2000_default_16M.dtsi``), giving 7936 KB MCUboot slots.

The 32 MB table's 16128 KB slots make MCUboot size its per-slot
``boot_sector_t`` arrays at 4032 entries each
(:kconfig:option:`CONFIG_BOOT_MAX_IMG_SECTORS_AUTO`), which pushes roughly
64 KB into ``.bss`` and the bootloader stops linking::

   section `.bss' will not fit in region `dram_seg'
   region `dram_seg' overflowed by 31468 bytes

That reproduces byte-for-byte on the in-tree ``esp32p4_wifi6``, which pairs the
same 32 MB table with a ``Kconfig.sysbuild`` defaulting to MCUboot - it is an
upstream combination that has never been built, not a property of this board.

The upper 16 MB of flash is left unpartitioned. Add a board-local partition for
it if an application wants bulk storage.

Supported Features
==================

.. zephyr:board-supported-hw::

Camera
======

The CSI connector is wired for the ``raspberry_pi_camera_module_1`` shield
(OV5647, this repository) and the in-tree ``raspberry_pi_camera_module_2``
(IMX219). Verified on hardware with the OV5647 at 800x640 and 800x800 RAW8,
capturing continuously at ~35 fps:

.. code-block:: console

   mise x -- west build -b esp32p4_wifi6_poe_eth/esp32p4/hpcore \
       deps/zephyr/samples/subsys/video/capture \
       --shield raspberry_pi_camera_module_1

Frames are raw Bayer GBRG; nothing in the ESP32-P4 CSI path demosaics them.
Video buffers must come from PSRAM - a 800x800 RAW8 frame is 640000 bytes and
two of them do not fit the 384 KB of internal SRAM - so an application needs
``CONFIG_ESP_SPIRAM=y`` with ``CONFIG_VIDEO_BUFFER_USE_SHARED_MULTI_HEAP=y`` and
``CONFIG_VIDEO_BUFFER_SMH_ATTRIBUTE=2``.

Two limits worth knowing before choosing a mode:

- The ESP32-P4 ISP accepts at most 1920x1080 (``ISP_LL_HSIZE_MAX`` /
  ``ISP_LL_VSIZE_MAX``), and every CSI capture goes through it, so no larger
  frame can be captured on this SoC whatever the sensor supports.
- Only 8 bits per pixel is currently usable. RAW10/RAW12 need the ISP's bypass
  path, which does not yet deliver complete frames.

Do not drive capture from the ``video`` shell on this platform:
``subsys/video/shell.c`` allocates buffers with 4-byte alignment while the CSI
driver invalidates whole cache lines, which corrupts the heap chunk header
before the buffer. The samples allocate with
``CONFIG_VIDEO_BUFFER_POOL_ALIGN`` and are unaffected.

Not modelled by this board definition:

- **Audio.** Zephyr has no ES8311 codec driver, so the I2S0 pins and the
  amplifier enable are documented above but not wired into devicetree.
- **USB Serial/JTAG.** On the ESP32-P4 it lives on GPIO24/25, which this board
  only brings out on the 40-pin header - there is no connector for it.
- **MIPI DSI.** The lanes are wired to J1 but no panel is fitted.

Silicon revision
================

The unit this port was developed against reports ``Chip rev: v1.3``, so
:kconfig:option:`CONFIG_SOC_ESP32P4_REV_1_3` is selected and both cores are
clocked at 360 MHz - 400 MHz exists only on v3.x parts, and
``soc/espressif/esp32p4/soc.c`` turns a mismatch into a build failure.

The clock ceiling is the only consequence. MCUboot and PSRAM both work on v1.3
silicon in this workspace - see the ``data_collection`` application on the
sibling ``esp32p4_nano``, which runs revert-capable OTA over SMP/UDP under
sysbuild and draws ~2.5 MB RAW10 video buffers from PSRAM. Accordingly this
board ships a ``Kconfig.sysbuild`` defaulting to
:kconfig:option:`SB_CONFIG_BOOTLOADER_MCUBOOT`, and an application only needs
``CONFIG_ESP_SPIRAM=y`` to reach the 32 MB of stacked PSRAM.

Confirm the revision on a new unit with::

   esptool --port /dev/ttyUSB0 chip-id

If a later production run ships v3.x silicon, change the ``select`` in
``Kconfig.esp32p4_wifi6_poe_eth`` and the two ``clock-frequency`` properties in
``esp32p4_wifi6_poe_eth_hpcore.dts`` together.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Build and flash from the workspace root:

.. code-block:: console

   mise run app <app> --board esp32p4_wifi6_poe_eth/esp32p4/hpcore
   mise run app <app> --board esp32p4_wifi6_poe_eth/esp32p4/hpcore --sysbuild
   mise run flash <app>

``--sysbuild`` adds MCUboot, which this board defaults to. An application that
wants revert-capable OTA should also set
``SB_CONFIG_MCUBOOT_MODE_SWAP_USING_MOVE=y`` in its ``sysbuild.conf`` -
``soc/espressif/Kconfig.sysbuild`` otherwise forces overwrite-only on every
ESP32-family board, where a bad image cannot be rolled back.

The ESP32-C6's firmware is a separate image that lives on the co-processor and
is not managed by Zephyr. Unlike the ESP32-P4-Nano, this board brings the C6's
UART0 out on header H7 (pin 1 TX, pin 2 RX), so it can be re-flashed with
``esptool`` directly rather than pushed over SDIO from an ESP-IDF host
application.

References
**********

- `ESP32-P4-WIFI6-POE-ETH product page <https://www.waveshare.com/wiki/ESP32-P4-WIFI6-POE-ETH>`_
- `Schematic <https://files.waveshare.com/wiki/ESP32-P4-WIFI6-POE-ETH/ESP32-P4-WIFI6-POE-ETH-Schematic.pdf>`_
- `ESP32-P4 datasheet <https://documentation.espressif.com/esp32-p4_datasheet_en.pdf>`_
