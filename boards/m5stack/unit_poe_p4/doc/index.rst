.. _m5stack_unit_poe_p4:

M5Stack Unit PoE-P4
###################

Overview
********

The `M5Stack Unit PoE-P4 <https://docs.m5stack.com/en/unit/Unit_PoE-P4>`_ is an
ESP32-P4 module with 10/100M Ethernet and IEEE 802.3at Power over Ethernet, so a
single RJ45 carries both the network and the unit's power (6 W budget). The
ESP32-P4 has **no radio**: Ethernet is the only network interface.

Hardware
********

- ESP32-P4NRW32: dual-core RISC-V HP @ 360 MHz + LP core @ 40 MHz
- 16 MB flash, 32 MB octal PSRAM
- IP101GRI 10/100M PHY over RMII, RJ45 with PoE (802.3at, 6 W)
- 1x USB-C host (USB 2.0 HS, native UTMI PHY)
- 1x USB-C firmware download / OTG (USB0, G25/G24) — also the USB Serial/JTAG console
- Grove HY2.0-4P port: GND / 5V / G53 (yellow) / G54 (white)
- Hat2-Bus 2.54-16P and SDIO-Bus 2.54-9P expansion headers
- MIPI DSI display and MIPI CSI camera FPC connectors
- RGB LED (G14 green, G15 blue, G16 red), IR TX (G17), user button (G45)

Supported Features
==================

Only the essentials are enabled: Ethernet, the USB Serial/JTAG
console, UART0 on the SDIO-Bus header, UART1 on the Grove port (claimed by the
application overlay), GPIO, the RGB LED, the user button, the watchdog, TRNG and
the core temperature sensor.

Not enabled: MIPI DSI/CSI, the touch controller, I2C, and SDIO. There is no
microSD slot on this unit, so unlike the Waveshare ESP32-P4-ETH there is no
``sdhc0``.

The LP core is not supported — no ``lpcore`` devicetree is provided, so only
``m5stack_unit_poe_p4/esp32p4/hpcore`` builds.

Relationship to the Waveshare ESP32-P4-ETH
==========================================

The RMII/MDIO pinout and the PHY reset pin are **identical** to
``waveshare_esp32p4_eth`` — both follow the Espressif IP101 reference wiring.
The differences that matter are:

+------------------+---------------------------+-------------------------+
| Item             | Waveshare ESP32-P4-ETH    | M5Stack Unit PoE-P4     |
+==================+===========================+=========================+
| Flash            | 32 MB                     | 16 MB                   |
+------------------+---------------------------+-------------------------+
| Console          | USB-UART bridge on uart0  | USB Serial/JTAG         |
+------------------+---------------------------+-------------------------+
| microSD          | yes (``sdhc0``)           | no                      |
+------------------+---------------------------+-------------------------+
| Power            | USB-C / header            | PoE (802.3at, 6 W)      |
+------------------+---------------------------+-------------------------+
| Expansion IO     | PICO-2*20 header (P1)     | Grove + Hat2/SDIO buses |
+------------------+---------------------------+-------------------------+

This is why it is a separate board rather than an overlay on the Waveshare one:
the Waveshare DTS declares 32 MB of flash (it once pulled that in through
``esp32p4_n32r32.dtsi``; Zephyr dropped the per-variant files in 2026-09 and
each board now declares its own flash and PSRAM sizes on top of
``esp32p4.dtsi``), and a partition table sized against that on a 16 MB part is
a landmine, not a config difference.

RS485 with the M5Stack Unit ISO485
**********************************

The `Unit ISO485 <https://docs.m5stack.com/en/unit/iso485>`_ is the isolated
RS485 transceiver that pairs with the Unit PoE-P4. It plugs into the Grove
port, e.g. for a ``zephyr,modbus-serial`` bus.

- CA-IS3082W with an isolated DC-DC: 1000 VRMS isolation, ±15 kV ESD, rated to
  500 kbps
- Grove HY2.0-4P (PORT.C): GND / 5V / UART_RX (yellow) / UART_TX (white)
- **No DE line.** The connector is four pins, so there is nowhere to put one;
  the transceiver's driver-enable is switched on the unit from TX. Zephyr's
  ``zephyr,modbus-serial`` binding treats ``de-gpios`` as optional, so it is
  omitted rather than pointed at a GPIO that does nothing.
- A 120 Ω termination resistor ships in the box. Fit it at the far end of the
  bus, not at both ends of every stub.

Wiring to the Unit PoE-P4's Grove port gives host **TX = G53** and host
**RX = G54**, measured on hardware 2026-09-15. Note this is the *opposite* of
what M5Stack's PORT.C convention (yellow = host RX) reads as, because those pins
are labelled from the module's perspective rather than the host's. The
convention-derived orientation was tried first and was completely silent —
Modbus requests got no reply from any slave, which looks exactly like a dead
transceiver — and swapping the pair brought every slave back in one boot.

Programming and Debugging
*************************

Build and flash with the standard Zephyr ESP32 flow, e.g.:

.. code-block:: console

   west build -b m5stack_unit_poe_p4/esp32p4/hpcore samples/hello_world
   west flash

Flash over the download USB-C port (G25/G24). The console is the USB Serial/JTAG
device on that same port; for a console independent of USB, UART0 is available
on the SDIO-Bus header (G37 TX / G38 RX) — repoint ``zephyr,console`` and
``zephyr,shell-uart`` at ``&uart0``.

Silicon revision
================

The board selects ``SOC_ESP32P4_REV_1_3``. This was inferred from the
ESP32-P4NRW32 part number (the v3.x die is marketed as ESP32-P4X) and
**confirmed on hardware 2026-09-15**: ``esptool chip-id`` on the first unit
reports ``ESP32-P4 (revision v1.3)``, with ``WAFER_VERSION_MAJOR`` 1 and
``WAFER_VERSION_MINOR`` 3 in the eFuses. Should a later batch report v3.x,
change the ``select`` in ``Kconfig.m5stack_unit_poe_p4``. It will not fail
quietly: ``soc/espressif/esp32p4/hw_init.c`` compares the fused revision against
``SOC_ESP32P4_REV_MIN_FULL`` and refuses to boot.
