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
- 100M Ethernet (IP101 PHY over RMII, reset on GPIO51)
- Watchdog, TRNG (entropy), core temperature, DMA, LDO regulators

The ``esp32p4_nano/esp32p4/lpcore`` target runs minimal firmware on the LP core.

Not Yet Supported
*****************

- **WiFi / Bluetooth.** The radio is an on-board ESP32-C6 connected to the P4
  over **SDIO** (Reset GPIO54, CLK GPIO18, CMD GPIO19, D0-D3 GPIO14-17). Zephyr's
  ``esp_hosted`` WiFi driver is **SPI-only**, so it cannot drive this link.
  Enabling WiFi requires an SDIO esp-hosted transport (not yet in Zephyr) plus
  esp-hosted slave firmware on the C6.
- MIPI DSI display / CSI camera, the ES8311 audio path (mic/speaker), and the
  RTC battery are not modelled.
- **USB.** The SoC's USB 2.0 HS OTG port lands on a **USB-A** connector (J2), so
  this is a host port, not a device port - there is deliberately no
  ``zephyr_udc0``. Zephyr's ``drivers/usb/uhc/uhc_dwc2.c`` binds ``snps,dwc2``,
  which the SoC's ``usb_otg`` node already declares, so adding a ``zephyr_uhc0``
  label is the route to host support. Untested on P4 high-speed.

The MicroSD VDD enable (GPIO45) is default-on in hardware and is not driven by
the board. GPIO51 is **not** a power rail despite earlier notes here saying so:
the schematic names it ``PHY_RESET`` and it is modelled as the IP101's
``reset-gpios``.

Programming and Debugging
*************************

Build and flash with the standard Zephyr ESP32 flow, e.g.:

.. code-block:: console

   west build -b esp32p4_nano/esp32p4/hpcore samples/hello_world
   west flash
