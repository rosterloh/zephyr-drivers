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
- MIPI CSI-2 camera receiver on the 15-pin Raspberry Pi CSI connector (J3),
  disabled by default. It carries the ``csi_interface`` / ``csi_ep_in`` /
  ``csi_capture_port`` / ``csi_i2c`` labels that Zephyr's camera shields bind
  to, so a Raspberry Pi Camera v2 works with the in-tree shield::

     west build -b esp32p4_nano/esp32p4/hpcore --shield raspberry_pi_camera_module_2 <app>
- WiFi and Bluetooth via the on-board ESP32-C6 (see below)
- Watchdog, TRNG (entropy), core temperature, DMA, LDO regulators

The ``esp32p4_nano/esp32p4/lpcore`` target runs minimal firmware on the LP core.

WiFi and Bluetooth
******************

The ESP32-P4 has no radio of its own. Wireless connectivity is provided by the
on-board ESP32-C6, which runs the esp-hosted-mcu co-processor firmware and is
reached over **SDIO** (Reset GPIO54, CLK GPIO18, CMD GPIO19, D0-D3 GPIO14-17) -
the same wiring Espressif's own ESP32-P4-Function-EV-Board uses. The C6 sits on
SDIO slot 1 and the microSD socket on slot 0; the two share the single SDMMC
controller, which serialises transactions between them.

Zephyr's ``esp_hosted_mcu`` driver exposes the co-processor as a standard WiFi
interface and, when Bluetooth is enabled, as an HCI controller over the same
link. An application only needs ``CONFIG_WIFI=y`` (and ``CONFIG_BT=y`` for the
HCI); the driver and its SDIO transport are selected from devicetree. Note that
this is the newer ``espressif,esp-hosted-mcu`` driver, not the SPI-only
``espressif,esp-hosted`` one, which cannot drive this link.

.. warning::

   The radio firmware runs on the C6 and is flashed independently of Zephyr.
   The host driver expects the **3.x** esp-hosted-mcu line
   (``CONFIG_ESP_HOSTED_MCU_FW_VERSION_MAJOR``); it queries the running firmware
   at start-up and logs a warning when the major version differs, but cannot
   update it. The C6's UART is not brought out on this board - SDIO is the only
   link to it - so re-flashing the co-processor means pushing the image over
   SDIO from an ESP-IDF host application once, then returning to Zephyr. Boards
   shipped with an older slave build will connect, if at all, with RPC failures.

The co-processor firmware, its supported chipsets and transports, and the
protocol are documented in the upstream `ESP-Hosted-MCU`_ project.

Not Yet Supported
*****************

- MIPI DSI display, the ES8311 audio path (mic/speaker), and the RTC battery are
  not modelled.
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

.. _`ESP-Hosted-MCU`: https://github.com/espressif/esp-hosted-mcu
