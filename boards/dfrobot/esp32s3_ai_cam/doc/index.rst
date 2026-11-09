.. zephyr:board:: dfrobot_esp32s3_ai_cam

Overview
********

The DFRobot ESP32-S3 AI Camera (`DFR1154`_) is a 42 x 42 mm ESP32-S3 board
with an OV3660 camera, IR illumination, a PDM microphone, a MAX98357 speaker
amplifier, an LTR-308 light sensor and a microSD slot.

Hardware
********

- ESP32-S3R8 (dual-core Xtensa LX7, 240 MHz), 16 MB flash, 8 MB octal PSRAM
- OV3660 (3 MP, no IR-cut filter) on an 8-bit DVP into LCD_CAM
  (``espressif,esp32-lcd-cam-dvp``), JPEG only (``ovti,ov3660``)

  - D0..D7 = GPIO16, 18, 21, 17, 14, 7, 6, 4; VSYNC GPIO1, HREF GPIO2,
    PCLK GPIO15
  - XCLK 20 MHz on GPIO5 from LCD_CAM
  - SCCB on GPIO8 (SDA) / GPIO9 (SCL); no power-down or reset line

- 940 nm IR LEDs on GPIO47, PWM on LEDC channel 0 (``ir_led``)
- Status LED on GPIO3 (``led0``), BOOT button on GPIO0 (``sw0``)
- Gravity header: UART0 TX GPIO43, RX GPIO44
- USB-C to the ESP32-S3 USB Serial/JTAG, which is the console
- LTR-308 ambient light sensor at 0x53 on the camera SCCB bus (``light-sensor``
  alias, ``liteon,ltr308`` driver)
- MAX98357 class-D amplifier on I2S1 TX: BCLK GPIO45, LRCLK GPIO46, DIN GPIO42
  (``i2s-tx`` alias). It has no control bus, so ``CONFIG_I2S`` is all it needs.
- microSD slot on SPI2: SCK GPIO12, MOSI GPIO11, MISO GPIO13, CS GPIO10
  (disk ``SD`` via ``zephyr,sdhc-spi-slot``)
- HM6245 1 A LDO: no digital interface, nothing to configure

The microphone (PDM CLK GPIO38, DATA GPIO39) is not described. Some units ship with a GC2145 sensor instead of the OV3660; it has no JPEG
encoder and is not supported here. Pins are from DFRobot's
`DFR1154_Examples`_.

Targets:

- ``dfrobot_esp32s3_ai_cam/esp32s3/procpu``: the board
- ``dfrobot_esp32s3_ai_cam/esp32s3/appcpu``: second core, for AMP with the
  ``procpu`` image (flash uses the AMP partition layout)
- ``dfrobot_esp32s3_ai_cam/esp32s3/procpu/qemu``: Espressif QEMU, see
  `Emulation`_

Programming
***********

Hold BOOT while pressing RST to enter the ROM bootloader if the board does
not take a flash on its own, then::

   west build -b dfrobot_esp32s3_ai_cam/esp32s3/procpu <app>
   west flash

Emulation
*********

The ``/qemu`` variant runs under Espressif's QEMU fork:

.. code-block:: console

   west build -b dfrobot_esp32s3_ai_cam/esp32s3/procpu/qemu <app>
   west build -t run

It disables what QEMU does not model (Wi-Fi, Bluetooth, USB Serial/JTAG,
LEDC, I2C and so the camera and light sensor, I2S, SPI and so the microSD
slot, LCD_CAM) and moves the console to UART0. See
:zephyr_file:`boards/espressif/common/qemu.rst` for prerequisites.

.. _DFR1154: https://wiki.dfrobot.com/dfr1154/
.. _DFR1154_Examples: https://github.com/DFRobot/DFR1154_Examples
