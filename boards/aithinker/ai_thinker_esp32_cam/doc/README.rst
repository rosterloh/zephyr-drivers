.. zephyr:board:: ai_thinker_esp32_cam

Overview
********

The AI-Thinker ESP32-CAM is a small ESP32 module board with an OV2640
camera, 4 MB of PSRAM, 4 MB of flash, a white flash LED and a microSD slot. It
has no USB: the console is UART0 on GPIO1/GPIO3, normally through the ESP32-CAM-MB
adapter or a 3.3 V USB-UART.

Hardware
********

- ESP32 (dual-core Xtensa LX6, 240 MHz), 4 MB flash, 4 MB PSRAM
- OV2640 camera on an 8-bit DVP, captured through I2S0 camera mode
  (``espressif,esp32-i2s-cam``), JPEG only

  - D0..D7 = GPIO5, 18, 19, 21, 36, 39, 34, 35; VSYNC GPIO25, HREF GPIO23, PCLK GPIO22
  - XCLK 20 MHz on GPIO0 from LEDC channel 0
  - SCCB on GPIO26 (SDA) / GPIO27 (SCL), power-down on GPIO32

- Flash LED on GPIO4, PWM on LEDC channel 1 (``flash_led``)
- Red status LED on GPIO33, active low (``led0``)

Only the ``procpu`` target is provided. The microSD slot is not described: it
shares GPIO4 with the flash LED.

Programming
***********

Hold GPIO0 low (the IO0 button on the MB adapter) while resetting to enter the
ROM bootloader, then::

   west build -b ai_thinker_esp32_cam/esp32/procpu <app>
   west flash

GPIO0 is also the camera XCLK, so release it before the application starts.
