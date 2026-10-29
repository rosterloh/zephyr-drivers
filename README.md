# Zephyr Drivers

This repository provides a shared drivers module for all my other projects

## Drivers

- [Adafruit Seesaw](https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/overview) — MFD core plus GPIO, ADC, PWM, NeoPixel and rotary-encoder sub-drivers
- Adafruit QT Py header GPIO
- [Robotis Dynamixel](https://www.dynamixel.com/) serial servo (single + sync/bulk group transactions)
- [Waveshare bus servo](https://www.waveshare.com/wiki/ST3235_Servo) (serial bus servo)
- ESP32 PCNT quadrature encoder (one sensor device per counting unit)
- [SingleTact](http://www.singletact.com/) capacitive force sensor
- ESP32-P4 MIPI CSI-2 camera receiver (`espressif,esp-mipi-csi`) — RAW8/RAW10/RAW12 capture through the ISP passthrough
- Camera sensors: OmniVision OV5647, Arducam Pivariety (incl. ToF), and a fork of Zephyr's Sony IMX219
- [Microchip LAN9252](https://www.microchip.com/en-us/product/lan9252) EtherCAT SubDevice controller (SPI) — System CSR and indirect EtherCAT core CSR access
- [Analog Devices ADIN3310](https://www.analog.com/en/products/adin3310.html) 3-port TSN Ethernet switch — SPI management transport and reset for ADI's host driver

## Subsystems & libraries

- [Actuator subsystem](docs/actuator.md) — generic actuator API with typed SI setpoints, capabilities, a state machine, group operations, and pluggable backends (H-bridge DC motor, Dynamixel, Waveshare bus servo, fake)
- Kinematics joint descriptors (`rosterloh,joint`) — URDF-style joint metadata exposed over a transport-agnostic CBOR wire format

## Boards

- `waveshare/ros_driver` — ESP32-S3 dual-core board (`procpu` + `appcpu`)
- `waveshare/esp32p4_nano` — ESP32-P4 board (`hpcore` + `lpcore`); Ethernet, SD, I2C, UART console
- `m5stack/unit_poe_p4` — M5Stack Unit PoE-P4, ESP32-P4 (`hpcore`) with 16 MB flash and PoE Ethernet
- `adafruit/qt_py_esp32c3` — Adafruit QT Py ESP32-C3 with a Seeed XIAO connector

### Shields

- `adafruit_neokey_1x4` — Adafruit NeoKey 1x4 keypad
- `adafruit_neoslider` — Adafruit NeoSlider potentiometer + NeoPixels
- `adafruit_rotary_encoder` — Adafruit I2C rotary encoder

## Usage

Add the following to your west.yml

```yaml
manifest:
  remotes:
    - name: rosterloh
      url-base: https://github.com/rosterloh

  projects:
    - name: rosterloh-drivers
      repo-path: zephyr-drivers
      remote: rosterloh
      revision: main
      path: modules/lib/rosterloh-drivers
```