# Zephyr Drivers

This repository provides a shared drivers module for all my other projects

## Drivers

- [Adafruit Seesaw](https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/overview) — MFD core plus GPIO, ADC, PWM, NeoPixel and rotary-encoder sub-drivers
- Adafruit QT Py header GPIO
- [Robotis Dynamixel](https://www.dynamixel.com/) serial servo (single + sync/bulk group transactions)
- [Waveshare bus servo](https://www.waveshare.com/wiki/Bus_Servo_Adapter) (serial bus servo)
- ESP32 PCNT quadrature encoder (one sensor device per counting unit)
- [AMS AS7341](https://ams.com/en/as7341) 11-channel multi-spectral sensor
- [SingleTact](http://www.singletact.com/) capacitive force sensor

## Subsystems & libraries

- [Actuator subsystem](docs/actuator.md) — generic actuator API with typed SI setpoints, capabilities, a state machine, group operations, and pluggable backends (H-bridge DC motor, Dynamixel, Waveshare bus servo, fake)
- Kinematics joint descriptors (`rosterloh,joint`) — URDF-style joint metadata exposed over a transport-agnostic CBOR wire format

## Boards

- `waveshare/ros_driver` — ESP32-S3 dual-core board (`procpu` + `appcpu`)

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