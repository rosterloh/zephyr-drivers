# Zephyr Drivers

This repository provides a shared drivers module for all my other projects

### Drivers

- [Adafruit SeeSaw](https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/overview)
- [Dynamixel Serial Servo](https://www.dynamixel.com/)

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