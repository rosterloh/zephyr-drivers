# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_VIDEO_ESP32_CSI video_esp32_csi.c)
zephyr_library_sources_ifdef(CONFIG_VIDEO_ESP32_I2S_CAM video_esp32_i2s_cam.c)
zephyr_library_sources_ifdef(CONFIG_VIDEO_IMX219_OOT imx219.c)
zephyr_library_sources_ifdef(CONFIG_VIDEO_OV3660 ov3660.c)
zephyr_library_sources_ifdef(CONFIG_VIDEO_OV5647 ov5647.c)
zephyr_library_sources_ifdef(CONFIG_VIDEO_ARDUCAM_PIVARIETY arducam_pivariety.c)

# VIDEO_DEVICE_DEFINE lives in the video subsystem's private header; reuse it so
# the driver registers with the video device list (and the `video` shell).
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/video)
