# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(video_esp32_csi.c)

# VIDEO_DEVICE_DEFINE lives in the video subsystem's private header; reuse it so
# the driver registers with the video device list (and the `video` shell).
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/video)

# The ESP32-P4 MIPI CSI-2 receiver has no Zephyr HAL wrapper of its own; pull the
# required source files straight out of the hal_espressif module. Their include
# dirs are already exported globally by that module's esp32p4 CMakeLists, so no
# extra include_directories are needed here. The DW-GDMA is driven through its
# header-only LL (register instance provided by the SoC linker script), and the
# bridge clock/claim (esp_hw_support/mipi_csi_share_hw_ctrl.c) is reimplemented
# inline in the driver because that file depends on FreeRTOS.
set(esp_hal ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components)
zephyr_library_sources(
  ${esp_hal}/esp_hal_cam/mipi_csi_hal.c
  ${esp_hal}/esp_hal_cam/${CONFIG_SOC_SERIES}/mipi_csi_periph.c
)

# mipi_csi_hal.c uses MHZ(x) but, outside ESP-IDF's build, nothing pulls in the
# header that provides the function-macro form. Define it just for that file.
# (COMPILE_DEFINITIONS drops function-like macros, so pass a raw -D option.)
set_source_files_properties(${esp_hal}/esp_hal_cam/mipi_csi_hal.c
  PROPERTIES COMPILE_OPTIONS "-DMHZ(x)=((x)*1000000)")
