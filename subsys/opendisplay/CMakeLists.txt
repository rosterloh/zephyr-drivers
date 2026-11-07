# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(opendisplay)
zephyr_library_sources(
  od_config.c
  od_core.c
)
zephyr_library_sources_ifdef(CONFIG_OPENDISPLAY_BLE od_ble.c)
