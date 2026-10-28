# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(ina3221.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API ina3221_decoder.c)
