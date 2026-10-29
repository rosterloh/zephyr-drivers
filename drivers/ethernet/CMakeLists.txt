# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ADIN3310)
  zephyr_library()
  zephyr_library_sources(adin3310.c)
endif()
