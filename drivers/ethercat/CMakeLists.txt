# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LAN9252)
  zephyr_library()
  zephyr_library_sources(lan9252.c)
endif()
