# Copyright (c) 2026 Richard Osterloh
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lan9252_driver_test)

target_sources(app PRIVATE src/main.c src/lan9252_emul.c)
