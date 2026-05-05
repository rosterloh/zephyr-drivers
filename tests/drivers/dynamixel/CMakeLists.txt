# Copyright (c) 2024 Richard Osterloh <richard.osterloh@gmail.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_dynamixel)

target_sources(app PRIVATE
    src/main.c
    src/fake_servo.c
    src/test_protocol.c
    src/test_tables.c
    src/test_motors.c
)

zephyr_include_directories(./src)
zephyr_include_directories(${ZEPHYR_DRIVERS_REPO_ROOT}/drivers/dynamixel)
