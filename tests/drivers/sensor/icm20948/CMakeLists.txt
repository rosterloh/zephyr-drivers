cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(device)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Include the ICM-20948 driver path and unit-test path for private header inclusion
zephyr_include_directories(./src)
zephyr_include_directories(${ZEPHYR_DRIVERS_REPO_ROOT}/drivers/sensor/icm20948)
