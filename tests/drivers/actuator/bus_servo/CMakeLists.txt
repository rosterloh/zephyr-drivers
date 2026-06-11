cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_actuator_bus_servo)

target_sources(app PRIVATE
    src/main.c
    src/test_set_position.c
    ${CMAKE_CURRENT_LIST_DIR}/../../bus_servo/src/fake_bus.c
)

zephyr_include_directories(../../bus_servo/src)
zephyr_include_directories(${ZEPHYR_DRIVERS_REPO_ROOT}/drivers/bus_servo)
