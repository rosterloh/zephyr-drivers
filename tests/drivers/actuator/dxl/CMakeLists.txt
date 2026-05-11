cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(actuator_dxl_tests)

set(DXL_TEST_SRC
    ${CMAKE_CURRENT_LIST_DIR}/../../dynamixel/src/fake_servo.c
    ${CMAKE_CURRENT_LIST_DIR}/../../dynamixel/src/fake_bus.c
)

target_sources(app PRIVATE
    src/main.c
    src/test_set_position.c
    src/test_group.c
    ${DXL_TEST_SRC}
)

zephyr_include_directories(../../dynamixel/src)
zephyr_include_directories(${ZEPHYR_DRIVERS_REPO_ROOT}/drivers/dynamixel)
