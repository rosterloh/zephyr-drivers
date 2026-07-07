set(ZEPHYR_DRIVERS_REPO_ROOT ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Repo root directory")

# This is needed so that custom driver classes using system calls are taken into
# account
zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory_ifdef(CONFIG_ACTUATOR subsys/actuator)
add_subdirectory_ifdef(CONFIG_WIFI_CONNECTIVITY subsys/net/wifi_connectivity)
add_subdirectory_ifdef(CONFIG_ACTUATOR_LIB lib/actuator)
add_subdirectory_ifdef(CONFIG_KINEMATICS lib/kinematics)
add_subdirectory_ifdef(CONFIG_CDR lib/cdr)