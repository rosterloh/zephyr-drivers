zephyr_library()
zephyr_library_sources(actuator_icl57.c icl57_bus.c icl57_homing.c icl57_provision.c icl57_tuning.c)
zephyr_library_sources_ifdef(CONFIG_SHELL icl57_shell.c)
