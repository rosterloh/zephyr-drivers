zephyr_library()
zephyr_library_sources(actuator_icl57.c icl57_homing.c)
zephyr_library_sources_ifdef(CONFIG_SHELL icl57_shell.c)
