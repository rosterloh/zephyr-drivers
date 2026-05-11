/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/actuator/actuator.h>

static const struct device *resolve(const struct shell *sh, const char *name)
{
	const struct device *dev = device_get_binding(name);
	if (dev == NULL) {
		shell_error(sh, "actuator '%s' not found", name);
	}
	return dev;
}

static int cmd_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	int err = actuator_enable(dev);
	shell_print(sh, "enable: %d", err);
	return err;
}

static int cmd_disable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	int err = actuator_disable(dev);
	shell_print(sh, "disable: %d", err);
	return err;
}

static int cmd_clear_fault(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	return actuator_clear_fault(dev);
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_error(sh, "usage: actuator <name> set <position|velocity|effort> <value>");
		return -EINVAL;
	}
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	float v = strtof(argv[3], NULL);
	if (strcmp(argv[2], "position") == 0) {
		return actuator_set_position(dev, v);
	}
	if (strcmp(argv[2], "velocity") == 0) {
		return actuator_set_velocity(dev, v);
	}
	if (strcmp(argv[2], "effort") == 0) {
		return actuator_set_effort(dev, v);
	}
	shell_error(sh, "unknown mode: %s", argv[2]);
	return -EINVAL;
}

static int cmd_get_state(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	static const char *const names[] = {"DISABLED", "READY", "ALIGNING", "ACTIVE", "FAULT"};
	enum actuator_state s = actuator_get_state(dev);
	shell_print(sh, "%s", (size_t)s < ARRAY_SIZE(names) ? names[s] : "?");
	return 0;
}

static int cmd_get_feedback(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	struct actuator_feedback fb;
	int err = actuator_read_feedback(dev, &fb);
	if (err) {
		return err;
	}
	shell_print(sh, "pos=%.4f vel=%.4f eff=%.4f temp=%.1f flags=0x%08x", (double)fb.position,
		    (double)fb.velocity, (double)fb.effort, (double)fb.temperature, fb.fault_flags);
	return 0;
}

static int cmd_get_caps(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	uint32_t c = actuator_get_capabilities(dev);
	shell_print(sh, "caps=0x%02x%s%s%s%s%s%s", c,
		    (c & ACTUATOR_CAP_POSITION) ? " POSITION" : "",
		    (c & ACTUATOR_CAP_VELOCITY) ? " VELOCITY" : "",
		    (c & ACTUATOR_CAP_EFFORT) ? " EFFORT" : "",
		    (c & ACTUATOR_CAP_NEEDS_ALIGN) ? " NEEDS_ALIGN" : "",
		    (c & ACTUATOR_CAP_GROUP_NATIVE) ? " GROUP_NATIVE" : "",
		    (c & ACTUATOR_CAP_FAULT_LATCHING) ? " FAULT_LATCHING" : "");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	get_subcmds, SHELL_CMD_ARG(state, NULL, "get state", cmd_get_state, 2, 0),
	SHELL_CMD_ARG(feedback, NULL, "get feedback", cmd_get_feedback, 2, 0),
	SHELL_CMD_ARG(caps, NULL, "get caps", cmd_get_caps, 2, 0), SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	actuator_subcmds, SHELL_CMD_ARG(enable, NULL, "<name> enable", cmd_enable, 2, 0),
	SHELL_CMD_ARG(disable, NULL, "<name> disable", cmd_disable, 2, 0),
	SHELL_CMD_ARG(clear_fault, NULL, "<name> clear-fault", cmd_clear_fault, 2, 0),
	SHELL_CMD_ARG(set, NULL, "<name> set <mode> <val>", cmd_set, 4, 0),
	SHELL_CMD(get, &get_subcmds, "<name> get state|feedback|caps", NULL), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(actuator, &actuator_subcmds, "Actuator subsystem", NULL);
