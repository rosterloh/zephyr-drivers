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

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *devs;
	size_t n = z_device_get_all_static(&devs);
	size_t count = 0;

	for (size_t i = 0; i < n; i++) {
		const struct device *dev = &devs[i];

		if (!DEVICE_API_IS(actuator, dev)) {
			continue;
		}
		shell_print(sh, "%s%s", dev->name, device_is_ready(dev) ? "" : " (not ready)");
		count++;
	}
	if (count == 0) {
		shell_print(sh, "no actuators configured");
	}
	return 0;
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
		shell_error(sh, "usage: actuator set <name> <position|velocity|effort> <value>");
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
	shell_print(sh, "caps=0x%02x%s%s%s%s%s%s%s", c,
		    (c & ACTUATOR_CAP_POSITION) ? " POSITION" : "",
		    (c & ACTUATOR_CAP_VELOCITY) ? " VELOCITY" : "",
		    (c & ACTUATOR_CAP_EFFORT) ? " EFFORT" : "",
		    (c & ACTUATOR_CAP_NEEDS_ALIGN) ? " NEEDS_ALIGN" : "",
		    (c & ACTUATOR_CAP_GROUP_NATIVE) ? " GROUP_NATIVE" : "",
		    (c & ACTUATOR_CAP_FAULT_LATCHING) ? " FAULT_LATCHING" : "",
		    (c & ACTUATOR_CAP_DRIVE_MODE) ? " DRIVE_MODE" : "");
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "usage: actuator mode <name> <normal|brake|coast>");
		return -EINVAL;
	}
	const struct device *dev = resolve(sh, argv[1]);
	if (!dev) {
		return -ENODEV;
	}
	enum actuator_drive_mode mode;

	if (strcmp(argv[2], "normal") == 0) {
		mode = ACTUATOR_DRIVE_MODE_NORMAL;
	} else if (strcmp(argv[2], "brake") == 0) {
		mode = ACTUATOR_DRIVE_MODE_BRAKE;
	} else if (strcmp(argv[2], "coast") == 0) {
		mode = ACTUATOR_DRIVE_MODE_COAST;
	} else {
		shell_error(sh, "unknown mode: %s", argv[2]);
		return -EINVAL;
	}
	int err = actuator_set_drive_mode(dev, mode);

	shell_print(sh, "mode: %d", err);
	return err;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	get_subcmds,
	SHELL_CMD_ARG(state, NULL,
		      "Print the state-machine state.\nUsage: actuator get state <name>",
		      cmd_get_state, 2, 0),
	SHELL_CMD_ARG(feedback, NULL,
		      "Read live position/velocity/effort/temperature.\n"
		      "Usage: actuator get feedback <name>",
		      cmd_get_feedback, 2, 0),
	SHELL_CMD_ARG(caps, NULL,
		      "Print the capability flags advertised by the backend.\n"
		      "Usage: actuator get caps <name>",
		      cmd_get_caps, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	actuator_subcmds,
	SHELL_CMD(list, NULL, "List the names of all configured actuators.\nUsage: actuator list",
		  cmd_list),
	SHELL_CMD_ARG(enable, NULL, "Energize an actuator.\nUsage: actuator enable <name>",
		      cmd_enable, 2, 0),
	SHELL_CMD_ARG(disable, NULL, "De-energize an actuator.\nUsage: actuator disable <name>",
		      cmd_disable, 2, 0),
	SHELL_CMD_ARG(clear_fault, NULL,
		      "Clear a latched fault and return to READY.\n"
		      "Usage: actuator clear_fault <name>",
		      cmd_clear_fault, 2, 0),
	SHELL_CMD_ARG(set, NULL,
		      "Command a setpoint (position rad, velocity rad/s, effort Nm).\n"
		      "Usage: actuator set <name> <position|velocity|effort> <value>",
		      cmd_set, 4, 0),
	SHELL_CMD_ARG(mode, NULL,
		      "Set the power-stage drive mode.\n"
		      "Usage: actuator mode <name> <normal|brake|coast>",
		      cmd_mode, 3, 0),
	SHELL_CMD(get, &get_subcmds, "Read actuator status (state|feedback|caps).", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(actuator, &actuator_subcmds, "Control and inspect actuator devices.", NULL);
