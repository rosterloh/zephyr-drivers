/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motion-profile tuning for the iCL-RS driver: validated speed and ramp sets,
 * shared across a homing group, and persisted per slave id with settings.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/actuator/icl57.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "icl57_profile.h"
#include "icl57_internal.h"

LOG_MODULE_DECLARE(actuator_icl57, CONFIG_ACTUATOR_LOG_LEVEL);

/* Validate one tuning field, rejecting an out-of-range value rather than
 * clamping: a silently truncated speed is the kind of thing that reads as a
 * mechanical fault later. Shared by every set path and the settings loader, so
 * a persisted value from an older build with a higher limit cannot slip past
 * the guard. */
static bool icl57_tuning_valid(const struct device *dev, enum icl57_tuning_field field,
			       uint32_t value)
{
	const struct icl57_config *cfg = dev->config;

	return field == ICL57_TUNING_RPM
		       ? icl57_tuning_rpm_valid(value, cfg->profile_velocity_max_rpm)
		       : icl57_tuning_accel_valid(value);
}

static uint16_t *icl57_tuning_slot(const struct device *dev, enum icl57_tuning_field field)
{
	struct icl57_data *d = dev->data;

	return field == ICL57_TUNING_RPM     ? &d->tuning.rpm
	       : field == ICL57_TUNING_ACCEL ? &d->tuning.accel
					     : &d->tuning.decel;
}

/* `dev` and the other members of its homing group, or `dev` alone. Motors that
 * home together move one rigid load (e.g. two screws lifting one beam), so they
 * also share one tuning: a ceiling or ramp set on one member only racks the load
 * on the next move. */
static size_t icl57_group(const struct device *dev, const struct device **out)
{
	const struct icl57_config *cfg = dev->config;
	size_t n = 0;

	if (cfg->homing_group == 0) {
		out[0] = dev;
		return 1;
	}
	for (size_t i = 0; i < ICL57_MOTOR_COUNT; i++) {
		const struct icl57_config *other = icl57_devs[i]->config;

		if (other->homing_group == cfg->homing_group) {
			out[n++] = icl57_devs[i];
		}
	}
	return n;
}

int icl57_tuning_apply_group(const struct device *dev, enum icl57_tuning_field field,
			     uint32_t value)
{
	const struct device *g[ICL57_MOTOR_COUNT];
	size_t n = icl57_group(dev, g);

	for (size_t i = 0; i < n; i++) {
		if (!icl57_tuning_valid(g[i], field, value)) {
			return -EINVAL;
		}
	}
	/* Under the bus lock, which every PR0 write that reads the tuning holds:
	 * an application writing a group's setpoints inside it sees a set land
	 * before them or after, never between (two speeds on one load). It also keeps
	 * two group sets from interleaving. Take no other lock inside it. */
	icl57_bus_lock();
	for (size_t i = 0; i < n; i++) {
		*icl57_tuning_slot(g[i], field) = (uint16_t)value;
	}
	icl57_bus_unlock();
	return 0;
}

#if defined(CONFIG_SETTINGS)

static int icl57_tuning_apply(const struct device *dev, enum icl57_tuning_field field,
			      uint32_t value)
{
	if (!icl57_tuning_valid(dev, field, value)) {
		return -EINVAL;
	}
	icl57_bus_lock();
	*icl57_tuning_slot(dev, field) = (uint16_t)value;
	icl57_bus_unlock();
	return 0;
}

/* Persisted as icl57/<slave-id>/<rpm|accel|decel>, so a value follows the drive
 * it was tuned for rather than a devicetree index. */
#define ICL57_SETTINGS_ROOT "icl57"

static const char *const icl57_field_names[] = {
	[ICL57_TUNING_RPM] = "rpm",
	[ICL57_TUNING_ACCEL] = "accel",
	[ICL57_TUNING_DECEL] = "decel",
};

static int icl57_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	const char *field_name;
	size_t id_len = settings_name_next(name, &field_name);

	if (field_name == NULL || id_len == 0 || id_len > 3) {
		return -ENOENT;
	}

	char id_str[4] = {0};

	memcpy(id_str, name, id_len);

	const struct device *dev = icl57_dev_by_id((uint8_t)strtoul(id_str, NULL, 10));

	if (dev == NULL) {
		return -ENOENT;
	}

	for (size_t f = 0; f < ARRAY_SIZE(icl57_field_names); f++) {
		if (!settings_name_steq(field_name, icl57_field_names[f], NULL)) {
			continue;
		}

		uint16_t value;

		if (len != sizeof(value)) {
			return -EINVAL;
		}
		if (read_cb(cb_arg, &value, sizeof(value)) < 0) {
			return -EIO;
		}

		/* Re-validate: the bound is a devicetree value, so a rebuild that
		 * lowered profile-velocity-max-rpm must not silently inherit a
		 * larger stored ceiling from the old image. */
		if (icl57_tuning_apply(dev, (enum icl57_tuning_field)f, value) < 0) {
			LOG_WRN("motor %u: stored %s=%u out of range, keeping devicetree value",
				(unsigned)((const struct icl57_config *)dev->config)->slave_id,
				icl57_field_names[f], value);
		}
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(icl57, ICL57_SETTINGS_ROOT, NULL, icl57_settings_set, NULL, NULL);

static int icl57_tuning_save(const struct device *dev, enum icl57_tuning_field field,
			     uint16_t value)
{
	const struct icl57_config *cfg = dev->config;
	char key[32];

	snprintk(key, sizeof(key), ICL57_SETTINGS_ROOT "/%u/%s", cfg->slave_id,
		 icl57_field_names[field]);
	return settings_save_one(key, &value, sizeof(value));
}

/* A group stored unequal -- tuned one motor at a time by an older build, or a
 * persist that failed on one member -- comes up on its gentlest values, saved
 * back so the warning does not repeat. */
static void icl57_tuning_reconcile_groups(void)
{
	for (size_t i = 0; i < ICL57_MOTOR_COUNT; i++) {
		const struct device *g[ICL57_MOTOR_COUNT];
		struct icl57_tuning t[ICL57_MOTOR_COUNT];
		struct icl57_tuning out;
		size_t n = icl57_group(icl57_devs[i], g);

		/* Once per group, from its first member. */
		if (g[0] != icl57_devs[i]) {
			continue;
		}
		for (size_t k = 0; k < n; k++) {
			t[k] = ((const struct icl57_data *)g[k]->data)->tuning;
		}
		if (!icl57_tuning_reconcile(t, n, &out)) {
			continue;
		}
		LOG_WRN("motor %u: homing group tuning differed; all %u now speed %u rpm, "
			"ramp %u/%u",
			(unsigned int)((const struct icl57_config *)g[0]->config)->slave_id,
			(unsigned int)n, out.rpm, out.accel, out.decel);
		icl57_bus_lock();
		for (size_t k = 0; k < n; k++) {
			((struct icl57_data *)g[k]->data)->tuning = out;
		}
		icl57_bus_unlock();
		for (int f = ICL57_TUNING_RPM; f <= ICL57_TUNING_DECEL; f++) {
			int rc = icl57_tuning_persist_group(g[0], (enum icl57_tuning_field)f);

			if (rc < 0) {
				LOG_WRN("reconciled %s not persisted (%d)", icl57_field_names[f],
					rc);
			}
		}
	}
}

/* Driver init runs at POST_KERNEL, before the settings backend's flash device is
 * usable, so the stored profile is loaded here instead. settings_subsys_init()
 * is idempotent, so this is safe alongside an application that also calls it. */
static int icl57_settings_load(void)
{
	int rc = settings_subsys_init();

	if (rc < 0) {
		LOG_WRN("settings init failed (%d); using devicetree motion profile", rc);
		return 0;
	}

	rc = settings_load_subtree(ICL57_SETTINGS_ROOT);
	if (rc < 0) {
		LOG_WRN("settings load failed (%d); using devicetree motion profile", rc);
	}
	icl57_tuning_reconcile_groups();
	return 0;
}

SYS_INIT(icl57_settings_load, APPLICATION, 0);

#else /* !CONFIG_SETTINGS */

static int icl57_tuning_save(const struct device *dev, enum icl57_tuning_field field,
			     uint16_t value)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(field);
	ARG_UNUSED(value);
	return -ENOTSUP;
}

#endif /* CONFIG_SETTINGS */

/* Save the live value of `field` for every member of dev's homing group. The
 * first failure is returned; the other members are still attempted. */
int icl57_tuning_persist_group(const struct device *dev, enum icl57_tuning_field field)
{
	const struct device *g[ICL57_MOTOR_COUNT];
	size_t n = icl57_group(dev, g);
	int first = 0;

	for (size_t i = 0; i < n; i++) {
		int rc = icl57_tuning_save(g[i], field, *icl57_tuning_slot(g[i], field));

		if (rc < 0 && first == 0) {
			first = rc;
		}
	}
	return first;
}

/* set_limits: the generic speed and ramp limits, live on dev's homing group
 * like any tuning set, not persisted (icl57_tuning_set() persists). */
int icl57_set_limits(const struct device *dev, const struct actuator_limits *l)
{
	/* Travel is devicetree's position-min/max-rad-milli; there is no torque
	 * limit to set over this protocol. */
	if (!isnan(l->min_position) || !isnan(l->max_position) || !isnan(l->max_effort)) {
		return -ENOTSUP;
	}

	const struct icl57_config *cfg = dev->config;
	struct icl57_tuning t = icl57_get_tuning(dev);
	int rc = icl57_limits_to_tuning(l->max_velocity, l->max_accel, l->max_decel,
					cfg->profile_velocity_max_rpm, &t);

	if (rc != 0) {
		return rc;
	}
	if (!isnan(l->max_velocity)) {
		rc = icl57_tuning_apply_group(dev, ICL57_TUNING_RPM, t.rpm);
	}
	if (rc == 0 && !isnan(l->max_accel)) {
		rc = icl57_tuning_apply_group(dev, ICL57_TUNING_ACCEL, t.accel);
	}
	if (rc == 0 && !isnan(l->max_decel)) {
		rc = icl57_tuning_apply_group(dev, ICL57_TUNING_DECEL, t.decel);
	}
	return rc;
}

/* The shell's write path: the whole homing group, then persisted. A failed
 * persist keeps the live change and is reported in *save_rc: tuning that works
 * until reboot still beats refusing it, and a pair left unequal in storage is
 * reconciled at the next boot. */
int icl57_tuning_set(const struct device *dev, enum icl57_tuning_field field, uint32_t value,
		     int *save_rc)
{
	int rc = icl57_tuning_apply_group(dev, field, value);

	if (rc < 0) {
		return rc;
	}
	*save_rc = icl57_tuning_persist_group(dev, field);
	return 0;
}
