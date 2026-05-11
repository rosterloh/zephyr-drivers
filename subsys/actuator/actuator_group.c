/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/actuator_group.h>

#include "../../drivers/actuator/actuator_internal.h"

static int register_group_once(const struct actuator_group *grp);

static bool all_share_api(const struct actuator_group *grp,
			  const struct actuator_driver_api **out_api)
{
	if (grp->n == 0) {
		return false;
	}
	const struct actuator_driver_api *api =
		(const struct actuator_driver_api *)grp->devs[0]->api;
	for (size_t i = 1; i < grp->n; i++) {
		if (grp->devs[i]->api != api) {
			return false;
		}
	}
	*out_api = api;
	return true;
}

int actuator_group_set_fault_policy(const struct actuator_group *grp,
				    enum actuator_group_fault_policy policy)
{
	(void)register_group_once(grp);
	grp->data->policy = policy;
	return 0;
}

int actuator_group_enable(const struct actuator_group *grp)
{
	(void)register_group_once(grp);
	if (grp->data->latched) {
		return -EPERM;
	}
	int rc = 0;
	for (size_t i = 0; i < grp->n; i++) {
		int err = actuator_enable(grp->devs[i]);
		if (err != 0 && rc == 0) {
			rc = err;
		}
	}
	return rc;
}

int actuator_group_disable(const struct actuator_group *grp)
{
	int rc = 0;
	for (size_t i = 0; i < grp->n; i++) {
		int err = actuator_disable(grp->devs[i]);
		if (err != 0 && rc == 0) {
			rc = err;
		}
	}
	return rc;
}

int actuator_group_clear_fault(const struct actuator_group *grp)
{
	grp->data->latched = false;
	int rc = 0;
	for (size_t i = 0; i < grp->n; i++) {
		int err = actuator_clear_fault(grp->devs[i]);
		if (err != 0 && rc == 0) {
			rc = err;
		}
	}
	return rc;
}

static int per_device_set(const struct actuator_group *grp, enum actuator_mode mode,
			  const float *values)
{
	int rc = 0;
	for (size_t i = 0; i < grp->n; i++) {
		int err;
		switch (mode) {
		case ACTUATOR_MODE_POSITION:
			err = actuator_set_position(grp->devs[i], values[i]);
			break;
		case ACTUATOR_MODE_VELOCITY:
			err = actuator_set_velocity(grp->devs[i], values[i]);
			break;
		case ACTUATOR_MODE_EFFORT:
			err = actuator_set_effort(grp->devs[i], values[i]);
			break;
		default:
			return -EINVAL;
		}
		if (err != 0 && rc == 0) {
			rc = err;
		}
	}
	return rc;
}

static int dispatch_set(const struct actuator_group *grp, enum actuator_mode mode,
			const float *values)
{
	(void)register_group_once(grp);
	if (grp->data->latched) {
		return -EPERM;
	}
	const struct actuator_driver_api *api;
	if (all_share_api(grp, &api) && api->group_set_setpoints != NULL) {
		return api->group_set_setpoints(grp->devs, grp->n, mode, values);
	}
	return per_device_set(grp, mode, values);
}

int actuator_group_set_position(const struct actuator_group *grp, const float rad[])
{
	return dispatch_set(grp, ACTUATOR_MODE_POSITION, rad);
}

int actuator_group_set_velocity(const struct actuator_group *grp, const float rad_s[])
{
	return dispatch_set(grp, ACTUATOR_MODE_VELOCITY, rad_s);
}

int actuator_group_set_effort(const struct actuator_group *grp, const float nm[])
{
	return dispatch_set(grp, ACTUATOR_MODE_EFFORT, nm);
}

int actuator_group_read_feedback(const struct actuator_group *grp, struct actuator_feedback fb[])
{
	(void)register_group_once(grp);
	const struct actuator_driver_api *api;
	if (all_share_api(grp, &api) && api->group_read_feedback != NULL) {
		return api->group_read_feedback(grp->devs, grp->n, fb);
	}
	int rc = 0;
	for (size_t i = 0; i < grp->n; i++) {
		int err = actuator_read_feedback(grp->devs[i], &fb[i]);
		if (err != 0 && rc == 0) {
			rc = err;
		}
	}
	return rc;
}

/* Lightweight group registry. Each device may be a member of multiple
 * groups; we walk the registry whenever a device transitions to FAULT and
 * apply each group's policy. */

static sys_slist_t group_registry = SYS_SLIST_STATIC_INIT(&group_registry);

struct group_registry_node {
	sys_snode_t node;
	const struct actuator_group *grp;
};

static int register_group_once(const struct actuator_group *grp)
{
	static struct group_registry_node nodes[CONFIG_ACTUATOR_MAX_GROUPS];
	static atomic_t used;

	struct group_registry_node *n;
	SYS_SLIST_FOR_EACH_CONTAINER(&group_registry, n, node) {
		if (n->grp == grp) {
			return 0;
		}
	}
	atomic_val_t prev = atomic_inc(&used);
	if ((size_t)prev >= ARRAY_SIZE(nodes)) {
		atomic_dec(&used);
		return -ENOMEM;
	}
	nodes[prev].grp = grp;
	sys_slist_append(&group_registry, &nodes[prev].node);
	return 0;
}

void actuator_group_on_member_fault(const struct device *dev)
{
	struct group_registry_node *n;
	SYS_SLIST_FOR_EACH_CONTAINER(&group_registry, n, node) {
		bool member = false;
		for (size_t i = 0; i < n->grp->n; i++) {
			if (n->grp->devs[i] == dev) {
				member = true;
				break;
			}
		}
		if (!member) {
			continue;
		}
		switch (n->grp->data->policy) {
		case ACTUATOR_GROUP_POLICY_ISOLATE:
			break;
		case ACTUATOR_GROUP_POLICY_DISABLE_ALL:
		case ACTUATOR_GROUP_POLICY_ESTOP:
			for (size_t i = 0; i < n->grp->n; i++) {
				if (n->grp->devs[i] != dev) {
					(void)actuator_disable(n->grp->devs[i]);
				}
			}
			if (n->grp->data->policy == ACTUATOR_GROUP_POLICY_ESTOP) {
				n->grp->data->latched = true;
			}
			break;
		}
	}
}
