/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal contract between subsys/actuator and backend drivers.
 * Not part of the public API.
 */

#ifndef ROSTERLOH_DRIVERS_ACTUATOR_INTERNAL_H_
#define ROSTERLOH_DRIVERS_ACTUATOR_INTERNAL_H_

#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>
#include <zephyr/spinlock.h>
#include <zephyr/actuator/actuator_types.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/actuator/internal/state_machine.h>

#ifdef __cplusplus
extern "C" {
#endif

struct actuator_driver_api {
	int (*enable)(const struct device *dev);
	int (*disable)(const struct device *dev);
	int (*clear_fault)(const struct device *dev);

	int (*set_setpoint)(const struct device *dev, enum actuator_mode mode, float value);

	int (*read_feedback)(const struct device *dev, struct actuator_feedback *out);

	int (*set_limits)(const struct device *dev, const struct actuator_limits *limits);

	/** Optional. NULL = unsupported; backend must also not advertise
	 *  ACTUATOR_CAP_DRIVE_MODE if NULL. */
	int (*set_drive_mode)(const struct device *dev, enum actuator_drive_mode mode);

	/* Optional: same-backend group fast path. NULL = use loop fallback. */
	int (*group_set_setpoints)(const struct device *const *devs, size_t n,
				   enum actuator_mode mode, const float *values);
	int (*group_read_feedback)(const struct device *const *devs, size_t n,
				   struct actuator_feedback *out);

	/* Reserved for v2 RTIO. NULL in v1. */
	/* int (*submit)(const struct device *dev, struct rtio_iodev_sqe *sqe); */
};

/**
 * Common per-device data the subsystem maintains. Each backend embeds this
 * as the FIRST field of its own data struct.
 */
struct actuator_common_data {
	enum actuator_state state;
	uint32_t caps;
	enum actuator_mode current_mode;
	struct actuator_feedback cached_fb;
	struct actuator_limits limits;
	struct k_spinlock lock;
};

/**
 * One entry in a device's callback slist. Backends allocate a pool of these
 * inside their per-device data struct.
 */
struct actuator_cb_node {
	sys_snode_t node;
	enum {
		ACTUATOR_CB_KIND_STATE,
		ACTUATOR_CB_KIND_FEEDBACK,
	} kind;
	union {
		actuator_state_cb_t state;
		actuator_feedback_cb_t feedback;
	} fn;
	void *user_data;
};

/**
 * Per-device callback storage. Backends embed an instance of this in their
 * data struct and reference it via actuator_cb_offsets in their config.
 */
struct actuator_cb_storage {
	sys_slist_t list;
	struct actuator_cb_node *pool;
	size_t pool_n;
	atomic_t used;
};

/**
 * Backends embed this as the FIRST member of their per-device config struct.
 * It tells the subsystem where to find the callback storage inside the data
 * struct.
 */
struct actuator_cb_offsets {
	size_t storage_offset;
};

/**
 * Backend reports a state-machine event (e.g. fault detected, alignment done).
 * Called from driver context (worker thread or setpoint path; never ISR).
 * Subsystem applies the transition, updates fault flags, fires state callbacks.
 */
void actuator_report_state(const struct device *dev, enum actuator_sm_event event,
			   uint32_t fault_flags);

/**
 * Backend reports fresh feedback. Subsystem updates the cache and fires
 * registered feedback callbacks. If fb->fault_flags is non-zero, the
 * subsystem also reports a FAULT event.
 */
void actuator_report_feedback(const struct device *dev, const struct actuator_feedback *fb);

#ifdef __cplusplus
}
#endif

#endif /* ROSTERLOH_DRIVERS_ACTUATOR_INTERNAL_H_ */
