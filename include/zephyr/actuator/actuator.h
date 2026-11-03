/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_H_
#define ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_H_

#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/sys/slist.h>
#include <zephyr/spinlock.h>
#include <zephyr/actuator/actuator_types.h>
#include <zephyr/actuator/internal/state_machine.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup actuator Generic actuator API
 * @{
 */

/**
 * Energise an actuator. Never resumes a target set before the last disable,
 * stop or fault: a backend that cannot guarantee that refuses the enable.
 *
 * @retval -EPERM  In FAULT (clear it first) or STOPPED (actuator_clear_stop()).
 */
__syscall int actuator_enable(const struct device *dev);
__syscall int actuator_disable(const struct device *dev);
__syscall int actuator_clear_fault(const struct device *dev);

/**
 * Stop the actuator and latch it in ACTUATOR_STATE_STOPPED.
 *
 * The latch is taken before any I/O, so from the moment this is called every
 * enable and setpoint is refused (-EPERM), whatever else the actuator reports.
 * Then the backend's own controlled stop runs (or disable, for a backend
 * without one). Idempotent: calling it again retries that stop.
 *
 * @return The backend stop's result. The latch holds either way.
 */
__syscall int actuator_stop(const struct device *dev);

/**
 * Latch ACTUATOR_STATE_STOPPED with NO I/O. The hardware is not stopped: the
 * caller owes an actuator_stop(). For latching several actuators before any of
 * their blocking stops, or keeping one latched between paced stop retries.
 */
__syscall int actuator_stop_latch(const struct device *dev);

/**
 * Release a stop: STOPPED -> DISABLED, or FAULT if a fault was outstanding when
 * stopped or reported since (clear that with actuator_clear_fault()). No I/O.
 *
 * @retval -EINVAL  Not stopped.
 */
__syscall int actuator_clear_stop(const struct device *dev);

/**
 * Reference (home) an actuator that advertises ACTUATOR_CAP_NEEDS_REFERENCE.
 * Blocks until done. It must be READY or ACTIVE. While referencing it is in
 * ACTUATOR_STATE_ALIGNING, so setpoints get -EAGAIN. The backend may also
 * reference actuators mechanically coupled to this one. A failure leaves it in
 * FAULT with ACTUATOR_FAULT_REFERENCE; a stop during it leaves it STOPPED.
 *
 * Until referenced, position setpoints on such an actuator get -EACCES.
 *
 * @retval -ENOTSUP  No ACTUATOR_CAP_NEEDS_REFERENCE.
 * @retval -EPERM    Not READY or ACTIVE.
 */
__syscall int actuator_reference(const struct device *dev);

/** Whether the actuator has been referenced since boot or the last invalidation. */
__syscall bool actuator_is_referenced(const struct device *dev);

/**
 * Drop the reference for a reason only the caller can see, e.g. a coupled load
 * found racked. Position setpoints get -EACCES until it is referenced again.
 */
__syscall void actuator_invalidate_reference(const struct device *dev);

__syscall enum actuator_state actuator_get_state(const struct device *dev);
__syscall uint32_t actuator_get_capabilities(const struct device *dev);

/**
 * Command a position setpoint (radians). The actuator must be enabled: a
 * setpoint never enables it.
 *
 * @retval 0         Setpoint accepted.
 * @retval -ENOTSUP  Backend does not support position mode.
 * @retval -EAGAIN   Actuator is in ALIGNING state; retry later.
 * @retval -EPERM    Actuator is DISABLED, STOPPED or in FAULT.
 */
__syscall int actuator_set_position(const struct device *dev, float rad);
__syscall int actuator_set_velocity(const struct device *dev, float rad_s);
__syscall int actuator_set_effort(const struct device *dev, float nm);

/**
 * Command an open-loop drive: a signed fraction of the supply, -1.0..1.0.
 * The sign sets the direction; out-of-range values are clamped. Nothing
 * regulates speed or torque, so use this rather than velocity/effort on a
 * backend without a closed loop.
 *
 * @retval 0         Setpoint accepted.
 * @retval -EINVAL   @p duty is NaN.
 * @retval -ENOTSUP  Backend does not advertise ACTUATOR_CAP_DUTY.
 * @retval -EPERM    Actuator is in FAULT state; clear fault first.
 */
__syscall int actuator_set_duty(const struct device *dev, float duty);

/**
 * Set the output policy of the actuator's power stage.
 *
 * Orthogonal to the actuator state machine: setting a drive mode does not
 * change actuator_get_state(). Setting any setpoint (position/velocity/effort)
 * implicitly returns the stage to ACTUATOR_DRIVE_MODE_NORMAL.
 *
 * NORMAL semantics: a backend that supports drive_mode treats NORMAL as
 * "release brake/coast and stop driving" — outputs are left at zero (high-Z
 * on IN1/IN2-style hardware) until the next setpoint is issued. Use NORMAL
 * to clear an explicit BRAKE/COAST policy; do not expect the motor to keep
 * tracking its previous setpoint after calling it standalone.
 *
 * @retval 0         Mode applied.
 * @retval -ENOTSUP  Backend does not advertise ACTUATOR_CAP_DRIVE_MODE.
 * @retval -EPERM    Actuator is DISABLED, STOPPED or in FAULT.
 */
__syscall int actuator_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode);

/** Synchronous read: forces a backend transaction. */
__syscall int actuator_read_feedback(const struct device *dev, struct actuator_feedback *out);

/** Cached read: returns last sample populated by the driver-internal worker. */
__syscall int actuator_get_feedback(const struct device *dev, struct actuator_feedback *out);

/**
 * Apply limits; start from ACTUATOR_LIMITS_NONE. With a backend set_limits op
 * the backend applies them. Without one, position/velocity/effort limits are
 * enforced here (setpoints outside get -ERANGE) and all-NaN clears them.
 *
 * @retval -ENOTSUP  A limit nobody here can enforce (e.g. accel without an op).
 */
__syscall int actuator_set_limits(const struct device *dev, const struct actuator_limits *limits);

typedef void (*actuator_state_cb_t)(const struct device *dev, enum actuator_state new_state,
				    void *user_data);
typedef void (*actuator_feedback_cb_t)(const struct device *dev, const struct actuator_feedback *fb,
				       void *user_data);

int actuator_register_state_cb(const struct device *dev, actuator_state_cb_t cb, void *user_data);
int actuator_register_feedback_cb(const struct device *dev, actuator_feedback_cb_t cb,
				  void *user_data);

/** @} */

/**
 * @cond INTERNAL_HIDDEN
 *
 * Internal contract between subsys/actuator and backend drivers. Backends
 * include this header to implement a driver; application code must not use the
 * declarations below.
 */

__subsystem struct actuator_driver_api {
	int (*enable)(const struct device *dev);
	int (*disable)(const struct device *dev);
	int (*clear_fault)(const struct device *dev);
	/* Optional: a controlled stop that ends de-energised, ordered after any
	 * setpoint transaction already in progress. NULL = disable. */
	int (*stop)(const struct device *dev);
	/* Optional: reference devs (and any devices mechanically coupled to
	 * them), blocking. Report each device moved with
	 * actuator_report_reference_begin() and actuator_report_referenced(). */
	int (*reference)(const struct device *const *devs, size_t n);

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
	/* Faults reported while STOPPED; actuator_clear_stop() lands in FAULT
	 * when non-zero. Separate from cached_fb, which each sample overwrites. */
	uint32_t stopped_faults;
	bool referenced;
	/* limits holds caller limits for the subsystem to enforce. */
	bool limits_set;
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
 * Backend reports a successful setpoint transaction that was validated against
 * a previously observed state. If the state changed since validation, no state
 * change is forced.
 */
/**
 * Backend: a device is about to be referenced. Steps it to ALIGNING and drops
 * its reference. Returns -EPERM (or -EBUSY) if it may not start.
 */
int actuator_report_reference_begin(const struct device *dev);

/** Backend: referencing finished. ALIGNING -> READY, referenced. A device
 *  that left ALIGNING meanwhile (stopped, faulted) stays as it is. */
void actuator_report_referenced(const struct device *dev);

int actuator_report_setpoint_if_state(const struct device *dev, enum actuator_mode mode,
				      enum actuator_state expected_state);

/**
 * Backend reports fresh feedback. Subsystem updates the cache and fires
 * registered feedback callbacks. If fb->fault_flags is non-zero, the
 * subsystem also reports a FAULT event.
 */
void actuator_report_feedback(const struct device *dev, const struct actuator_feedback *fb);

/** @endcond */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/actuator.h>

#endif /* ZEPHYR_INCLUDE_ACTUATOR_ACTUATOR_H_ */
