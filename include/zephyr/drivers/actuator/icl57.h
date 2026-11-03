/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * iCL57 extensions to the generic actuator API, for what <zephyr/actuator/
 * actuator.h> has no equivalent of: a homing datum, the drive's own stop
 * sequence, the shared RS485 bus and the runtime motion profile.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ACTUATOR_ICL57_H_
#define ZEPHYR_INCLUDE_DRIVERS_ACTUATOR_ICL57_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/actuator/actuator.h>

#include <zephyr/drivers/actuator/icl57_tuning.h>

/*
 * Emergency stop, reported as an actuator fault so the subsystem refuses every
 * setpoint from every caller. Bit 2 because bits 0 and 1 are taken in
 * icl57_read_feedback(): DRIVER(0) is an ADC/EEPROM/autotune alarm and
 * DRIVER(1) is "the motion-state fault bit is set but the alarm code was
 * unreadable or unrecognised".
 */
#define ICL57_FAULT_ESTOP ACTUATOR_FAULT_DRIVER(2)

/*
 * Emergency stop one motor: trigger the drive's own estop ramp (Pr8.02), then
 * remove the software enable (Pr0.07) so torque is gone once it has stopped.
 * Both writes are attempted even if the first fails -- a stop that cannot reach
 * the drive must still try to de-energise it.
 */
int icl57_estop(const struct device *dev);

/*
 * Home `dev` and every drive in its homing group, or every drive when `dev` is
 * NULL, and establish the datum icl57_is_homed() reports. All or nothing:
 * blocks for the whole search (seconds to minutes), and a failure or a stop
 * (ACTUATOR_STATE_FAULT on any drive in the set) e-stops and releases the
 * whole set. Returns -ECANCELED for a stop, -ENODEV with no bus or no drives.
 */
int icl57_home(const struct device *dev);

/* Has this motor been referenced since boot? See icl57_data.homed. */
bool icl57_is_homed(const struct device *dev);
/* Drop the datum for a reason only the caller can see, e.g. an application's
 * skew guard on a homing group: a racked load is still racked after a fault
 * clear. */
void icl57_datum_void(const struct device *dev);
/* A homing routine (remote or shell) holds this drive, through an aborted
 * run's settle and re-pin. No bus traffic. */
bool icl57_homing_active(const struct device *dev);
/* The devicetree homing-group: 0 = independent; a shared non-zero value MUST
 * be homed together. */
int32_t icl57_homing_group(const struct device *dev);

/* False from the drive's first failed feedback read until its next good one:
 * the poller's backoff flag, so asking costs no bus traffic. */
bool icl57_is_answering(const struct device *dev);

/* Recheck FAULT and retire PR0 under one bus hold; any failure leaves repin_pending set. */
int icl57_repin(const struct device *dev, bool stopped);
/* See icl57_data.repin_pending. No bus traffic. */
bool icl57_repin_pending(const struct device *dev);

/* The one RS485 bus, held by the feedback poller for each poll's reads. A caller
 * that needs several writes back to back (e.g. to a homing group) holds it
 * across them, so a poll can precede or follow them but never land between.
 * Recursive; never held while feedback callbacks run. */
void icl57_bus_lock(void);
void icl57_bus_unlock(void);

/* The live motion profile, and the devicetree profile-velocity-max-rpm that
 * bounds its rpm. */
struct icl57_tuning icl57_get_tuning(const struct device *dev);
uint16_t icl57_max_rpm(const struct device *dev);

/* Tuning is per homing group: members of one share every value (a group lifts
 * one load). apply_group sets the group live, or returns -EINVAL with nothing
 * changed if any member refuses the value; persist_group saves the group's live
 * value. icl57_tuning_set is both, with the persist result in *save_rc. */
int icl57_tuning_apply_group(const struct device *dev, enum icl57_tuning_field field,
			     uint32_t value);
int icl57_tuning_persist_group(const struct device *dev, enum icl57_tuning_field field);
int icl57_tuning_set(const struct device *dev, enum icl57_tuning_field field, uint32_t value,
		     int *save_rc);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ACTUATOR_ICL57_H_ */
