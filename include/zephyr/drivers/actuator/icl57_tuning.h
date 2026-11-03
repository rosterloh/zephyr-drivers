/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * The iCL57 runtime motion profile and its limits. Pure: no Zephyr, so a host
 * test can include it.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ACTUATOR_ICL57_TUNING_H_
#define ZEPHYR_INCLUDE_DRIVERS_ACTUATOR_ICL57_TUNING_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Runtime-tunable motion profile. Seeded from devicetree at init and overlaid
 * by persisted settings at boot; `icl57 speed` / `icl57 ramp` edit it live.
 *
 * accel/decel are the PR0 Pr9.04/Pr9.05 words in ms per 1000 rpm — a TIME to
 * slew 1000 rpm, so a LARGER value is a SLOWER ramp. At the factory 50, a move
 * to 600 rpm ramps in 30 ms (~1.3 mm at the 5 mm/rev lead), which reads as no
 * ramp at all; ~500 gives 0.3 s and a visible trapezoid.
 */
struct icl57_tuning {
	uint16_t rpm;   /* velocity ceiling, Pr9.03            */
	uint16_t accel; /* Pr9.04, ms/1000rpm                  */
	uint16_t decel; /* Pr9.05, ms/1000rpm                  */
};

/* Selects a field of struct icl57_tuning for the shell and settings paths,
 * which both set one named value at a time. */
enum icl57_tuning_field {
	ICL57_TUNING_RPM,
	ICL57_TUNING_ACCEL,
	ICL57_TUNING_DECEL,
};

/* Pr6.03's documented range bounds the PR0 accel/decel words too. */
#define ICL57_ACCEL_MAX 10000U

/*
 * Validate a velocity ceiling (rpm) against the per-instance hard maximum.
 *
 * The ceiling is not just the streaming cap: it is also the rate used for a
 * delta the driver cannot size — the first setpoint after boot. Because the
 * runtime value persists, an over-large setting would silently become the speed
 * of a blind, potentially full-travel move on the next power-up, so the DT
 * `profile-velocity-max-rpm` bounds what can be stored at all.
 *
 * Returns true when `rpm` is usable.
 */
static inline bool icl57_tuning_rpm_valid(uint32_t rpm, uint16_t max_rpm)
{
	return rpm >= 1U && rpm <= (uint32_t)max_rpm;
}

/* Validate a PR0 accel/decel word (ms per 1000 rpm). 0 is the drive's
 * "no ramp shaping" value and is legal. */
static inline bool icl57_tuning_accel_valid(uint32_t accel)
{
	return accel <= ICL57_ACCEL_MAX;
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_ACTUATOR_ICL57_TUNING_H_ */
