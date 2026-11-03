/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Checks for the pure arithmetic in icl57_profile.h. A ztest suite under
 * Zephyr; without it, a plain host program, so it also runs where native_sim
 * does not:
 *   cc -I include -I drivers/actuator/icl57 -o /tmp/t \
 *      tests/drivers/actuator/icl57/src/main.c && /tmp/t
 */

#include "icl57_profile.h"

#ifdef CONFIG_ZTEST
#include <zephyr/ztest.h>
#define CHECK(cond) zassert_true(cond, #cond)
#else
#include <assert.h>
#include <stdio.h>
#define CHECK(cond) assert(cond)
#endif

/* A typical setup: Pr0.00 pulse/rev and a 10 Hz command stream. */
#define CPR    10000U
#define PERIOD 100U
#define CEIL   300U

static void icl57_profile_checks(void)
{
	/* An ordinary abort re-pins as it always has; after a stop only a drive
	 * confirmed de-energised is written to. */
	CHECK(icl57_abort_may_repin(false, false, true));
	CHECK(icl57_abort_may_repin(true, true, false));
	CHECK(!icl57_abort_may_repin(true, true, true));
	CHECK(!icl57_abort_may_repin(true, false, false));

	/* A step traversed in one period runs at the average rate the stream asks
	 * for, not the drive's accel limit. 0.05 rad = 80 pulses -> 4.8 rpm. */
	CHECK(icl57_profile_rpm(80U, CPR, PERIOD, CEIL) == 5U);

	/* 2.0 rad = 3183 pulses in 100 ms = 20 rad/s = 191 rpm. */
	CHECK(icl57_profile_rpm(3183U, CPR, PERIOD, CEIL) == 191U);

	/* One full rev per period = 600 rpm, above the ceiling -> clamped. */
	CHECK(icl57_profile_rpm(CPR, CPR, PERIOD, CEIL) == CEIL);

	/* An unsizable delta (first setpoint after boot) clamps to the ceiling
	 * rather than overflowing. */
	CHECK(icl57_profile_rpm(UINT32_MAX, CPR, PERIOD, CEIL) == CEIL);

	/* A repeat command is zero-distance, but PR0 will not start at 0 rpm. */
	CHECK(icl57_profile_rpm(0U, CPR, PERIOD, CEIL) == 1U);

	/* At a 5 mm/rev lead one rev is 5 mm, so a full 0.5 m travel is
	 * 100 rev = 628.3 rad = 1000000 pulses. Streaming that in one period would
	 * need 600000 rpm, so the ceiling is what actually governs a blind move. */
	CHECK(icl57_profile_rpm(1000000U, CPR, PERIOD, CEIL) == CEIL);

	/* Sub-1-rpm steps floor to 1 rather than 0. */
	CHECK(icl57_profile_rpm(1U, CPR, PERIOD, CEIL) == 1U);

	/* Halving the period doubles the rate needed to cover the same step. */
	CHECK(icl57_profile_rpm(3183U, CPR, PERIOD / 2U, 1000U) == 382U);

	/* Degenerate config must not divide by zero. */
	CHECK(icl57_profile_rpm(80U, 0U, PERIOD, CEIL) == 1U);
	CHECK(icl57_profile_rpm(80U, CPR, 0U, CEIL) == 1U);
	CHECK(icl57_profile_rpm(80U, CPR, PERIOD, 0U) == 1U);

	/* Monotonic in delta, never above the ceiling. */
	uint16_t prev = 0U;
	for (uint32_t d = 0U; d <= 20000U; d += 137U) {
		uint16_t rpm = icl57_profile_rpm(d, CPR, PERIOD, CEIL);

		CHECK(rpm >= prev || prev == CEIL);
		CHECK(rpm >= 1U && rpm <= CEIL);
		prev = rpm;
	}

	/*
	 * Velocity sizing must reference the axis's measured position, not the last
	 * commanded target. This is the bench regression: a 30-rev move was still in
	 * flight (axis at ~2.5 rev) when the SAME target was re-sent. Differencing
	 * against the previous target gives ~0 -> the 1 rpm floor -> the drive creeps
	 * 27.5 rev at 1 rpm (28 minutes), which reads as a dead axis with no fault.
	 */
	int32_t target = 30 * (int32_t)CPR;
	int32_t last = target;    /* commanded, never reached */
	int32_t measured = 25000; /* ~2.5 rev */

	CHECK(icl57_delta_pulses(target, last, true, measured, false) == 0U);
	CHECK(icl57_profile_rpm(0U, CPR, PERIOD, CEIL) == 1U); /* the crawl */

	CHECK(icl57_delta_pulses(target, last, true, measured, true) == 275000U);
	CHECK(icl57_profile_rpm(275000U, CPR, PERIOD, CEIL) == CEIL); /* the fix */

	/* With the axis actually AT the previous target, both references agree, so a
	 * normally streamed step is sized exactly as before. */
	CHECK(icl57_delta_pulses(target + 80, target, true, target, true) == 80U);
	CHECK(icl57_profile_rpm(80U, CPR, PERIOD, CEIL) == 5U);

	/* No reference at all (first setpoint after boot) stays unsizable, which the
	 * caller runs at the ceiling rather than at 1 rpm. */
	CHECK(icl57_delta_pulses(target, 0, false, 0, false) == UINT32_MAX);

	/* Direction must not matter: the delta is a distance. */
	CHECK(icl57_delta_pulses(-5000, 0, true, 0, true) == 5000U);
	CHECK(icl57_delta_pulses(0, 0, true, -5000, true) == 5000U);

	/* Opposite extremes must not overflow the uint32 return. */
	CHECK(icl57_delta_pulses(INT32_MAX, 0, true, INT32_MIN, true) == UINT32_MAX);

	/* Travel clamp at +/-628319 milli-rad (+/-0.5 m at a 5 mm/rev lead); a
	 * setpoint inside that must pass through untouched. */
	bool clamped = true;

	CHECK(icl57_clamp_rad(0.5f, -628319, 628319, &clamped) == 0.5f);
	CHECK(!clamped);

	CHECK(icl57_clamp_rad(700.0f, -628319, 628319, &clamped) == 628.319f);
	CHECK(clamped);

	CHECK(icl57_clamp_rad(-700.0f, -628319, 628319, &clamped) == -628.319f);
	CHECK(clamped);

	/* Exactly on the limit is in range, not clamped. */
	CHECK(icl57_clamp_rad(628.319f, -628319, 628319, &clamped) == 628.319f);
	CHECK(!clamped);

	/* Sentinels mean "no limit", so nothing is ever truncated. */
	CHECK(icl57_clamp_rad(1e6f, INT32_MIN, INT32_MAX, &clamped) == 1e6f);
	CHECK(!clamped);

	/* A one-sided limit clamps only that side. */
	CHECK(icl57_clamp_rad(-5.0f, 0, INT32_MAX, &clamped) == 0.0f);
	CHECK(clamped);
	CHECK(icl57_clamp_rad(5.0f, 0, INT32_MAX, &clamped) == 5.0f);
	CHECK(!clamped);

	/* The out-param is optional. */
	CHECK(icl57_clamp_rad(700.0f, -628319, 628319, NULL) == 628.319f);

	/* Runtime tuning bounds. The overlays ship 600 rpm (50 mm/s) against a
	 * 1000 rpm hard maximum, so the operating value can be tuned upward on the
	 * bench but never persisted past the guard. */
	CHECK(icl57_tuning_rpm_valid(600U, 1000U));
	CHECK(icl57_tuning_rpm_valid(1000U, 1000U)); /* exactly at the max is fine */
	CHECK(!icl57_tuning_rpm_valid(1001U, 1000U));

	/* 0 rpm would leave PR0 unable to start a move; reject it at the edge
	 * rather than silently flooring it like the per-move sizing does. */
	CHECK(!icl57_tuning_rpm_valid(0U, 1000U));
	CHECK(icl57_tuning_rpm_valid(1U, 1000U));

	/* A persisted value from a larger-limit build must not survive a rebuild
	 * that lowered the hard maximum. */
	CHECK(!icl57_tuning_rpm_valid(600U, 300U));

	/* Ramp words: 0 is the drive's "no shaping" value and is legal; the useful
	 * tuning range is hundreds (500 = 0.3 s to 600 rpm). */
	CHECK(icl57_tuning_accel_valid(0U));
	CHECK(icl57_tuning_accel_valid(50U)); /* the factory/DT default */
	CHECK(icl57_tuning_accel_valid(500U));
	CHECK(icl57_tuning_accel_valid(ICL57_ACCEL_MAX));
	CHECK(!icl57_tuning_accel_valid(ICL57_ACCEL_MAX + 1U));

	/* A homing group reconciles to its gentlest member: after `icl57 ramp 1 500`
	 * and before `icl57 ramp 2 500`, a boot would otherwise leave drive 2 on its
	 * devicetree value of 5556. */
	{
		const struct icl57_tuning same[2] = {{180U, 500U, 500U}, {180U, 500U, 500U}};
		const struct icl57_tuning split[2] = {{200U, 500U, 600U}, {180U, 5556U, 0U}};
		const struct icl57_tuning harsh[2] = {{180U, 0U, 0U}, {180U, 500U, 500U}};
		struct icl57_tuning out;

		CHECK(!icl57_tuning_reconcile(same, 2, &out));
		CHECK(out.rpm == 180U && out.accel == 500U && out.decel == 500U);
		CHECK(icl57_tuning_reconcile(split, 2, &out));
		CHECK(out.rpm == 180U && out.accel == 5556U && out.decel == 600U);
		/* 0 is no ramp at all, so it never wins. */
		CHECK(icl57_tuning_reconcile(harsh, 2, &out));
		CHECK(out.accel == 500U && out.decel == 500U);
		CHECK(!icl57_tuning_reconcile(same, 1, &out));
	}

	/* The tuned ceiling must actually govern the per-move sizing: a blind
	 * (unsizable) delta runs at whatever the tuned value is, not the DT one. */
	CHECK(icl57_profile_rpm(UINT32_MAX, CPR, PERIOD, 600U) == 600U);

	/* Fixed homing distances: milli-radians -> command pulses. One revolution is
	 * 6283 milli-rad, which must come back as exactly counts-per-rev. */
	CHECK(icl57_milli_rad_to_pulses(0, CPR) == 0);
	CHECK(icl57_milli_rad_to_pulses(6283, CPR) == 10000);
	CHECK(icl57_milli_rad_to_pulses(-6283, CPR) == -10000);

	/* Rounding is to nearest and symmetric, as elsewhere in this header. */
	CHECK(icl57_milli_rad_to_pulses(1, CPR) == 2);
	CHECK(icl57_milli_rad_to_pulses(-1, CPR) == -2);

	/* The widest real input must not overflow the intermediate: 628319
	 * milli-rad at 10000 pulses/rev makes the scaled numerator
	 * 6.3e12, far past int32. (1000001 rather than 1000000 because 628.319 is
	 * itself the rounded form of 200*pi.) */
	CHECK(icl57_milli_rad_to_pulses(628319, CPR) == 1000001);

	/* Degenerate config must not divide by zero. */
	CHECK(icl57_milli_rad_to_pulses(6283, 0U) == 0);

	/* Search duration: one revolution at 30 rpm is 2 s. */
	CHECK(icl57_homing_timeout_ms(10000U, CPR, 30U, 0U) == 2000U);
	CHECK(icl57_homing_timeout_ms(10000U, CPR, 60U, 0U) == 1000U);

	/* The margin covers the drive's own accel/decel and slow re-approach. */
	CHECK(icl57_homing_timeout_ms(10000U, CPR, 30U, 500U) == 2500U);

	/* A zero-distance or unconfigured search still yields the margin, never 0:
	 * a 0 ms deadline would make the wait loop fail before the drive replies. */
	CHECK(icl57_homing_timeout_ms(0U, CPR, 30U, 1000U) == 1000U);
	CHECK(icl57_homing_timeout_ms(10000U, CPR, 0U, 750U) == 750U);
	CHECK(icl57_homing_timeout_ms(10000U, 0U, 30U, 750U) == 750U);

	/* A full-travel search at the 1 rpm floor is 100 minutes and must not
	 * overflow the uint32 return. */
	CHECK(icl57_homing_timeout_ms(1000000U, CPR, 1U, 0U) == 6000000U);

	/*
	 * Wrong-way detection. Slack 500 pulses, origin 10000.
	 *
	 * The sign convention is the whole point of these: getting it backwards
	 * aborts every good search and passes every runaway, and the bench cost of
	 * finding that out is an axis against its end stop.
	 */
	/* Searching negative (home_dir_negative): progress is decreasing position. */
	CHECK(!icl57_search_is_wrong_way(9000, 10000, -1, 500));  /* toward home    */
	CHECK(!icl57_search_is_wrong_way(10400, 10000, -1, 500)); /* within slack   */
	CHECK(icl57_search_is_wrong_way(10600, 10000, -1, 500));  /* away from home */

	/* Searching positive: mirror image. */
	CHECK(!icl57_search_is_wrong_way(11000, 10000, 1, 500));
	CHECK(!icl57_search_is_wrong_way(9600, 10000, 1, 500));
	CHECK(icl57_search_is_wrong_way(9400, 10000, 1, 500));

	/* Exactly at the slack boundary is NOT wrong-way: the comparison is strict,
	 * so settle after the nudge cannot trip it. */
	CHECK(!icl57_search_is_wrong_way(10500, 10000, -1, 500));

	/* The far end of a legitimate search, having reversed onto the switch: well
	 * past origin in the correct direction, so the re-approach never trips it. */
	CHECK(!icl57_search_is_wrong_way(-20000, 10000, -1, 500));

	/* Drive-frame positions near the int32 ends must not overflow the
	 * difference. Both of these are wrong-way by a wide margin. */
	CHECK(icl57_search_is_wrong_way(INT32_MAX, INT32_MIN + 1, -1, 500));
	CHECK(icl57_search_is_wrong_way(INT32_MIN, INT32_MAX - 1, 1, 500));

	/*
	 * Dead-sensor floor: the search must have COVERED ground, not merely taken
	 * time. A N.C. home input that is unwired reads "at home", so the drive
	 * completes the search without moving; progress from the trigger point is
	 * then ~0 and must fail against a quarter of the pre-move, while a genuine
	 * search's furthest sample sits near the whole pre-move. Signed the same way
	 * as the wrong-way check, so a search that ran backwards cannot pass either.
	 */
	CHECK(icl57_search_progress(10000, 10000, -1) == 0);        /* did not move   */
	CHECK(icl57_search_progress(1000, 10000, -1) == 9000);      /* toward home    */
	CHECK(icl57_search_progress(19000, 10000, 1) == 9000);      /* mirror image   */
	CHECK(icl57_search_progress(10600, 10000, -1) == -600);     /* away from home */
	CHECK(icl57_search_progress(INT32_MIN, INT32_MAX, -1) > 0); /* no overflow    */

	/*
	 * Arrival tolerance, against a belt axis and a loaded vertical screw axis.
	 * Two independent invariants, both of which have been violated in shipped
	 * code:
	 *
	 *   A. the tolerance must EXCEED the axis's resting following error, or a
	 *      move that completed can never be seen to have arrived;
	 *   B. the move must comfortably EXCEED its own tolerance, or never having
	 *      set off cannot be told apart from having arrived.
	 *
	 * Measured resting following errors, reproducible to within 10 pulses in
	 * both directions: the belt axis 65, the screw axis ~685-1115.
	 */
#define BELT_REST  65
#define SCREW_REST 1115
	/* homing-preroll-rad-milli: belt 1257 mrad = 2001 pulses, screw 15708 = 25000. */
	const int32_t belt_preroll = 2001;
	const int32_t screw_preroll = 25000;
	/* ICL57_HOMING_NUDGE_DIV is 4. Keep this in step with icl57_homing.c. */
	const int32_t nudge_div = 4;

	/* A: the pre-move clears both axes' resting error. */
	CHECK(icl57_arrive_tol(CPR, belt_preroll) > BELT_REST);
	CHECK(icl57_arrive_tol(CPR, screw_preroll) > SCREW_REST);

	/* B: the pre-move is several times its own tolerance. */
	CHECK(belt_preroll > 3 * icl57_arrive_tol(CPR, belt_preroll));
	CHECK(screw_preroll > 3 * icl57_arrive_tol(CPR, screw_preroll));

	/*
	 * The NUDGE must satisfy both as well, judged by ITS OWN distance. This is
	 * the pair of asserts that fails if ICL57_HOMING_NUDGE_DIV goes back to 10:
	 * the belt axis's nudge would be 200 pulses with a 50-pulse tolerance against
	 * 65 of resting error, and the screw axis's 2500 with 625 against 1115.
	 */
	CHECK(icl57_arrive_tol(CPR, belt_preroll / nudge_div) > BELT_REST);
	CHECK(icl57_arrive_tol(CPR, screw_preroll / nudge_div) > SCREW_REST);
	CHECK(belt_preroll / nudge_div > 3 * icl57_arrive_tol(CPR, belt_preroll / nudge_div));
	CHECK(screw_preroll / nudge_div > 3 * icl57_arrive_tol(CPR, screw_preroll / nudge_div));

	/* Judging the nudge by the PRE-MOVE's distance is the defect this replaced.
	 * Stated at the divisor it shipped with (10): the belt axis's 500-pulse pre-move
	 * tolerance exceeded the whole 200-pulse nudge, so a nudge that never
	 * happened passed as arrived. */
	CHECK(icl57_arrive_tol(CPR, belt_preroll) > belt_preroll / 10);

	/* The nudge must still leave the search more ground than the dead-sensor
	 * floor (ICL57_HOMING_MIN_TRAVEL_DIV = 4) demands of it. */
	CHECK(belt_preroll - belt_preroll / nudge_div >= belt_preroll / 4);
	CHECK(screw_preroll - screw_preroll / nudge_div >= screw_preroll / 4);

	/*
	 * Search budget: the bound PLUS the pre-move, because the search starts from
	 * behind the pre-move and must re-cover it. A SUCCESSFUL screw-axis search measured
	 * ~106314 pulses against a 100000-pulse homing-travel-limit-rad-milli, so a
	 * guard at the bound alone aborts a good home -- which is exactly what a
	 * first cut of the travel guard would have done.
	 */
	const int32_t screw_bound = 100000;
	const int32_t screw_measured_search = 106314;

	CHECK(screw_measured_search > screw_bound);                 /* the trap */
	CHECK(screw_measured_search < screw_bound + screw_preroll); /* the fix  */

	/*
	 * PR-block read-back transposition. A qty-2 verify read of register A holds
	 * A at index 1 and A+1 at index 0, as measured: 0x600F, written 0x001e,
	 * "read back" 0x0002, which is what 0x6010 held. Comparing index 0 reads
	 * every PR-block verify as a mismatch.
	 */
	const uint16_t pr_read[2] = {0x0002, 0x001e}; /* raw qty-2 read at 0x600F */

	CHECK(pr_read[icl57_pr_block_index(0)] == 0x001e); /* 0x600F, as written */
	CHECK(pr_read[icl57_pr_block_index(1)] == 0x0002); /* 0x6010, as written */
	CHECK(icl57_pr_block_index(6) == 7 && icl57_pr_block_index(7) == 6);

	/* Mapped-block decode: sentinel gate, and sign extension of both fields. */
	uint16_t blk[ICL57_MAP_N] = {0};
	struct icl57_map_sample smp = {0};

	blk[ICL57_MAP_STATE] = 0x0032;
	/* Drive-frame position, natural word order: a MAPPED read of the 0x60xx
	 * region is not pair-transposed (measured), unlike a direct one. */
	blk[ICL57_MAP_FB_HI] = 0xFFFF;
	blk[ICL57_MAP_FB_LO] = 0x8413;  /* -31725, a measured post-home position */
	blk[ICL57_MAP_VEL_LO] = 0xFFFD; /* the -3 rpm rest dither */
	blk[ICL57_MAP_DI] = 0x0002;
	blk[ICL57_MAP_SENTINEL] = (uint16_t)CPR;
	CHECK(icl57_map_decode(blk, (uint16_t)CPR, &smp));
	CHECK(smp.state == 0x0032);
	CHECK(smp.fb_pulses == -31725);
	CHECK(smp.rpm == -3);
	CHECK(smp.di == 0x0002);

	/* A block that is not our mapping -- unprogrammed, wrong read address, or
	 * the drive echoing the TARGET addresses back -- fails the sentinel and
	 * leaves the sample untouched, so a position of 0 can never leak through. */
	blk[ICL57_MAP_SENTINEL] = 0x0001;
	smp.fb_pulses = 7;
	CHECK(!icl57_map_decode(blk, (uint16_t)CPR, &smp));
	CHECK(smp.fb_pulses == 7);

	/* Pr8.02 read-back decode, one value per row of the manual's table. The
	 * homing trigger (0x20) and the quick stop (0x40) read back as themselves
	 * until the drive acts, which is the "pending" this exists to show. */
	unsigned int path = 99U;

	CHECK(icl57_pr_status_decode(0x0000, &path) == ICL57_PR_IDLE && path == 0U);
	CHECK(icl57_pr_status_decode(0x0003, &path) == ICL57_PR_IDLE && path == 3U);
	CHECK(icl57_pr_status_decode(0x0010, &path) == ICL57_PR_PENDING); /* PR0 trigger */
	CHECK(icl57_pr_status_decode(0x0020, &path) == ICL57_PR_PENDING); /* homing      */
	CHECK(icl57_pr_status_decode(0x0040, &path) == ICL57_PR_PENDING); /* quick stop  */
	CHECK(icl57_pr_status_decode(0x0100, &path) == ICL57_PR_RUNNING && path == 0U);
	CHECK(icl57_pr_status_decode(0x0200, &path) == ICL57_PR_POSITIONING);
	CHECK(icl57_pr_status_decode(0x0205, &path) == ICL57_PR_FOLLOW_ERR && path == 5U);
	CHECK(icl57_pr_status_decode(0x1234, &path) == ICL57_PR_UNKNOWN);

	/*
	 * Frame divergence, as arithmetic. A PR0 target is absolute in the DRIVE's
	 * frame, so a target computed from the REPORTED frame lands D pulses away.
	 * These are measured numbers: a belt axis after an aborted home, then two
	 * screw-axis failures the same arithmetic explains.
	 */
	const int32_t D_belt = 13571; /* reported - drive, after run 1's abort */

	CHECK(2001 + D_belt == 15572);   /* pre-move target 2001 landed at 15572   */
	CHECK(-500 + D_belt == 13071);   /* nudge should move -500, moved +13071   */
	CHECK(-25000 + 10154 == -14846); /* screw drive 1: offset and travel      */
	CHECK(-25000 + 14772 == -10228); /* screw drive 2, same session           */

	/*
	 * Frame loss: a drive that resets comes back reading ~0 wherever it stopped.
	 * A drive can only be between where it was trusted and the last target it
	 * accepted, plus a quarter rev (2500 pulses at 10000/rev) of slack. Elapsed
	 * time is deliberately not a term: a power cycle is seconds of failed reads,
	 * and any time-based reach grows until the reset looks plausible.
	 */
	struct icl57_frame_watch w = {0};

	/* At rest, no target since the last home: only the slack is plausible. */
	CHECK(icl57_frame_plausible(-9985, 0, false, -7485, CPR));
	CHECK(!icl57_frame_plausible(-9985, 0, false, -7484, CPR));
	/* Moving: anywhere between trusted and target, either direction, plus slack. */
	CHECK(icl57_frame_plausible(-9985, -22435, true, -16000, CPR));
	CHECK(icl57_frame_plausible(-22435, -9985, true, -16000, CPR));
	CHECK(icl57_frame_plausible(-9985, -22435, true, -24935, CPR)); /* overshoot */
	CHECK(!icl57_frame_plausible(-9985, -22435, true, -24936, CPR));

	/* Ordinary motion is never suspect. */
	CHECK(icl57_frame_check(&w, -147404, -22488, true, -139154, CPR) == ICL57_FRAME_OK);

	/* The case the time-based check missed: an axis parked 25 mm down (-63700), at
	 * rest, the drive power-cycles for 5 s, and comes back reading ~0. Suspect,
	 * then confirmed by a second sample still near 0. */
	CHECK(icl57_frame_check(&w, -63700, -63700, true, -3, CPR) == ICL57_FRAME_SUSPECT);
	CHECK(icl57_frame_check(&w, -63700, -63700, true, 2, CPR) == ICL57_FRAME_LOST);
	CHECK(!w.pending);

	/* A reset at the home park (-9985) is now seen too: 9985 > 2500. */
	w = (struct icl57_frame_watch){0};
	CHECK(icl57_frame_check(&w, -9985, 0, false, 0, CPR) == ICL57_FRAME_SUSPECT);
	CHECK(icl57_frame_check(&w, -9985, 0, false, 1, CPR) == ICL57_FRAME_LOST);

	/* A mispaired read is a one-off: the next sample is back on the trusted
	 * track, so nothing is lost and the suspect is forgotten. */
	w = (struct icl57_frame_watch){0};
	CHECK(icl57_frame_check(&w, -147404, -147404, true, 7, CPR) == ICL57_FRAME_SUSPECT);
	CHECK(icl57_frame_check(&w, -147404, -147404, true, -147400, CPR) == ICL57_FRAME_OK);
	CHECK(!w.pending);

	/* Two different garbage reads in a row do not agree, so neither confirms. */
	CHECK(icl57_frame_check(&w, -147404, -147404, true, 7, CPR) == ICL57_FRAME_SUSPECT);
	CHECK(icl57_frame_check(&w, -147404, -147404, true, 400000, CPR) == ICL57_FRAME_SUSPECT);
	CHECK(w.pending && w.pulses == 400000);

	/* A drive idle short of its target dropped the trigger: send it again. A
	 * measured miss: stopped at 33.74 rad against a final setpoint of 37.416. */
	CHECK(icl57_needs_retrigger(false, 53700, 59550, 200, PERIOD));
	/* Still running, or polled before RUNNING could rise: not yet a miss. */
	CHECK(!icl57_needs_retrigger(true, 53700, 59550, 200, PERIOD));
	CHECK(!icl57_needs_retrigger(false, 53700, 59550, 20, PERIOD));
	/* Arrived, within read-back noise either side, and far past int32 range. */
	CHECK(!icl57_needs_retrigger(false, 59550, 59550, 200, PERIOD));
	CHECK(!icl57_needs_retrigger(false, 59540, 59550, 200, PERIOD));
	CHECK(icl57_needs_retrigger(false, 59539, 59550, 200, PERIOD));
	CHECK(icl57_needs_retrigger(false, INT32_MIN, INT32_MAX, 200, PERIOD));

	/* A group write goes ahead only while every member is energised and
	 * referenced, re-checked before each write: a member stopped after the
	 * first write keeps the second from being sent. */
	const bool live2[] = {true, true}, ref2[] = {true, true};
	const bool one_down[] = {true, false}, one_unref[] = {true, false};

	CHECK(icl57_group_may_write(live2, ref2, 2) == 0);
	CHECK(icl57_group_may_write(one_down, ref2, 2) == -EPERM);
	CHECK(icl57_group_may_write(live2, one_unref, 2) == -EACCES);
	/* Not energised outranks unreferenced: the stop is the cause to report. */
	CHECK(icl57_group_may_write(one_down, one_unref, 2) == -EPERM);
	CHECK(icl57_group_may_write(live2, ref2, 0) == 0);
}

#ifdef CONFIG_ZTEST
ZTEST_SUITE(icl57_profile, NULL, NULL, NULL, NULL, NULL);

ZTEST(icl57_profile, test_profile)
{
	icl57_profile_checks();
}
#else
int main(void)
{
	icl57_profile_checks();
	puts("icl57_profile: all checks passed");
	return 0;
}
#endif
