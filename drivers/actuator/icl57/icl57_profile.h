/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure setpoint-profile arithmetic for the iCL57 backend: no Zephyr, no I/O, no
 * globals, so it is unit-testable on the host (see tests/drivers/actuator/icl57).
 */

#ifndef ICL57_PROFILE_H_
#define ICL57_PROFILE_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/actuator/icl57_tuning.h>

/*
 * Velocity (rpm) that traverses `delta_pulses` over one `period_ms` command
 * period.
 *
 * The drive's PR0 block carries a velocity word on every write, but a constant
 * value makes streamed absolute setpoints "dash and dwell": with accel 50
 * (ms per 1000 rpm) a move is acceleration-limited until the step exceeds
 * v^2/a, so a small step slams to the target at the accel limit and then idles
 * out the rest of the command period. Sizing the velocity so the move fills the
 * period instead keeps the axis moving continuously and drops the peak rate to
 * the average the command stream actually asks for.
 *
 * Clamped to [1, max_rpm]: PR0 never starts a move at 0 rpm (a zero-distance
 * repeat command still has to be a legal block), and the ceiling bounds a delta
 * we cannot size — notably the first setpoint after boot, where there is no
 * previous target to difference against. Choose max_rpm to be safe for an
 * unbounded move.
 */
static inline uint16_t icl57_profile_rpm(uint32_t delta_pulses, uint32_t counts_per_rev,
					 uint32_t period_ms, uint16_t max_rpm)
{
	uint64_t den = (uint64_t)counts_per_rev * (uint64_t)period_ms;

	if (den == 0U || max_rpm == 0U) {
		return 1U;
	}

	/* rpm = delta_revs / period_minutes, rounded to nearest. */
	uint64_t rpm = ((uint64_t)delta_pulses * 60000ULL + den / 2ULL) / den;

	if (rpm < 1ULL) {
		return 1U;
	}
	if (rpm > (uint64_t)max_rpm) {
		return max_rpm;
	}

	return (uint16_t)rpm;
}

/*
 * A fixed distance in milli-radians expressed in command pulses (Pr0.00
 * resolution).
 *
 * Homing works in distances the operator sets once in devicetree — the pre-move
 * off home, the search bound — rather than in streamed setpoints, so unlike the
 * setpoint path this applies no travel clamp, no host-side zero and no direction
 * sign. A homing distance is a magnitude in the drive's own frame.
 */
static inline int32_t icl57_milli_rad_to_pulses(int32_t milli_rad, uint32_t counts_per_rev)
{
	if (counts_per_rev == 0U) {
		return 0;
	}

	/* pulses = (milli_rad / 1000) * counts_per_rev / (2*pi). Scaled to integers:
	 * 2*pi*1000 = 6283.185307, so take the numerator x1000 over 6283185. The
	 * widest real input (628319 milli-rad at 10000 pulses/rev) makes that
	 * numerator 6.3e12, which needs the 64-bit intermediate. */
	int64_t num = (int64_t)milli_rad * (int64_t)counts_per_rev * 1000LL;
	const int64_t den = 6283185LL;
	int64_t half = den / 2;

	/* Round to nearest, away from zero. */
	return (int32_t)((num >= 0 ? num + half : num - half) / den);
}

/*
 * Worst-case duration (ms) of a move or search that covers `bound_pulses` at
 * `rpm`, plus `margin_ms`.
 *
 * The margin covers what this cannot model: the drive's own accel and decel
 * ramps, and a homing search's slow second-segment re-approach. Sizing the wait
 * loop's deadline too tightly is worse than sizing it loosely — an early give-up
 * reports a failure while the axis is still legitimately moving, and the caller
 * then stops an axis that was working.
 *
 * Returns `margin_ms` alone for an unconfigured axis rather than 0, so a caller
 * that uses this as a deadline always gives the drive at least one chance to
 * reply.
 */
static inline uint32_t icl57_homing_timeout_ms(uint32_t bound_pulses, uint32_t counts_per_rev,
					       uint16_t rpm, uint32_t margin_ms)
{
	if (rpm == 0U || counts_per_rev == 0U) {
		return margin_ms;
	}

	/* ms = revs / (rpm per ms) = pulses * 60000 / (counts_per_rev * rpm). */
	uint64_t ms =
		((uint64_t)bound_pulses * 60000ULL) / ((uint64_t)counts_per_rev * (uint64_t)rpm);

	return (uint32_t)ms + margin_ms;
}

/*
 * Distance the next move actually has to cover, in command pulses.
 *
 * Measured from where the axis IS whenever a fresh feedback sample exists, and
 * only from the previous target when one does not.
 *
 * Differencing against the previous target alone is what made an interrupted
 * move crawl. PR0's INS bit lets a new setpoint abandon a move still in flight,
 * so the previous target is a position the axis never reached: re-commanding
 * (almost) the same target yields a delta of ~0, which sizes at the 1 rpm floor
 * below, and the drive then creeps the WHOLE remaining distance at 1 rpm. On the
 * bench that read as "the axis has stopped accepting commands" — with no fault,
 * because nothing had failed. The same staleness is why a streamed axis that
 * cannot keep up lags: its velocity is sized from a target it is nowhere near.
 *
 * Returns UINT32_MAX when there is no reference at all (the first setpoint after
 * boot), which callers size at the velocity ceiling.
 */
static inline uint32_t icl57_delta_pulses(int32_t target, int32_t last, bool have_last,
					  int32_t measured, bool have_measured)
{
	int32_t ref;

	if (have_measured) {
		ref = measured;
	} else if (have_last) {
		ref = last;
	} else {
		return UINT32_MAX;
	}

	int64_t diff = (int64_t)target - (int64_t)ref;

	return (uint32_t)(diff < 0 ? -diff : diff);
}

/*
 * Reconcile the tunings of motors that must agree -- a homing group, which moves
 * one rigid load -- to the gentlest of them: the lowest ceiling and the slowest ramps.
 * A larger accel/decel word is slower, and 0 ("no ramp shaping") the harshest,
 * so max() is the gentler for both. Each result is valid for every member: the
 * min rpm is no larger than any member's own, valid, value.
 *
 * Returns true when the members differed, i.e. `*out` changes at least one.
 */
static inline bool icl57_tuning_reconcile(const struct icl57_tuning *t, size_t n,
					  struct icl57_tuning *out)
{
	bool differ = false;

	*out = t[0];
	for (size_t i = 1; i < n; i++) {
		differ |= t[i].rpm != t[0].rpm || t[i].accel != t[0].accel ||
			  t[i].decel != t[0].decel;
		out->rpm = t[i].rpm < out->rpm ? t[i].rpm : out->rpm;
		out->accel = t[i].accel > out->accel ? t[i].accel : out->accel;
		out->decel = t[i].decel > out->decel ? t[i].decel : out->decel;
	}
	return differ;
}

/*
 * Clamp a position setpoint (radians) to the configured travel limits, given in
 * milli-radians with INT32_MIN/INT32_MAX meaning "no limit". Sets *clamped when
 * the setpoint was truncated, so the caller can report it: a clamp at the wrong
 * place silently stops the axis reaching commanded positions, and the limits are
 * derived placeholders rather than measured travel.
 */
static inline float icl57_clamp_rad(float rad, int32_t min_milli, int32_t max_milli, bool *clamped)
{
	float out = rad;

	if (min_milli != INT32_MIN) {
		float lo = (float)min_milli / 1000.0f;

		if (out < lo) {
			out = lo;
		}
	}
	if (max_milli != INT32_MAX) {
		float hi = (float)max_milli / 1000.0f;

		if (out > hi) {
			out = hi;
		}
	}

	if (clamped != NULL) {
		*clamped = (out != rad);
	}

	return out;
}

/*
 * How far a homing search has progressed toward home, in pulses.
 *
 * `origin` is the position when the search was triggered and `sign` is +1 if
 * the search should make position increase, -1 if decrease. Positive is toward
 * home, negative is away from it.
 *
 * 64-bit: `pos` and `origin` are drive-frame pulse counts that can legitimately
 * sit near the ends of int32, and their difference would overflow.
 */
static inline int64_t icl57_search_progress(int32_t pos, int32_t origin, int32_t sign)
{
	return (int64_t)sign * ((int64_t)pos - (int64_t)origin);
}

/*
 * Is a homing search travelling AWAY from home?
 *
 * `slack` is how far backwards from `origin` is tolerated before the answer is
 * yes.
 *
 * Measured against `origin` rather than the previous sample on purpose: a
 * legitimate search reverses near the end, because the drive decelerates past
 * the switch and creeps back onto it. That reversal happens far past `origin`
 * in the correct direction, so it cannot be mistaken for a wrong-way run. Only
 * NET travel backwards from where the search began counts.
 */
static inline bool icl57_search_is_wrong_way(int32_t pos, int32_t origin, int32_t sign,
					     int32_t slack)
{
	return icl57_search_progress(pos, origin, sign) < -(int64_t)slack;
}

/* Arrival tolerance as a fraction of a revolution: cpr/5 = 2000 pulses on these
 * drives. Sized for the MOVING following error (161-259 pulses measured) and
 * for the resting error of a loaded vertical screw axis (~685-1115 pulses) --
 * both of which a tighter value can never satisfy, so a move that completed
 * perfectly well reports "did not arrive". Measure before tightening. */
#define ICL57_HOMING_ARRIVE_DIV 5

/* ...and as a fraction of the move being checked, so a SHORT move is not judged
 * by a tolerance approaching its own length. */
#define ICL57_HOMING_ARRIVE_MOVE_DIV 4

/*
 * Arrival tolerance in pulses, for a move of `move_pulses`.
 *
 * cpr/5 alone is wrong for short moves. A 2001-pulse pre-move against a
 * 2000-pulse motor-derived tolerance, so arrival was satisfied by two pulses of
 * drift and the move it is supposed to confirm could be skipped entirely -- the
 * axis would then start its search still sitting on its switch, which presents
 * as the "0 pulses of search travel" failure.
 *
 * So bound it by the move as well as by the motor: a quarter of the distance
 * being checked, whichever is smaller.
 *
 * THE MOVE PASSED HERE MUST BE THE MOVE BEING WAITED ON. Passing a different,
 * longer one silently reintroduces the same defect from the other side: the
 * direction-seeding nudge was checked with the PRE-MOVE's distance, giving a
 * 500-pulse tolerance for a 200-pulse nudge, so a nudge the drive refused
 * outright passed as arrived.
 *
 * And keep any move this judges comfortably LARGER than the tolerance it
 * produces, or arrival cannot be distinguished from never having set off. If an
 * axis ever needs BOTH a short move and a tolerance above its resting error,
 * those two demands are in direct conflict and the answer is a longer move, not
 * a looser tolerance.
 */
static inline int32_t icl57_arrive_tol(int32_t counts_per_rev, int32_t move_pulses)
{
	int32_t by_motor = counts_per_rev / ICL57_HOMING_ARRIVE_DIV;
	int32_t by_move =
		(move_pulses < 0 ? -move_pulses : move_pulses) / ICL57_HOMING_ARRIVE_MOVE_DIV;

	return by_move < by_motor ? by_move : by_motor;
}

/*
 * FC03 reads of the Pr8/Pr9 block (0x6000 and up) return every ADJACENT WORD
 * PAIR TRANSPOSED relative to the block start: register base+i arrives at index
 * i^1. Established by writing eight distinct values with FC06 and reading
 * them back as one block. The manual documents the
 * Pr0-Pr7 parameters as 32-bit pairs "high word first" and the PR block as
 * single 16-bit registers; a drive serving both through one pair-oriented path
 * would emit exactly this, and it also explains why a qty-1 read of the block
 * is rejected -- there is no half-pair to return. FC16 writes and the 0x10xx
 * status registers are not affected.
 *
 * Index into a PR-block read with this, never directly. It is what made the
 * write-verify in icl57_shell_write() report the NEXT register's value on every
 * Pr8 write and made "0x600A always read 0": word 0 of
 * that read was 0x600B, the home-switch position high word, which is 0.
 *
 * Read an EVEN quantity: the last word of an odd-length read has no partner and
 * what the drive returns there is unknown.
 */
static inline size_t icl57_pr_block_index(size_t i)
{
	return i ^ 1U;
}

/*
 * Slot layout of the register-mapping block the driver programs at enable
 * under CONFIG_ACTUATOR_ICL57_MAPPED_POLL (ICL57_REG_MAP_TARGETS in
 * icl57_internal.h). Kept here, not there, because the decoder below is pure
 * and host-tested; actuator_icl57.c's icl57_map_targets[] is indexed by it.
 */
enum icl57_map_slot {
	ICL57_MAP_STATE,    /* 0x1003 motion state                  */
	ICL57_MAP_FB_HI,    /* 0x602C drive-frame position, high word */
	ICL57_MAP_FB_LO,    /* 0x602D drive-frame position, low word  */
	ICL57_MAP_VEL_LO,   /* 0x1047 feedback velocity, int16 rpm */
	ICL57_MAP_DI,       /* 0x0179 digital input state          */
	ICL57_MAP_SENTINEL, /* 0x0001 Pr0.00; must read counts/rev */
	ICL57_MAP_N,
};

struct icl57_map_sample {
	uint16_t state;
	int32_t fb_pulses; /* drive frame, COMMAND pulses -- see ICL57_REG_DRIVE_FB_POS */
	int16_t rpm;
	uint16_t di;
};

/*
 * Decode a mapped-block read. Returns false, leaving *out untouched, when the
 * sentinel slot does not hold `sentinel`: the block is then not the mapping this
 * driver programmed -- an unprogrammed area, the wrong read address, a drive
 * that power-cycled, or the same pair transposition the PR block shows -- and
 * nothing in it may be believed.
 */
static inline bool icl57_map_decode(const uint16_t *blk, uint16_t sentinel,
				    struct icl57_map_sample *out)
{
	if (blk[ICL57_MAP_SENTINEL] != sentinel) {
		return false;
	}

	out->state = blk[ICL57_MAP_STATE];
	/* Natural word order: a mapped read of the 0x60xx region is NOT
	 * pair-transposed, unlike a direct block read of it. Measured by mapping
	 * 0x6006/0x6007 and comparing both ways. */
	out->fb_pulses = (int32_t)(((uint32_t)blk[ICL57_MAP_FB_HI] << 16) | blk[ICL57_MAP_FB_LO]);
	out->rpm = (int16_t)blk[ICL57_MAP_VEL_LO];
	out->di = blk[ICL57_MAP_DI];
	return true;
}

/*
 * Pr8.02 (0x6002) read back. The register the driver triggers moves through is
 * also the drive's own progress report (manual 5.5.3 and Appendix A, which adds
 * the following-error row 5.5.3 omits):
 *   0x000P                        path P complete; a new command is accepted
 *   0x001P, 0x0020, 0x0021, 0x0040  the written command, not yet acted on
 *   0x010P                        path P running
 *   0x0200                        command complete, still settling
 *   0x020P (P != 0)               position following error during path P
 * Read it at qty 2 and un-swap (icl57_pr_block_index). The homing trace prints
 * it, so "did the drive take the trigger" is read rather than inferred.
 */
enum icl57_pr_status {
	ICL57_PR_IDLE,
	ICL57_PR_PENDING,
	ICL57_PR_RUNNING,
	ICL57_PR_POSITIONING,
	ICL57_PR_FOLLOW_ERR,
	ICL57_PR_UNKNOWN,
};

static inline enum icl57_pr_status icl57_pr_status_decode(uint16_t v, unsigned int *path)
{
	*path = v & 0x0FU;

	if (v <= 0x000FU) {
		return ICL57_PR_IDLE;
	}
	if ((v & 0xFFF0U) == 0x0010U || v == 0x0020U || v == 0x0021U || v == 0x0040U) {
		return ICL57_PR_PENDING;
	}
	if ((v & 0xFFF0U) == 0x0100U) {
		return ICL57_PR_RUNNING;
	}
	if (v == 0x0200U) {
		return ICL57_PR_POSITIONING;
	}
	if ((v & 0xFFF0U) == 0x0200U) {
		return ICL57_PR_FOLLOW_ERR;
	}
	return ICL57_PR_UNKNOWN;
}

/*
 * Could a drive last trusted at `trusted`, whose last accepted target is
 * `target` (if `have_target`), now read `now`? A PR0 move only runs from where
 * the drive was toward its target, so the answer is "between the two", widened
 * by a quarter rev for overshoot and jitter. Elapsed time is deliberately not a
 * term: a power cycle is seconds of failed reads, and a time-based reach grows
 * over that outage until the reset it should catch looks reachable.
 */
static inline bool icl57_frame_plausible(int32_t trusted, int32_t target, bool have_target,
					 int32_t now, uint32_t counts_per_rev)
{
	int64_t lo = trusted, hi = trusted;
	int64_t slack = counts_per_rev / 4U;

	if (have_target) {
		lo = target < lo ? target : lo;
		hi = target > hi ? target : hi;
	}
	return (int64_t)now >= lo - slack && (int64_t)now <= hi + slack;
}

/*
 * Frame-loss check for one feedback sample against the last trusted one.
 *
 * An iCL57 that browns out or resets comes back reading position ~0 wherever it
 * stopped, and nothing on the drive records it: no uptime or boot counter in the
 * register map and no undervoltage alarm. A position the drive could not have
 * reached is that signature, but one such sample is only SUSPECT: 0x2203 has
 * been seen returning another transaction's payload, and a mispaired position
 * read would look identical once. A reset leaves a drive at rest in a new frame,
 * so the next sample decides: still within slack of the suspect is LOST, back
 * on the trusted track was a bad read. The caller must not adopt a SUSPECT
 * sample as its new baseline.
 *
 * It cannot see a reset whose ~0 falls inside the plausible band, i.e. a drive
 * within a quarter rev of 0 or moving across it. The mapped-poll sentinel has
 * no such blind spot; this check is what covers the direct-read path.
 */
enum icl57_frame {
	ICL57_FRAME_OK,
	ICL57_FRAME_SUSPECT,
	ICL57_FRAME_LOST,
};

struct icl57_frame_watch {
	int32_t pulses; /* the unconfirmed sample */
	bool pending;
};

static inline enum icl57_frame icl57_frame_check(struct icl57_frame_watch *w, int32_t trusted,
						 int32_t target, bool have_target, int32_t now,
						 uint32_t counts_per_rev)
{
	if (icl57_frame_plausible(trusted, target, have_target, now, counts_per_rev)) {
		w->pending = false;
		return ICL57_FRAME_OK;
	}
	if (w->pending && icl57_frame_plausible(w->pulses, 0, false, now, counts_per_rev)) {
		w->pending = false;
		return ICL57_FRAME_LOST;
	}
	*w = (struct icl57_frame_watch){.pulses = now, .pending = true};
	return ICL57_FRAME_SUSPECT;
}

/* How far the drive's command position may sit from the target and still count
 * as having taken it. Not a following-error allowance: the command frame lands
 * exactly on a completed PR target, so this only absorbs read-back noise. */
#define ICL57_RETRIGGER_TOL_PULSES 10

/*
 * Has the drive gone idle without taking the last accepted target?
 *
 * The drive can drop a PR0 trigger silently: the axis stops at an earlier
 * setpoint with no fault and no warning. A host that publishes only on change
 * (as ros2_control does) never repeats the final one. `cmd_pulses` is the
 * drive's command position (0x602A), not feedback: a loaded axis can rest
 * 685-1115 pulses off target, which no feedback tolerance can
 * tell apart from a missed trigger. `settle_ms` keeps a poll that lands just
 * after a write, before RUNNING rises, from re-sending it.
 */
static inline bool icl57_needs_retrigger(bool running, int32_t cmd_pulses, int32_t target,
					 uint32_t since_cmd_ms, uint32_t settle_ms)
{
	if (running || since_cmd_ms < settle_ms) {
		return false;
	}

	int64_t diff = (int64_t)target - (int64_t)cmd_pulses;

	return (diff < 0 ? -diff : diff) > ICL57_RETRIGGER_TOL_PULSES;
}

/*
 * May an aborted home re-pin PR0 to where the axis stopped?
 *
 * The re-pin is a PR0 write, and it is the only thing that retires the target
 * an e-stop leaves queued (the drive runs that target on its next enable). On
 * an ordinary abort the drive is still energised, as it always was. After a
 * STOP (the actuator went to FAULT mid-home) a PR0 write may only reach a
 * drive confirmed de-energised: there it cannot move now and replaces the
 * queued target with "stay here". A drive that still reads enabled, or does
 * not answer, gets nothing and the abort says a queued target may survive.
 */
static inline bool icl57_abort_may_repin(bool stopped, bool state_read, bool enabled)
{
	return !stopped || (state_read && !enabled);
}

/*
 * May a group setpoint write go to the next member? Asked before EACH write of
 * a group, with a fresh snapshot: live[i] is "READY or ACTIVE" and
 * referenced[i] the member's reference. A member stopped after the first write
 * must keep the rest from being sent, or a pair moves one motor of two.
 * -EPERM (a member is not energised and tracking) outranks -EACCES (one is
 * unreferenced): the stop is the cause to report.
 */
static inline int icl57_group_may_write(const bool *live, const bool *referenced, size_t n)
{
	int rc = 0;

	for (size_t i = 0; i < n; i++) {
		if (!live[i]) {
			return -EPERM;
		}
		if (!referenced[i]) {
			rc = -EACCES;
		}
	}
	return rc;
}

#endif /* ICL57_PROFILE_H_ */
