/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Homing for the iCL-RS driver: icl57_home() for applications, and the same
 * routine behind `icl57 home` and `icl57 unjam`. Output goes to the calling
 * shell when there is one and to the log otherwise (see ICL57_PRINT).
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/device.h>
#include <zephyr/drivers/actuator/icl57.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "icl57_internal.h"
#include "icl57_profile.h"

LOG_MODULE_DECLARE(actuator_icl57, CONFIG_ACTUATOR_LOG_LEVEL);

/* DI2's function word for this axis: ORG, or the limit home is at, N.C. unless
 * the node says the switch is normally open. */
uint16_t icl57_di2_func(const struct icl57_config *cfg)
{
	uint16_t func = !cfg->home_input_is_limit ? ICL57_DI_FUNC_ORG
			: cfg->home_dir_negative  ? ICL57_DI_FUNC_NL
						  : ICL57_DI_FUNC_PL;

	return cfg->home_switch_no ? func : (func | ICL57_DI_FUNC_NC);
}

int icl57_write_homing_mode(const struct shell *sh, const struct device *dev, uint8_t id)
{
	const struct icl57_config *cfg = dev->config;
	/* Bit0 is a rotation sense (0=CCW, 1=CW), not a position sign, so it comes
	 * from its own devicetree property rather than from home_dir_negative. Which
	 * sense travels toward home is a per-axis mechanical fact: measured CW on one
	 * screw axis and CCW on a ZLW-1040 belt axis, so one hardcoded mapping cannot
	 * serve both. See the binding for how to measure it. */
	/*
	 * Bit2 picks the METHOD: 1 = home-switch, 0 = limit-switch. Home-switch
	 * homing assumes the drive can run past the switch and reverse back off it;
	 * with the switch at the mechanical end of travel there is no room to do
	 * that, which is what the manual's limit-switch method is for.
	 */
	/*
	 * Bit1 (move-to) finishes the search by travelling to the Pr8.13/14 stop
	 * position, which `icl57 setup` writes as 0. That is right for the
	 * home-switch method: the manual warns a search decelerates past the switch
	 * so "the actual read position value may not be 0", and coming back to
	 * exactly 0 is what makes the frame check in icl57_home_finish() meaningful.
	 *
	 * It is WRONG for the limit-switch method, because a completed search
	 * redefines the frame so that 0 IS the limit. The drive therefore ends every
	 * limit home by driving itself back onto the switch -- after the jog that
	 * cleared it, and asynchronously, so it happens once `icl57 home` has already
	 * reported success. Measured on a belt axis: the jog left the axis at
	 * fb -6995 and it was found at -7397 moments later, 62 pulses PAST the datum,
	 * i.e. at exactly 0 in the drive's new frame.
	 *
	 * On the limit path the parking distance is ours to choose (see
	 * icl57_home_clear_limits), so leave the bit clear and let the search stop where
	 * it stops.
	 */
	uint16_t mode = 0;

	if (!cfg->home_input_is_limit) {
		mode |= ICL57_HOMING_MODE_SWITCH | ICL57_HOMING_MODE_MOVE_TO;
	}

	if (cfg->home_search_cw) {
		mode |= ICL57_HOMING_MODE_DIR_BIT0;
	}

	return icl57_write_verified(sh, id, ICL57_REG_HOMING_MODE, mode, "homing mode");
}

/* Poll interval while waiting on a move or a search. Slow on purpose: the bus is
 * shared by every motor, and a homing traverse takes seconds. */
#define ICL57_HOMING_POLL_MS 200

/* Added to a pre-move / nudge deadline, for the drive's accel/decel ramps. */
/*
 * Slack added to every computed homing deadline.
 *
 * Do NOT raise this to cure a pre-move that "did not arrive". It was raised
 * 5000 -> 15000 -> 30000 chasing exactly that, on the theory that the drive
 * ignores the commanded rate and crawls. It does not: with the deadline at
 * 15833 ms and again at 30833 ms, the two drives of a screw pair travelled
 * 24311 and 23880 pulses BOTH TIMES -- identical to the pulse. A move whose distance is
 * independent of the time allowed is finishing early, not running slow. The
 * real fault was ICL57_HOMING_ARRIVE_DIV being tighter than the axis's resting
 * error; see there.
 *
 * Generous costs nothing: icl57_wait_arrived() polls and returns the instant
 * the axis lands, so this bounds only the failure case -- which is why raising
 * it looks harmless and hides the real bug.
 *
 * Nor size it from a "measured rate" got by dividing the distance travelled by
 * the deadline that truncated it: that is circular. Raising the deadline
 * 15833 -> 45000 ms changed the distance a screw pair's pre-move travelled by
 * ZERO PULSES (14846 and 10228 on every run).
 *
 * So the rule: IF A MOVE'S DISTANCE DOES NOT CHANGE WHEN THE DEADLINE DOES, THE
 * DEADLINE IS NOT THE PROBLEM. Test that first -- one run at a different
 * deadline -- before deriving any rate from it. Still open: why each drive of
 * such a pair stopped after its own fixed distance (~14850, ~10228) with no
 * alarm and no PR warning. ~15000 pulses is close to the distance covered by the
 * acceleration ramp alone at accel 5556 and 180 rpm, which is a lead, not a
 * conclusion.
 */
#define ICL57_HOMING_MARGIN_MS 15000

/*
 * The search deadline gets its own, much tighter margin, because it is the only
 * thing that stops an axis whose home switch is never seen. Every millisecond
 * here is travel past the switch at homing_velocity_rpm: at 30 rpm on a 5 mm
 * lead, one second is 2.5 mm.
 */
#define ICL57_HOMING_SEARCH_MARGIN_MS 1500

/*
 * Flat allowance for the slow second-segment re-approach after the switch is
 * touched. Deliberately a fixed time, not a fraction of the bound converted at
 * homing_approach_rpm: that rate is very low on a belt axis (X approaches at
 * 2 rpm), so the derived figure ran to tens of seconds, and every one of those
 * seconds is search-speed travel if the switch is never seen. The drive's
 * backoff-and-creep is a short fixed manoeuvre, so bound it as one.
 */
#define ICL57_HOMING_REAPPROACH_MS 3000

/*
 * The rate the drive actually runs a homing search at, in rpm.
 *
 * NOT homing-velocity-rpm. `icl57 setup` writes that to 0x600F (Pr8.15, the
 * first-segment search velocity) and the drive ignores it: with 0x600F set to
 * 180, a screw axis searched ~106314 pulses in 14750 ms -- about 43 rpm,
 * a quarter of the commanded rate. Deliberately rounded DOWN to 40 so the
 * deadline it produces is longer than a real search rather than shorter.
 *
 * This is used ONLY to size the search deadline. The distance bound is enforced
 * separately and in pulses by icl57_wait_homed() -- do NOT read this constant as
 * the thing that stops a runaway search. Pr8.21 is clamped to 32767 on a long
 * screw axis and its unit is undocumented.
 */
#define ICL57_HOMING_SEARCH_RPM_MEASURED 40

/* How long the drive may take, after raising the "homing completed" bit, to
 * redefine its position frame and run the MOVE_TO travel to Pr8.13/14. */
#define ICL57_HOMING_SETTLE_MS 5000

/* How many clearance rounds icl57_home_clear_limits() may run before giving up.
 * One move of homing-limit-clearance-rad-milli clears an ordinary stop; needing
 * more means the search buried the axis deeper than that knob allows, and three
 * says the knob is simply too small. Bounded rather than open-ended because each
 * attempt is verified against Pr4.28 directly. */
#define ICL57_CLEAR_ATTEMPTS 3

/*
 * The arrival tolerance and icl57_arrive_tol() live in icl57_profile.h with the
 * rest of the pure homing maths, so the invariant they carry -- every move this
 * file waits on must be larger than the tolerance it is judged by -- is locked
 * in by the checks in tests/drivers/actuator/icl57 rather than a comment.
 */

/*
 * Post-homing frame check, in milli-radians. The drive was told to finish at
 * exactly 0 (Pr8.13/14), so its own feedback must agree.
 *
 * This guards against a command/feedback frame DESYNC, not against positioning
 * precision — the symptom it exists to catch is the 2.0 rad offset that
 * Pr8.02 <- 0x21 produced on all four drives, an order of magnitude outside
 * this threshold. It does not need to be tight, and must not be so tight that
 * ordinary settling trips it.
 *
 * Not 10, which no drive could ever satisfy: the MOVE_TO drives
 * the COMMAND frame to 0 and this reads the FEEDBACK frame, and the two sit
 * 65 pulses (41 mrad on the 10000-cpr drives) apart AT REST -- see the arrival
 * tolerance above for the same gap while moving. 10 mrad is 15.9 pulses, so a
 * good ORG home would fail with "the frames DISAGREE" and, because that path
 * returns before icl57_homing_succeeded(), leave host setpoints blocked. Only
 * the ORG path reaches here; the limit path returns above it, so a limit-homed
 * axis never exercises this.
 */
#define ICL57_HOMED_EPSILON_MILLI 100

/*
 * Dead-sensor floor in icl57_home_finish(): the search must have progressed
 * toward home by at least this fraction of the pre-move before a completion is
 * believed. The pre-move puts the whole pre-move distance between the axis and
 * its switch, so a real search's furthest sample lies near that distance, while
 * a permanently-asserted home input completes without moving at all: progress
 * ~0. A quarter sits far from both. It is not tighter because the furthest
 * sample lags the true extreme by up to one poll period at search speed.
 *
 * Travel, not elapsed time: a time floor was defeated by ONE dropped Modbus
 * reply, since a failed read in the completion poll costs the rx timeout plus
 * another poll period, which alone exceeded a 300 ms floor on a belt axis.
 */
#define ICL57_HOMING_MIN_TRAVEL_DIV 4

/*
 * The direction-seeding nudge, as a fraction of the pre-move.
 *
 * The drive's homing search follows the direction of the motion preceding the
 * trigger, and that OVERRIDES Pr8.10 bit0. Measured: triggered from a standstill
 * the search honoured bit0 and ran toward home; triggered straight after the
 * positive pre-move it ran positive instead, away from home, until it stalled
 * into the end stop -- with bit0 set and freshly written in both cases.
 *
 * So the pre-move is followed by a short move back the way the search must go.
 * A QUARTER, not a tenth. This move is waited on with icl57_wait_arrived(), so
 * it obeys the same rule as every other: it must be comfortably LARGER than the
 * tolerance it is judged by, or arrival cannot be told from never having set
 * off. A tenth fails that on both axis types -- a belt axis's 200 pulses against
 * a 65-pulse resting error yields a 50-pulse tolerance, a loaded screw axis's
 * 2500 against 1115 yields 625. At a quarter: the belt nudges 500 with a 125
 * tolerance, the screw 6250 with 1562. Both clear.
 *
 * A quarter still leaves the search 75% of the pre-move to cover, which stays
 * clear of the quarter-of-pre-move travel floor icl57_home_finish() uses to
 * detect a permanently-asserted home input -- but they are now equal at the
 * limit, so do not raise this without moving ICL57_HOMING_MIN_TRAVEL_DIV too.
 */
#define ICL57_HOMING_NUDGE_DIV 4

/*
 * The limit-clearance distance is homing-limit-clearance-rad-milli, its own
 * devicetree value rather than a fraction of the pre-move: it is sized by the
 * switch's release travel, which has nothing to do with how far the pre-move
 * runs. Tying the two meant every change to one silently moved the other.
 *
 * Unlike the pre-move and nudge waits, the check on this move compares two
 * FEEDBACK-frame readings, so the command/feedback frame gap does not apply and
 * a small distance is still verifiable.
 */

/*
 * How far the axis may travel AWAY from where its search started before the
 * search is called wrong-way, as a fraction of the pre-move. A quarter is
 * ~3.7 mm on one belt axis and ~1.3 mm on one screw axis — comfortably more than the settle after
 * the nudge plus the arrival tolerance it was waited on with, and far less than the travel bound
 * the deadline would otherwise let it cover.
 */
#define ICL57_HOMING_WRONGWAY_DIV 4

struct icl57_home_ctx {
	const struct device *dev;
	int32_t start_cmd;      /* command frame when the pre-move was issued   */
	int32_t preroll_to;     /* drive-frame target of the pre-move          */
	int32_t nudge_to;       /* target of the direction-seeding nudge       */
	int32_t preroll_pulses; /* its magnitude, for the minimum-travel floor */
	int32_t nudge_pulses;   /* the nudge's magnitude, for ITS arrival tol  */
	int32_t search_origin;  /* position when the search was triggered      */
	int32_t search_sign;    /* +1 if the search should increase position   */
	int64_t triggered_ms;   /* uptime when the search was triggered        */
	int64_t search_travel;  /* furthest progress toward home seen searching */
	uint32_t elapsed_ms;    /* search duration, for the success line         */
	int32_t datum_pulses;   /* feedback position the datum was taken at      */
	int32_t clear_to;       /* target of this round's clearance move         */
	int32_t clear_step;     /* its magnitude, for the wait's tolerance       */
	bool clearing;          /* a clearance move is in flight this round      */
	/* icl57_homing_begin() was called for this motor, so it needs a release
	 * (icl57_homing_end) and, if the whole home is aborted, an e-stop first --
	 * even if this specific motor never itself failed. Cleared once released. */
	bool claimed;
};

/*
 * A stop or a fault has landed on this axis mid-home. actuator_stop() latches
 * STOPPED before it touches the bus, and a drive fault reports FAULT, so these
 * two are the signals the driver can see without knowing about the app. Every
 * homing wait polls them, and no homing motion write is issued once set.
 */
static bool icl57_home_cancelled(const struct device *dev)
{
	enum actuator_state s = actuator_get_state(dev);

	return s == ACTUATOR_STATE_FAULT || s == ACTUATOR_STATE_STOPPED;
}

/*
 * Every homing motion write goes through one of these two, which make the stop
 * check and the write one bus hold. A stop reports FAULT before it takes the
 * bus, so it either lands before the check (nothing written) or queues behind
 * the write and de-energises the drive after it -- never a move queued on a
 * drive the stop has just released.
 */
static int icl57_home_pr0_move(const struct device *dev, int32_t pulses, uint16_t rpm)
{
	int rc;

	icl57_bus_lock();
	rc = icl57_home_cancelled(dev) ? -ECANCELED : icl57_pr0_move(dev, pulses, rpm);
	icl57_bus_unlock();
	return rc;
}

static int icl57_home_trigger_write(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int rc;

	icl57_bus_lock();
	rc = icl57_home_cancelled(dev) ? -ECANCELED
				       : icl57_mb_write_reg(d->iface, cfg->slave_id,
							    ICL57_REG_TRIGGER, ICL57_TRIG_HOMING);
	icl57_bus_unlock();
	return rc;
}

/*
 * Print the axis's state at a phase boundary: the drive-frame command and
 * feedback positions and their skew, the divergence from the REPORTED frame,
 * the drive's own following error, what Pr8.02 says it is doing, and DI2.
 *
 * Because "homing reported complete after only 0 pulses of search travel" is
 * not attributable on its own: it says the search did not travel, and then
 * blames a permanently-asserted input, but a drive has hit that message with
 * DI2 reading CLEAR beforehand and ASSERTED afterwards. Without this there is
 * no way to tell whether the pre-move moved, whether the nudge moved, or where
 * the axis actually was when the drive declared the search finished. Four lines per motor per home
 * is a cheap price for an attributable failure, and this only runs from an operator command.
 *
 * Deliberately best-effort: a dropped read prints "?" rather than failing the
 * home, because this is instrumentation and must not change the outcome it is
 * measuring.
 *
 * The skew is computed from two separate reads and can be off by one poll's
 * motion, so the drive's own following error (0x1010/11) sits beside it; and
 * Pr8.02 reports whether the drive took the trigger, which is otherwise only
 * inferable from whether the axis moved.
 */
static const char *const icl57_pr_status_name[] = {
	[ICL57_PR_IDLE] = "idle",
	[ICL57_PR_PENDING] = "pending",
	[ICL57_PR_RUNNING] = "running",
	[ICL57_PR_POSITIONING] = "positioning",
	[ICL57_PR_FOLLOW_ERR] = "FOLLOWING ERROR",
	[ICL57_PR_UNKNOWN] = "?",
};

static void icl57_home_trace(const struct shell *sh, const struct device *dev, const char *phase)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int32_t pos = 0, cmd = 0, ferr = 0, rep = 0;
	uint16_t di = 0, pr[2] = {0};
	char pos_s[12] = "?", cmd_s[12] = "?", skew_s[12] = "?", ferr_s[16] = "?";
	char div_s[14] = "?";
	char pr_s[32] = "?";
	const char *di_s = "?";
	bool pos_ok = icl57_read_pulses(dev, &pos) == 0;
	/* BOTH frames. Their divergence is invisible in either one alone, and it is
	 * what can send the pre-move into the switch -- a trace has shown a perfectly
	 * sensible feedback position throughout while the command frame, the one the
	 * drive actually moves in, was 157524 pulses away. */
	bool cmd_ok = icl57_read_command_pulses(dev, &cmd) == 0;

	if (pos_ok) {
		snprintk(pos_s, sizeof(pos_s), "%d", pos);
	}
	if (cmd_ok) {
		snprintk(cmd_s, sizeof(cmd_s), "%d", cmd);
	}
	if (pos_ok && cmd_ok) {
		snprintk(skew_s, sizeof(skew_s), "%d", pos - cmd);
	}
	/*
	 * Divergence between the REPORTED command frame and the drive's own, which
	 * silently breaks every absolute target. It should
	 * be constant within a run and zero on a freshly power-cycled drive; if it
	 * moves mid-home, the drive re-zeroed itself and the datum below it is not
	 * what it appears. See ICL57_REG_DRIVE_CMD_POS.
	 */
	if (cmd_ok && icl57_read_reported_pulses(dev, &rep) == 0) {
		snprintk(div_s, sizeof(div_s), "%d", rep - cmd);
	}
	/* The drive's own following error, beside the skew computed from the two
	 * separate reads above: they should agree to within one poll's motion, and
	 * where they do not, one of the frames is lying. Printed in encoder counts:
	 * the register holds (command - feedback) x 256, see ICL57_REG_FOLLOW_ERR. */
	if (icl57_read_u32(d->iface, cfg->slave_id, ICL57_REG_FOLLOW_ERR, &ferr) == 0) {
		snprintk(ferr_s, sizeof(ferr_s), "%d cnt", ferr / 256);
	}
	/* What the drive says it is doing (icl57_pr_status_decode). Qty 2 and
	 * un-swapped like every PR-block read. */
	if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_TRIGGER, pr, 2) == 0) {
		uint16_t v = pr[icl57_pr_block_index(0)];
		unsigned int path;

		snprintk(pr_s, sizeof(pr_s), "0x%04x %s", v,
			 icl57_pr_status_name[icl57_pr_status_decode(v, &path)]);
	}
	if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_DI_STATUS, &di, 1) == 0) {
		di_s = ((di & ICL57_DI_STATUS_DI2) != 0U) ? "ASSERTED" : "clear";
	}

	ICL57_PRINT(sh, "motor %u: [%s] fb %s, cmd %s (skew %s), div %s, ferr %s, PR %s, DI2 %s",
		    cfg->slave_id, phase, pos_s, cmd_s, skew_s, div_s, ferr_s, pr_s, di_s);
}

/* Wait for a move to reach `target` within `tol` pulses. Valid only for the
 * pre-move: it compares positions in one frame, and a completed homing search
 * redefines the frame.
 *
 * Reads the COMMAND frame (0x1012), the same frame the target was computed and
 * written in. This used to read the FEEDBACK frame and absorb the difference in
 * `tol`, on the assumption that the two differ only by the servo following error
 * (65-259 pulses, measured). That assumption fails after a frame re-zero: the
 * two have been measured 157524 pulses apart, which no tolerance can
 * cover. Comparing like with like removes the guess entirely.
 */
static int icl57_wait_arrived(const struct device *dev, int32_t target, int32_t tol,
			      uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline) {
		int32_t pos = 0;

		k_msleep(ICL57_HOMING_POLL_MS);
		if (icl57_home_cancelled(dev)) {
			return -ECANCELED;
		}

		if (icl57_read_command_pulses(dev, &pos) < 0) {
			continue; /* a dropped read is not a failed move */
		}

		int64_t err = (int64_t)pos - (int64_t)target;

		if ((err < 0 ? -err : err) <= (int64_t)tol) {
			return 0;
		}
	}
	return -ETIMEDOUT;
}

/*
 * Wait for the drive to stop running a PR path.
 *
 * icl57_wait_arrived() is a POSITION test with a tolerance of a hundredth of a
 * revolution, so it returns while the axis is still decelerating into the target
 * and the drive's PR path still owns it. A Pr8.02 homing trigger issued in that
 * window is silently ignored: the first bench run pre-moved correctly, triggered
 * into a still-running path, then sat out its whole 17 s deadline having never
 * searched at all, while a hand trigger from a standstill searched immediately.
 *
 * Bit2 is not read-and-clear (bits 4/5/6 are), so polling it here costs the
 * later completion check nothing.
 */
static int icl57_wait_idle(const struct device *dev, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline) {
		uint16_t state = 0;

		k_msleep(ICL57_HOMING_POLL_MS);
		if (icl57_home_cancelled(dev)) {
			return -ECANCELED;
		}

		if (icl57_read_motion_state(dev, &state) < 0) {
			continue; /* a dropped read is not a failed move */
		}
		if (state & ICL57_MOTION_FAULT) {
			return -EIO;
		}
		if (!(state & ICL57_MOTION_RUNNING)) {
			return 0;
		}
	}
	return -ETIMEDOUT;
}

/* icl57_wait_idle() for the abort path, best effort: no stop check, because
 * the abort has to settle a stopped axis too. */
static void icl57_wait_settled(const struct device *dev, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	uint16_t state = 0;

	while (k_uptime_get() < deadline) {
		k_msleep(ICL57_HOMING_POLL_MS);

		if (icl57_read_motion_state(dev, &state) == 0 &&
		    ((state & ICL57_MOTION_FAULT) || !(state & ICL57_MOTION_RUNNING))) {
			return;
		}
	}
}

/*
 * Wait for Pr8.02's search to report completion in 0x1003 bit6, aborting early
 * if the axis is travelling the WRONG WAY.
 *
 * The deadline alone is a poor guard: it only fires after the axis has covered
 * the whole travel bound, so a search that sets off away from the switch runs
 * the full distance before anything stops it. Every wrong-direction run on the
 * bench ended against a mechanical stop for exactly that reason. Position is
 * already being polled here for the state word, so checking its sign costs one
 * extra register read per poll and catches the failure in millimetres.
 *
 * Measured against `origin`, the position when the search was triggered, NOT
 * against the previous sample — because a legitimate search reverses near the
 * end. The drive decelerates past the switch and creeps back at
 * homing_approach_rpm, which is motion opposite to `sign`; by then the axis is
 * far past `origin` in the correct direction, so it cannot trip this. Only net
 * travel backwards from where the search began does.
 *
 * `slack` absorbs the settle after the direction-seeding nudge and the arrival
 * tolerance it was waited on with.
 */
static int icl57_wait_homed(const struct device *dev, uint32_t timeout_ms, int32_t origin,
			    int32_t sign, int32_t slack, int32_t budget, int64_t *furthest)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	bool started = false;

	*furthest = 0;

	while (k_uptime_get() < deadline) {
		uint16_t state = 0;

		k_msleep(ICL57_HOMING_POLL_MS);
		if (icl57_home_cancelled(dev)) {
			return -ECANCELED;
		}

		if (icl57_read_motion_state(dev, &state) < 0) {
			continue;
		}
		if (state & ICL57_MOTION_FAULT) {
			return -EIO;
		}
		/*
		 * Bit6 reads SET on an idle drive that has never homed -- measured on
		 * two drives straight after a power cycle, and it stays
		 * set across repeated reads rather than clearing as the manual's
		 * "returned to its initial state after it is read" implies. It behaves
		 * as "no search in progress", not as a completion latch.
		 *
		 * So accepting it directly is a RACE: this runs immediately after the
		 * trigger write, and a poll that lands before the drive has actually
		 * begun moving sees the resting bit and reports a home that never
		 * happened. The caller then jogs off and records a datum while the real
		 * search is still running underneath it, asynchronously -- which is how
		 * an axis ends up back on its switch after a "successful" home, and why
		 * results were irreproducible run to run.
		 *
		 * Wait for evidence the search started before believing it finished.
		 * Either signal will do: bit6 dropping, or bit2 (running) coming up.
		 */
		if ((state & ICL57_MOTION_HOMED) == 0 || (state & ICL57_MOTION_RUNNING) != 0) {
			started = true;
		}

		int32_t pos = 0;

		if (icl57_read_pulses(dev, &pos) < 0) {
			continue; /* a dropped read is not a wrong-way move */
		}

		/*
		 * Position is the other proof the search ran, and it is needed because
		 * the state word alone can miss the whole event.
		 *
		 * `icl57 home` triggers EVERY motor before waiting on any of them (the
		 * racking rule), so a later motor's search can start AND FINISH while an
		 * earlier one is still being waited on. By the time this loop first
		 * polls it, bit6 is back to its resting set state and bit2 is clear --
		 * indistinguishable, to the state word, from a search that never began.
		 * Measured: one drive of a pair moved 29593 pulses onto its switch
		 * between the nudge trace and the search-triggered trace, then sat out
		 * all 19500 ms of its deadline being reported as failed, having already
		 * homed.
		 *
		 * An axis that has travelled further than the wrong-way slack has
		 * unambiguously moved under the search, so take that as the start.
		 */
		int64_t travelled = icl57_search_progress(pos, origin, sign);

		if (travelled > (int64_t)slack) {
			started = true;
		}

		/* Record BEFORE the completion return, not only on the polls that fall
		 * through to the bookkeeping below. A search that completes between two
		 * polls is seen exactly once, on the poll that accepts it, and if that
		 * poll returns without recording anything the dead-sensor floor reads
		 * back 0 travel and condemns a good datum as a stuck switch -- seen
		 * after ~21400 pulses of real search. */
		if (travelled > *furthest) {
			*furthest = travelled;
		}

		if (started && (state & ICL57_MOTION_HOMED)) {
			return 0;
		}
		if (icl57_search_is_wrong_way(pos, origin, sign, slack)) {
			return -ERANGE;
		}

		/*
		 * Excessive travel TOWARD home: the switch was not seen.
		 *
		 * Without it the search is bounded by TIME alone. Pr8.21 is clamped to
		 * 32767 for any axis whose bound exceeds it (e.g. a screw axis at 100000
		 * pulses) in a unit the manual never states, and the deadline is sized
		 * at ICL57_HOMING_SEARCH_RPM_MEASURED rounded DOWN -- so every rpm the
		 * drive beats that by is travel nobody bounds. At a 19500 ms deadline a
		 * drive honouring its commanded 180 rpm covers ~585000
		 * pulses, about 230 mm against a 39 mm bound.
		 *
		 * Checked AFTER the completion return above, so a search that
		 * legitimately finds its switch at the far end still wins.
		 */
		if (travelled > (int64_t)budget) {
			return -EOVERFLOW;
		}
	}
	return -ETIMEDOUT;
}

/* Decode Pr8.29 into the operator's actual next action. */
static void icl57_report_pr_warning(const struct shell *sh, const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	uint16_t warn = 0;

	if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_PR_WARNING, &warn, 1) < 0) {
		ICL57_WARN(sh, "motor %u: PR warning unreadable", cfg->slave_id);
		return;
	}

	switch (warn) {
	case ICL57_PR_WARN_HOMING_OVERTRAVEL:
		ICL57_ERROR(sh,
			    "motor %u: search ran to its bound without seeing the switch "
			    "(0x601D=0x%04x). Check: sensor powered and COMI on 0 V; DI2 = "
			    "0x%04x; home-direction-negative and home-search-clockwise correct "
			    "for this axis",
			    cfg->slave_id, warn, icl57_di2_func(cfg));
		break;
	case ICL57_PR_WARN_LIMIT_DURING_HOMING:
		/* Only a fault when the axis homes against a mid-travel ORG switch. With
		 * home-input-is-limit the limit IS the home input, so the drive raising
		 * this means it hit the limit at the WRONG point of the routine -- the
		 * far-end limit, or the near one before the search proper began. */
		ICL57_ERROR(sh,
			    "motor %u: a LIMIT input asserted during the search "
			    "(0x601D=0x%04x). %s",
			    cfg->slave_id, warn,
			    cfg->home_input_is_limit
				    ? "This axis homes against a limit, so this means the "
				      "WRONG limit fired: check the search direction and "
				      "that only one end is wired to DI2"
				    : "This axis homes against a mid-travel ORG switch and "
				      "wires no limit, so a DI is assigned to POT/NOT by "
				      "mistake");
		break;
	default:
		ICL57_ERROR(sh, "motor %u: homing failed, 0x601D=0x%04x", cfg->slave_id, warn);
		break;
	}
}

/*
 * Is the power stage live? READY *or* ACTIVE.
 *
 * What these operations need is an energised drive, and the state machine has
 * two states for that: READY is "enabled, holding zero effort" and ACTIVE is
 * "tracking a setpoint" (actuator_types.h). Any `actuator set` leaves the axis
 * in ACTIVE and it stays there, so a guard written as `!= ACTUATOR_STATE_READY`
 * rejects a perfectly healthy axis for the rest of the session -- and then
 * blames the operator for not enabling it. Bench symptom: `icl57 unjam` and
 * `icl57 home` refusing with "enable the axis first" while `icl57 alarm` reads
 * 0x0000 and 0x1003 bit1 is set.
 */
static bool icl57_stage_is_live(const struct device *dev)
{
	enum actuator_state s = actuator_get_state(dev);

	return s == ACTUATOR_STATE_READY || s == ACTUATOR_STATE_ACTIVE;
}

/* Every motor of the set must be able to start before any of them moves: a
 * homing-group peer found unable after its partner's pre-move racks the pair. */
static int icl57_home_set_check(const struct shell *sh, const struct device *const *devs, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		const struct icl57_config *cfg = devs[i]->config;
		const struct icl57_data *d = devs[i]->data;

		if (!icl57_stage_is_live(devs[i])) {
			ICL57_ERROR(sh, "motor %u: enable the axis before homing (state %d)",
				    cfg->slave_id, (int)actuator_get_state(devs[i]));
			return -EPERM;
		}
		if (d->homing) {
			ICL57_ERROR(sh, "motor %u: already homing (another shell session?)",
				    cfg->slave_id);
			return -EBUSY;
		}
	}
	return 0;
}

/* Phase 1: claim the drive and move it off home. Returns 0 when the pre-move has
 * been ISSUED, not when it has arrived — every motor must be moving before any
 * of them is waited on, or a homing group racks. */
/* Traced at entry so the pre-move's effect is measurable as a delta. */
static int icl57_home_start(const struct shell *sh, struct icl57_home_ctx *c)
{
	const struct device *dev = c->dev;
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	if (!icl57_stage_is_live(dev)) {
		ICL57_ERROR(sh, "motor %u: enable the axis before homing (state %d)", cfg->slave_id,
			    (int)actuator_get_state(dev));
		return -EPERM;
	}

	icl57_home_trace(sh, dev, "start");

	/*
	 * The homing mode write does NOT belong here. It is issued in
	 * icl57_home_nudge(), immediately before the search trigger.
	 *
	 * Not here, "before the pre-move so a failure here costs no motion": that
	 * weighs only the cost of the write FAILING, not of it SUCCEEDING. With homing
	 * mode already configured, the drive can treat the pre-move's PR0 trigger as
	 * a homing command -- so instead of stepping 2001 pulses AWAY from the
	 * switch, a belt axis ran a search TOWARD it and straight past it, at a
	 * distance unrelated to the pre-move. Measured: start cmd -16, pre-move
	 * target +1985, axis finished at cmd -5989, i.e. 5973 pulses the wrong way;
	 * it moved, paused, then jumped onto the limit. Axes that survive this
	 * ordering are lucky, not correct.
	 */
	/* Refuse a drive someone else is already homing. An app can run several
	 * shell backends (e.g. serial and telnet), each on its own thread, so two concurrent
	 * `icl57 home` invocations against one motor are reachable — and a search
	 * takes seconds, so the window is wide. Without this, both would claim the
	 * drive and issue conflicting pre-move targets, and whichever finished first
	 * would clear d->homing and restart the feedback poller underneath the other
	 * search, which is precisely what lets the poller eat 0x1003's read-and-clear
	 * completion bit.
	 *
	 * Deliberately a plain check, not a lock: two threads could still pass it in
	 * the same instant, but that leaves the system no worse off than before while
	 * closing the realistic case of starting a home while one is already running.
	 * A mutex here would be the correct answer if this ever needs to be airtight.
	 */
	if (d->homing) {
		ICL57_ERROR(sh, "motor %u: already homing (another shell session?)", cfg->slave_id);
		return -EBUSY;
	}

	int begin_rc = actuator_report_reference_begin(dev);

	if (begin_rc < 0) {
		ICL57_ERROR(sh, "motor %u: cannot start referencing (state %d)", cfg->slave_id,
			    (int)actuator_get_state(dev));
		return begin_rc;
	}
	icl57_homing_begin(d);
	c->claimed = true;

	int32_t start = 0;

	/*
	 * COMMAND frame (0x1012), not feedback. The pre-move target below is handed
	 * to icl57_pr0_move(), which the drive executes as an ABSOLUTE position in
	 * its own command frame -- so the reference it is computed from has to be a
	 * position in that same frame, or "start - preroll" is not below where the
	 * drive thinks it is.
	 *
	 * Not the FEEDBACK frame (0x1014). The two normally sit within a following
	 * error of each other, which hides the difference; but a frame
	 * re-zero leaves them far apart -- 0x1012 read 0 while 0x1014 read 157524 --
	 * and then a pre-move DOWN is a command to travel 132524 pulses UP, into the
	 * very switch it is supposed to back away from. Measured doing exactly that:
	 * start 157524, target 132524, axis moved +1755 and asserted DI2.
	 */
	if (icl57_read_command_pulses(dev, &start) < 0) {
		ICL57_ERROR(sh, "motor %u: position unreadable, not homing", cfg->slave_id);
		return -EIO;
	}

	c->start_cmd = start;
	c->preroll_pulses =
		icl57_milli_rad_to_pulses(cfg->homing_preroll_milli, (uint32_t)cfg->counts_per_rev);
	if (c->preroll_pulses < 0) {
		c->preroll_pulses = -c->preroll_pulses;
	}

	/* Away from the switch. The search direction is a drive-frame property
	 * (Pr8.10 bit0), so the pre-move target is computed in the drive's frame too
	 * — no invert sign, no host-side zero. */
	c->preroll_to =
		cfg->home_dir_negative ? start + c->preroll_pulses : start - c->preroll_pulses;

	/* Back towards the switch, far enough to establish the direction the drive
	 * will then search in. See ICL57_HOMING_NUDGE_DIV. */
	c->nudge_pulses = c->preroll_pulses / ICL57_HOMING_NUDGE_DIV;

	c->nudge_to = cfg->home_dir_negative ? c->preroll_to - c->nudge_pulses
					     : c->preroll_to + c->nudge_pulses;

	/* The search runs opposite to the pre-move, by construction. */
	c->search_sign = cfg->home_dir_negative ? -1 : 1;

	/* A failed write here is AMBIGUOUS, not a guaranteed rejection -- the caller
	 * (icl57_home_abort) e-stops this motor along with the rest of the set
	 * rather than merely marking it out, in case the move went out anyway. */
	int rc = icl57_home_pr0_move(dev, c->preroll_to, cfg->homing_velocity_rpm);

	if (rc == -ECANCELED) {
		return rc;
	}
	if (rc < 0) {
		ICL57_ERROR(sh, "motor %u: pre-move write failed", cfg->slave_id);
		return -EIO;
	}

	ICL57_PRINT(sh, "motor %u: moving %d pulses off home before searching", cfg->slave_id,
		    c->preroll_pulses);
	return 0;
}

/* Phase 2b: seed the search direction. Issued for every motor before any is
 * waited on, for the same racking reason as the pre-move. */
static int icl57_home_nudge(const struct shell *sh, struct icl57_home_ctx *c)
{
	const struct icl57_config *cfg = c->dev->config;

	int rc = icl57_home_pr0_move(c->dev, c->nudge_to, cfg->homing_approach_rpm);

	if (rc == -ECANCELED) {
		return rc;
	}
	if (rc < 0) {
		ICL57_ERROR(sh, "motor %u: direction-seed write failed", cfg->slave_id);
		return -EIO;
	}

	return 0;
}

/* Phase 3: trigger the search. Issued for every motor before any is waited on. */
static int icl57_home_trigger(const struct shell *sh, struct icl57_home_ctx *c)
{
	const struct icl57_config *cfg = c->dev->config;

	if (icl57_home_cancelled(c->dev)) {
		return -ECANCELED;
	}

	/*
	 * Re-assert the search direction HERE, not in icl57_home_start(). Saved-and-
	 * power-cycled is not enough; see icl57_write_homing_mode() for why it has to
	 * be written at runtime at all.
	 *
	 * This is the last thing before the trigger and after every position move in
	 * the sequence has been issued AND waited on, so configuring homing mode can
	 * no longer change how a pre-move or nudge already in flight is interpreted.
	 * That is not hypothetical: with this write done first, a belt axis's 2001-pulse
	 * pre-move AWAY from the switch ran as a search TOWARD it and past it
	 * (start cmd -16, target +1985, finished at cmd -5989).
	 */
	if (icl57_write_homing_mode(sh, c->dev, cfg->slave_id) < 0) {
		ICL57_ERROR(sh, "motor %u: homing mode write failed; not homing", cfg->slave_id);
		return -EIO;
	}

	/*
	 * A failed write here is AMBIGUOUS, not a guaranteed rejection: this bus is
	 * documented to lose and mispair responses under load, so an error return
	 * does NOT mean the drive rejected the trigger -- it may be searching right
	 * now. Releasing this motor with no quick stop would leave that unmonitored
	 * search running underneath a feedback poller the caller then restarts, so
	 * the caller (icl57_home_abort) e-stops this motor along with the rest of
	 * the set on any failure rather than merely marking it out.
	 */
	int rc = icl57_home_trigger_write(c->dev);

	if (rc == -ECANCELED) {
		return rc;
	}
	if (rc < 0) {
		ICL57_ERROR(sh, "motor %u: homing trigger failed", cfg->slave_id);
		return -EIO;
	}

	/* Stamped only once the write is known to have gone out, so bus latency is
	 * excluded from the elapsed time icl57_home_finish() reports. */
	c->triggered_ms = k_uptime_get();

	/* Datum for the wrong-way check. The commanded nudge target is the fallback
	 * rather than a failure: it is where the axis was just waited to, so it is
	 * right to within the arrival tolerance, which the slack already covers. */
	if (icl57_read_pulses(c->dev, &c->search_origin) < 0) {
		c->search_origin = c->nudge_to;
	}
	return 0;
}

/*
 * Move an axis clear of its home limit with an ORDINARY PR0 MOVE.
 *
 * This replaces a ~150-line JOG routine that existed solely to work around a
 * restriction that does not exist. icl57_internal.h used to assert: "JOG is the
 * ONLY way to move an axis parked on an asserted limit: the drive refuses to
 * execute PR0 paths there (0x601D = 0x200) even away from the limit."
 *
 * FALSE, measured with the axis confirmed DI2-asserted
 * immediately after a homing search:
 *
 *   motor 3: still on its limit after the drive's park
 *   motor 3: PR0 clearance -> target 1617, now 1256, 0x601D=0x0000, DI2 clear
 *
 * A plain PR0 absolute move took the axis off the switch, with no warning
 * latched. The observations that suggested otherwise have better explanations:
 * "pre-move asked 2001 pulses, axis moved 1" is a pre-move computed in the wrong
 * frame, which lands on top of the current position; and an axis returning to
 * its switch after a jog is the post-jog re-pin, below.
 *
 * Using no JOG matters far beyond tidiness. A JOG is the ONLY operation that
 * moves the carriage WITHOUT advancing the command frame, and that divergence
 * causes the "homes, backs off, jumps straight back onto the limit" defect: the
 * drive treats the
 * command frame as authoritative, so the re-pin that followed every jog hauled
 * the axis back to where that stale frame believed it was -- the switch, at
 * tuning.rpm, ~35x the jog rate. PR0 advances the frame normally, so using it
 * here deletes the defect class at source instead of working around it.
 *
 * Stepped and re-checked rather than one long move: the distance needed is not a
 * constant, because how deep the search buries the axis is not a constant. A
 * single move sized for the worst case parks every ordinary home far out --
 * which is exactly the "it ends up miles from the switch" complaint. One
 * clearance step usually suffices; Pr4.28 says when it has
 * not.
 */
static int icl57_home_clear_limits(const struct shell *sh, struct icl57_home_ctx *ctx, size_t n)
{
	for (int attempt = 0; attempt < ICL57_CLEAR_ATTEMPTS; attempt++) {
		bool any = false;

		/*
		 * Round N, part 1: ISSUE a clearance move for every motor still on its
		 * switch, before waiting on ANY of them. Same rule as the pre-move and
		 * the nudge, and for the same reason -- a grouped pair lifts one rigid
		 * beam, so backing one side off while the other holds racks it.
		 */
		for (size_t i = 0; i < n; i++) {
			const struct device *dev = ctx[i].dev;
			const struct icl57_config *cfg = dev->config;
			struct icl57_data *d = dev->data;

			ctx[i].clearing = false;

			if (!cfg->home_input_is_limit) {
				continue;
			}

			uint16_t di = 0;

			if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_DI_STATUS, &di,
					       1) < 0) {
				ICL57_ERROR(
					sh,
					"motor %u: limit input unreadable; clearance UNVERIFIED",
					cfg->slave_id);
				return -EIO;
			}
			if ((di & ICL57_DI_STATUS_DI2) == 0U) {
				continue; /* already clear */
			}

			int32_t step = icl57_milli_rad_to_pulses(cfg->homing_clearance_milli,
								 (uint32_t)cfg->counts_per_rev);

			if (step < 0) {
				step = -step;
			}

			/* Away from home is the opposite of the search direction: a POSITION
			 * sign, so home-direction-negative's job -- not
			 * home-search-clockwise's, which is a rotation sense. */
			int32_t away = cfg->home_dir_negative ? 1 : -1;
			int32_t cmd = 0;

			if (icl57_read_command_pulses(dev, &cmd) < 0) {
				ICL57_ERROR(sh,
					    "motor %u: command position unreadable; cannot "
					    "clear the limit",
					    cfg->slave_id);
				return -EIO;
			}

			ctx[i].clear_to = cmd + away * step;
			ctx[i].clear_step = step;

			int rc =
				icl57_home_pr0_move(dev, ctx[i].clear_to, cfg->homing_approach_rpm);

			if (rc == -ECANCELED) {
				ICL57_ERROR(sh, "motor %u: stopped; not clearing the limit",
					    cfg->slave_id);
				return rc;
			}
			if (rc < 0) {
				ICL57_ERROR(sh, "motor %u: clearance move write failed",
					    cfg->slave_id);
				return -EIO;
			}

			ctx[i].clearing = true;
			any = true;
		}

		if (!any) {
			return 0; /* every motor is clear of its switch */
		}

		/* Round N, part 2: now wait. They are all already moving. */
		for (size_t i = 0; i < n; i++) {
			if (!ctx[i].clearing) {
				continue;
			}

			const struct icl57_config *cfg = ctx[i].dev->config;
			uint32_t timeout = icl57_homing_timeout_ms(
				(uint32_t)ctx[i].clear_step, (uint32_t)cfg->counts_per_rev,
				cfg->homing_approach_rpm, ICL57_HOMING_MARGIN_MS);

			/* Judged by the distance actually being travelled, like every other
			 * move this file waits on. */
			(void)icl57_wait_arrived(
				ctx[i].dev, ctx[i].clear_to,
				icl57_arrive_tol(cfg->counts_per_rev, ctx[i].clear_step), timeout);
			(void)icl57_wait_idle(ctx[i].dev, ICL57_HOMING_MARGIN_MS);
		}
	}

	/* Out of attempts: report every motor that is still detecting. */
	int rc = 0;

	for (size_t i = 0; i < n; i++) {
		const struct icl57_config *cfg = ctx[i].dev->config;
		struct icl57_data *d = ctx[i].dev->data;
		uint16_t di = 0;
		uint16_t warn = 0xFFFF;

		if (!cfg->home_input_is_limit) {
			continue;
		}
		if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_DI_STATUS, &di, 1) < 0 ||
		    (di & ICL57_DI_STATUS_DI2) != 0U) {
			(void)icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_PR_WARNING,
						 &warn, 1);
			ICL57_ERROR(sh,
				    "motor %u: STILL on its limit after %d clearance moves "
				    "(0x601D=0x%04x). Raise homing-limit-clearance-rad-milli",
				    cfg->slave_id, ICL57_CLEAR_ATTEMPTS, warn);
			rc = -EIO;
		}
	}
	return rc;
}

/*
 * `icl57 unjam` -- move an axis off its limit switch.
 *
 * An ordinary PR0 move away, repeated until Pr4.28 bit1 clears. It used to drive
 * a JOG, on the claim that PR0 is refused on an asserted limit; that claim is
 * false (see icl57_home_clear_limits), and the JOG was itself the cause of the
 * defect that made this command appear to work and then undo itself.
 */
int icl57_unjam(const struct shell *sh, const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;

	if (!icl57_stage_is_live(dev)) {
		ICL57_ERROR(sh,
			    "motor %u: enable the axis first (`actuator enable <name>`; state %d)",
			    cfg->slave_id, (int)actuator_get_state(dev));
		return -EPERM;
	}

	/* One-element set: the phased routine IS the single-motor routine when
	 * n is 1, so there is one implementation of "clear a limit". */
	struct icl57_home_ctx c = {.dev = dev};
	int rc = icl57_home_clear_limits(sh, &c, 1);

	if (rc == 0) {
		ICL57_PRINT(sh, "motor %u: clear of its limit", cfg->slave_id);
	}
	return rc;
}

/* Phase 4: wait for completion, then verify the result is trustworthy. */
static int icl57_home_finish(const struct shell *sh, struct icl57_home_ctx *c)
{
	const struct device *dev = c->dev;
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int32_t bound = icl57_milli_rad_to_pulses(cfg->homing_travel_limit_milli,
						  (uint32_t)cfg->counts_per_rev);

	if (bound < 0) {
		bound = -bound;
	}

	icl57_home_trace(sh, dev, "search triggered");

	/*
	 * Size the deadline at the rate the search ACTUALLY runs at.
	 *
	 * This used to bound the whole search at homing_approach_rpm, but that is
	 * only the slow final re-approach after the switch is touched -- the search
	 * itself runs at homing_velocity_rpm, three to ten times faster. The deadline
	 * therefore came out several times longer than any real search, and an axis
	 * that sailed past its switch kept going for tens of millimetres before the
	 * host stopped it. Measured on the bench: a 17 s deadline against a search
	 * that needs ~4 s, i.e. ~8 revolutions of travel past the switch.
	 *
	 * This deadline is the only thing that stops an axis whose switch is not
	 * seen, so it is a SAFETY bound, not a convenience. Keep it just above the
	 * worst legitimate search: the bound at search speed, plus a re-approach
	 * allowance, plus a small fixed margin for bus and poll latency.
	 */
	/*
	 * Sized at the rate the drive MEASURABLY searches at, not the rate it was
	 * told to. `icl57 setup` writes homing_velocity_rpm (180) to 0x600F, the
	 * first-segment search velocity, and the drive does not honour it: a screw
	 * axis's search covered ~106314 pulses in 14750 ms, i.e. ~43 rpm. Computed
	 * at 180 rpm the deadline came out 7833 ms -- barely half a real search --
	 * so a drive was stopped mid-search and only reached its switch afterwards,
	 * asynchronously, while the shell reported failure.
	 *
	 * WHY 0x600F is ignored is not established. Until it is, a deadline derived
	 * from the commanded rate is fiction; this constant is the measurement, with
	 * room underneath it.
	 */
	/*
	 * The search's budget is the bound PLUS THE PRE-MOVE, and both the deadline
	 * and the travel guard are sized from it. homing-travel-limit-rad-milli is
	 * the worst-case distance from where the axis is PARKED to its switch, but
	 * the search does not start there: icl57_home_start() has already driven it
	 * homing-preroll-rad-milli further away, and that must be re-covered before
	 * any progress is made against the original distance.
	 *
	 * Ignoring the term aborts good homes. A search measured ~106314 pulses
	 * against a 100000-pulse bound and SUCCEEDED, which reads as "the travel
	 * limit is smaller than the travel a search can need". It is not: 106314
	 * sits comfortably inside 100000 + 25000.
	 */
	int32_t budget = bound + c->preroll_pulses;
	uint32_t cpr = (uint32_t)cfg->counts_per_rev;
	uint32_t timeout = icl57_homing_timeout_ms((uint32_t)budget, cpr,
						   ICL57_HOMING_SEARCH_RPM_MEASURED, 0U) +
			   ICL57_HOMING_REAPPROACH_MS + ICL57_HOMING_SEARCH_MARGIN_MS;
	int rc = icl57_wait_homed(dev, timeout, c->search_origin, c->search_sign,
				  c->preroll_pulses / ICL57_HOMING_WRONGWAY_DIV, budget,
				  &c->search_travel);

	if (rc == -ECANCELED) {
		/* The stop already halted and de-energised the drive. */
		ICL57_ERROR(sh, "motor %u: stopped during the search", cfg->slave_id);
		return rc;
	}
	if (rc == -EOVERFLOW) {
		/* Same stop obligation as the other two paths, and the one the deadline
		 * was wrongly trusted to cover. */
		(void)icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_TRIGGER,
					 ICL57_TRIG_ESTOP);
		ICL57_ERROR(sh,
			    "motor %u: search travelled %d pulses toward home without the "
			    "switch, past its %d-pulse budget (%d bound + %d pre-move); axis "
			    "stopped. Sensor unpowered or COMI off 0 V, DI2 function wrong, or "
			    "homing-travel-limit-rad-milli shorter than the real distance",
			    cfg->slave_id, (int32_t)c->search_travel, budget, bound,
			    c->preroll_pulses);
		icl57_report_pr_warning(sh, dev);
		return rc;
	}
	if (rc == -ERANGE) {
		/* Same stop-the-axis obligation as the timeout path, and more urgent:
		 * this one fires while the axis is confirmed to be moving away from
		 * its switch, with nothing ahead of it but the end of travel. */
		(void)icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_TRIGGER,
					 ICL57_TRIG_ESTOP);
		ICL57_ERROR(sh,
			    "motor %u: search is running the WRONG WAY (away from home); "
			    "axis stopped. The drive inherits its search direction from the "
			    "motion before the trigger, so suspect the direction-seeding "
			    "nudge, then `home-direction-negative` against this axis's "
			    "actual sign (`icl57 invert` identifies it on a stopped axis)",
			    cfg->slave_id);
		return rc;
	}
	if (rc == -ETIMEDOUT) {
		/* The host deadline is the authoritative bound, so stopping the axis is
		 * this path's job. Pr8.21 is a belt-and-braces guard whose unit the
		 * manual never states, and the drive keeps searching regardless of our
		 * giving up on it. */
		(void)icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_TRIGGER,
					 ICL57_TRIG_ESTOP);
		ICL57_ERROR(sh, "motor %u: homing did not complete in %u ms; axis stopped",
			    cfg->slave_id, timeout);
		icl57_report_pr_warning(sh, dev);
		return rc;
	}
	if (rc < 0) {
		ICL57_ERROR(sh, "motor %u: drive faulted during homing", cfg->slave_id);
		return rc;
	}

	/*
	 * Minimum travel, sampled DURING the search.
	 *
	 * What this catches: the sensors are normally-closed, so an unpowered sensor,
	 * a broken wire, or DI2 left at the normally-open code all read as "at home",
	 * and the drive then reports homing COMPLETE without moving at all. The axis
	 * was deliberately driven off home first, so a real search has to cover that
	 * distance, and its furthest sample in icl57_wait_homed() lies near it.
	 *
	 * A position delta measured ACROSS the completion would compare two frames --
	 * a completed ORG search redefines the drive's frame so home becomes 0 -- so
	 * the samples are taken only while the search is still running, and it is the
	 * furthest of them that counts. Not an elapsed-time floor, which one dropped
	 * Modbus reply defeats: a failed read in the completion poll costs the rx
	 * timeout plus another poll period, and that alone exceeded the 300 ms floor
	 * a belt axis's pre-move produced, so a dead sensor passed as a home.
	 */
	uint32_t elapsed = (uint32_t)(k_uptime_get() - c->triggered_ms);
	int32_t least = c->preroll_pulses / ICL57_HOMING_MIN_TRAVEL_DIV;

	if (c->search_travel < least) {
		ICL57_ERROR(
			sh,
			"motor %u: homing reported complete after only %d pulses of search "
			"travel (expected at least %d). The home input looks PERMANENTLY "
			"ASSERTED — sensor unpowered or unplugged, wire broken, COMI on the "
			"wrong rail, or DI2 left at the N.O. code. The datum is NOT trustworthy",
			cfg->slave_id, (int32_t)c->search_travel, least);
		return -EIO;
	}

	/*
	 * Let the drive FINISH before reading the datum.
	 *
	 * 0x1003 bit6 goes up when the search succeeds, but the drive is not done at
	 * that instant: it still has to redefine its position frame and, with
	 * ICL57_HOMING_MODE_MOVE_TO set, travel to the Pr8.13/14 stop position.
	 * Reading position immediately caught it mid-sequence and reported the raw
	 * search distance -- a bench run announced "feedback reads -106974 pulses,
	 * not ~0" and failed a home that had in fact worked, with the very next read
	 * returning exactly 0.
	 *
	 * Best-effort: if the settle wait times out the position check below still
	 * runs and is still the authority on whether the datum is trustworthy.
	 */
	if (icl57_wait_idle(dev, ICL57_HOMING_SETTLE_MS) == -ECANCELED) {
		ICL57_ERROR(sh, "motor %u: stopped before the datum was taken", cfg->slave_id);
		return -ECANCELED;
	}

	int32_t pos = 0;

	if (icl57_read_pulses(dev, &pos) < 0) {
		ICL57_ERROR(sh, "motor %u: homed but position unreadable; datum unverified",
			    cfg->slave_id);
		return -EIO;
	}

	c->datum_pulses = pos;

	/*
	 * Two datum models, because the drive behaves differently per method.
	 *
	 * HOME-SWITCH (ORG): the drive redefines its own position frame at the
	 * switch, so feedback reads ~0 and that IS the datum -- verified below, and
	 * any host-side offset measured in the old frame must be dropped.
	 *
	 * LIMIT-SWITCH: it does NOT. Measured on a belt axis -- the
	 * drive reported homing complete with 0x1003 bit6 set, no fault, not running,
	 * and left position stable at -499.7 mm across repeated reads. So there is no
	 * drive-side datum to check; the axis is instead at a known PHYSICAL point,
	 * and the datum is established host-side by zeroing there.
	 *
	 * That host zero is lost on reboot, which is acceptable only because host
	 * setpoints should be refused until an axis has been homed (icl57_is_homed()) -- so a
	 * reboot forces a re-home rather than silently commanding against a datum
	 * that no longer exists. Deliberately NOT solved with the drive's own
	 * Pr8.02 <- 0x21 "set current position as origin": that leaves the position
	 * registers reporting in the old frame while absolute targets move to the
	 * new one, i.e. reported = commanded + offset.
	 */
	if (cfg->home_input_is_limit) {
		struct actuator_feedback fb;

		if (icl57_read_feedback(dev, &fb) < 0) {
			ICL57_ERROR(sh, "motor %u: homed but feedback unreadable; datum not set",
				    cfg->slave_id);
			return -EIO;
		}

		/* Accumulate, do not assign: the read is already in the offset frame. */
		d->zero_rad += fb.position;

		/*
		 * STOP HERE. The datum is taken ON the switch and the axis is left on
		 * it; clearing is a separate PHASE, run across the whole set at once by
		 * icl57_home_clear_limits().
		 *
		 * Clearing right here, inside this per-motor function, would make it
		 * sequential: one drive of a screw pair backing 4 mm off its switch while
		 * the other still holds on its own. Those two lift ONE RIGID BEAM, so that
		 * is a fight, and both sides absorb part of it. Measured on a home that
		 * otherwise succeeded -- same commanded 10000-pulse clearance, one drive
		 * travelled 4911 pulses and the other 2605, leaving the pair 1.83 mm
		 * apart against 0.91 mm of switch misalignment. The clearance DOUBLED the
		 * error it inherited.
		 *
		 * The rule that breaks is the one in icl57_home_run()'s header and
		 * already obeyed by the pre-move and the nudge: issue the move for every
		 * motor before waiting on any of them.
		 */
		c->elapsed_ms = elapsed;
		return 0;
	}

	int32_t eps =
		icl57_milli_rad_to_pulses(ICL57_HOMED_EPSILON_MILLI, (uint32_t)cfg->counts_per_rev);

	if (pos > eps || pos < -eps) {
		ICL57_ERROR(sh,
			    "motor %u: homing completed but feedback reads %d pulses, not ~0. "
			    "The command and feedback frames DISAGREE — do not command this "
			    "axis until that is understood",
			    cfg->slave_id, pos);
		return -EIO;
	}

	/* The datum now lives in the drive, so a host-side offset measured in the old
	 * frame must not stack on top of it. */
	d->zero_rad = 0.0f;
	c->elapsed_ms = elapsed;
	return 0;
}

/*
 * Phase 6: restore the profile velocity, then release.
 *
 * Separated from the verify above so that nothing in it runs until EVERY motor
 * in the set has been verified and cleared. This is a PR0 move, so on a shared
 * beam it is subject to the same ordering rule as the clearance.
 */
static int icl57_home_settle(const struct shell *sh, struct icl57_home_ctx *c)
{
	const struct device *dev = c->dev;
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int32_t pos = c->datum_pulses;

	/*
	 * Restore the profile velocity before releasing the axis.
	 *
	 * Every move homing makes is at a homing rate -- the pre-move at
	 * homing-velocity-rpm, the nudge at homing-approach-rpm -- and the PR0 block
	 * keeps whichever was written last. The streamed setpoint path writes its own
	 * velocity per move so it never notices, but the drive RESUMES the block it
	 * holds when it is next enabled, and that would be a 2 rpm creep nobody
	 * asked for. Pin the block at the settled position and the tuned profile
	 * velocity so the resting state is "hold here, at the axis's normal rate".
	 *
	 * Command frame, not `pos`: `pos` is feedback,
	 * and pinning PR0 there commands a move of the following error toward the
	 * switch -- on this path that is a datum the host has just been told is
	 * trustworthy, so the drift is silent as well as wrong.
	 */
	int32_t hold = pos;

	if (icl57_read_command_pulses(dev, &hold) < 0) {
		hold = pos;
	}

	/* A stop since the datum was verified: leave the axis to the stop, and do
	 * not mark it homed. */
	int rc = icl57_home_pr0_move(dev, hold, d->tuning.rpm);

	if (rc == -ECANCELED) {
		return rc;
	}
	if (rc < 0) {
		ICL57_WARN(sh, "motor %u: homed but could not restore the profile velocity",
			   cfg->slave_id);
	}

	/* Last, after every check above: this is what unblocks host setpoints, so it
	 * must not be set on a run that completed but failed verification. */
	icl57_homing_succeeded(dev);

	if (cfg->home_input_is_limit) {
		ICL57_PRINT(sh,
			    "motor %u: homed against its limit in %u ms; host zero set at the "
			    "limit (offset %.4f rad, lost on reboot -- re-home after one)",
			    cfg->slave_id, c->elapsed_ms, (double)d->zero_rad);
	} else {
		ICL57_PRINT(sh, "motor %u: homed in %u ms, feedback %d pulses, offset cleared",
			    cfg->slave_id, c->elapsed_ms, pos);
	}
	return 0;
}

static size_t icl57_home_select(const struct shell *sh, const struct device *dev,
				const struct device **out, size_t max)
{
	if (dev == NULL) {
		size_t n = MIN(icl57_dev_count(), max);

		for (size_t i = 0; i < n; i++) {
			out[i] = icl57_devs[i];
		}
		return n;
	}

	/*
	 * A dual-drive axis lifts one rigid beam, so homing this motor without its
	 * homing-group sibling(s) would rack it. Naming one member therefore homes
	 * the WHOLE group.
	 *
	 * Not a refusal saying "use `icl57 home all`": that protects the beam but
	 * forces every group home to drag in unrelated axes, so one axis's pre-move
	 * failure aborts the whole set and voids a good datum on a group that could
	 * have homed alone. Expanding is as safe -- the siblings still move
	 * together -- and it is what the operator asked for.
	 */
	const struct icl57_config *cfg = dev->config;

	if (cfg->homing_group == 0) {
		out[0] = dev;
		return 1;
	}

	size_t n = 0;

	for (size_t i = 0; i < icl57_dev_count() && n < max; i++) {
		const struct icl57_config *other = icl57_devs[i]->config;

		if (other->homing_group == cfg->homing_group) {
			out[n++] = icl57_devs[i];
		}
	}

	if (n == 0) {
		/* Unreachable: `dev` is its own group member. Belt and braces so a
		 * `max` of 0 cannot silently home nothing and report success. */
		ICL57_ERROR(sh, "motor %u: homing group %d selected no motors", cfg->slave_id,
			    cfg->homing_group);
		return 0;
	}

	if (n > 1) {
		ICL57_PRINT(sh,
			    "motor %u is in homing group %d: homing all %u of its motors "
			    "together, because they share one beam",
			    cfg->slave_id, cfg->homing_group, (unsigned int)n);
	}

	return n;
}

/*
 * Stop and release every motor in the set that was successfully claimed
 * (icl57_homing_begin() called), issuing an e-stop to each before releasing it.
 * Called whenever ANY motor in the set fails at ANY phase: a failed pre-move or
 * trigger write on this bus is AMBIGUOUS, not a guaranteed rejection (see
 * icl57_home_start / icl57_home_trigger), so the motor it belongs to may be
 * moving right now -- and every sibling that lifts the same rigid beam must
 * stop with it rather than keep searching alone.
 */
static void icl57_home_abort(const struct shell *sh, struct icl57_home_ctx *ctx, size_t n)
{
	/*
	 * Pass 1: STOP EVERYTHING FIRST. Nothing may come between one motor's
	 * e-stop and its sibling's.
	 *
	 * Not e-stop, settle and re-pin each motor before moving to the next: that
	 * puts an icl57_wait_idle() bounded at ICL57_HOMING_MARGIN_MS -- up to 15
	 * SECONDS -- between one sibling's stop and the next's. Siblings move one
	 * rigid load, so a sibling left searching for 15 s after its partner halted is the
	 * racking case this command is phased to avoid, arriving on the one path
	 * where something has already gone wrong.
	 */
	for (size_t i = 0; i < n; i++) {
		if (!ctx[i].claimed) {
			continue;
		}

		const struct icl57_config *cfg = ctx[i].dev->config;
		struct icl57_data *d = ctx[i].dev->data;

		(void)icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_TRIGGER,
					 ICL57_TRIG_ESTOP);
	}

	/* Pass 2: settle, retire the queued operation, release. Everything is
	 * already stopped, so this no longer delays anyone's stop. */
	for (size_t i = 0; i < n; i++) {
		if (!ctx[i].claimed) {
			continue;
		}

		/*
		 * The e-stop halts the motion but does NOT retire the operation behind
		 * it, and the drive resumes that operation the next time it is enabled.
		 * Observed on the bench as an axis moving on its own immediately after a
		 * clear_fault + enable, and then as every subsequent pre-move landing at
		 * roughly twice its start position: the leftover target and the new one
		 * both applied. Each aborted run corrupted the next, which is what made
		 * homing results irreproducible.
		 *
		 * So settle the axis and then pin PR0 to wherever it actually stopped,
		 * which replaces any queued target with a no-op. Best-effort throughout:
		 * this runs on the failure path, the bus may already be why we are here,
		 * and there is nothing better to do if it does not answer.
		 *
		 * Waiting also reads 0x1003, whose completion bits are read-and-clear, so
		 * it consumes a stale "homing completed" that would otherwise be waiting
		 * to satisfy the NEXT run's completion check without a search.
		 */
		icl57_wait_settled(ctx[i].dev, ICL57_HOMING_MARGIN_MS);
		/* A stop during settling also requires confirmed de-energisation. */
		bool stopped = icl57_home_cancelled(ctx[i].dev);
		int repin_rc = icl57_repin(ctx[i].dev, stopped);

		if (repin_rc < 0) {
			const struct icl57_config *acfg = ctx[i].dev->config;

			ICL57_ERROR(sh,
				    "motor %u: PR0 NOT re-pinned (%s), so a queued "
				    "target may run on its next enable",
				    acfg->slave_id,
				    repin_rc == -EBUSY ? "drive not confirmed de-energised"
						       : "re-pin write failed");
			icl57_repin_skipped(ctx[i].dev, repin_rc);
		}

		icl57_homing_end(ctx[i].dev);
		ctx[i].claimed = false;
		/* Never left ALIGNING. A stop has already latched STOPPED. */
		if (actuator_get_state(ctx[i].dev) == ACTUATOR_STATE_ALIGNING) {
			actuator_report_state(ctx[i].dev, ACTUATOR_SM_EVT_FAULT,
					      ACTUATOR_FAULT_REFERENCE);
		}
	}
}

/* Any motor in the set stopped (see icl57_home_cancelled()). */
static bool icl57_home_set_cancelled(const struct icl57_home_ctx *ctx, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (icl57_home_cancelled(ctx[i].dev)) {
			return true;
		}
	}
	return false;
}

/* Abort the set and return why: a stop wins over the failure it caused. */
static int icl57_home_fail(const struct shell *sh, struct icl57_home_ctx *ctx, size_t n, int rc)
{
	if (icl57_home_set_cancelled(ctx, n)) {
		ICL57_ERROR(sh, "homing stopped: no further motion issued");
		rc = -ECANCELED;
	}
	icl57_home_abort(sh, ctx, n);
	return rc;
}

/*
 * Reference one or more axes to their home switches.
 *
 * Phased across the whole set rather than run to completion per motor, because
 * a homing group moves ONE rigid load (e.g. a vertical screw pair lifting a
 * beam): moving one drive while its sibling holds position racks it. Every
 * motor is issued its pre-move before any is waited on, and
 * every search is triggered before any is waited on. Modbus RTU has no atomic
 * multi-slave write, so ~1 ms of skew at 115200 baud is the floor — negligible
 * against a search that runs for seconds. Triggering both sides and letting each
 * find its own switch is also what squares the pair.
 *
 * All-or-nothing: a failure at ANY phase, for ANY motor in the set, aborts the
 * whole set (icl57_home_abort) rather than letting the motors that are still
 * succeeding keep moving while a sibling sits stalled. `icl57 home <id>` also
 * refuses a motor with a homing-group sibling for the same reason
 * (icl57_home_select); only `all` can home a grouped pair.
 */
static int icl57_home_set(const struct shell *sh, const struct device *const *devs, size_t n)
{
	if (icl57_shell_iface < 0) {
		ICL57_ERROR(sh, "modbus bus not initialised");
		return -ENODEV;
	}
	if (n == 0) {
		return -ENODEV;
	}

	int check = icl57_home_set_check(sh, devs, n);

	if (check != 0) {
		return check;
	}

	struct icl57_home_ctx ctx[ICL57_MOTOR_COUNT];
	bool failed = false;

	/* Phase 1: claim every drive and start every pre-move. */
	for (size_t i = 0; i < n; i++) {
		ctx[i] = (struct icl57_home_ctx){.dev = devs[i]};
		if (icl57_home_start(sh, &ctx[i]) < 0) {
			failed = true;
		}
	}

	if (failed || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, -EIO);
	}

	/* Phase 2: wait for the pre-moves. They are all in flight already, so
	 * waiting on them one after another costs nothing. */
	for (size_t i = 0; i < n; i++) {
		const struct icl57_config *cfg = ctx[i].dev->config;
		int32_t tol = icl57_arrive_tol(cfg->counts_per_rev, ctx[i].preroll_pulses);
		uint32_t timeout = icl57_homing_timeout_ms(
			(uint32_t)ctx[i].preroll_pulses, (uint32_t)cfg->counts_per_rev,
			cfg->homing_velocity_rpm, ICL57_HOMING_MARGIN_MS);

		if (icl57_wait_arrived(ctx[i].dev, ctx[i].preroll_to, tol, timeout) < 0) {
			/*
			 * Report the target and the budget, not just the failure. This
			 * has fired on an axis that had actually travelled 77% of the way, i.e. it
			 * was moving at roughly a sixth of the commanded rpm -- which is a
			 * completely different fault from the "drive refuses PR0 while a limit is
			 * asserted" this message had always been read as. Without the numbers the
			 * two are indistinguishable.
			 */
			/*
			 * DISTANCE TRAVELLED, not just the target. Whether a move fell
			 * short because it is slow or because it STOPPED EARLY is the
			 * single most expensive question this path has ever asked, and
			 * both readings are indistinguishable from a target alone --
			 * ICL57_HOMING_MARGIN_MS carries a warning about it in prose that
			 * has now been walked into twice. One number settles it: run
			 * again at a different deadline and see whether this figure
			 * moves. If it does not, the deadline is not the problem.
			 */
			int32_t now = ctx[i].start_cmd;
			bool now_ok = icl57_read_command_pulses(ctx[i].dev, &now) >= 0;

			ICL57_ERROR(sh,
				    "motor %u: pre-move did not arrive within %u ms; target %d, "
				    "travelled %d of %d, not searching",
				    cfg->slave_id, timeout, ctx[i].preroll_to,
				    now_ok ? now - ctx[i].start_cmd : 0,
				    ctx[i].preroll_to - ctx[i].start_cmd);
			icl57_home_trace(sh, ctx[i].dev, "pre-move TIMED OUT");
			failed = true;
		} else if (icl57_wait_idle(ctx[i].dev, ICL57_HOMING_MARGIN_MS) < 0) {
			/* Arrived positionally but the PR path still owns the drive, so a
			 * trigger now would be swallowed. */
			ICL57_ERROR(sh, "motor %u: pre-move still running; not searching",
				    cfg->slave_id);
			failed = true;
		} else {
			icl57_home_trace(sh, ctx[i].dev, "pre-move done");
		}
	}

	if (failed || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, -EIO);
	}

	/* Phase 2b: nudge every axis back the way its search must go, so the drive
	 * inherits that direction rather than the pre-move's. */
	for (size_t i = 0; i < n; i++) {
		if (icl57_home_nudge(sh, &ctx[i]) < 0) {
			failed = true;
		}
	}

	if (failed || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, -EIO);
	}

	for (size_t i = 0; i < n; i++) {
		const struct icl57_config *cfg = ctx[i].dev->config;
		/* The NUDGE's own distance, not the pre-move's. A pre-move-derived
		 * tolerance is 500 pulses on a belt axis against a 200-pulse nudge, so a nudge
		 * the drive refused outright -- leaving the axis stationary -- passed
		 * both this check and the idle check below, and the search then ran
		 * with its direction unseeded. */
		int32_t tol = icl57_arrive_tol(cfg->counts_per_rev, ctx[i].nudge_pulses);
		uint32_t timeout = icl57_homing_timeout_ms(
			(uint32_t)ctx[i].nudge_pulses, (uint32_t)cfg->counts_per_rev,
			cfg->homing_approach_rpm, ICL57_HOMING_MARGIN_MS);

		if (icl57_wait_arrived(ctx[i].dev, ctx[i].nudge_to, tol, timeout) < 0) {
			ICL57_ERROR(sh, "motor %u: direction seed did not arrive; not searching",
				    cfg->slave_id);
			failed = true;
		} else if (icl57_wait_idle(ctx[i].dev, ICL57_HOMING_MARGIN_MS) < 0) {
			ICL57_ERROR(sh, "motor %u: direction seed still running; not searching",
				    cfg->slave_id);
			failed = true;
		} else {
			icl57_home_trace(sh, ctx[i].dev, "nudge done, about to search");
		}
	}

	if (failed || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, -EIO);
	}

	/* Phase 3: trigger every search. */
	for (size_t i = 0; i < n; i++) {
		if (icl57_home_trigger(sh, &ctx[i]) < 0) {
			failed = true;
		}
	}

	if (failed || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, -EIO);
	}

	/* Phase 4: wait for every search and take every datum. NO MOTION happens
	 * here -- an axis that homed is left sitting on its own switch, which is
	 * exactly where its datum was taken, and it stays there until the whole set
	 * has been verified. An e-stop first if any of them failed, so a sibling
	 * still mid-search does not keep going alone. */
	int result = 0;

	for (size_t i = 0; i < n; i++) {
		if (icl57_home_finish(sh, &ctx[i]) < 0) {
			result = -EIO;
		}
	}

	if (result < 0 || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, result < 0 ? result : -EIO);
	}

	/*
	 * Phase 5: clear every limit TOGETHER.
	 *
	 * Not inside phase 4, per motor, which makes it sequential -- see
	 * icl57_home_finish(): one sibling backs off its switch while the other
	 * holds, and the clearance doubles the misalignment it inherited.
	 */
	if (icl57_home_clear_limits(sh, ctx, n) < 0) {
		return icl57_home_fail(sh, ctx, n, -EIO);
	}

	/* Phase 6: restore each profile velocity and release. Last, because
	 * icl57_homing_succeeded() is what unblocks host setpoints and must not fire
	 * on a set where a sibling failed. */
	for (size_t i = 0; i < n; i++) {
		if (icl57_home_settle(sh, &ctx[i]) < 0) {
			result = -EIO;
		}
	}

	if (result < 0 || icl57_home_set_cancelled(ctx, n)) {
		return icl57_home_fail(sh, ctx, n, result < 0 ? result : -EIO);
	}

	for (size_t i = 0; i < n; i++) {
		icl57_homing_end(ctx[i].dev);
	}

	return result;
}

int icl57_home_run(const struct shell *sh, const struct device *dev)
{
	const struct device *devs[ICL57_MOTOR_COUNT];
	size_t n = icl57_home_select(sh, dev, devs, ICL57_MOTOR_COUNT);

	return icl57_home_set(sh, devs, n);
}

int icl57_reference_op(const struct device *const *devs, size_t n)
{
	size_t count = icl57_dev_count();
	int32_t groups[ICL57_MOTOR_COUNT];
	bool named[ICL57_MOTOR_COUNT] = {false};
	bool sel[ICL57_MOTOR_COUNT];
	const struct device *set[ICL57_MOTOR_COUNT];
	size_t m = 0;

	for (size_t i = 0; i < count; i++) {
		groups[i] = ((const struct icl57_config *)icl57_devs[i]->config)->homing_group;
		for (size_t j = 0; j < n; j++) {
			named[i] |= devs[j] == icl57_devs[i];
		}
	}
	(void)icl57_home_expand(groups, named, sel, count);
	for (size_t i = 0; i < count; i++) {
		if (sel[i]) {
			set[m++] = icl57_devs[i];
		}
	}
	return icl57_home_set(NULL, set, m);
}

int icl57_home(const struct device *dev)
{
	return dev != NULL ? actuator_reference(dev)
			   : icl57_reference_op(icl57_devs, icl57_dev_count());
}
