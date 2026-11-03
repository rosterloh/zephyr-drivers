/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * StepperOnline/Leadshine iCL-RS integrated closed-loop stepper actuator over
 * RS485 Modbus RTU. Streams absolute position setpoints via the PR0
 * immediate-trigger block; reads position/velocity/status back with FC03.
 */

#define DT_DRV_COMPAT rosterloh_actuator_icl57

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>
#include <zephyr/actuator/actuator.h>

#include "icl57_profile.h"
#include "icl57_internal.h"

LOG_MODULE_REGISTER(actuator_icl57, CONFIG_ACTUATOR_LOG_LEVEL);

/* The feedback poller runs a *blocking* Modbus client transaction, and the
 * client's RX completion is delivered on the system workqueue (modbus_core.c
 * hands the reply to server_work, which gives client_wait_sem). Polling from the
 * system workqueue would therefore self-deadlock: the poll blocks the one
 * system-workqueue thread that must run to complete the read, so every read
 * times out with -EIO ("Client wait-for-RX timeout"). Run the poller on a
 * dedicated workqueue so the system workqueue stays free to complete the read.
 * One queue for all motors is correct — transactions on the shared RS485 bus
 * must serialise anyway. */
#define ICL57_FEEDBACK_WQ_STACK_SIZE 4096
#define ICL57_FEEDBACK_WQ_PRIO       K_PRIO_PREEMPT(8)

static K_THREAD_STACK_DEFINE(icl57_feedback_wq_stack, ICL57_FEEDBACK_WQ_STACK_SIZE);
static struct k_work_q icl57_feedback_wq;
static atomic_t icl57_feedback_wq_started;

static void icl57_feedback_wq_start_once(void)
{
	if (atomic_test_and_set_bit(&icl57_feedback_wq_started, 0)) {
		return;
	}
	/* Name through the queue config, not k_thread_name_set() on
	 * k_work_q::thread -- that member is __deprecated in favour of thread_id. */
	static const struct k_work_queue_config wq_cfg = {.name = "icl57_fb"};

	k_work_queue_start(&icl57_feedback_wq, icl57_feedback_wq_stack,
			   K_THREAD_STACK_SIZEOF(icl57_feedback_wq_stack), ICL57_FEEDBACK_WQ_PRIO,
			   &wq_cfg);
}

/* Unit conversions. invert negates both command and feedback so a mirrored
 * motor (e.g. one of two driving a shared load) tracks the same joint target as
 * its partner. The sign comes from
 * the live data, not the config, so `icl57 invert` moves both paths together. */

static float icl57_sign(const struct icl57_data *d)
{
	return d->invert ? -1.0f : 1.0f;
}

static int32_t rad_to_pulses(const struct icl57_config *cfg, const struct icl57_data *d, float rad,
			     bool *clamped)
{
	/* Offset before clamping: the devicetree limits guard the drive's own
	 * travel, so re-zeroing must not drag the guard rails along with it. */
	rad = icl57_clamp_rad(rad + d->zero_rad, cfg->pos_min_milli, cfg->pos_max_milli, clamped);

	float pulses = icl57_sign(d) * rad * (float)cfg->counts_per_rev / (2.0f * (float)M_PI);

	return (int32_t)(pulses + (pulses >= 0.0f ? 0.5f : -0.5f));
}

/*
 * Drive-frame position (command pulses) to radians. The inverse of
 * rad_to_pulses' scaling, so a setpoint and the feedback that answers it are in
 * the same frame at the same resolution -- which is the whole point of reading
 * ICL57_REG_DRIVE_FB_POS rather than the reported encoder frame.
 */
static float pulses_to_rad(const struct icl57_config *cfg, const struct icl57_data *d,
			   int32_t pulses)
{
	return icl57_sign(d) * (float)pulses * (2.0f * (float)M_PI) / (float)cfg->counts_per_rev;
}

static float rpm_to_rad_s(const struct icl57_data *d, int32_t rpm)
{
	return icl57_sign(d) * (float)rpm * (2.0f * (float)M_PI / 60.0f);
}

/* Driver ops */

int icl57_read_motion_state(const struct device *dev, uint16_t *state)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	return icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_MOTION_STATE, state, 1);
}

/*
 * Claim the drive for a homing routine: stop the feedback poller so it cannot
 * consume the read-and-clear completion bit, and stop the setpoint path from
 * issuing commands the drive would reject mid-search.
 */
void icl57_homing_begin(struct icl57_data *d)
{
	d->homing = true;
	/* Dropped up front, not on failure: a run that aborts halfway leaves the
	 * axis somewhere the previous datum no longer describes, so the datum must
	 * not survive the attempt merely because the attempt did not finish. */
	d->homed = false;
	d->retriggers_left = 0;
	k_work_cancel_delayable(&d->feedback_work);
}

void icl57_homing_succeeded(const struct device *dev)
{
	struct icl57_data *d = dev->data;

	d->homed = true;
	d->repin_pending = false;
}

bool icl57_is_homed(const struct device *dev)
{
	struct icl57_data *d = dev->data;

	return d->homed;
}

int32_t icl57_homing_group(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;

	return cfg->homing_group;
}

struct icl57_tuning icl57_get_tuning(const struct device *dev)
{
	const struct icl57_data *d = dev->data;

	return d->tuning;
}

uint16_t icl57_max_rpm(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;

	return cfg->profile_velocity_max_rpm;
}

void icl57_datum_void(const struct device *dev)
{
	struct icl57_data *d = dev->data;

	d->homed = false;
}

bool icl57_is_answering(const struct device *dev)
{
	const struct icl57_data *d = dev->data;

	return !d->fb_backoff;
}

int icl57_repin(const struct device *dev, bool stopped)
{
	struct icl57_data *d = dev->data;
	uint16_t state = 0;
	int32_t here = 0;
	int rc = -EBUSY;

	/* A completed Stop must not land between the FAULT check and the PR0 write. */
	icl57_bus_lock();
	stopped |= actuator_get_state(dev) == ACTUATOR_STATE_FAULT;
	bool state_read = icl57_read_motion_state(dev, &state) == 0;

	if (icl57_abort_may_repin(stopped, state_read, (state & ICL57_MOTION_ENABLED) != 0U)) {
		/* The command frame avoids commanding the following error as a fresh move. */
		rc = 0;
		if (icl57_read_command_pulses(dev, &here) < 0 ||
		    icl57_pr0_move(dev, here, d->tuning.rpm) < 0) {
			rc = -EIO;
		}
	}
	d->repin_pending = rc != 0;
	icl57_bus_unlock();
	return rc;
}

void icl57_repin_skipped(const struct device *dev, int repin_rc)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	d->repin_pending = true;
	LOG_ERR("motor %u: aborted home left a queued PR0 target that the next enable "
		"may run (%s); re-enabling is refused until it is retired",
		cfg->slave_id,
		repin_rc == -EBUSY ? "drive not confirmed de-energised" : "re-pin write failed");
}

bool icl57_repin_pending(const struct device *dev)
{
	const struct icl57_data *d = dev->data;

	return d->repin_pending;
}

bool icl57_homing_active(const struct device *dev)
{
	const struct icl57_data *d = dev->data;

	return d->homing;
}

/*
 * Release the drive and restart the poller.
 *
 * Both motion references are dropped, whether or not the homing succeeded. A
 * successful search redefines the drive's position frame (home becomes 0), and a
 * failed one leaves the axis somewhere the setpoint path never commanded it to;
 * either way `last_pulses` and the cached feedback sample describe a frame that
 * no longer exists. Carrying them over would size the next streamed move against
 * a fiction — the same class of bug as the interrupted-move crawl.
 */
void icl57_homing_end(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	d->have_last = false;
	d->retriggers_left = 0;
	d->have_fb = false;
	d->frame_watch.pending = false; /* a suspect from the old frame means nothing now */
	d->homing = false;

	/* Restarted on abort as well as success: a failed home must not leave the
	 * axis without feedback. */
	if (d->common.state != ACTUATOR_STATE_DISABLED) {
		k_work_schedule_for_queue(&icl57_feedback_wq, &d->feedback_work,
					  K_MSEC(cfg->update_period_ms));
	}
}

/* Feedback position in command pulses, drive frame, read synchronously. The
 * cached `fb_pulses` cannot be used here: the poller that maintains it is
 * deliberately stopped for the duration of a homing routine. */
int icl57_read_pulses(const struct device *dev, int32_t *pulses)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int rc = icl57_read_u32(d->iface, cfg->slave_id, ICL57_REG_DRIVE_FB_POS, pulses);

	if (rc < 0) {
		return rc;
	}

	/*
	 * Feed the cache the poller would otherwise maintain.
	 *
	 * Homing polls position several times a second anyway — pre-move arrival,
	 * the nudge, the wrong-way check — and while it runs the poller is
	 * deliberately not running. Publishing what has already been read costs no
	 * extra bus traffic and keeps JointState alive through a search instead of
	 * the host seeing a hole.
	 *
	 * Velocity is deliberately not synthesised from successive samples: these
	 * reads are irregularly spaced, so a difference over them is noise. The
	 * stale rpm stands until the poller resumes.
	 */
	d->fb_pulses = *pulses;
	d->fb_uptime_ms = k_uptime_get_32(); /* 32-bit, matching the poller's stamp */
	d->have_fb = true;
	return 0;
}

/*
 * Command-frame position, in command pulses, drive frame, read synchronously.
 *
 * The counterpart of icl57_read_pulses() for the one job that frame matters
 * for: pinning PR0 at where the axis is. No counts conversion -- 0x1012 is
 * already at Pr0.00 resolution -- and deliberately no feedback-cache update,
 * because this is not a feedback sample and JointState must not be fed it.
 */
int icl57_read_command_pulses(const struct device *dev, int32_t *pulses)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	return icl57_read_u32(d->iface, cfg->slave_id, ICL57_REG_DRIVE_CMD_POS, pulses);
}

/* The REPORTED command frame (0x1012), for the homing trace alone: its
 * divergence from the drive's frame is the thing worth seeing, and nothing
 * should compute a target from it. See ICL57_REG_DRIVE_CMD_POS. */
int icl57_read_reported_pulses(const struct device *dev, int32_t *pulses)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	return icl57_read_u32(d->iface, cfg->slave_id, ICL57_REG_CMD_POSITION, pulses);
}

/*
 * A PR0 absolute move in raw drive-frame pulses at an explicit velocity.
 *
 * icl57_set_setpoint cannot be reused for the homing pre-move: it applies the
 * host-side zero, the direction sign and the travel clamp, and sizes its own
 * velocity from the command period. A homing pre-move is a fixed distance in the
 * drive's own frame at a fixed creep rate, and it deliberately runs while the
 * setpoint path is locked out.
 */
int icl57_pr0_move(const struct device *dev, int32_t pulses, uint16_t rpm)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	uint16_t block[ICL57_PR0_NREGS] = {
		ICL57_PR_MODE_ABS_POS | ICL57_PR_MODE_INS,
		(uint16_t)((uint32_t)pulses >> 16),
		(uint16_t)((uint32_t)pulses & 0xFFFF),
		rpm,
		d->tuning.accel,
		d->tuning.decel,
		0, /* delay */
		ICL57_PR_TRIGGER,
	};

	/*
	 * Every homing move goes through here -- pre-move, nudge, clearance, re-pin
	 * -- so this is the one place that records what each commanded. The PR0
	 * block is a poor check of a staged target: mode and trigger self-clear, and
	 * the target words read back word-swapped per pair.
	 *
	 * Silent at the default level; CONFIG_ACTUATOR_LOG_LEVEL_DBG turns it on.
	 */
	LOG_DBG("motor %u PR0 -> %d pulses at %u rpm (accel %u, decel %u)", cfg->slave_id, pulses,
		rpm, d->tuning.accel, d->tuning.decel);

	if (icl57_mb_write_regs(d->iface, cfg->slave_id, ICL57_REG_PR0_BASE, block,
				ICL57_PR0_NREGS) < 0) {
		return -EIO;
	}
	return 0;
}

/*
 * What each slot of the register-mapping block points at. Indexed by
 * enum icl57_map_slot, whose decoder (icl57_map_decode) is pure and host-tested;
 * the two must agree, which is why the enum owns the order.
 */
static const uint16_t icl57_map_targets[ICL57_MAP_N] = {
	[ICL57_MAP_STATE] = ICL57_REG_MOTION_STATE,
	[ICL57_MAP_FB_HI] = ICL57_REG_DRIVE_FB_POS,
	[ICL57_MAP_FB_LO] = ICL57_REG_DRIVE_FB_POS + 1U,
	[ICL57_MAP_VEL_LO] = ICL57_REG_FB_VEL_LO,
	[ICL57_MAP_DI] = ICL57_REG_DI_STATUS,
	[ICL57_MAP_SENTINEL] = ICL57_REG_PULSES_PER_REV,
};

/*
 * Program the mapping block and find where it reads back.
 *
 * Runtime only -- no 0x2244 EEPROM save -- so it is re-established at every
 * enable, and a drive that lost it to a power-cycle heals on the next enable
 * instead of depending on provisioning. Both documented read addresses are
 * probed, because the manual gives two (see ICL57_REG_MAP_VALUES_A/B), and the
 * block is trusted only once its sentinel slot reads back as counts-per-rev.
 * Failure is not an error: the poll keeps its direct reads, and says so once.
 */
static void icl57_map_setup(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	const uint16_t bases[] = {ICL57_REG_MAP_VALUES_A, ICL57_REG_MAP_VALUES_B};
	uint16_t targets[ICL57_MAP_N];
	uint16_t blk[ICL57_MAP_N] = {0};
	struct icl57_map_sample s;

	d->map_base = 0;
	memcpy(targets, icl57_map_targets, sizeof(targets));

	if (icl57_mb_write_regs(d->iface, cfg->slave_id, ICL57_REG_MAP_TARGETS, targets,
				ICL57_MAP_N) < 0) {
		LOG_WRN("motor %u: mapping write rejected; polling with direct reads",
			cfg->slave_id);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(bases); i++) {
		if (icl57_mb_read_regs(d->iface, cfg->slave_id, bases[i], blk, ICL57_MAP_N) == 0 &&
		    icl57_map_decode(blk, (uint16_t)cfg->counts_per_rev, &s)) {
			d->map_base = bases[i];
			LOG_INF("motor %u: mapped feedback poll active, values at 0x%04x",
				cfg->slave_id, d->map_base);
			return;
		}
	}

	LOG_WRN("motor %u: mapped block failed its sentinel at 0x%04x and 0x%04x (Pr0.00 "
		"must read %d there); polling with direct reads",
		cfg->slave_id, bases[0], bases[1], cfg->counts_per_rev);
}

static int icl57_enable(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	if (icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_FORCE_ENABLE, 1) < 0) {
		return -EIO;
	}

	/* Verify instead of trusting the write ACK. A faulted drive acknowledges
	 * the enable write and stays disabled, which used to leave the subsystem
	 * reporting READY for a drive whose output stage was off. */
	k_msleep(ICL57_STATE_SETTLE_MS);

	uint16_t state = 0;

	if (icl57_read_motion_state(dev, &state) < 0) {
		LOG_WRN("motor %u: enable written but motion state unreadable", cfg->slave_id);
	} else if ((state & ICL57_MOTION_ENABLED) == 0) {
		LOG_ERR("motor %u: enable REJECTED (motion state 0x%04x)%s", cfg->slave_id, state,
			(state & ICL57_MOTION_FAULT)
				? " - drive is faulted; relieve the cause then clear_fault"
				: "");
		return -EIO;
	}

	/* Enable moves Pr0.07 behind set_drive_mode()'s back, so keep its cache
	 * truthful or the next NORMAL is skipped as redundant when it is not. */
	d->coasting = false;

	if (IS_ENABLED(CONFIG_ACTUATOR_ICL57_MAPPED_POLL)) {
		icl57_map_setup(dev);
	}

	/* From enable, not from the first homing run: icl57_read_feedback() is the
	 * only path that turns a drive fault into an actuator fault, and an un-homed
	 * axis may be moved from the shell. The poller stays out of a homing search
	 * regardless -- icl57_homing_begin() cancels it and the handler bails on
	 * d->homing -- so starting it here costs the search nothing. */
	k_work_schedule_for_queue(&icl57_feedback_wq, &d->feedback_work,
				  K_MSEC(cfg->update_period_ms));
	return 0;
}

static int icl57_disable(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	/* Clear software enable. This releases torque only on a drive provisioned
	 * with DI1 at N.O. (`icl57 setup`); at the N.C. factory default the IO enable
	 * keeps the stage energised and the shaft stays locked. See
	 * ICL57_REG_DI1_FUNC. */
	(void)icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_FORCE_ENABLE, 0);
	d->coasting = true; /* keep the set_drive_mode() cache truthful, as in enable */
	d->retriggers_left = 0;
	k_work_cancel_delayable(&d->feedback_work);
	return 0;
}

int icl57_estop(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int rc = 0;

	/* Held so a poller re-trigger cannot land between the two writes below.
	 * Costs at most one poll's transactions of latency. */
	icl57_bus_lock();
	d->retriggers_left = 0;

	/* Drive-side estop first: it decelerates on its own ramp rather than
	 * coasting from whatever speed it was running. */
	if (icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_TRIGGER, ICL57_TRIG_ESTOP) < 0) {
		LOG_DBG("motor %u: e-stop trigger write failed", cfg->slave_id);
		rc = -EIO;
	}

	/* Then remove torque. Attempted even when the trigger failed: on a dead
	 * bus both fail and the caller reports the fault anyway, but a partly
	 * reachable drive must not be left energised because of an earlier error. */
	if (icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_FORCE_ENABLE, 0) < 0) {
		LOG_DBG("motor %u: e-stop de-energise write failed", cfg->slave_id);
		rc = -EIO;
	}

	/* Keep the set_drive_mode() cache truthful, as icl57_disable() does. */
	d->coasting = true;
	/* A re-enabled drive resumes an unfinished PR0 target, so the reset re-pins first.
	 * OR, not assign: homing_end clears have_last but not an abort's pending re-pin. */
	d->repin_pending |= d->have_last;
	k_work_cancel_delayable(&d->feedback_work);
	icl57_bus_unlock();

	/* `homed` is deliberately NOT cleared: the operator's chosen recovery is a
	 * reset that keeps the datum. The drive's position register does not follow
	 * an unpowered axis, so that trusts nobody moved the carriage -- the reset
	 * path says so out loud. */
	return rc;
}

/*
 * COAST releases the output stage; NORMAL re-energises it.
 *
 * BRAKE is not offered: it means "short the windings", which a step/direction
 * drive with its own current loop does not expose. An enabled iCL57 already
 * holds position, so the useful distinction here is only energised vs not.
 *
 * COAST clears the homed flag, which is the whole point of having it. The usual
 * reason to drop torque is so a human can move the axis by hand, and once they
 * have, the datum is a fiction -- the drive's own position register
 * does not follow an unpowered axis. Leaving `homed` set would let the next host
 * setpoint command an absolute move against a stale reference, at profile
 * velocity, from an unknown place. The axis must be re-homed, so say so here
 * rather than discovering it as a fast move in the wrong direction.
 */
static int icl57_set_drive_mode(const struct device *dev, enum actuator_drive_mode mode)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	uint16_t enable;

	bool want_coast;

	switch (mode) {
	case ACTUATOR_DRIVE_MODE_COAST:
		want_coast = true;
		enable = 0U;
		break;
	case ACTUATOR_DRIVE_MODE_NORMAL:
		want_coast = false;
		enable = 1U;
		break;
	default:
		return -ENOTSUP;
	}

	/*
	 * Nothing to do is not the same as a bus transaction that writes the value
	 * already there. The subsystem calls this with NORMAL before every setpoint,
	 * so an unconditional write costs a Modbus round trip per setpoint and turns
	 * any single lost reply into a driver fault. See icl57_data.coasting.
	 */
	if (want_coast == d->coasting) {
		return 0;
	}

	if (icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_FORCE_ENABLE, enable) < 0) {
		return -EIO;
	}

	d->coasting = want_coast;

	if (want_coast) {
		d->homed = false;
		LOG_WRN("motor %u: coasting; the datum is void until the axis is re-homed",
			cfg->slave_id);
	}

	return 0;
}

static int icl57_clear_fault(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	if (icl57_mb_write_reg(d->iface, cfg->slave_id, ICL57_REG_CONTROL_WORD,
			       ICL57_CTRL_RESET_ALARM) < 0) {
		return -EIO;
	}

	/* Confirm rather than report a success the hardware did not perform. The
	 * reset word DOES clear a stall fault on the iCL57-RS13 (verified on the
	 * bench: 0x1003 went 0x0001 -> 0x0032), but the manual warns some classes -
	 * over-current in particular - need the drive powered off instead, and a
	 * stall that has not actually been relieved will simply re-fault. */
	k_msleep(ICL57_STATE_SETTLE_MS);

	uint16_t state = 0;

	if (icl57_read_motion_state(dev, &state) < 0) {
		LOG_WRN("motor %u: alarm reset sent but motion state unreadable", cfg->slave_id);
		return 0;
	}

	if (state & ICL57_MOTION_FAULT) {
		LOG_ERR("motor %u: fault did NOT clear (motion state 0x%04x) - relieve the stall, "
			"or power-cycle the drive if the alarm class is not resettable",
			cfg->slave_id, state);
		return -EIO;
	}

	return 0;
}

static int icl57_set_setpoint_locked(const struct device *dev, enum actuator_mode mode, float value)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	if (mode != ACTUATOR_MODE_POSITION) {
		return -ENOTSUP;
	}

	/* The drive rejects commands during a search, and a streamed setpoint would
	 * fight the pre-move. -EBUSY rather than a silent drop: a caller streaming at
	 * 10 Hz should see that its commands are not being applied. */
	if (d->homing) {
		return -EBUSY;
	}

	bool clamped = false;
	int32_t pulses = rad_to_pulses(cfg, d, value, &clamped);

	/* Edge-triggered: a setpoint stream sits at 10 Hz per motor, so warning on
	 * every clamped command would flood the log. Report entering and leaving the
	 * clamp instead — silently truncating a commanded position is exactly the
	 * kind of failure that otherwise looks like a mechanical or host problem. */
	if (clamped != d->clamp_warned) {
		d->clamp_warned = clamped;
		if (clamped) {
			LOG_WRN("motor %u setpoint %.4f rad clamped to [%d, %d] milli-rad",
				cfg->slave_id, (double)value, cfg->pos_min_milli,
				cfg->pos_max_milli);
		} else {
			LOG_INF("motor %u setpoint back within limits", cfg->slave_id);
		}
	}

	/*
	 * A setpoint issued with the home-end limit asserted goes nowhere, and the
	 * drive says so nowhere: it refuses moves toward home and truncates moves
	 * away from it, with no alarm, no PR warning and the command and feedback
	 * frames agreeing -- declining, not failing. With a sourcing (24 V) switch
	 * on DI2, an UNPOWERED switch is indistinguishable from an asserted one
	 * (Pr4.28 polarity), so a switch supply that is off looks exactly like
	 * this. So the setpoint path reads the input and logs it.
	 *
	 * Edge-triggered, matching the clamp warning above: a stream sits at 10 Hz.
	 *
	 * ponytail: one extra Modbus read per setpoint. Accepted because the free
	 * alternatives -- anything that infers a stall from successive samples --
	 * cannot fire for a ONE-SHOT `actuator set`, which is the case that actually
	 * happened. Unlike the Pr0.07 write that `coasting` exists to avoid, a lost
	 * reply here is ignored rather than faulting the axis. If the bus becomes
	 * the constraint, gate it on moves toward home rather than dropping it.
	 * With CONFIG_ACTUATOR_ICL57_MAPPED_POLL the poll already carries Pr4.28
	 * and this read is skipped.
	 */
	uint16_t di = 0;
	bool have_di;

	if (d->map_base != 0U) {
		/* Stale by at most one poll period, which an edge-triggered warning
		 * can afford. */
		di = d->fb_di;
		have_di = d->have_fb;
	} else {
		have_di = icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_DI_STATUS, &di,
					     1) == 0;
	}

	if (have_di) {
		bool limited = (di & ICL57_DI_STATUS_DI2) != 0U;

		if (limited != d->limit_warned) {
			d->limit_warned = limited;
			if (limited) {
				LOG_WRN("motor %u: DI2 asserted - the drive will refuse moves "
					"toward home and truncate moves away from it. If the "
					"carriage is NOT on its limit, check the switch 24 V "
					"supply: unpowered reads the same as asserted",
					cfg->slave_id);
			} else {
				LOG_INF("motor %u limit input clear", cfg->slave_id);
			}
		}
	}

	/* Size the move to fill one command period instead of running at a fixed
	 * rate, so streamed setpoints produce continuous motion rather than an
	 * accel-limited dash followed by a dwell. `profile-velocity-rpm` is the
	 * ceiling, which is also what an unsizable first setpoint after boot gets.
	 *
	 * Measure the distance from the polled position when it is recent enough to
	 * still describe the axis, and only fall back to the last accepted setpoint
	 * when it is not. The fallback is the sole reference this used to have, and
	 * it is wrong exactly when a move is still in flight - see
	 * icl57_delta_pulses(). A stale sample is dropped rather than trusted, so a
	 * failed or backed-off read still cannot perturb commands. */
	uint32_t fb_age_ms = k_uptime_get_32() - d->fb_uptime_ms;
	bool fb_fresh = d->have_fb && fb_age_ms <= cfg->update_period_ms * ICL57_FB_FRESH_PERIODS;

	uint32_t delta_pulses =
		icl57_delta_pulses(pulses, d->last_pulses, d->have_last, d->fb_pulses, fb_fresh);

	uint16_t rpm = icl57_profile_rpm(delta_pulses, (uint32_t)cfg->counts_per_rev,
					 cfg->command_period_ms, d->tuning.rpm);

	/* One FC16 write of the PR0 block; the trigger word (Pr9.07, mapped to
	 * Pr8.02) fires PR0 immediately — the manual's position-streaming path.
	 * Shared with the homing pre-move, which needs the identical block at a
	 * fixed velocity, so the block itself is built in one place only. */
	/* The conversion is not observable from the drive afterwards: the PR0 target
	 * reads back in pulses (mode and trigger self-clear),
	 * never the radians it came from, and the command-position register reports
	 * where the drive ended up. So say it here, AFTER converting: a `value` that
	 * disagrees with its pulses is a register changed under us (unshared FPU).
	 * Silent at the default log level; CONFIG_ACTUATOR_LOG_LEVEL_DBG turns it on. */
	LOG_DBG("motor %u setpoint %.4f rad -> %d pulses at %u rpm (counts/rev %d, zero %.4f)",
		cfg->slave_id, (double)value, pulses, rpm, cfg->counts_per_rev,
		(double)d->zero_rad);

	if (icl57_pr0_move(dev, pulses, rpm) < 0) {
		return -EIO;
	}

	/* Only a setpoint the drive accepted becomes the reference for the next
	 * delta; otherwise a dropped write would understate the following move. */
	d->last_pulses = pulses;
	d->have_last = true;
#if defined(CONFIG_ACTUATOR_ICL57_BUS_STATS)
	/* Under 3 s only: a pause between moves is not a stall. */
	if (k_uptime_get_32() - d->cmd_uptime_ms < 3000U) {
		icl57_mb_stats_write_gap(k_uptime_get_32() - d->cmd_uptime_ms);
	}
#endif
	d->cmd_uptime_ms = k_uptime_get_32();
	d->retriggers_left = ICL57_RETRIGGER_MAX;

	return 0;
}

/* Under the bus lock (recursive, so a caller already holding it is fine) so the
 * poller's re-trigger can never send an older target after this one. */
static int icl57_set_setpoint(const struct device *dev, enum actuator_mode mode, float value)
{
	icl57_bus_lock();
	int rc = icl57_set_setpoint_locked(dev, mode, value);

	icl57_bus_unlock();
	return rc;
}

int icl57_read_feedback(const struct device *dev, struct actuator_feedback *out)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	int32_t pos = 0; /* drive frame, command pulses */
	int16_t vel = 0;
	uint16_t state = 0, alarm = 0;
	bool have_vel = false, have_state = false;
	bool mapped = false, frame_lost = false;

	if (d->map_base != 0U) {
		/* One transaction for the lot, through the mapping icl57_map_setup()
		 * programmed at enable. The sentinel slot proves the block is still
		 * that mapping. The block is RAM-only, so losing it means the drive
		 * reset, and its position -- 0 wherever it stopped -- is in a new frame:
		 * nothing in the block may be believed, and neither may the datum. */
		uint16_t blk[ICL57_MAP_N] = {0};
		struct icl57_map_sample s;

		if (icl57_mb_read_regs(d->iface, cfg->slave_id, d->map_base, blk, ICL57_MAP_N) <
		    0) {
			return -EIO;
		}
		if (icl57_map_decode(blk, (uint16_t)cfg->counts_per_rev, &s)) {
			pos = s.fb_pulses;
			vel = s.rpm;
			state = s.state;
			d->fb_di = s.di;
			have_vel = true;
			have_state = true;
			mapped = true;
		} else {
			LOG_ERR("motor %u: mapped block lost its sentinel: the drive has reset "
				"and its position frame is void; re-home. Direct reads until the "
				"next enable",
				cfg->slave_id);
			d->map_base = 0;
			frame_lost = true;
		}
	}
	if (!mapped) {
		/* Position is essential; the rest is best-effort telemetry. */
		if (icl57_read_u32(d->iface, cfg->slave_id, ICL57_REG_DRIVE_FB_POS, &pos) < 0) {
			return -EIO;
		}

		/* One 16-bit signed register, NOT the documented 0x1046/0x1047 pair: the
		 * drive does not latch the pair and a read straddling a sign change
		 * composes +/-65536 rpm. See ICL57_REG_FB_VEL_LO. */
		uint16_t rpm_raw = 0;

		have_vel = icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_FB_VEL_LO,
					      &rpm_raw, 1) == 0;
		vel = (int16_t)rpm_raw;

		have_state = icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_MOTION_STATE,
						&state, 1) == 0;
	}

	/* The direct-read path's reset detector, and a second opinion on the
	 * sentinel. Only a homed axis has a frame worth guarding; homing itself
	 * clears have_fb, so its legitimate re-zero is never compared. */
	uint32_t now_ms = k_uptime_get_32();
	bool suspect = false;

	if (d->homed && d->have_fb) {
		switch (icl57_frame_check(&d->frame_watch, d->fb_pulses, d->last_pulses,
					  d->have_last, pos, (uint32_t)cfg->counts_per_rev)) {
		case ICL57_FRAME_SUSPECT:
			if (frame_lost) {
				LOG_ERR("motor %u: and its position jumped %d -> %d, as a reset "
					"would",
					cfg->slave_id, d->fb_pulses, pos);
			} else {
				LOG_WRN("motor %u: position %d unreachable from %d; re-reading "
					"before trusting it",
					cfg->slave_id, pos, d->fb_pulses);
				suspect = true;
			}
			break;
		case ICL57_FRAME_LOST:
			LOG_ERR("motor %u: position jumped %d -> %d and stayed: the drive has "
				"reset and its position frame is void; re-home",
				cfg->slave_id, d->fb_pulses, pos);
			frame_lost = true;
			break;
		default:
			break;
		}
	}

	if (suspect) {
		/* Publish the last trusted position for this one poll rather than adopt
		 * a sample that may be a mispaired read. Not -EIO: the poller's backoff
		 * would delay the confirming read by seconds. */
		pos = d->fb_pulses;
	} else {
		/* Already command pulses in the drive's own frame, which is the frame
		 * the setpoint path computes targets in. No invert sign, no host-side
		 * zero. */
		d->fb_pulses = pos;
		d->fb_uptime_ms = now_ms;
		d->have_fb = true;
	}

	d->have_fb_state = have_state;
	d->fb_running = have_state && (state & ICL57_MOTION_RUNNING) != 0U;

	*out = (struct actuator_feedback){
		.valid_mask = ACTUATOR_FB_POSITION | (have_vel ? ACTUATOR_FB_VELOCITY : 0),
		.position = pulses_to_rad(cfg, d, pos) - d->zero_rad,
		.velocity = have_vel ? rpm_to_rad_s(d, vel) : 0.0f,
		.timestamp_us = (uint64_t)k_uptime_get() * 1000,
	};

	if (frame_lost) {
		out->fault_flags |= ICL57_FAULT_FRAME_LOST;
		d->homed = false;
	}

	if (have_state && (state & ICL57_MOTION_FAULT)) {
		uint32_t flags = 0;

		if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_ALARM, &alarm, 1) == 0 &&
		    alarm != ICL57_ALARM_NO_INFO) {
			if (alarm & ICL57_ALARM_OVERCURRENT) {
				flags |= ACTUATOR_FAULT_OVERCURRENT;
			}
			if (alarm & ICL57_ALARM_OVERVOLTAGE) {
				flags |= ACTUATOR_FAULT_OVERVOLTAGE;
			}
			if (alarm & ICL57_ALARM_STALL) {
				flags |= ACTUATOR_FAULT_OVERLOAD;
			}
			if (alarm & (ICL57_ALARM_ADC | ICL57_ALARM_EEPROM | ICL57_ALARM_AUTOTUNE)) {
				flags |= ACTUATOR_FAULT_DRIVER(0);
			}
		}

		/* The motion-state fault bit is authoritative ON ITS OWN; the alarm code
		 * only refines which flag. Never let an unreadable or unrecognised code
		 * downgrade a real fault to "no fault" - that is exactly what happened
		 * on the bench, where a stall left the drive faulted and disabled while
		 * the subsystem still reported READY with no flags. The code cannot be
		 * relied on: a real stall produced a 7-flash error absent from the
		 * manual's table, and 0x2203 has been seen returning another
		 * transaction's payload. */
		out->fault_flags |= (flags != 0U) ? flags : ACTUATOR_FAULT_DRIVER(1);

		/* A fault invalidates the datum. A stall is the common case and it means
		 * the carriage stopped while the commanded profile ran on, so the
		 * position frame no longer describes where the axis physically is. Home
		 * again before trusting an absolute setpoint. */
		d->homed = false;
	}
	return 0;
}

static DEVICE_API(actuator, icl57_api) = {
	.enable = icl57_enable,
	.disable = icl57_disable,
	.clear_fault = icl57_clear_fault,
	.set_drive_mode = icl57_set_drive_mode,
	.set_setpoint = icl57_set_setpoint,
	.read_feedback = icl57_read_feedback,
	/* No group ops: Modbus RTU has no atomic multi-slave write, so the
	 * subsystem falls back to sequential per-motor set_position. */
};

/*
 * One-shot audit of the drive's alarm detection mask, run from the feedback
 * workqueue after the first successful poll. It cannot be done in icl57_init:
 * that runs at POST_KERNEL and performs no bus transaction by design, because a
 * blocking Modbus client read needs its RX completion delivered on a workqueue
 * that is not necessarily scheduled during device init (the same hazard that
 * made feedback reads fail with -EIO from the system workqueue).
 *
 * Reports only - arming a drive is an explicit operator action (`icl57 setup`),
 * not something the firmware does behind the operator's back on every boot.
 */
static void icl57_check_alarm_detect(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	uint16_t mask = 0;

	/* One attempt per boot either way: this is a diagnostic, and retrying a
	 * failing register read every poll would just add bus traffic. */
	d->alarm_detect_checked = true;

	if (icl57_mb_read_regs(d->iface, cfg->slave_id, ICL57_REG_ALARM_DETECT, &mask, 1) < 0) {
		LOG_DBG("motor %u: could not read alarm detection mask", cfg->slave_id);
		return;
	}

	uint16_t missing = (uint16_t)(ICL57_ALARM_DETECT_MOTION & ~mask);

	if (missing != 0) {
		LOG_WRN("motor %u: alarm detection 0x%04x is missing 0x%04x - stall/overload "
			"will NOT fault. Run 'icl57 setup %u'",
			cfg->slave_id, mask, missing, cfg->slave_id);
	}
}

/*
 * One-shot identity audit, same slot and same constraints as the alarm-mask
 * check above. Pr0.00 is the pulses-per-revolution every setpoint is scaled by
 * and Pr7.01 the encoder resolution every feedback sample is scaled by. Both
 * are otherwise devicetree assumptions the drive is never asked to confirm, and
 * a mismatch mis-scales every move without any other symptom. The
 * rest is provenance: peak current, the following-error threshold behind the
 * bit2 detection `icl57 setup` arms, and the firmware words, so a bench finding
 * that contradicts the manual (0x1003 bit6 did) can be tied to a revision.
 *
 * Pr7.01 is logged, NOT checked. iCL57-RS13 drives on fw 0x00ca/0x422a read
 * 4000 there while 0x1014 measurably advances
 * 65536 counts per revolution (the 6.5589 ratio against Pr0.00 recorded in
 * icl57_internal.h), so whatever Pr7.01 describes, it is not the feedback
 * resolution this driver divides by, and a warning against it is a false alarm.
 */
static void icl57_check_drive_params(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	uint16_t ppr = 0, ferr = 0, amps = 0, ver = 0, fw = 0, enc = 0;
	const struct {
		uint16_t reg;
		uint16_t *out;
	} regs[] = {
		{ICL57_REG_PULSES_PER_REV, &ppr}, {ICL57_REG_MAX_FOLLOW_ERR, &ferr},
		{ICL57_REG_PEAK_CURRENT, &amps},  {ICL57_REG_VERSION, &ver},
		{ICL57_REG_FIRMWARE, &fw},        {ICL57_REG_ENCODER_RES, &enc},
	};

	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		if (icl57_mb_read_regs(d->iface, cfg->slave_id, regs[i].reg, regs[i].out, 1) < 0) {
			LOG_WRN("motor %u: identity audit skipped, reg 0x%04x unreadable",
				cfg->slave_id, regs[i].reg);
			return;
		}
	}

	LOG_INF("motor %u: fw 0x%04x/0x%04x, %u pulses/rev, encoder %u, peak %u.%u A, max "
		"following error %u pulses",
		cfg->slave_id, ver, fw, ppr, enc, amps / 10U, amps % 10U, ferr);

	if ((uint32_t)ppr != (uint32_t)cfg->counts_per_rev) {
		LOG_ERR("motor %u: drive Pr0.00 is %u pulses/rev but counts-per-rev is %d -- "
			"EVERY setpoint is mis-scaled. Fix one of them before moving the axis",
			cfg->slave_id, ppr, cfg->counts_per_rev);
	}
}

/*
 * Re-send the last accepted target if the drive went idle without taking it.
 * Bus lock held. See icl57_needs_retrigger() for why this exists.
 */
static void icl57_retrigger_if_dropped(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	if (d->retriggers_left == 0U || !d->have_last || !d->have_fb_state || d->fb_running ||
	    d->coasting || d->homing || d->common.state == ACTUATOR_STATE_FAULT ||
	    d->common.state == ACTUATOR_STATE_DISABLED) {
		return;
	}

	uint32_t now_ms = k_uptime_get_32();
	int32_t cmd;

	if (icl57_read_command_pulses(dev, &cmd) < 0) {
		return;
	}
	if (!icl57_needs_retrigger(false, cmd, d->last_pulses, now_ms - d->cmd_uptime_ms,
				   cfg->update_period_ms)) {
		/* Taken, or too soon to tell; only the former ends the watch. */
		if (now_ms - d->cmd_uptime_ms >= cfg->update_period_ms) {
			d->retriggers_left = 0;
		}
		return;
	}

	d->retriggers_left--;
	uint32_t delta = icl57_delta_pulses(d->last_pulses, 0, false, d->fb_pulses, d->have_fb);
	uint16_t rpm = icl57_profile_rpm(delta, (uint32_t)cfg->counts_per_rev,
					 cfg->command_period_ms, d->tuning.rpm);

	LOG_INF("motor %u idle with command %d, target %d: re-triggering (%u left)", cfg->slave_id,
		cmd, d->last_pulses, d->retriggers_left);
	if (icl57_pr0_move(dev, d->last_pulses, rpm) == 0) {
		d->cmd_uptime_ms = now_ms;
	}
}

static void icl57_feedback_work_handler(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct icl57_data *d = CONTAINER_OF(dw, struct icl57_data, feedback_work);

	/* Homing owns the drive and needs 0x1003's read-and-clear completion bit for
	 * itself. Cancelling this work in icl57_homing_begin() is not enough:
	 * k_work_cancel_delayable() neither stops a handler that is already running nor
	 * prevents it from rescheduling (see its API notes), and this handler spends a
	 * large fraction of every period inside blocking Modbus reads. A poll in flight
	 * when homing began would otherwise re-arm itself and keep eating the homing
	 * completion bit for the whole search, making a SUCCESSFUL home look like a
	 * timeout. Deliberately no reschedule here — icl57_homing_end() restarts us. */
	if (d->homing) {
		return;
	}

	const struct device *dev = d->self;
	const struct icl57_config *cfg = dev->config;
	struct actuator_feedback fb;

	/* Released before reporting: the callbacks can stop motors, which takes
	 * app locks that are held while this one is waited for. */
	icl57_bus_lock();
	int rc = icl57_read_feedback(dev, &fb);

	if (rc == 0) {
		icl57_retrigger_if_dropped(dev);
	}
	icl57_bus_unlock();
	if (rc == 0) {
		if (d->fb_backoff) {
			LOG_INF("motor %u feedback recovered", cfg->slave_id);
			d->fb_backoff = false;
		}
		actuator_report_feedback(dev, &fb);
		if (!d->alarm_detect_checked) {
			icl57_check_alarm_detect(dev);
			icl57_check_drive_params(dev);
		}
	} else if (!d->fb_backoff) {
		LOG_WRN("motor %u feedback read failed (%d); backing off", cfg->slave_id, rc);
		d->fb_backoff = true;
	}
	if (d->common.state != ACTUATOR_STATE_DISABLED) {
		/* Poll a non-responding motor slowly so one absent/faulty drive does
		 * not saturate the shared RS485 bus (or the log) at the full rate. */
		uint32_t delay = rc == 0 ? cfg->update_period_ms
					 : MAX(cfg->update_period_ms * 10, ICL57_FB_BACKOFF_MS);

		k_work_schedule_for_queue(&icl57_feedback_wq, &d->feedback_work, K_MSEC(delay));
	}
}

static int icl57_init(const struct device *dev)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	d->iface = modbus_iface_get_by_name(cfg->iface_name);
	if (d->iface < 0) {
		LOG_ERR("modbus iface '%s' not found", cfg->iface_name);
		return -ENODEV;
	}
	if (icl57_client_init_once(d->iface, cfg) < 0) {
		return -EIO;
	}

	d->self = dev;
	d->tuning = (struct icl57_tuning){
		.rpm = cfg->profile_velocity_rpm,
		.accel = cfg->accel,
		.decel = cfg->decel,
	};
	d->invert = cfg->invert;
	d->common.state = ACTUATOR_STATE_DISABLED;
	d->common.caps = cfg->caps;
	d->cb_storage.pool = d->cb_pool;
	d->cb_storage.pool_n = ICL57_CB_POOL;
	atomic_set(&d->cb_storage.used, 0);
	sys_slist_init(&d->cb_storage.list);
	k_work_init_delayable(&d->feedback_work, icl57_feedback_work_handler);
	icl57_feedback_wq_start_once();
	return 0;
}

#define ICL57_DEFINE(inst)                                                                         \
	static struct icl57_data icl57_data_##inst;                                                \
	static const struct icl57_config icl57_config_##inst = {                                   \
		.cb_offsets =                                                                      \
			{                                                                          \
				.storage_offset = offsetof(struct icl57_data, cb_storage),         \
			},                                                                         \
		.iface_name = DEVICE_DT_NAME(DT_INST_PARENT(inst)),                                \
		.baud = DT_PROP(DT_PARENT(DT_INST_PARENT(inst)), current_speed),                   \
		.rx_timeout_us = DT_INST_PROP(inst, rx_timeout_us),                                \
		.slave_id = DT_INST_REG_ADDR(inst),                                                \
		.caps = ICL57_CAPS,                                                                \
		.update_period_ms = DT_INST_PROP(inst, update_period_ms),                          \
		.command_period_ms = DT_INST_PROP(inst, command_period_ms),                        \
		.counts_per_rev = DT_INST_PROP(inst, counts_per_rev),                              \
		.pos_min_milli = DT_INST_PROP_OR(inst, position_min_rad_milli, INT32_MIN),         \
		.pos_max_milli = DT_INST_PROP_OR(inst, position_max_rad_milli, INT32_MAX),         \
		.profile_velocity_rpm = DT_INST_PROP(inst, profile_velocity_rpm),                  \
		.profile_velocity_max_rpm = DT_INST_PROP(inst, profile_velocity_max_rpm),          \
		.accel = DT_INST_PROP(inst, accel),                                                \
		.decel = DT_INST_PROP(inst, decel),                                                \
		.homing_velocity_rpm = DT_INST_PROP(inst, homing_velocity_rpm),                    \
		.homing_approach_rpm = DT_INST_PROP(inst, homing_approach_rpm),                    \
		.homing_travel_limit_milli = DT_INST_PROP(inst, homing_travel_limit_rad_milli),    \
		.homing_preroll_milli = DT_INST_PROP(inst, homing_preroll_rad_milli),              \
		.homing_group = DT_INST_PROP(inst, homing_group),                                  \
		.home_dir_negative = DT_INST_PROP(inst, home_direction_negative),                  \
		.home_search_cw = DT_INST_PROP(inst, home_search_clockwise),                       \
		.home_input_is_limit = DT_INST_PROP(inst, home_input_is_limit),                    \
		.home_switch_no = DT_INST_PROP(inst, home_switch_normally_open),                   \
		.enable_input_no = DT_INST_PROP(inst, enable_input_normally_open),                 \
		.homing_clearance_milli = DT_INST_PROP(inst, homing_limit_clearance_rad_milli),    \
		.max_follow_err = DT_INST_PROP_OR(inst, max_following_error_pulses, -1),           \
		.limit_stop_ms = DT_INST_PROP_OR(inst, limit_stop_ms, -1),                         \
		.quick_stop_ms = DT_INST_PROP_OR(inst, quick_stop_ms, -1),                         \
		.invert = DT_INST_PROP(inst, invert_position),                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, icl57_init, NULL, &icl57_data_##inst, &icl57_config_##inst,    \
			      POST_KERNEL, CONFIG_ACTUATOR_INIT_PRIORITY, &icl57_api);

DT_INST_FOREACH_STATUS_OKAY(ICL57_DEFINE)

/* Every instance, for the by-slave-id lookup and the `all` form of the tuning
 * shell commands (and the settings loader, which keys on slave id). */
#define ICL57_DEV_REF(inst) DEVICE_DT_INST_GET(inst),
const struct device *const icl57_devs[] = {DT_INST_FOREACH_STATUS_OKAY(ICL57_DEV_REF)};

size_t icl57_dev_count(void)
{
	return ARRAY_SIZE(icl57_devs);
}

const struct device *icl57_dev_by_id(uint8_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(icl57_devs); i++) {
		const struct icl57_config *cfg = icl57_devs[i]->config;

		if (cfg->slave_id == id) {
			return icl57_devs[i];
		}
	}
	return NULL;
}
