/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell commands for the iCL-RS driver (drivers/actuator/icl57): provisioning
 * (setup/save/estop), raw register access (rd/wr/praw/alarm/baud), and motion-
 * profile tuning (speed/ramp/invert/set-zero). Split out of actuator_icl57.c so
 * new commands land here directly instead of being moved later.
 *
 * No in-file CONFIG_SHELL guard: CMakeLists.txt already builds this file only
 * when CONFIG_SHELL is set (zephyr_library_sources_ifdef), so one here would
 * be redundant.
 */

#define DT_DRV_COMPAT rosterloh_actuator_icl57

#include <stdlib.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/crc.h>

#include "icl57_internal.h"
#include "icl57_profile.h"

/*
 * There is deliberately no `zero` command. Writing Pr8.02 <- 0x21 ("set the
 * current position as origin") does NOT zero the drive: the position registers
 * are left untouched, but subsequent absolute PR0 targets become relative to the
 * position at which it was issued, while 0x1012/0x1014 keep reporting in the
 * original frame. The result is a permanent `reported = commanded + position at
 * zero` offset — demonstrated against an un-zeroed axis as a control.
 * The manual's "manual set to zero" also requires selecting the mode via 0x600A
 * first, and that register cannot be read back to confirm, so a
 * wrong value risks starting a real homing traverse. The drives power up at
 * exactly 0, so park the carriage at home and power-cycle them instead.
 */

static int cmd_icl57_estop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return icl57_write_verified(sh, (uint8_t)strtoul(argv[1], NULL, 0), ICL57_REG_TRIGGER,
				    ICL57_TRIG_ESTOP, "e-stop");
}

/*
 * Save to EEPROM and wait for the drive's own verdict.
 *
 * The 0x2211 write is acknowledged before the commit starts, and the drive then
 * ignores Modbus for ~1.3 s (ICL57_EEPROM_SAVE_RETRY_MS x RETRIES covers it).
 * 0x1901 reports the outcome: 0x5555 saved, 0xAAAA failed. It reads 0x1111 both
 * before any save and again once a 0x5555 has been read -- the flag is
 * read-once (manual 4.3.5) -- so 0x1111 during the wait means "no verdict yet"
 * and the loop keeps going. Before this the callers waited a fixed number of
 * retries on some other register and took the drive answering at all as
 * success, which cannot tell a failed save from a slow one.
 *
 * No verdict within the window is reported but NOT treated as failure: this is
 * untested against a read-once flag, and aborting provisioning on an unproven
 * status word would be a worse outcome than an unverified save. The callers
 * that care read back what they saved.
 */
static int icl57_shell_save(const struct shell *sh, uint8_t id)
{
	if (icl57_shell_iface < 0) {
		shell_error(sh, "modbus bus not initialised");
		return -ENODEV;
	}
	if (id < 1 || id > 31) {
		shell_error(sh, "slave id must be 1..31");
		return -EINVAL;
	}

	int rc = icl57_mb_write_reg(icl57_shell_iface, id, ICL57_REG_CONTROL_WORD,
				    ICL57_CTRL_SAVE_EEPROM);

	if (rc < 0) {
		shell_error(sh, "save to EEPROM (id %u) failed: %d", id, rc);
		return rc;
	}

	bool answered = false;
	uint16_t st = 0;

	for (int try = 0; try < ICL57_EEPROM_SAVE_RETRIES; try++) {
		k_msleep(ICL57_EEPROM_SAVE_RETRY_MS);
		if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_SAVE_STATUS, &st, 1) < 0) {
			continue; /* still committing: the drive is not answering */
		}
		answered = true;
		if (st == ICL57_SAVE_OK) {
			shell_print(sh, "save to EEPROM (id %u) ok, 0x1901 = 0x5555", id);
			return 0;
		}
		if (st == ICL57_SAVE_FAIL) {
			shell_error(sh, "save to EEPROM (id %u) FAILED: 0x1901 = 0xAAAA", id);
			return -EIO;
		}
	}

	if (!answered) {
		shell_error(sh, "save to EEPROM (id %u): drive silent for %d ms after the save", id,
			    ICL57_EEPROM_SAVE_RETRIES * ICL57_EEPROM_SAVE_RETRY_MS);
		return -EIO;
	}

	/*
	 * Measured on fw 0x00ca/0x422a drives: 0x1901
	 * reads 0x0000 before, during and after a save. Not the manual's 0x1111
	 * idle value, and never 0x5555 -- this firmware posts no verdict there at
	 * all, so the callers' own read-back is the check and this is the normal
	 * outcome, not a warning. Anything other than 0x0000 is still worth a look.
	 */
	if (st == 0U) {
		shell_print(sh,
			    "save to EEPROM (id %u) sent; 0x1901 posts no verdict on this "
			    "firmware, so the read-back that follows is the check",
			    id);
	} else {
		shell_warn(sh,
			   "save to EEPROM (id %u): 0x1901 reads 0x%04x, neither ok nor "
			   "failed, after %d ms; verify a saved register by hand",
			   id, st, ICL57_EEPROM_SAVE_RETRIES * ICL57_EEPROM_SAVE_RETRY_MS);
	}
	return 0;
}

static int cmd_icl57_save(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return icl57_shell_save(sh, (uint8_t)strtoul(argv[1], NULL, 0));
}

/*
 * Write Pr8.10, the homing mode.
 *
 * Called from `setup` (which also saves it) AND from `home` immediately before
 * every run, because saving it is not sufficient: the value survives a drive
 * power-cycle in EEPROM -- 0x600A still reads back correctly afterwards -- but
 * the homing routine does not use it until it has been written at runtime. Bench
 * evidence: with bit0 set the search ran toward home correctly; after a drive
 * power-cycle, with 0x600A still reading 0x0007, the same search ran the other
 * way until it stalled into the end stop.
 *
 * So this is not redundant with setup. Removing the call in `home` reintroduces
 * a runaway that only appears on the first homing after a drive power-cycle.
 */

/* One-shot bring-up, persisted to EEPROM:
 *   1. RS485 format 8N2 (Pr5.24=5) - required on every drive. Modbus RTU
 *      characters are 11 bits, so Zephyr's modbus_serial.c forces 2 stop bits
 *      when parity is NONE and this master runs 8N2 on both TX and RX; a
 *      factory 8N1 drive then puts the next start bit where the receiver
 *      expects a second stop bit and every reply collapses after byte one.
 *      TX works even before this is set,
 *      because an 8N1 drive reads an 8N2 master fine (the extra stop bit just
 *      looks like idle) - which is what makes provisioning possible at all.
 *   2. Alarm detection (Pr4.22) - factory 0 leaves stall/following-error
 *      detection OFF, so the drive never raises the faults this driver maps.
 * Both are drive-side configuration, so this stays an explicit operator action
 * rather than something the driver applies at init.
 */
static int cmd_icl57_setup(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (icl57_shell_iface < 0) {
		shell_error(sh, "modbus bus not initialised");
		return -ENODEV;
	}

	uint8_t id = (uint8_t)strtoul(argv[1], NULL, 0);

	/* Writing the RS485-format register makes the drive re-init its UART, so the
	 * echo of this write is normally lost even though the write takes effect.
	 * Don't treat that as failure; proceed regardless. */
	int rc = icl57_mb_write_reg(icl57_shell_iface, id, ICL57_REG_RS485_FORMAT,
				    ICL57_RS485_FORMAT_8N2);

	shell_print(sh, "set 8N2 (id %u)%s", id, rc < 0 ? " (echo lost, expected)" : " ok");

	/* Preserve detection bits already set (e.g. the hardware-fault ones this
	 * driver does not arm) rather than clobbering the operator's mask. */
	uint16_t mask = 0;
	uint16_t want = ICL57_ALARM_DETECT_MOTION;
	const struct device *dev_for_mask = icl57_dev_by_id(id);
	bool arm_follow_err =
		dev_for_mask != NULL &&
		((const struct icl57_config *)dev_for_mask->config)->max_follow_err >= 0;

	if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_ALARM_DETECT, &mask, 1) < 0) {
		shell_warn(sh, "could not read alarm detection (id %u); writing bare mask", id);
	} else {
		if (mask == ICL57_ALARM_DETECT_ALL_ONES) {
			shell_print(sh,
				    "id %u: detection 0x%04x covers the motion bits, but may "
				    "be erased EEPROM; only a stall test confirms it",
				    id, mask);
		}
		want = (uint16_t)(mask | ICL57_ALARM_DETECT_MOTION);
	}

	/*
	 * Following-error detection is armed only when its threshold is configured,
	 * and actively CLEARED otherwise -- the drives ship at 0xffff, so leaving it
	 * alone would leave it armed. See ICL57_ALARM_DETECT_FOLLOW_ERR for what
	 * that cost on the bench.
	 */
	if (arm_follow_err) {
		want |= ICL57_ALARM_DETECT_FOLLOW_ERR;
	} else {
		want &= (uint16_t)~ICL57_ALARM_DETECT_FOLLOW_ERR;
		shell_print(sh,
			    "id %u: following-error detection (Pr4.22 bit2) DISABLED -- it "
			    "stops a homing search short and `max-following-error-pulses` "
			    "is unset. Shaft-lock detection stays armed",
			    id);
	}

	rc = icl57_write_verified(sh, id, ICL57_REG_ALARM_DETECT, want, "arm alarm detection");
	if (rc < 0) {
		return rc;
	}

	/*
	 * DI1 to SRV-ON normally-OPEN when the node asks for it, so an unwired input
	 * stops asserting the IO enable and Pr0.07 alone decides whether the stage
	 * is live. Without it `actuator mode <axis> coast` writes Pr0.07 and changes
	 * nothing, because the IO enable outranks it. See ICL57_REG_DI1_FUNC for
	 * why, and for the per-axis safety argument -- it makes the drive come up
	 * de-energised. A drive with no node gets the factory N.C.
	 */
	bool enable_no = dev_for_mask != NULL &&
			 ((const struct icl57_config *)dev_for_mask->config)->enable_input_no;

	rc = icl57_write_verified(
		sh, id, ICL57_REG_DI1_FUNC, enable_no ? ICL57_DI1_SRV_ON_NO : ICL57_DI1_SRV_ON_NC,
		enable_no ? "DI1 to software-only enable" : "DI1 to IO enable (N.C.)");
	if (rc < 0) {
		return rc;
	}
	if (enable_no) {
		shell_print(sh,
			    "id %u: the drive now powers up DE-ENERGISED; `actuator enable` "
			    "is required before it holds position",
			    id);
	}

	rc = icl57_shell_save(sh, id);
	if (rc < 0) {
		return rc;
	}

	/* Read back: this register is the difference between a stalled axis faulting
	 * and silently swallowing setpoints, so confirm rather than assume. The save
	 * helper has already waited out the drive's unresponsive window. */
	if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_ALARM_DETECT, &mask, 1) < 0) {
		shell_error(sh,
			    "id %u: alarm detection written but read-back FAILED - "
			    "stall/overload protection unverified",
			    id);
		return -EIO;
	}

	if ((mask & want) != want || (mask & ~want & ICL57_ALARM_DETECT_FOLLOW_ERR) != 0U) {
		shell_error(sh, "id %u: alarm detection read back 0x%04x, wanted 0x%04x", id, mask,
			    want);
		return -EIO;
	}

	shell_print(sh, "id %u: alarm detection 0x%04x verified", id, mask);

	/* Homing block. Provisioned here rather than at init for the same reasons
	 * the alarm mask is: it is drive-side configuration, an explicit operator
	 * action, and icl57_init does no bus transaction by design. */
	const struct device *dev = icl57_dev_by_id(id);

	if (dev == NULL) {
		shell_warn(sh, "id %u: no devicetree motor node, skipping homing setup", id);
		return 0;
	}

	const struct icl57_config *cfg = dev->config;

	if (icl57_write_homing_mode(sh, dev, id) < 0) {
		return -EIO;
	}

	/*
	 * Post-home stop position (Pr8.13/14), 0x600D/0x600E as a 32-bit hi/lo pair.
	 * ZERO for every axis.
	 *
	 * Not non-zero for limit axes, on the theory that a completed limit search
	 * redefines the frame so 0 IS the limit and parking at 0 drives the axis back
	 * onto its switch. Both halves are false: the frame is NOT redefined (the
	 * limit's coordinate stays put across homes and moves), and the drive never
	 * travels to this target at all -- four homes reported a datum near -360
	 * against a +286 target they never reached. The motion that suggested it was
	 * a post-JOG PR0 re-pin hauling the axis back to a stale command frame, which
	 * is why homing uses no JOG.
	 *
	 * Zero is also the SAFE value: with no frame redefinition this is an
	 * ABSOLUTE machine coordinate, and a non-zero value such as -10000 in the
	 * drive's EEPROM is an arbitrary point for a vertical lifting pair, should
	 * the drive ever execute it.
	 */
	int32_t park = 0;

	uint16_t stop_at[2] = {
		(uint16_t)((uint32_t)park >> 16),
		(uint16_t)((uint32_t)park & 0xFFFF),
	};

	if (icl57_mb_write_regs(icl57_shell_iface, id, ICL57_REG_HOMING_STOP, stop_at, 2) < 0) {
		shell_error(sh, "id %u: homing stop position write failed", id);
		return -EIO;
	}
	shell_print(sh, "homing stop position %d (id %u) ok", park, id);

	int32_t bound = icl57_milli_rad_to_pulses(cfg->homing_travel_limit_milli,
						  (uint32_t)cfg->counts_per_rev);

	if (bound < 0) {
		bound = -bound;
	}
	if (bound > ICL57_PR8_21_MAX) {
		shell_warn(sh,
			   "id %u: travel limit %d pulses exceeds Pr8.21's 16-bit range; "
			   "writing %d. The host-side deadline still bounds the search",
			   id, bound, ICL57_PR8_21_MAX);
		bound = ICL57_PR8_21_MAX;
	}

	/*
	 * Which DI2 function: a mid-travel home switch (ORG), or the limit at the
	 * end of travel this axis actually homes against (PL/NL, chosen by which end
	 * home is at). Normally-closed unless home-switch-normally-open -- see
	 * ICL57_DI_FUNC_ORG.
	 */
	uint16_t di_func = icl57_di2_func(cfg);
	static const char *const di_whats[3][2] = {
		{"DI2 = home switch (N.C.)", "DI2 = home switch (N.O.)"},
		{"DI2 = POSITIVE limit (N.C.)", "DI2 = POSITIVE limit (N.O.)"},
		{"DI2 = NEGATIVE limit (N.C.)", "DI2 = NEGATIVE limit (N.O.)"},
	};
	const char *di_what = di_whats[!cfg->home_input_is_limit ? 0
				       : cfg->home_dir_negative  ? 2
								 : 1][cfg->home_switch_no];

	const struct {
		uint16_t reg;
		uint16_t val;
		const char *what;
	} homing[] = {
		{ICL57_REG_HOMING_HI_VEL, cfg->homing_velocity_rpm, "homing search velocity"},
		{ICL57_REG_HOMING_LO_VEL, cfg->homing_approach_rpm, "homing approach velocity"},
		{ICL57_REG_HOMING_ACCEL, cfg->accel, "homing accel"},
		{ICL57_REG_HOMING_DECEL, cfg->decel, "homing decel"},
		{ICL57_REG_HOMING_BOUND, (uint16_t)bound, "homing travel bound"},
		{ICL57_REG_DI2, di_func, di_what},
	};

	for (size_t i = 0; i < ARRAY_SIZE(homing); i++) {
		if (icl57_write_verified(sh, id, homing[i].reg, homing[i].val, homing[i].what) <
		    0) {
			return -EIO;
		}
	}

	/*
	 * Stop ramps: Pr8.22 (limit-switch deceleration) and Pr8.23 (quick-stop
	 * deceleration), both in ms. The e-stop is Pr8.02 <- 0x40 and its stopping
	 * distance is whatever Pr8.23 holds. Written only when the devicetree names
	 * a value; always read back and printed, so a commissioning record can note
	 * what the drive is running with. Qty 2 at 0x6016 covers both, pair-transposed as ever.
	 */
	if (cfg->limit_stop_ms >= 0 &&
	    icl57_write_verified(sh, id, ICL57_REG_LIMIT_STOP_MS, (uint16_t)cfg->limit_stop_ms,
				 "limit-switch stop time (Pr8.22)") < 0) {
		return -EIO;
	}
	if (cfg->quick_stop_ms >= 0 &&
	    icl57_write_verified(sh, id, ICL57_REG_QUICK_STOP_MS, (uint16_t)cfg->quick_stop_ms,
				 "quick-stop time (Pr8.23)") < 0) {
		return -EIO;
	}

	uint16_t stops[2] = {0};

	if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_LIMIT_STOP_MS, stops, 2) < 0) {
		shell_warn(sh, "id %u: stop times (Pr8.22/Pr8.23) unreadable", id);
	} else {
		shell_print(sh,
			    "id %u: limit-switch stop %u ms (Pr8.22), quick stop %u ms (Pr8.23)%s",
			    id, stops[icl57_pr_block_index(0)], stops[icl57_pr_block_index(1)],
			    (cfg->limit_stop_ms < 0 && cfg->quick_stop_ms < 0)
				    ? " -- the drive's own values; devicetree sets neither"
				    : "");
	}

	/*
	 * Pr0.05, the following-error threshold behind the bit2 detection armed
	 * above. Same treatment as the stop ramps: written only when devicetree
	 * names a value, reported either way. Drives as shipped (fw 0x00ca/0x422a)
	 * hold 65535, i.e. the detection is armed against a threshold it can
	 * never cross; the manual's own default is 4000.
	 */
	if (cfg->max_follow_err >= 0 &&
	    icl57_write_verified(sh, id, ICL57_REG_MAX_FOLLOW_ERR, (uint16_t)cfg->max_follow_err,
				 "max following error (Pr0.05)") < 0) {
		return -EIO;
	}

	uint16_t ferr_max = 0;

	if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_MAX_FOLLOW_ERR, &ferr_max, 1) ==
	    0) {
		shell_print(sh, "id %u: max following error %u pulses (Pr0.05)%s", id, ferr_max,
			    cfg->max_follow_err < 0 ? " -- the drive's own value; devicetree does "
						      "not set it"
						    : "");
	}

	if (icl57_shell_save(sh, id) < 0) {
		return -EIO;
	}

	/* The DI function change does NOT take effect until the drive restarts: the
	 * manual says input functions are "effective after clicking save and restart
	 * drive". A drive that has been saved but not power-cycled looks provisioned
	 * and behaves unprovisioned, so say so rather than leaving it to be
	 * rediscovered on the bench. */
	shell_warn(sh,
		   "id %u: POWER-CYCLE THE DRIVE now — the DI2 function change only "
		   "takes effect after a restart",
		   id);
	return 0;
}

/*
 * `icl57 factory-reset <id> confirm [all]` -- 0x1801 <- 0x2222 (every parameter
 * except the motor ones) or, with `all`, 0x2233 (those too). The manual's own
 * remedy for the EEPROM alarm, and the alternative to "power-cycle and hope"
 * when a drive's state has become irreproducible.
 *
 * Two guards: the literal word `confirm`, and the axis must be disabled --
 * resetting Pr0.07, Pr4.02 and the PR block under a live actuator state would
 * leave the subsystem believing an enable the drive no longer has.
 *
 * WHAT IT COSTS, so nobody discovers it on the bench: Pr5.22 and Pr5.24 go back
 * to factory, so the drive answers at 38400 (or 9600 -- the manual calls one
 * "factory" and the other "default") and 8N1, i.e. it falls silent on this bus
 * the instant the reset lands. Writes still reach an 8N1 drive from an 8N2
 * master, which is what makes the recovery ladder below
 * possible. Whether the reset persists without a save, and whether the drive
 * needs a power-cycle before it takes effect, are not stated: do both. Not yet
 * exercised on a drive.
 */
static int cmd_icl57_factory_reset(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id = (uint8_t)strtoul(argv[1], NULL, 0);

	if (argc < 3 || strcmp(argv[2], "confirm") != 0) {
		shell_error(sh,
			    "refusing: this wipes the drive's provisioning and takes it off the "
			    "bus. Re-run with `confirm` if that is what you want");
		return -EINVAL;
	}

	bool all = argc > 3 && strcmp(argv[3], "all") == 0;
	const struct device *dev = icl57_dev_by_id(id);

	if (dev != NULL && actuator_get_state(dev) != ACTUATOR_STATE_DISABLED) {
		shell_error(sh, "motor %u: disable the axis first", id);
		return -EBUSY;
	}

	int rc = icl57_write_verified(sh, id, ICL57_REG_CONTROL_WORD,
				      all ? ICL57_CTRL_RESET_ALL : ICL57_CTRL_RESET_PARAMS,
				      all ? "factory reset, ALL parameters"
					  : "factory reset, motor parameters kept");

	if (rc < 0) {
		return rc;
	}

	shell_warn(sh,
		   "id %u: EXPECT SILENCE from this drive now -- Pr5.22/Pr5.24 are back at "
		   "factory baud and 8N1. Recover: `icl57 baud 38400` (or 9600; `icl57 praw "
		   "%u 0x0001` until it answers), `icl57 wr %u 0x01C1 5`, `icl57 wr %u 0x01BD "
		   "6`, `icl57 baud 115200`, `icl57 save %u`, power-cycle, `icl57 setup %u`",
		   id, id, id, id, id, id);
	return 0;
}

static int cmd_icl57_alarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (icl57_shell_iface < 0) {
		shell_error(sh, "modbus bus not initialised");
		return -ENODEV;
	}

	uint8_t id = (uint8_t)strtoul(argv[1], NULL, 0);
	uint16_t alarm = 0;
	int rc = icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_ALARM, &alarm, 1);

	if (rc < 0) {
		shell_error(sh, "read alarm (id %u) failed: %d", id, rc);
		return rc;
	}
	shell_print(sh, "id %u alarm 0x%04x", id, alarm);
	return 0;
}

/* Bring-up diagnostic: re-init the Modbus client at a given baud at runtime so
 * the MCU RX rate can be swept against a fixed-baud drive without reflashing. */
static int cmd_icl57_baud(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (icl57_shell_iface < 0) {
		shell_error(sh, "modbus bus not initialised");
		return -ENODEV;
	}

	uint32_t baud = (uint32_t)strtoul(argv[1], NULL, 0);
	struct modbus_iface_param param = {
		.mode = MODBUS_MODE_RTU,
		.rx_timeout = 100000,
		.serial = {.baud = baud,
			   .parity = UART_CFG_PARITY_NONE,
			   .stop_bits =
				   UART_CFG_STOP_BITS_2}, /* 8N2, as icl57_client_init_once() */
	};

	(void)modbus_disable(icl57_shell_iface);
	int rc = modbus_init_client(icl57_shell_iface, param);

	if (rc < 0) {
		shell_error(sh, "re-init at %u baud failed: %d", baud, rc);
		return rc;
	}
	shell_print(sh, "modbus client re-initialised at %u baud", baud);
	return 0;
}

/* Raw poll-mode Modbus FC03 read, bypassing modbus_serial's interrupt RX: drive
 * DE by hand, clock the request out with uart_poll_out, then read the reply with
 * uart_poll_in. If this returns clean bytes where modbus_serial garbles them,
 * the fault is in the interrupt-read path; if it garbles too, it is the shared
 * UART sampling/config. Uses the modbus-serial parent UART and its de-gpios. */
#define RAW_MODBUS_NODE DT_INST_PARENT(0)
static const struct device *const raw_uart = DEVICE_DT_GET(DT_PARENT(RAW_MODBUS_NODE));
static const struct gpio_dt_spec raw_de = GPIO_DT_SPEC_GET_OR(RAW_MODBUS_NODE, de_gpios, {0});

/* de-gpios is optional in the zephyr,modbus-serial binding -- an auto
 * direction-sensing transceiver (as on the waveshare_esp32p4_eth carrier) has
 * no DE line at all, and the {0} fallback above then leaves .port NULL.
 * gpio_pin_set_dt() dereferences it, so this command used to take the board
 * down with a CPU exception on exactly the boards it exists to diagnose. */
static void raw_de_set(int value)
{
	if (raw_de.port != NULL) {
		gpio_pin_set_dt(&raw_de, value);
	}
}

static int cmd_icl57_praw(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id = (uint8_t)strtoul(argv[1], NULL, 0);
	uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);
	uint8_t req[8] = {id, 0x03, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), 0x00, 0x01, 0, 0};
	uint16_t crc = crc16_ansi(req, 6);

	req[6] = (uint8_t)(crc & 0xFF);
	req[7] = (uint8_t)(crc >> 8);

	/* Drain any stale RX. */
	uint8_t junk;

	while (uart_poll_in(raw_uart, &junk) == 0) {
	}

	/* Transmit under DE, then release once the whole frame has shifted out.
	 * uart_poll_out only queues into the TX FIFO, so hold DE for a fixed
	 * 4.4 ms before dropping it, or the request is truncated. That is generous
	 * at the bus's 115200-8N2 (8 x 11 bits is ~0.76 ms) and still covers a
	 * 19200 sweep with `icl57 baud`. */
	raw_de_set(1);
	for (size_t i = 0; i < sizeof(req); i++) {
		uart_poll_out(raw_uart, req[i]);
	}
	k_busy_wait(4400);
	raw_de_set(0);

	/* Collect the reply: stop after ~5 ms of RX idle or the buffer fills. */
	uint8_t rx[32];
	size_t n = 0;
	int64_t deadline = k_uptime_get() + 60;

	while (k_uptime_get() < deadline && n < sizeof(rx)) {
		uint8_t c;

		if (uart_poll_in(raw_uart, &c) == 0) {
			rx[n++] = c;
			deadline = k_uptime_get() + 5;
		}
	}

	if (n == 0) {
		shell_warn(sh, "poll-raw id %u reg 0x%04x: no reply", id, addr);
		return -ETIMEDOUT;
	}

	char hex[3 * sizeof(rx) + 1];
	size_t p = 0;

	for (size_t i = 0; i < n; i++) {
		p += snprintk(hex + p, sizeof(hex) - p, "%02x ", rx[i]);
	}
	shell_print(sh, "poll-raw id %u reg 0x%04x rx(%zu): %s", id, addr, n, hex);
	return 0;
}

/* Motion-profile tuning: `<id|all>` with no value reads, with a value writes.
 * Unlike the other icl57 commands these touch no register directly — the profile
 * rides along in the next PR0 block — so there is nothing to `save` on the drive;
 * the value is persisted host-side in the settings backend instead. */
struct icl57_tuning_op {
	enum icl57_tuning_field field;
	uint32_t value;
};

static int icl57_tuning_for_each(const struct shell *sh, const char *sel,
				 int (*fn)(const struct shell *sh, const struct device *dev,
					   const struct icl57_tuning_op *op),
				 const struct icl57_tuning_op *op)
{
	if (strcmp(sel, "all") == 0) {
		for (size_t i = 0; i < icl57_dev_count(); i++) {
			int rc = fn(sh, icl57_devs[i], op);

			if (rc < 0) {
				return rc;
			}
		}
		return 0;
	}

	const struct device *dev = icl57_dev_by_id((uint8_t)strtoul(sel, NULL, 0));

	if (dev == NULL) {
		shell_error(sh, "no motor with slave id '%s'", sel);
		return -ENODEV;
	}
	return fn(sh, dev, op);
}

static int icl57_show_speed(const struct shell *sh, const struct device *dev,
			    const struct icl57_tuning_op *op)
{
	ARG_UNUSED(op);
	const struct icl57_config *cfg = dev->config;
	const struct icl57_data *d = dev->data;

	shell_print(sh, "motor %u speed %u rpm (max %u)", cfg->slave_id, d->tuning.rpm,
		    cfg->profile_velocity_max_rpm);
	return 0;
}

static int icl57_show_ramp(const struct shell *sh, const struct device *dev,
			   const struct icl57_tuning_op *op)
{
	ARG_UNUSED(op);

	const struct icl57_config *cfg = dev->config;
	const struct icl57_data *d = dev->data;

	shell_print(sh, "motor %u ramp accel %u decel %u (ms/1000rpm; larger is slower)",
		    cfg->slave_id, d->tuning.accel, d->tuning.decel);
	return 0;
}

/* Set one field on dev's homing group and persist it (icl57_tuning_set),
 * reporting a failed persist. */
static int icl57_apply_one(const struct shell *sh, const struct device *dev,
			   const struct icl57_tuning_op *op)
{
	const struct icl57_config *cfg = dev->config;
	int save_rc;
	int rc = icl57_tuning_set(dev, op->field, op->value, &save_rc);

	if (rc < 0) {
		if (op->field == ICL57_TUNING_RPM) {
			shell_error(sh, "motor %u: speed must be 1..%u rpm%s", cfg->slave_id,
				    cfg->profile_velocity_max_rpm,
				    cfg->homing_group != 0 ? " on every motor of its homing group"
							   : "");
		} else {
			shell_error(sh, "motor %u: ramp must be 0..%u ms/1000rpm", cfg->slave_id,
				    ICL57_ACCEL_MAX);
		}
		return rc;
	}

	if (cfg->homing_group != 0) {
		shell_print(sh,
			    "motor %u: set on every motor of homing group %d, which shares "
			    "one beam",
			    cfg->slave_id, cfg->homing_group);
	}
	if (save_rc < 0) {
		shell_warn(sh, "motor %u: applied but not persisted (%d)", cfg->slave_id, save_rc);
	}
	return 0;
}

/* Capture the current position as the caller-facing zero. The read comes back
 * already in the offset frame, so accumulating onto the existing offset makes
 * re-zeroing an already-zeroed axis work without tracking the raw frame. */
static int icl57_zero_capture(const struct shell *sh, const struct device *dev,
			      const struct icl57_tuning_op *op)
{
	ARG_UNUSED(op);
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;
	struct actuator_feedback fb;
	int rc = icl57_read_feedback(dev, &fb);

	if (rc < 0) {
		shell_error(sh, "motor %u: feedback read failed (%d)", cfg->slave_id, rc);
		return rc;
	}

	d->zero_rad += fb.position;
	shell_print(sh, "motor %u zeroed here (offset %.4f rad, lost on reboot)", cfg->slave_id,
		    (double)d->zero_rad);
	return 0;
}

static int icl57_zero_clear(const struct shell *sh, const struct device *dev,
			    const struct icl57_tuning_op *op)
{
	ARG_UNUSED(op);
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	d->zero_rad = 0.0f;
	shell_print(sh, "motor %u offset cleared", cfg->slave_id);
	return 0;
}

static int icl57_show_invert(const struct shell *sh, const struct device *dev,
			     const struct icl57_tuning_op *op)
{
	ARG_UNUSED(op);
	const struct icl57_config *cfg = dev->config;
	const struct icl57_data *d = dev->data;

	shell_print(sh, "motor %u invert %u (devicetree default %u)", cfg->slave_id, d->invert,
		    cfg->invert);
	return 0;
}

/*
 * Flip the direction sign of a STOPPED axis.
 *
 * Refused unless the actuator is disabled, because inverting under a live
 * setpoint stream mirrors the commanded absolute position: the very next
 * command would ask an axis at +0.4 m to go to -0.4 m, and it would be sized
 * as an unsizable delta, i.e. at the full velocity ceiling. There is no safe
 * way to do that mid-motion, so the guard is the feature.
 *
 * Both the last-setpoint reference and the host-side zero were measured in the
 * old sign frame and cannot be carried across, so both are dropped.
 */
static int icl57_set_invert(const struct shell *sh, const struct device *dev,
			    const struct icl57_tuning_op *op)
{
	const struct icl57_config *cfg = dev->config;
	struct icl57_data *d = dev->data;

	if (actuator_get_state(dev) != ACTUATOR_STATE_DISABLED) {
		shell_error(sh, "motor %u: disable the axis before inverting it", cfg->slave_id);
		return -EBUSY;
	}

	d->invert = (op->value != 0U);
	d->have_last = false;
	d->zero_rad = 0.0f;

	shell_print(sh, "motor %u invert %u; offset cleared, re-zero before use", cfg->slave_id,
		    d->invert);
	return 0;
}

static int cmd_icl57_invert(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		return icl57_tuning_for_each(sh, argv[1], icl57_show_invert, NULL);
	}

	struct icl57_tuning_op op = {
		.value = (uint32_t)strtoul(argv[2], NULL, 0),
	};

	return icl57_tuning_for_each(sh, argv[1], icl57_set_invert, &op);
}

static int cmd_icl57_set_zero(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 2 && strcmp(argv[2], "clear") == 0) {
		return icl57_tuning_for_each(sh, argv[1], icl57_zero_clear, NULL);
	}
	return icl57_tuning_for_each(sh, argv[1], icl57_zero_capture, NULL);
}

static int icl57_unjam_one(const struct shell *sh, const struct device *dev,
			   const struct icl57_tuning_op *op)
{
	ARG_UNUSED(op);
	return icl57_unjam(sh, dev);
}

static int cmd_icl57_unjam(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return icl57_tuning_for_each(sh, argv[1], icl57_unjam_one, NULL);
}

/* `all`, or one slave id, which homes its whole homing group: icl57_home_run(). */
static int cmd_icl57_home(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *dev = NULL;

	if (strcmp(argv[1], "all") != 0) {
		dev = icl57_dev_by_id((uint8_t)strtoul(argv[1], NULL, 0));
		if (dev == NULL) {
			shell_error(sh, "no motor with slave id '%s'", argv[1]);
			return -ENODEV;
		}
	}
	return icl57_home_run(sh, dev);
}

static int cmd_icl57_speed(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		return icl57_tuning_for_each(sh, argv[1], icl57_show_speed, NULL);
	}

	struct icl57_tuning_op op = {
		.field = ICL57_TUNING_RPM,
		.value = (uint32_t)strtoul(argv[2], NULL, 0),
	};

	return icl57_tuning_for_each(sh, argv[1], icl57_apply_one, &op);
}

static int cmd_icl57_ramp(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		return icl57_tuning_for_each(sh, argv[1], icl57_show_ramp, NULL);
	}

	struct icl57_tuning_op op = {
		.field = ICL57_TUNING_ACCEL,
		.value = (uint32_t)strtoul(argv[2], NULL, 0),
	};
	int rc = icl57_tuning_for_each(sh, argv[1], icl57_apply_one, &op);

	if (rc < 0) {
		return rc;
	}

	/* One argument sets a symmetric ramp, which is what most axes want;
	 * a second sets decel independently. */
	op.field = ICL57_TUNING_DECEL;
	if (argc > 3) {
		op.value = (uint32_t)strtoul(argv[3], NULL, 0);
	}
	return icl57_tuning_for_each(sh, argv[1], icl57_apply_one, &op);
}

static int cmd_icl57_rd(const struct shell *sh, size_t argc, char **argv)
{

	if (icl57_shell_iface < 0) {
		shell_error(sh, "modbus bus not initialised");
		return -ENODEV;
	}

	uint8_t id = (uint8_t)strtoul(argv[1], NULL, 0);
	uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);
	uint16_t qty = (argc > 3) ? (uint16_t)strtoul(argv[3], NULL, 0) : 1;
	/* Zeroed, not left to the stack: a read that returns success without filling
	 * the buffer otherwise prints whatever the previous shell command left there.
	 * That is not hypothetical -- qty-1 reads of the 0x6xxx PR block printed a
	 * constant 0x694c (ASCII "iL", model-string bytes from an earlier call) and
	 * were misread on the bench as the homing registers being unwritable, when
	 * the same registers at qty 2 held exactly what `icl57 setup` had written. */
	uint16_t val[16] = {0};

	if (qty < 1 || qty > ARRAY_SIZE(val)) {
		shell_error(sh, "qty must be 1..%zu", ARRAY_SIZE(val));
		return -EINVAL;
	}

	int rc = icl57_mb_read_regs(icl57_shell_iface, id, addr, val, qty);

	if (rc < 0) {
		shell_error(sh, "read id %u reg 0x%04x qty %u failed: %d", id, addr, qty, rc);
		return rc;
	}
	if (qty == 1) {
		shell_print(sh, "id %u reg 0x%04x = 0x%04x (%u)", id, addr, val[0], val[0]);
	} else {
		char hex[6 * ARRAY_SIZE(val) + 1];
		size_t p = 0;

		for (uint16_t i = 0; i < qty; i++) {
			p += snprintk(hex + p, sizeof(hex) - p, "%04x ", val[i]);
		}
		shell_print(sh, "id %u reg 0x%04x qty %u: %s", id, addr, qty, hex);
	}
	return 0;
}

static int cmd_icl57_wr(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (icl57_shell_iface < 0) {
		shell_error(sh, "modbus bus not initialised");
		return -ENODEV;
	}

	uint8_t id = (uint8_t)strtoul(argv[1], NULL, 0);
	uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);
	uint16_t val = (uint16_t)strtoul(argv[3], NULL, 0);
	int rc = icl57_mb_write_reg(icl57_shell_iface, id, addr, val);

	/* The drive acts on a valid request even when we misread its echoed
	 * response, so report the transport result but note the write was sent. */
	if (rc < 0) {
		shell_warn(sh, "wrote id %u reg 0x%04x = 0x%04x (response rc=%d)", id, addr, val,
			   rc);
		return rc;
	}
	shell_print(sh, "id %u reg 0x%04x <- 0x%04x ok", id, addr, val);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_icl57,
	SHELL_CMD_ARG(estop, NULL, "Emergency-stop a motor.\nUsage: icl57 estop <slave-id>",
		      cmd_icl57_estop, 2, 0),
	SHELL_CMD_ARG(save, NULL,
		      "Save a motor's parameters to EEPROM and wait for its 0x1901 verdict.\n"
		      "Usage: icl57 save <slave-id>",
		      cmd_icl57_save, 2, 0),
	SHELL_CMD_ARG(factory - reset, NULL,
		      "Reset a drive's parameters to factory (0x1801 <- 0x2222; `all` also\n"
		      "resets the motor parameters, 0x2233). The drive then answers at\n"
		      "factory baud and 8N1, so it goes SILENT on this bus; the output\n"
		      "prints the recovery ladder. Disable the axis first.\n"
		      "Usage: icl57 factory-reset <slave-id> confirm [all]",
		      cmd_icl57_factory_reset, 3, 1),
	SHELL_CMD_ARG(setup, NULL,
		      "Provision a drive and save it to EEPROM: RS485 8N2, the alarm detection\n"
		      "mask, DI1 as software-only enable, the homing block and DI2's function.\n"
		      "Power-cycle the drive afterwards; the DI change needs a restart.\n"
		      "Usage: icl57 setup <slave-id>",
		      cmd_icl57_setup, 2, 0),
	SHELL_CMD_ARG(alarm, NULL,
		      "Read a motor's current alarm code.\nUsage: icl57 alarm <slave-id>",
		      cmd_icl57_alarm, 2, 0),
	/*
	 * `set-zero`, not `offset` and not `zero`.
	 *
	 * `offset` read like a query and was twice invoked expecting one, silently
	 * moving the host frame instead -- an imperative name makes the write
	 * obvious. `zero` was rejected too: `icl57 zero` used to mean the DRIVE-side
	 * Pr8.02 <- 0x21 origin trigger, which was removed for creating a permanent
	 * command/report offset (see the deliberately absent `zero` command above).
	 * Reusing that name for a different
	 * operation would resurrect exactly the confusion it was removed to end.
	 */
	SHELL_CMD_ARG(set - zero, NULL,
		      "Zero an axis host-side: make its current position read 0.\n"
		      "This WRITES; there is no read form.\n"
		      "Does NOT touch the drive's own origin (see README).\n"
		      "Cleared by a reboot -- re-zero after one.\n"
		      "Usage: icl57 set-zero <slave-id|all> [clear]",
		      cmd_icl57_set_zero, 2, 1),
	SHELL_CMD_ARG(unjam, NULL,
		      "Step an axis clear of a limit it is parked on with ordinary PR0 moves\n"
		      "of homing-limit-clearance-rad-milli, re-reading Pr4.28 after each.\n"
		      "For when `home` leaves an axis on its switch. Enable the axis first.\n"
		      "Usage: icl57 unjam <slave-id|all>",
		      cmd_icl57_unjam, 2, 0),
	SHELL_CMD_ARG(home, NULL,
		      "Reference axes to their home switches (drives must be provisioned\n"
		      "by `icl57 setup` and power-cycled, and the axes enabled).\n"
		      "Moves off home first, then searches; homes concurrently so a\n"
		      "shared beam squares instead of racking. Naming any motor in a\n"
		      "homing-group homes that whole group, so `home 1` moves the\n"
		      "vertical pair and leaves other axes alone.\n"
		      "Usage: icl57 home <slave-id|all>",
		      cmd_icl57_home, 2, 0),
	SHELL_CMD_ARG(invert, NULL,
		      "Get or set an axis's direction sign; the axis must be disabled.\n"
		      "Bench aid only -- NOT persisted, and it clears the offset.\n"
		      "Put the final sign in the devicetree `invert-position`.\n"
		      "Usage: icl57 invert <slave-id|all> [0|1]",
		      cmd_icl57_invert, 2, 1),
	SHELL_CMD_ARG(baud, NULL, "Re-init the Modbus client at a baud.\nUsage: icl57 baud <rate>",
		      cmd_icl57_baud, 2, 0),
	SHELL_CMD_ARG(speed, NULL,
		      "Get or set the velocity ceiling (persists across reboots).\n"
		      "A set covers the motor's whole homing group.\n"
		      "Usage: icl57 speed <slave-id|all> [rpm]",
		      cmd_icl57_speed, 2, 1),
	SHELL_CMD_ARG(ramp, NULL,
		      "Get or set accel/decel in ms per 1000 rpm; LARGER is SLOWER.\n"
		      "One value sets both. Persists across reboots.\n"
		      "A set covers the motor's whole homing group.\n"
		      "Usage: icl57 ramp <slave-id|all> [accel] [decel]",
		      cmd_icl57_ramp, 2, 2),
	SHELL_CMD_ARG(praw, NULL,
		      "Poll-mode raw FC03 read (bypasses modbus RX).\n"
		      "Usage: icl57 praw <slave-id> <addr>",
		      cmd_icl57_praw, 3, 0),
	SHELL_CMD_ARG(rd, NULL, "Read holding registers.\nUsage: icl57 rd <slave-id> <addr> [qty]",
		      cmd_icl57_rd, 3, 1),
	SHELL_CMD_ARG(wr, NULL,
		      "Write a holding register.\nUsage: icl57 wr <slave-id> <addr> <value>",
		      cmd_icl57_wr, 4, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(icl57, &sub_icl57, "iCL-RS stepper provisioning", NULL);
