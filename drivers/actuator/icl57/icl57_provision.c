/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drive provisioning for the iCL-RS driver: icl57_provision() and icl57_save(),
 * behind `icl57 setup` and `icl57 save`. Output goes to the calling shell when
 * there is one and to the log otherwise (see ICL57_PRINT).
 */

#include <stdint.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/device.h>
#include <zephyr/drivers/actuator/icl57.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "icl57_internal.h"
#include "icl57_profile.h"

LOG_MODULE_DECLARE(actuator_icl57, CONFIG_ACTUATOR_LOG_LEVEL);

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
int icl57_save(const struct shell *sh, uint8_t id)
{
	if (icl57_shell_iface < 0) {
		ICL57_ERROR(sh, "modbus bus not initialised");
		return -ENODEV;
	}
	if (id < 1 || id > 31) {
		ICL57_ERROR(sh, "slave id must be 1..31");
		return -EINVAL;
	}

	int rc = icl57_mb_write_reg(icl57_shell_iface, id, ICL57_REG_CONTROL_WORD,
				    ICL57_CTRL_SAVE_EEPROM);

	if (rc < 0) {
		ICL57_ERROR(sh, "save to EEPROM (id %u) failed: %d", id, rc);
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
			ICL57_PRINT(sh, "save to EEPROM (id %u) ok, 0x1901 = 0x5555", id);
			return 0;
		}
		if (st == ICL57_SAVE_FAIL) {
			ICL57_ERROR(sh, "save to EEPROM (id %u) FAILED: 0x1901 = 0xAAAA", id);
			return -EIO;
		}
	}

	if (!answered) {
		ICL57_ERROR(sh, "save to EEPROM (id %u): drive silent for %d ms after the save", id,
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
		ICL57_PRINT(sh,
			    "save to EEPROM (id %u) sent; 0x1901 posts no verdict on this "
			    "firmware, so the read-back that follows is the check",
			    id);
	} else {
		ICL57_WARN(sh,
			   "save to EEPROM (id %u): 0x1901 reads 0x%04x, neither ok nor "
			   "failed, after %d ms; verify a saved register by hand",
			   id, st, ICL57_EEPROM_SAVE_RETRIES * ICL57_EEPROM_SAVE_RETRY_MS);
	}
	return 0;
}

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
int icl57_provision(const struct shell *sh, uint8_t id)
{
	if (icl57_shell_iface < 0) {
		ICL57_ERROR(sh, "modbus bus not initialised");
		return -ENODEV;
	}

	/* Writing the RS485-format register makes the drive re-init its UART, so the
	 * echo of this write is normally lost even though the write takes effect.
	 * Don't treat that as failure; proceed regardless. */
	int rc = icl57_mb_write_reg(icl57_shell_iface, id, ICL57_REG_RS485_FORMAT,
				    ICL57_RS485_FORMAT_8N2);

	ICL57_PRINT(sh, "set 8N2 (id %u)%s", id, rc < 0 ? " (echo lost, expected)" : " ok");

	/* Preserve detection bits already set (e.g. the hardware-fault ones this
	 * driver does not arm) rather than clobbering the operator's mask. */
	uint16_t mask = 0;
	uint16_t want = ICL57_ALARM_DETECT_MOTION;
	const struct device *dev_for_mask = icl57_dev_by_id(id);
	bool arm_follow_err =
		dev_for_mask != NULL &&
		((const struct icl57_config *)dev_for_mask->config)->max_follow_err >= 0;

	if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_ALARM_DETECT, &mask, 1) < 0) {
		ICL57_WARN(sh, "could not read alarm detection (id %u); writing bare mask", id);
	} else {
		if (mask == ICL57_ALARM_DETECT_ALL_ONES) {
			ICL57_PRINT(sh,
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
		ICL57_PRINT(sh,
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
		ICL57_PRINT(sh,
			    "id %u: the drive now powers up DE-ENERGISED; `actuator enable` "
			    "is required before it holds position",
			    id);
	}

	rc = icl57_save(sh, id);
	if (rc < 0) {
		return rc;
	}

	/* Read back: this register is the difference between a stalled axis faulting
	 * and silently swallowing setpoints, so confirm rather than assume. The save
	 * helper has already waited out the drive's unresponsive window. */
	if (icl57_mb_read_regs(icl57_shell_iface, id, ICL57_REG_ALARM_DETECT, &mask, 1) < 0) {
		ICL57_ERROR(sh,
			    "id %u: alarm detection written but read-back FAILED - "
			    "stall/overload protection unverified",
			    id);
		return -EIO;
	}

	if ((mask & want) != want || (mask & ~want & ICL57_ALARM_DETECT_FOLLOW_ERR) != 0U) {
		ICL57_ERROR(sh, "id %u: alarm detection read back 0x%04x, wanted 0x%04x", id, mask,
			    want);
		return -EIO;
	}

	ICL57_PRINT(sh, "id %u: alarm detection 0x%04x verified", id, mask);

	/* Homing block. Provisioned here rather than at init for the same reasons
	 * the alarm mask is: it is drive-side configuration, an explicit operator
	 * action, and icl57_init does no bus transaction by design. */
	const struct device *dev = icl57_dev_by_id(id);

	if (dev == NULL) {
		ICL57_WARN(sh, "id %u: no devicetree motor node, skipping homing setup", id);
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
		ICL57_ERROR(sh, "id %u: homing stop position write failed", id);
		return -EIO;
	}
	ICL57_PRINT(sh, "homing stop position %d (id %u) ok", park, id);

	int32_t bound = icl57_milli_rad_to_pulses(cfg->homing_travel_limit_milli,
						  (uint32_t)cfg->counts_per_rev);

	if (bound < 0) {
		bound = -bound;
	}
	if (bound > ICL57_PR8_21_MAX) {
		ICL57_WARN(sh,
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
		ICL57_WARN(sh, "id %u: stop times (Pr8.22/Pr8.23) unreadable", id);
	} else {
		ICL57_PRINT(sh,
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
		ICL57_PRINT(sh, "id %u: max following error %u pulses (Pr0.05)%s", id, ferr_max,
			    cfg->max_follow_err < 0 ? " -- the drive's own value; devicetree does "
						      "not set it"
						    : "");
	}

	if (icl57_save(sh, id) < 0) {
		return -EIO;
	}

	/* The DI function change does NOT take effect until the drive restarts: the
	 * manual says input functions are "effective after clicking save and restart
	 * drive". A drive that has been saved but not power-cycled looks provisioned
	 * and behaves unprovisioned, so say so rather than leaving it to be
	 * rediscovered on the bench. */
	ICL57_WARN(sh,
		   "id %u: POWER-CYCLE THE DRIVE now — the DI2 function change only "
		   "takes effect after a restart",
		   id);
	return 0;
}
