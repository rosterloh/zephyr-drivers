/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver-internal contract between actuator_icl57.c and icl57_shell.c. The
 * shell needs the register map, both driver structs, and a handful of the
 * driver's read helpers, so they live here rather than being duplicated.
 *
 * NOT a public API, and not a stable one: anything here may change with the
 * driver. Applications use <zephyr/drivers/actuator/icl57.h> instead; nothing
 * outside drivers/actuator/icl57/ includes this file.
 */

#ifndef ICL57_INTERNAL_H_
#define ICL57_INTERNAL_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/actuator/actuator.h>
#include <zephyr/drivers/actuator/icl57.h>

#include "icl57_profile.h"

struct shell;

/*
 * Output for code the shell and icl57_home() share: to the calling shell when
 * there is one, else to the log, where a routine step is debug-level noise.
 */
#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#define ICL57_OUT(sh, shell_fn, log_fn, ...)                                                       \
	do {                                                                                       \
		if ((sh) != NULL) {                                                                \
			shell_fn(sh, __VA_ARGS__);                                                 \
		} else {                                                                           \
			log_fn(__VA_ARGS__);                                                       \
		}                                                                                  \
	} while (0)
#else
#define ICL57_OUT(sh, shell_fn, log_fn, ...)                                                       \
	do {                                                                                       \
		ARG_UNUSED(sh);                                                                    \
		log_fn(__VA_ARGS__);                                                               \
	} while (0)
#endif
#define ICL57_PRINT(sh, ...) ICL57_OUT(sh, shell_print, LOG_DBG, __VA_ARGS__)
#define ICL57_WARN(sh, ...)  ICL57_OUT(sh, shell_warn, LOG_WRN, __VA_ARGS__)
#define ICL57_ERROR(sh, ...) ICL57_OUT(sh, shell_error, LOG_ERR, __VA_ARGS__)

#define ICL57_CB_POOL CONFIG_ACTUATOR_MAX_CALLBACKS_PER_DEVICE

/* Motor instance count as a COMPILE-TIME constant, for sizing local arrays.
 * icl57_dev_count() is the runtime form, needed because icl57_devs[] reaches
 * icl57_shell.c only as an extern array of incomplete type; this is the form a
 * local array declaration needs. Both describe the same devicetree instances. */
#define ICL57_MOTOR_COUNT DT_NUM_INST_STATUS_OKAY(rosterloh_actuator_icl57)

/* Slow-retry period for the feedback worker after a failed read. */
#define ICL57_FB_BACKOFF_MS 2000

/* How many poll periods a cached feedback sample stays usable for sizing a move.
 * Tolerates one missed poll; past that the axis may have travelled a long way
 * since, and the last commanded target is the safer of two stale references. */
#define ICL57_FB_FRESH_PERIODS 3
/* Re-sends of a dropped target per accepted setpoint; see icl57_needs_retrigger(). */
#define ICL57_RETRIGGER_MAX    3

/* Holding-register map (see iCL-RS user manual section 4). 32-bit values live
 * in a high/low register pair; feedback reads take the high register first. */
#define ICL57_REG_FORCE_ENABLE   0x000F /* Pr0.07: write 1 = software enable   */
/*
 * DI1's function word (Pr4.02). Low byte is the function code, bit7 selects the
 * sense: 0x88 is SRV-ON normally-CLOSED (the factory default) and 0x08 is the
 * same function normally-OPEN.
 *
 * Flipping this is what makes a software torque release possible at all. The
 * manual is explicit that Pr0.07 "forced enable by software" has HIGHER priority
 * than the IO enable and that "when this value is 0, the enable status depends
 * only on the IO signal" -- so Pr0.07 can only ever ADD enable. With DI1 at the
 * N.C. default and nothing wired to it, the input is permanently asserted, the
 * drive is enabled from power-on regardless of the bus, and `actuator disable`
 * leaves the shaft locked. Recovering a jammed axis then means switching off the
 * drive and moving it by hand.
 *
 * At N.O. an unwired DI1 reads inactive, so Pr0.07 becomes the sole authority
 * and COAST is one register write. `icl57 setup` writes N.O. only for a node
 * with `enable-input-normally-open`, and the factory N.C. otherwise.
 *
 * CONSEQUENCE, and it is a real one: the drive now comes up DE-ENERGISED and
 * stays that way until something writes Pr0.07. Safe only on an axis that holds
 * position unpowered (a self-locking screw, or one with no gravity load), which
 * is why it is opt-in: see `enable-input-normally-open` in the binding. Leave
 * Pr4.02 at 0x88 on an axis that can back-drive under load.
 */
#define ICL57_REG_DI1_FUNC       0x0145 /* Pr4.02: DI1 function + sense         */
#define ICL57_DI1_SRV_ON_NO      0x0008 /* SRV-ON, normally OPEN                */
#define ICL57_DI1_SRV_ON_NC      0x0088 /* SRV-ON, normally closed (factory)    */
#define ICL57_REG_MOTION_STATE   0x1003 /* bitfield, read-only                 */
/*
 * Pr4.28: raw digital-input states, bit0-bit6 = DI1-DI7.
 *
 * The ONLY direct read of the limit switch: whether the input is asserted is
 * otherwise inferred from whether the drive accepts a PR0 move, which is the
 * same evidence a dozen other faults produce. Verified by pushing the carriage
 * by hand against DI2.
 */
#define ICL57_REG_DI_STATUS      0x0179 /* Pr4.28: DI1-DI7 in bit0-bit6        */
#define ICL57_DI_STATUS_DI2      BIT(1) /* set = asserted (0x0002 pressed)     */
/*
 * Profile (COMMAND-frame) position, 0x1012 hi / 0x1013 lo, in command pulses at
 * Pr0.00 resolution -- NOT the encoder counts 0x1014 reports.
 *
 * The distinction is the whole reason this exists. PR0 targets are command-frame
 * values, so anything that pins PR0 at "where the axis is now" must read THIS,
 * not the feedback position: the two differ by the servo following error, and
 * writing a feedback number into PR0 commands a move of exactly that error.
 * After a limit home that move points at the switch, which on an axis with a
 * 2 mm clearance is enough to drive back onto it.
 *
 * Units confirmed on a drive at rest: 0x1014 read -468925
 * against 0x1012's -71487, a ratio of 6.5589 against the 65536/10000 = 6.5536
 * the two resolutions predict.
 */
#define ICL57_REG_CMD_POSITION   0x1012 /* 0x1012 hi / 0x1013 lo, command pulses */
/*
 * THE DRIVE'S OWN POSITION FRAME (Pr8.42-8.45), AND THE ONE PR0 TARGETS ARE
 * EXECUTED IN. Both in COMMAND PULSES, like 0x1012 and unlike 0x1014.
 *
 * The registers above are the REPORTED frame. A completed homing search
 * re-zeros THIS frame and leaves the reported one counting, so the two diverge
 * by a constant D and every PR0 target computed from 0x1012 then lands D pulses
 * away. Measured after a good home: 0x602A read -9 while 0x1012 read -31725,
 * and a pre-move to +2001 landed at +15572 with D = 13571.
 *
 * So the driver works in THIS frame throughout -- targets, feedback and the
 * host-side zero -- and the reported frame survives only in the homing trace,
 * where the divergence is worth seeing. A power cycle zeroes both together,
 * which is why "power-cycle after an aborted home" always worked.
 *
 * DIRECT reads of these are pair-transposed like the rest of the 0x6xxx block
 * (icl57_read_u32 un-swaps); reads THROUGH the mapping block are NOT, measured
 * the same session against 0x6006/0x6007.
 */
#define ICL57_REG_DRIVE_CMD_POS  0x602A /* Pr8.42/43 profile position, pulses */
#define ICL57_REG_DRIVE_FB_POS   0x602C /* Pr8.44/45 actual position, pulses   */
/*
 * Following error, 0x1010 hi / 0x1011 lo. The manual says "pulses"; the drive
 * says otherwise. Measured on three drives at rest with the
 * command frame at 0: feedback -8 counts read 2048, -4 read 1024, +45 read
 * -11520 -- (command - feedback) in ENCODER COUNTS x 256, exactly, every time.
 */
#define ICL57_REG_FOLLOW_ERR     0x1010
/*
 * Feedback velocity. The map documents a 32-bit pair, and the drive does
 * sign-extend it, but IT DOES NOT LATCH THE TWO REGISTERS: one FC03 of both can
 * return a high word snapshotted either side of a sign change from the low one.
 * Measured with the axis at rest: 4 of 140 pair reads (2.9%) came back
 * `ffff 0000` or `0000 ffff`, composing to -65536 or +65535 rpm -- +/-6862.9
 * rad/s, reported as a real feedback velocity. At rest the velocity dithers +/-3 rpm across
 * zero continuously, so the sign flips constantly and the window is always open.
 *
 * Read the low register alone instead. The whole value lives there: across
 * those 140 reads the high word was only ever 0x0000 or 0xffff, i.e. pure sign
 * extension, and the drive tops out around 3000 rpm against int16's 32767. One
 * register cannot tear against itself, and it is one less register on the bus.
 */
#define ICL57_REG_FB_VEL_LO      0x1047 /* int16 rpm, signed; READ THIS ONE      */
#define ICL57_REG_CONTROL_WORD   0x1801 /* 0x1111 reset alarm, 0x2211 save     */
#define ICL57_REG_SAVE_STATUS    0x1901 /* read-once verdict of the last 0x2211 */
#define ICL57_REG_ALARM          0x2203 /* current alarm code, read-only       */
/* Pr8.02: 0x20 home, 0x21 set-origin (never; see the shell), 0x40 estop. Reads
 * back as the drive's progress report -- icl57_pr_status_decode(). */
#define ICL57_REG_TRIGGER        0x6002
#define ICL57_REG_RS485_FORMAT   0x01C1 /* Pr5.24: RS485 data/parity/stop format */
#define ICL57_RS485_FORMAT_8N2   0x0005 /* 8-bit, no parity, 2 stop bits         */
#define ICL57_REG_ALARM_DETECT   0x016D /* Pr4.22: alarm DETECTION ENABLE mask   */
#define ICL57_REG_DI2            0x0147 /* Pr4.03: DI2 function assignment       */
/*
 * Drive identity, read once per boot by icl57_check_drive_params() and checked
 * against the devicetree scale factors. Pr0.00 and Pr7.01 are the two numbers
 * every pulse <-> radian conversion in this driver rests on, so a mismatch
 * would otherwise skew every position without any error.
 */
#define ICL57_REG_PULSES_PER_REV 0x0001 /* Pr0.00: must equal counts-per-rev       */
#define ICL57_REG_MAX_FOLLOW_ERR 0x000B /* Pr0.05: pulses; the bit2 fault threshold */
#define ICL57_REG_PEAK_CURRENT   0x0191 /* Pr5.00: 0.1 A                            */
#define ICL57_REG_VERSION        0x01FF /* Pr6.15: read-only                        */
#define ICL57_REG_FIRMWARE       0x0201 /* Pr6.16: read-only                        */
#define ICL57_REG_ENCODER_RES    0x0233 /* Pr7.01: encoder resolution, unit unstated */
/*
 * Start of the Pr8/Pr9 register space. Two rules for READING it:
 *  - qty must be >= 2, and even: the drive REJECTS a single-register read of
 *    the PR block with exception 8. Before the icl57_mb_* wrappers existed that
 *    rejection came back as a positive code, was read as success, and the
 *    unfilled buffer was printed as the register's value -- easily misread as
 *    "the homing registers are unwritable".
 *  - the words come back PAIR-TRANSPOSED: see icl57_pr_block_index().
 * FC16 writes and the 0x10xx status registers are not affected.
 */
#define ICL57_REG_PR_BLOCK_BASE  0x6000
#define ICL57_REG_HOMING_MODE    0x600A /* Pr8.10: direction/move-to/method bits */
#define ICL57_REG_HOMING_STOP    0x600D /* Pr8.13 hi / Pr8.14 lo: post-home target */
#define ICL57_REG_HOMING_HI_VEL  0x600F /* Pr8.15: 1st-segment search rpm        */
#define ICL57_REG_HOMING_LO_VEL  0x6010 /* Pr8.16: 2nd-segment approach rpm      */
#define ICL57_REG_HOMING_ACCEL   0x6011 /* Pr8.17: ms/1000rpm                    */
#define ICL57_REG_HOMING_DECEL   0x6012 /* Pr8.18: ms/1000rpm                    */
#define ICL57_REG_HOMING_BOUND   0x6015 /* Pr8.21: stop after this much travel   */
#define ICL57_REG_PR_WARNING     0x601D /* Pr8.29: PR warning, incl. homing errors */
/* The two stop ramps, in ms. Pr8.23 is the ramp behind Pr8.02 <- 0x40, i.e. the
 * e-stop's own stopping distance; Pr8.22 the one after a limit input asserts.
 * `icl57 setup` writes them from `quick-stop-ms` / `limit-stop-ms` when the
 * devicetree sets those, and reports them either way. */
#define ICL57_REG_LIMIT_STOP_MS  0x6016 /* Pr8.22 */
#define ICL57_REG_QUICK_STOP_MS  0x6017 /* Pr8.23 */

/*
 * DI2's function code. ORG (home switch) is 0x27, and a function code +0x80
 * selects normally-closed sensing, so 0xA7 is "ORG, N.C." -- the default;
 * `home-switch-normally-open` drops the +0x80.
 *
 * N.C. is what the purchased sensors are (igus INI-AB-I-025-B-AA, PNP N.C.
 * inductive): the output sources 24 V while the axis is AWAY from home and goes
 * open AT home, so the drive has to treat de-energised as asserted. Wiring note
 * that goes with it: COMI (CN2 pin 8) must go to 0 V, not +24 V — the input opto
 * is bidirectional and COMI selects common-cathode (for a sourcing/PNP sensor)
 * or common-anode (for a sinking/NPN one). COMI on +24 V leaves a PNP sensor
 * unable to conduct at all, which looks exactly like an unwired switch.
 */
#define ICL57_DI_FUNC_ORG 0x0027
/* The limit inputs, for `home-input-is-limit`. Same +0x80 = normally-closed rule
 * as ORG above. */
#define ICL57_DI_FUNC_PL  0x0025
#define ICL57_DI_FUNC_NL  0x0026
#define ICL57_DI_FUNC_NC  0x0080 /* OR in: normally-closed sensing */

/* Pr8.10 (0x600A) mode bits. */
/* Bit0 is a ROTATION SENSE, not a position sign. The manual gives its polarity
 * both ways (section 5.2.1 says 0=CCW, Appendix A says 0=CW), so it is measured,
 * not read off the page. NOT interchangeable with
 * `home-direction-negative`, which says which END home is at; the sense that
 * travels that way is the separate `home-search-clockwise` property. */
/*
 * Pr8.10 bit0. DELIBERATELY NOT NAMED FOR A ROTATION SENSE -- do not "fix" it
 * to one without re-testing on hardware. Two facts that do not reconcile:
 *
 *  - The manual's Pr8.10 row says "Bit0: homing direction, 0--CW, 1--CCW"
 *    (docs/iCL-RS_Series_User_Manual.pdf).
 *  - The bit values that actually home real axes are what
 *    `home-search-clockwise` produces through the line below.
 *
 * Renaming this to match the manual and inverting the write accordingly was
 * tried and BROKE a working configuration: an axis had homed cleanly with
 * bit0 = 1, and with the flip every axis aborted with "search is running the
 * WRONG WAY". Reverted. The property's NAME may well be wrong
 * about which rotation is which, but its value is self-consistent across both
 * consumers -- this bit and the JOG word in icl57_jog_off_limit(), whose
 * direction was separately confirmed correct by `icl57 unjam` moving an axis
 * off its switch. Flipping the property would fix the name and break the jog.
 *
 * That test also refuted a standing belief: **bit0 is NOT inert.** The earlier
 * finding that "setting bit0 to 0 and to 1 produced identical behaviour
 * whenever a pre-move preceded the trigger" does not hold -- flipping it on
 * every axis changed a successful home into a wrong-way abort.
 *
 * The verify on this register used to read 0x0000 whatever was written. That
 * was the PR-block pair transposition (icl57_pr_block_index): word 0 of the
 * qty-2 read-back is 0x600B, the home-switch position high word, which is 0.
 * icl57_shell_write() now compares the right word, so the write is verifiable
 * -- and it always landed, which is the only way the flip above could have
 * changed behaviour.
 */
#define ICL57_HOMING_MODE_DIR_BIT0 BIT(0) /* Pr8.10 bit0; see above before touching */
#define ICL57_HOMING_MODE_MOVE_TO  BIT(1) /* 1 = finish at Pr8.13/14 after homing */
#define ICL57_HOMING_MODE_SWITCH   BIT(2) /* 1 = home-switch method, 0 = limit switch */

/* Pr8.02 (0x6002) command to start a homing search. NOT 0x21 — see the note on
 * the absent `zero` command below. */
#define ICL57_TRIG_HOMING 0x0020

/* Pr8.29 (0x601D) values that explain a failed search. */
#define ICL57_PR_WARN_LIMIT_DURING_HOMING 0x0100
#define ICL57_PR_WARN_HOMING_OVERTRAVEL   0x0102

/* Pr8.21 is one 16-bit PR register, so its range is the 32767 the manual gives
 * for the other PR position words. A converted bound above that is clamped, and
 * the host-side deadline is what actually bounds the search. */
#define ICL57_PR8_21_MAX 32767

#define ICL57_REG_PR0_BASE 0x6200 /* Pr9.00..Pr9.07 immediate-trigger    */
#define ICL57_PR0_NREGS    8

/*
 * Register mapping (manual section 4.5). Writing a register ADDRESS into
 * 0x0F10..0x0F19 makes that register's VALUE readable from the matching slot of
 * a mapped area, so one FC03 fetches registers scattered across the map. The
 * manual's prose says the values read back at 0x0F10 itself; its worked example
 * frame reads them at 0x0F00 and gets the right values. icl57_map_setup() probes
 * both and keeps whichever passes the sentinel (enum icl57_map_slot in
 * icl57_profile.h owns the slot layout). On every drive measured it is 0x0F00,
 * and 0x0F10 reads back the targets. Gated by
 * CONFIG_ACTUATOR_ICL57_MAPPED_POLL until a home has been polled through it.
 */
#define ICL57_REG_MAP_TARGETS  0x0F10
#define ICL57_REG_MAP_VALUES_A 0x0F00 /* the worked example's read address */
#define ICL57_REG_MAP_VALUES_B 0x0F10 /* the prose's read address          */

/* 0x1003 motion state: bit0 Fault, bit1 Enable, bit2 Running, bit4 Command
 * completed, bit5 Path completed, bit6 Homing completed. */
#define ICL57_MOTION_FAULT   BIT(0)
#define ICL57_MOTION_ENABLED BIT(1)
/* Bit2, "running". A PR path still owns the drive while this is set, and Pr8.02
 * writes are ignored until it clears -- which is why the homing trigger has to
 * wait on this and not merely on the pre-move's position. */
#define ICL57_MOTION_RUNNING BIT(2)
/* Bit6, "homing completed". Note the manual states the status word "is
 * automatically returned to its initial state after it is read", so this bit is
 * read-and-clear: whoever reads 0x1003 first consumes it. That is why homing
 * cancels the feedback poller for its duration rather than merely tolerating it. */
#define ICL57_MOTION_HOMED   BIT(6)

/* Time for the drive to act on an enable / alarm-reset write before its motion
 * state reflects the change. */
#define ICL57_STATE_SETTLE_MS 100

/*
 * Pr4.22 (0x016D) is an alarm DETECTION ENABLE mask whose factory value the
 * manual gives as 0: only over-current (bit0) is unmaskable. Every other alarm
 * this driver maps - including the locked-shaft (0x80) fault that becomes
 * ACTUATOR_FAULT_OVERLOAD - would then be disabled out of the box, so an
 * unprovisioned drive silently keeps accepting setpoints while stalled.
 * `icl57 setup` arms the motion-safety bits. NOTE: the bench drives did NOT
 * read 0 here - see the 0xFFFF note below.
 *
 * bit3 (ADC), bit5 (EEPROM) and bit6 (auto-tuning) are deliberately left off:
 * they are hardware-fault detections that were also off before, and bit6 in
 * particular risks nuisance faults because auto-tune-at-power-on (Pr5.13) is
 * enabled by default. Add them once there is bench time to confirm they are
 * quiet.
 */
#define ICL57_ALARM_DETECT_OVERVOLTAGE BIT(1)
#define ICL57_ALARM_DETECT_FOLLOW_ERR  BIT(2)
#define ICL57_ALARM_DETECT_SHAFT_LOCK  BIT(4)
/*
 * Armed unconditionally. Shaft-lock (bit4) is the real stall detection and the
 * one that backs ACTUATOR_FAULT_OVERLOAD.
 */
#define ICL57_ALARM_DETECT_MOTION (ICL57_ALARM_DETECT_OVERVOLTAGE | ICL57_ALARM_DETECT_SHAFT_LOCK)
/*
 * bit2, position following error, is armed ONLY when `max-following-error-pulses`
 * sets its threshold -- and no board sets it today, so `icl57 setup` CLEARS it.
 *
 * Arming a detection whose threshold nobody configured is what broke homing.
 * Measured on a homing search: with bit2 armed it faulted after
 * 15075 pulses, well short of the switch, with NO alarm code, NO red LED and no
 * PR warning -- the signature of the one fault class absent from the manual's
 * table. With bit2 cleared and nothing else changed the identical search ran
 * 33212 pulses and reached the switch.
 *
 * And the threshold cannot simply be raised: Pr0.05 ships at 65535, its maximum,
 * while 0x1010 reports the error in counts x 256, so if the drive compares the
 * scaled value the effective trip is ~256 counts against a 29-79 count resting
 * error. That makes the detection unusable as shipped rather than merely
 * mis-tuned. Set `max-following-error-pulses` once a stall test says what a real
 * following error reads, and this arms itself again.
 */

/*
 * The bench drives all read 0xFFFF here. That IS a real stored value, not an
 * unimplemented-register artifact: writing 0x0016 to one drive and reading it
 * back returned 0x0016 while its untouched neighbour still read 0xFFFF.
 *
 * It remains ambiguous whether 0xFFFF means "every detection deliberately
 * enabled" or is simply erased EEPROM, since the manual documents the default
 * as 0. Both readings satisfy the motion-safety bits, so this code treats it as
 * armed and does not warn - only a stall test can settle it. The other half of
 * that story is Pr0.05 (ICL57_REG_MAX_FOLLOW_ERR), the threshold bit2 fires at:
 * icl57_check_drive_params() logs it once per boot, and nothing sets it.
 */
#define ICL57_ALARM_DETECT_ALL_ONES 0xFFFFU

/*
 * An EEPROM save leaves the drive unresponsive while it commits: after 0x2211
 * the bench drives ignored Modbus for roughly 1.3 s (a read at +0.5 s still
 * timed out, and the feedback poller logged -EIO over the same window). Retry
 * the verify rather than guessing one delay, or it reports a bogus failure that
 * is really just the save still in progress.
 */
#define ICL57_EEPROM_SAVE_RETRY_MS 500
#define ICL57_EEPROM_SAVE_RETRIES  6
/* 0x1901 after a save. 0x1111 is what it reads before any save and again after
 * a 0x5555 has been read once (manual 4.3.5), so it is "no verdict", not "ok". */
#define ICL57_SAVE_OK              0x5555U
#define ICL57_SAVE_FAIL            0xAAAAU

#define ICL57_PR_MODE_ABS_POS 0x0001 /* Pr9.00: bit0-3=1 position, bit6=0 abs */
/*
 * Pr9.00 bit4 (INS). Without it a trigger arriving while PR0 is still running is
 * not honoured -- the drive finishes the in-flight move first, so a new setpoint
 * lands late and, on a streamed trajectory that falls behind its command period,
 * the axis tracks a queue of stale targets. With it the drive abandons the
 * running path and takes the new one, carrying the current speed across (manual
 * S5.4.3 "Interrupt function"). Every setpoint is an absolute target, so the
 * abandoned move has no residual value: overriding is always what we want.
 */
#define ICL57_PR_MODE_INS     0x0010
#define ICL57_PR_TRIGGER      0x0010 /* Pr9.07 -> Pr8.02: run PR0 immediately */

#define ICL57_CTRL_RESET_ALARM  0x1111
#define ICL57_CTRL_SAVE_EEPROM  0x2211
#define ICL57_CTRL_RESET_PARAMS 0x2222 /* factory, except the motor parameters */
#define ICL57_CTRL_RESET_ALL    0x2233 /* factory, everything                  */
#define ICL57_TRIG_ESTOP        0x0040

/* Alarm codes (0x2203) -> generic actuator fault flags. */
#define ICL57_ALARM_OVERCURRENT 0x01
#define ICL57_ALARM_OVERVOLTAGE 0x02
#define ICL57_ALARM_ADC         0x40
#define ICL57_ALARM_STALL       0x80
#define ICL57_ALARM_EEPROM      0x200
#define ICL57_ALARM_AUTOTUNE    0x100

/*
 * The drive reset since it was configured, so its position frame is void: the
 * mapped block lost its sentinel, or the position jumped and stayed (see
 * icl57_frame_check()). Faulted rather than just un-homed so a drive in a
 * homing group takes the group down with it: two drives in different frames
 * rack their shared load on every paired move.
 */
#define ICL57_FAULT_FRAME_LOST ACTUATOR_FAULT_DRIVER(3)

/*
 * 0x2203 carries no fault information on the bench drives: it reads a constant
 * 0x8200 on all four, identical on the one that genuinely faulted during a
 * stall test and on three that never faulted, and unchanged after the faulted
 * one recovered via power-cycle. Decoding it would be actively harmful, because
 * 0x8200 contains bit9 (ICL57_ALARM_EEPROM) and would therefore stamp a bogus
 * "EEPROM fault" on every single fault. Treat it as no-information and let the
 * caller fall back to the generic driver fault.
 *
 * Note the manual's alarm table is incomplete regardless: a real stall produced
 * a 7-flash error code that does not appear in it at all.
 */
#define ICL57_ALARM_NO_INFO 0x8200U

/*
 * ACTUATOR_CAP_FAULT_LATCHING is required, not cosmetic. Without it the state
 * machine rejects CLEAR_FAULT with -ENOTSUP, the subsystem treats the clear as
 * a no-op success and NEVER calls the backend, and the actuator is then stuck
 * in FAULT until reboot (disable returns 0 without transitioning, so enable is
 * refused with -EPERM forever). iCL-RS faults latch in the strongest sense:
 * most alarm classes need a drive power-cycle.
 */
#define ICL57_CAPS                                                                                 \
	(ACTUATOR_CAP_POSITION | ACTUATOR_CAP_FAULT_LATCHING | ACTUATOR_CAP_DRIVE_MODE |           \
	 ACTUATOR_CAP_GROUP_NATIVE)

struct icl57_data {
	struct actuator_common_data common;
	struct actuator_cb_storage cb_storage;
	struct actuator_cb_node cb_pool[ICL57_CB_POOL];
	const struct device *self;
	int iface;
	struct k_work_delayable feedback_work;
	bool fb_backoff; /* true while the feedback worker is in slow-retry mode */
	/* Last setpoint actually accepted by the drive, in (clamped, signed)
	 * command pulses. Differenced against the next setpoint to size the PR0
	 * profile velocity; `have_last` is false until the first successful write,
	 * when there is nothing to difference against. */
	int32_t last_pulses;
	bool have_last;
	/* Re-sends of `last_pulses` still allowed if the drive goes idle short of
	 * it (icl57_needs_retrigger). Refilled by each accepted setpoint, zeroed
	 * once the drive is seen to have taken it and by every stop. */
	uint8_t retriggers_left;
	uint32_t cmd_uptime_ms; /* when PR0 was last triggered */
	bool fb_running;        /* 0x1003 RUNNING at the last poll that read it */
	bool have_fb_state;
	/* Measured position in command pulses, cached from the feedback poll so the
	 * setpoint path can size a move against where the axis IS rather than a
	 * target it may never have reached. `fb_uptime_ms` is deliberately the 32-bit
	 * uptime: both fields are then single-word, so the setpoint thread can read
	 * what the feedback workqueue wrote without a lock. */
	int32_t fb_pulses;
	uint32_t fb_uptime_ms;
	bool have_fb;
	struct icl57_frame_watch frame_watch; /* an unconfirmed jump; see icl57_frame_check() */
	bool clamp_warned;         /* true while a clamped-setpoint warning is outstanding */
	bool limit_warned;         /* true while a limit-asserted warning is outstanding   */
	bool alarm_detect_checked; /* Pr4.22 and the identity registers, once per boot */
	/* Where the register-mapping block reads back from (ICL57_REG_MAP_VALUES_*),
	 * or 0 to poll with direct reads. Set by icl57_map_setup() at enable, only
	 * under CONFIG_ACTUATOR_ICL57_MAPPED_POLL, and cleared by the poll the moment
	 * the block fails its sentinel. */
	uint16_t map_base;
	uint16_t fb_di; /* Pr4.28 from the last mapped poll; meaningful when map_base != 0 */
	/* True while a homing routine owns this drive. Two independent reasons, both
	 * load-bearing: the manual states "control instruction can not be sent to
	 * drive during homing", and 0x1003's completion bits are read-and-clear, so
	 * the 100 ms feedback poller would consume ICL57_MOTION_HOMED before the wait
	 * loop could see it. */
	bool homing;
	/*
	 * True only between a fully successful `icl57 home` and anything that
	 * invalidates the datum. Boots false: the drives come up with position 0
	 * wherever the carriage happens to sit, so an un-homed axis's absolute
	 * setpoints mean something different every power-up. An application should
	 * refuse absolute setpoints until this is set: see icl57_is_homed().
	 *
	 * Cleared on a new homing attempt (a failed one must not leave the previous
	 * datum looking valid) and on a fault, since a stall means the carriage may
	 * have moved without the encoder frame following it.
	 */
	bool homed;
	/* Set by an e-stop after any accepted setpoint, or a failed abort re-pin; blocks
	 * re-enable until re-pinned or successfully homed. */
	bool repin_pending;
	/* Live motion profile: seeded from devicetree at init, overlaid by persisted
	 * settings before main(), edited by `icl57 speed` / `icl57 ramp`. Read on the
	 * setpoint path so a change takes effect on the next command with no drive
	 * write of its own (the PR0 block carries all three words every time). */
	struct icl57_tuning tuning;
	/* Live direction sign, seeded from the devicetree `invert-position` and
	 * flipped by `icl57 invert`. Deliberately NOT persisted, unlike the tuning
	 * above: a stored inversion would silently reverse an axis on the next
	 * power-up, and the direction is a wiring fact that belongs in devicetree
	 * once known. The shell flag is for identifying the sign on the bench. */
	bool invert;
	/* True between a COAST and the next NORMAL. Exists to make a redundant mode
	 * change free: the subsystem issues set_drive_mode(NORMAL) before EVERY
	 * setpoint (subsys/actuator/actuator.c), so writing Pr0.07 unconditionally
	 * would add a Modbus transaction to each one -- ten per second on a streamed
	 * axis -- and a single lost reply is reported as a driver fault that drops
	 * the axis out of READY while the drive itself is perfectly healthy. Observed
	 * exactly that on the bench: `actuator set` followed by "enable the axis
	 * first", with alarm 0x0000 and 0x1003 bit1 still set. */
	bool coasting;
	/* Host-side zero, in post-invert radians: the drive-frame position that the
	 * caller's 0 maps to. Added on the command path and subtracted on the
	 * feedback path so both frames stay consistent -- which is exactly what the
	 * drive's own origin trigger (Pr8.02 <- 0x21) fails to do, and why that one
	 * must never be used. Deliberately NOT persisted: the drives reset their own
	 * position to 0 on power-up, so a stored offset would silently outlive the
	 * frame it was measured in. Losing it on reboot is the safer failure. */
	float zero_rad;
};

struct icl57_config {
	struct actuator_cb_offsets cb_offsets; /* must be first */
	const char *iface_name;
	uint32_t baud;
	uint32_t rx_timeout_us;
	uint8_t slave_id;
	uint32_t caps;
	uint32_t update_period_ms;
	uint32_t command_period_ms;
	int32_t counts_per_rev;
	int32_t pos_min_milli;
	int32_t pos_max_milli;
	uint16_t profile_velocity_rpm;
	uint16_t profile_velocity_max_rpm;
	uint16_t accel;
	uint16_t decel;
	uint16_t homing_velocity_rpm;
	uint16_t homing_approach_rpm;
	int32_t homing_travel_limit_milli;
	int32_t homing_preroll_milli;
	int32_t homing_group; /* 0 = independent; a shared non-zero value MUST be homed together */
	bool home_dir_negative;
	/* Which ROTATION SENSE travels toward home; Pr8.10 bit0. A separate
	 * mechanical fact from home_dir_negative -- see the binding. */
	bool home_search_cw;
	/* The home input is a LIMIT switch at the end of travel, not a mid-travel
	 * home switch: DI2 is provisioned PL/NL rather than ORG and Pr8.10 bit2 is
	 * cleared. See the binding. */
	bool home_input_is_limit;
	bool home_switch_no;            /* home-switch-normally-open */
	bool enable_input_no;           /* enable-input-normally-open */
	int32_t homing_clearance_milli; /* PR0 clearance step for a limit home */
	/* Pr8.22 / Pr8.23 stop ramps in ms, or -1 to leave the drive's stored value
	 * alone and only report it. See the binding for why no board sets them. */
	int32_t limit_stop_ms;
	int32_t quick_stop_ms;
	/* Pr0.05, the following-error fault threshold in pulses, or -1 to leave
	 * and report. The shipped drives hold 65535, the
	 * register's maximum, so the bit2 detection `icl57 setup` arms cannot fire
	 * until something lowers this. */
	int32_t max_follow_err;
	bool invert; /* boot default for icl57_data.invert */
};

/* First initialised interface, for the provisioning shell (single bus). */
extern int icl57_shell_iface;

/* Bring up the Modbus client on `iface` once, however many motors share it. */
int icl57_client_init_once(int iface, const struct icl57_config *cfg);

#if defined(CONFIG_ACTUATOR_ICL57_BUS_STATS)
void icl57_mb_stats_write_gap(uint32_t gap_ms);
#endif

/* Every instance, for the by-slave-id lookup and the `all` form of the tuning
 * shell commands (and the settings loader, which keys on slave id). */
extern const struct device *const icl57_devs[];

/* icl57_devs[] is visible to icl57_shell.c only as an extern array of
 * incomplete type, so ARRAY_SIZE(icl57_devs) cannot be taken there. */
size_t icl57_dev_count(void);

const struct device *icl57_dev_by_id(uint8_t id);

/*
 * Modbus wrappers. USE THESE, never modbus_*_holding_reg*() directly.
 *
 * Zephyr's client returns a NEGATIVE errno for a transport failure but a
 * POSITIVE Modbus exception code when the drive REJECTS the request
 * (modbus_client.c:mbc_validate_response_fc). So the obvious `if (rc < 0)` test
 * reads a rejection as success, and it was written that way at 7 call sites.
 *
 * On a read that is worse than losing the error, because the client never fills
 * the caller's buffer on a rejected transaction: the caller then composes a
 * value out of whatever was on the stack and is told it is valid. That is not
 * hypothetical. icl57_read_u32() did exactly this, and its garbage positions
 * are what `icl57_wait_arrived()` was comparing against the target -- so a
 * pre-move that had physically ARRIVED was reported as "pre-move did not
 * arrive", and homes aborted on a measurement failure that looked like a motion
 * failure: positions read +24956 pulses against a tape-measured 4 mm.
 *
 * The shell's `icl57 rd` already defends against the same thing by zeroing its
 * buffer (see cmd_icl57_rd) -- that fix was made for the identical symptom in
 * the 0x6xxx block and never reached the driver. These wrappers close it for
 * everyone: buffer zeroed before the call, and any non-zero rc normalised to a
 * negative errno so `< 0` and `!= 0` both behave.
 */
int icl57_mb_read_regs(int iface, uint8_t id, uint16_t addr, uint16_t *regs, uint16_t qty);
int icl57_mb_write_reg(int iface, uint8_t id, uint16_t addr, uint16_t val);
int icl57_mb_write_regs(int iface, uint8_t id, uint16_t addr, uint16_t *regs, uint16_t qty);

int icl57_read_u32(int iface, uint8_t id, uint16_t addr, int32_t *out);

int icl57_read_feedback(const struct device *dev, struct actuator_feedback *out);

int icl57_read_motion_state(const struct device *dev, uint16_t *state);

/* Homing claim/release and the raw move primitive, called from icl57_shell.c. */
void icl57_homing_begin(struct icl57_data *d);
void icl57_homing_end(const struct device *dev);
/* Mark the datum established. Called only by `icl57 home` once every check on
 * the result has passed, never on a partial or aborted run. */
void icl57_homing_succeeded(const struct device *dev);
/* Record, and log, that an aborted home left a queued target in place;
 * `repin_rc` is icl57_repin()'s refusal, so the log says which kind. */
void icl57_repin_skipped(const struct device *dev, int repin_rc);
int icl57_read_pulses(const struct device *dev, int32_t *pulses);
int icl57_read_reported_pulses(const struct device *dev, int32_t *pulses);
int icl57_read_command_pulses(const struct device *dev, int32_t *pulses);
int icl57_pr0_move(const struct device *dev, int32_t pulses, uint16_t rpm);

/* Write one holding register, then read it back and report the result; the
 * readback is skipped for registers that latch an action. `sh` may be NULL. */
int icl57_write_verified(const struct shell *sh, uint8_t id, uint16_t addr, uint16_t val,
			 const char *what);

/* icl57_provision.c, behind `icl57 setup` and `icl57 save`. `sh` may be NULL. */
int icl57_provision(const struct shell *sh, uint8_t id);
int icl57_save(const struct shell *sh, uint8_t id);

/* icl57_homing.c, shared with icl57 setup / home / unjam. `sh` may be NULL. */
uint16_t icl57_di2_func(const struct icl57_config *cfg);
int icl57_write_homing_mode(const struct shell *sh, const struct device *dev, uint8_t id);
int icl57_home_run(const struct shell *sh, const struct device *dev);
int icl57_unjam(const struct shell *sh, const struct device *dev);

#endif /* ICL57_INTERNAL_H_ */
