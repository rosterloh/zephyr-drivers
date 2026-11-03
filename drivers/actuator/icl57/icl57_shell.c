/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell commands for the iCL-RS driver (drivers/actuator/icl57): provisioning
 * (setup/save/estop; the work is in icl57_provision.c), raw register access
 * (rd/wr/praw/alarm/baud), and motion-profile tuning (speed/ramp/invert/
 * set-zero). Commands stay thin: behaviour belongs in the driver files.
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

static int cmd_icl57_save(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return icl57_save(sh, (uint8_t)strtoul(argv[1], NULL, 0));
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

static int cmd_icl57_setup(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return icl57_provision(sh, (uint8_t)strtoul(argv[1], NULL, 0));
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
