/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modbus transport for the iCL-RS driver: the per-bus client, the wrappers
 * every transaction goes through, the bus lock, and the optional bus stats.
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/actuator/actuator.h>

#include "icl57_profile.h"
#include "icl57_internal.h"

LOG_MODULE_DECLARE(actuator_icl57, CONFIG_ACTUATOR_LOG_LEVEL);

/* One Modbus client per bus, shared by every motor on it. The first instance
 * to initialise brings the interface up; the bit tracks which ifaces are done. */
static atomic_t icl57_iface_inited;
/* First initialised interface, for the provisioning shell (single bus). */
int icl57_shell_iface = -1;

int icl57_client_init_once(int iface, const struct icl57_config *cfg)
{
	if (atomic_test_and_set_bit(&icl57_iface_inited, iface)) {
		return 0;
	}

	struct modbus_iface_param param = {
		.mode = MODBUS_MODE_RTU,
		.rx_timeout = cfg->rx_timeout_us,
		.serial =
			{
				.baud = cfg->baud,
				.parity = UART_CFG_PARITY_NONE,
				/* 8N2: an 11-bit RTU character, and what `icl57 setup`
				 * provisions on every drive (Pr5.24 = 5). With
				 * CONFIG_MODBUS_NONCOMPLIANT_SERIAL_MODE off, Zephyr's
				 * modbus_serial.c ignores this field and forces 2 stop bits
				 * for parity NONE -- the only reason STOP_BITS_1 here ever
				 * worked. Enabling that option would have made it live and
				 * dropped the master to 8N1 against 8N2 drives. */
				.stop_bits = UART_CFG_STOP_BITS_2,
			},
	};
	int rc = modbus_init_client(iface, param);

	if (rc < 0) {
		atomic_clear_bit(&icl57_iface_inited, iface);
		LOG_ERR("modbus client init failed on '%s': %d", cfg->iface_name, rc);
		return rc;
	}

	if (icl57_shell_iface < 0) {
		icl57_shell_iface = iface;
	}
	return 0;
}

/*
 * A rejected request comes back as a POSITIVE Modbus exception code, so fold
 * every non-zero return into a negative errno. See the header for what this
 * cost when it was not done.
 */
static int icl57_mb_rc(int rc)
{
	if (rc == 0) {
		return 0;
	}
	return rc < 0 ? rc : -EIO;
}

#if defined(CONFIG_ACTUATOR_ICL57_BUS_STATS)
/* Shared by every motor on the bus; callers hold the bus lock, and a rare race
 * only skews a diagnostic count. */
static uint32_t mbs_n, mbs_fail, mbs_slow, mbs_max_ms, mbs_win_ms, mbs_max_wgap;

static int icl57_mb_timed(int rc, uint32_t t0, char kind, uint8_t id, uint16_t addr)
{
	uint32_t now = k_uptime_get_32();
	uint32_t ms = now - t0;

	mbs_n++;
	mbs_max_ms = MAX(mbs_max_ms, ms);
	mbs_fail += rc != 0;
	mbs_slow += ms > 30U;
	if (rc != 0 || ms > 30U) {
		LOG_INF("mb %c id %u reg 0x%04x rc %d took %u ms", kind, id, addr, rc, ms);
	}
	if (now - mbs_win_ms >= 1000U) {
		LOG_INF("mb stats: %u txn, %u failed, %u slow, max %u ms, max write gap %u ms",
			mbs_n, mbs_fail, mbs_slow, mbs_max_ms, mbs_max_wgap);
		mbs_n = mbs_fail = mbs_slow = mbs_max_ms = mbs_max_wgap = 0U;
		mbs_win_ms = now;
	}
	return rc;
}
/* The longest gap between two setpoint writes, for the next summary line. */
void icl57_mb_stats_write_gap(uint32_t gap_ms)
{
	mbs_max_wgap = MAX(mbs_max_wgap, gap_ms);
}

#define MB_RC(expr, kind, id, addr)                                                                \
	({                                                                                         \
		uint32_t _t0 = k_uptime_get_32();                                                  \
		icl57_mb_timed(icl57_mb_rc(expr), _t0, kind, id, addr);                            \
	})
#else
#define MB_RC(expr, kind, id, addr) icl57_mb_rc(expr)
#endif

int icl57_mb_read_regs(int iface, uint8_t id, uint16_t addr, uint16_t *regs, uint16_t qty)
{
	/* Zeroed BEFORE the call, not after a failure: the client does not fill
	 * the buffer on a rejected transaction, and a caller that ignores the
	 * return then reads whatever the stack held. Zeros are still wrong, but
	 * they are consistently wrong and do not masquerade as a plausible
	 * position. */
	memset(regs, 0, (size_t)qty * sizeof(*regs));

	return MB_RC(modbus_read_holding_regs(iface, id, addr, regs, qty), 'r', id, addr);
}

int icl57_mb_write_reg(int iface, uint8_t id, uint16_t addr, uint16_t val)
{
	return MB_RC(modbus_write_holding_reg(iface, id, addr, val), 'w', id, addr);
}

int icl57_mb_write_regs(int iface, uint8_t id, uint16_t addr, uint16_t *regs, uint16_t qty)
{
	return MB_RC(modbus_write_holding_regs(iface, id, addr, regs, qty), 'W', id, addr);
}

int icl57_read_u32(int iface, uint8_t id, uint16_t addr, int32_t *out)
{
	uint16_t regs[2] = {0};
	int rc = icl57_mb_read_regs(iface, id, addr, regs, 2);

	if (rc != 0) {
		return rc;
	}

	/* A direct block read of the PR region comes back pair-transposed, so the
	 * high word is the SECOND one there. See icl57_pr_block_index(). */
	if (addr >= ICL57_REG_PR_BLOCK_BASE) {
		*out = (int32_t)(((uint32_t)regs[icl57_pr_block_index(0)] << 16) |
				 regs[icl57_pr_block_index(1)]);
	} else {
		*out = (int32_t)(((uint32_t)regs[0] << 16) | regs[1]);
	}
	return 0;
}

/*
 * Registers that LATCH AN ACTION rather than hold a setting. They do not read
 * back as written -- the drive consumes the code and clears or replaces it --
 * so verifying them would report a false mismatch on every write.
 */
static bool icl57_reg_is_command(uint16_t addr)
{
	return addr == ICL57_REG_CONTROL_WORD || addr == ICL57_REG_TRIGGER;
}

/*
 * Write a holding register, then READ IT BACK and compare.
 *
 * A Modbus ack alone does not mean the drive stored anything: that gap leaves a
 * drive that "looks provisioned and behaves unprovisioned", and it applies to
 * every register this writes.
 *
 * Read qty 2 in the 0x6xxx block, never 1: the drive REJECTS a single-register
 * read of the PR block (exception 8), and the pair it does return is
 * TRANSPOSED, so the written register is the SECOND word -- see
 * icl57_pr_block_index(). Comparing the first word reports the NEXT register's
 * value as a mismatch on every Pr8 write, which reads exactly like block and
 * per-register reads of 0x6xxx disagreeing. They do not.
 *
 * CAVEAT, and it is a large one: 0x6xxx reads also MISPAIR while the feedback
 * poller runs on any other axis, and they lie convincingly -- twice now that has
 * been misread as "setup's writes are not landing". So a mismatch reported here
 * is only trustworthy with the bus quiet (`actuator disable` on the other axes).
 * The message says so, because this function cannot tell the difference.
 *
 * A mismatch is reported and NOT made fatal. `icl57_provision()` returns on the
 * first error, and the Pr8 block is written before DI2, so failing hard here
 * would abort provisioning before the DI2 function -- leaving the drive on
 * whatever function code it already had while the firmware assumed the new one.
 * That is a worse failure than the one being detected. The operator gets every
 * register's verdict instead.
 */
int icl57_write_verified(const struct shell *sh, uint8_t id, uint16_t addr, uint16_t val,
			 const char *what)
{
	if (icl57_shell_iface < 0) {
		ICL57_ERROR(sh, "modbus bus not initialised");
		return -ENODEV;
	}
	if (id < 1 || id > 31) {
		ICL57_ERROR(sh, "slave id must be 1..31");
		return -EINVAL;
	}
	int rc = icl57_mb_write_reg(icl57_shell_iface, id, addr, val);

	if (rc < 0) {
		ICL57_ERROR(sh, "%s (id %u) failed: %d", what, id, rc);
		return rc;
	}

	if (icl57_reg_is_command(addr)) {
		ICL57_PRINT(sh, "%s (id %u) ok", what, id);
		return 0;
	}

	uint16_t back[2] = {0};
	uint16_t qty = (addr >= ICL57_REG_PR_BLOCK_BASE) ? 2U : 1U;

	if (icl57_mb_read_regs(icl57_shell_iface, id, addr, back, qty) < 0) {
		ICL57_WARN(sh, "%s (id %u): written, but READ-BACK FAILED -- value UNVERIFIED",
			   what, id);
		return 0;
	}

	/* A PR-block pair comes back transposed, so the register that was written
	 * is the second word of a qty-2 read. See icl57_pr_block_index(). */
	uint16_t got = (qty == 2U) ? back[icl57_pr_block_index(0)] : back[0];

	if (got != val) {
		ICL57_ERROR(sh, "%s (id %u): wrote 0x%04x, reg 0x%04x reads 0x%04x", what, id, val,
			    addr, got);
		return 0;
	}

	ICL57_PRINT(sh, "%s (id %u) ok, verified 0x%04x", what, id, val);
	return 0;
}

/* Without it a poll's reads can land between two paired setpoint writes:
 * measured on about half of all moves, putting them ~130 ms apart (14 ms bare),
 * which at 180 rpm is 2.4 rad of real racking before the second motor starts. */
static K_MUTEX_DEFINE(icl57_bus_mutex);

void icl57_bus_lock(void)
{
	k_mutex_lock(&icl57_bus_mutex, K_FOREVER);
}

void icl57_bus_unlock(void)
{
	k_mutex_unlock(&icl57_bus_mutex);
}
