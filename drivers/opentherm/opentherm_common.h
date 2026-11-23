/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared OpenTherm v2.2 line coding. Levels are logical: 1 is the first half
 * of a '1' bit and the line idles at 0.
 */

#ifndef ZEPHYR_DRIVERS_OPENTHERM_COMMON_H_
#define ZEPHYR_DRIVERS_OPENTHERM_COMMON_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define OT_HALF_BIT_US 500U
#define OT_FRAME_BITS  34U /* start + 32 data + stop */
/* Frame time plus the slave's 800 ms reply window, with margin. */
#define OT_REPLY_MS    1100

enum ot_rx_state {
	OT_RX_WAIT,
	OT_RX_START,
	OT_RX_DATA,
	OT_RX_DONE,
};

/* Manchester decoder fed one edge at a time; timestamps in hardware cycles. */
struct ot_rx {
	enum ot_rx_state state;
	uint32_t t_mid;
	uint32_t frame;
	uint8_t bits;
	int err;
};

/* Line level (0/1) of half-bit @p half of the 34-bit frame around @p frame. */
static inline int ot_tx_level(uint32_t frame, uint32_t half)
{
	/* Start and stop bits are both '1'. */
	uint64_t word = BIT64(33) | ((uint64_t)frame << 1) | 1U;
	bool one = ((word >> (OT_FRAME_BITS - 1U - half / 2U)) & 1U) != 0U;

	return (((half & 1U) == 0U) == one) ? 1 : 0;
}

static inline void ot_rx_reset(struct ot_rx *rx)
{
	rx->state = OT_RX_WAIT;
	rx->err = 0;
}

/**
 * Feed the line level after an edge at @p t_cyc.
 *
 * @retval true Frame finished: rx->err is 0 and rx->frame holds it, or
 *         rx->err is -EBADMSG.
 * @retval false Need more edges.
 */
bool ot_rx_edge(struct ot_rx *rx, uint32_t t_cyc, int level);

/* Check parity of a decoded frame and copy it out. */
int ot_rx_result(const struct ot_rx *rx, uint32_t *response);

/* Sleep until the spec's 100 ms gap since @p last_ms has passed. */
void ot_msg_gap(int64_t last_ms);

#endif /* ZEPHYR_DRIVERS_OPENTHERM_COMMON_H_ */
