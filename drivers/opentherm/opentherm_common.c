/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>

#include "opentherm_common.h"

/* Edges closer than this to the last mid-bit edge are bit boundaries. */
#define OT_MID_MIN_US 750U
/* Spec allows mid-bit edges 900..1150 us apart; anything later lost an edge. */
#define OT_MID_MAX_US 1250U
#define OT_MSG_GAP_MS 100

bool ot_rx_edge(struct ot_rx *rx, uint32_t t_cyc, int level)
{
	uint32_t gap = k_cyc_to_us_floor32(t_cyc - rx->t_mid);

	switch (rx->state) {
	case OT_RX_WAIT:
		if (level == 1) {
			rx->state = OT_RX_START;
			rx->t_mid = t_cyc;
		}
		break;
	case OT_RX_START:
		if (level == 0 && gap < OT_MID_MIN_US) {
			rx->state = OT_RX_DATA;
			rx->frame = 0U;
			rx->bits = 0U;
		} else {
			/* Glitch, not a start bit: wait for the next rising edge. */
			rx->state = (level == 1) ? OT_RX_START : OT_RX_WAIT;
		}
		rx->t_mid = t_cyc;
		break;
	case OT_RX_DATA:
		if (gap < OT_MID_MIN_US) {
			break;
		}
		rx->t_mid = t_cyc;
		if (gap > OT_MID_MAX_US) {
			rx->err = -EBADMSG;
		} else if (rx->bits < 32U) {
			/* '1' is active then idle, so the line is idle after its mid-bit edge. */
			rx->frame = (rx->frame << 1) | (level == 0 ? 1U : 0U);
			rx->bits++;
			break;
		} else if (level != 0) {
			rx->err = -EBADMSG; /* stop bit must be '1' */
		}
		rx->state = OT_RX_DONE;
		return true;
	default:
		break;
	}

	return false;
}

int ot_rx_result(const struct ot_rx *rx, uint32_t *response)
{
	if (rx->err != 0) {
		return rx->err;
	}
	if ((__builtin_popcount(rx->frame) & 1) != 0) {
		return -EBADMSG;
	}
	*response = rx->frame;
	return 0;
}

void ot_msg_gap(int64_t last_ms)
{
	int64_t gap = k_uptime_get() - last_ms;

	if (gap < OT_MSG_GAP_MS) {
		k_msleep((int32_t)(OT_MSG_GAP_MS - gap));
	}
}
