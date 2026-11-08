/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAPTURE_DISPLAY_H_
#define CAPTURE_DISPLAY_H_

#include <stdbool.h>
#include <zephyr/drivers/display.h>

#define CAP_W 100
#define CAP_H 24

struct capture_state {
	/* Set by the test. */
	enum display_pixel_format fmt;
	uint32_t screen_info;
	int blanking_off_ret;
	/* Recorded. */
	bool white[CAP_H][CAP_W];
	bool blanked;
	int refreshes;
	int writes;
	bool last_incomplete;
};

extern struct capture_state capture;

#endif
