/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulated boiler on gpio_emul: a timer samples the master's output pin, a
 * thread decodes the request and a second timer drives the reply.
 */

#include <errno.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/opentherm.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define OT_NODE DT_NODELABEL(opentherm0)

static const struct device *const ot = DEVICE_DT_GET(OT_NODE);
static const struct gpio_dt_spec out = GPIO_DT_SPEC_GET(OT_NODE, out_gpios);
static const struct gpio_dt_spec in = GPIO_DT_SPEC_GET(OT_NODE, in_gpios);

#define SAMPLE_US 50U
#define MAX_EDGES 80

/* Boiler behaviour for the next request. */
static struct {
	bool silent;
	bool flip_parity;
	uint32_t half_bit_us;
} boiler;

static struct {
	uint32_t t[MAX_EDGES];
	uint8_t level[MAX_EDGES];
	size_t n;
	int last;
} rx;

static uint32_t last_request;
static uint8_t tx_halves[68];
static size_t tx_pos;

static int out_logical(void)
{
	/* gpio_emul reports the physical level; the out pin is active low. */
	return gpio_emul_output_get_dt(&out) == 0 ? 1 : 0;
}

static void sample(struct k_timer *timer)
{
	uint32_t now = k_cyc_to_us_floor32(k_cycle_get_32());
	int level = out_logical();

	ARG_UNUSED(timer);

	if (level != rx.last) {
		if (rx.n < MAX_EDGES) {
			rx.t[rx.n] = now;
			rx.level[rx.n] = (uint8_t)level;
			rx.n++;
		}
		rx.last = level;
	}
}

K_TIMER_DEFINE(sampler, sample, NULL);

static int level_at(uint32_t t)
{
	int level = 0;

	for (size_t i = 0; i < rx.n && rx.t[i] <= t; i++) {
		level = rx.level[i];
	}
	return level;
}

static uint32_t decode_request(void)
{
	uint32_t t0 = rx.t[0];
	uint32_t frame = 0U;

	zassert_equal(rx.level[0], 1, "request must start active");
	zassert_equal(level_at(t0 + 250U), 1, "start bit");
	zassert_equal(level_at(t0 + 33U * 1000U + 250U), 1, "stop bit");
	for (uint32_t bit = 1U; bit <= 32U; bit++) {
		frame = (frame << 1) | (uint32_t)level_at(t0 + bit * 1000U + 250U);
	}
	return frame;
}

static void drive(struct k_timer *timer)
{
	if (tx_pos < ARRAY_SIZE(tx_halves)) {
		gpio_emul_input_set_dt(&in, tx_halves[tx_pos++]);
	} else {
		gpio_emul_input_set_dt(&in, 0);
		k_timer_stop(timer);
	}
}

K_TIMER_DEFINE(driver, drive, NULL);

static void send_reply(uint32_t frame)
{
	uint64_t word = BIT64(33) | ((uint64_t)frame << 1) | 1U;

	for (size_t bit = 0U; bit < 34U; bit++) {
		bool one = ((word >> (33U - bit)) & 1U) != 0U;

		tx_halves[2U * bit] = one ? 1U : 0U;
		tx_halves[2U * bit + 1U] = one ? 0U : 1U;
	}
	tx_pos = 0U;
	k_timer_start(&driver, K_NO_WAIT, K_USEC(boiler.half_bit_us));
}

static void boiler_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (true) {
		uint32_t req;
		uint32_t reply;

		/* A request is complete once the line has been idle for 2 ms. */
		do {
			k_msleep(1);
		} while (rx.n == 0U || rx.last != 0 ||
			 k_cyc_to_us_floor32(k_cycle_get_32()) - rx.t[rx.n - 1U] < 2000U);

		req = decode_request();
		last_request = req;
		rx.n = 0U;

		if (boiler.silent) {
			continue;
		}

		reply = opentherm_frame(
			opentherm_frame_type(req) == OPENTHERM_READ_DATA ? OPENTHERM_READ_ACK
									 : OPENTHERM_WRITE_ACK,
			opentherm_frame_id(req),
			opentherm_frame_id(req) == 25U ? 0x2A80U /* 42.5 C */
						       : opentherm_frame_value(req));
		if (boiler.flip_parity) {
			reply ^= BIT(31);
		}

		k_msleep(30);
		send_reply(reply);
	}
}

K_THREAD_DEFINE(boiler_tid, 1024, boiler_thread, NULL, NULL, NULL, 5, 0, 0);

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	boiler.silent = false;
	boiler.flip_parity = false;
	boiler.half_bit_us = 500U;
	rx.n = 0U;
	rx.last = 0;
	k_timer_start(&sampler, K_USEC(SAMPLE_US), K_USEC(SAMPLE_US));
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	k_timer_stop(&sampler);
	k_timer_stop(&driver);
	gpio_emul_input_set_dt(&in, 0);
}

ZTEST(opentherm, test_frame_parity)
{
	zassert_equal(opentherm_frame(OPENTHERM_READ_DATA, 25, 0), 0x80190000U);
	zassert_equal(opentherm_frame(OPENTHERM_READ_DATA, 0, 0x0300), 0x00000300U);
	zassert_equal(opentherm_frame_type(0xC0190000U), OPENTHERM_READ_ACK);
	zassert_equal(opentherm_frame_id(0xC0192A80U), 25U);
	zassert_equal(opentherm_frame_value(0xC0192A80U), 0x2A80U);
}

ZTEST(opentherm, test_read)
{
	uint32_t req = opentherm_frame(OPENTHERM_READ_DATA, 25, 0);
	uint32_t resp = 0U;

	zassert_true(device_is_ready(ot));
	zassert_ok(opentherm_transfer(ot, req, &resp));
	zassert_equal(last_request, req, "boiler saw 0x%08x", last_request);
	zassert_equal(resp, opentherm_frame(OPENTHERM_READ_ACK, 25, 0x2A80), "got 0x%08x", resp);
}

ZTEST(opentherm, test_write_back_to_back)
{
	for (uint16_t v = 0x3C00U; v < 0x3C03U; v++) {
		uint32_t req = opentherm_frame(OPENTHERM_WRITE_DATA, 1, v);
		uint32_t resp;

		zassert_ok(opentherm_transfer(ot, req, &resp));
		zassert_equal(resp, opentherm_frame(OPENTHERM_WRITE_ACK, 1, v));
	}
}

ZTEST(opentherm, test_slow_slave_within_tolerance)
{
	uint32_t resp;

	/* 1.1 ms bits: inside the spec's 900..1150 us mid-bit window. */
	boiler.half_bit_us = 550U;
	zassert_ok(opentherm_transfer(ot, opentherm_frame(OPENTHERM_READ_DATA, 0, 0x0300), &resp));
	zassert_equal(opentherm_frame_type(resp), OPENTHERM_READ_ACK);
}

ZTEST(opentherm, test_bad_timing)
{
	uint32_t resp;

	boiler.half_bit_us = 700U;
	zassert_equal(opentherm_transfer(ot, opentherm_frame(OPENTHERM_READ_DATA, 0, 0), &resp),
		      -EBADMSG);
}

ZTEST(opentherm, test_bad_parity)
{
	uint32_t resp;

	boiler.flip_parity = true;
	zassert_equal(opentherm_transfer(ot, opentherm_frame(OPENTHERM_READ_DATA, 0, 0), &resp),
		      -EBADMSG);
}

ZTEST(opentherm, test_timeout)
{
	uint32_t resp;

	boiler.silent = true;
	zassert_equal(opentherm_transfer(ot, opentherm_frame(OPENTHERM_READ_DATA, 0, 0), &resp),
		      -ETIMEDOUT);
}

static volatile bool spin;
static uint32_t max_gap_us;

static void spinner(void *a, void *b, void *c)
{
	uint32_t last = k_cycle_get_32();

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (spin) {
		uint32_t now = k_cycle_get_32();

		max_gap_us = MAX(max_gap_us, k_cyc_to_us_floor32(now - last));
		last = now;
		k_busy_wait(50);
	}
}

K_THREAD_STACK_DEFINE(spinner_stack, 1024);

ZTEST(opentherm, test_transfer_does_not_block)
{
	static struct k_thread thread;
	uint32_t resp;

	/* A thread below every other one still runs while the frame goes out. */
	spin = true;
	max_gap_us = 0U;
	k_thread_create(&thread, spinner_stack, K_THREAD_STACK_SIZEOF(spinner_stack), spinner, NULL,
			NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	zassert_ok(opentherm_transfer(ot, opentherm_frame(OPENTHERM_READ_DATA, 0, 0), &resp));
	spin = false;
	k_thread_join(&thread, K_FOREVER);

	zassert_true(max_gap_us < 2000U, "spinner starved for %u us", max_gap_us);
}

ZTEST(opentherm, test_pulse_io_loopback)
{
	const struct device *pio = DEVICE_DT_GET(DT_NODELABEL(ot_pio));
	const struct device *inv = DEVICE_DT_GET(DT_NODELABEL(ot_pio_inv));
	const struct device *bad = DEVICE_DT_GET(DT_NODELABEL(ot_pio_mismatch));
	uint32_t req = opentherm_frame(OPENTHERM_WRITE_DATA, 1, 0x3C80);
	uint32_t resp = 0U;

	zassert_true(device_is_ready(pio) && device_is_ready(inv) && device_is_ready(bad));

	zassert_ok(opentherm_transfer(pio, req, &resp));
	zassert_equal(resp, req, "got 0x%08x", resp);

	resp = 0U;
	zassert_ok(opentherm_transfer(inv, req, &resp));
	zassert_equal(resp, req, "got 0x%08x", resp);

	zassert_equal(opentherm_transfer(bad, req, &resp), -EBADMSG);
}

ZTEST_SUITE(opentherm, NULL, NULL, before, after, NULL);
