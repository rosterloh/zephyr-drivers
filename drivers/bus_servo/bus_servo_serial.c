#include <errno.h>
#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/time_units.h>

#include "bus_servo_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bus_servo, CONFIG_BUS_SERVO_LOG_LEVEL);

#define BUS_SERVO_STATUS_OVERHEAD 5
#define BUS_SERVO_BITS_PER_BYTE   11
#define BUS_SERVO_RX_DRAIN_LIMIT  BUS_SERVO_MAX_PACKET_SIZE

/* Time to wait for a unicast write's status packet to start arriving before
 * assuming the servo is configured not to reply, and the idle gap that marks
 * the end of that reply once bytes have been seen.
 */
#define BUS_SERVO_REPLY_START_US 2000
#define BUS_SERVO_BUS_IDLE_US    200

static int set_tx_en(struct bus_servo_context *ctx, int value)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;

	if (cfg->tx_en == NULL) {
		return 0;
	}

	return gpio_pin_set_dt(cfg->tx_en, value);
}

static int configure_gpio(struct bus_servo_context *ctx)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;

	if (cfg->tx_en == NULL) {
		return 0;
	}
	if (!device_is_ready(cfg->tx_en->port)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(cfg->tx_en, GPIO_OUTPUT_INACTIVE);
}

int bus_servo_serial_init(struct bus_servo_context *ctx, struct bus_servo_iface_param param)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;
	struct uart_config uart_cfg = {
		.baudrate = param.serial.baud,
		.parity = param.serial.parity,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	int rc;

	if (!device_is_ready(cfg->dev)) {
		return -ENODEV;
	}

	rc = uart_configure(cfg->dev, &uart_cfg);
	if (rc != 0) {
		return rc;
	}

	rc = configure_gpio(ctx);
	if (rc != 0) {
		return rc;
	}

	cfg->uart_buf_len = 0;
	cfg->baud = param.serial.baud;
	return 0;
}

static void wait_tx_frame_time(struct bus_servo_context *ctx, size_t tx_len)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;
	uint64_t delay_us;

	if (cfg->tx_en == NULL || cfg->baud == 0) {
		return;
	}

	delay_us = DIV_ROUND_UP((uint64_t)tx_len * BUS_SERVO_BITS_PER_BYTE * 1000000ULL,
				(uint64_t)cfg->baud);
	k_busy_wait((uint32_t)MIN(delay_us, (uint64_t)UINT32_MAX));
}

static int drain_rx(struct bus_servo_context *ctx)
{
	unsigned char byte;

	for (size_t i = 0; i < BUS_SERVO_RX_DRAIN_LIMIT; i++) {
		int rc = uart_poll_in(ctx->cfg->dev, &byte);

		if (rc == -1) {
			return 0;
		}
		if (rc < 0) {
			return rc;
		}
	}

	return -EIO;
}

static int tx_packet(struct bus_servo_context *ctx, const uint8_t *tx_buf, size_t tx_len)
{
	int rc;

	rc = set_tx_en(ctx, 1);
	if (rc != 0) {
		return rc;
	}

	for (size_t i = 0; i < tx_len; i++) {
		uart_poll_out(ctx->cfg->dev, tx_buf[i]);
	}

	wait_tx_frame_time(ctx, tx_len);

	rc = set_tx_en(ctx, 0);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

static int validate_status_packet(struct bus_servo_context *ctx)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;
	const uint8_t *buf = cfg->uart_buf;
	uint8_t length;
	size_t param_len;
	uint8_t checksum;

	if (cfg->uart_buf_len < BUS_SERVO_STATUS_OVERHEAD + 1) {
		return -EBADMSG;
	}
	if (buf[0] != 0xff || buf[1] != 0xff) {
		return -EBADMSG;
	}
	if (buf[2] != ctx->expected_id) {
		return -EBADMSG;
	}

	length = buf[3];
	if (length < 2 || cfg->uart_buf_len != (size_t)length + 4 ||
	    cfg->uart_buf_len > BUS_SERVO_MAX_PACKET_SIZE) {
		return -EBADMSG;
	}

	checksum = bus_servo_checksum(&buf[2], (size_t)length + 1);
	if (checksum != buf[cfg->uart_buf_len - 1]) {
		return -EIO;
	}

	param_len = length - 2;
	ctx->status_error = buf[4];
	ctx->response_param_len = param_len;
	if (param_len > 0) {
		memcpy(ctx->response_params, &buf[5], param_len);
	}

	return 0;
}

static int rx_status_packet(struct bus_servo_context *ctx)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;
	int64_t deadline = k_uptime_ticks() + k_us_to_ticks_ceil64(ctx->rx_timeout_us);

	cfg->uart_buf_len = 0;

	while (k_uptime_ticks() <= deadline) {
		unsigned char byte;
		int rc;

		if (cfg->uart_buf_len >= BUS_SERVO_MAX_PACKET_SIZE) {
			return -EMSGSIZE;
		}

		rc = uart_poll_in(cfg->dev, &byte);
		if (rc < 0 && rc != -1) {
			return rc;
		}
		if (rc == 0) {
			cfg->uart_buf[cfg->uart_buf_len] = byte;
			cfg->uart_buf_len++;

			if (cfg->uart_buf_len >= 2 &&
			    (cfg->uart_buf[0] != 0xff || cfg->uart_buf[1] != 0xff)) {
				return -EBADMSG;
			}
			if (cfg->uart_buf_len >= 3 && cfg->uart_buf[2] != ctx->expected_id) {
				return -EBADMSG;
			}
			if (cfg->uart_buf_len >= 4) {
				uint8_t length = cfg->uart_buf[3];
				size_t packet_len = (size_t)length + 4;

				if (length < 2 || packet_len > BUS_SERVO_MAX_PACKET_SIZE) {
					return -EBADMSG;
				}
				if (cfg->uart_buf_len >= packet_len) {
					if (cfg->uart_buf_len > packet_len) {
						return -EBADMSG;
					}
					return validate_status_packet(ctx);
				}
			}
		}

		k_busy_wait(50);
	}

	return -ETIMEDOUT;
}

void bus_servo_drain_reply(struct bus_servo_context *ctx)
{
	struct bus_servo_serial_config *cfg = ctx->cfg;
	int64_t start_deadline = k_uptime_ticks() + k_us_to_ticks_ceil64(BUS_SERVO_REPLY_START_US);
	int64_t idle_deadline = 0;
	bool seen = false;
	unsigned char byte;

	/* A unicast write leaves a status packet on the shared half-duplex bus
	 * (unless the servo's status-return is disabled). Consume it so it does
	 * not collide with the next transaction's transmission. Wait up to
	 * BUS_SERVO_REPLY_START_US for the reply to begin, then discard bytes
	 * until the bus has been idle for BUS_SERVO_BUS_IDLE_US.
	 */
	while (true) {
		int rc = uart_poll_in(cfg->dev, &byte);

		if (rc == 0) {
			seen = true;
			idle_deadline =
				k_uptime_ticks() + k_us_to_ticks_ceil64(BUS_SERVO_BUS_IDLE_US);
			continue;
		}
		if (rc != -1) {
			return; /* unexpected UART error, give up */
		}
		if (seen) {
			if (k_uptime_ticks() >= idle_deadline) {
				return; /* reply received, bus idle */
			}
		} else if (k_uptime_ticks() >= start_deadline) {
			return; /* servo did not reply */
		}
		k_busy_wait(20);
	}
}

int bus_servo_tx_rx(struct bus_servo_context *ctx, const uint8_t *tx_buf, size_t tx_len,
		    bool expect_response)
{
	int rc;

	if (ctx == NULL || tx_buf == NULL || tx_len == 0 || tx_len > BUS_SERVO_MAX_PACKET_SIZE) {
		return -EINVAL;
	}

	rc = drain_rx(ctx);
	if (rc != 0) {
		return rc;
	}

	rc = tx_packet(ctx, tx_buf, tx_len);
	if (rc != 0) {
		return rc;
	}

	if (!expect_response) {
		return 0;
	}

	return rx_status_packet(ctx);
}
