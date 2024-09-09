#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "dynamixel_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(dynamixel, CONFIG_DYNAMIXEL_LOG_LEVEL);

/* http://emanual.robotis.com/docs/en/dxl/protocol2/#status-packet */
#define DYNAMIXEL_MIN_MSG_SIZE 11
/* https://emanual.robotis.com/docs/en/dxl/crc */
#define CRC_POLYNOMIAL         0x8005
#define CRC_SEED               0x0000

static void dxl_serial_tx_on(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;

	if (cfg->tx_en != NULL) {
		gpio_pin_set(cfg->tx_en->port, cfg->tx_en->pin, 1);
	}

	uart_irq_tx_enable(cfg->dev);
}

static void dxl_serial_tx_off(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;

	uart_irq_tx_disable(cfg->dev);
	if (cfg->tx_en != NULL) {
		gpio_pin_set(cfg->tx_en->port, cfg->tx_en->pin, 0);
	}
}

static void dxl_serial_rx_on(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;

	uart_irq_rx_enable(cfg->dev);
}

static void dxl_serial_rx_off(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;

	uart_irq_rx_disable(cfg->dev);
}

/* Copy frame and check if the CRC is valid. */
static int rx_frame(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;
	uint16_t calc_crc;
	uint16_t crc_idx;
	uint8_t *data_ptr;

	/* Is the message long enough? */
	if ((cfg->uart_buf_ctr < DYNAMIXEL_MIN_MSG_SIZE) ||
	    (cfg->uart_buf_ctr > CONFIG_DYNAMIXEL_BUFFER_SIZE)) {
		LOG_WRN("Frame length error. %d bytes", cfg->uart_buf_ctr);
		return -EMSGSIZE;
	}
	LOG_DBG("Reply frame received");
	LOG_HEXDUMP_DBG(cfg->uart_buf, cfg->uart_buf_ctr, "uart_buf");

	ctx->rx_frame.id = cfg->uart_buf[4];
	ctx->rx_frame.length = sys_get_le16(&cfg->uart_buf[5]);
	ctx->rx_frame.ic = cfg->uart_buf[7]; /* 0x55 is status packet */
	data_ptr = &cfg->uart_buf[8];
	/* CRC index */
	crc_idx = cfg->uart_buf_ctr - sizeof(uint16_t);

	memcpy(ctx->rx_frame.data, data_ptr, ctx->rx_frame.length);

	ctx->rx_frame.crc = sys_get_le16(&cfg->uart_buf[crc_idx]);
	/* Calculate CRC */
	calc_crc = crc16(CRC_POLYNOMIAL, CRC_SEED, &cfg->uart_buf[0],
			 cfg->uart_buf_ctr - sizeof(ctx->rx_frame.crc));

	if (ctx->rx_frame.crc != calc_crc) {
		LOG_WRN("Calculated CRC does not match received CRC");
		return -EIO;
	}

	return 0;
}

static void tx_frame(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;
	uint16_t tx_bytes = 0;
	uint8_t *data_ptr;

	cfg->uart_buf[0] = 0xFF; // Header 1
	cfg->uart_buf[1] = 0xFF; // Header 2
	cfg->uart_buf[2] = 0xFD; // Header 3
	cfg->uart_buf[3] = 0x00; // Reserved

	cfg->uart_buf[4] = ctx->tx_frame.id;
	sys_put_le16(ctx->tx_frame.length, &cfg->uart_buf[5]);
	cfg->uart_buf[7] = ctx->tx_frame.ic;
	/* The Length indicates the Byte size of Instruction, Parameters and CRC
	 * fields */
	tx_bytes = ctx->tx_frame.length + 7;
	data_ptr = &cfg->uart_buf[8];

	memcpy(data_ptr, ctx->tx_frame.data, ctx->tx_frame.length);

	ctx->tx_frame.crc = crc16(CRC_POLYNOMIAL, CRC_SEED, &cfg->uart_buf[0], tx_bytes - 2);
	sys_put_le16(ctx->tx_frame.crc, &cfg->uart_buf[tx_bytes - 2]);

	cfg->uart_buf_ctr = tx_bytes;
	cfg->uart_buf_ptr = &cfg->uart_buf[0];

	LOG_HEXDUMP_DBG(cfg->uart_buf, cfg->uart_buf_ctr, "uart_buf");
	LOG_DBG("Start frame transmission");

	dxl_serial_rx_off(ctx);
	dxl_serial_tx_on(ctx);
}

/*
 * A byte has been received from a serial port. We just store it in the buffer
 * for processing when a complete packet has been received.
 */
static void cb_handler_rx(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;
	int n;

	/* Restart timer on a new character */
	k_timer_start(&cfg->packet_timer, K_USEC(cfg->packet_timeout), K_NO_WAIT);

	n = uart_fifo_read(cfg->dev, cfg->uart_buf_ptr,
			   (CONFIG_DYNAMIXEL_BUFFER_SIZE - cfg->uart_buf_ctr));

	cfg->uart_buf_ptr += n;
	cfg->uart_buf_ctr += n;
}

static void cb_handler_tx(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;
	int n;

	if (cfg->uart_buf_ctr > 0) {
		n = uart_fifo_fill(cfg->dev, cfg->uart_buf_ptr, cfg->uart_buf_ctr);
		cfg->uart_buf_ctr -= n;
		cfg->uart_buf_ptr += n;
	} else {
		/* Disable transmission */
		cfg->uart_buf_ptr = &cfg->uart_buf[0];
		dxl_serial_tx_off(ctx);
		dxl_serial_rx_on(ctx);
	}
}

static void uart_cb_handler(const struct device *dev, void *app_data)
{
	struct dxl_context *ctx = (struct dxl_context *)app_data;
	struct dxl_serial_config *cfg;

	if (ctx == NULL) {
		LOG_ERR("Dynamixel hardware is not properly initialised");
		return;
	}

	cfg = ctx->cfg;

	while (uart_irq_update(cfg->dev) && uart_irq_is_pending(cfg->dev)) {

		if (uart_irq_rx_ready(cfg->dev)) {
			cb_handler_rx(ctx);
		}

		if (uart_irq_tx_ready(cfg->dev)) {
			cb_handler_tx(ctx);
		}
	}
}

/* This function is called when the packet timer expires. */
static void packet_tmr_handler(struct k_timer *t_id)
{
	struct dxl_context *ctx;

	ctx = (struct dxl_context *)k_timer_user_data_get(t_id);

	if (ctx == NULL) {
		LOG_ERR("Failed to get Dynamixel context");
		return;
	}

	k_work_submit(&ctx->handler_work);
}

static int configure_gpio(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;

	if (cfg->tx_en != NULL) {
		if (!device_is_ready(cfg->tx_en->port)) {
			return -ENODEV;
		}

		if (gpio_pin_configure_dt(cfg->tx_en, GPIO_OUTPUT_INACTIVE)) {
			return -EIO;
		}
	}

	return 0;
}

void dxl_serial_rx_disable(struct dxl_context *ctx)
{
	dxl_serial_rx_off(ctx);
}

void dxl_serial_rx_enable(struct dxl_context *ctx)
{
	dxl_serial_rx_on(ctx);
}

int dxl_serial_rx(struct dxl_context *ctx)
{
	struct dxl_serial_config *cfg = ctx->cfg;
	int rc = 0;

	rc = rx_frame(ctx);

	cfg->uart_buf_ctr = 0;
	cfg->uart_buf_ptr = &cfg->uart_buf[0];

	return rc;
}

int dxl_serial_tx(struct dxl_context *ctx)
{
	tx_frame(ctx);
	return 0;
}

int dxl_serial_init(struct dxl_context *ctx, struct dxl_iface_param param)
{
	struct dxl_serial_config *cfg = ctx->cfg;
	const uint32_t if_delay_max = 3500000;
	const uint32_t numof_bits = 11;
	struct uart_config uart_cfg;

	if (!device_is_ready(cfg->dev)) {
		LOG_ERR("Bus device %s is not ready", cfg->dev->name);
		return -ENODEV;
	}

	uart_cfg.baudrate = param.serial.baud, uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
	uart_cfg.parity = UART_CFG_PARITY_NONE;
	uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;

	if (uart_configure(cfg->dev, &uart_cfg) != 0) {
		LOG_ERR("Failed to configure UART");
		return -EINVAL;
	}

	if (param.serial.baud <= 38400) {
		cfg->packet_timeout = (numof_bits * if_delay_max) / param.serial.baud;
	} else {
		cfg->packet_timeout = (numof_bits * if_delay_max) / 38400;
	}

	if (configure_gpio(ctx) != 0) {
		return -EIO;
	}

	cfg->uart_buf_ctr = 0;
	cfg->uart_buf_ptr = &cfg->uart_buf[0];

	uart_irq_callback_user_data_set(cfg->dev, uart_cb_handler, ctx);
	k_timer_init(&cfg->packet_timer, packet_tmr_handler, NULL);
	k_timer_user_data_set(&cfg->packet_timer, ctx);

	dxl_serial_rx_on(ctx);
	LOG_INF("Packet timeout %u us", cfg->packet_timeout);

	return 0;
}

void dxl_serial_disable(struct dxl_context *ctx)
{
	dxl_serial_tx_off(ctx);
	dxl_serial_rx_off(ctx);
	k_timer_stop(&ctx->cfg->packet_timer);
}