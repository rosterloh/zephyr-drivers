#ifndef ZEPHYR_INCLUDE_DYNAMIXEL_INTERNAL_H_
#define ZEPHYR_INCLUDE_DYNAMIXEL_INTERNAL_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/dynamixel.h>

struct dxl_serial_config {
	/* UART device */
	const struct device *dev;
	/* Packet timeout (maximum inter-frame delay) */
	uint32_t packet_timeout;
	/* Pointer to current position in buffer */
	uint8_t *uart_buf_ptr;
	/* Pointer to driver enable (TX_EN) pin config */
	struct gpio_dt_spec *tx_en;
	/* Timer to detect frame end point */
	struct k_timer packet_timer;
	/* Number of bytes received or to send */
	uint16_t uart_buf_ctr;
	/* Storage of received characters or characters to send */
	uint8_t uart_buf[CONFIG_DYNAMIXEL_BUFFER_SIZE];
};

#define DXL_STATE_CONFIGURED 0

struct dxl_context {
	/* Interface name */
	const char *iface_name;
	/* Serial line configuration */
	struct dxl_serial_config *cfg;
	/* Amount of time to wait for response from device */
	uint32_t rxwait_to;
	/* Interface state */
	atomic_t state;

	/* Client's mutually exclusive access */
	struct k_mutex iface_lock;
	/* Wait for response semaphore */
	struct k_sem wait_sem;
	/* Handler work item */
	struct k_work handler_work;
	/* Received packet */
	struct dxl_frame rx_frame;
	/* Packet to transmit */
	struct dxl_frame tx_frame;

	/* Records error from frame reception, e.g. CRC error */
	int rx_frame_err;

	/* ID of the device the current request is addressed to */
	uint8_t expected_id;
};

/**
 * @brief Get Dynamixel interface context.
 *
 * @param ctx        Dynamixel interface context
 *
 * @retval           Pointer to interface context or NULL
 *                   if interface not available or not configured;
 */
struct dxl_context *dxl_get_context(const uint8_t iface);

/**
 * @brief Get Dynamixel interface index.
 *
 * @param ctx        Pointer to Dynamixel interface context
 *
 * @retval           Interface index or negative error value.
 */
int dxl_iface_get_by_ctx(const struct dxl_context *ctx);

/**
 * @brief Send packet.
 *
 * @param ctx        Dynamixel interface context
 */
void dxl_tx(struct dxl_context *ctx);

/**
 * @brief Send packet and wait certain time for response.
 *
 * @param ctx        Dynamixel interface context
 *
 * @retval           0 If the function was successful,
 *                   -ENOTSUP if Dynamixel mode is not supported,
 *                   -ETIMEDOUT on timeout,
 *                   -EMSGSIZE on length error,
 *                   -EIO on CRC error.
 */
int dxl_tx_wait_rx(struct dxl_context *ctx);

/**
 * @brief Disable serial line reception.
 *
 * @param ctx        Dynamixel interface context
 */
void dxl_serial_rx_disable(struct dxl_context *ctx);

/**
 * @brief Enable serial line reception.
 *
 * @param ctx        Dynamixel interface context
 */
void dxl_serial_rx_enable(struct dxl_context *ctx);

/**
 * @brief Assemble packet from serial line RX buffer
 *
 * @param ctx        Dynamixel interface context
 *
 * @retval           0 If the function was successful,
 *                   -ENOTSUP if serial line mode is not supported,
 *                   -EMSGSIZE on length error,
 *                   -EIO on CRC error.
 */
int dxl_serial_rx(struct dxl_context *ctx);

/**
 * @brief Assemble packet from serial line TX buffer
 *
 * @param ctx        Dynamixel interface context
 *
 * @retval           0 If the function was successful,
 *                   -ENOTSUP if serial line mode is not supported.
 */
int dxl_serial_tx(struct dxl_context *ctx);

/**
 * @brief Initialise serial line support.
 *
 * @param ctx        Dynamixel interface context
 * @param param      Configuration parameter of the interface
 *
 * @retval           0 If the function was successful.
 */
int dxl_serial_init(struct dxl_context *ctx, struct dxl_iface_param param);

/**
 * @brief Disable serial line support.
 *
 * @param ctx        Dynamixel interface context
 */
void dxl_serial_disable(struct dxl_context *ctx);

/**
 * @brief Look up a control register's address and length.
 *
 * @param item    Control register identifier.
 * @param addr    Optional out: register address.
 * @param length  Optional out: register length in bytes.
 *
 * @retval 0 on success
 * @retval -EINVAL if item is out of range or has no entry
 */
int dxl_table_lookup(enum dxl_control item, uint16_t *addr, uint8_t *length);

/* Model info (currently unused by driver code; available for unit conversions). */
extern const struct dxl_model_info dxl_info_x330;

/**
 * @brief Parse a status-packet payload into a uint32_t value.
 *
 * @param data   Pointer to the status packet's data area, where data[0] is
 *               the device-error byte and data[1..] is the value payload.
 * @param width  Register width in bytes. Must be 1, 2, or 4.
 * @param out    On success, receives the value as a uint32_t (LE-decoded).
 *
 * @retval 0       Device replied success and *out is valid.
 * @retval >0      Device-error byte (see enum dxl_error). *out untouched.
 * @retval -EINVAL Unsupported width.
 */
int parse_status_payload(const uint8_t *data, uint8_t width, uint32_t *out);

#endif /* ZEPHYR_INCLUDE_DYNAMIXEL_INTERNAL_H_ */