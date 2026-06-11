#ifndef ZEPHYR_INCLUDE_DRIVERS_BUS_SERVO_INTERNAL_H_
#define ZEPHYR_INCLUDE_DRIVERS_BUS_SERVO_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <drivers/bus_servo.h>

#define BUS_SERVO_MAX_PACKET_SIZE 64

struct bus_servo_serial_config {
	const struct device *dev;
	struct gpio_dt_spec *tx_en;
	uint8_t uart_buf[BUS_SERVO_MAX_PACKET_SIZE];
	size_t uart_buf_len;
	uint32_t baud;
};

struct bus_servo_context {
	const char *iface_name;
	struct bus_servo_serial_config *cfg;
	struct k_mutex iface_lock;
	struct k_sem wait_sem;
	uint32_t rx_timeout_us;
	bool configured;
	uint8_t expected_id;
	uint8_t status_error;
	uint8_t response_params[BUS_SERVO_MAX_PACKET_SIZE];
	size_t response_param_len;
};

uint8_t bus_servo_checksum(const uint8_t *data, size_t len);
int bus_servo_serial_init(struct bus_servo_context *ctx, struct bus_servo_iface_param param);
int bus_servo_tx_rx(struct bus_servo_context *ctx, const uint8_t *tx_buf, size_t tx_len,
		    bool expect_response);
struct bus_servo_context *bus_servo_get_context(int iface);

#endif /* ZEPHYR_INCLUDE_DRIVERS_BUS_SERVO_INTERNAL_H_ */
