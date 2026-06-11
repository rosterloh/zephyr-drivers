#ifndef TEST_BUS_SERVO_FAKE_BUS_H_
#define TEST_BUS_SERVO_FAKE_BUS_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

struct fake_bus {
	uint8_t last_tx[64];
	size_t last_tx_len;
	uint8_t pending_rx[64];
	size_t pending_rx_len;
	const struct device *uart;
	bool drop_response;
};

void fake_bus_init(struct fake_bus *bus);
void fake_bus_attach(struct fake_bus *bus, const struct device *uart_dev);
void fake_bus_queue_rx(struct fake_bus *bus, const uint8_t *data, size_t len);

#endif /* TEST_BUS_SERVO_FAKE_BUS_H_ */
