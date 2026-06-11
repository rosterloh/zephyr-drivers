#include "fake_bus.h"

#include <string.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

static void tx_data_ready_cb(const struct device *dev, size_t size, void *user_data)
{
	struct fake_bus *bus = user_data;
	uint8_t buf[16];
	size_t remaining = sizeof(bus->last_tx) - bus->last_tx_len;
	size_t pending = size;

	while (pending > 0) {
		size_t chunk = MIN(pending, sizeof(buf));
		uint32_t got = uart_emul_get_tx_data(dev, buf, chunk);

		if (got == 0) {
			break;
		}

		size_t copy_len = MIN((size_t)got, remaining);

		if (copy_len > 0) {
			memcpy(bus->last_tx + bus->last_tx_len, buf, copy_len);
			bus->last_tx_len += copy_len;
			remaining -= copy_len;
		}
		zassert_equal(copy_len, (size_t)got, "fake bus TX capture overflow");
		pending -= got;
	}

	if (!bus->drop_response && bus->pending_rx_len > 0) {
		uint32_t written = uart_emul_put_rx_data(dev, bus->pending_rx, bus->pending_rx_len);

		zassert_equal(written, (uint32_t)bus->pending_rx_len,
			      "fake bus RX injection truncated");
		bus->pending_rx_len = 0;
	}
}

void fake_bus_init(struct fake_bus *bus)
{
	memset(bus, 0, sizeof(*bus));
}

void fake_bus_attach(struct fake_bus *bus, const struct device *uart_dev)
{
	bus->uart = uart_dev;
	uart_emul_callback_tx_data_ready_set(uart_dev, tx_data_ready_cb, bus);
}

void fake_bus_queue_rx(struct fake_bus *bus, const uint8_t *data, size_t len)
{
	if (bus->drop_response) {
		return;
	}

	zassert_true(len <= sizeof(bus->pending_rx), "fake bus RX injection too large");

	memcpy(bus->pending_rx, data, len);
	bus->pending_rx_len = len;
}
