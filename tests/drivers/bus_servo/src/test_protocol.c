#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>
#include <drivers/bus_servo.h>

#include "fake_bus.h"

#define BUS_NODE   DT_NODELABEL(bus_servo0)
#define UART_NODE  DT_PARENT(BUS_NODE)
#define IFACE_NAME DEVICE_DT_NAME(BUS_NODE)

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static struct fake_bus bus;
static int iface;

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	struct bus_servo_iface_param param = {
		.rx_timeout_us = 50000,
		.serial = {.baud = 115200, .parity = UART_CFG_PARITY_NONE},
	};

	fake_bus_init(&bus);
	fake_bus_attach(&bus, uart_dev);
	iface = bus_servo_iface_get_by_name(IFACE_NAME);
	zassert_true(iface >= 0, "iface lookup failed: %d", iface);
	zassert_ok(bus_servo_init(iface, param), "bus_servo_init failed");
}

static void after_each(void *fixture)
{
	ARG_UNUSED(fixture);
	bus_servo_disable(iface);
}

ZTEST_SUITE(bus_servo_protocol, NULL, NULL, before_each, after_each, NULL);

ZTEST(bus_servo_protocol, test_write_u16_frame_bytes)
{
	/* FF FF id len inst addr value_l value_h checksum */
	zassert_ok(bus_servo_write_u16(iface, 2, 0x2a, 0x07ff));

	const uint8_t expected[] = {
		0xff, 0xff, 0x02, 0x05, 0x03, 0x2a, 0xff, 0x07, 0xc5,
	};
	zassert_equal(bus.last_tx_len, sizeof(expected));
	zassert_mem_equal(bus.last_tx, expected, sizeof(expected));
}

ZTEST(bus_servo_protocol, test_read_u16_frame_and_response_parse)
{
	const uint8_t response[] = {
		0xff, 0xff, 0x01, 0x04, 0x00, 0x34, 0x12, 0xb4,
	};
	uint16_t value = 0;

	fake_bus_queue_rx(&bus, response, sizeof(response));
	zassert_ok(bus_servo_read_u16(iface, 1, 0x38, &value));

	const uint8_t expected_request[] = {
		0xff, 0xff, 0x01, 0x04, 0x02, 0x38, 0x02, 0xbe,
	};
	zassert_equal(value, 0x1234);
	zassert_equal(bus.last_tx_len, sizeof(expected_request));
	zassert_mem_equal(bus.last_tx, expected_request, sizeof(expected_request));
}

ZTEST(bus_servo_protocol, test_timeout_returns_etimedout)
{
	uint16_t value = 0;

	bus.drop_response = true;
	zassert_equal(bus_servo_read_u16(iface, 1, 0x38, &value), -ETIMEDOUT);
}
