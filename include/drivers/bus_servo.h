#ifndef ZEPHYR_INCLUDE_DRIVERS_BUS_SERVO_H_
#define ZEPHYR_INCLUDE_DRIVERS_BUS_SERVO_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/uart.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUS_SERVO_BROADCAST_ID 0xfe

enum bus_servo_instruction {
	BUS_SERVO_INST_PING = 0x01,
	BUS_SERVO_INST_READ = 0x02,
	BUS_SERVO_INST_WRITE = 0x03,
	BUS_SERVO_INST_REG_WRITE = 0x04,
	BUS_SERVO_INST_ACTION = 0x05,
	BUS_SERVO_INST_SYNC_WRITE = 0x83,
};

enum bus_servo_register {
	BUS_SERVO_REG_TORQUE_ENABLE = 40,
	BUS_SERVO_REG_GOAL_ACCEL = 41,
	BUS_SERVO_REG_GOAL_POSITION_L = 42,
	BUS_SERVO_REG_GOAL_TIME_L = 44,
	BUS_SERVO_REG_GOAL_SPEED_L = 46,
	BUS_SERVO_REG_PRESENT_POSITION_L = 56,
	BUS_SERVO_REG_PRESENT_SPEED_L = 58,
	BUS_SERVO_REG_PRESENT_LOAD_L = 60,
	BUS_SERVO_REG_PRESENT_VOLTAGE = 62,
	BUS_SERVO_REG_PRESENT_TEMPERATURE = 63,
	BUS_SERVO_REG_MOVING = 66,
	BUS_SERVO_REG_PRESENT_CURRENT_L = 69,
};

struct bus_servo_iface_param {
	uint32_t rx_timeout_us;
	struct {
		uint32_t baud;
		enum uart_config_parity parity;
	} serial;
};

int bus_servo_iface_get_by_name(const char *iface_name);
int bus_servo_init(int iface, struct bus_servo_iface_param param);
int bus_servo_disable(int iface);
int bus_servo_ping(int iface, uint8_t id);

int bus_servo_read_u8(int iface, uint8_t id, uint8_t addr, uint8_t *out);
int bus_servo_read_u16(int iface, uint8_t id, uint8_t addr, uint16_t *out);
int bus_servo_write_u8(int iface, uint8_t id, uint8_t addr, uint8_t value);
int bus_servo_write_u16(int iface, uint8_t id, uint8_t addr, uint16_t value);

int bus_servo_write_position_ex(int iface, uint8_t id, uint16_t position, uint16_t speed,
				uint8_t accel);
int bus_servo_sync_write_position_ex(int iface, const uint8_t ids[], const uint16_t positions[],
				     const uint16_t speeds[], const uint8_t accels[], size_t n);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_BUS_SERVO_H_ */
