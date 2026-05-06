/**
 * @brief DYNAMIXEL transport protocol API
 * @defgroup dynamixel DYNAMIXEL
 * @ingroup io_interfaces
 * @{
 */

#ifndef ZEPHYR_INCLUDE_DYNAMIXEL_H_
#define ZEPHYR_INCLUDE_DYNAMIXEL_H_

#include <stddef.h>
#include <zephyr/drivers/uart.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DXL_BROADCAST_ID 0xFE

/* https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#hardware-error-status70 */
#define DXL_ALERT                BIT(7)
#define DXL_INPUT_VOLTAGE_ERR    BIT(0)
#define DXL_OVERHEATING_ERR      BIT(2)
#define DXL_ELECTRICAL_SHOCK_ERR BIT(4)
#define DXL_OVERLOAD_ERR         BIT(5)

#define XL330_M077 1190
#define XL330_M288 1200
#define XC330_M181 1230
#define XC330_M288 1240
#define XC330_T181 1210
#define XC330_T288 1220

/* http://emanual.robotis.com/docs/en/dxl/protocol2/#instruction */
enum dxl_instruction {
	DXL_INST_PING = 0x01,
	DXL_INST_READ = 0x02,
	DXL_INST_WRITE = 0x03,
	DXL_INST_REG_WRITE = 0x04,
	DXL_INST_ACTION = 0x05,
	DXL_INST_FACTORY_RESET = 0x06,
	DXL_INST_REBOOT = 0x08,
	DXL_INST_CLEAR = 0x10,
	DXL_INST_STATUS = 0x55,
	DXL_INST_SYNC_READ = 0x82,
	DXL_INST_SYNC_WRITE = 0x83,
	DXL_INST_BULK_READ = 0x92,
	DXL_INST_BULK_WRITE = 0x93
};

/* https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#operating-mode */
enum dxl_mode {
	DXL_OP_CURRENT = 0,
	DXL_OP_VELOCITY = 1,
	DXL_OP_POSITION = 3,
	DXL_OP_EXTENDED_POSITION = 4,
	DXL_OP_CURRENT_BASED_POSITION = 5,
	DXL_OP_PWM = 16
};

/* https://emanual.robotis.com/docs/en/dxl/protocol2/#error */
enum dxl_error {
	DXL_ERR_NONE = 0x00,
	DXL_ERR_RESULT_FAIL,
	DXL_ERR_INST_ERROR,
	DXL_ERR_CRC_ERROR,
	DXL_ERR_DATA_RANGE,
	DXL_ERR_DATA_LENGTH,
	DXL_ERR_DATA_LIMIT,
	DXL_ERR_ACCESS
};

enum dxl_control {
	/* EEPROM */
	MODEL_NUMBER = 0,
	MODEL_INFORMATION,
	FIRMWARE_VERSION,
	ID,
	BAUD_RATE,
	RETURN_DELAY_TIME,
	DRIVE_MODE,
	OPERATING_MODE,
	SECONDARY_ID,
	PROTOCOL_VERSION,
	HOMING_OFFSET,
	MOVING_THRESHOLD,
	TEMPERATURE_LIMIT,
	MAX_VOLTAGE_LIMIT,
	MIN_VOLTAGE_LIMIT,
	PWM_LIMIT,
	CURRENT_LIMIT,
	VELOCITY_LIMIT,
	MAX_POSITION_LIMIT,
	MIN_POSITION_LIMIT,
	STARTUP_CONFIGURATION,
	PWM_SLOPE,
	SHUTDOWN,
	/* RAM */
	TORQUE_ENABLE,
	LED,
	STATUS_RETURN_LEVEL,
	REGISTERED_INSTRUCTION,
	HARDWARE_ERROR_STATUS,
	VELOCITY_I_GAIN,
	VELOCITY_P_GAIN,
	POSITION_D_GAIN,
	POSITION_I_GAIN,
	POSITION_P_GAIN,
	FEEDFORWARD_2ND_GAIN,
	FEEDFORWARD_1ST_GAIN,
	BUS_WATCHDOG,
	GOAL_PWM,
	GOAL_CURRENT,
	GOAL_VELOCITY,
	PROFILE_ACCELERATION,
	PROFILE_VELOCITY,
	GOAL_POSITION,
	REALTIME_TICK,
	MOVING,
	MOVING_STATUS,
	PRESENT_PWM,
	PRESENT_CURRENT,
	PRESENT_VELOCITY,
	PRESENT_POSITION,
	VELOCITY_TRAJECTORY,
	POSITION_TRAJECTORY,
	PRESENT_INPUT_VOLTAGE,
	PRESENT_TEMPERATURE
};

/**
 * @brief Control table info struct used internally.
 */
struct dxl_control_info {
	/** Control Address */
	uint16_t address;
	/** Length of control data */
	uint8_t length;
};

/**
 * @brief Model info struct.
 */
struct dxl_model_info {
	/** Single unit speed */
	float rpm;
	/** Minimum position in encoder units */
	int64_t value_of_min_radian_position;
	/** Zero position in encoder units */
	int64_t value_of_zero_radian_position;
	/** Maximum position in encoder units */
	int64_t value_of_max_radian_position;
	/** Minimum position in radians */
	float min_radian;
	/** Maximum position in radians */
	float max_radian;
};

/**
 * @brief Frame struct used internally.
 */
struct dxl_frame {
	/** ID of the device that should receive the Instruction Packet and process it */
	uint8_t id;
	/** Length of packet field */
	uint16_t length;
	/** Instruction Code */
	uint8_t ic;
	/** Auxiliary data field for Instruction */
	uint8_t data[CONFIG_DYNAMIXEL_BUFFER_SIZE - 10];
	/** CRC */
	uint16_t crc;
};

/**
 * @brief Ping Instruction
 *
 * Sends a ping message to a specific device.
 *
 * @param iface Dynamixel interface index
 * @param id    Packet ID of the device that should receive the Instruction Packet
 *
 * @retval      0 If the function was successful
 */
int dxl_ping(const int iface, const uint8_t id);

/**
 * @brief Reboot Instruction
 *
 * Sends a reboot message to a specific device.
 *
 * @param iface Dynamixel interface index
 * @param id    Packet ID of the device that should receive the Instruction Packet
 *
 * @retval      0 If the function was successful
 */
int dxl_reboot(const int iface, const uint8_t id);

/**
 * @brief Read an 8-bit register.
 *
 * @param iface Dynamixel interface index.
 * @param id    Bus ID of the device.
 * @param item  Control register identifier.
 * @param out   Output pointer.
 *
 * @retval 0 on success, with device-success.
 * @retval >0 enum dxl_error from the device's status packet.
 * @retval <0 errno: -ETIMEDOUT, -EIO, -ENODEV, -EINVAL.
 */
int dxl_read_u8(int iface, uint8_t id, enum dxl_control item, uint8_t *out);
int dxl_read_u16(int iface, uint8_t id, enum dxl_control item, uint16_t *out);
int dxl_read_u32(int iface, uint8_t id, enum dxl_control item, uint32_t *out);

int dxl_write_u8(int iface, uint8_t id, enum dxl_control item, uint8_t val);
int dxl_write_u16(int iface, uint8_t id, enum dxl_control item, uint16_t val);
int dxl_write_u32(int iface, uint8_t id, enum dxl_control item, uint32_t val);

/**
 * @brief Write the same register on multiple servos in one transaction.
 *
 * SYNC_WRITE (Protocol-2 0x83) is broadcast and produces no status replies.
 * Per-servo failure (e.g. servo unplugged, register read-only) is not
 * detectable without a follow-up read.
 *
 * @param iface Dynamixel interface index.
 * @param item  Control register identifier (must match the typed width).
 * @param ids   Array of N servo bus IDs.
 * @param vals  Array of N values to write.
 * @param n     Number of servos (must be > 0).
 *
 * @retval 0       All bytes written to the bus.
 * @retval -EINVAL n=0, NULL pointers, or item width mismatch.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_sync_write_u8 (int iface, enum dxl_control item,
		       const uint8_t ids[], const uint8_t  vals[], size_t n);
int dxl_sync_write_u16(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint16_t vals[], size_t n);
int dxl_sync_write_u32(int iface, enum dxl_control item,
		       const uint8_t ids[], const uint32_t vals[], size_t n);

/**
 * @brief Read the same register from multiple servos in one transaction.
 *
 * SYNC_READ (Protocol-2 0x82) sends a broadcast instruction; each addressed
 * servo replies with its own status packet in ID-list order.
 *
 * @param iface Dynamixel interface index.
 * @param item  Control register identifier (must match the typed width).
 * @param ids   Array of N servo bus IDs.
 * @param vals  On per-slot success, receives that servo's value.
 * @param errs  Optional. If non-NULL, receives per-slot result:
 *              0 = ok, >0 = device-error byte, <0 = transport error.
 *              Pass NULL to opt out of per-slot detail; failed slots leave
 *              vals[i] untouched.
 * @param n     Number of servos (must be > 0).
 *
 * @retval 0       All servos succeeded.
 * @retval -EIO    At least one servo failed; check errs[] if non-NULL.
 * @retval -EINVAL n=0, NULL ids/vals, or item width mismatch.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_sync_read_u8 (int iface, enum dxl_control item,
		      const uint8_t ids[], uint8_t  vals[],
		      int errs[], size_t n);
int dxl_sync_read_u16(int iface, enum dxl_control item,
		      const uint8_t ids[], uint16_t vals[],
		      int errs[], size_t n);
int dxl_sync_read_u32(int iface, enum dxl_control item,
		      const uint8_t ids[], uint32_t vals[],
		      int errs[], size_t n);

/**
 * @brief Per-entry record for BULK_READ.
 */
struct dxl_bulk_read_entry {
	uint8_t id;
	enum dxl_control item;
};

/**
 * @brief Per-entry record for BULK_WRITE.
 *
 * @c value is interpreted at the width returned by @c dxl_table_lookup(item).
 */
struct dxl_bulk_write_entry {
	uint8_t id;
	enum dxl_control item;
	uint32_t value;
};

/**
 * @brief Read different registers from multiple servos in one transaction.
 *
 * BULK_READ (Protocol-2 0x92) sends a broadcast instruction; each addressed
 * servo replies with its own status packet in entry order.
 *
 * @param iface Dynamixel interface index.
 * @param req   Array of N {id, item} entries.
 * @param vals  On per-slot success, receives that entry's value as uint32_t.
 *              All current control-table items fit in 32 bits.
 * @param errs  Optional. Same semantics as dxl_sync_read_*.
 * @param n     Number of entries (must be > 0).
 *
 * @retval 0       All entries succeeded.
 * @retval -EIO    At least one entry failed; check errs[] if non-NULL.
 * @retval -EINVAL n=0, NULL pointers, or any req[i].item out of range.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_bulk_read(int iface, const struct dxl_bulk_read_entry req[],
		  uint32_t vals[], int errs[], size_t n);

/**
 * @brief Write different registers to multiple servos in one transaction.
 *
 * BULK_WRITE (Protocol-2 0x93) is broadcast and produces no status replies.
 *
 * @param iface Dynamixel interface index.
 * @param req   Array of N {id, item, value} entries.
 * @param n     Number of entries (must be > 0).
 *
 * @retval 0       All bytes written to the bus.
 * @retval -EINVAL n=0, NULL req, or any req[i].item out of range.
 * @retval -ENOSPC Computed packet exceeds CONFIG_DYNAMIXEL_BUFFER_SIZE.
 * @retval -ENODEV Interface not initialised.
 */
int dxl_bulk_write(int iface, const struct dxl_bulk_write_entry req[], size_t n);

/**
 * @brief Get Dynamixel interface index according to interface name
 *
 * If there is more than one interface, it can be used to clearly
 * identify interfaces in the application.
 *
 * @param iface_name Dynamixel interface name
 *
 * @retval           Dynamixel interface index or negative error value.
 */
int dxl_iface_get_by_name(const char *iface_name);

/**
 * @brief Dynamixel serial line parameter
 */
struct dxl_serial_param {
	/** Baudrate of the serial line */
	uint32_t baud;
	/** parity UART's parity setting:
	 *    UART_CFG_PARITY_NONE,
	 *    UART_CFG_PARITY_EVEN,
	 *    UART_CFG_PARITY_ODD
	 */
	enum uart_config_parity parity;
};

/**
 * @brief User parameter structure to configure Dynamixel interface.
 */
struct dxl_iface_param {
	/** Amount of time to wait for a response (in us) */
	uint32_t rx_timeout;
	/** Serial support parameter of the interface */
	struct dxl_serial_param serial;
};

/**
 * @brief Configure Dynamixel Interface
 *
 * @param iface      Dynamixel interface index
 * @param param      Configuration parameter of the interface
 *
 * @retval           0 If the function was successful
 */
int dxl_init(const int iface, struct dxl_iface_param param);

/**
 * @brief Disable Dynamixel Interface
 *
 * This function is called to disable Dynamixel interface.
 *
 * @param iface      Dynamixel interface index
 *
 * @retval           0 If the function was successful
 */
int dxl_disable(int iface);

/**
 * @brief Motor metadata derived from devicetree child nodes.
 */
struct dxl_motor {
	/** Optional label string from DT. May be NULL. */
	const char *label;
	/** Parent Dynamixel interface index. */
	int iface;
	/** Bus ID of the motor. */
	uint8_t id;
};

/**
 * @brief Number of motors discovered in devicetree.
 */
size_t dxl_motor_count(void);

/**
 * @brief Get the motor entry at @a idx, or NULL if out of range.
 */
const struct dxl_motor *dxl_motor_get(size_t idx);

/**
 * @brief Find a motor by its DT @c label, or NULL if not found.
 *
 * The comparison is by @c strcmp; both arguments must be non-NULL.
 */
const struct dxl_motor *dxl_motor_get_by_label(const char *label);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DYNAMIXEL_H_ */