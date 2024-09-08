/**
 * @brief DYNAMIXEL transport protocol API
 * @defgroup dynamixel DYNAMIXEL
 * @ingroup io_interfaces
 * @{
 */

#ifndef ZEPHYR_INCLUDE_DYNAMIXEL_H_
#define ZEPHYR_INCLUDE_DYNAMIXEL_H_

#include <zephyr/drivers/uart.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DXL_BROADCAST_ID 0xFE

/* https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#hardware-error-status70 */
#define DXL_ALERT BIT(7)
#define DXL_INPUT_VOLTAGE_ERR BIT(0)
#define DXL_OVERHEATING_ERR BIT(2)
#define DXL_ELECTRICAL_SHOCK_ERR BIT(4)
#define DXL_OVERLOAD_ERR BIT(5)

#define XL330_M077 1190
#define XL330_M288 1200
#define XC330_M181 1230
#define XC330_M288 1240
#define XC330_T181 1210
#define XC330_T288 1220

	/* http://emanual.robotis.com/docs/en/dxl/protocol2/#instruction */
	enum dxl_instruction
	{
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
	enum dxl_mode
	{
		DXL_OP_CURRENT = 0,
		DXL_OP_VELOCITY = 1,
		DXL_OP_POSITION = 3,
		DXL_OP_EXTENDED_POSITION = 4,
		DXL_OP_CURRENT_BASED_POSITION = 5,
		DXL_OP_PWM = 16
	};

	/* https://emanual.robotis.com/docs/en/dxl/protocol2/#error */
	enum dxl_error
	{
		DXL_ERR_NONE = 0x00,
		DXL_ERR_RESULT_FAIL,
		DXL_ERR_INST_ERROR,
		DXL_ERR_CRC_ERROR,
		DXL_ERR_DATA_RANGE,
		DXL_ERR_DATA_LENGTH,
		DXL_ERR_DATA_LIMIT,
		DXL_ERR_ACCESS
	};

	enum dxl_control
	{
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
	struct dxl_control_info
	{
		/** Control Address */
		uint16_t address;
		/** Length of control data */
		uint8_t length;
	};

	/* https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#control-table-of-eeprom-area */
	static const struct dxl_control_info control_table[] = {
	    {0, 2},  /* MODEL_NUMBER */
	    {2, 4},  /* MODEL_INFORMATION */
	    {6, 1},  /* FIRMWARE_VERSION */
	    {7, 1},  /* ID */
	    {8, 1},  /* BAUD_RATE */
	    {9, 1},  /* RETURN_DELAY_TIME */
	    {10, 1}, /* DRIVE_MODE */
	    {11, 1}, /* OPERATING_MODE */
	    {12, 1}, /* SECONDARY_ID */
	    {13, 1}, /* PROTOCOL_VERSION */
	    {20, 4}, /* HOMING_OFFSET */
	    {24, 4}, /* MOVING_THRESHOLD */
	    {31, 1}, /* TEMPERATURE_LIMIT */
	    {32, 2}, /* MAX_VOLTAGE_LIMIT */
	    {34, 2}, /* MIN_VOLTAGE_LIMIT */
	    {36, 2}, /* PWM_LIMIT */
	    {38, 2}, /* CURRENT_LIMIT */
	    {44, 4}, /* VELOCITY_LIMIT */
	    {48, 4}, /* MAX_POSITION_LIMIT */
	    {52, 4}, /* MIN_POSITION_LIMIT */
	    {60, 1}, /* STARTUP_CONFIGURATION */
	    {62, 1}, /* PWM_SLOPE */
	    {63, 1}, /* SHUTDOWN */

	    {64, 1},  /* TORQUE_ENABLE */
	    {65, 1},  /* LED */
	    {68, 1},  /* STATUS_RETURN_LEVEL */
	    {69, 1},  /* REGISTERED_INSTRUCTION */
	    {70, 1},  /* HARDWARE_ERROR_STATUS */
	    {76, 2},  /* VELOCITY_I_GAIN */
	    {78, 2},  /* VELOCITY_P_GAIN */
	    {80, 2},  /* POSITION_D_GAIN */
	    {82, 2},  /* POSITION_I_GAIN */
	    {84, 2},  /* POSITION_P_GAIN */
	    {88, 2},  /* FEEDFORWARD_2ND_GAIN */
	    {90, 2},  /* FEEDFORWARD_1ST_GAIN */
	    {98, 1},  /* BUS_WATCHDOG */
	    {100, 2}, /* GOAL_PWM */
	    {102, 2}, /* GOAL_CURRENT */
	    {104, 4}, /* GOAL_VELOCITY */
	    {108, 4}, /* PROFILE_ACCELERATION */
	    {112, 4}, /* PROFILE_VELOCITY */
	    {116, 4}, /* GOAL_POSITION */
	    {120, 2}, /* REALTIME_TICK */
	    {122, 1}, /* MOVING */
	    {123, 1}, /* MOVING_STATUS */
	    {124, 2}, /* PRESENT_PWM */
	    {126, 2}, /* PRESENT_CURRENT */
	    {128, 4}, /* PRESENT_VELOCITY */
	    {132, 4}, /* PRESENT_POSITION */
	    {136, 4}, /* VELOCITY_TRAJECTORY */
	    {140, 4}, /* POSITION_TRAJECTORY */
	    {144, 2}, /* PRESENT_INPUT_VOLTAGE */
	    {146, 1}  /* PRESENT_TEMPERATURE */
	};

	/**
	 * @brief Model info struct.
	 */
	struct dxl_model_info
	{
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

	static const struct dxl_model_info info_x330 = {
	    0.229,
	    0,
	    2048,
	    4096,
	    -3.14159265,
	    3.14159265};

	/**
	 * @brief Frame struct used internally.
	 */
	struct dxl_frame
	{
		/** Packet header */
		uint32_t header;
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
	 * @brief Read Instruction
	 *
	 * Sends a read instruction to a specific device.
	 *
	 * @param iface    Dynamixel interface index
	 * @param id       Packet ID of the device that should receive the Instruction Packet
	 * @param item_idx Index of control register to read. @see dxl_control
	 * @param data     Pointer to the data to be received
	 *
	 * @retval         0 If the function was successful
	 */
	int dxl_read(const int iface, const uint8_t id, uint8_t item_idx, void *data);

	/**
	 * @brief Write Instruction
	 *
	 * Sends a write instruction to a specific device.
	 *
	 * @param iface    Dynamixel interface index
	 * @param id       Packet ID of the device that should receive the Instruction Packet
	 * @param item_idx Index of control register to read. @see dxl_control
	 * @param data     Data to write
	 *
	 * @retval         0 If the function was successful
	 */
	int dxl_write(const int iface, const uint8_t id, uint8_t item_idx, uint32_t data);

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
	 * @brief Dynamixel raw callback function signature
	 *
	 * @param iface      Dynamixel interface index
	 * @param frame      Pointer to the packet struct to send
	 *
	 * @retval           0 If transfer was successful
	 */
	typedef int (*dxl_raw_cb_t)(const int iface, const struct dxl_frame *frame);

	/**
	 * @brief Dynamixel serial line parameter
	 */
	struct dxl_serial_param
	{
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
	struct dxl_iface_param
	{
		/** Amount of time to wait for a response (in us) */
		uint32_t rx_timeout;
		/** Serial support parameter of the interface */
		struct dxl_serial_param serial;
	};

	/**
	 * @brief Motor parameter structure to configure Dynamixel motor.
	 */
	struct dxl_motor_config
	{
		/** Name of the motor */
		const char *label;
		/** Motor ID on the bus */
		uint8_t id;
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
	int dxl_disable(const uint8_t iface);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DYNAMIXEL_H_ */