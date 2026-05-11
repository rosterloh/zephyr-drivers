#ifndef MFD_SEESAW_REGS_H_
#define MFD_SEESAW_REGS_H_

#include <stdint.h>

/** The module base addresses for different seesaw modules. */
enum {
	SEESAW_STATUS_BASE = 0x00,
	SEESAW_GPIO_BASE = 0x01,
	SEESAW_SERCOM0_BASE = 0x02,

	SEESAW_TIMER_BASE = 0x08,
	SEESAW_ADC_BASE = 0x09,
	SEESAW_DAC_BASE = 0x0A,
	SEESAW_INTERRUPT_BASE = 0x0B,
	SEESAW_DAP_BASE = 0x0C,
	SEESAW_EEPROM_BASE = 0x0D,
	SEESAW_NEOPIXEL_BASE = 0x0E,
	SEESAW_TOUCH_BASE = 0x0F,
	SEESAW_KEYPAD_BASE = 0x10,
	SEESAW_ENCODER_BASE = 0x11,
	SEESAW_SPECTRUM_BASE = 0x12,
	SEESAW_SOIL_BASE = 0x13,
};

/** GPIO module function addres registers */
enum {
	SEESAW_GPIO_DIRSET_BULK = 0x02,
	SEESAW_GPIO_DIRCLR_BULK = 0x03,
	SEESAW_GPIO_BULK = 0x04,
	SEESAW_GPIO_BULK_SET = 0x05,
	SEESAW_GPIO_BULK_CLR = 0x06,
	SEESAW_GPIO_BULK_TOGGLE = 0x07,
	SEESAW_GPIO_INTENSET = 0x08,
	SEESAW_GPIO_INTENCLR = 0x09,
	SEESAW_GPIO_INTFLAG = 0x0A,
	SEESAW_GPIO_PULLENSET = 0x0B,
	SEESAW_GPIO_PULLENCLR = 0x0C,
};

/** status module function addres registers */
enum {
	SEESAW_STATUS_HW_ID = 0x01,
	SEESAW_STATUS_VERSION = 0x02,
	SEESAW_STATUS_OPTIONS = 0x03,
	SEESAW_STATUS_TEMP = 0x04,
	SEESAW_STATUS_SWRST = 0x7F,
};

/** timer module function addres registers */
enum {
	SEESAW_TIMER_STATUS = 0x00,
	SEESAW_TIMER_PWM = 0x01,
	SEESAW_TIMER_FREQ = 0x02,
};

/** ADC module function addres registers */
enum {
	SEESAW_ADC_STATUS = 0x00,
	SEESAW_ADC_INTEN = 0x02,
	SEESAW_ADC_INTENCLR = 0x03,
	SEESAW_ADC_WINMODE = 0x04,
	SEESAW_ADC_WINTHRESH = 0x05,
	SEESAW_ADC_CHANNEL_OFFSET = 0x07,
};

/** Sercom module function addres registers */
enum {
	SEESAW_SERCOM_STATUS = 0x00,
	SEESAW_SERCOM_INTEN = 0x02,
	SEESAW_SERCOM_INTENCLR = 0x03,
	SEESAW_SERCOM_BAUD = 0x04,
	SEESAW_SERCOM_DATA = 0x05,
};

/** neopixel module function addres registers */
enum {
	SEESAW_NEOPIXEL_STATUS = 0x00,
	SEESAW_NEOPIXEL_PIN = 0x01,
	SEESAW_NEOPIXEL_SPEED = 0x02,
	SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
	SEESAW_NEOPIXEL_BUF = 0x04,
	SEESAW_NEOPIXEL_SHOW = 0x05,
};

/** touch module function addres registers */
enum {
	SEESAW_TOUCH_CHANNEL_OFFSET = 0x10,
};

/** keypad module function addres registers */
enum {
	SEESAW_KEYPAD_STATUS = 0x00,
	SEESAW_KEYPAD_EVENT = 0x01,
	SEESAW_KEYPAD_INTENSET = 0x02,
	SEESAW_KEYPAD_INTENCLR = 0x03,
	SEESAW_KEYPAD_COUNT = 0x04,
	SEESAW_KEYPAD_FIFO = 0x10,
};

/** keypad module edge definitions */
enum {
	SEESAW_KEYPAD_EDGE_HIGH = 0,
	SEESAW_KEYPAD_EDGE_LOW,
	SEESAW_KEYPAD_EDGE_FALLING,
	SEESAW_KEYPAD_EDGE_RISING,
};

/** encoder module edge definitions */
enum {
	SEESAW_ENCODER_STATUS = 0x00,
	SEESAW_ENCODER_INTENSET = 0x02,
	SEESAW_ENCODER_INTENCLR = 0x03,
	SEESAW_ENCODER_POSITION = 0x04,
	SEESAW_ENCODER_DELTA = 0x05,
};

/** Audio spectrum module function address registers */
enum {
	SEESAW_SPECTRUM_RESULTS_LOWER = 0x00, /* Audio spectrum bins 0-31 */
	SEESAW_SPECTRUM_RESULTS_UPPER = 0x01, /* Audio spectrum bins 32-63 */
	SEESAW_SPECTRUM_CHANNEL = 0xFD,
	SEESAW_SPECTRUM_RATE = 0xFE,
	SEESAW_SPECTRUM_STATUS = 0xFF,
};

/** soil moisture module function address registers */
enum {
	/* 0x00..0x0F Global Settings */
	SEESAW_SOIL_STATUS = 0x00,
	SEESAW_SOIL_RATE = 0x01,
	/* 0x10..0xF0 Sensor Settings */
	SEESAW_SOIL_VALUE = 0x10,
	SEESAW_SOIL_SAMPLES = 0x20,
	SEESAW_SOIL_XDELAY = 0x30,
	SEESAW_SOIL_TIMEOUT = 0x40,
};

enum {
	SEESAW_PRODUCT_JOYWING = 3632,
	SEESAW_PRODUCT_CRICKITHAT = 3957,
	SEESAW_PRODUCT_SOIL = 4026,
	SEESAW_PRODUCT_ROBOHAT = 9998,
	SEESAW_PRODUCT_CRICKIT = 9999,
};

#define SEESAW_HW_ID_CODE_SAMD09   0x55 /* seesaw HW ID code for SAMD09 */
#define SEESAW_HW_ID_CODE_TINY806  0x84 /* seesaw HW ID code for ATtiny806 */
#define SEESAW_HW_ID_CODE_TINY807  0x85 /* seesaw HW ID code for ATtiny807 */
#define SEESAW_HW_ID_CODE_TINY816  0x86 /* seesaw HW ID code for ATtiny816 */
#define SEESAW_HW_ID_CODE_TINY817  0x87 /* seesaw HW ID code for ATtiny817 */
#define SEESAW_HW_ID_CODE_TINY1616 0x88 /* seesaw HW ID code for ATtiny1616 */
#define SEESAW_HW_ID_CODE_TINY1617 0x89 /* seesaw HW ID code for ATtiny1617 */

/** raw key event stucture for keypad module */
union keyEventRaw {
	struct {
		uint8_t EDGE: 2; ///< the edge that was triggered
		uint8_t NUM: 6;  ///< the event number
	} bit;                   ///< bitfield format
	uint8_t reg;             ///< register format
};

/** extended key event stucture for keypad module */
union keyEvent {
	struct {
		uint8_t EDGE: 2;  ///< the edge that was triggered
		uint16_t NUM: 14; ///< the event number
	} bit;                    ///< bitfield format
	uint16_t reg;             ///< register format
};

/** key state struct that will be written to seesaw chip keypad module */
union keyState {
	struct {
		uint8_t STATE: 1;  ///< the current state of the key
		uint8_t ACTIVE: 4; ///< the registered events for that key
	} bit;                     ///< bitfield format
	uint8_t reg;               ///< register format
};

#define SEESAW_WAIT_STARTUP_US       10000
#define SEESAW_WAIT_INITIAL_RESET_US 10000

#endif /* MFD_SEESAW_REGS_H_ */
