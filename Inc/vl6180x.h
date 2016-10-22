/*
 *                                 Sensors description:
 *
 *                       _____|____________________________|_____             FFL - FRONT FRONT	   LEFT  (SENSOR2)
 *                      /   _FFL_                        _FFR_   \            FFR - FRONT FRONT    RIGHT (SENSOR5)
 *                     /                                          \           FDL - FRONT DIAGONAL LEFT  (SENSOR3)
 *                    /    \  /                            \  /    \          FDR - FRONT DIAGONAL RIGHT (SENSOR6)
 *                   | F|   FDL                            FDR   |F |         FSL - FRONT SIDE     LEFT  (SENSOR1)
 *                  -|-S|   /                                \   |S-|-        FSR - FRONT SIDE     RIGHT (SENSOR4)
 *                   | L|                                        |R |         RRL - REAR  REAR     LEFT  (SENSOR8)
 *                   |                                              |         RRR - REAR  REAR     RIGHT (SENSOR11)
 *                   |                                              |         RDL - REAR  DIAGONAL LEFT  (SENSOR9)
 *                   |                                              |         RDR - REAR  DIAGONAL RIGHT (SENSOR12)
 *                   |                                              |         RSL - REAR  SIDE     LEFT  (SENSOR7)
 *                   |                ---------------               |         RSR - REAR  SIDE     RIGHT (SENSOR10)
 *                   |                |             |               |
 *                   |                |   BlueRay   |               |
 *                   |                |             |               |
 *                   |                ---------------               |
 *                   | R|                                        |R |
 *                  -|-S|   \                O               /   |S-|-
 *                   | L|   RDL          O   O   O         RDR   |R |
 *                    \     / \              O             / \     /
 *                     \    _____                        _____    /
 *                      \____RRL__________________________RRR____/
 *                            |                            |
 *
 *
 */


#ifndef VL6180X_H_
#define VL6180X_H_

#include "stm32f4xx_hal.h"

#define ADDR_DEF_VL6180X 0x29<<1

// starting i2c address for sensors
#define SENSOR_ADDR_START 0x11
enum sensorsid {FSL=0, FFL, FDL, FSR, FFR, FDR, RSL, RRL, RSR, RRR};
const uint16_t sensor_en_pins[] = {SENSOR1_Pin, SENSOR2_Pin, SENSOR3_Pin, SENSOR4_Pin, SENSOR5_Pin, SENSOR6_Pin, SENSOR7_Pin, SENSOR8_Pin, SENSOR10_Pin, SENSOR11_Pin};
extern uint8_t i2c1_fail;

class VL6180X {
public:
	enum registers {
		IDENTIFICATION__MODEL_ID              = 0x000,
		IDENTIFICATION__MODEL_REV_MAJOR       = 0x001,
		IDENTIFICATION__MODEL_REV_MINOR       = 0x002,
		IDENTIFICATION__MODULE_REV_MAJOR      = 0x003,
		IDENTIFICATION__MODULE_REV_MINOR      = 0x004,
		IDENTIFICATION__DATE_HI               = 0x006,
		IDENTIFICATION__DATE_LO               = 0x007,
		IDENTIFICATION__TIME                  = 0x008,
		SYSTEM__MODE_GPIO0                    = 0x010,
		SYSTEM__MODE_GPIO1                    = 0x011,
		SYSTEM__HISTORY_CTRL                  = 0x012,
		SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x014,
		SYSTEM__INTERRUPT_CLEAR               = 0x015,
		SYSTEM__FRESH_OUT_OF_RESET            = 0x016,
		SYSTEM__GROUPED_PARAMETER_HOLD        = 0x017,
		SYSRANGE__START                       = 0x018,
		SYSRANGE__THRESH_HIGH                 = 0x019,
		SYSRANGE__THRESH_LOW                  = 0x01A,
		SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x01B,
		SYSRANGE__MAX_CONVERGENCE_TIME        = 0x01C,
		SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E,
		SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x021,
		SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x022,
		SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x024,
		SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x025,
		SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x026,
		SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x02C,
		SYSRANGE__RANGE_CHECK_ENABLES         = 0x02D,
		SYSRANGE__VHV_RECALIBRATE             = 0x02E,
		SYSRANGE__VHV_REPEAT_RATE             = 0x031,
		SYSALS__START                         = 0x038,
		SYSALS__THRESH_HIGH                   = 0x03A,
		SYSALS__THRESH_LOW                    = 0x03C,
		SYSALS__INTERMEASUREMENT_PERIOD       = 0x03E,
		SYSALS__ANALOGUE_GAIN                 = 0x03F,
		SYSALS__INTEGRATION_PERIOD            = 0x040,
		RESULT__RANGE_STATUS                  = 0x04D,
		RESULT__ALS_STATUS                    = 0x04E,
		RESULT__INTERRUPT_STATUS_GPIO         = 0x04F,
		RESULT__ALS_VAL                       = 0x050,
		RESULT__HISTORY_BUFFER_0              = 0x052,
		RESULT__HISTORY_BUFFER_1              = 0x054,
		RESULT__HISTORY_BUFFER_2              = 0x056,
		RESULT__HISTORY_BUFFER_3              = 0x058,
		RESULT__HISTORY_BUFFER_4              = 0x05A,
		RESULT__HISTORY_BUFFER_5              = 0x05C,
		RESULT__HISTORY_BUFFER_6              = 0x05E,
		RESULT__HISTORY_BUFFER_7              = 0x060,
		RESULT__RANGE_VAL                     = 0x062,
		RESULT__RANGE_RAW                     = 0x064,
		RESULT__RANGE_RETURN_RATE             = 0x066,
		RESULT__RANGE_REFERENCE_RATE          = 0x068,
		RESULT__RANGE_RETURN_SIGNAL_COUNT     = 0x06C,
		RESULT__RANGE_REFERENCE_SIGNAL_COUNT  = 0x070,
		RESULT__RANGE_RETURN_AMB_COUNT        = 0x074,
		RESULT__RANGE_REFERENCE_AMB_COUNT     = 0x078,
		RESULT__RANGE_RETURN_CONV_TIME        = 0x07C,
		RESULT__RANGE_REFERENCE_CONV_TIME     = 0x080,
		READOUT__AVERAGING_SAMPLE_PERIOD      = 0x10A,
		FIRMWARE__BOOTUP                      = 0x119,
		FIRMWARE__RESULT_SCALER               = 0x120,
		I2C_SLAVE__DEVICE_ADDRESS             = 0x212,
		INTERLEAVED_MODE__ENABLE              = 0x2A3
	};

	// constructor, params: i2c bus, enable GPIO port, enable GPIO pin
	//VL6180X(I2C_HandleTypeDef* i2c_handle, GPIO_TypeDef* gpio_port_handle, uint16_t gpio_pin_handle);
	VL6180X(void);
	// initializes the sensor and sets an unique I2C address
	void init(uint8_t sensorid, uint8_t new_addr);

	uint8_t read8(uint16_t reg);
	uint16_t read16(uint16_t reg);
	uint32_t read32(uint16_t reg);
	void write8(uint16_t reg, uint8_t data);
	void write16(uint16_t reg, uint16_t data);
	void write32(uint16_t reg, uint32_t data);

	void startRangeContinous(void);
	uint8_t readRange(void);
	void clearInterrupts(void);
	void poll(void);

private:
	// hardware handlers
	uint8_t address;
	I2C_HandleTypeDef* i2c_handle;
	GPIO_TypeDef* gpio_port_handle;
	uint16_t gpio_pin_handle;
};

extern VL6180X sensors[10];

#endif /* VL6180X_H_ */
