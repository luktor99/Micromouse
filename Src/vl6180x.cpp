#include <common.h>
#include <vl6180x.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void MX_I2C1_Init(void);

VL6180X sensors[10];
uint8_t i2c1_fail=0;

VL6180X::VL6180X(void) {
	// set the default VL6180X I2C address
	address=ADDR_DEF_VL6180X;
	// set the I2C peripheral sensors are connected to
	i2c_handle=&hi2c1;
}

void VL6180X::init(uint8_t sensorid, uint8_t new_addr) {
	// Enable sensor
	HAL_GPIO_WritePin(SENSOR1_GPIO_Port, sensor_en_pins[sensorid], GPIO_PIN_SET);
	osDelay(10);

	// Required - SR03 settings from application note AN4545
	write8(0x0207, 0x01);
	write8(0x0208, 0x01);
	write8(0x0096, 0x00);
	write8(0x0097, 0xfd);
	write8(0x00e3, 0x00);
	write8(0x00e4, 0x04);
	write8(0x00e5, 0x02);
	write8(0x00e6, 0x01);
	write8(0x00e7, 0x03);
	write8(0x00f5, 0x02);
	write8(0x00d9, 0x05);
	write8(0x00db, 0xce);
	write8(0x00dc, 0x03);
	write8(0x00dd, 0xf8);
	write8(0x009f, 0x00);
	write8(0x00a3, 0x3c);
	write8(0x00b7, 0x00);
	write8(0x00bb, 0x3c);
	write8(0x00b2, 0x09);
	write8(0x00ca, 0x09);
	write8(0x0198, 0x01);
	write8(0x01b0, 0x17);
	write8(0x01ad, 0x00);
	write8(0x00ff, 0x05);
	write8(0x0100, 0x05);
	write8(0x0199, 0x05);
	write8(0x01a6, 0x1b);
	write8(0x01ac, 0x3e);
	write8(0x01a7, 0x1f);
	write8(0x0030, 0x00);

	// Recommended / optional settings:
	write8(READOUT__AVERAGING_SAMPLE_PERIOD, 0x0); // No averaging
	write8(SYSRANGE__VHV_REPEAT_RATE, 0xFF); // Number of range measurements after which auto calibration of system is performed
	write8(SYSRANGE__VHV_RECALIBRATE, 0x01); // perform a single temperature calibration of the ranging sensor
	write8(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x0); // Time delay between measurements in Ranging continuous mode (0 -> 10ms)
	write8(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’
	write8(SYSRANGE__MAX_CONVERGENCE_TIME, 0x4); // 4ms

	// Interrupt setup
	write8(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x04); // Interrupt source: new range sample ready
	write8(SYSTEM__MODE_GPIO0, 0x10); // Enable GPIO0 interrupt output, active LOW
	write8(SYSTEM__MODE_GPIO1, 0x0); // Disable GPIO1 interrupt output

	// set the provided I2C address
	write8(I2C_SLAVE__DEVICE_ADDRESS, new_addr);
	// use the new address in the future
	address=new_addr<<1;
}

void VL6180X::startRangeContinous(void) {
	write8(SYSRANGE__START, 0x03);
}

uint8_t VL6180X::readRange(void) {
	return read8(RESULT__RANGE_VAL);
}

void VL6180X::clearInterrupts(void) {
	write8(SYSTEM__INTERRUPT_CLEAR, 0x07);
}

void VL6180X::poll(void) {
	// wait for new measurement ready status
	while(read8(RESULT__INTERRUPT_STATUS_GPIO) != 0x04);
}

uint8_t VL6180X::read8(uint16_t reg) {
	uint8_t packet;
	HAL_I2C_Mem_Read_IT(i2c_handle, address, reg, I2C_MEMADD_SIZE_16BIT, &packet, 1);
	if(osSemaphoreWait(i2c1_semaphore_id, 2) != osOK) {
		i2c1_fail=1;
		// reinit the I2C structure - dirty but works
		hi2c1.ErrorCode = 0;
		hi2c1.State = HAL_I2C_STATE_READY;
		hi2c1.PreviousState = 0;
		hi2c1.Mode = HAL_I2C_MODE_NONE;
	}
	return packet;
}

uint16_t VL6180X::read16(uint16_t reg) {
	uint8_t packet[2];
	HAL_I2C_Mem_Read_IT(i2c_handle, address, reg, I2C_MEMADD_SIZE_16BIT, packet, 2);
	osSemaphoreWait(i2c1_semaphore_id, osWaitForever);
	return (uint16_t)(packet[1]<<1) | packet[0];
}

uint32_t VL6180X::read32(uint16_t reg) {
	uint8_t packet[4];
	HAL_I2C_Mem_Read_IT(i2c_handle, address, reg, I2C_MEMADD_SIZE_16BIT, packet, 4);
	osSemaphoreWait(i2c1_semaphore_id, osWaitForever);
	return (uint32_t)(packet[3]<<1) | (packet[2]<<1) | (packet[1]<<1) | packet[0];
}

void VL6180X::write8(uint16_t reg, uint8_t data) {
	HAL_I2C_Mem_Write_IT(i2c_handle, (uint16_t) address, reg, I2C_MEMADD_SIZE_16BIT, &data, 1);
	if(osSemaphoreWait(i2c1_semaphore_id, 2) != osOK) {
		i2c1_fail=1;
		// reinit the I2C structure - dirty but works
		hi2c1.ErrorCode = 0;
		hi2c1.State = HAL_I2C_STATE_READY;
		hi2c1.PreviousState = 0;
		hi2c1.Mode = HAL_I2C_MODE_NONE;
	}
}
void VL6180X::write16(uint16_t reg, uint16_t data) {
	uint8_t packet[]={(data>>8) & 0xFF, data & 0xFF};
	HAL_I2C_Mem_Write_IT(i2c_handle, (uint16_t) address, reg, I2C_MEMADD_SIZE_16BIT, packet, 2);
	osSemaphoreWait(i2c1_semaphore_id, osWaitForever);
}
void VL6180X::write32(uint16_t reg, uint32_t data) {
	uint8_t packet[]={(data>>24) & 0xFF, (data>>16) & 0xFF, (data>>8) & 0xFF, data & 0xFF};
	HAL_I2C_Mem_Write_IT(i2c_handle, (uint16_t) address, reg, I2C_MEMADD_SIZE_16BIT, packet, 4);
	osSemaphoreWait(i2c1_semaphore_id, osWaitForever);
}
