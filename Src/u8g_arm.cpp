/*
 * u8g_arm.cpp
 *
 *  Created on: Aug 19, 2016
 *      Author: luktor99
 */
#include <common.h>
#include "u8g_arm.h"

u8g_t u8g;
uint8_t OLEDbuffer[OLED_BUFFER_SIZE];

void u8g_Delay(uint16_t val) {
	osDelay(val);
}

uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
	switch(msg) {
		case U8G_COM_MSG_ADDRESS:
			if (arg_val == 0) OLEDbuffer[0] = 0; // command mode
			else OLEDbuffer[0] = 0x40; // data mode
			break;

		case U8G_COM_MSG_WRITE_BYTE:
			OLEDbuffer[1] = arg_val;

			HAL_I2C_Master_Transmit_IT(&hi2c3, ADDR_OLED, (uint8_t*)OLEDbuffer, 2);
			LED7_GPIO_Port->BSRR = (uint32_t)LED7_Pin;
			osSemaphoreWait(i2c3_semaphore_id, 300);
			break;

		case U8G_COM_MSG_WRITE_SEQ:
		case U8G_COM_MSG_WRITE_SEQ_P:
			memcpy(OLEDbuffer+1, arg_ptr, arg_val);

			HAL_I2C_Master_Transmit_DMA(&hi2c3, ADDR_OLED, (uint8_t *)OLEDbuffer, arg_val+1);
			LED7_GPIO_Port->BSRR = (uint32_t)LED7_Pin;
			osSemaphoreWait(i2c3_semaphore_id, 300);
			break;
		default:
			break;
	}

	return 1;
}
