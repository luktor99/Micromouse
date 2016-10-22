/*
 * u8g_arm.h
 *
 *  Created on: Aug 19, 2016
 *      Author: luktor99
 */

#ifndef U8G_ARM_H_
#define U8G_ARM_H_

#include <hardware.h>
#include "u8g.h"
#include "stm32f4xx_hal.h"

#define ADDR_OLED			0x78
#define OLED_BUFFER_SIZE	256

uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

extern u8g_t u8g;

#endif /* U8G_ARM_H_ */
