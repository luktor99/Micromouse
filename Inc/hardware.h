/*
 * hardware.h
 *
 *  Created on: Jun 30, 2016
 *      Author: luktor99
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "stm32f4xx_hal.h"

// Provide global access to hardware handlers
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern IWDG_HandleTypeDef hiwdg;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;

// Hardware initialization function
void hardware_init(void);

#endif /* HARDWARE_H_ */
