/*
 * common.cpp
 *
 *  Created on: Jun 30, 2016
 *      Author: luktor99
 */
#include <common.h>
#include <hardware.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

float heading=0.0;
float targetX=0.0, targetY=0.0;
float testvar=0.0;
uint32_t debug;

uint16_t adc_lipo[3];

// Create buffers
uint8_t buffer_print[UART3_TX_bufferlen];
char buffer_CLI_RX[UART3_RX_bufferlenCLI];
char buffer_CLI_TX[UART3_TX_bufferlenCLI];
char buffer_CLI_RX_prev[UART3_RX_bufferlenCLI];

// Declare MotorCtrl instance
MotionCtrl Motion;

uint8_t ranges[10];


// *** Queues ***
osMessageQId uart3rx_queue_id;
osMessageQId vl6180x_queue_id;
osMessageQId buttons_queue_id;


// *** Mutexes ***


// *** Semaphores ***
osSemaphoreId uart3tx_semaphore_id;
osSemaphoreId i2c1_semaphore_id;
osSemaphoreId i2c2_semaphore_id;
osSemaphoreId i2c3_semaphore_id;

// *** Utility functions ***
// UART print function
void print(const char *format, ...) {
    if(osSemaphoreWait(uart3tx_semaphore_id, osWaitForever) == osOK) {
    	// Generate output
        va_list argptr;
        va_start(argptr, format);
        vsprintf((char*)buffer_print, format, argptr);
        va_end(argptr);

    	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buffer_print, strlen((const char*)buffer_print));
    	// Wait for UART to finish
    }
}

// UART print function
void printCLI(char *buffer) {
	uint8_t len=strlen((const char*)buffer);
	if(len>0) {
		if(osSemaphoreWait(uart3tx_semaphore_id, osWaitForever) == osOK) {
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buffer, len);
			// Wait for UART to finish
		}
	}
}

// print a single character
void printChar(uint8_t ch) {
	if(osSemaphoreWait(uart3tx_semaphore_id, osWaitForever) == osOK) {
		HAL_UART_Transmit_DMA(&huart3, &ch, 1);
		// Wait for UART to finish
	}
}

float clampAngle(float ang) {
	if(ang>M_PI)
		return ang-2.0*M_PI;
	else if(ang<-M_PI)
		return ang+2.0*M_PI;
	else
		return ang;
}
