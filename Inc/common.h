/* COMMON THINGS - QUEUES, SEMAPHORES ETC
 * common.h
 *
 *  Created on: Jun 30, 2016
 *      Author: luktor99
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <motion.h>
#include <threads.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define TIMER TIM5->CNT

extern uint16_t adc_lipo[3];
extern float heading;
extern float targetX, targetY;
extern float testvar;
extern uint32_t debug;

// Buffers' lengths
#define UART3_TX_bufferlen		500 // max length of print() function output
#define UART3_TX_bufferlenCLI 	500 // size of CLI's output buffer
#define UART3_RX_bufferlenCLI 	500 // size of CLI's input buffer


// Global variables
extern MotionCtrl Motion;
extern char buffer_CLI_RX[UART3_RX_bufferlenCLI];
extern char buffer_CLI_TX[UART3_TX_bufferlenCLI];
extern char buffer_CLI_RX_prev[UART3_RX_bufferlenCLI];

extern uint8_t ranges[10];


// *** Queues ***
const osMessageQDef_t uart3rx_queue = {200, sizeof(uint8_t)};
extern osMessageQId uart3rx_queue_id;
const osMessageQDef_t vl6180x_queue = {30, sizeof(uint8_t)};
extern osMessageQId vl6180x_queue_id;
const osMessageQDef_t buttons_queue = {10, sizeof(uint8_t)};
extern osMessageQId buttons_queue_id;

// *** Mutexes ***


// *** Semaphores ***
const osSemaphoreDef_t uart3tx_semaphore = {0};
extern osSemaphoreId uart3tx_semaphore_id;
const osSemaphoreDef_t i2c1_semaphore = {0};
extern osSemaphoreId i2c1_semaphore_id;
const osSemaphoreDef_t i2c2_semaphore = {0};
extern osSemaphoreId i2c2_semaphore_id;
const osSemaphoreDef_t i2c3_semaphore = {0};
extern osSemaphoreId i2c3_semaphore_id;


// *** Utility functions ***
// UART print function (like printf)
void print(const char *format, ...);
// print CLI TX buffer
void printCLI(char *buffer);
// print a single character
void printChar(uint8_t ch);

// clamp angle to -pi..pi range
float clampAngle(float ang);

enum buttons {B_OK=0, B_UP, B_DOWN, B_LEFT, B_RIGHT};

#endif /* COMMON_H_ */
