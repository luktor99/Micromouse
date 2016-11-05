/*
 * threads.h
 *
 *  Created on: Jul 2, 2016
 *      Author: luktor99
 */

#ifndef THREADS_H_
#define THREADS_H_

#include <cli_commands.h>
#include <common.h>
#include <FreeRTOS_CLI.h>
#include <hardware.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <vl6180x.h>
#include "oled.h"

// Debug task
extern osThreadId defaultTaskHandle;
// User buttons state polling
extern osThreadId buttonsPollingTaskHandle;
// Motors control task
extern osThreadId motionControlTaskHandle;
// Range sensors reading task
extern osThreadId rangeSensorsTaskHandle;
// Range sensors watchdog task
extern osThreadId rangeSensorsTaskWDGHandle;
// Command line interface task
extern osThreadId cliTaskHandle;
// Activity LEDs task
extern osThreadId activityLEDsTaskHandle;
// OLED screen interface task
extern osThreadId OLEDTaskHandle;
// LiPo voltage monitor task
extern osThreadId LiPoMonitorTaskHandle;
// Maze algorithm task
extern osThreadId MazeAlgorithmTaskHandle;

void StartDefaultTask(void const * argument);
void StartButtonsPollingTask(void const * argument);
void StartMotionControlTask(void const * argument);
void StartRangeSensorsTask(void const * argument);
void StartRangeSensorsWDGTask(void const * argument);
void StartCLITask(void const * argument);
void StartActivityLEDsTask(void const * argument);
void StartOLEDTask(void const * argument);
void StartLiPoMonitorTask(void const * argument);
void StartMazeAlgorithmTask(void const * argument);

// Create all the threads
void createThreads(void);

// Inflate the OLED menu
void menu_inflate(void);

#endif /* THREADS_H_ */
