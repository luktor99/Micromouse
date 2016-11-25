/*
 * threads.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: luktor99
 */
#include <common.h>
#include <oled.h>
#include <math.h>
#include <threads.h>
#include <vl6180x.h>
#include <MPU6050.h>
#include <trajectory.h>
#include <maze.h>
#include <bitmaps/bitmap_splash.h>

uint8_t buttons_states[5] = { 0, 0, 0, 0, 0 };

osThreadId defaultTaskHandle;
osThreadId buttonsPollingTaskHandle;
osThreadId motionControlTaskHandle;
osThreadId rangeSensorsTaskHandle;
osThreadId rangeSensorsWDGTaskHandle;
osThreadId cliTaskHandle;
osThreadId activityLEDsTaskHandle;
osThreadId OLEDTaskHandle;
osThreadId LiPoMonitorTaskHandle;
osThreadId MazeAlgorithmTaskHandle;

Menu menu(MENU_ITEMS);
void m_reboot(Menu *m, uint8_t parent);
void m_author1(Menu *m, uint8_t parent);
void m_author2(Menu *m, uint8_t parent);
void m_author3(Menu *m, uint8_t parent);
void m_author4(Menu *m, uint8_t parent);
void m_sensor_vl6180x(Menu *m, uint8_t parent);
void m_sensor_encoders(Menu *m, uint8_t parent);
void m_sensor_mpu6050(Menu *m, uint8_t parent);
void m_battery(Menu *m, uint8_t parent);
void m_localisation(Menu *m, uint8_t parent);
void m_walls(Menu *m, uint8_t parent);
void m_run(Menu *m, uint8_t parent);

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

void createThreads(void) {
	// DEBUG
	osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 2560);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	// Buttons polling task
	osThreadDef(buttonsPollingTask, StartButtonsPollingTask, osPriorityLow, 0,
			128);
	buttonsPollingTaskHandle = osThreadCreate(osThread(buttonsPollingTask),
			NULL);
	// Motors control task
	osThreadDef(motionControlTask, StartMotionControlTask, osPriorityRealtime,
			0, 2560);
	motionControlTaskHandle = osThreadCreate(osThread(motionControlTask), NULL);
	// Range sensors reading task
	osThreadDef(rangeSensorsTask, StartRangeSensorsTask, osPriorityRealtime, 0,
			1280);
	rangeSensorsTaskHandle = osThreadCreate(osThread(rangeSensorsTask), NULL);
	// Command line interface task
	osThreadDef(cliTask, StartCLITask, osPriorityRealtime, 0, 2560); // normally PriorityNormal, realtime for debugging
	cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);
	// Activity LEDs task
	osThreadDef(activityLEDsTask, StartActivityLEDsTask, osPriorityLow, 0, 128);
	activityLEDsTaskHandle = osThreadCreate(osThread(activityLEDsTask), NULL);
	// OLED screen interface task
	osThreadDef(OLEDTask, StartOLEDTask, osPriorityBelowNormal, 0, 2560);
	OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);
	// LiPo voltage monitor task
	osThreadDef(LiPoMonitorTask, StartLiPoMonitorTask, osPriorityNormal, 0,
			128);
	LiPoMonitorTaskHandle = osThreadCreate(osThread(LiPoMonitorTask), NULL);
	// Maze algorithm task
	osThreadDef(MazeAlgorithmTask, StartMazeAlgorithmTask, osPriorityHigh, 0,
			8192); //5120
	MazeAlgorithmTaskHandle = osThreadCreate(osThread(MazeAlgorithmTask), NULL);
}

void menu_inflate(void) {
	enum menus {
		MENU_MAIN, MENU_SETTINGS, MENU_EXTRAS, MENU_AUTHORS, MENU_SENSORS
	};
	menu.set_menu(MENU_MAIN);
	menu.add_func("Run", m_run);
	menu.add_goto("Settings", MENU_SETTINGS);
	menu.add_goto("Extras", MENU_EXTRAS);
	menu.add_goto("Authors", MENU_AUTHORS);
	menu.add_func("Reboot", m_reboot);

	menu.set_menu(MENU_SETTINGS);
	menu.add_goto("« Back", MENU_MAIN);
	menu.add_text("Option 1");

	menu.set_menu(MENU_AUTHORS);
	menu.add_goto("« Back", MENU_MAIN);
	menu.add_func("Lukasz Kilaszewski", m_author1);
	menu.add_func("Karol Skorulski", m_author2);
	menu.add_func("Jerzy Baranowski", m_author3);
	menu.add_func("Andrzej Halicki", m_author4);

	menu.set_menu(MENU_EXTRAS);
	menu.add_goto("« Back", MENU_MAIN);
	menu.add_goto("Sensor data", MENU_SENSORS);
	menu.add_func("Battery info", m_battery);
	menu.add_func("Localisation", m_localisation);
	menu.add_func("Wall detection", m_walls);

	menu.set_menu(MENU_SENSORS);
	menu.add_goto("« Back", MENU_EXTRAS);
	menu.add_func("Encoders", m_sensor_encoders);
	menu.add_func("VL6180X", m_sensor_vl6180x);
	menu.add_func("MPU6050", m_sensor_mpu6050);
}

void StartDefaultTask(void const * argument) {
	// Generate an example trajectory
//	Trajectory.addSearchMove(MS_RIGHT);
//	for(uint16_t i=0; i<100; i++) {
//		Trajectory.addSearchMove(MS_RIGHT);
//		Trajectory.addSearchMove(MS_BACK);
//		Trajectory.addSearchMove(MS_LEFT);
//		Trajectory.addSearchMove(MS_BACKLEFT);
//		Trajectory.addSearchMove(MS_FORWARD);
//		Trajectory.addSearchMove(MS_BACKRIGHT);
//		Trajectory.addSearchMove(MS_LEFT);
//		Trajectory.addSearchMove(MS_BACKLEFT);
//	}

	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
}

void StartButtonsPollingTask(void const * argument) {
	// Initialize the buttons interface queue
	buttons_queue_id = osMessageCreate(&buttons_queue, NULL);

	const uint8_t debounce_cycles = 11; // (11-1)*5ms=50ms
	const uint8_t hold_cycles = 81; // (81-1)*5ms=400ms
	static uint8_t buttons_debounce[] = { 0, 0, 0, 0, 0 };
	static uint8_t buttons_held[] = { 0, 0, 0, 0, 0 };
	const uint8_t buttons_pins[] = { SWITCH_OK_Pin, SWITCH_UP_Pin,
			SWITCH_DOWN_Pin, SWITCH_LEFT_Pin, SWITCH_RIGHT_Pin };
	/* Infinite loop */
	for (;;) {
		for (uint8_t i = 0; i < 5; i++) {
			// decrease counters (debouncing)
			buttons_debounce[i] -= (buttons_debounce[i] > 0);

			// check if button was pressed
			if (HAL_GPIO_ReadPin(SWITCH_OK_GPIO_Port, buttons_pins[i])) {
				buttons_debounce[i] = debounce_cycles;
				buttons_held[i] = 0;
			} else if (buttons_debounce[i] == 1
					|| buttons_held[i] == hold_cycles) {
				// button pressed - process event
				buttons_states[i] = !buttons_states[i]; // temporary
				buttons_held[i] = 0;

				// send key to the queue so it can be processed by another task
				osMessagePut(buttons_queue_id, i, 0);

				// Buzzer ON
				BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
				// LED8 ON
				LED8_GPIO_Port->BSRR = LED8_Pin;

				osDelay(8);

				// Buzzer OFF
				BUZZER_GPIO_Port->BSRR = BUZZER_Pin << 16U;
				// LED8 OFF
				LED8_GPIO_Port->BSRR = LED8_Pin << 16U;
			} else {
				buttons_held[i]++;
			}
		}
		osDelay(5);
	}
}

void StartMotionControlTask(void const * argument) {
	// initialize encoders interface
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	// Initialize motion control
	Motion.init();

	// I2C2 semaphore initialization
	i2c2_semaphore_id = osSemaphoreCreate(&i2c2_semaphore, 1);
	// Take the semaphore, so next osSemaphoreWait() will actually wait
	osSemaphoreWait(i2c2_semaphore_id, osWaitForever);

	// Test is sensor is responding
	if (!mpu6050.testConnection()) {
		HAL_NVIC_SystemReset();
	}
	// Reinitialize MPU6050 sensor
	mpu6050.reset();
	osDelay(100);
	mpu6050.initialize();
	mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	// Start PID timer
	HAL_TIM_Base_Start_IT(&htim9);

	// reset robot's heading and position
	Motion.resetLocalisation();

	/* Infinite loop */
	for (;;) {
		osSignalWait(1, osWaitForever); // wait for 1kHz tick signal
		uint32_t timer = TIMER; // used to check if this takes more than 1ms (failure)

		Trajectory.tick(); //
		Motion.tick(); // Reads gyro, performs motor PID and localisation update

		uint16_t tick_length = TIMER - timer; // used to check if this loop takes more than 1ms (failure)
		if (tick_length > 500)
			print("MOTION TICK TOO LONG!\r\n"); // something went wrong
	}
}

void StartRangeSensorsTask(void const * argument) {
	// I2C1 semaphore initialization
	i2c1_semaphore_id = osSemaphoreCreate(&i2c1_semaphore, 1);
	// Take the semaphore, so next osSemaphoreWait() will actually wait
	osSemaphoreWait(i2c1_semaphore_id, osWaitForever);

	// Initialize sensor read queue
	vl6180x_queue_id = osMessageCreate(&vl6180x_queue, NULL);

	// Power off all range sensors
	HAL_GPIO_WritePin(SENSORS_POWER_GPIO_Port, SENSORS_POWER_Pin, GPIO_PIN_SET);
	osDelay(200);

	// Power on all the sensors
	HAL_GPIO_WritePin(SENSORS_POWER_GPIO_Port, SENSORS_POWER_Pin,
			GPIO_PIN_RESET);
	osDelay(200);

	// Initialize I2C
	MX_I2C1_Init();

	// initialize the sensors and start continous ranging mode
	for (uint8_t i = 0; i < 10; i++) {
		sensors[i].init(i, SENSOR_ADDR_START + i);
		osDelay(5);
		sensors[i].startRangeContinous();
		osDelay(5);
	}

	// Configure data ready interrupts
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = SENSOR1_Pin | SENSOR2_Pin | SENSOR3_Pin
			| SENSOR4_Pin | SENSOR5_Pin | SENSOR6_Pin | SENSOR7_Pin
			| SENSOR8_Pin | SENSOR10_Pin | SENSOR11_Pin;
	HAL_GPIO_Init(SENSOR1_GPIO_Port, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ (EXTI3_IRQn);
	HAL_NVIC_EnableIRQ (EXTI4_IRQn);
	HAL_NVIC_EnableIRQ (EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);

	// clear interrupt registers
	for (uint8_t i = 0; i < 10; i++) {
		sensors[i].clearInterrupts();
	}

	/* Infinite loop */
	for (;;) {
		// wait for sensor interrupt
		osEvent event = osMessageGet(vl6180x_queue_id, 11);
		if (event.status == osEventMessage) {
			// retrieve ID of the sensor to be read
			uint8_t sensor_id = event.value.v;

			// LED6 ON
			LED6_GPIO_Port->BSRR = (uint32_t) LED6_Pin;
			// Read range from sensor and reset its interrupt flag
			uint8_t max_tries = 5;
			do {
				i2c1_fail = 0;
				ranges[sensor_id] = sensors[sensor_id].readRange();
				sensors[sensor_id].clearInterrupts();
				max_tries--;
			} while (i2c1_fail && max_tries);
			// LED6 OFF
			LED6_GPIO_Port->BSRR = (uint32_t) LED6_Pin << 16U;
		} else {
			//something went wrong - reset interrupts
			for (uint8_t i = 0; i < 10; i++) {
				sensors[i].clearInterrupts();

			}
		}
	}
}

void StartCLITask(void const * argument) {
	uint8_t status;
	uint16_t RX_counter = 0;

	// UART3 RX queue initialization
	uart3rx_queue_id = osMessageCreate(&uart3rx_queue, NULL);
	// UART1 TX semaphore initialization
	uart3tx_semaphore_id = osSemaphoreCreate(&uart3tx_semaphore, 1);
	// Start UART3 data reception
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	// Register all CLI commands
	register_commands();

	// Print the welcome message and prompt
	print(CLI_welcome_message);
	print(CLI_prompt);

	/* Infinite loop */
	for (;;) {
		// Wait for a new character
		osEvent event = osMessageGet(uart3rx_queue_id, osWaitForever);
		if (event.status == osEventMessage) {
			// Character received
			uint8_t character = event.value.v;

			// Enter pressed
			if (character == '\r') {
				// Reply with an additional line feed
				print("\r\n");

				// Let the interpreter do its job
				do {
					// Interpret the command
					status = FreeRTOS_CLIProcessCommand(
							(const char* const ) buffer_CLI_RX, buffer_CLI_TX,
							UART3_TX_bufferlenCLI);

					// Print the output
					printCLI(buffer_CLI_TX);
				} while (status != pdFALSE);

				// Command processed. Clear up
				RX_counter = 0;
				memcpy(buffer_CLI_RX_prev, buffer_CLI_RX,
						UART3_RX_bufferlenCLI);
				memset(buffer_CLI_RX, 0x0, UART3_RX_bufferlenCLI);
				print(CLI_prompt);
			} else if (character == '\b' || character == 127) { // Backspace
			// Remove the last character from buffer if possible
				if (RX_counter > 0) {
					RX_counter--;
					buffer_CLI_RX[RX_counter] = '\0';
					printChar(127);
				}
			} else if (character >= 32 && character <= 126) {
				// Process printable characters:
				if (RX_counter < UART3_TX_bufferlenCLI) {
					// check if up arrow key has been pressed
					if (RX_counter == 1 && buffer_CLI_RX[0] == '['
							&& character == 'A') {
						// print the last command
						print("\b");
						print(buffer_CLI_RX_prev);
						// restore previous command
						memcpy(buffer_CLI_RX, buffer_CLI_RX_prev,
								UART3_RX_bufferlenCLI);
						// execute it by sending a fake ENTER key
						osMessagePut(uart3rx_queue_id, '\r', 0);
					} else {
						// else process the key normally
						buffer_CLI_RX[RX_counter] = character;
						RX_counter++;
						printChar(character);
					}
				}
			}
		}
	}
}

void StartActivityLEDsTask(void const * argument) {
	for (;;) {
		// clear UART RX, TX LEDs
		LED1_GPIO_Port->BSRR = (uint32_t) LED1_Pin << 16U;
		LED2_GPIO_Port->BSRR = (uint32_t) LED2_Pin << 16U;
		// clear OLED refresh LED
		LED7_GPIO_Port->BSRR = (uint32_t) LED7_Pin << 16U;

		osDelay(10);
	}
}

void StartOLEDTask(void const * argument) {
	// I2C3 semaphore initialization
	i2c3_semaphore_id = osSemaphoreCreate(&i2c3_semaphore, 1);
	// Take the semaphore, so next osSemaphoreWait() will actually wait
	osSemaphoreWait(i2c3_semaphore_id, osWaitForever);

	// check if I2C periph requires restarting - problem fix
	if (I2C3->SR2 & I2C_SR2_BUSY)
		I2C3->CR1 = I2C_CR1_SWRST;

	osDelay(100);
	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
	osDelay(100);

	// draw splash image
	u8g_FirstPage(&u8g);
	do {
		u8g_DrawBitmap(&u8g, 0, 0, bitmap_splash_cnt, bitmap_splash_h,
				bitmap_splash);
	} while (u8g_NextPage(&u8g));

	osDelay(1500);

	// draw menu for the first time
	menu.draw();

	for (;;) {
		// menu control:
		osEvent event = osMessageGet(buttons_queue_id, 500);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			menu.handle(key);
		} else {
			// refresh the screen to update battery voltage (every 500ms)
			menu.draw();
		}
	}
}

void StartLiPoMonitorTask(void const * argument) {
	const uint16_t lipo_cell_thresh = 320; // low cell voltage warning @ 3.20V
	const uint16_t lipo_pack_thresh = 640; // pack low voltage warning @ 6.40V
	const uint16_t avg_samples = 500;
	uint32_t sum = 0;
	uint8_t buzzer_on = 0;

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	for (;;) {
		sConfig.Channel = ADC_CHANNEL_4;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		sum = 0;
		for (uint16_t i = 0; i < avg_samples; i++) {
			HAL_ADC_Start(&hadc1);
			osDelay(1);
			sum += HAL_ADC_GetValue(&hadc1);
		}
		adc_lipo[0] = sum / avg_samples * 770 / 3420; // cell2+cell1

		// Disable buzzer
		if (buzzer_on) {
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			buzzer_on = 0;
		}

		sConfig.Channel = ADC_CHANNEL_10;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		sum = 0;
		for (uint16_t i = 0; i < avg_samples; i++) {
			HAL_ADC_Start(&hadc1);
			osDelay(1);
			sum += HAL_ADC_GetValue(&hadc1);
		}
		adc_lipo[1] = sum / avg_samples * 382 / 3590; // cell1
		adc_lipo[2] = adc_lipo[0] - adc_lipo[1]; // cell2

		// Enable buzzer if:
		//	a. LiPo battery is connected using 3-pin connector and it's low
		//  b. LiPo battery is connected using 2-pin connector and it's low (but must be above USB's 5.25V in case that's the power source)
		if (adc_lipo[1] > 80
				&& (adc_lipo[1] < lipo_cell_thresh
						|| adc_lipo[2] < lipo_cell_thresh)
				|| adc_lipo[1] < 80
						&& (adc_lipo[0] > 525 && adc_lipo[0] < lipo_pack_thresh)) {
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			buzzer_on = 1;
		}
	}
}

void StartMazeAlgorithmTask(void const * argument) {
	// Set the default (search run) speed
	Motion.velLinMax = speed_searchrun;
	speed_dmps = 3;

	/*debugging fast fast moves functions*/
//	Motion.resetLocalisation();
//	Trajectory.reset();
//	Trajectory.updateTarget(0.0);
//	Trajectory.addSearchMove(M_START);
//	Trajectory.addFastMove(MF_FORWARD);
//	Trajectory.addFastMove(MF_RIGHT);
//
//	Trajectory.loadCurve();
//	Trajectory.updateTarget(0.0);
//	for(;;)
//	{
//	osSignalWait(SIGNAL_SCAN, osWaitForever);
//	Motion.disable();
//	}
	/*end of debugging functions*/
	// SEARCH RUNS LOOP
	bool fastrun = false;
	for(;;) {
	 // Go to the first border, to initiate the first scan
	 Motion.resetLocalisation();
	 Trajectory.reset();
	 Trajectory.addSearchMove(M_START);
	 // Set the first target point
	 Trajectory.loadCurve();
	 Trajectory.updateTarget(0.0);

	 for(;;) {
	 // wait for a scan signal
	 osSignalWait(SIGNAL_SCAN, osWaitForever);
	 // enable buzzer
	 //BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
	 // DEBUG: print wall info
	 //print("\r\nS%u%u%u -> ", Motion.wallState[LEFT], Motion.wallState[FRONT], Motion.wallState[RIGHT]);
	 uint8_t result=maze.nextscanstep();
	 if(result != MAZE_STEP) {
	 if(result == MAZE_FASTRUN){
	 fastrun=true;
	 }
	 // wait for the last command to be executed
	 osSignalWait(SIGNAL_SCAN, osWaitForever);
	 // stop motors
	 Motion.disable();
	 // disable buzzer
	 //BUZZER_GPIO_Port->BSRR = BUZZER_Pin << 16;
	 break;
	 }
	 osDelay(2);
	 // disable buzzer
	 BUZZER_GPIO_Port->BSRR = BUZZER_Pin << 16;
	 }
	 if(fastrun) break;
	 }

	 // Update the OLED text
	 scanning_step=1;
	 // Set fast run speed
	 Motion.velLinMax=speed_fastrun;
	 speed_dmps=4;
	 // SCANNING DONE - BEEP
	 for(uint8_t i=0; i<20; i++) {
	 osDelay(50);
	 BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
	 osDelay(50);
	 BUZZER_GPIO_Port->BSRR = BUZZER_Pin << 16;
	 }

	 // FAST RUNS LOOP
	 for(;;) {
	 // Go to the first border, to initiate the first scan
	 Motion.resetLocalisation();
	 Trajectory.reset();

	 // Calculate fast run path
	 maze.szybko();

	 // Set the first target point
	 Trajectory.loadCurve();
	 Trajectory.updateTarget(0.0);

	 osSignalWait(SIGNAL_SCAN, osWaitForever);
	 Motion.disable();
	 }

}

// OLED screen "apps" below
void m_reboot(Menu *m, uint8_t parent) {
	m->current_item->text = "Rebooting...";
	m->draw(); // redraw menu

	osDelay(500);
	HAL_NVIC_SystemReset(); // perform a system reset
}

void m_author1(Menu *m, uint8_t parent) {
	static uint8_t show_email = 1;
	if (show_email)
		m->current_item->text = "luktor99@gmail.com";
	else
		m->current_item->text = "Lukasz Kilaszewski";

	show_email = !show_email;
}

void m_author2(Menu *m, uint8_t parent) {
	static uint8_t show_email = 1;
	if (show_email)
		m->current_item->text = "karol@blueray.com";
	else
		m->current_item->text = "Karol Skorulski";

	show_email = !show_email;
}

void m_author3(Menu *m, uint8_t parent) {
	static uint8_t show_email = 1;
	if (show_email)
		m->current_item->text = "jerzy@blueray.com";
	else
		m->current_item->text = "Jerzy Baranowski";

	show_email = !show_email;
}

void m_author4(Menu *m, uint8_t parent) {
	static uint8_t show_email = 1;
	if (show_email)
		m->current_item->text = "andrzej@blueray.com";
	else
		m->current_item->text = "Andrzej Halicki";

	show_email = !show_email;
}

void m_sensor_vl6180x(Menu *m, uint8_t parent) {
	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		char buffer[3];
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();

			uint8_t posY = 10;
			u8g_DrawStr(&u8g, 0, posY, "FFL:");
			sprintf(buffer, "%u", ranges[FFL]);
			u8g_DrawStr(&u8g, 30, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "FFR:");
			sprintf(buffer, "%u", ranges[FFR]);
			u8g_DrawStr(&u8g, 93, posY, buffer);
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "FDL:");
			sprintf(buffer, "%u", ranges[FDL]);
			u8g_DrawStr(&u8g, 30, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "FDR:");
			sprintf(buffer, "%u", ranges[FDR]);
			u8g_DrawStr(&u8g, 93, posY, buffer);
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "FSL:");
			sprintf(buffer, "%u", ranges[FSL]);
			u8g_DrawStr(&u8g, 30, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "FSR:");
			sprintf(buffer, "%u", ranges[FSR]);
			u8g_DrawStr(&u8g, 93, posY, buffer);
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "RSL:");
			sprintf(buffer, "%u", ranges[RSL]);
			u8g_DrawStr(&u8g, 30, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "RSR:");
			sprintf(buffer, "%u", ranges[RSR]);
			u8g_DrawStr(&u8g, 93, posY, buffer);
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "RRL:");
			sprintf(buffer, "%u", ranges[RRL]);
			u8g_DrawStr(&u8g, 30, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "RRR:");
			sprintf(buffer, "%u", ranges[RRR]);
			u8g_DrawStr(&u8g, 93, posY, buffer);
			posY += 9;
		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 50);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
		}
	}
}

void m_sensor_mpu6050(Menu *m, uint8_t parent) {
	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		char buffer[3];
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();

			uint8_t posY = 10;
			u8g_DrawStr(&u8g, 0, posY, "ACCEL");
			u8g_DrawStr(&u8g, 63, posY, "GYRO");
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "x:");
			sprintf(buffer, "%d", Motion.ax);
			u8g_DrawStr(&u8g, 20, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "x:");
			sprintf(buffer, "%d", Motion.gx);
			u8g_DrawStr(&u8g, 83, posY, buffer);
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "y:");
			sprintf(buffer, "%d", Motion.ay);
			u8g_DrawStr(&u8g, 20, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "y:");
			sprintf(buffer, "%d", Motion.gy);
			u8g_DrawStr(&u8g, 83, posY, buffer);
			posY += 9;

			u8g_DrawStr(&u8g, 0, posY, "z:");
			sprintf(buffer, "%d", Motion.az);
			u8g_DrawStr(&u8g, 20, posY, buffer);
			u8g_DrawStr(&u8g, 63, posY, "z:");
			sprintf(buffer, "%d", Motion.gz);
			u8g_DrawStr(&u8g, 83, posY, buffer);
		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 50);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
		}
	}
}

void m_sensor_encoders(Menu *m, uint8_t parent) {
	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		char buffer[5];
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();

			uint8_t posY = 10;
			u8g_DrawStr(&u8g, 0, posY, "EncL:");
			sprintf(buffer, "%lu", __HAL_TIM_GET_COUNTER(&htim2));
			u8g_DrawStr(&u8g, 36, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "EncR:");
			sprintf(buffer, "%lu", __HAL_TIM_GET_COUNTER(&htim3));
			u8g_DrawStr(&u8g, 36, posY, buffer);
			posY += 18;
			u8g_DrawStr(&u8g, 0, posY, "M.EncL:");
			sprintf(buffer, "%ld", (int32_t) Motion.encL);
			u8g_DrawStr(&u8g, 48, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "M.EncR:");
			sprintf(buffer, "%ld", (int32_t) Motion.encR);
			u8g_DrawStr(&u8g, 48, posY, buffer);

		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 50);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
		}
	}
}

void m_battery(Menu *m, uint8_t parent) {
	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		char buffer[8];
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();

			uint8_t posY = 10;
			u8g_DrawStr(&u8g, 0, posY, "Sum:");
			sprintf(buffer, "%u.%02uV", adc_lipo[0] / 100, adc_lipo[0] % 100);
			u8g_DrawStr(&u8g, 42, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "Cell1:");
			sprintf(buffer, "%u.%02uV", adc_lipo[1] / 100, adc_lipo[1] % 100);
			u8g_DrawStr(&u8g, 42, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "Cell2:");
			sprintf(buffer, "%u.%02uV", adc_lipo[2] / 100, adc_lipo[2] % 100);
			u8g_DrawStr(&u8g, 42, posY, buffer);
		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 250);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
		}
	}
}

void m_localisation(Menu *m, uint8_t parent) {
	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		char buffer[8];
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();

			uint8_t posY = 10;
			u8g_DrawStr(&u8g, 0, posY, "Heading:");
			sprintf(buffer, "%ld", (int32_t) (Motion.heading / M_PI * 180));
			u8g_DrawStr(&u8g, 54, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "posX:");
			sprintf(buffer, "%ld", (int32_t) (Motion.posX * 1000));
			u8g_DrawStr(&u8g, 54, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "posY:");
			sprintf(buffer, "%ld", (int32_t) (Motion.posY * 1000));
			u8g_DrawStr(&u8g, 54, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "Orient:");
			if (Motion.orientation == 0)
				sprintf(buffer, "UP");
			else if (Motion.orientation == 1)
				sprintf(buffer, "DOWN");
			else if (Motion.orientation == 2)
				sprintf(buffer, "LEFT");
			else if (Motion.orientation == 3)
				sprintf(buffer, "RIGHT");
			u8g_DrawStr(&u8g, 54, posY, buffer);
			posY += 9;
			u8g_DrawStr(&u8g, 0, posY, "Cell:");
			sprintf(buffer, "(%u, %u)", Motion.cellX, Motion.cellY);
			u8g_DrawStr(&u8g, 54, posY, buffer);

		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 100);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
		}
	}
}

void m_walls(Menu *m, uint8_t parent) {
	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();

			if (Motion.wallState[FRONT])
				u8g_DrawBox(&u8g, 49, 20, 30, 8);
			if (Motion.wallState[LEFT])
				u8g_DrawBox(&u8g, 41, 28, 8, 30);
			if (Motion.wallState[RIGHT])
				u8g_DrawBox(&u8g, 79, 28, 8, 30);

		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 100);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
		}
	}
}

void m_run(Menu *m, uint8_t parent) {
	static char buffer[8];

	for (;;) {
		u8g_SetDefaultForegroundColor(&u8g);
		u8g_FirstPage(&u8g);
		do {
			draw_statusbar();
			if (scanning_step == 0)
				u8g_DrawStr(&u8g, 0, 12, "*** SEARCH RUN ***");
			else if (scanning_step == 1)
				u8g_DrawStr(&u8g, 0, 12, "**** FAST RUN ****");
			if (speed_dmps == 3) {
				u8g_DrawStr(&u8g, 0, 24, "0.3 m/s");
			} else if (speed_dmps == 4) {
				u8g_DrawStr(&u8g, 0, 24, "0.4 m/s");
			} else if (speed_dmps == 5) {
				u8g_DrawStr(&u8g, 0, 24, "0.5 m/s");
			} else if (speed_dmps == 6) {
				u8g_DrawStr(&u8g, 0, 24, "0.6 m/s");
			} else if (speed_dmps == 7) {
				u8g_DrawStr(&u8g, 0, 24, "0.7 m/s");
			} else if (speed_dmps == 8) {
				u8g_DrawStr(&u8g, 0, 24, "0.8 m/s");
			} else if (speed_dmps == 9) {
				u8g_DrawStr(&u8g, 0, 24, "0.9 m/s");
			} else {
				u8g_DrawStr(&u8g, 0, 24, ">=1.0 m/s");
			}

			u8g_DrawStr(&u8g, 0, 38, "Press DOWN to start.");
			u8g_DrawStr(&u8g, 0, 48, "Speed: LEFT(-) RIGHT{+)");
		} while (u8g_NextPage(&u8g));

		// check is button has been pressed
		osEvent event = osMessageGet(buttons_queue_id, 100);
		if (event.status == osEventMessage) {
			// Key has been pressed
			uint8_t key = event.value.v;
			if (key == B_OK)
				break; // exit
			else if (key == B_DOWN) {
				// run
				osDelay(1500);
				Motion.calib();
				Motion.resetLocalisation();
				Motion.enable();
			} else if (key == B_UP) {
				// fast run emergency stop
				// Stop the motors
				Motion.disable();
				// Remove the rest of the trajectory
				Trajectory.clear();
				// Force the algorithm to start a fresh fast run
				osSignalSet(MazeAlgorithmTaskHandle, SIGNAL_SCAN);
			} else if (key == B_RIGHT) {
				// Increase the speed
				Motion.velLinMax += speed_fastrun_step;
				speed_dmps += 1;
			} else if (key == B_LEFT) {
				// Decrease the speed
				Motion.velLinMax -= speed_fastrun_step;
				speed_dmps -= 1;
			}
		}
	}
	Motion.disable();
}

extern "C" {
void vApplicationStackOverflowHook(TaskHandle_t xTask,
		signed char *pcTaskName) {
	print("STACK!");
	//vTaskDelete(xTask);
}

void vApplicationTickHook() {
	HAL_IncTick();
}
}
