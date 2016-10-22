/*
 * cli_commands.cpp
 *
 *  Created on: Jul 1, 2016
 *      Author: luktor99
 */
#include <cli_commands.h>
#include <common.h>
#include <FreeRTOS_CLI.h>
#include <hardware.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <vl6180x.h>

// Functions:
static BaseType_t funEcho(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdEcho = {
		(const char * const)"echo",
		(const char * const)"Echoes back 1 argument.",
		funEcho,
		1
};
static BaseType_t funClr(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdClr = {
		(const char * const)"clr",
		(const char * const)"Clears the console.",
		funClr,
		0
};

static BaseType_t funMenu(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdMenu = {
		(const char * const)"menu",
		(const char * const)"Enables control of the OLED menu.",
		funMenu,
		0
};

static BaseType_t funRC(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdRC = {
		(const char * const)"rc",
		(const char * const)"Enables RC mode.",
		funRC,
		0
};

static BaseType_t funBattery(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdBattery = {
		(const char * const)"batt",
		(const char * const)"Displays battery state.",
		funBattery,
		0
};

static BaseType_t funTasks(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdTasks = {
		(const char * const)"tasks",
		(const char * const)"Displays CPU utilization stats.",
		funTasks,
		0
};

static BaseType_t funCalib(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdCalib = {
		(const char * const)"calib",
		(const char * const)"Performs gyroscope calibration.",
		funCalib,
		0
};

static BaseType_t funReboot(char *buffer, size_t bufferLen, const char *commandStr);
static const CLI_Command_Definition_t cmdReboot = {
		(const char * const)"reboot",
		(const char * const)"Performs a system reboot.",
		funReboot,
		0
};

static BaseType_t funEcho(char *buffer, size_t bufferLen, const char *commandStr) {
	const char *param1;
	long param1StrLen;

	// Get the parameter
	param1=FreeRTOS_CLIGetParameter((const char *)commandStr, 1, &param1StrLen);

	snprintf(buffer, bufferLen, param1);
	snprintf(buffer+param1StrLen, bufferLen, "\r\n");

	return pdFALSE;
}

static BaseType_t funMenu(char *buffer, size_t bufferLen, const char *commandStr) {
	print("Menu control mode.\r\nUse arrow keys and ENTER to control the menu. Press x to exit.\r\n");

	uint8_t character='\0';
	// Loop
	do {
		// Wait for a new character
		osEvent event = osMessageGet(uart3rx_queue_id, osWaitForever);
		if(event.status == osEventMessage) {
			// Character received
			character=event.value.v;

			switch(character) {
				case 'A':
					osMessagePut(buttons_queue_id, B_UP, osWaitForever);
					break;
				case 'B':
					osMessagePut(buttons_queue_id, B_DOWN, osWaitForever);
					break;
				case 'C':
					osMessagePut(buttons_queue_id, B_RIGHT, osWaitForever);
					break;
				case 'D':
					osMessagePut(buttons_queue_id, B_LEFT, osWaitForever);
					break;
				case '\r':
					osMessagePut(buttons_queue_id, B_OK, osWaitForever);
					break;
				default:
					break;
			}
		}
	} while(character!='x');

	snprintf(buffer, bufferLen, "\r");
	return pdFALSE;
}

static BaseType_t funClr(char *buffer, size_t bufferLen, const char *commandStr) {
	snprintf(buffer, bufferLen, "\f");

	return pdFALSE;
}

static BaseType_t funBattery(char *buffer, size_t bufferLen, const char *commandStr) {
	print("Press x to exit.\r\n\n\n\n\n\n\n\n\n\n");

	// Loop
	uint8_t character='\0';
	do {
		// Wait for a new character
		osEvent event = osMessageGet(uart3rx_queue_id, 100);
		if(event.status == osEventMessage) {
			// Character received
			character=event.value.v;
		}
		//print("\033[1A\r\033[KEncL: %u\r\n\033[KEncR: %u", Motors.encL, Motors.encR);
		//print("\033[2A\r\033[KposL: %u\r\n\033[KencL: %u\r\n\033[KerrL: %f", (uint32_t)Motors.posL, Motors.encL, (float)Motors.errL);
		print("\033[9A\r\033[KFFL: %u\r\n\033[KFFR: %u\r\n\033[KFSR: %u\r\n\033[KRSR: %u\r\n\033[KRRR: %u\r\n\033[KRRL: %u\r\n\033[KRSL: %u\r\n\033[KFSL: %u\r\n\033[KFDL: %u\r\n\033[KFDR: %u", ranges[FFL], ranges[FFR], ranges[FSR], ranges[RSR], ranges[RRR], ranges[RRL], ranges[RSL], ranges[FSL], ranges[FDL], ranges[FDR]);
	} while(character!='x');


	//print("Batt: %.2f", (float)read/3160.*7.4);

	buffer[0]='\n';
	buffer[1]='\r';
	buffer[2]='\0';
	return pdFALSE;
}

static BaseType_t funRC(char *buffer, size_t bufferLen, const char *commandStr) {
	print("Remote control mode enabled.\r\nUse W,A,S,D keys to steer the robot. Press x to exit.\r\n");

	Motion.enable();

	float spd=0.05;
	uint8_t forward=0;

	// Loop
	uint8_t character='\0';
	do {
		// Wait for a new character
		osEvent event = osMessageGet(uart3rx_queue_id, osWaitForever);
		if(event.status == osEventMessage) {
			// Character received
			character=event.value.v;

			switch(character) {
			case 'w':
				if(!forward) {
					Motion.setVelLin(spd);
					forward=1;
				}
				else {
					Motion.setVelLin(0.0);
					forward=0;
				}
				break;
			case 's':
				Motion.setVelLin(0.0);
				Motion.setVelRot(0.0);
				break;
			case 'a':
				Motion.setVelRot(6.28318530718);
				break;
			case 'd':
				Motion.setVelRot(6.28318530718);
				break;
			case 'q':
				break;
			case 'e':
				break;
			case '+':
				spd+=0.05;
				if(forward) Motion.setVelLin(spd);
				break;
			case '-':
				spd-=0.05;
				if(forward) Motion.setVelLin(spd);
				break;
			case '7':
				Motion.setkP(Motion.kP+1.0);
				print("kP: %.3f\r\n", Motion.kP);
				break;
			case '1':
				Motion.setkP(Motion.kP-1.0);
				print("kP: %.3f\r\n", Motion.kP);
				break;
			case '8':
				Motion.setkI(Motion.kI+1.0);
				print("kI: %.3f\r\n", Motion.kI);
				break;
			case '2':
				Motion.setkI(Motion.kI-1.0);
				print("kI: %.3f\r\n", Motion.kI);
				break;
			case '9':
				Motion.setkD(Motion.kD+100.0);
				print("kD: %.3f\r\n", Motion.kD);
				break;
			case '3':
				Motion.setkD(Motion.kD-100.0);
				print("kD: %.3f\r\n", Motion.kD);
				break;
			case 'C':
				heading-=3.1415/2.0;
				break;
			case 'D':
				heading+=3.1415/2.0;
				break;
			case 'v':
				targetX=0.0;
				targetY=0.0;
				break;
			case 'b':
				targetX=0.2;
				targetY=0.0;
				break;
			case 'n':
				targetX=0.2;
				targetY=0.2;
				break;
			case 'm':
				targetX=0.0;
				targetY=0.2;
				break;
			case ' ':
				targetX=2.0;
				targetY=0.0;
				break;
			case ',':
				testvar-=0.01;
				print("%.3f\r\n", testvar);
				break;
			case '.':
				testvar+=0.01;
				print("%.3f\r\n", testvar);
				break;
			}
		}
	} while(character!='x');

	// Stop motors, just in case
	Motion.disable();
	print("Motors stopped.");

	snprintf(buffer, bufferLen, "\r\n");
	return pdFALSE;
}

static BaseType_t funTasks(char *buffer, size_t bufferLen, const char *commandStr) {
	print("\033[1;32mTask\t\tAbs. time\tCPU load\033[0m\r\n");
	vTaskGetRunTimeStats(buffer);
	return pdFALSE;
}

static BaseType_t funCalib(char *buffer, size_t bufferLen, const char *commandStr) {
	float sum=0.0;
	for(uint16_t i=0; i<gyro_avg_count; i++) {
		sum+=Motion.gz_samples[i];
	}
	Motion.gz_bias=sum/(float)gyro_avg_count;

	print("Gyro calibration completed. GyroZ bias: %f", Motion.gz_bias);
	snprintf(buffer, bufferLen, "\r\n");
	return pdFALSE;
}

static BaseType_t funReboot(char *buffer, size_t bufferLen, const char *commandStr) {
	HAL_NVIC_SystemReset();
}

void register_commands(void) {
	FreeRTOS_CLIRegisterCommand(&cmdEcho);
	FreeRTOS_CLIRegisterCommand(&cmdClr);
	FreeRTOS_CLIRegisterCommand(&cmdBattery);
	FreeRTOS_CLIRegisterCommand(&cmdRC);
	FreeRTOS_CLIRegisterCommand(&cmdMenu);
	FreeRTOS_CLIRegisterCommand(&cmdTasks);
	FreeRTOS_CLIRegisterCommand(&cmdCalib);
	FreeRTOS_CLIRegisterCommand(&cmdReboot);
}
