//*****************************************************************************
//
// command_task.c - Virtual COM Port Task manage messages to and from terminal.
//
// Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.1.71 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "drivers/pinout.h"
#include "drivers/buttons.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "adxl_task.h"
#include "command_task.h"
#include "xbee_task.h"
#include "xbeeuart.h"
#include "ravvn.h"
#include "util.h"

#define XBEE_INPUT_BUF_SIZE  120
#define BSM_SIZE  120

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xXBEEHandle;
extern xQueueHandle xQueue1;
extern xSemaphoreHandle g_xBsmDataSemaphore;
//*****************************************************************************
//
// A mutex semaphore to manage the UART buffer in utils
// calling UARTprintf each task must take this semaphore.
//
//*****************************************************************************
xSemaphoreHandle g_xbeeUARTSemaphore;

//*****************************************************************************
//
// Global variable to hold the system clock speed.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;

//extern rBSMData_t g_rBSMData;

void calcAlert(rBSMData_t tmpBSMData);
uint8_t calcDir(rBSMData_t tmpBSMData);
uint8_t calcColor(rBSMData_t tmpBSMData, int size, int dist);
float tCollide(int dist, int bear, float myVeloc, int myHead, float otherVeloc,
		int otherHead);
float min(float v1, float v2);


float oldTime;
//*****************************************************************************
//
//
//
//*****************************************************************************
void bsmSend() {
	char tmp[BSM_SIZE];
	char bsm[BSM_SIZE];
	xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);

	if (DTYPE) {
		sprintf(tmp, "B,%0.6f,%0.6f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f",
				g_rBSMData.latitiude, g_rBSMData.longitude, g_rBSMData.speed,
				g_rBSMData.heading, g_rBSMData.btime, g_rBSMData.date,
				g_rBSMData.latAccel, g_rBSMData.longAccel, g_rBSMData.vertAccel,
				g_rBSMData.yawRate);
	} else {
		//todo change time to status
		sprintf(tmp, "I,%0.6f,%0.6f,%d,%0.1f", g_rBSMData.latitiude,
				g_rBSMData.longitude, g_rBSMData.heading, g_rBSMData.btime);
	}
	xSemaphoreGive(g_xBsmDataSemaphore);
	if (g_rBSMData.btime == oldTime || g_rBSMData.date == 0)
		return;
	oldTime = g_rBSMData.btime;
	nmea_generateChecksum(tmp, bsm);
	xbeeUARTprintf("%s\n", bsm);

}

//*****************************************************************************
//
//
//
//*****************************************************************************
void bsmParse(char *cInput) {
	rBSMData_t tmpBSMData;
	char bsm[BSM_SIZE];
	int i;

	if (nmea_validateChecksum(cInput, XBEE_INPUT_BUF_SIZE)) {
		char** tokens;
		tokens = str_split(cInput, ',');
		if (tokens) {

			if (!strcmp(tokens[0], "$B")) {
				tmpBSMData.latitiude = strtod(tokens[1], NULL);
				tmpBSMData.longitude = strtod(tokens[2], NULL);
				tmpBSMData.speed = strtod(tokens[3], NULL);
				tmpBSMData.heading = strtol(tokens[4], NULL, 10);
				tmpBSMData.btime = strtod(tokens[5], NULL);
				tmpBSMData.date = strtol(tokens[6], NULL, 10);
				tmpBSMData.latAccel = strtol(tokens[7], NULL, 10);
				tmpBSMData.longAccel = strtol(tokens[8], NULL, 10);
				tmpBSMData.vertAccel = strtol(tokens[9], NULL, 10);
				tmpBSMData.yawRate = strtod(tokens[10], NULL);

				sprintf(bsm,
						"$B,%0.6f,%0.6f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f,%0.4f,%d",
						tmpBSMData.latitiude, tmpBSMData.longitude,
						tmpBSMData.speed, tmpBSMData.heading, tmpBSMData.btime,
						tmpBSMData.date, tmpBSMData.latAccel * 29,
						tmpBSMData.longAccel * 29, tmpBSMData.vertAccel * 29,
						tmpBSMData.yawRate,
						distance(deg2dec(g_rBSMData.latitiude),
								deg2dec(g_rBSMData.longitude),
								deg2dec(tmpBSMData.latitiude),
								deg2dec(tmpBSMData.longitude), 'm'),
						direction(deg2dec(g_rBSMData.latitiude),
								deg2dec(g_rBSMData.longitude),
								deg2dec(tmpBSMData.latitiude),
								deg2dec(tmpBSMData.longitude), 'K'));

				xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
				UARTprintf("%s\n", bsm);
				xSemaphoreGive(g_xUARTSemaphore);

				calcAlert(tmpBSMData);

			}

		}
		// free memory
		for (i = 0; *(tokens + i); i++) {
			vPortFree(*(tokens + i));
		}
		vPortFree(tokens);
	}

}

float tCollide(int dist, int bear, float myVeloc, int myHead, float otherVeloc,
		int otherHead) {
	float d_y = dist * cos(deg2rad(bear)); //y component of distance in meters
	float d_x = dist * sin(deg2rad(bear)); //x component of distance in meters
	float V_ry = otherVeloc * cos(deg2rad(otherHead))
			- myVeloc * cos(deg2rad(myHead)); //y component of relative velocity
	float V_rx = otherVeloc * sin(deg2rad(otherHead))
			- myVeloc * sin(deg2rad(myHead)); //x component of relative velocity

	float V_r = pow(V_ry,2) + pow(V_rx,2); //relative velocity
	if (V_r == 0)
		return -1; //travelling parallel at same velocity, same direction. No collision

	float exp1 = -pow(d_x,2) * pow(V_ry,2) + 2 * d_x * d_y * V_rx * V_ry
			- pow(d_y,2) * pow(V_rx,2) + 4 * (V_r); //expression 1 of solution
	if (exp1 < 0)
		return -1; //solution is not real-paths are parallel, circles do not collide

	exp1 = sqrt(exp1); //previous statement avoides taking square root of negative

	float exp2 = d_x * V_rx - d_y * V_ry; //expression 2 of solution
	float sol1 = (exp1 - exp2) / V_r; //solution 1
	float sol2 = (-exp1 - exp2) / V_r; //solution 2

	if (sol1 < 0 && sol2 < 0)
		return -1; //vehicles are moving away from eachother
	else if (sol1 >= 0 && sol2 < 0)
		return sol1; //sol1 is time to collision
	else if (sol1 < 0 && sol2 >= 0)
		return sol2; //sol2 is time to collision
	else
		return min(sol1, sol2); //the minimum of sol1 and sol2 is time to collision

}

float min(float v1, float v2){
	if(v1<v2)
		return v1;
	return v2;
}


void calcAlert(rBSMData_t tmpBSMData) {

	// send alert to queue
	if (xQueue1 != 0) {
		uint8_t byte1, byte2;
		int size, dist;
		//construct the bytes

		//calc distance
		dist = (int) (distance(deg2dec(g_rBSMData.latitiude),
				deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitiude),
				deg2dec(tmpBSMData.longitude), 'm'));
		if (dist > 60)
			return;

		byte1 = calcDir(tmpBSMData);

		//size relative to distance
		if (dist > 15)
			size = 0;
		else if (dist < 3)
			size = 7;
		else {
			float tmp3 = dist * (-7 / 12.0) + (35 / 4.0);
			size = tmp3;
		}

		// set dir and size
		byte1 = (byte1 * 8) + size;
		// set color
		byte2 = calcColor(tmpBSMData, size, dist);
		//set night bit
		if (tmpBSMData.btime > 20000 && tmpBSMData.btime < 140000)
			byte2 |= 0x80;

		uint16_t tmp = (byte1 << 8) | byte2;
		xQueueSendToBackFromISR(xQueue1, &tmp, 0);

	}

}

uint8_t calcColor(rBSMData_t tmpBSMData, int size, int dist) {
	uint8_t color = 0x0f;
	int16_t dir;

	dir = direction(deg2dec(g_rBSMData.latitiude),
			deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitiude),
			deg2dec(tmpBSMData.longitude), 'K');
	dir = g_rBSMData.heading - dir;
	if (dir <= 0)
		dir += 360;

	float coll = tCollide(dist, dir, g_rBSMData.speed, g_rBSMData.heading , tmpBSMData.speed,
			tmpBSMData.heading);

	xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
	if (size <= 7) {
		// far away use intersection with constant speed

		if (coll < 0 || coll > 12)
			color = 1;
		else
			color = ((coll * -10.5) +127);

		UARTprintf("far size: %d color: %x color %d\n", size, color, coll);
	}
	xSemaphoreGive(g_xUARTSemaphore);
	return color;
}

uint8_t calcDir(rBSMData_t tmpBSMData) {
	int16_t dir;

	dir = direction(deg2dec(g_rBSMData.latitiude),
			deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitiude),
			deg2dec(tmpBSMData.longitude), 'K');
	dir = g_rBSMData.heading - dir;
	if (dir <= 0)
		dir += 360;
	dir = dir * 4 / 45;
	return (uint8_t) dir;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureXBEEUART(uint32_t ui32SysClock) {
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	//
	// Enable UART3
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

	//
	// Configure GPIO Pins for UART mode.
	//

	ROM_GPIOPinConfigure(GPIO_PK0_U4RX);
	ROM_GPIOPinConfigure(GPIO_PK1_U4TX);
	ROM_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the system clock for the UART.
	//
	UARTClockSourceSet(UART4_BASE, UART_CLOCK_SYSTEM);

	//
	// Initialize the UART for console I/O.
	//
	xbeeUARTxConfig(4, 115200, ui32SysClock);

	xbeeUARTEchoSet(false);
}

//*****************************************************************************
//
// The main function of the Command Task.
//
//*****************************************************************************
static void XBEETask(void *pvParameters) {
	portTickType xLastWakeTime;
	int32_t i32DollarPosition;
	char cInput[XBEE_INPUT_BUF_SIZE];
	int8_t i;
	//
	// Get the current time as a reference to start our delays.
	//
	xLastWakeTime = xTaskGetTickCount();

	while (1) {

		//
		// Wait for the required amount of time to check back.
		//
		vTaskDelayUntil(&xLastWakeTime, XBEE_TASK_PERIOD_MS /
		portTICK_RATE_MS);
		//get up to 5 messages
		for (i = 0; i < 4; i++) {
			i32DollarPosition = xbeeUARTPeek('*');

			if (i32DollarPosition != (-1)) {

				xSemaphoreTake(g_xbeeUARTSemaphore, portMAX_DELAY);
				int t = xbeeUARTgetr(cInput, XBEE_INPUT_BUF_SIZE);
				xSemaphoreGive(g_xbeeUARTSemaphore);

				bsmParse(cInput);
			}
		}
		bsmSend();
	}
}

//*****************************************************************************
//
// Initializes the Command task.
//
//*****************************************************************************
uint32_t XBEETaskInit(void) {
	//
	// Configure the UART and the UARTStdio library.
	//
	ConfigureXBEEUART(g_ui32SysClock);

	oldTime = -1.0;
	//
	// Make sure the UARTStdioIntHandler priority is low to not interfere
	// with the RTOS. This may not be needed since the int handler does not
	// call FreeRTOS functions ("fromISR" or otherwise).
	//
	IntPrioritySet(INT_UART4, 0xE0);

	//
	// Create a mutex to guard the UART.
	//
	g_xbeeUARTSemaphore = xSemaphoreCreateMutex();

	//
	// Create the switch task.
	//
	if (xTaskCreate(XBEETask, (signed portCHAR *)"xbee",
			XBEE_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_XBEE_TASK, g_xXBEEHandle) != pdTRUE) {
		//
		// Task creation failed.
		//
		return (1);
	}

	//
	// Check if queue creation and semaphore was successful.
	//
	if (g_xbeeUARTSemaphore == NULL) {
		//
		// queue was not created successfully.
		//
		return (1);
	}

	return (0);

}
