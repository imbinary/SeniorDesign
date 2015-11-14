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
	nmea_generateChecksum(tmp, bsm);
	xbeeUARTprintf("%s\n", bsm);
	xSemaphoreGive(g_xBsmDataSemaphore);
	//TODO remove this it is for testing
	//bsmParse(bsm);
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
						tmpBSMData.date, tmpBSMData.latAccel,
						tmpBSMData.longAccel, tmpBSMData.vertAccel,
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

void calcAlert(rBSMData_t tmpBSMData) {
	//TODO calculate stuff
	Vector v1p, v1d, v2p, v2d;

	// send alert to queue
	if (xQueue1 != 0) {
		uint8_t byte1, byte2, dir;
		//construct the bytes
		dir = direction(deg2dec(g_rBSMData.latitiude),
				deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitiude),
				deg2dec(tmpBSMData.longitude), 'K');

		byte1 = dir / 11;

		int size;

		size = 7
				- (((int) (distance(deg2dec(g_rBSMData.latitiude),
						deg2dec(g_rBSMData.longitude),
						deg2dec(tmpBSMData.latitiude),
						deg2dec(tmpBSMData.longitude), 'm'))
						* (int) tmpBSMData.speed) / 4);

		//v1 us v2 them p start position d speed
		v1p.x = g_rBSMData.longitude;
		v1p.y = g_rBSMData.latitiude;
		v2p.x = tmpBSMData.longitude;
		v2p.y = tmpBSMData.latitiude;

		//calc speed in x(longitude) y(latitude) components using heading

		v1d.x = g_rBSMData.speed * cos(deg2rad(g_rBSMData.heading));
		v1d.y = g_rBSMData.speed * sin(deg2rad(g_rBSMData.heading));
		v2d.x = tmpBSMData.speed * cos(deg2rad(tmpBSMData.heading));
		v2d.y = tmpBSMData.speed * sin(deg2rad(tmpBSMData.heading));

		// t contains intersection parameters
		Intersection t = intersectVectors(v1p, v1d, v2p, v2d);
		uint8_t color = 0x0f;

		xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
		if (size <= 5) {
			// far away use intersection with constant speed
			//sprintf(msg, " %0.6f %0.6f", t.parameter1, t.parameter2);
			if (abs(t.parameter1 - t.parameter2) < 2)
				color += 0x0f;
			if (abs(t.parameter1 - t.parameter2) < 4)
				color += 0x0f;
			if (abs(t.parameter1 - t.parameter2) < 8)
				color += 0x0f;
			if (abs(t.parameter1 - t.parameter2) < 16)
				color += 0x0f;
			UARTprintf("far direction: %d size: %d -- %d color\n", byte1, size,
					color);
		} else {
			//close use accel data to calculate danger
			// -y left +y right +x forward -x back
			if (dir >= 45 && dir < 135) { //east +y

			}
			if (dir >= 135 && dir < 225) { //south -x

			}
			if (dir >= 225 && dir < 315) { //west -y

			}
			if (dir >= 315 || dir < 45) { //north +x

			}
			uint8_t color = abs(t.parameter1 - t.parameter2);
			UARTprintf("close direction: %d size: %d -- %d\n", byte1, size,
					color);
		}
		xSemaphoreGive(g_xUARTSemaphore);
		// set dir and size
		byte1 = (byte1 * 8) + size;
		// set color
		byte2 = color;
		//set night bit
		if (tmpBSMData.btime > 20000 && tmpBSMData.btime < 140000)
			byte2 |= 0x80;

		uint16_t tmp = (byte1 << 8) | byte2;
		xQueueSendToBackFromISR(xQueue1, &tmp, 0);

	}

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
