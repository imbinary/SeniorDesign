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
		sprintf(tmp, "B,%0.4f,%0.4f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f",
				g_rBSMData.latitiude, g_rBSMData.longitude, g_rBSMData.speed,
				g_rBSMData.heading, g_rBSMData.time, g_rBSMData.date,
				g_rBSMData.latAccel, g_rBSMData.longAccel, g_rBSMData.vertAccel,
				g_rBSMData.yawRate);
	} else {
		//todo change time to status
		sprintf(tmp, "I,%0.4f,%0.4f,%d,%0.1f", g_rBSMData.latitiude,
				g_rBSMData.longitude, g_rBSMData.heading, g_rBSMData.time);
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

	if (nmea_validateChecksum(cInput, XBEE_INPUT_BUF_SIZE )) {
		//UARTprintf("%s\n", cInput);
		char** tokens;
		tokens = str_split(cInput, ',');

		if (tokens) {
			int i;
			if (!strcmp(tokens[0], "$B")) {
				if(tokens[1])
					tmpBSMData.latitiude = strtod(tokens[1], NULL);
				if(tokens[2])
					tmpBSMData.longitude = strtod(tokens[2], NULL);
				if(tokens[3])
					tmpBSMData.speed = strtol(tokens[3], NULL, 10);
				if(tokens[4])
					tmpBSMData.heading = strtod(tokens[4], NULL);
				if(tokens[5])
					tmpBSMData.time = strtol(tokens[5], NULL, 10);
				if(tokens[6])
					tmpBSMData.date = strtod(tokens[6], NULL);
				if(tokens[7])
					tmpBSMData.latAccel = strtod(tokens[7], NULL);
				if(tokens[8])
					tmpBSMData.longAccel = strtod(tokens[8], NULL);
				if(tokens[9])
					tmpBSMData.vertAccel = strtod(tokens[9], NULL);
				if(tokens[10])
					tmpBSMData.yawRate = strtol(tokens[10], NULL, 10);

				sprintf(bsm,
						"$B,%0.4f,%0.4f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f,%0.2f,%d,%0.5f",
						tmpBSMData.latitiude, tmpBSMData.longitude,
						tmpBSMData.speed, tmpBSMData.heading, tmpBSMData.time,
						tmpBSMData.date, tmpBSMData.latAccel, tmpBSMData.longAccel,
						tmpBSMData.vertAccel, tmpBSMData.yawRate,
						distance(deg2dec(g_rBSMData.latitiude), deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitiude), deg2dec(tmpBSMData.longitude), 'K'),
						direction(deg2dec(g_rBSMData.latitiude), deg2dec(g_rBSMData.longitude),deg2dec(tmpBSMData.latitiude), deg2dec(tmpBSMData.longitude), 'K'),
						deg2dec(tmpBSMData.latitiude));

				xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
				UARTprintf("%s\n", bsm);
				xSemaphoreGive(g_xUARTSemaphore);

				//TODO calculate stuff

				// send alert to queue
				if (xQueue1 != 0) {
					uint8_t byte1, byte2;
					//construct the bytes
					byte1 = direction(deg2dec(g_rBSMData.latitiude), deg2dec(g_rBSMData.longitude),deg2dec(tmpBSMData.latitiude), deg2dec(tmpBSMData.longitude), 'K')/11 * 8 + 2;
					byte2 = 0x34;
					//set night bit
					if (tmpBSMData.time > 20000 && tmpBSMData.time < 140000)
						byte2 |= 0x80;
					uint16_t tmp = (byte1 << 8) | byte2;
					xQueueSendToBackFromISR(xQueue1, &tmp, 0);

				}

			}
			// free memory
			for (i = 0; *(tokens + i); i++) {
				vPortFree(*(tokens + i));
			}
			vPortFree(tokens);
		}
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

		i32DollarPosition = xbeeUARTPeek('*');

		if (i32DollarPosition != (-1)) {
			//
			// Take the xbee semaphore.
			//
			xSemaphoreTake(g_xbeeUARTSemaphore, portMAX_DELAY);
			int t = xbeeUARTgets(cInput, XBEE_INPUT_BUF_SIZE);
			xSemaphoreGive(g_xbeeUARTSemaphore);

			//xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
			//UARTprintf(">%d\n", t);
			//xSemaphoreGive(g_xUARTSemaphore);
			bsmParse(cInput);
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
