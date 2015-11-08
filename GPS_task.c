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
#include "utils/uartstdio.h"
#include "drivers/pinout.h"
#include "drivers/buttons.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "adxl_task.h"
#include "command_task.h"
#include "GPS_task.h"
#include "gpsuart.h"
#include "util.h"
#include "xbeeuart.h"
#include "ravvn.h"

#define GPS_INPUT_BUF_SIZE  85


extern xSemaphoreHandle g_xBsmDataSemaphore;


//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xGPSHandle;

//*****************************************************************************
//
// A mutex semaphore to manage the UART buffer in utils
// calling UARTprintf each task must take this semaphore.
//
//*****************************************************************************
xSemaphoreHandle g_gpsUARTSemaphore;

//*****************************************************************************
//
// Global variable to hold the system clock speed.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Global flag indicates if we are online currently.
//
//*****************************************************************************
extern bool g_bOnline;

// this takes a nmea gps string, and validates it againts the checksum
// at the end if the string. (requires first byte to be $)
int8_t nmea_validateChecksum(char *strPtr) {
	int p;
	char c;
	uint8_t chksum;
	uint8_t nmeaChk;
	int8_t flagValid;
	char hx[5] = "0x00";

	flagValid = 1; // we start true, and make it false if things are not right

	if (strPtr[0] != '$') {
		flagValid = 0;
	}

	// if we are still good, test all bytes
	if (flagValid == 1) {
		c = strPtr[1]; // get first chr
		chksum = c;
		p = 2;
		while ((c != '*') && (p < GPS_INPUT_BUF_SIZE)) {
			c = strPtr[p]; // get next chr
			if (c != '*') {
				chksum = chksum ^ c;
			}
			p++;
		}
		// at this point we are either at * or at end of string
		hx[2] = strPtr[p];
		hx[3] = strPtr[p + 1];
		hx[4] = 0x00;
		nmeaChk = strtol(hx, NULL, 16);
		if (chksum != nmeaChk) {
			flagValid = 0;
		}
	}

	return flagValid;
}

// this returns a single binary byte that is the checksum
// you must convert it to hex if you are going to print it or send it
const char * nmea_generateChecksum(char *strPtr, char *dstStr) {
	int p;
	char c;
	uint8_t chksum;

	c = strPtr[0]; // get first chr
	chksum = c;
	p = 1;
	while (c != 0x00) {
		c = strPtr[p]; // get next chr
		if (c != 0x00) {
			chksum = chksum ^ c;
		}
		p++;
	}
	sprintf(&dstStr[0], "$%s*%02x", strPtr, chksum);
	return dstStr;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void GPSparse(char *gpsString) {
	//"$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n"
	//todo maybe add a semephore
	if (gpsString[0] != '$')
		return;
	if (nmea_validateChecksum(gpsString)) {
	    char** tokens;

	    xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);

	    tokens = str_split(gpsString, ',');

	    if (tokens)
	    {
	        int i;
	        if((!strcmp(tokens[2],"A")) && (!strcmp(tokens[0],"$GPRMC"))){
	        	g_rBSMData.time = strtod(tokens[1],NULL);
	        	g_rBSMData.latitiude = strtod(tokens[3],NULL);
	        	if(!strcmp(tokens[4],"S"))
	        		g_rBSMData.latitiude *= -1;
	        	g_rBSMData.longitude = strtod(tokens[5],NULL);
	        	if(!strcmp(tokens[6],"W"))
	        		g_rBSMData.longitude *= -1;
	        	g_rBSMData.speed = strtod(tokens[7],NULL);
	        	g_rBSMData.heading = strtol(tokens[8],NULL,10);
	        	g_rBSMData.date = strtol(tokens[9],NULL,10);
	        }
	        for (i = 0; *(tokens + i); i++)
	        {
	            vPortFree(*(tokens + i));
	        }
	        vPortFree(tokens);
	    }
	    xSemaphoreGive(g_xBsmDataSemaphore);
	} else
	{
		//UARTprintf("> %s\n", gpsString);
	}


}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureGPSUART(uint32_t ui32SysClock) {
	char gpsStr[GPS_INPUT_BUF_SIZE];
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART3
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PA4_U3RX);
	ROM_GPIOPinConfigure(GPIO_PA5_U3TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	//
	// Configure the UART for 115,200, 8-N-1 operation. GPS
	//
	//  UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 9600,
	//                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	//                          UART_CONFIG_PAR_NONE));

	//
	// Use the system clock for the UART.
	//
	UARTClockSourceSet(UART3_BASE, UART_CLOCK_SYSTEM);

	//
	// Initialize the UART for console I/O.
	//
	gpsUARTxConfig(3, 9600, ui32SysClock);

	gpsUARTEchoSet(false);
	gpsUARTprintf("%s\n",
	nmea_generateChecksum("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", gpsStr));
	gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3", gpsStr));
	gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK220,200", gpsStr));
	gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3", gpsStr));
	//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK251,38400",gpsStr));
	//gpsUARTxConfig(3, 38400, ui32SysClock);
	//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3",gpsStr));
	//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3",gpsStr));
	//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3",gpsStr));

}

//*****************************************************************************
//
// The main function of the Command Task.
//
//*****************************************************************************
static void GPSTask(void *pvParameters) {
	portTickType xLastWakeTime;
	int32_t i32DollarPosition;
	char cInput[GPS_INPUT_BUF_SIZE];

	//
	// Get the current time as a reference to start our delays.
	//
	xLastWakeTime = xTaskGetTickCount();

	while (1) {

		 //
		 // Wait for the required amount of time to check back.
		 //
		vTaskDelayUntil(&xLastWakeTime, GPS_TASK_PERIOD_MS /
		portTICK_RATE_MS);

			// Peek at the buffer to see if a \r is there.  If so we have a
			// complete command that needs processing. Make sure your terminal
			// sends a \r when you press 'enter'.
			//
		i32DollarPosition = gpsUARTPeek('*');

		if (i32DollarPosition != (-1)) {
			//
			// Take the gps semaphore.
			//
			xSemaphoreTake(g_gpsUARTSemaphore, portMAX_DELAY);
			gpsUARTgets(cInput, GPS_INPUT_BUF_SIZE);
			GPSparse(cInput);
			xSemaphoreGive(g_gpsUARTSemaphore);
		}
	}
}

//*****************************************************************************
//
// Initializes the Command task.
//
//*****************************************************************************
uint32_t GPSTaskInit(void) {
	//
	// Configure the UART and the UARTStdio library.
	//
	ConfigureGPSUART(g_ui32SysClock);

	//
	// Make sure the UARTStdioIntHandler priority is low to not interfere
	// with the RTOS. This may not be needed since the int handler does not
	// call FreeRTOS functions ("fromISR" or otherwise).
	//
	IntPrioritySet(INT_UART3, 0xE0);

	//
	// Create a mutex to guard the UART.
	//
	g_gpsUARTSemaphore = xSemaphoreCreateMutex();

	//
	// Create the switch task.
	//
	if (xTaskCreate(GPSTask, (signed portCHAR *)"GPS",
		GPS_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
		PRIORITY_GPS_TASK, g_xGPSHandle) != pdTRUE) {
			//
			// Task creation failed.
			//
		return (1);
	}

	//
	// Check if queue creation and semaphore was successful.
	//
	if (g_gpsUARTSemaphore == NULL) {
		//
		// queue was not created successfully.
		//
		return (1);
	}

	return (0);

}
