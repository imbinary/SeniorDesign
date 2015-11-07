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

#define XBEE_INPUT_BUF_SIZE  80
#define BSM_SIZE  50

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xXBEEHandle;
extern xQueueHandle xQueue1;
//*****************************************************************************
//
// A mutex semaphore to manage the UART buffer in utils\uartstdio.  Before
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
// Global flag indicates if we are online currently.
//
//*****************************************************************************
extern bool g_bOnline;

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
XBEEreadUART(){
	//xbeeUARTgets(cInput, COMMAND_INPUT_BUF_SIZE);
	UARTprintf("%c", ROM_UARTCharGetNonBlocking(UART3_BASE));

}

//*****************************************************************************
//
//
//
//*****************************************************************************
void
bsmSend(){
	char bsm[BSM_SIZE];
	//xbeeUARTgets(cInput, COMMAND_INPUT_BUF_SIZE);
	//g_rBSMData.latitiude = 81.1;

	if(DTYPE){
		sprintf(bsm, "$B,%0.4f,%0.4f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f", g_rBSMData.latitiude,
				g_rBSMData.longitude, g_rBSMData.speed, g_rBSMData.heading, g_rBSMData.time, g_rBSMData.date,
				g_rBSMData.latAccel, g_rBSMData.longAccel, g_rBSMData.vertAccel, g_rBSMData.yawRate);
		xbeeUARTprintf("%s\n", bsm);
	}else
	{
		//todo change time to status
		sprintf(bsm, "$I,%0.4f,%0.4f,%d,%0.1f", g_rBSMData.latitiude, g_rBSMData.longitude, g_rBSMData.heading, g_rBSMData.time);
		xbeeUARTprintf("%s\n", bsm);
	}
	bsmParse(bsm);
}


//*****************************************************************************
//
//
//
//*****************************************************************************
void
bsmParse(char *cInput){
	//g_rBSMData.latitiude = 81.1;
	rBSMData_t tmpBSMData;
	char** tokens;
	char bsm[BSM_SIZE];


	tokens = str_split(cInput, ',');
	/*
	sprintf(bsm, "$B,%0.4f,%0.4f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f", g_rBSMData.latitiude,
					g_rBSMData.longitude, g_rBSMData.speed, g_rBSMData.heading, g_rBSMData.time, g_rBSMData.date,
					g_rBSMData.latAccel, g_rBSMData.longAccel, g_rBSMData.vertAccel, g_rBSMData.yawRate);

	*/
	if (tokens)
	{
		int i;
		if(!strcmp(tokens[0],"$B")){
			tmpBSMData.latitiude = strtod(tokens[1],NULL);
			tmpBSMData.longitude = strtod(tokens[2],NULL);
			tmpBSMData.speed = strtol(tokens[3],NULL,10);
			tmpBSMData.heading = strtod(tokens[4],NULL);
			tmpBSMData.time = strtol(tokens[5],NULL,10);
			tmpBSMData.date = strtod(tokens[6],NULL);
			tmpBSMData.latAccel = strtod(tokens[7],NULL);
			tmpBSMData.longAccel = strtod(tokens[8],NULL);
			tmpBSMData.vertAccel = strtod(tokens[9],NULL);
			tmpBSMData.yawRate = strtol(tokens[10],NULL,10);

			sprintf(bsm, "$B,%0.4f,%0.4f,%0.2f,%d,%0.1f,%d,%d,%d,%d,%0.5f,%0.2f", tmpBSMData.latitiude,
					tmpBSMData.longitude, tmpBSMData.speed, tmpBSMData.heading, tmpBSMData.time, tmpBSMData.date,
					tmpBSMData.latAccel, tmpBSMData.longAccel, tmpBSMData.vertAccel, tmpBSMData.yawRate,distance(28.505121,-81.429598,28.522604,-81.464130,'K'));
			//UARTprintf("%s\n", bsm);
		}
		// free memory
		for (i = 0; *(tokens + i); i++)
		{
		   // UARTprintf("parts=[%s]\n", *(tokens + i));
			vPortFree(*(tokens + i));
		}
		//UARTprintf("\n");
		vPortFree(tokens);
	}

	//TODO calculate stuff


	 if( xQueue1 != 0 )
	    {
		 	 uint8_t byte1, byte2, night;

		//construct the bytes
			byte1 = 16 * 8 + 2;
			byte2 = 0x34; //color * 128;
		 //set night bit
			 if(tmpBSMData.time > 20000 && tmpBSMData.time < 140000)
				 byte2 |= 0x80;
			//byte2 *= 128;
			uint16_t tmp = (byte1 << 8)|byte2;
	        xQueueSendToBackFromISR( xQueue1,  &tmp, 0 );


	    }

}
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureXBEEUART(uint32_t ui32SysClock)
{
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

}



//*****************************************************************************
//
// The main function of the Command Task.
//
//*****************************************************************************
static void
XBEETask(void *pvParameters)
{
    portTickType xLastWakeTime;
    int32_t i32DollarPosition;
    char cInput[XBEE_INPUT_BUF_SIZE];

    //
    // Get the current time as a reference to start our delays.
    //
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {

        //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&xLastWakeTime, XBEE_TASK_PERIOD_MS /
                        portTICK_RATE_MS);

		// Peek at the buffer to see if a \r is there.  If so we have a
		// complete command that needs processing. Make sure your terminal
		// sends a \r when you press 'enter'.
		//
		i32DollarPosition = xbeeUARTPeek('\r');

		if(i32DollarPosition != (-1))
			{
				//
				// Take the xbee semaphore.
				//
				xSemaphoreTake(g_xbeeUARTSemaphore, portMAX_DELAY);
				//xbeereadUART();
				xbeeUARTgets(cInput, XBEE_INPUT_BUF_SIZE);
				UARTprintf("%s\n",cInput);
				bsmParse(cInput);
				xSemaphoreGive(g_xbeeUARTSemaphore);
			}
		bsmSend();
    }
}

//*****************************************************************************
//
// Initializes the Command task.
//
//*****************************************************************************
uint32_t XBEETaskInit(void)
{
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
    if(xTaskCreate(XBEETask, (signed portCHAR *)"xbee",
    			XBEE_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_XBEE_TASK, g_xXBEEHandle) != pdTRUE)
    {
        //
        // Task creation failed.
        //
        return(1);
    }

    //
    // Check if queue creation and semaphore was successful.
    //
    if(g_xbeeUARTSemaphore == NULL)
    {
        //
        // queue was not created successfully.
        //
        return(1);
    }

    return(0);

}
