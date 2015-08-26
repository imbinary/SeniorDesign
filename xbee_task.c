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
#include "compdcm_task.h"
#include "command_task.h"
#include "xbee_task.h"
#include "xbeeuart.h"
#include "ravvn.h"

#define XBEE_INPUT_BUF_SIZE  80
#define BSM_SIZE  50
//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xXBEEHandle;

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
	g_rBSMData.latitiude = 81.1;
	xbeeUARTprintf("$B%s\n", "test");
	if(DTYPE){
		xbeeUARTprintf("$B%f%d%f%f%f%d%d%f%f%f%f%c%d%d\n", g_rBSMData.latitiude,g_rBSMData.elevation,g_rBSMData.longitude,g_rBSMData.pAccuracy,
			g_rBSMData.speed,g_rBSMData.heading,g_rBSMData.steeringAngle,g_rBSMData.longAccel,g_rBSMData.vertAccel,g_rBSMData.latAccel,g_rBSMData.yawRate
			,g_rBSMData.brake,g_rBSMData.length,g_rBSMData.width);
	}else
	{
		xbeeUARTprintf("$I%f%d%f%d%d\n", g_rBSMData.latitiude,g_rBSMData.elevation,g_rBSMData.longitude,g_rBSMData.heading,1);
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
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    //
    // Enable UART3
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

    //
    // Configure GPIO Pins for UART mode.
    //
    //ROM_GPIOPinConfigure(GPIO_PA6_U2RX);
    //ROM_GPIOPinConfigure(GPIO_PA7_U2TX);
    //ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    ROM_GPIOPinConfigure(GPIO_PK0_U4RX);
    ROM_GPIOPinConfigure(GPIO_PK1_U4TX);
    ROM_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation. xbee
    //
  //  UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 9600,
   //                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
   //                          UART_CONFIG_PAR_NONE));

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
        vTaskDelayUntil(&xLastWakeTime, COMMAND_TASK_PERIOD_MS /
                        portTICK_RATE_MS);

		// Peek at the buffer to see if a \r is there.  If so we have a
		// complete command that needs processing. Make sure your terminal
		// sends a \r when you press 'enter'.
		//
		i32DollarPosition = xbeeUARTPeek('$');

		if(i32DollarPosition != (-1))
			{
				//
				// Take the xbee semaphore.
				//
				xSemaphoreTake(g_xbeeUARTSemaphore, portMAX_DELAY);
				//xbeereadUART();
				xbeeUARTgets(cInput, XBEE_INPUT_BUF_SIZE);
				UARTprintf("%s\n",cInput);
				xSemaphoreGive(g_xbeeUARTSemaphore);
			}
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
