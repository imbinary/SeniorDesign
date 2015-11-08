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
#include "adxl_task.h"
#include "command_task.h"
#include "ui_task.h"
#include "uiuart.h"
#include "ravvn.h"
#define UI_INPUT_BUF_SIZE  80

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xUIHandle;
extern xQueueHandle xQueue1;
//*****************************************************************************
//
// A mutex semaphore to manage the UART buffer in utils
//rtstdio. Before
// calling UARTprintf each task must take this semaphore.
//
//*****************************************************************************
xSemaphoreHandle g_uiUARTSemaphore;

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
void ConfigureUIUART(uint32_t ui32SysClock) {
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	//
	// Enable UART3
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PC4_U7RX);
	ROM_GPIOPinConfigure(GPIO_PC5_U7TX);
	ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	//
	// Configure the UART for 115,200, 8-N-1 operation. ui
	//
	//  UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 9600,
	//                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	//                          UART_CONFIG_PAR_NONE));

	//
	// Use the system clock for the UART.
	//
	UARTClockSourceSet(UART7_BASE, UART_CLOCK_SYSTEM);

	//
	// Initialize the UART for console I/O.
	//
	uiUARTxConfig(7, 9600, ui32SysClock);

}

//*****************************************************************************
//
// The main function of the Command Task.
//
//*****************************************************************************
static void UITask(void *pvParameters) {
	portTickType xLastWakeTime;
	int32_t i32DollarPosition;
	char cInput[UI_INPUT_BUF_SIZE];
	uint16_t Atest;
	//int8_t test=0;

	//
	// Get the current time as a reference to start our delays.
	//
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{

		//
		// Wait for the required amount of time to check back.
		//
		vTaskDelayUntil(&xLastWakeTime, UI_TASK_PERIOD_MS /
		portTICK_RATE_MS);

		// Peek at the buffer to see if a \r is there.  If so we have a
		// complete command that needs processing. Make sure your terminal
		// sends a \r when you press 'enter'.
		//
		i32DollarPosition = uiUARTPeek('\r');

		if (i32DollarPosition != (-1)) {
			//
			// Take the ui semaphore.
			//
			xSemaphoreTake(g_uiUARTSemaphore, portMAX_DELAY);
			uiUARTgets(cInput, UI_INPUT_BUF_SIZE);
			UARTprintf("%s\n", cInput);
			xSemaphoreGive(g_uiUARTSemaphore);
		}
		//uiUARTprintf("%s","$73");
		//test = ++test%10;

		uint8_t byte1, byte2;
		int j;

		if(xQueue1 != 0){
			//UARTprintf("queue count: %d\n",uxQueueMessagesWaiting( xQueue1 ));

			for(j=0;j<2;j++){
				if(xQueueReceive(xQueue1, &(Atest),0)){

					byte2 = Atest;
					byte1 = (Atest >> 8);
					int i;
					xSemaphoreTake(g_uiUARTSemaphore, portMAX_DELAY);
					for(i=0;i<5;i++){
						uiUARTprintf("$%c%c",byte1,byte2);
					}
					xSemaphoreGive(g_uiUARTSemaphore);

				}
			}
		}


		/* DEMO STUFF */
		/*
		  		uint8_t color=0x01;
				uint8_t dir=0;
				uint16_t tim = 0;

				uint8_t siz = 0;
		  		tim = UIQpointer->time;
				dir = UIQpointer->dir;
				color = UIQpointer->color;
				siz = UIQpointer->size;
				byte1 = dir * 8 + siz;
				byte2 = color + night * 128;
				int i;
				for(i=0;i<4;i++){
					uiUARTprintf("$%c%c",byte1,byte2);
					UARTprintf("$%c%c",byte1,byte2);
				}
				dir += 3;

				if(dir >= 32){
					dir = 0;
					color += 4;
					if(color >= 0x7F)
							color = 0x01;}
							*/
	}
}
//*****************************************************************************
//
// Initializes the Command task.
//
//*****************************************************************************
		uint32_t UITaskInit(void) {
			//
			// Configure the UART and the UARTStdio library.
			//
			ConfigureUIUART(g_ui32SysClock);

			//
			// Make sure the UARTStdioIntHandler priority is low to not interfere
			// with the RTOS. This may not be needed since the int handler does not
			// call FreeRTOS functions ("fromISR" or otherwise).
			//
			IntPrioritySet(INT_UART7, 0xE0);

			//
			// Create a mutex to guard the UART.
			//
			g_uiUARTSemaphore = xSemaphoreCreateMutex();

			//
			// Create the switch task.
			//
			if (xTaskCreate(UITask, (signed portCHAR *)"ui",
					UI_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
					PRIORITY_UI_TASK, g_xUIHandle) != pdTRUE) {
				//
				// Task creation failed.
				//
				return (1);
			}

			//
			// Check if queue creation and semaphore was successful.
			//
			if (g_uiUARTSemaphore == NULL) {
				//
				// queue was not created successfully.
				//
				return (1);
			}

			return (0);

		}
