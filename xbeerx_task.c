//*****************************************************************************
//
// Xbee rx task
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
#include "xbeerx_task.h"
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
xTaskHandle g_xXBEErxHandle;
extern xQueueHandle xQueue1;
extern xSemaphoreHandle g_xBsmDataSemaphore;
//*****************************************************************************
//
// A mutex semaphore to manage the UART buffer in utils
// calling UARTprintf each task must take this semaphore.
//
//*****************************************************************************
extern xSemaphoreHandle g_xbeeUARTSemaphore;

//*****************************************************************************
//
// Global variable to hold the system clock speed.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;



//*****************************************************************************
//
//
//
//*****************************************************************************
	static void XBEErxTask(void *pvParameters) {
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

					//bsmParse(cInput);
				}
			}
		}
	}

//*****************************************************************************
//
// Initializes the Command task.
//
//*****************************************************************************
	uint32_t XBEErxTaskInit(void) {




		//
		// Create the switch task.
		//
		if (xTaskCreate(XBEErxTask, (signed portCHAR *)"xbeerx",
				XBEERX_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
				PRIORITY_XBEE_TASK, g_xXBEErxHandle) != pdTRUE) {
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
