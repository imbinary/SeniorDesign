//*****************************************************************************
//
// Xbee task
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
float oldTime;
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
		sprintf(tmp, "B,%0.6f,%0.6f,%0.2f,%d,%0.1f,%d,%d,%d,%d",
				g_rBSMData.latitiude, g_rBSMData.longitude, g_rBSMData.speed,
				g_rBSMData.heading, g_rBSMData.btime, g_rBSMData.date,
				g_rBSMData.latAccel, g_rBSMData.longAccel,
				g_rBSMData.vertAccel);
	} else {
		//todo change time to status
		int color = 0x0f;
		sprintf(tmp, "I,%0.6f,%0.6f,%d,%0.1f,%d", g_rBSMData.latitiude,
				g_rBSMData.longitude, g_rBSMData.heading, g_rBSMData.btime,
				color);
	}
	xSemaphoreGive(g_xBsmDataSemaphore);

	if (g_rBSMData.date == 0 || oldTime == g_rBSMData.btime)
		return;
	oldTime = g_rBSMData.btime;

	nmea_generateChecksum(tmp, bsm);

	xSemaphoreTake(g_xbeeUARTSemaphore, portMAX_DELAY);
	xbeeUARTprintf("%s\n", bsm);
	xSemaphoreGive(g_xbeeUARTSemaphore);

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
