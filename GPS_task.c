//*****************************************************************************
//
// gps task based on TI code
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
uint16_t oldHeading;
bool revFlag;
bool start;
int init_accel;


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


//*****************************************************************************
//
//
//
//*****************************************************************************
void GPSparse(char *gpsString) {
	//"$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n"
	if (gpsString[0] != '$')
		return;
	if (nmea_validateChecksum(gpsString, GPS_INPUT_BUF_SIZE )) {
	    char** tokens;

	    xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);

	    tokens = str_split(gpsString, ',');

	    if (tokens)
	    {
	        int i;
	        if((!strcmp(tokens[2],"A")) && (!strcmp(tokens[0],"$GPRMC"))){
	        	g_rBSMData.btime = strtod(tokens[1],NULL);
	        	g_rBSMData.latitiude = strtod(tokens[3],NULL);
	        	if(!strcmp(tokens[4],"S"))
	        		g_rBSMData.latitiude *= -1;
	        	g_rBSMData.longitude = strtod(tokens[5],NULL);
	        	if(!strcmp(tokens[6],"W"))
	        		g_rBSMData.longitude *= -1;
	        	g_rBSMData.speed = strtod(tokens[7],NULL);
	        	g_rBSMData.speed *=0.51444444444;
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
	}

	//starting
	if(start){
		if( (g_rBSMData.speed >= .05) && (g_rBSMData.speed <= .5) && (abs(g_rBSMData.latAccel) >= 500) ){
			init_accel+= g_rBSMData.latAccel;
		}
		else if( (g_rBSMData.speed >= .5) && (g_rBSMData.latAccel < 0) ){
			start = false;
			revFlag = true;
		}
		else if(g_rBSMData.speed >= .5)
			start = false;
	}
	else
	{
		// stopping
		if( g_rBSMData.speed < .05){
			g_rBSMData.heading = oldHeading;
		}
		else{
			if( abs(g_rBSMData.heading-oldHeading) > 180)
				revFlag=!revFlag;
			oldHeading = g_rBSMData.heading;
		}
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
	gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK220,100", gpsStr)); // 100 for 10 hz
	gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3", gpsStr));
	//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK251,38400",gpsStr));
	//gpsUARTxConfig(3, 38400, ui32SysClock);
	//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,604,3",gpsStr)); // ack


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
	oldHeading = 0;
	revFlag = false;
	start = false;
	init_accel = 0;
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
