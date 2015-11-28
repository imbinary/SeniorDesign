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
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
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

void DKFinit(float GPSlat, float GPSlon, float GPShead);
float * DKF(float * Z);
void DKFlininit(float GPSlat, float GPSlon, float GPShead);
float * DKFlin(float * Z);
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

//conversion constants
#define G  		9.88 				//gravitational constant
//convert meters to degrees lat/lon
//#define deltaT  (1 / 10) 		//time between messages

//Sensor noise R, will use R/3 for process noise.
const float R[7] = { 9.213E-7, 3.038E-6, 0.00902, 8.073482933, 393.804, 378.24,
		1005.645 };

float Xo[7] = { 0, 0, 0, 0, 0, 0, 0 }; 		//initial predicted actual values
float Zo[7] = { 0, 0, 0, 0, 0, 0, 0 }; 		//initial predicted sensor readings
float Po[7] = { 0, 0, 0, 0, 0, 0, 0 }; 				//initial predicted error

float Gk[7] = { 0, 0, 0, 0, 0, 0, 0 };			//Kalman Gain
float X[7] = { 0, 0, 0, 0, 0, 0, 0 };			//predictions

float F[7] = { 0, 0, 0, 0, 0, 0, 0 };//matrix for approximation of non-linear f(Xo) functions to scalars a*Xo
float H[7] = { 0, 0, 0, 0, 0, 0, 0 };//matrix for approximation of non-linear h(Xo) functions to scalars c*Xo
float P[7] = { 0, 0, 0, 0, 0, 0, 0 };			//error

//initialize the DKF stuff with initial GPS data
void DKFlininit(float GPSlat, float GPSlon, float GPShead) {
	//initial values for Xo, Zo and Po
	Xo[0] = GPSlat;			// + R[0];//initial predicted actual values
	Xo[1] = GPSlon;			// + R[1];
	Xo[2] = 0.01;
	Xo[3] = GPShead + 5;
	Xo[4] = 100;
	Xo[5] = 100;
	Xo[6] = 10000;

	Po[0] = sqrt(R[0]);			//initial predicted error
	Po[1] = sqrt(R[1]);
	Po[2] = 0.2;
	Po[3] = 5;
	Po[4] = 200;
	Po[5] = 200;
	Po[6] = 500;
}

//call this function with the parameters stuffed in an array. will return the same array
float * DKFlin(float *Z) {//Z[7] = {GPSlat, GPSlon, GPSvel, GPShead, Yacc, Xacc, Zacc}
	uint8_t i;
//the following for-loop completes the predict and update stages
	for (i = 0; i <= 7; i++) {
		X[i] = Xo[i];
		P[i] = Po[i] + R[i] / 2;//error prediction Pk = a*(Pk-1)*a + Q, where R/3 is used as process noise
//update stage
		Gk[i] = P[i] / (P[i] + R[i]);//update kalman gain gk = Pk*c/(c*Pk*c + r)
		Xo[i] = X[i] + Gk[i] * (*(Z + i) - X[i]);//update predicted values for next iteration Xk = Xk + gk(Z - c*Xk)
		Po[i] = (1 - Gk[i]) * P[i];	//update error for next iteration Pk = (1-gk*c)*Pk
	}
	return Xo;			//push out the predicted values
}
//initialize the DKF stuff with initial GPS data
void DKFinit(float GPSlat, float GPSlon, float GPShead) {
//initial values for Xo, Zo and Po
	Xo[0] = GPSlat + R[0];			//initial predicted actual values
	Xo[1] = GPSlon + R[1];
	Xo[2] = 0.01;
	Xo[3] = GPShead + 5;
	Xo[4] = 100;
	Xo[5] = 100;
	Xo[6] = 10000;

	Zo[0] = GPSlat;			//initial predicted sensor readings
	Zo[1] = GPSlon;
	Zo[2] = 0.2;
	Zo[3] = 5;
	Zo[4] = 400;
	Zo[5] = 400;
	Zo[6] = 10000 - 1000;

	Po[0] = sqrt(R[0]);			//initial predicted error
	Po[1] = sqrt(R[1]);
	Po[2] = 0.2;
	Po[3] = 5;
	Po[4] = 200;
	Po[5] = 200;
	Po[6] = 500;

}

//call this function with the parameters stuffed in an array. will return the same array
float* DKF(float * Z) {	//Z[7] = {GPSlat, GPSlon, GPSvel, GPShead, Yacc, Xacc, Zacc}
	uint8_t i;
	float deltaT = (1 / 10);
	float IMUcon = (G / 10000);	//convert decimilliGs to m/s^2
	float m2deg = (1 / 111111);
	//modeling/prediction stage:(equations were modeled, just computing functions for prediction)
//the following is f(Xo), expected actual values. F matrix is a linearization of the results.
	X[0] = Xo[0] + Xo[2] * cos(deg2rad(Xo[3])) * m2deg * deltaT;	//Lat
	X[1] = Xo[1] + Xo[2] * sin(deg2rad(Xo[3])) * m2deg * deltaT;	//Lon
	X[2] = Xo[2] + Xo[4] * IMUcon * deltaT;	//Vel
//	X[3] = Xo[3] + Xo[5]*IMUcon*deltaT/(Xo[2]+R[2]);//heading
	X[3] = Xo[3] + rad2deg(Xo[5] * IMUcon * deltaT / (Xo[2] + R[2]));
//	X[4] = Xo[4] + (X[2]-Xo[2])/IMUcon;//Yacc in decimilliGs
//	X[5] = Xo[5] + (X[3]-Xo[3])*Xo[2]/IMUcon;//Xacc in decimilliGs
	//0.5*Xo[4/5]+0.5
	X[4] = 0.5 * Xo[4] + 0.5 * (X[2] - Xo[2]) / (IMUcon * deltaT);
	X[5] = 0.5 * Xo[5] + (deg2rad(X[3] - Xo[3]) / deltaT) * Xo[2] / IMUcon;
	X[6] = 10000;		//ideal Zacc in decimilliGs

	//the following is h(Xo)
	float XYG = sqrt(10000 * 10000 - Zo[6] * Zo[6]);//Gravity affecting x/y plane, computed by Z, in decimilliGs
	float YG = Zo[4] - X[4];//gravity affecting Y acc readings in decimilliGs
	float XG = Zo[5] - X[5];//gravity affecting X acc readings in decimilliGs
	float XYG2 = sqrt(XG * XG + YG * YG);//Gravity affecting x/y plane, computed by X/Y in decimilliGs

	// expected readings before sensor noise
	Zo[0] = X[0];		//lat
	Zo[1] = X[1];		//lon
	Zo[2] = X[2];		//vel
	Zo[3] = X[3];		//heading
	if (XYG2 >= 4400) {
		Zo[4] = X[4] + XYG * YG / XYG2; //Yacc
		Zo[5] = X[5] + XYG * XG / XYG2; //Xacc
	} else {
		Zo[4] = X[4];
		Zo[5] = X[5];
	}
	Zo[6] = sqrt(10000 * 10000 - XYG2 * XYG2); //Zacc

	//the following for-loop completes the predict and update stages
	for (i = 0; i <= 7; i++) {
		if (Xo[i] > 1) {
			F[i] = X[i] / Xo[i]; //approximates f(Xo) to scalar a*Xo
			H[i] = Zo[i] / Xo[i]; //approximates h(Xo) to scalar c*Xo
		} else {
			F[i] = 1; //if Xo is zero, just set the scalars to 1
			H[i] = 1;
		}
		P[i] = F[i] * Po[i] * F[i] + R[i] / 2; //error prediction Pk = a*(Pk-1)*a + Q, where R/3 is used as process noise

		//update stage
		Gk[i] = P[i] * H[i] / (H[i] * P[i] * H[i] + R[i]); //update kalman gain gk = Pk*c/(c*Pk*c + r)
		Xo[i] = X[i] + Gk[i] * (*(Z + i) - Zo[i]);
		Po[i] = (1 - Gk[i] * H[i]) * P[i]; //update error for next iteration Pk = (1-gk*c)*Pk
		Zo[i] = *(Z + i); //Z[i];//update Zo for the next iteration
	}
	char bsm[80];
	sprintf(bsm, "___>>>>> g4: %7.1f, g5: %7.6f, g6: %7.6f", Gk[4], Gk[5],
			Gk[6]);

	xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
	UARTprintf("%s\n", bsm);
	xSemaphoreGive(g_xUARTSemaphore);
	return Xo; //push out the predicted values
}

int pdate = 0;
bool fix = false;
//*****************************************************************************
//
//
//
//*****************************************************************************
void GPSparse(char *gpsString) {
	//"$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n"

	if (gpsString[0] != '$')
		return;
	if (nmea_validateChecksum(gpsString, GPS_INPUT_BUF_SIZE)) {

		char tokens[12][25];
		int cnt = sstr_split(tokens, gpsString, ',');

		if (tokens) {
			xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);
			if ((!strcmp(tokens[2], "A")) && (!strcmp(tokens[0], "$GPRMC"))) {
				g_rBSMData.btime = strtod(tokens[1], NULL);
				g_rBSMData.latitude = strtod(tokens[3], NULL);
				if (!strcmp(tokens[4], "S"))
					g_rBSMData.latitude *= -1;
				g_rBSMData.longitude = strtod(tokens[5], NULL);
				if (!strcmp(tokens[6], "W"))
					g_rBSMData.longitude *= -1;
				g_rBSMData.speed = strtod(tokens[7], NULL);
				g_rBSMData.speed *= 0.51444444444;
				g_rBSMData.heading = strtol(tokens[8], NULL, 10);
				g_rBSMData.date = strtol(tokens[9], NULL, 10);
			}



			//todo test bkf
			if ((pdate == 0) && (g_rBSMData.date != 0)) {
				fix = true;
				DKFlininit(g_rBSMData.latitude, g_rBSMData.longitude,
						g_rBSMData.heading);

			} else
				pdate = g_rBSMData.date;

			xSemaphoreGive(g_xBsmDataSemaphore);

			if (fix) {
				float Z[7];
				float * z;
				Z[0] = g_rBSMData.latitude; //initial predicted sensor readings
				Z[1] = g_rBSMData.longitude;
				Z[2] = g_rBSMData.speed;
				Z[3] = g_rBSMData.heading;
				Z[4] = (float) g_rBSMData.latAccel * 116;
				Z[5] = (float) g_rBSMData.longAccel * 116;
				Z[6] = (float) g_rBSMData.vertAccel * 116;

				z = DKFlin(Z);

//				g_rBSMData.latitude = 2836.114000; //initial predicted sensor readings
//				g_rBSMData.longitude = -8111.737000;
				g_rBSMData.speed = *(z+2);
//				g_rBSMData.speed = 0.0;
//				g_rBSMData.heading = *(z+3);
				g_rBSMData.latAccel = *(z+4)/116;
				g_rBSMData.longAccel = *(z+5)/116;
				g_rBSMData.vertAccel = *(z+6)/116;


			}

			//starting
			if (start) {
				if ((g_rBSMData.speed >= .05) && (g_rBSMData.speed <= .1)
						&& (abs(g_rBSMData.latAccel) >= 10)) {
					init_accel += g_rBSMData.latAccel;
				} else if ((g_rBSMData.speed >= .1)
						&& (g_rBSMData.latAccel < 0)) {
					start = false;
					revFlag = true;
				} else if (g_rBSMData.speed >= .1)
					start = false;
			} else {
				// stopping
				if (g_rBSMData.speed < .2) {
					g_rBSMData.heading = oldHeading;
				} else {
					if ((abs(g_rBSMData.heading - oldHeading) > 150)
							&& (abs(g_rBSMData.heading - oldHeading) < 210))
						revFlag = !revFlag;
					oldHeading = g_rBSMData.heading;
				}
			}
			//todo remove
			g_rBSMData.heading = 270;
			g_rBSMData.speed = 0;
		}

	}

}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureGPSUART(uint32_t ui32SysClock) {

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



}

void GPSconfig(uint32_t job){
	char gpsStr[GPS_INPUT_BUF_SIZE];
	// set gps messages

	if(job == 0)
	{
		gpsUARTFlushTx(false);
		gpsUARTprintf("%s\n",
				nmea_generateChecksum(
						"PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", gpsStr));
		//gpsUARTFlushTx(true);
		// set update rate
		gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK220,100", gpsStr));// 100 for 10 hz
	}else{
		gpsUARTxConfig(3, 9600, g_ui32SysClock);
		gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK251,38400",gpsStr));
		gpsUARTxConfig(3, 38400, g_ui32SysClock);
		gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,251,3", gpsStr));

	}
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
	int8_t i=0;

	//
	// Get the current time as a reference to start our delays.
	//
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		if(i<50){
			GPSconfig(0);
			i++;
		}
		else if(i<75){
			GPSconfig(1);
			i++;
				}
		else if(i==75){
			gpsUARTxConfig(3, 38400, g_ui32SysClock);
			//gpsUARTprintf("%s\n", nmea_generateChecksum("PMTK001,251,3", gpsStr));

			i++;
				}

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
	GPSconfig(0);
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
