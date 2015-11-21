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


void calcAlert(rBSMData_t tmpBSMData);
uint8_t calcDir(rBSMData_t tmpBSMData);
uint8_t calcColor(rBSMData_t tmpBSMData, int size, int dist);

// globals
extern bool revFlag;



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
		//char** tokens;
		char tokens[10][25];
		//tokens = str_split(cInput, ',');
		int cnt = sstr_split(tokens, cInput, ',');
		//char** tokens;
		//tokens = str_split(cInput, ',');
		if (tokens) {

			if (!strcmp(tokens[0], "$B")) {
				tmpBSMData.latitude = strtod(tokens[1], NULL);
				tmpBSMData.longitude = strtod(tokens[2], NULL);
				tmpBSMData.speed = strtod(tokens[3], NULL);
				tmpBSMData.heading = strtol(tokens[4], NULL, 10);
				tmpBSMData.btime = strtod(tokens[5], NULL);
				tmpBSMData.date = strtol(tokens[6], NULL, 10);
				tmpBSMData.latAccel = strtol(tokens[7], NULL, 10);
				tmpBSMData.longAccel = strtol(tokens[8], NULL, 10);
				tmpBSMData.vertAccel = strtol(tokens[9], NULL, 10);

				sprintf(bsm, "mS %3.2f, mH %d, mT %7.1f, oS %3.2f, oH %d, oT %7.1f, dist  %5.1f, d2o %d, pix %d, cnt %d",
						g_rBSMData.speed, g_rBSMData.heading, g_rBSMData.btime,
						tmpBSMData.speed, tmpBSMData.heading, tmpBSMData.btime,
						distance(deg2dec(g_rBSMData.latitude),
								deg2dec(g_rBSMData.longitude),
								deg2dec(tmpBSMData.latitude),
								deg2dec(tmpBSMData.longitude), 'm'),
						direction(deg2dec(g_rBSMData.latitude),
								deg2dec(g_rBSMData.longitude),
								deg2dec(tmpBSMData.latitude),
								deg2dec(tmpBSMData.longitude), 'K'),
								calcDir(tmpBSMData), cnt);

				xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
				UARTprintf("\n%s\n\n", bsm);
				xSemaphoreGive(g_xUARTSemaphore);

				calcAlert(tmpBSMData);

			} else if (!strcmp(tokens[0], "$I")) {
				tmpBSMData.latitude = strtod(tokens[1], NULL);
				tmpBSMData.longitude = strtod(tokens[2], NULL);
				tmpBSMData.heading = strtol(tokens[3], NULL, 10);
				tmpBSMData.btime = strtod(tokens[5], NULL);
				uint8_t color = strtol(tokens[6], NULL, 10);

				sprintf(bsm, "time: %07.1f, color: %5d, dist: %05.1f, dir: %d",
						tmpBSMData.btime, color,
						distance(deg2dec(g_rBSMData.latitude),
								deg2dec(g_rBSMData.longitude),
								deg2dec(tmpBSMData.latitude),
								deg2dec(tmpBSMData.longitude), 'm'),
						direction(deg2dec(g_rBSMData.latitude),
								deg2dec(g_rBSMData.longitude),
								deg2dec(tmpBSMData.latitude),
								deg2dec(tmpBSMData.longitude), 'K'));

				xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
				//UARTprintf("%s\n", bsm);
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

//*****************************************************************************
//
//
//
//*****************************************************************************
float tCollideAcc(int dist, int bear, float myV, int myA_y, int myA_x,
		int myHead, float oV, int oA_y, int oA_x, int oHead) {

	oHead = deg2rad(oHead);
	myHead = deg2rad(myHead);
	bear = deg2rad(bear);

	float D_y = dist * cos(bear); //y component of distance in meters
	float D_x = dist * sin(bear); //x component of distance in meters
	float V_ry = oV * cos(oHead) - myV * cos(myHead); //y component of relative velocity
	float V_rx = oV * sin(oHead) - myV * sin(myHead); //x component of relative velocity
	float A_ry = (oA_y * cos(oHead) - oA_x * sin(oHead) - myA_y * cos(myHead)
			+ myA_x * sin(myHead)) * 9.88 / 10000; //convert to m/s^2 from g*10^-4
	float A_rx = (oA_y * sin(oHead) + oA_x * sin(oHead) - myA_y * sin(myHead)
			- myA_x * cos(myHead)) * 9.88 / 10000; //convert to m/s^2 from g*10^-4
	float A_r2 = A_rx * A_rx + A_ry * A_ry;

	//if there is no relative acceleration, check for collision by velocity
	if (A_r2 < 25){
		xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
		UARTprintf("VELOCITY\n");
		xSemaphoreGive(g_xUARTSemaphore);
		return tCollide(dist, bear, myV, myHead, oV, oHead);
	}else{
		xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
		UARTprintf("ACCELERATION\n");
		xSemaphoreGive(g_xUARTSemaphore);
	}
	//expression 1 of solution
	float ex1 = dist * dist * A_r2 - 4 * dist * A_r2
			- A_rx * A_rx * (V_ry * V_ry - 4) + 2 * A_rx * A_ry * V_rx * V_ry
			- A_ry * A_ry * (V_rx * V_rx - 4);

	if (ex1 < 0)
		return -1; //solution is not real-paths are parallel

	ex1 = sqrt(ex1); //previous statement avoids taking square root of negative
	float ex2 = A_rx * V_rx + A_ry * V_ry; //expression 2 of solution
	float t1 = (ex1 - ex2) / A_r2; //solution 1
	float t2 = (0 - ex1 - ex2) / A_r2; //solution 2
	float D_t1, D_t2;
	//t1 *= -1;
	//t2 *= -1;

	if (t1 >= 0 && t1 <= 12) { //assumes we don't care to predict collisions more than 12 seconds out
		D_t1 = pow(D_x + V_rx * t1 + A_rx * t1 * t1 / 2, 2)
				+ pow(D_y + V_ry * t1 + A_ry * t1 * t1 / 2, 2);
		D_t1 = sqrt(D_t1); //distance at time 1
	} else
		D_t1 = -1; //not valid solution

	if (t2 >= 0 && t2 <= 12) { //assumes we don't care to predict collisions more than 12 seconds out
		D_t2 = pow(D_x + V_rx * t2 + A_rx * t2 * t2 / 2, 2)
				+ pow(D_y + V_ry * t2 + A_ry * t2 * t2 / 2, 2);
		D_t2 = sqrt(D_t2); //distance at time 2
	} else
		D_t2 = -1; //not valid solution

	if( (D_t1 >= dist && D_t2 >= dist) || (D_t1 < 0 && D_t2 < 0) ) return -1; //vehicles are moving away from eachother
	else if( D_t1 >= 0 && D_t1 < dist && (D_t2 >=dist || D_t2 < 0) ) return t1; //t1 is time to collision
	else if( (D_t1 >= dist || D_t1 < 0 ) && D_t2 >= 0 && D_t2 < dist) return t2; //t2 is time to collision
	else return min(t1, t2); //the minimum of t1 and t2 is time to collision
}

//*****************************************************************************
//
//
//
//*****************************************************************************
float tCollide(int dist, int bear, float myVeloc, int myHead, float oVeloc,
		int oHead) {

	float d_y = dist * cos(bear); //y component of distance in meters
	float d_x = dist * sin(bear); //x component of distance in meters
	float V_ry = 0-oVeloc * cos(oHead) + myVeloc * cos(myHead); //y component of relative velocity
	float V_rx = 0-oVeloc * sin(oHead) + myVeloc * sin(myHead); //x component of relative velocity
	float V_r2 = V_ry * V_ry + V_rx * V_rx; //relative velocity

	if (V_r2 < 0.001)
		return -1; //travelling parallel at same velocity, same direction. No collision

	float ex1 = 0 - (d_x + d_x) * (V_ry * V_ry) + 2 * d_x * d_y * V_rx * V_ry
			- (d_y * d_y) * (V_rx * V_rx) + 4 * (V_r2); //expression 1 of solution

	if (ex1 < 0)
		return -1; //solution is not real-paths are parallel

	ex1 = sqrt(ex1); //previous statement avoides taking square root of negative
	float ex2 = d_x * V_rx - d_y * V_ry; //expression 2 of solution
	//todo we changed this
	float t1 = (ex1 - ex2) / V_r2; //solution 1
	float t2 = (0 - ex1 - ex2) / V_r2; //solution 2

	if((t1<0 || t1>12) && (t2 < 0 || t2 > 12) ) return -1; //vehicles will not collide within 12 seconds
	else if(t2<0 || t2 >12) return t1; //t1 is TTC (time to collision)
	else if(t1<0 || t1 >12) return t2; //t2 is TTC
	else return min(t1,t2); //the smaller time is the TTC
}

//*****************************************************************************
//
//
//
//*****************************************************************************
float min(float v1, float v2) {
	if (v1 < v2)
		return v1;
	return v2;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
void calcAlert(rBSMData_t tmpBSMData) {

	// send alert to queue
	if (xQueue1 != 0) {
		uint8_t byte1, byte2;
		int size, dist;
		//construct the bytes

		//calc distance
		dist = (int) (distance(deg2dec(g_rBSMData.latitude),
				deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitude),
				deg2dec(tmpBSMData.longitude), 'm'));
		if (dist > 60)
			return;

		byte1 = calcDir(tmpBSMData);

		//size relative to distance
		if (dist > 20)
			size = 0;
		else if (dist < 3)
			size = 7;
		else {
			float tmp3 = ((dist * -7) + 140) / 17;
			size = tmp3;
		}

		// set dir and size
		byte1 = (byte1 * 8) + size;
		// set color
		byte2 = calcColor(tmpBSMData, size, dist);
		//set night bit
		if (tmpBSMData.btime > 20000 && tmpBSMData.btime < 140000)
			byte2 |= 0x80;

		uint16_t tmp = (byte1 << 8) | byte2;
		xQueueSendToBackFromISR(xQueue1, &tmp, 0);

	}

}

//*****************************************************************************
//
//
//
//*****************************************************************************
uint8_t calcColor(rBSMData_t tmpBSMData, int size, int dist) {
	uint8_t color = 0x0f;
	int16_t dir;

	dir = direction(deg2dec(g_rBSMData.latitude),
			deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitude),
			deg2dec(tmpBSMData.longitude), 'K');

	float coll = tCollideAcc(dist, 360- dir, g_rBSMData.speed, g_rBSMData.latAccel * 116,
			g_rBSMData.longAccel * 116, g_rBSMData.heading, tmpBSMData.speed,
			tmpBSMData.latAccel * 116, tmpBSMData.longAccel * 116,
			tmpBSMData.heading);

	if (size <= 7) {
		// far away use intersection with constant speed

		if (coll < 0 || coll > 12)
			color = 1;
		else{
			color = ((coll * -10.5) + 127);
			xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
			UARTprintf("far size: %d color: %d COLLISION!\n", size, color);
			xSemaphoreGive(g_xUARTSemaphore);
		}

	}


	return color;
}

uint8_t calcDir(rBSMData_t tmpBSMData) {
	int16_t dir;

	dir = direction(deg2dec(g_rBSMData.latitude),
			deg2dec(g_rBSMData.longitude), deg2dec(tmpBSMData.latitude),
			deg2dec(tmpBSMData.longitude), 'K');
	dir = g_rBSMData.heading - dir;

	if (dir <= 0)
		dir += 360;
	dir = dir * 4 / 45;
/*
	if (revFlag) {
		dir = 16 - dir;
		if (dir < 0)
			dir += 32;
	}
	*/
	return (uint8_t) dir;
}


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
			vTaskDelayUntil(&xLastWakeTime, XBEERX_TASK_PERIOD_MS /
			portTICK_RATE_MS);
			//get up to 5 messages
			for (i = 0; i < 2; i++) {
				i32DollarPosition = xbeeUARTPeek('*');

				if (i32DollarPosition != (-1)) {

					xSemaphoreTake(g_xbeeUARTSemaphore, portMAX_DELAY);
					int t = xbeeUARTgetr(cInput, XBEE_INPUT_BUF_SIZE);
					xSemaphoreGive(g_xbeeUARTSemaphore);

					bsmParse(cInput);
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

return 0;
	}
