//*****************************************************************************
//
// compdcm_task.c - Manage the 9-Axis sensor and Complimentary Filtered DCM.
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "utils/hw_adxl312.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/adxl312.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "command_task.h"
#include "adxl_task.h"
#include "ravvn.h"
#include "drivers/pinout.h"
#include "drivers/buttons.h"
#include "driverlib/pin_map.h"


extern xSemaphoreHandle g_xBsmDataSemaphore;
extern xQueueHandle xQueue1;
//*****************************************************************************
//
// The I2C mutex
//
//*****************************************************************************
extern xSemaphoreHandle g_xI2CSemaphore;

extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Define how many iterations between a UART print update. This also
// currently defines how often the ADXL data gets re-computed.
//
//*****************************************************************************
#define ADXL_PRINT_SKIP_COUNT 50




// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xADXLHandle;



void updateBSM( int16_t x, int16_t y, int16_t z){


    xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);


    g_rBSMData.longAccel = x;
    g_rBSMData.latAccel = y;
    g_rBSMData.vertAccel = z;
    g_rBSMData.yawRate = 5.5; //pfAngularVelocity[1];


    xSemaphoreGive(g_xBsmDataSemaphore);



}


#define THRESHOLD 15
int16_t xo,yo,zo;


static void
ADXLTask(void *pvParameters)
{
	portTickType xLastWakeTime;


	I2CSend(ADXL312_I2CADR_ALT, 2, ADXL_POWER_CTL, 0x08 );
	//I2CSend(ADXL312_I2CADR_ALT, 2, ADXL_FIFO_CTL, 0x64 );
	//
	// Get the current time as a reference to start our delays.
	//
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{

		//
		// Wait for the required amount of time to check back.
		//
		vTaskDelayUntil(&xLastWakeTime, ADXL_TASK_PERIOD_MS / portTICK_RATE_MS);

        //
        //
        // Take the I2C semaphore.
        //
        xSemaphoreTake(g_xI2CSemaphore, portMAX_DELAY);

        uint16_t x1=0,x2=0,x3=0;

		x1 = I2CReceiveMulti(ADXL312_I2CADR_ALT, ADXL_DATAX0,2);
		x2 = I2CReceiveMulti(ADXL312_I2CADR_ALT, ADXL_DATAY0,2);
		x3 = I2CReceiveMulti(ADXL312_I2CADR_ALT, ADXL_DATAZ0,2);

		int16_t x,y,z;
		x = (int16_t)(x1);
		y = (int16_t)(x2);
		z = (int16_t)(x3);


		if(abs(xo-x) > THRESHOLD|| abs(yo-y) > THRESHOLD ||abs(zo-z) > THRESHOLD)
		{
			z=z*2.9/100;
			y=y*2.9/100;
			x=x*2.9/100;

			xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
			//UARTprintf("adxl: X(%d), Y(%d), Z(%x)\n",x,y,z);
			xSemaphoreGive(g_xUARTSemaphore);
		}


		xo=x;
		yo=y;
		zo=z;

        // Give back the I2C Semaphore so other can use the I2C interface.
        //
        xSemaphoreGive(g_xI2CSemaphore);


        updateBSM(x,y,z);



    }
}


//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    I2CSlaveEnable(I2C0_BASE);
    //I2CSlaveInit(I2C0_BASE, ADXL312_I2CADR_ALT);
    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

//*****************************************************************************
//
// Initializes the ADXL task.
//
//*****************************************************************************
uint32_t
ADXLTaskInit(void)
{


	InitI2C0();

    //
    // Create the compdcm task itself.
    //
    if(xTaskCreate(ADXLTask, (signed portCHAR *)"ADXL   ",
                   ADXL_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ADXL_TASK, g_xADXLHandle) != pdTRUE)
    {
        //
        // Task creation failed.
        //
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
