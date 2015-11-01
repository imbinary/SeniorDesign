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
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/mpu.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "command_task.h"
#include "mpu_task.h"
#include "ravvn.h"
#include "drivers/pinout.h"
#include "drivers/buttons.h"
#include "driverlib/pin_map.h"


extern xSemaphoreHandle g_xBsmDataSemaphore;
//*****************************************************************************
//
// The I2C mutex
//
//*****************************************************************************
extern xSemaphoreHandle g_xI2CSemaphore;



//*****************************************************************************
//
// Define how many iterations between a UART print update. This also
// currently defines how often the MPU data gets re-computed.
//
//*****************************************************************************
#define MPU_PRINT_SKIP_COUNT 50




// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xMPUHandle;



void mupdateBSM( float* pfAcceleration, float* pfAngularVelocity){


    xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);


    g_rBSMData.longAccel = pfAcceleration[1];
    g_rBSMData.latAccel = pfAcceleration[0];
    g_rBSMData.vertAccel = pfAcceleration[2];
    g_rBSMData.yawRate = 7.7; //pfAngularVelocity[1];


    xSemaphoreGive(g_xBsmDataSemaphore);



}


uint8_t mpuReadAccel(uint8_t reg)
{
    uint8_t accelData =  mI2CReceive(0x0c, 0x00);

    return accelData;
}

//*****************************************************************************
//
// This task gathers data from the MPU9150, calculates board orientation in
// Euler angles (roll, pitch and yaw) as well as Quaternions. It then makes
// this data available to the other tasks.
//
//*****************************************************************************
static void
MPUTask(void *pvParameters)
{
    float pfAccel[3];
    uint8_t x1,y1,z1,x2,y2,z2,me;
    uint16_t x,y,z;
	portTickType xLastWakeTime;
	//
	// Get the current time as a reference to start our delays.
	//
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{

		//
		// Wait for the required amount of time to check back.
		//
		vTaskDelayUntil(&xLastWakeTime, MPU_TASK_PERIOD_MS / portTICK_RATE_MS);

        //
        //
        // Take the I2C semaphore.
        //
        //xSemaphoreTake(g_xI2CSemaphore, portMAX_DELAY);

        x1 = mpuReadAccel(MPU9150_ACCEL_XOUT_L);
        y1 = mpuReadAccel(MPU9150_ACCEL_YOUT_L);
        z1 = mpuReadAccel(MPU9150_ACCEL_ZOUT_L);
        x2 = mpuReadAccel(MPU9150_ACCEL_XOUT_H);
        y2 = mpuReadAccel(MPU9150_ACCEL_YOUT_H);
        z2 = mpuReadAccel(MPU9150_ACCEL_ZOUT_H);

        me = mpuReadAccel(0x00);
        x= x2<<8+x1;
        y= y2<<8+y1;
        z= z2<<8+z1;
        //
        // Give back the I2C Semaphore so other can use the I2C interface.
        //
        //xSemaphoreGive(g_xI2CSemaphore);
        UARTprintf("%d, %d, %d, %d\n",x,y,z,me);

        //mupdateBSM(pfAccel,me);
    }
}


//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C7(void)
{
    //enable I2C module 0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);

    //reset module
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C7);

    //enable GPIO peripheral that contains I2C 0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    ROM_GPIOPinConfigure(GPIO_PD0_I2C7SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C7SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    ROM_I2CMasterInitExpClk(I2C7_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C7_BASE + I2C_O_FIFOCTL) = 80008000;
}

//*****************************************************************************
//
// Initializes the MPU task.
//
//*****************************************************************************
uint32_t
MPUTaskInit(void)
{


	InitI2C7();

    //
    // Create the compdcm task itself.
    //
    if(xTaskCreate(MPUTask, (signed portCHAR *)"MPU   ",
                   MPU_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_MPU_TASK, g_xMPUHandle) != pdTRUE)
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
