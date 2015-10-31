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
//*****************************************************************************
//
// The I2C mutex
//
//*****************************************************************************
extern xSemaphoreHandle g_xI2CSemaphore;


//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define ADXL_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Define how many iterations between a UART print update. This also
// currently defines how often the ADXL data gets re-computed.
//
//*****************************************************************************
#define ADXL_PRINT_SKIP_COUNT 50


//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tADXL312 g_sADXLInst;

//*****************************************************************************
//
// Binary semaphore to control task flow and wait for the I2C transaction to
// complete. Sensor data has been read and is now ready for processing.
//
//*****************************************************************************
xSemaphoreHandle g_xADXLTransactionCompleteSemaphore;

//*****************************************************************************
//
// Binary semaphore to control task flow and wait for the GPIO interrupt which
// indicates that the sensor has data that needs to be read.
//
//*****************************************************************************
xSemaphoreHandle g_xADXLDataReadySemaphore;

//*****************************************************************************
//
// Global new error flag to store the error condition if encountered.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ADXLI2CErrorStatus;

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xADXLHandle;

//*****************************************************************************
//
// Global instance structure for the BMP180 sensor data to be published.
//
//*****************************************************************************
sADXLData_t g_sADXLData;


void updateBSM( float* pfAcceleration, float* pfAngularVelocity){


    xSemaphoreTake(g_xBsmDataSemaphore, portMAX_DELAY);


    g_rBSMData.longAccel = pfAcceleration[1];
    g_rBSMData.latAccel = pfAcceleration[0];
    g_rBSMData.vertAccel = pfAcceleration[2];
    g_rBSMData.yawRate = pfAngularVelocity[1];


    xSemaphoreGive(g_xBsmDataSemaphore);



}


//*****************************************************************************
//
// Function converts a floating point value to a string.
//
// \param fValue is the value to be converted.
// \param pcStr is a pointer to a character buffer where the result will be
// stored.
// \param ui32Size is the size of the buffer where the result is stored.
// \param ui32Precision is the number of digits after the decimal point.
// Result will be truncated after this many digits.
//
// This function performs a brute force conversion of a float to a string.
// First checks for a negative value and adds the '-' char if needed. Then
// casts the float to an integer and uses existing usnprintf from
// utils/ustdlib.c to convert the integer part to the string. Then loops
// through the decimal portion multiplying by 10 and using the same integer
// conversion for each position of precision. Returns when ui32Size is reached
// or conversion is complete.
//
// \return the number of characters written to the buffer.
//
//*****************************************************************************
uint32_t
luftostr(char * pcStr, uint32_t ui32Size, uint32_t ui32Precision, float fValue)
{
    uint32_t ui32Integer;
    uint32_t ui32SpaceUsed;
    uint32_t ui32PrecisionCounter;

    //
    // Initialize local variable.
    //
    ui32SpaceUsed = 0;

    //
    // decrement size to account for room for a null character.
    //
    ui32Size -= 1;

    //
    // Account for negative values.
    //
    if(fValue < 0.0f)
    {
        if(ui32Size > 1)
        {
            pcStr[0] = '-';
            ui32SpaceUsed = 1;
        }
    }

    //
    // Initialize the loop conditions.
    //
    ui32PrecisionCounter = 0;
    ui32Integer = 0;

    //
    // Perform the conversion.
    //
    while((ui32PrecisionCounter <= ui32Precision) &&
          (ui32SpaceUsed < ui32Size))
    {
        //
        // Convert the new integer part.
        //
        ui32Integer = (uint32_t) fValue;

        //
        // Use usnprintf to convert the integer part to a string.
        //
        ui32SpaceUsed += usnprintf(&(pcStr[ui32SpaceUsed]),
                                   ui32Size - ui32SpaceUsed,
                                   "%d", ui32Integer);
        //
        // Subtract off the previous integer part.  Subtracts zero on first
        // time through loop.
        //
        fValue = fValue - ((float) ui32Integer);

        //
        // Multiply by 10 so next time through the most significant remaining
        // decimal will be beceome the integer part.
        //
        fValue *= 10;

        //
        // iF this is first time through the loop then add the decimal point
        // after the integer in the string. Also makes sure that there is room
        // for the decimal point.
        //
        if((ui32PrecisionCounter == 0) && (ui32SpaceUsed < ui32Size))
        {
             pcStr[ui32SpaceUsed] = '.';
             ui32SpaceUsed++;
        }

        //
        // Increment the precision counter to so we only convert as much as the
        // caller asked for.
        //
        ui32PrecisionCounter++;
    }

    //
    // Check if we quit because we ran out of buffer space.
    //
    if(ui32SpaceUsed >= ui32Size)
    {
        //
        // Since we decremented size at the beginning we should still have room
        // for the null char.
        //
        pcStr[ui32Size] = '\0';

        //
        // Return amount of space used plus the number of precision digits that
        // were not accommodated.
        //
        return (ui32SpaceUsed + (ui32Precision - ui32PrecisionCounter));
    }

    //
    // Terminate the string with null character.
    //
    pcStr[ui32SpaceUsed] = '\0';


    //
    // Return the amount of buffer space used. Not including null character.
    //
    return (ui32SpaceUsed);

}

//*****************************************************************************
void ADXLDataPrint(float *pfRPY, float *pfQuaternion)
{
    uint32_t ui32Idx;
    float pfEulerDegrees[3];
    char pcEulerBuf[3][12];
    char pcQuaternionBuf[4][12];

    //
    // Convert Eulers to degrees. 180/PI = 57.29...
    // Convert Yaw to 0 to 360 to approximate compass headings.
    //
    pfEulerDegrees[0] = pfRPY[0] * 57.295779513082320876798154814105f;
    pfEulerDegrees[1] = pfRPY[1] * 57.295779513082320876798154814105f;
    pfEulerDegrees[2] = pfRPY[2] * 57.295779513082320876798154814105f;
    if(pfEulerDegrees[2] < 0)
    {
        pfEulerDegrees[2] += 360.0f;
    }

    //
    // Convert floats in the structure to strings.
    //
    for(ui32Idx = 0; ui32Idx < 3; ui32Idx++)
    {
        luftostr(pcEulerBuf[ui32Idx], 12, 3, pfEulerDegrees[ui32Idx]);
        luftostr(pcQuaternionBuf[ui32Idx], 12, 3, pfQuaternion[ui32Idx]);
    }

    //
    // Convert the last quaternion from float to string. Special handling
    // since we have four quaternions and three of everything else.
    //
    luftostr(pcQuaternionBuf[ui32Idx], 12, 3, pfQuaternion[ui32Idx]);

    //
    // Attempt to grab the UART semaphore so we can send the error info to the
    // user locally.
    //
    xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);

    //
    // Print the Quaternions data.
    //
    UARTprintf("\nADXL:\tQ1: %s\tQ2: %s\tQ3: %s\tQ4: %s\n",
               pcQuaternionBuf[0], pcQuaternionBuf[1], pcQuaternionBuf[2],
               pcQuaternionBuf[3]);

    //
    // Print the Quaternions data.
    //
    UARTprintf("ADXL:\tRoll: %s\tPitch: %s\tYaw: %s\n", pcEulerBuf[0],
               pcEulerBuf[1], pcEulerBuf[2]);

    //
    // Give back the UART semaphore.
    //
    xSemaphoreGive(g_xUARTSemaphore);

    }



//*****************************************************************************
//
// This task gathers data from the MPU9150, calculates board orientation in
// Euler angles (roll, pitch and yaw) as well as Quaternions. It then makes
// this data available to the other tasks.
//
//*****************************************************************************
static void
ADXLTask(void *pvParameters)
{
    float pfAccel[3];
    uint32_t ui32ADXLStarted;
//    uint32_t ui32Idx;

    //
    //
    // Take the I2C semaphore.
    //
    xSemaphoreTake(g_xI2CSemaphore, portMAX_DELAY);



    //
    // Give back the I2C Semaphore so other can use the I2C interface.
    //
    xSemaphoreGive(g_xI2CSemaphore);



    //
    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    //
   // ADXLInit(&g_sADXLInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);

    //
    // Clear the flag showing if we have "started" the DCM.  Starting
    // requires an initial valid data set.
    //
    ui32ADXLStarted = 0;

    while(1)
    {




        //
        // Get floating point version of the Accel Data in m/s^2.
        //

        //todo put data in our struct
        //
        // Check if this is our first data ever.
        //
        if(ui32ADXLStarted == 0)
        {
            //
            // Set flag indicating that DCM is started.
            // Perform the seeding of the DCM with the first data set.
            //
            ui32ADXLStarted = 1;

        }
        else
        {
            //
            // DCM Is already started.  Perform the incremental update.
            //


            //
            // Get Quaternions.
            //



        }

        //updateR(g_sADXLInst.pfAccel,g_sADXLInst.pfGyro);
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
    // Set the active flag that shows this task is running properly.
    //
    g_sADXLData.bActive = true;
    g_sADXLData.xTimeStampTicks = 0;

    //
    // point the global data structure to my local task data structuer.
    //

    //
    // Success.
    //
    return(0);
}
