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
#include "sensorlib/i2cm_drv.h"
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


extern xSemaphoreHandle g_xBsmDataSemaphore;
//*****************************************************************************
//
// The I2C mutex
//
//*****************************************************************************
extern xSemaphoreHandle g_xI2CSemaphore;

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
extern tI2CMInstance g_sI2CInst;

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
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
//tMPU9150 g_sMPU9150Inst;

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
//
// ADXL Sensor callback function.  Called at the end of sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// give the Transaction complete semaphore so that task is triggered to do the
// bulk of data handling.
//
//*****************************************************************************
void
ADXLAppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    portBASE_TYPE xHigherPriorityTaskWokenTransaction;

    xHigherPriorityTaskWokenTransaction = pdFALSE;

    //
    // Let the task resume so that it can check the status flag and process
    // data.
    //
    xSemaphoreGiveFromISR(g_xADXLTransactionCompleteSemaphore,
                          &xHigherPriorityTaskWokenTransaction);

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ADXLI2CErrorStatus = ui8Status;

    //
    // If a higher priority task was waiting for a semaphore released by this
    // isr then that high priority task will run when the ISR exits.
    //
    if(xHigherPriorityTaskWokenTransaction == pdTRUE)
    {
        portYIELD_FROM_ISR(true);
    }
}

//*****************************************************************************
//
// ADXL Application error handler.
//
//*****************************************************************************
void
ADXLAppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Take the UART semaphore to guarantee the sequence of messages on VCP.
    //
    xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);

    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1mADXL I2C Error: %d, File: %s, Line: %d\n",
               g_vui8ADXLI2CErrorStatus, pcFilename, ui32Line);

    //
    // Tell users where to find more information about I2C errors and
    // reset the terminal color to normal.
    //
    UARTprintf("See I2C status definitions in utils\\i2cm_drv.h\n\033[0m");

    //
    // Give back the UART Semaphore.
    //
    xSemaphoreGive(g_xUARTSemaphore);

    //
    // Change the active flag in the cloud data struct to show this sensor
    // is no longer being actively updated.
    //

    //
    // Since we got an I2C error we will suspend this task and let other
    // tasks continue.  We will never again execute unless some other task
    // calls vTaskResume on us.
    //
    vTaskSuspend(NULL);
}


//*****************************************************************************
//
// ADXL Data print function.  Takes the float versions of ambient and object
// temperature and prints them to the UART in a pretty format.
//
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
    float pfMag[3], pfAccel[3], pfGyro[3];
    float pfQuaternion[4], pfEuler[3];
    uint32_t ui32ADXLStarted;
//    uint32_t ui32Idx;

    //
    // The binary semaphore is created full so we take it up front and use it
    // later to sync between this task and the AppCallback function which is in
    // the I2C interrupt context. Likewise the GPIO port interrupt.
    //
    xSemaphoreTake(g_xADXLTransactionCompleteSemaphore, 0);
    xSemaphoreTake(g_xADXLDataReadySemaphore, 0);

    //
    // Take the I2C semaphore. Keep it until all init is complete for this
    // sensor.
    //
    xSemaphoreTake(g_xI2CSemaphore, portMAX_DELAY);

    //
    // Initialize the MPU9150 Driver.
    //
    //MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
      //          ADXLAppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    xSemaphoreTake(g_xADXLTransactionCompleteSemaphore, portMAX_DELAY);

    //
    // Give back the I2C Semaphore so other can use the I2C interface.
    //
    xSemaphoreGive(g_xI2CSemaphore);

    //
    // Check for I2C Errors and responds
    //
    if(g_vui8ADXLI2CErrorStatus)
    {
        ADXLAppErrorHandler(__FILE__, __LINE__);
    }

    //
    // Take the I2C semaphore.
    //
    xSemaphoreTake(g_xI2CSemaphore, portMAX_DELAY);

    //
    // Write application specific sensor configuration such as filter settings
    // and sensor range settings.
    //

    //
    // Wait for transaction to complete
    //
    xSemaphoreTake(g_xADXLTransactionCompleteSemaphore, portMAX_DELAY);

    //
    // Check for I2C Errors and responds
    //
    if(g_vui8ADXLI2CErrorStatus)
    {
        //
        // Give back the I2C Semaphore so other can use the I2C interface.
        //
        xSemaphoreGive(g_xI2CSemaphore);

        //
        // Call the error handler.
        //
        ADXLAppErrorHandler(__FILE__, __LINE__);
    }

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //


    //
    // Wait for transaction to complete
    //
    xSemaphoreTake(g_xADXLTransactionCompleteSemaphore, portMAX_DELAY);

    //
    // Give back the I2C Semaphore so other can use the I2C interface.
    //
    xSemaphoreGive(g_xI2CSemaphore);

    //
    // Check for I2C Errors and responds
    //
    if(g_vui8ADXLI2CErrorStatus)
    {
        ADXLAppErrorHandler(__FILE__, __LINE__);
    }

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
        // wait for the GPIO interrupt that tells us MPU9150 has data.
        //
        xSemaphoreTake(g_xADXLDataReadySemaphore, portMAX_DELAY);

        //
        // Take the I2C semaphore.
        //
        xSemaphoreTake(g_xI2CSemaphore, portMAX_DELAY);

        //
        // Use I2C to go get data from the sensor into local memory.
        //
        //MPU9150DataRead(&g_sMPU9150Inst, ADXLAppCallback, &g_sMPU9150Inst);

        //
        // Wait for transaction to complete
        //
        xSemaphoreTake(g_xADXLTransactionCompleteSemaphore, portMAX_DELAY);

        //
        // Give back the I2C Semaphore so other can use the I2C interface.
        //
        xSemaphoreGive(g_xI2CSemaphore);

        //
        // Check for I2C Errors and responds
        //
        if(g_vui8ADXLI2CErrorStatus)
        {
            ADXLAppErrorHandler(__FILE__, __LINE__);
        }

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

//*****************************************************************************
//
// Initializes the ADXL task.
//
//*****************************************************************************
uint32_t
ADXLTaskInit(void)
{
    //
    // Reset the GPIO port M to make sure that all previous configuration is
    // cleared.
    //
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOM);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
    {
        //
        // Do nothing, waiting.
        //
    }

    //
    // Configure and Enable the GPIO interrupt. Used for DRDY from the MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_3);
    ROM_GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);

    //
    // Must adjust the priority of the interrupt so that it can call FreeRTOS
    // APIs.
    //
    IntPrioritySet(INT_GPIOM, 0xE0);
    ROM_IntEnable(INT_GPIOM);

    //
    // Create binary semaphores for flow control of the task synchronizing with
    // the I2C data read complete ISR (callback) and the GPIO pin ISR that
    // alerts software that the sensor has data ready to be read.
    //
    vSemaphoreCreateBinary(g_xADXLDataReadySemaphore);
    vSemaphoreCreateBinary(g_xADXLTransactionCompleteSemaphore);

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
    // Check if Semaphore creation was successful.
    //
    if((g_xADXLDataReadySemaphore == NULL) ||
       (g_xADXLTransactionCompleteSemaphore == NULL))
    {
        //
        // At least one semaphore was not created successfully.
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
