//*****************************************************************************
//
// senshub_iot.c - Example to publish SensorHub BoosterPack data to the cloud.
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
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "sensorlib/i2cm_drv.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "GPS_task.h"
#include "adxl_task.h"
#include "mpu_task.h"
#include "xbee_task.h"
#include "xbeerx_task.h"
#include "command_task.h"
#include "led_task.h"
#include "ui_task.h"
#include "drivers/pinout.h"
#include "drivers/buttons.h"
#include "ravvn.h"

//*****************************************************************************
//
//  RAVVN started with TI senshub base
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************

xQueueHandle xQueue1;
rBSMData_t g_rBSMData;

//*****************************************************************************
//
// Global variable to hold the system clock speed.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The mutex that protects concurrent access of I2C from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_xI2CSemaphore;

//*****************************************************************************
//
// The mutex that protects concurrent access of Cloud Data from multiple
// tasks.
//
//*****************************************************************************
xSemaphoreHandle g_xBsmDataSemaphore;


//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Counter value used by the FreeRTOS run time stats feature.
// http://www.freertos.org/rtos-run-time-stats.html
//
//*****************************************************************************
volatile unsigned long g_vulRunTimeStatsCountValue;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// Interrupt handler for Timer0A.
//
// This function will be called periodically on the expiration of Timer0A It
// performs periodic tasks, such as looking for input on the physical buttons,
// and reporting usage statistics to the cloud.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Keep track of the number of times this interrupt handler has been
    // called.
    //
    g_vulRunTimeStatsCountValue++;

}

//*****************************************************************************
//
// Configure and start the timer that will increment the variable used to
// track FreeRTOS task statistics.
//
//*****************************************************************************
void SensorCloudStatTimerConfig(void)
{
    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Configure the two 32-bit periodic timers.  The period of the timer for
    // FreeRTOS run time stats must be at least 10 times faster than the tick
    // rate.
    //
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock /
                                           (configTICK_RATE_HZ * 10));

    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);

}



//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{

    //
    // Configure the system frequency.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);





    //
    // Configure the device pins for this board.
    // This application uses Ethernet but not USB.
    //
    PinoutSet(false, false);
    ButtonsInit();

    // Create a mutex to guard the I2C.
    //
    g_xI2CSemaphore = xSemaphoreCreateMutex();

    //
    // Create a mutex to guard the Cloud Data structure.
    //
    g_xBsmDataSemaphore = xSemaphoreCreateMutex();


    xQueue1 = xQueueCreate( UIQSIZE, sizeof( uint16_t  ) );
    //
    // Create the virtual com port task.
    // Doing this task first initializes the UART.
    //
    if(CommandTaskInit() != 0)
    {
        //
        // Init returned an error. Print an alert to the user and
        // spin forever.  Wait for reset or user to debug.
        //
        UARTprintf("Virtual COM Port: Task Init Failed!");
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }


    //
    // Create the xbee task.
    //
    if(XBEETaskInit() != 0)
    {
        //
        // Init returned an error. Print an alert to the user and
        // spin forever.  Wait for reset or user to debug.
        //
        UARTprintf("XBEE: Task Init Failed!\n");
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }

    //
    // Create the xbee task.
    //
    if(XBEErxTaskInit() != 0)
    {
        //
        // Init returned an error. Print an alert to the user and
        // spin forever.  Wait for reset or user to debug.
        //
        UARTprintf("XBEERX: Task Init Failed!\n");
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }


    //
    // Create the xbee task.
    //
    if(LedTaskInit() != 0)
    {
        //
        // Init returned an error. Print an alert to the user and
        // spin forever.  Wait for reset or user to debug.
        //
        UARTprintf("LED: Task Init Failed!\n");
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }
    //
    // Create the ui task.
    //
    if(UITaskInit() != 0)
    {
        //
        // Init returned an error. Print an alert to the user and
        // spin forever.  Wait for reset or user to debug.
        //
        UARTprintf("UI: Task Init Failed!\n");
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }





    //
     // Create the CompDCM 9 axis sensor task.
//todo re enable

     if(ADXL && ADXLTaskInit() != 0)
     {
         //
         // Init returned an error. Print an alert to the user and
         // spin forever.  Wait for reset or user to debug.
         //
         UARTprintf("ADXL: Task Init Failed!\n");
         while(1)
         {
             //
             // Do Nothing.
             //
         }
     }


    //
    // Verify that the semaphores were created correctly.
    //
    if((g_xI2CSemaphore == NULL) || (g_xBsmDataSemaphore == NULL))
    {
        //
        // I2C or CloudData semaphore was not created successfully.
        // Print an error message and wait for user to debug or reset.
        //
        UARTprintf("I2C or Cloud Data semaphore create failed.\n");
        UARTprintf("I2C Semaphore: 0x%X\t\tCloudData Semaphore: 0x%X",
                   (uint32_t) g_xI2CSemaphore,
                   (uint32_t) g_xBsmDataSemaphore);
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }
//    char bsm[60];
//    float coll = tCollideAcc(18.5,360- 6 , 0.00, 0,
//			0,0,9.89,
//			0, 0,
//			180);

//	sprintf(bsm, "%f",coll);
//	UARTprintf("tcoll: %s\n",bsm);


    //
    // Create the GPS  task.
    //
    if(GPSTaskInit() != 0)
    {
        //
        // Init returned an error. Print an alert to the user and
        // spin forever.  Wait for reset or user to debug.
        //
        UARTprintf("GPS: Task Init Failed!\n");
        while(1)
        {
            //
            // Do Nothing.
            //
        }
    }
    //
    // Config and start the timer that is used by FreeRTOS to determine
    // run time stats.
    //
    SensorCloudStatTimerConfig();

    //
    // Clear the terminal and print demo introduction. This is safe here
    // since we have not yet started the scheduler.  UART and UARTStdio
    // config happens in the VCP Task. Once scheduler starts tasks must take
    // the UART semaphore to safely print.
    //
    UARTprintf("RAVVN v1.2\n");

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
    UARTprintf("RTOS scheduler returned unexpectedly.\n");
    while(1)
    {
        //
        // Do Nothing.
        //
    }
}
