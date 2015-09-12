//*****************************************************************************
//
// command_task.c - Virtual COM Port Task manage messages to and from terminal.
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
#include "uiuart.h"
#include "compdcm_task.h"

#include "command_task.h"


//*****************************************************************************
//
// Buffer size for the UART buffer.
//
//*****************************************************************************
#define COMMAND_INPUT_BUF_SIZE  80

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
xTaskHandle g_xCommandHandle;

//*****************************************************************************
//
// A mutex semaphore to manage the UART buffer in utils\uartstdio.  Before
// calling UARTprintf each task must take this semaphore.
//
//*****************************************************************************
xSemaphoreHandle g_xUARTSemaphore;

//*****************************************************************************
//
// Global variable to hold the system clock speed.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// This is the table that holds the command names, implementing functions, and
// brief description.
//
//*****************************************************************************
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,     ": Display list of commands" },
    { "h",        Cmd_help,     ": alias for help" },
    { "?",        Cmd_help,     ": alias for help" },
    { "stats",    Cmd_stats,    ": Display collected stats for this board" },
    { "activate", Cmd_activate, ": Get a CIK from exosite"},
    { "clear",    Cmd_clear,    ": Clear the display "},
    { "proxy",    Cmd_proxy,    ": Set or disable a HTTP proxy server." },
    { "connect",  Cmd_connect,  ": Tries to establish a connection with"
                                " exosite."},
    { "s",    Cmd_sync,      ": Syncronize data with exosite now." },
    { 0, 0, 0 }
};

//*****************************************************************************
//
// Global flag indicates if we are online currently.
//
//*****************************************************************************
extern bool g_bOnline;

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(uint32_t ui32SysClock)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the system clock for the UART.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, ui32SysClock);
}

//*****************************************************************************
//
// This function implements the "help" command.  It prints a simple list of the
// available commands with a brief description.
//
//*****************************************************************************
int
Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Get the UART semaphore
    //
    xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);

    //
    // Print some header text.
    //
    UARTprintf("\nAvailable commands\n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%15s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Give back the UART semaphore.
    //
    xSemaphoreGive(g_xUARTSemaphore);

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function prints a list of local statistics for this board.
//
//*****************************************************************************
int
Cmd_stats(int argc, char *argv[])
{




    return 0;
}

//*****************************************************************************
//
// Connects to Exosite and attempts to obtain a CIK. If no connection is made
// Cmd_activate will return and report the failure.  Use this command if you
// know you have not acquired a CIK with exosite.  Will replace any existing
// CIK with a new if acquired.
//
//*****************************************************************************
int
Cmd_activate(int argc, char *argv[])
{


    return 0;
}

//*****************************************************************************
//
// Sends a request to the cloud task for performing a sync with the server
// now.  Does not affect the normally scheduled syncs controlled by the cloud
// timer.
//
//*****************************************************************************
int
Cmd_sync(int argc, char *argv[])
{
	//
	    // Get the UART semaphore
	    //
	    xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);

	    if(argc>=2){
	    	uiUARTprintf("%d\n",argv[2]);
	    	//UARTprintf(" 1 2 3 %d\n",argv[2]);
	    }


	    //
	    // Give back the UART semaphore.
	    //
	    xSemaphoreGive(g_xUARTSemaphore);


    return 0;

}

//*****************************************************************************
//
// The "connect" command alerts the main application that it should attempt to
// re-establish a link with the exosite server. Use this when you want to
// connect again after a cable unplug or other loss of internet connectivity.
// Will use the existing CIK if valid, will acquire a new CIK as needed.
//
//*****************************************************************************
int
Cmd_connect(int argc, char *argv[])
{


    return 0;
}


//*****************************************************************************
//
// The proxy command accepts a URL string as a parameter. This string is then
// used as a HTTP proxy for all future internet communicaitons.
//
//*****************************************************************************
int
Cmd_proxy(int argc, char *argv[])
{

    return 0;
}

//*****************************************************************************
//
// The "clear" command sends an ascii control code to the UART that should
// clear the screen for most PC-side terminals.
//
//*****************************************************************************
int
Cmd_clear(int argc, char *argv[])
{
    //
    // Take the UART semaphore, send a clear escape sequence and give back
    // the semaphore.
    //
    xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);
    UARTprintf("\033[2J\033[H");
    xSemaphoreGive(g_xUARTSemaphore);

    return 0;
}

//*****************************************************************************
//
// The main function of the Command Task.
//
//*****************************************************************************
static void
CommandTask(void *pvParameters)
{
    int32_t i32CarriageReturnPosition;
    portTickType xLastWakeTime;
    char cInput[COMMAND_INPUT_BUF_SIZE];
    int iStatus;

    //
    // Get the current time as a reference to start our delays.
    //
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {

        //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&xLastWakeTime, COMMAND_TASK_PERIOD_MS /
                        portTICK_RATE_MS);

        //
        // Enter a critical section to check for new commands.
        // Could probably do this with just turning off/on UART interrupts.
        //
        vPortEnterCritical();

        //
        // Peek at the buffer to see if a \r is there.  If so we have a
        // complete command that needs processing. Make sure your terminal
        // sends a \r when you press 'enter'.
        //
        i32CarriageReturnPosition = UARTPeek('\r');

        //
        // End Critical section
        //
        vPortExitCritical();

        if(i32CarriageReturnPosition != (-1))
        {
            //
            // Get the command and pass it to the command interpreter.
            //
            UARTgets(cInput, COMMAND_INPUT_BUF_SIZE);

            //
            // Process the received command
            //
            iStatus = CmdLineProcess(cInput);

            //
            // Take the UART semaphore
            //
            xSemaphoreTake(g_xUARTSemaphore, portMAX_DELAY);

            //
            // Handle the case of bad command.
            //
            if(iStatus == CMDLINE_BAD_CMD)
            {
                UARTprintf("Bad command!\n");
            }

            //
            // Handle the case of too many arguments.
            //
            else if(iStatus == CMDLINE_TOO_MANY_ARGS)
            {
                UARTprintf("Too many arguments for command processor!\n");
            }

            //
            // Handle the case of too few arguments.
            //
            else if(iStatus == CMDLINE_TOO_FEW_ARGS)
            {
                UARTprintf("Too few arguments for command processor!\n");
            }

            //
            // Print a command prompt.
            //
            UARTprintf("\n>");

            //
            // Give back the semaphore.
            //
            xSemaphoreGive(g_xUARTSemaphore);
        }
    }
}

//*****************************************************************************
//
// Initializes the Command task.
//
//*****************************************************************************
uint32_t CommandTaskInit(void)
{
    //
    // Configure the UART and the UARTStdio library.
    //
    ConfigureUART(g_ui32SysClock);

    //
    // Make sure the UARTStdioIntHandler priority is low to not interfere
    // with the RTOS. This may not be needed since the int handler does not
    // call FreeRTOS functions ("fromISR" or otherwise).
    //
    IntPrioritySet(INT_UART0, 0xE0);

    //
    // Create a mutex to guard the UART.
    //
    g_xUARTSemaphore = xSemaphoreCreateMutex();

    //
    // Create the switch task.
    //
    if(xTaskCreate(CommandTask, (signed portCHAR *)"Command",
                   COMMAND_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_COMMAND_TASK, g_xCommandHandle) != pdTRUE)
    {
        //
        // Task creation failed.
        //
        return(1);
    }

    //
    // Check if queue creation and semaphore was successful.
    //
    if(g_xUARTSemaphore == NULL)
    {
        //
        // queue was not created successfully.
        //
        return(1);
    }

    return(0);

}
