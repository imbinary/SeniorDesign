
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
#include "driverlib/timer.h"
#include "utils/ustdlib.h"
#include "drivers/pinout.h"
#include "drivers/buttons.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "uiuart.h"
#include "led_task.h"


xTaskHandle g_xLedHandle;

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureLED()
{
    //
    // Enable the GPIO Peripheral used by the LED.
    //
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);


	// Configure the LEDs as outputs and turn them on.
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0);





}


//*****************************************************************************
//
// The main function of the Command Task.
//
//*****************************************************************************
static void
LedTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    int8_t led=0;

    //
    // Get the current time as a reference to start our delays.
    //
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {

        //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&xLastWakeTime, LED_TASK_PERIOD_MS /
                        portTICK_RATE_MS);


        led ^= 3;
        ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, led);

    }
}


uint32_t LedTaskInit(void)
{

    ConfigureLED();

    //
    // Create the switch task.
    //
    if(xTaskCreate(LedTask, (signed portCHAR *)"Led",
                   LED_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_LED_TASK, g_xLedHandle) != pdTRUE)
    {
        //
        // Task creation failed.
        //
        return(1);
    }



    return(0);

}

