

#ifndef __LED_TASK_H__
#define __LED_TASK_H__


//*****************************************************************************
//
// Period in milliseconds to determine time between how often run the task.
// This determines how often we check for the carriage return which indicates
// we need to process a command.
//
//*****************************************************************************
#define LED_TASK_PERIOD_MS         500        // periodic rate of the task

#define LED_TASK_STACK_SIZE        512         // Stack size in words

extern xTaskHandle g_xLedHandle;

//*****************************************************************************
//
// Prototypes for the command task.
//
//*****************************************************************************
extern uint32_t LedTaskInit(void);


#endif // __LED_TASK_H__
