//*****************************************************************************
//
// xbee task based on TI code
//
//*****************************************************************************

#ifndef __XBEE_TASK_H__
#define __XBEE_TASK_H__

//*****************************************************************************
//
// The stack size for the COMMAND task.
//
//*****************************************************************************
#define XBEE_TASK_STACK_SIZE        1024         // Stack size in words

//*****************************************************************************
//
// Period in milliseconds to determine time between how often run the task.
// This determines how often we check for the carriage return which indicates
// we need to process a command.
//
//*****************************************************************************
#define XBEE_TASK_PERIOD_MS         100        // periodic rate of the task

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
extern xTaskHandle g_xXBEEHandle;

//*****************************************************************************
//
// A handle for the UART semaphore. Only used when more than one message must
// be kept in order without other tasks injecting messages in between.
//
//*****************************************************************************
extern xSemaphoreHandle g_xbeeUARTSemaphore;

//*****************************************************************************
//
// Prototypes for the command task.
//
//*****************************************************************************
extern uint32_t XBEETaskInit(void);

//*****************************************************************************
//
// Forward declarations for command-line operations.
//
//*****************************************************************************

void bsmSend();
//void bsmParse(char *cInput);

#endif // __XBEE_TASK_H__
