//*****************************************************************************
//
// xbee task based on TI code
//
//*****************************************************************************

#ifndef __XBEERX_TASK_H__
#define __XBEERX_TASK_H__

//*****************************************************************************
//
// The stack size for the COMMAND task.
//
//*****************************************************************************
#define XBEERX_TASK_STACK_SIZE        2048         // Stack size in words

//*****************************************************************************
//
// Period in milliseconds to determine time between how often run the task.
// This determines how often we check for the carriage return which indicates
// we need to process a command.
//
//*****************************************************************************
#define XBEERX_TASK_PERIOD_MS         100        // periodic rate of the task

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
extern uint32_t XBEErxTaskInit(void);

//*****************************************************************************
//
// Forward declarations for command-line operations.
//
//*****************************************************************************


void bsmParse(char *cInput);

float tCollide(int dist, int bear, float myVeloc, int myHead, float oVeloc,
		int oHead);
float tCollideAcc(int dist, int bear, float myV, int myA_y, int myA_x,
		int myHead, float oV, int oA_y, int oA_x, int oHead);
float min(float v1, float v2);

#endif // __XBEERX_TASK_H__
