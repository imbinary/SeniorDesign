//*****************************************************************************
//
// compdcm_task.h - Prototypes for the Complimentary Filtered DCM task.
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

#ifndef __ADXL_TASK_H__
#define __ADXL_TASK_H__

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define ADXL_TASK_STACK_SIZE        1024         // Stack size in words

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
extern xTaskHandle g_xADXLHandle;

#define ADXL_TASK_PERIOD_MS         200        // periodic rate of the task

//*****************************************************************************
//
// Prototypes for the switch task.
//
//*****************************************************************************
extern uint32_t ADXLTaskInit(void);
extern void updateBSM( int16_t x, int16_t y, int16_t z);
extern void InitI2C0(void);
#endif // __ADXLM_TASK_H__
