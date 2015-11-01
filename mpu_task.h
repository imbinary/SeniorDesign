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

#ifndef __MPU_TASK_H__
#define __MPU_TASK_H__

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define MPU_TASK_STACK_SIZE        1024         // Stack size in words

//*****************************************************************************
//
// A handle by which this task and others can refer to this task.
//
//*****************************************************************************
extern xTaskHandle g_xMPUHandle;

#define MPU_TASK_PERIOD_MS         100        // periodic rate of the task

//*****************************************************************************
//
// Prototypes for the switch task.
//
//*****************************************************************************
extern uint32_t MPUTaskInit(void);
extern void mupdateBSM( float* pfAcceleration, float* pfAngularVelocity);
extern void InitI2C7(void);
uint8_t mpuReadAccel(uint8_t reg);
#endif // __MPU_TASK_H__
