//*****************************************************************************
//
// mpu9150.h - Prototypes for the MPU9150 accelerometer, gyroscope, and
//             magnetometer driver.
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
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#ifndef __ADXL312_H__
#define __ADXL312_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The structure that defines the internal state of the MPU9150 driver.
//
//*****************************************************************************
typedef struct
{

    //
    // The I2C address of the MPU9150.
    //
    uint8_t ui8Addr;

    //
    // The state of the state machine used while accessing the MPU9150.
    //
    uint8_t ui8State;

    //
    // The current accelerometer afs_sel setting
    //
    uint8_t ui8AccelAfsSel;

    //
    // The new accelerometer afs_sel setting, which is used when a register
    // write succeeds.
    //
    uint8_t ui8NewAccelAfsSel;


    //
    // The data buffer used for sending/receiving data to/from the MPU9150.
    //
    uint8_t pui8Data[24];


}
tADXL312;

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
void I2CSendString(uint32_t slave_addr, char array[]);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __ADXL312_H__
