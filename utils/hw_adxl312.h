//*****************************************************************************
//
// hw_mpu9150.h - Macros used when accessing the Invensense ADXL312
//                accelerometer/gyroscope/magnetometer.
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

#ifndef __SENSORLIB_HW_ADXL312_H__
#define __SENSORLIB_HW_ADXL312_H__


# define ADXL312_I2CADR		(0x1D)
# define ADXL312_I2CADR_ALT	(0x53)
# define ADXL_DEVID			0x00	//R
# define ADXL_OFSX			0x1E	//R+W
# define ADXL_OFSY			0x1F	//R+W
# define ADXL_OSFZ			0x20	//R+W
# define ADXL_TRESH_ACT		0x24	//R+W
# define ADXL_TRESH_INACT	0x25	//R+W
# define ADXL_TIME_INACT	0x26	//R+W
# define ADXL_ACT_INACT_CTL 0x27	//R+W
# define ADXL_BW_RATE		0x2C	//R+W
# define ADXL_POWER_CTL		0x2D	//R+W
# define ADXL_INT_ENABLE	0x2E	//R+W
# define ADXL_INT_MAP		0x2F	//R+W
# define ADXL_INT_SOURCE	0x30	//R
# define ADXL_DATA_FORMAT	0x31	//R+W
# define ADXL_DATAX0		0x32	//R
# define ADXL_DATAX1		0x33	//R
# define ADXL_DATAY0		0x34	//R
# define ADXL_DATAY1		0x35	//R
# define ADXL_DATAZ0		0x36	//R
# define ADXL_DATAZ1		0x37	//R
# define ADXL_FIFO_CTL		0x38	//R+W
# define ADXL_FIFO_STATUS	0x39	//R


#endif // __SENSORLIB_HW_ADXL312_H__
