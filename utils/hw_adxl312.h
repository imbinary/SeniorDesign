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


#define ADXL312_I2CADR		(0x1D)
#define ADXL312_I2CADR_ALT	(0x53)
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


//*****************************************************************************
//
// The following are defines for the ADXL312 register addresses.
//
//*****************************************************************************
#define ADXL312_O_SELF_TEST_X   0x0D        // Self test X register
#define ADXL312_O_SELF_TEST_Y   0x0E        // Self test Y register
#define ADXL312_O_SELF_TEST_Z   0x0F        // Self test Z register
#define ADXL312_O_SELF_TEST_A   0x10        // Self test A register
#define ADXL312_O_SMPLRT_DIV    0x19        // Sample rate divider register
#define ADXL312_O_CONFIG        0x1A        // Configuration register
#define ADXL312_O_ACCEL_CONFIG  0x1C        // Accelerometer configuration
                                            // register
#define ADXL312_O_FF_THR        0x1D        // Free-fall threshold register
#define ADXL312_O_FF_DUR        0x1E        // Free-fall duration register
#define ADXL312_O_MOT_THR       0x1F        // Motion detection threshold
                                            // register
#define ADXL312_O_MOT_DUR       0x20        // Motion detection duration
                                            // register
#define ADXL312_O_ZRMOT_THR     0x21        // Zero motion detection threshold
                                            // register
#define ADXL312_O_ZRMOT_DUR     0x22        // Zero motion detection duration
                                            // register
#define ADXL312_O_FIFO_EN       0x38        // FIFO enable register
#define ADXL312_O_I2C_MST_CTRL  0x24        // I2C master control register

#define ADXL312_O_I2C_MST_STATUS                                              \
                                0x36        // I2C master status register
#define ADXL312_O_INT_PIN_CFG   0x37        // INT pin configuration register
#define ADXL312_O_INT_ENABLE    0x38        // Interrupt enable register
#define ADXL312_O_INT_STATUS    0x3A        // Interrupt status register
#define ADXL312_O_ACCEL_XOUT_H  0x32        // X-axis acceleration data MSB
                                            // register
#define ADXL312_O_ACCEL_XOUT_L  0x33        // X-axis acceleration data LSB
                                            // register
#define ADXL312_O_ACCEL_YOUT_H  0x34        // Y-axis acceleration data MSB
                                            // register
#define ADXL312_O_ACCEL_YOUT_L  0x35        // Y-axis accelearation data LSB
                                            // register
#define ADXL312_O_ACCEL_ZOUT_H  0x36        // Z-axis acceleration data MSB
                                            // register
#define ADXL312_O_ACCEL_ZOUT_L  0x37        // Z-axis acceleration data LSB
                                            // register


#define ADXL312_O_SIGNAL_PATH_RESET                                           \
                                0x68        // Signal path reset register
#define ADXL312_O_MOT_DETECT_CTRL                                             \
                                0x69        // Motion detection control
                                            // register
#define ADXL312_O_USER_CTRL     0x6A        // User control register
#define ADXL312_O_PWR_MGMT_1    0x6B        // Power management 1 register
#define ADXL312_O_PWR_MGMT_2    0x6C        // Power management 2 register
#define ADXL312_O_FIFO_COUNTH   0x72        // FIFO count MSB register
#define ADXL312_O_FIFO_COUNTL   0x73        // FIFO count LSB register
#define ADXL312_O_FIFO_R_W      0x74        // FIFO read write register
#define ADXL312_O_WHO_AM_I      0x75        // Who am I register

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_SELF_TEST_X
// register.
//
//*****************************************************************************
#define ADXL312_SELF_TEST_X_XA_TEST_M                                         \
                                0xE0        // Accelerometer XA_TEST[4:2]
#define ADXL312_SELF_TEST_X_XG_TEST_M                                         \
                                0x1F        // Gyro XG_TEST[4:0]
#define ADXL312_SELF_TEST_X_XA_TEST_S                                         \
                                5
#define ADXL312_SELF_TEST_X_XG_TEST_S                                         \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_SELF_TEST_Y
// register.
//
//*****************************************************************************
#define ADXL312_SELF_TEST_Y_YA_TEST_M                                         \
                                0xE0        // Accelerometer YA_TEST[4:2]
#define ADXL312_SELF_TEST_Y_YG_TEST_M                                         \
                                0x1F        // Gyro YG_TEST[4:0]
#define ADXL312_SELF_TEST_Y_YA_TEST_S                                         \
                                5
#define ADXL312_SELF_TEST_Y_YG_TEST_S                                         \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_SELF_TEST_Z
// register.
//
//*****************************************************************************
#define ADXL312_SELF_TEST_Z_ZA_TEST_M                                         \
                                0xE0        // Accelerometer ZA_TEST[4:2]
#define ADXL312_SELF_TEST_Z_ZG_TEST_M                                         \
                                0x1F        // Gyro ZG_TEST[4:0]
#define ADXL312_SELF_TEST_Z_ZA_TEST_S                                         \
                                5
#define ADXL312_SELF_TEST_Z_ZG_TEST_S                                         \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_SELF_TEST_A
// register.
//
//*****************************************************************************
#define ADXL312_SELF_TEST_A_XA_TEST_M                                         \
                                0x30        // Accelerometer XA_TEST[1:0]
#define ADXL312_SELF_TEST_A_YA_TEST_M                                         \
                                0x0C        // Accelerometer YA_TEST[1:0]
#define ADXL312_SELF_TEST_A_ZA_TEST_M                                         \
                                0x03        // Accelerometer ZA_TEST[1:0]
#define ADXL312_SELF_TEST_A_XA_TEST_S                                         \
                                4
#define ADXL312_SELF_TEST_A_YA_TEST_S                                         \
                                2
#define ADXL312_SELF_TEST_A_ZA_TEST_S                                         \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_SMPLRT_DIV
// register.
//
//*****************************************************************************
#define ADXL312_SMPLRT_DIV_M    0xFF        // Gyro output rate divider
#define ADXL312_SMPLRT_DIV_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_CONFIG
// register.
//
//*****************************************************************************
#define ADXL312_CONFIG_EXT_SYNC_SET_M                                         \
                                0x38        // FSYNC pin sample location
#define ADXL312_CONFIG_EXT_SYNC_SET_DIS                                       \
                                0x00        // FSYNC input disabled
#define ADXL312_CONFIG_EXT_SYNC_SET_TEMP_OUT_L                                \
                                0x08        // FSYNC on TEMP_OUT_L[0]
#define ADXL312_CONFIG_EXT_SYNC_SET_GYRO_XOUT_L                               \
                                0x10        // FSYNC on GYRO_XOUT_L[0]
#define ADXL312_CONFIG_EXT_SYNC_SET_GYRO_YOUT_L                               \
                                0x18        // FSYNC on GYRO_YOUT_L[0]
#define ADXL312_CONFIG_EXT_SYNC_SET_GYRO_ZOUT_L                               \
                                0x20        // FSYNC on GYRO_ZOUT_L[0]
#define ADXL312_CONFIG_EXT_SYNC_SET_ACCEL_XOUT_L                              \
                                0x28        // FSYNC on ACCEL_XOUT_L[0]
#define ADXL312_CONFIG_EXT_SYNC_SET_ACCEL_YOUT_L                              \
                                0x30        // FSYNC on ACCEL_YOUT_L[0]
#define ADXL312_CONFIG_EXT_SYNC_SET_ACCEL_ZOUT_L                              \
                                0x38        // FSYNC on ACCEL_ZOUT_L[0]
#define ADXL312_CONFIG_DLPF_CFG_M                                             \
                                0x07        // Digital low-pass filter
                                            // configuration
#define ADXL312_CONFIG_DLPF_CFG_260_256                                       \
                                0x00        // 260 Hz accelerometer bandwidth,
                                            // 256 Hz gyro bandwidth
#define ADXL312_CONFIG_DLPF_CFG_184_188                                       \
                                0x01        // 184 Hz accelerometer bandwidth,
                                            // 188 Hz gyro bandwidth
#define ADXL312_CONFIG_DLPF_CFG_94_98                                         \
                                0x02        // 94 Hz accelerometer bandwidth,
                                            // 98 Hz gyro bandwidth
#define ADXL312_CONFIG_DLPF_CFG_44_42                                         \
                                0x03        // 44 Hz accelerometer bandwidth,
                                            // 42 Hz gyro bandwidth
#define ADXL312_CONFIG_DLPF_CFG_21_20                                         \
                                0x04        // 21 Hz accelerometer bandwidth,
                                            // 20 Hz gyro bandwidth
#define ADXL312_CONFIG_DLPF_CFG_10_10                                         \
                                0x05        // 10 Hz accelerometer bandwidth,
                                            // 10 Hz gyro bandwidth
#define ADXL312_CONFIG_DLPF_CFG_5_5                                           \
                                0x06        // 5 Hz accelerometer bandwidth, 5
                                            // Hz gyro bandwidth
#define ADXL312_CONFIG_EXT_SYNC_SET_S                                         \
                                3
#define ADXL312_CONFIG_DLPF_CFG_S                                             \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_GYRO_CONFIG
// register.
//
//*****************************************************************************
#define ADXL312_GYRO_CONFIG_XG_ST                                             \
                                0x80        // X-axis gyro self-test enable
#define ADXL312_GYRO_CONFIG_YG_ST                                             \
                                0x40        // Y-axis gyro self-test enable
#define ADXL312_GYRO_CONFIG_ZG_ST                                             \
                                0x20        // Z-axis gyro self-test enable
#define ADXL312_GYRO_CONFIG_FS_SEL_M                                          \
                                0x18        // Gyro full-scale range
#define ADXL312_GYRO_CONFIG_FS_SEL_250                                        \
                                0x00        // Gyro full-scale range +/- 250
                                            // degrees/sec
#define ADXL312_GYRO_CONFIG_FS_SEL_500                                        \
                                0x08        // Gyro full-scale range +/- 500
                                            // degrees/sec
#define ADXL312_GYRO_CONFIG_FS_SEL_1000                                       \
                                0x10        // Gyro full-scale range +/- 1000
                                            // degrees/sec
#define ADXL312_GYRO_CONFIG_FS_SEL_2000                                       \
                                0x18        // Gyro full-scale range +/- 2000
                                            // degrees/sec
#define ADXL312_GYRO_CONFIG_FS_SEL_S                                          \
                                3

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_CONFIG
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_CONFIG_XA_ST                                            \
                                0x80        // X-axis accelerometer self-test
                                            // enable
#define ADXL312_ACCEL_CONFIG_YA_ST                                            \
                                0x40        // Y-axis accelerometer self-test
                                            // enable
#define ADXL312_ACCEL_CONFIG_ZA_ST                                            \
                                0x20        // Z-axis accelerometer self-test
                                            // enable
#define ADXL312_ACCEL_CONFIG_AFS_SEL_M                                        \
                                0x18        // Accelerometer full-scale range
#define ADXL312_ACCEL_CONFIG_AFS_SEL_2G                                       \
                                0x00        // Accelerometer full-scale range 2
                                            // g
#define ADXL312_ACCEL_CONFIG_AFS_SEL_4G                                       \
                                0x08        // Accelerometer full-scale range 4
                                            // g
#define ADXL312_ACCEL_CONFIG_AFS_SEL_8G                                       \
                                0x10        // Accelerometer full-scale range 8
                                            // g
#define ADXL312_ACCEL_CONFIG_AFS_SEL_16G                                      \
                                0x18        // Accelerometer full-scale range
                                            // 16 g
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_M                                      \
                                0x07        // High-pass filter setting
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_RESET                                  \
                                0x00        // High-pass filter reset
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_5HZ                                    \
                                0x01        // High-pass filter at 5 Hz
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_2_5HZ                                  \
                                0x02        // High-pass filter at 2.5 Hz
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_1_25HZ                                 \
                                0x03        // High-pass filter at 1.25 Hz
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_0_63HZ                                 \
                                0x04        // High-pass filter at 0.63 Hz
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_HOLD                                   \
                                0x07        // High-pass filter hold
#define ADXL312_ACCEL_CONFIG_AFS_SEL_S                                        \
                                3
#define ADXL312_ACCEL_CONFIG_ACCEL_HPF_S                                      \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_FF_THR
// register.
//
//*****************************************************************************
#define ADXL312_FF_THR_M        0xFF        // Free-fall threshold value
#define ADXL312_FF_THR_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_FF_DUR
// register.
//
//*****************************************************************************
#define ADXL312_FF_DUR_M        0xFF        // Free-fall duration value
#define ADXL312_FF_DUR_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_MOT_THR
// register.
//
//*****************************************************************************
#define ADXL312_MOT_THR_M       0xFF        // Motion detection threshold value
#define ADXL312_MOT_THR_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_MOT_DUR
// register.
//
//*****************************************************************************
#define ADXL312_MOT_DUR_M       0xFF        // Motion detection duration value
#define ADXL312_MOT_DUR_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ZRMOT_THR
// register.
//
//*****************************************************************************
#define ADXL312_ZRMOT_THR_M     0xFF        // Zero motion detection threshold
                                            // value
#define ADXL312_ZRMOT_THR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ZRMOT_DUR
// register.
//
//*****************************************************************************
#define ADXL312_ZRMOT_DUR_M     0xFF        // Zero motion detection duration
                                            // value
#define ADXL312_ZRMOT_DUR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_FIFO_EN
// register.
//
//*****************************************************************************
#define ADXL312_FIFO_EN_TEMP    0x80        // Temperature sensor FIFO enable
#define ADXL312_FIFO_EN_XG      0x40        // X-axis gyro FIFO enable
#define ADXL312_FIFO_EN_YG      0x20        // Y-axis gyro FIFO enable
#define ADXL312_FIFO_EN_ZG      0x10        // Z-axis gyro FIFO enable
#define ADXL312_FIFO_EN_ACCEL   0x08        // Accelerometer FIFO enable
#define ADXL312_FIFO_EN_SLV2    0x04        // Slave 2 FIFO enable
#define ADXL312_FIFO_EN_SLV1    0x02        // Slave 1 FIFO enable
#define ADXL312_FIFO_EN_SLV0    0x01        // Slave 0 FIFO enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_MST_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_I2C_MST_CTRL_MULT_MST_EN                                      \
                                0x80        // Multi-master enable
#define ADXL312_I2C_MST_CTRL_WAIT_FOR_ES                                      \
                                0x40        // Wait for external sensor data
#define ADXL312_I2C_MST_CTRL_SLV3_FIFO_EN                                     \
                                0x20        // Slave 3 FIFO enable
#define ADXL312_I2C_MST_CTRL_I2C_MST_P_NSR                                    \
                                0x10        // No repeated start conditions
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_M                                    \
                                0x0F        // I2C master clock speed
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_348                                  \
                                0x00        // 348 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_333                                  \
                                0x01        // 333 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_320                                  \
                                0x02        // 320 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_308                                  \
                                0x03        // 308 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_296                                  \
                                0x04        // 296 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_286                                  \
                                0x05        // 286 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_276                                  \
                                0x06        // 276 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_267                                  \
                                0x07        // 267 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_258                                  \
                                0x08        // 258 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_500                                  \
                                0x09        // 500 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_471                                  \
                                0x0A        // 471 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_444                                  \
                                0x0B        // 444 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_421                                  \
                                0x0C        // 421 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_400                                  \
                                0x0D        // 400 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_381                                  \
                                0x0E        // 381 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_364                                  \
                                0x0F        // 364 kHz I2C master clock
#define ADXL312_I2C_MST_CTRL_I2C_MST_CLK_S                                    \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV0_ADDR
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV0_ADDR_RW                                              \
                                0x80        // Read/not write
#define ADXL312_I2C_SLV0_ADDR_M 0x7F        // Slave address
#define ADXL312_I2C_SLV0_ADDR_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV0_REG
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV0_REG_M  0xFF        // Slave register number
#define ADXL312_I2C_SLV0_REG_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV0_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV0_CTRL_EN                                              \
                                0x80        // Enable slave
#define ADXL312_I2C_SLV0_CTRL_BYTE_SW                                         \
                                0x40        // Byte-swap word pairs
#define ADXL312_I2C_SLV0_CTRL_REG_DIS                                         \
                                0x20        // Disable register number transfer
#define ADXL312_I2C_SLV0_CTRL_GRP                                             \
                                0x10        // Word pair grouping
#define ADXL312_I2C_SLV0_CTRL_LEN_M                                           \
                                0x0F        // Number of bytes to transfer
#define ADXL312_I2C_SLV0_CTRL_LEN_S                                           \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV1_ADDR
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV1_ADDR_RW                                              \
                                0x80        // Read/not write
#define ADXL312_I2C_SLV1_ADDR_M 0x7F        // Slave address
#define ADXL312_I2C_SLV1_ADDR_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV1_REG
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV1_REG_M  0xFF        // Slave register number
#define ADXL312_I2C_SLV1_REG_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV1_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV1_CTRL_EN                                              \
                                0x80        // Enable slave
#define ADXL312_I2C_SLV1_CTRL_BYTE_SW                                         \
                                0x40        // Byte-swap word pairs
#define ADXL312_I2C_SLV1_CTRL_REG_DIS                                         \
                                0x20        // Disable register number transfer
#define ADXL312_I2C_SLV1_CTRL_GRP                                             \
                                0x10        // Word pair grouping
#define ADXL312_I2C_SLV1_CTRL_LEN_M                                           \
                                0x0F        // Number of bytes to transfer
#define ADXL312_I2C_SLV1_CTRL_LEN_S                                           \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV2_ADDR
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV2_ADDR_RW                                              \
                                0x80        // Read/not write
#define ADXL312_I2C_SLV2_ADDR_M 0x7F        // Slave address
#define ADXL312_I2C_SLV2_ADDR_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV2_REG
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV2_REG_M  0xFF        // Slave register number
#define ADXL312_I2C_SLV2_REG_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV2_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV2_CTRL_EN                                              \
                                0x80        // Enable slave
#define ADXL312_I2C_SLV2_CTRL_BYTE_SW                                         \
                                0x40        // Byte-swap word pairs
#define ADXL312_I2C_SLV2_CTRL_REG_DIS                                         \
                                0x20        // Disable register number transfer
#define ADXL312_I2C_SLV2_CTRL_GRP                                             \
                                0x10        // Word pair grouping
#define ADXL312_I2C_SLV2_CTRL_LEN_M                                           \
                                0x0F        // Number of bytes to transfer
#define ADXL312_I2C_SLV2_CTRL_LEN_S                                           \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV3_ADDR
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV3_ADDR_RW                                              \
                                0x80        // Read/not write
#define ADXL312_I2C_SLV3_ADDR_M 0x7F        // Slave address
#define ADXL312_I2C_SLV3_ADDR_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV3_REG
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV3_REG_M  0xFF        // Slave register number
#define ADXL312_I2C_SLV3_REG_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV3_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV3_CTRL_EN                                              \
                                0x80        // Enable slave
#define ADXL312_I2C_SLV3_CTRL_BYTE_SW                                         \
                                0x40        // Byte-swap word pairs
#define ADXL312_I2C_SLV3_CTRL_REG_DIS                                         \
                                0x20        // Disable register number transfer
#define ADXL312_I2C_SLV3_CTRL_GRP                                             \
                                0x10        // Word pair grouping
#define ADXL312_I2C_SLV3_CTRL_LEN_M                                           \
                                0x0F        // Number of bytes to transfer
#define ADXL312_I2C_SLV3_CTRL_LEN_S                                           \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV4_ADDR
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV4_ADDR_RW                                              \
                                0x80        // Read/not write
#define ADXL312_I2C_SLV4_ADDR_M 0x7F        // Slave address
#define ADXL312_I2C_SLV4_ADDR_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV4_REG
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV4_REG_M  0xFF        // Slave register number
#define ADXL312_I2C_SLV4_REG_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV4_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV4_CTRL_EN                                              \
                                0x80        // Enable slave
#define ADXL312_I2C_SLV4_CTRL_INT_EN                                          \
                                0x40        // Interrupt enable
#define ADXL312_I2C_SLV4_CTRL_REG_DIS                                         \
                                0x20        // Disable register number transfer
#define ADXL312_I2C_SLV4_CTRL_I2C_MST_DLY_M                                   \
                                0x1F        // Slave access delay
#define ADXL312_I2C_SLV4_CTRL_I2C_MST_DLY_S                                   \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_SLV4_DI
// register.
//
//*****************************************************************************
#define ADXL312_I2C_SLV4_DI_M   0xFF        // Input data
#define ADXL312_I2C_SLV4_DI_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_I2C_MST_STATUS
// register.
//
//*****************************************************************************
#define ADXL312_I2C_MST_STATUS_PASS_THROUGH                                   \
                                0x80        // Pass through FSYNC interrupt
                                            // status
#define ADXL312_I2C_MST_STATUS_I2C_SLV4_DONE                                  \
                                0x40        // I2C slave 4 completion status
#define ADXL312_I2C_MST_STATUS_I2C_LOST_ARB                                   \
                                0x20        // I2C arbitration lost status
#define ADXL312_I2C_MST_STATUS_I2C_SLV4_NACK                                  \
                                0x10        // I2C slave 4 NACK status
#define ADXL312_I2C_MST_STATUS_I2C_SLV3_NACK                                  \
                                0x08        // I2C slave 3 NACK status
#define ADXL312_I2C_MST_STATUS_I2C_SLV2_NACK                                  \
                                0x04        // I2C slave 2 NACK status
#define ADXL312_I2C_MST_STATUS_I2C_SLV1_NACK                                  \
                                0x02        // I2C slave 1 NACK status
#define ADXL312_I2C_MST_STATUS_I2C_SLV0_NACK                                  \
                                0x01        // I2C slave 0 NACK status

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_INT_PIN_CFG
// register.
//
//*****************************************************************************
#define ADXL312_INT_PIN_CFG_INT_LEVEL                                         \
                                0x80        // INT pin active low
#define ADXL312_INT_PIN_CFG_INT_OPEN                                          \
                                0x40        // INT pin open-drain
#define ADXL312_INT_PIN_CFG_LATCH_INT_EN                                      \
                                0x20        // Latch INT pin output
#define ADXL312_INT_PIN_CFG_INT_RD_CLEAR                                      \
                                0x10        // Interrupt clear on any read
#define ADXL312_INT_PIN_CFG_FSYNC_INT_LEVEL                                   \
                                0x08        // FSYNC pin active low
#define ADXL312_INT_PIN_CFG_FSYNC_INT_EN                                      \
                                0x04        // FSYNC pin interrupt enable
#define ADXL312_INT_PIN_CFG_I2C_BYPASS_EN                                     \
                                0x02        // I2C bypass enable
#define ADXL312_INT_PIN_CFG_CLKOUT_EN                                         \
                                0x01        // CLKOUT enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_INT_ENABLE
// register.
//
//*****************************************************************************
#define ADXL312_INT_ENABLE_FF_EN                                              \
                                0x80        // Free-fall interrupt enable
#define ADXL312_INT_ENABLE_MOT_EN                                             \
                                0x40        // Motion detection interrupt
                                            // enable
#define ADXL312_INT_ENABLE_ZMOT_EN                                            \
                                0x20        // Zero motion interrupt enable
#define ADXL312_INT_ENABLE_FIFO_OFLOW_EN                                      \
                                0x10        // FIFO overflow interrupt enable
#define ADXL312_INT_ENABLE_I2C_MST_INT_EN                                     \
                                0x08        // I2C master interrupt enable
#define ADXL312_INT_ENABLE_DATA_RDY_EN                                        \
                                0x01        // Data ready interrupt enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_INT_STATUS
// register.
//
//*****************************************************************************
#define ADXL312_INT_STATUS_FF_INT                                             \
                                0x80        // Free-fall interrupt status
#define ADXL312_INT_STATUS_MOT_INT                                            \
                                0x40        // Motion detection interrupt
                                            // status
#define ADXL312_INT_STATUS_ZMOT_INT                                           \
                                0x20        // Zero motion interrupt status
#define ADXL312_INT_STATUS_FIFO_OFLOW_INT                                     \
                                0x10        // FIFO overflow interrupt status
#define ADXL312_INT_STATUS_I2C_MST_INT                                        \
                                0x08        // I2C master interrupt status
#define ADXL312_INT_STATUS_DATA_RDY_INT                                       \
                                0x01        // Data ready interrupt status

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_XOUT_H
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_XOUT_H_M  0xFF        // Bits [15:8] of X-axis
                                            // acceleration data
#define ADXL312_ACCEL_XOUT_H_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_XOUT_L
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_XOUT_L_M  0xFF        // Bits [7:0] of X-axis
                                            // acceleration data
#define ADXL312_ACCEL_XOUT_L_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_YOUT_H
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_YOUT_H_M  0xFF        // Bits [15:8] of Y-axis
                                            // acceleration data
#define ADXL312_ACCEL_YOUT_H_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_YOUT_L
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_YOUT_L_M  0xFF        // Bits [7:0] of Y-axis
                                            // acceleration data
#define ADXL312_ACCEL_YOUT_L_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_ZOUT_H
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_ZOUT_H_M  0xFF        // Bits [15:8] of Z-axis
                                            // acceleration data
#define ADXL312_ACCEL_ZOUT_H_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_ACCEL_ZOUT_L
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_ZOUT_L_M  0xFF        // Bits [7:0] of Z-axis
                                            // acceleration data
#define ADXL312_ACCEL_ZOUT_L_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_TEMP_OUT_H
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_TEMP_OUT_H_M                                            \
                                0xFF        // Bits [15:8] of temperature data
#define ADXL312_ACCEL_TEMP_OUT_H_S                                            \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_TEMP_OUT_L
// register.
//
//*****************************************************************************
#define ADXL312_ACCEL_TEMP_OUT_L_M                                            \
                                0xFF        // Bits [7:0] of temperature data
#define ADXL312_ACCEL_TEMP_OUT_L_S                                            \
                                0



//*****************************************************************************
//
// The following are defines for the bit fields in the
// ADXL312_O_SIGNAL_PATH_RESET register.
//
//*****************************************************************************
#define ADXL312_SIGNAL_PATH_RESET_GYRO                                        \
                                0x04        // Reset gyro
#define ADXL312_SIGNAL_PATH_RESET_ACCEL                                       \
                                0x02        // Reset accelerometer
#define ADXL312_SIGNAL_PATH_RESET_TEMP                                        \
                                0x01        // Reset temperature sensor

//*****************************************************************************
//
// The following are defines for the bit fields in the
// ADXL312_O_MOT_DETECT_CTRL register.
//
//*****************************************************************************
#define ADXL312_MOT_DETECT_CTRL_ACCEL_ON_DELAY_M                              \
                                0x30        // Accelerometer wake-up delay
#define ADXL312_MOT_DETECT_CTRL_ACCEL_ON_DELAY_4MS                            \
                                0x00        // Delay 4 ms
#define ADXL312_MOT_DETECT_CTRL_ACCEL_ON_DELAY_5MS                            \
                                0x10        // Delay 5 ms
#define ADXL312_MOT_DETECT_CTRL_ACCEL_ON_DELAY_6MS                            \
                                0x20        // Delay 6 ms
#define ADXL312_MOT_DETECT_CTRL_ACCEL_ON_DELAY_7MS                            \
                                0x30        // Delay 7 ms
#define ADXL312_MOT_DETECT_CTRL_FF_COUNT_M                                    \
                                0x0C        // Free-fall counter decrement rate
#define ADXL312_MOT_DETECT_CTRL_FF_COUNT_RESET                                \
                                0x00        // Reset counter
#define ADXL312_MOT_DETECT_CTRL_FF_COUNT_1                                    \
                                0x04        // Decrement by 1
#define ADXL312_MOT_DETECT_CTRL_FF_COUNT_2                                    \
                                0x08        // Decrement by 2
#define ADXL312_MOT_DETECT_CTRL_FF_COUNT_4                                    \
                                0x0C        // Decrement by 4
#define ADXL312_MOT_DETECT_CTRL_MOT_COUNT_M                                   \
                                0x03        // Motion detect counter decrement
                                            // rate
#define ADXL312_MOT_DETECT_CTRL_MOT_COUNT_RESET                               \
                                0x00        // Reset counter
#define ADXL312_MOT_DETECT_CTRL_MOT_COUNT_1                                   \
                                0x04        // Decrement by 1
#define ADXL312_MOT_DETECT_CTRL_MOT_COUNT_2                                   \
                                0x08        // Decrement by 2
#define ADXL312_MOT_DETECT_CTRL_MOT_COUNT_4                                   \
                                0x0C        // Decrement by 4
#define ADXL312_MOT_DETECT_CTRL_ACCEL_ON_DELAY_S                              \
                                4
#define ADXL312_MOT_DETECT_CTRL_FF_COUNT_S                                    \
                                2
#define ADXL312_MOT_DETECT_CTRL_MOT_COUNT_S                                   \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_USER_CTRL
// register.
//
//*****************************************************************************
#define ADXL312_USER_CTRL_FIFO_EN                                             \
                                0x40        // FIFO enable
#define ADXL312_USER_CTRL_I2C_MST_EN                                          \
                                0x20        // I2C master mode enable
#define ADXL312_USER_CTRL_I2C_IF_DIS                                          \
                                0x10        // Write as zero
#define ADXL312_USER_CTRL_FIFO_RESET                                          \
                                0x04        // Reset FIFO buffer
#define ADXL312_USER_CTRL_I2C_MST_RESET                                       \
                                0x02        // Reset I2C master
#define ADXL312_USER_CTRL_SIG_COND_RESET                                      \
                                0x01        // Reset all sensors

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_PWR_MGMT_1
// register.
//
//*****************************************************************************
#define ADXL312_PWR_MGMT_1_DEVICE_RESET                                       \
                                0x80        // Device reset
#define ADXL312_PWR_MGMT_1_SLEEP                                              \
                                0x40        // Enter sleep mode
#define ADXL312_PWR_MGMT_1_CYCLE                                              \
                                0x20        // Enable automatic sleep
#define ADXL312_PWR_MGMT_1_TEMP_DIS                                           \
                                0x08        // Disable temperature sensor
#define ADXL312_PWR_MGMT_1_CLKSEL_M                                           \
                                0x07        // Clock source select
#define ADXL312_PWR_MGMT_1_CLKSEL_INT                                         \
                                0x00        // Internal 8 MHz oscillator
#define ADXL312_PWR_MGMT_1_CLKSEL_XG                                          \
                                0x01        // PLL with X-axis gyro reference
#define ADXL312_PWR_MGMT_1_CLKSEL_YG                                          \
                                0x02        // PLL with Y-axis gyro reference
#define ADXL312_PWR_MGMT_1_CLKSEL_ZG                                          \
                                0x03        // PLL with Z-axis gyro reference
#define ADXL312_PWR_MGMT_1_CLKSEL_EXT32K                                      \
                                0x04        // PLL with external 32.768 kHz
                                            // reference
#define ADXL312_PWR_MGMT_1_CLKSEL_EXT19M                                      \
                                0x05        // PLL with external 19.2 MHz
                                            // reference
#define ADXL312_PWR_MGMT_1_CLKSEL_STOP                                        \
                                0x07        // Clock disable
#define ADXL312_PWR_MGMT_1_CLKSEL_S                                           \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_PWR_MGMT_2
// register.
//
//*****************************************************************************
#define ADXL312_PWR_MGMT_2_LP_WAKE_CTRL_M                                     \
                                0xC0        // Wake-up frequency
#define ADXL312_PWR_MGMT_2_LP_WAKE_CTRL_1_25                                  \
                                0x00        // Wake-up at 1.25 Hz
#define ADXL312_PWR_MGMT_2_LP_WAKE_CTRL_5                                     \
                                0x40        // Wake-up at 5 Hz
#define ADXL312_PWR_MGMT_2_LP_WAKE_CTRL_20                                    \
                                0x80        // Wake-up at 20 Hz
#define ADXL312_PWR_MGMT_2_LP_WAKE_CTRL_40                                    \
                                0xC0        // Wake-up at 40 Hz
#define ADXL312_PWR_MGMT_2_STBY_XA                                            \
                                0x20        // Put X-axis accelerometer into
                                            // standby mode
#define ADXL312_PWR_MGMT_2_STBY_YA                                            \
                                0x10        // Put Y-axis accelerometer into
                                            // standby mode
#define ADXL312_PWR_MGMT_2_STBY_ZA                                            \
                                0x08        // Put Z-axis accelerometer into
                                            // standby mode
#define ADXL312_PWR_MGMT_2_STBY_XG                                            \
                                0x04        // Put X-axis gyro into standby
                                            // mode
#define ADXL312_PWR_MGMT_2_STBY_YG                                            \
                                0x02        // Put Y-axis gyro into standby
                                            // mode
#define ADXL312_PWR_MGMT_2_STBY_ZG                                            \
                                0x01        // Put Z-axis gyro into standby
                                            // mode
#define ADXL312_PWR_MGMT_2_LP_WAKE_CTRL_S                                     \
                                6

//*****************************************************************************
//
// The following are defines for the bit fields in the ADXL312_O_WHO_AM_I
// register.
//
//*****************************************************************************
#define ADXL312_WHO_AM_I_M      0x7E        // I2C address
#define ADXL312_WHO_AM_I_ADXL312                                              \
                                0xE5        // ADXL312
#define ADXL312_WHO_AM_I_S      1

#endif // __SENSORLIB_HW_ADXL312_H__
