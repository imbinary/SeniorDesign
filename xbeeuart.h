//*****************************************************************************
//
// uartstdio.h - Prototypes for the UART console functions.
//
// Copyright (c) 2007-2015 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.1.71 of the Tiva Utility Library.
//
//*****************************************************************************

#ifndef __XBEEUART_H__
#define __XBEEUART_H__

#include <stdarg.h>

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
// If built for buffered operation, the following labels define the sizes of
// the transmit and receive buffers respectively.
//
//*****************************************************************************
#ifdef UART_BUFFERED
#ifndef xUART_RX_BUFFER_SIZE
#define xUART_RX_BUFFER_SIZE     512
#endif
#ifndef xUART_TX_BUFFER_SIZE
#define xUART_TX_BUFFER_SIZE     1024
#endif
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void xbeeUARTxConfig(uint32_t ui32Port, uint32_t ui32Baud,
                            uint32_t ui32SrcClock);
extern int xbeeUARTgets(char *pcBuf, uint32_t ui32Len);
extern unsigned char xbeeUARTgetc(void);
extern void xbeeUARTprintf(const char *pcString, ...);
extern void xbeeUARTvprintf(const char *pcString, va_list vaArgP);
extern int xbeeUARTwrite(const char *pcBuf, uint32_t ui32Len);
#ifdef UART_BUFFERED
extern int xbeeUARTPeek(unsigned char ucChar);
extern void xbeeUARTFlushTx(bool bDiscard);
extern void xbeeUARTFlushRx(void);
extern int xbeeUARTRxBytesAvail(void);
extern int xbeeUARTTxBytesFree(void);
extern void xbeeUARTEchoSet(bool bEnable);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __XBEEUART_H__
