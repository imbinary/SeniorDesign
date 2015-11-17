//*****************************************************************************
//
// xbee uart based on TI code
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
#define xUART_RX_BUFFER_SIZE     4096
#endif
#ifndef xUART_TX_BUFFER_SIZE
#define xUART_TX_BUFFER_SIZE     2048
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
extern int xbeeUARTgetr(char *pcBuf, uint32_t ui32Len);
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
