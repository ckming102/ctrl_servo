/*==============================================================================
  Header for the UART

    Received bytes are copied into a ring buffer by the RX interrupt and read
    from the main loop with uart_ReadByte(). Sent bytes go through a TX ring
    buffer emptied by the UDRE interrupt.
 =============================================================================*/
#ifndef UART_H
#define UART_H

#include "global.h"

/* -----------------------*/
/*  UART Buffers defines  */
/* -----------------------*/
#define UART_RX_BUFFER_SIZE 64
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#define UART_TX_BUFFER_SIZE 128
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )

/* check power of 2 size */
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
  #error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
  #error TX buffer size is not a power of 2
#endif

/* Baud rate: 19.2 kbps */
#define UART_BAUD 19200UL
#define UART_UBRR ((F_CPU / (16UL * UART_BAUD)) - 1)

/* selected UART: set by uart init */
extern uint8_t UART_ID;

/* fixed bit positions */

/* UCSRnA */
#define RXCn    7
#define TXCn    6
#define UDREn   5
#define FEn     4
#define DORn    3
#define UPEn    2
#define U2Xn    1
#define MPCMn   0

/* UCSRnB */
#define RXCIEn  7
#define TXCIEn  6
#define UDRIEn  5
#define RXENn   4
#define TXENn   3
#define UCSZn2  2
#define RXB8n   1
#define TXB8n   0

/* UCSRnC */
#define UMSELn1 7
#define UMSELn0 6
#define UPMn1   5
#define UPMn0   4
#define USBSn   3
#define UCSZn1  2
#define UCSZn0  1
#define UCPOLn  0

/* ----------------- */
/*  uart interfaces  */
/* ----------------- */
extern void uart_Init(uint8_t);
extern int16_t uart_ReadByte(void);
extern void uart_SendByte(char data);
extern void uart_SendString(const char *text);
extern void uart_SendInt(int data);

#endif
