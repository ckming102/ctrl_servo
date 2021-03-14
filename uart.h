/*==============================================================================
  Header for the UART
 =============================================================================*/
#include "global.h"

#ifndef UART_H
#define UART_H

/* -----------------------*/
/*  UART Buffers defines  */
/* -----------------------*/
    #define UART_RX_BUFFER_SIZE 128
    #define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
    #define UART_TX_BUFFER_SIZE 64
    #define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )

    /* check power of 2 size */
    #if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
      #error RX buffer size is not a power of 2
    #endif
    #if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
      #error TX buffer size is not a power of 2
    #endif

    /* UDR empty interrupt */
    #define SET_UDRIE sethigh_1bit(UCSR0B, UDRIE0)
    #define CLR_UDRIE setlow_1bit(UCSR0B, UDRIE0)

/* ----------------- */
/*  uart interfaces  */
/* ----------------- */
    void uart_Init(void);
    void uart_SendByte(char data);
    void uart_SendString(char text[]);
    void uart_SendInt(int data);
    void uart_FlushRxBuffer(void);

/* ---------------------------------------------------- */
/*  String buffer from uart. Parsing is done in main()  */
/* ---------------------------------------------------- */
    extern char UART_RxBuffer[UART_RX_BUFFER_SIZE];
    extern unsigned char UART_RxPtr;

#endif
