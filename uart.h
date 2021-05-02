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

/*  String buffer from uart. Parsing is done in main()  */
extern char UART_RxBuffer[UART_RX_BUFFER_SIZE];
extern uint8_t UART_RxPtr;

/* check power of 2 size */
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
  #error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
  #error TX buffer size is not a power of 2
#endif

/* data register */
extern volatile uint8_t  *UDRn;

/* control and status registers */
extern volatile uint8_t *UCSRnA;
extern volatile uint8_t *UCSRnB;
extern volatile uint8_t *UCSRnC;

/* alias */
#define UART_UDRn UDRn
#define UART_UCSRnA UCSRnA
#define UART_UCSRnB UCSRnB
#define UART_UCSRnC UCSRnC

/* selected UART data register: set by uart init */
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

/* UDR empty interrupt */
#define SET_UDRIE sethigh_1bit(*UART_UCSRnB, UDRIEn)
#define CLR_UDRIE setlow_1bit(*UART_UCSRnB, UDRIEn)

/* ----------------- */
/*  uart interfaces  */
/* ----------------- */
void uart_Init(uint8_t);
void uart_SendByte(char data);
void uart_SendString(char text[]);
void uart_SendInt(int data);
void uart_FlushRxBuffer(void);

#endif
