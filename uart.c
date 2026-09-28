/*==============================================================================
  Function declarations and data structures for the UART
 =============================================================================*/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "global.h"
#include "uart.h"

/* ------------------ */
/*  Extern variables  */
/* ------------------ */

uint8_t UART_ID;

/* ------------------ */
/*  Static variables  */
/* ------------------ */

/* RX buffer and head/tail pointers (idx counters) */
static volatile char UART_RxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t UART_RxHead;
static volatile uint8_t UART_RxTail;

/* TX buffer and head/tail pointers (idx counters) */
static volatile char UART_TxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;

/* ===================== */
/* Pointers to Registers */
/* ===================== */

/* data register */
static volatile uint8_t  *UDRn;

/* control and status registers */
static volatile uint8_t *UCSRnA;
static volatile uint8_t *UCSRnB;
static volatile uint8_t *UCSRnC;

/* baud rate registers */
static volatile uint8_t  *UBRRnL;
static volatile uint8_t  *UBRRnH;

/* UDR empty interrupt */
#define SET_UDRIE sethigh_1bit(*UCSRnB, UDRIEn)
#define CLR_UDRIE setlow_1bit(*UCSRnB, UDRIEn)

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

static void uart_Select(uint8_t uart_id)
{
    switch(uart_id)
    {
        case 1:
            UBRRnL = &UBRR1L;
            UBRRnH = &UBRR1H;
            UDRn   = &UDR1;
            UCSRnA = &UCSR1A;
            UCSRnB = &UCSR1B;
            UCSRnC = &UCSR1C;
            UART_ID = 1;
        break;

        case 2:
            UBRRnL = &UBRR2L;
            UBRRnH = &UBRR2H;
            UDRn   = &UDR2;
            UCSRnA = &UCSR2A;
            UCSRnB = &UCSR2B;
            UCSRnC = &UCSR2C;
            UART_ID = 2;
        break;

        case 3:
            UBRRnL = &UBRR3L;
            UBRRnH = &UBRR3H;
            UDRn   = &UDR3;
            UCSRnA = &UCSR3A;
            UCSRnB = &UCSR3B;
            UCSRnC = &UCSR3C;
            UART_ID = 3;
        break;

        /* default is zero */
        default:
            UBRRnL = &UBRR0L;
            UBRRnH = &UBRR0H;
            UDRn   = &UDR0;
            UCSRnA = &UCSR0A;
            UCSRnB = &UCSR0B;
            UCSRnC = &UCSR0C;
            UART_ID = 0;
        break;
    }
}


void uart_Init(uint8_t uart_id)
{
    uart_Select(uart_id);

    /* -- Set baud rates, refer to datasheet -- */
    // 19.2 kbps: UBRR = 51 at 16 MHz, 25 at 8 MHz, 11 at 3.6864 MHz
    *UBRRnH = (uint8_t)(UART_UBRR >> 8);
    *UBRRnL = (uint8_t)UART_UBRR;

    /* Flush Buffers */
    UART_RxTail = 0;
    UART_RxHead = 0;
    UART_TxTail = 0;
    UART_TxHead = 0;

    /* Set frame format: 8data, 1stop bit */
    *UCSRnC = (3<<UCSZn0);

    /* Enable receiver and transmitter, rx int */
    *UCSRnB = (1<<RXENn)|(1<<TXENn)|(1<<RXCIEn);
}


/* # Next received byte, or -1 if none is waiting */
int16_t uart_ReadByte(void)
{
    uint8_t tmptail;
    uint8_t data;

    if(UART_RxHead == UART_RxTail)
        return -1;

    /* Calculate buffer index */
    tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK;
    data = UART_RxBuffer[tmptail];
    /* Store new index */
    UART_RxTail = tmptail;

    return data;
}


void uart_SendByte(char data)
{
    uint8_t tmphead;

    /* Calculate buffer index */
    tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;
    /* Wait for free space in buffer */
    while ( tmphead == UART_TxTail )
    ;
    /* Store data in buffer */
    UART_TxBuffer[tmphead] = data;
    /* Store new index */
    UART_TxHead = tmphead;
    /* Enable UDRE interrupt */
    SET_UDRIE;
}


void uart_SendString(const char *str)
{
    while(*str)
    {
       uart_SendByte(*str);
       str++;
    }
}

void uart_SendInt(int x)
{
    char str[8];

    itoa(x, str, 10);
    uart_SendString(str);
}

/* ---------------------- */
/*  RX interrupt handler  */
/* ---------------------  */

/*  Reading UDRn clears the RXCn flag. Until it is read the RX interrupt keeps
    firing, so the byte is always read here and queued for the main loop. */
static inline void _ReceiveByte(void)
{
    uint8_t tmphead;
    uint8_t data = *UDRn;

    /* Calculate buffer index */
    tmphead = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;
    /* Drop the byte if the buffer is full */
    if ( tmphead == UART_RxTail )
        return;
    /* Store data in buffer */
    UART_RxBuffer[tmphead] = data;
    /* Store new index */
    UART_RxHead = tmphead;
}

/* alter as needed */

ISR(USART0_RX_vect)
{
    if(UART_ID == 0) _ReceiveByte(); else (void)UDR0;
}

ISR(USART1_RX_vect)
{
    if(UART_ID == 1) _ReceiveByte(); else (void)UDR1;
}

ISR(USART2_RX_vect)
{
    if(UART_ID == 2) _ReceiveByte(); else (void)UDR2;
}

ISR(USART3_RX_vect)
{
    if(UART_ID == 3) _ReceiveByte(); else (void)UDR3;
}

/* ---------------------- */
/*  TX interrupt handler  */
/* ---------------------- */

static inline void _TransmitByte(void)
{
    uint8_t UART_TxTail_tmp;
    UART_TxTail_tmp = UART_TxTail;

    /* Check if all data is transmitted */
    if ( UART_TxHead !=  UART_TxTail_tmp )
    {
        /* Calculate buffer index */
        UART_TxTail_tmp = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
        /* Store new index */
        UART_TxTail =  UART_TxTail_tmp;
        /* Start transmition */
        *UDRn = UART_TxBuffer[ UART_TxTail_tmp];
    }
    else
        /* Disable UDRE interrupt */
        CLR_UDRIE;
}

/*  Activated by SendByte() and is turned off when TX buffer is empty */

/* alter as needed */

ISR(USART0_UDRE_vect)
{
    if(UART_ID == 0) _TransmitByte();
}

ISR(USART1_UDRE_vect)
{
    if(UART_ID == 1) _TransmitByte();
}

ISR(USART2_UDRE_vect)
{
    if(UART_ID == 2) _TransmitByte();
}

ISR(USART3_UDRE_vect)
{
    if(UART_ID == 3) _TransmitByte();
}

/* --------------------- */
/*  Catch bad interrupt  */
/* --------------------- */

/* Ignore unexpected interrupts. (This used to toggle the pin 13 LED, but PB7 /
   pin 13 is OC1C, the output of servo C0, so the toggle had no effect.) */
EMPTY_INTERRUPT(BADISR_vect);
