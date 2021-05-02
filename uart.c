/*==============================================================================
  Function declarations and data structures for the UART
 =============================================================================*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "global.h"
#include "uart.h"

/* ------------------ */
/*  Extern variables  */
/* ------------------ */

/* RX buffer and pointer for uart */
char UART_RxBuffer[UART_RX_BUFFER_SIZE];
uint8_t UART_RxPtr;

uint8_t UART_ID;

/* data register */
volatile uint8_t  *UDRn;

/* control and status registers */
volatile uint8_t *UCSRnA;
volatile uint8_t *UCSRnB;
volatile uint8_t *UCSRnC;

/* ------------------ */
/*  Static variables  */
/* ------------------ */

/* TX buffer and head/tail pointers (idx counters) */
static char UART_TxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;

/* ===================== */
/* Pointers to Registers */
/* ===================== */

/* baud rate registers */
static volatile uint16_t *UBRRn;
static volatile uint8_t  *UBRRnL;
static volatile uint8_t  *UBRRnH;

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

void uart_Select(uint8_t uart_id)
{
    switch(uart_id)
    {
        case 1:
            UBRRn  = &UBRR1;
            UBRRnL = &UBRR1L;
            UBRRnH = &UBRR1H;
            UDRn   = &UDR1;
            UCSRnA = &UCSR1A;
            UCSRnB = &UCSR1B;
            UCSRnC = &UCSR1C;
            UART_ID = 1;
        break;

        case 2:
            UBRRn  = &UBRR2;
            UBRRnL = &UBRR2L;
            UBRRnH = &UBRR2H;
            UDRn   = &UDR2;
            UCSRnA = &UCSR2A;
            UCSRnB = &UCSR2B;
            UCSRnC = &UCSR2C;
            UART_ID = 2;
        break;

        case 3:
            UBRRn  = &UBRR3;
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
            UBRRn  = &UBRR0;
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
    // 19.2 kbps trasfer speed running at 16 MHz.
    #define BAUD 51
    // 19.2 kbps trasfer speed running at 8 MHz.
    //#define BAUD 25
    // 19.2 kbps trasfer speed running at 3.6864 MHz.
    // #define BAUD 11

    *UBRRnH = (uint8_t)(BAUD>>8);
    *UBRRnL = (uint8_t)BAUD;

    /* Enable receiver and transmitter, rx int */
    *UCSRnB = (1<<RXENn)|(1<<TXENn)|(1<<RXCIEn)|(1<<TXCIEn);
 
    /* Set frame format: 8data, 1stop bit */
    *UCSRnC = (3<<UCSZn0);

    /* Flush Buffers */
    UART_RxPtr = 0;
    UART_RxBuffer[0] = '\0';
    UART_TxTail = 0;
    UART_TxHead = 0;
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


void uart_SendString(char Str[])
{
    uint8_t n = 0;
    while(Str[n])
       uart_SendByte(Str[n++]);
}

void uart_SendInt(int x)
{
    static const char dec[] = "0123456789";
    unsigned int div_val = 10000;

    if (x < 0)
    {
        x = - x;
        uart_SendByte('-');
    }

    while (div_val > 1 && div_val > x)
        div_val /= 10;

    do
    {
        uart_SendByte (dec[x / div_val]);
        x %= div_val;
        div_val /= 10;
    }
    while(div_val);
}

void uart_FlushRxBuffer()
{
    UART_RxPtr = 0;
    UART_RxBuffer[0] = '\0';
}

/* ---------------------- */
/*  RX interrupt handler  */
/* ---------------------  */

/* alter as needed */

ISR(USART0_RX_vect)
{
    if(UART_ID == 0) status.rx_int = TRUE;
}

ISR(USART1_RX_vect)
{
    if(UART_ID == 1) status.rx_int = TRUE;
}

ISR(USART2_RX_vect)
{
    if(UART_ID == 2) status.rx_int = TRUE;
}

ISR(USART3_RX_vect)
{
    if(UART_ID == 3) status.rx_int = TRUE;
}

/* ---------------------- */
/*  TX interrupt handler  */
/* ---------------------- */

void _TransmitByte()
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

/*  Activated when TX is complete */
EMPTY_INTERRUPT(USART0_TX_vect);
EMPTY_INTERRUPT(USART1_TX_vect);
EMPTY_INTERRUPT(USART2_TX_vect);
EMPTY_INTERRUPT(USART3_TX_vect);

/* --------------------- */
/*  Catch bad interrupt  */
/* --------------------- */

ISR(BADISR_vect)
{
    /* flip blink state */
    flip_1bit(PORTB,DDB7);
}