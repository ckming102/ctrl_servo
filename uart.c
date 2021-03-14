/*==============================================================================
  Function declarations and data structures for the UART
 =============================================================================*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "global.h"
#include "uart.h"


/* ------------------ */
/*  Static variables  */
/* ------------------ */

    /* TX buffer and head/tail pointers (idx counters) */
    static char UART_TxBuffer[UART_TX_BUFFER_SIZE];
    static volatile unsigned char UART_TxHead;
    static volatile unsigned char UART_TxTail;

/* ------------------ */
/*  Extern variables  */
/* ------------------ */

    /* RX buffer and pointer for uart */
    char UART_RxBuffer[UART_RX_BUFFER_SIZE];
    unsigned char UART_RxPtr;

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

    void uart_Init(void)
    {
        /* -- Set baud rates, refer to datasheet -- */
        // 19.2 kbps trasfer speed running at 16 MHz.
        #define BAUD 51
        // 19.2 kbps trasfer speed running at 8 MHz.
        //#define BAUD 25
        // 19.2 kbps trasfer speed running at 3.6864 MHz.
        // #define BAUD 11

        UBRR0H = (unsigned char)(BAUD>>8);
        UBRR0L = (unsigned char)BAUD;

        /* Enable receiver and transmitter, rx int */
        UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0);
     
        /* Set frame format: 8data, 1stop bit */
        UCSR0C = (3<<UCSZ00);

        /* Flush Buffers */
        UART_RxPtr = 0;
        UART_RxBuffer[0] = '\0';
        UART_TxTail = 0;
        UART_TxHead = 0;
    }


    void uart_SendByte(char data)
    {
        unsigned char tmphead;

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
        unsigned char n = 0;
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

    ISR(USART0_RX_vect)
    {
        status.rx_int = TRUE;
    }

/* ---------------------- */
/*  TX interrupt handler  */
/* ---------------------- */

    /*  Activated by SendByte() and is turned off when TX buffer is empty */
    ISR(USART0_UDRE_vect)
    {
        unsigned char UART_TxTail_tmp;
        UART_TxTail_tmp = UART_TxTail;

        /* Check if all data is transmitted */
        if ( UART_TxHead !=  UART_TxTail_tmp )
        {
            /* Calculate buffer index */
            UART_TxTail_tmp = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
            /* Store new index */
            UART_TxTail =  UART_TxTail_tmp;
            /* Start transmition */
            UDR0= UART_TxBuffer[ UART_TxTail_tmp];
        }
        else
            /* Disable UDRE interrupt */
            CLR_UDRIE;
    }

    /*  Activated when TX is complete */
    EMPTY_INTERRUPT(USART0_TX_vect);

/* --------------------- */
/*  Catch bad interrupt  */
/* --------------------- */

    ISR(BADISR_vect)
    {
        /* flip blink state */
        flip_1bit(PORTB,DDB7);
    }