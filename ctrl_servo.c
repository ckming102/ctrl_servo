/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Simple PWM controlling a DC motor example through serial terminal connected
  via UART.

- Connect to uart0's rx and tx of the atmega2560 board. Interface with USB to TTL
  module and GTKTerm serial program. Configured for 16Mhz and 19.2 kbps.

 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "global.h"
#include "uart.h"
#include "cmd.h"

/* ------------- */
/*  PWM control  */
/* ------------- */

    #define PWM_MAX 0x0271
    #define PWM_STEPS 0x007D
    #define PWM_INC 0x0001

    #define PWM_HIGH 72
    #define PWM_LOW 14
    #define PWM_STEPS_INT 58

    volatile uint16_t *pwm_select;
    volatile char pwm_select_char;
    volatile unsigned char *pwm_level;

    unsigned char slider_pos;

/* -------------- */
/*  Terminal I/O  */
/* -------------- */
    char str_buffer[80];

/* ---------------------------------- */
/*  Registered commands and callbacks */
/* ---------------------------------- */

    #define CMD_LIST_LEN 6 // exact fixed number of commands at runtime

    char str_help[] = 
        "\n\r# List of commands\n\r\n\r"
        "help : Displays this list\n\r"
        "status : output current PWM level \n\r"
        "inc : increase PWM level by one unit \n\r"
        "dec : decrease PWM level by one unit \n\r"
        "mode : change mode to 'manual' \n\r"
        "select: change PWM channel to 'A,B,C' \n\r"
        "\n\r";

    char *cmd_name[CMD_LIST_LEN] = {
        "help", "status", "inc", "dec", "mode", "select"
    };

    int cbk_help(unsigned char argc, char **argv)
    {
        uart_SendString(str_help);
        return 0;
    }

    int cbk_print_pwm_level(unsigned char argc, char **argv)
    {
        sprintf(
            str_buffer,
            "PWM Level: %d / %d  LOW / HIGH: %d / %d  "
            "PWM Select: %c \n\r",
            *pwm_level, PWM_STEPS, PWM_LOW, PWM_HIGH, pwm_select_char
        );
        uart_SendString(str_buffer);
        return 0;
    }

    int cbk_inc_pwm_level(unsigned char argc, char **argv)
    {
        if(*pwm_level < PWM_HIGH)
        {
            *pwm_select += PWM_INC;
            (*pwm_level)++;
        }
        return 0;
    }

    int cbk_dec_pwm_level(unsigned char argc, char **argv)
    {
        if(*pwm_level > PWM_LOW)
        {
            *pwm_select -= PWM_INC;
            (*pwm_level)--;
        }
        return 0;
    }

    int cbk_mode(unsigned char argc, char **argv)
    {
        if(argc < 2)
        {
            uart_SendString("Insufficient number of inputs\n\r");
        }
        else if(strcmp(argv[1], "manual") == 0)
        {
            unsigned char i;

            /* change context */
            context = context_manual;

            uart_SendString(
                "\n\r[MANUAL MODE]"
                " use up and down keys to change angle; enter to exit\n\r"
            );

            for (i = 0; i < PWM_STEPS_INT + 2; i++) uart_SendByte(' ');
            uart_SendByte(']');
            uart_SendString("\r[");
            for (i = 0; i < *pwm_level - PWM_LOW; i++) uart_SendByte('=');
            slider_pos = *pwm_level - PWM_LOW;
        }
        else uart_SendString("Unknown mode\n\r");
        return 0;
    }

    int cbk_select(unsigned char argc, char **argv)
    {
        if(argc < 2)
        {
            uart_SendString("Insufficient number of inputs\n\r");
        }
        else if(strcmp(argv[1], "A") == 0)
        {
            pwm_select = &OCR1A;
            pwm_level = &pwm_A1_level;
            pwm_select_char = 'A';
            uart_SendString("Channel A selected\n\r");
        }
        else if(strcmp(argv[1], "B") == 0)
        {
            pwm_select = &OCR1B;
            pwm_level = &pwm_B1_level;
            pwm_select_char = 'B';
            uart_SendString("Channel B selected\n\r");
        }
        else if(strcmp(argv[1], "C") == 0)
        {
            pwm_select = &OCR1C;
            pwm_level = &pwm_C1_level;
            pwm_select_char = 'C';
            uart_SendString("Channel C selected\n\r");
        }
        else uart_SendString("Unknown channel\n\r");

        return 0;
    }

    int (*cmd_list[CMD_LIST_LEN])(unsigned char, char **) = {
        &cbk_help,
        &cbk_print_pwm_level, &cbk_inc_pwm_level, &cbk_dec_pwm_level,
        &cbk_mode, &cbk_select
    };

/* --------------------------- */
/*  Manual moode event handler */
/* --------------------------- */
    void manual_Keypress()
    {
        /* copy-in byte */
        char data = UDR0;

        switch(data)
        {
            /* escape character */
            case '\x1b':
                status.esc_char = TRUE;
            break;

            case '[':
                status.bracket = (status.esc_char == TRUE) ? TRUE : FALSE; 
                status.esc_char = FALSE;
            break;

            /* up key */
            case 'A':
                if(status.bracket)
                {
                    cbk_inc_pwm_level(1, argv);
                    status.bracket = FALSE;
                    if(slider_pos < PWM_STEPS_INT)
                    {
                        uart_SendByte('=');
                        slider_pos++;
                    }
                }
            break;

            /* down key */
            case 'B':
                if(status.bracket)
                {
                    cbk_dec_pwm_level(1, argv);
                    status.bracket = FALSE;
                    if(slider_pos > 0)
                    {
                        uart_SendByte('\b');
                        uart_SendByte(' ');
                        uart_SendByte('\b');
                        slider_pos--;
                    }
                }
            break;

            /* enter key */
            case 13:
                context = context_cli;
                status.esc_char = FALSE;
                status.bracket = FALSE;
                uart_SendString("\n\r");
                uart_FlushRxBuffer();
            break;

            default:
            break;
        }

        status.rx_int = FALSE;
    }

/* ---------------- */
/*  Initialization  */
/* ---------------- */

    void InitUART()
    {
        /* uart */
        uart_Init();
        sei();

        /* blinker */
        sethigh_1bit(DDRB, DDB7);
    }

    void InitPWM()
    {

        /*   16 bit timer-counter TCNT0 with output to */
        /*   OC1C = pin PB7 [PWM 13]                   */
        /*   OC1B = pin PB6 [PWM 12]                   */
        /*   OC1A = pin PB5 [PWM 11]                   */
        sethigh_1bit(DDRB, PB7);
        sethigh_1bit(DDRB, PB6);
        sethigh_1bit(DDRB, PB5);

        /* PWM modes in pg. 145 of ATmega2560 data-sheet */
        set_1bit(TCCR1B, WGM13, 1);
        set_1bit(TCCR1B, WGM12, 0);
        set_1bit(TCCR1A, WGM11, 0);
        set_1bit(TCCR1A, WGM10, 0);

        /* inverted [11] /uninverted mode [10] */
        set_1bit(TCCR1A, COM1A1, 1);
        set_1bit(TCCR1A, COM1A0, 0);

        set_1bit(TCCR1A, COM1B1, 1);
        set_1bit(TCCR1A, COM1B0, 0);

        set_1bit(TCCR1A, COM1C1, 1);
        set_1bit(TCCR1A, COM1C0, 0);

        /* choose clock source and prescalar */
        /* prescalar     | CS12:0    */
        /* --------------|-----------*/
        /* 1             | 001       */
        /* 8             | 010       */
        /* 64            | 011       */
        /* 256           | 100       */
        /* 1024          | 101       */
        set_1bit(TCCR1B, CS12, 1);
        set_1bit(TCCR1B, CS11, 0);
        set_1bit(TCCR1B, CS10, 0);

        /* Configured for -90 to 90 deg with increments of 3 deg */

        /* set TOP */
        ICR1 = PWM_MAX;

        /* do a sweep to init and reset angle */
        OCR1C = PWM_INC * PWM_LOW;
        OCR1B = PWM_INC * PWM_LOW;
        OCR1A = PWM_INC * PWM_LOW;

        int level;
        for(level=PWM_LOW; level <= PWM_HIGH; level++)
        {
            OCR1C += PWM_INC;
            OCR1B += PWM_INC;
            OCR1A += PWM_INC;
            _delay_ms(25);
        }
        for(level=PWM_HIGH; level >= PWM_LOW; level--)
        {
            OCR1C -= PWM_INC;
            OCR1B -= PWM_INC;
            OCR1A -= PWM_INC;
            _delay_ms(25);
        }

        pwm_C1_level = PWM_LOW;
        pwm_B1_level = PWM_LOW;
        pwm_A1_level = PWM_LOW;

        OCR1C = pwm_C1_level * PWM_INC;
        OCR1B = pwm_B1_level * PWM_INC;
        OCR1A = pwm_A1_level * PWM_INC;

       /* start from zero */
        TCNT1 = 0x0000;

        /* pick PWM12 [PIN 6] */
        pwm_select = &OCR1B;
        pwm_level = &pwm_B1_level;
        pwm_select_char = 'B';
    }

    void InitState()
    {
        /* command state and errors */
        context = context_cli;

        status.cmd_check = FALSE;
        status.cmd_executed = FALSE;
        status.rx_int = FALSE;
        status.esc_char = FALSE;
        status.bracket = FALSE;

        err_no = 0;
        argv[0] = UART_RxBuffer;
    }


/* ----------- */
/*  Main Loop  */
/* ----------- */
int main()
{
    /* hardware */
    InitUART();
    InitPWM();

    /* software */
    InitState();

    /* superloop */
    while(1)
    {
        switch(context)
        {
            case context_cli:
                if(status.rx_int == TRUE) cli_Keypress();
                cli_ParseCommand(CMD_LIST_LEN);
            break;

            case context_manual:
                if(status.rx_int == TRUE) manual_Keypress();
            break;

            default:
            break;
        }
    }
}
