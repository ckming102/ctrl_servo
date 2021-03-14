/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Simple PWM controlling a DC motor example through serial terminal connected
  via UART. Based on `simple_uart` and `simple_pwm` examples.

- Connect to uart0's rx and tx of the atmega2560 board. Interface with USB to TTL
  module and GTKTerm serial program. Configured for 16Mhz and 19.2 kbps.

- PWM output to pin PB6 a.k.a. PWM12.

 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#define F_CPU 16000000 // CPU freq 16 MHz
#include <avr/io.h> // io library
#include <avr/interrupt.h> // interupt library
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
    #define PWM_INC 0x0005

    #define PWM_HIGH 12
    #define PWM_LOW 6
    #define PWM_STEPS_INT 6

    // #define PWM_HIGH 20
    // #define PWM_LOW 0
    // #define PWM_STEPS_INT 6

    unsigned char pwm_level;
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
        "set : set PWM level with first argument \n\r"
        "mode : change mode to 'manual' \n\r"
        "\n\r";

    char *cmd_name[CMD_LIST_LEN] = {
        "help", "status", "inc", "dec", "set", "mode"
    };

    int cbk_help(unsigned char argc, char **argv)
    {
        uart_SendString(str_help);
        return 0;
    }

    int cbk_print_pwm_level(unsigned char argc, char **argv)
    {
        sprintf(str_buffer, "PWM Level: %d / %d\n\r", pwm_level, PWM_STEPS);
        uart_SendString(str_buffer);
        return 0;
    }

    int cbk_inc_pwm_level(unsigned char argc, char **argv)
    {
        // if(pwm_level < PWM_STEPS)
        if(pwm_level < PWM_HIGH)
        {
            OCR1B += PWM_INC;
            pwm_level++;
        }
        return 0;
    }

    int cbk_dec_pwm_level(unsigned char argc, char **argv)
    {
        // if(pwm_level > 0)
        if(pwm_level > PWM_LOW)
        {
            OCR1B -= PWM_INC;
            pwm_level--;
        }
        return 0;
    }

    int cbk_set_pwm_level(unsigned char argc, char **argv)
    {
        if(argc < 2)
        {
            uart_SendString("Insufficient number of inputs\n\r");
        }
        else
        {
            int val = atoi(argv[1]);
            val = val > PWM_STEPS ? PWM_STEPS : val;
            val = val < 0 ? 0 : val;
            pwm_level = val;
            OCR1B = val * PWM_INC;
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
                " use up and down keys to change level; enter to exit\n\r"
            );

            // for (i = 0; i < PWM_STEPS + 2; i++) uart_SendByte(' ');
            for (i = 0; i < PWM_STEPS_INT + 2; i++) uart_SendByte(' ');
            uart_SendByte(']');
            uart_SendString("\r[");
            for (i = 0; i < pwm_level - PWM_LOW; i++) uart_SendByte('=');
            slider_pos = pwm_level - PWM_LOW;
        }
        else uart_SendString("unknown mode\n\r");
        return 0;
    }

    int (*cmd_list[CMD_LIST_LEN])(unsigned char, char **) = {
        &cbk_help,
        &cbk_print_pwm_level, &cbk_inc_pwm_level, &cbk_dec_pwm_level, &cbk_set_pwm_level,
        &cbk_mode
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
                    // if(slider_pos < PWM_STEPS)
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
        /*  set PORTB to output */
        /* # 16 bit timer-counter TCNT0 with output to OC1B = pin PB6 [PWM 12]*/
        sethigh_1bit(DDRB, PB5);
        sethigh_1bit(DDRB, PB6);
        sethigh_1bit(DDRB, PB7);

        /* PWM modes in pg. 145 of ATmega2560 data-sheet */
        set_1bit(TCCR1B, WGM13, 1);
        set_1bit(TCCR1B, WGM12, 0);
        set_1bit(TCCR1A, WGM11, 0);
        set_1bit(TCCR1A, WGM10, 1);

        /* inverted [11] /uninverted mode [10] */
        set_1bit(TCCR1A, COM1B1, 1);
        set_1bit(TCCR1A, COM1B0, 0);

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

        /* set TOP */
        OCR1A = PWM_MAX;

        /* init duty cycle(%) fraction of TOP */
        // pwm_level = 0;
        pwm_level = 0;
        OCR1B = pwm_level * PWM_INC;

        _delay_ms(500);

        pwm_level = PWM_LOW;
        OCR1B = pwm_level * PWM_INC;

       /* start from zero */
        TCNT1 = 0x0000;
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
        uart_SendInt(ICR1);
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
