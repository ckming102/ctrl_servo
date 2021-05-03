/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Simple PWM controlling a DC motor example through serial terminal connected
  via UART.

- Connect to uart1's rx and tx of the atmega2560 board. Interface with USB to TTL
  module and GTKTerm serial program. Configured for 16Mhz and 19.2 kbps.

# TODO:
- Decrease pre-scalar to get finest decent angular resolution for the MG996R,
  1.8 deg I believe. 0.9 deg it as the dead-band.

- Encapsulate each timer into its own PWM control class; distinguish the 8-bit and
  16 bit. Use offsets for the pointer locations! OCR[0-3][A-C]-> OCRnX etc.

- Encapsulate slider / progress bar. Use function pointer for the uart_SendString

- Generalize manual mode to switch interactively between channels

- Decide on scheduling protocol and basic path planning
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
#include "timer.h"
#include "pwm.h"

/* ------------- */
/*  PWM control  */
/* ------------- */

#define PWM_STEPS 0x007D
#define PWM_MAX 0x0271
#define PWM_INC 0x0001

#define PWM_HIGH 72
#define PWM_LOW 14
#define PWM_IDLE 50
#define PWM_STEPS_INT 58

TIMER timer;
PWM pwm;

volatile PWM_Channel pwm_chn;
volatile char pwm_select_char;

uint8_t slider_pos;

/* -------------- */
/*  Terminal I/O  */
/* -------------- */
char str_buffer[80];

// for short temp strings
char str_temp[20];

/* ---------------------------------- */
/*  Registered commands and callbacks */
/* ---------------------------------- */

#define CMD_LIST_LEN 9 // exact fixed number of commands at runtime

char str_help[] = 
    "\n\r# List of commands\n\r\n\r"
    "help : Displays this list\n\r"
    "status : output current PWM level \n\r"
    "inc : increase PWM level by one unit \n\r"
    "dec : decrease PWM level by one unit \n\r"
    "idle : set PWM level to idle \n\r"
    "mode : change mode to 'manual' \n\r"
    "select: change PWM channel to 'A,B,C' \n\r"
    "frequency : Displays the pwm frequency in Hz\n\r"
    "duty_cycle : Displays the duty cycle of currently selected channel\n\r"
    "\n\r";

char *cmd_name[CMD_LIST_LEN] = {
    "help", "status", "inc", "dec", "idle", "mode", "select", "frequency", "duty_cycle"
};

int cbk_help(uint8_t argc, char **argv)
{
    uart_SendString(str_help);
    return 0;
}

int cbk_print_pwm_level(uint8_t argc, char **argv)
{
    sprintf(
        str_buffer,
        "PWM Level: %d / %d  LOW / IDLE / HIGH: %d / %d / %d  "
        "PWM Select: %c \n\r",
        pwm.pwm_level[pwm_chn],
        pwm.pwm_step[pwm_chn],
        pwm.pwm_level_min[pwm_chn],
        pwm.pwm_level_idle[pwm_chn],
        pwm.pwm_level_max[pwm_chn],
        pwm_select_char
    );
    uart_SendString(str_buffer);
    return 0;
}

int cbk_inc_pwm_level(uint8_t argc, char **argv)
{
    return PWM_Inc(&pwm, pwm_chn);
}

int cbk_dec_pwm_level(uint8_t argc, char **argv)
{
    return PWM_Dec(&pwm, pwm_chn);
}

int cbk_idle_pwm_level(uint8_t argc, char **argv)
{
    return PWM_Idle(&pwm, pwm_chn);
}

int cbk_mode(uint8_t argc, char **argv)
{
    if(argc < 2)
    {
        uart_SendString("Insufficient number of inputs\n\r");
    }
    else if(strcmp(argv[1], "manual") == 0)
    {
        uint8_t i;

        /* change context */
        context = context_manual;

        uart_SendString(
            "\n\r[MANUAL MODE]"
            " use up and down keys to change angle; enter to exit\n\r"
        );

        for (i = 0; i < PWM_STEPS_INT + 2; i++) uart_SendByte(' ');
        uart_SendByte(']');
        uart_SendString("\r[");
        for (i = 0; i < pwm.pwm_level[pwm_chn] - PWM_LOW; i++) uart_SendByte('=');
        slider_pos = pwm.pwm_level[pwm_chn] - PWM_LOW;
    }
    else uart_SendString("Unknown mode\n\r");
    return 0;
}

int cbk_select(uint8_t argc, char **argv)
{
    if(argc < 2)
    {
        uart_SendString("Insufficient number of inputs\n\r");
    }
    else if(strcmp(argv[1], "A") == 0)
    {
        pwm_select_char = 'A';
        pwm_chn = chn_A;
        uart_SendString("Channel A selected\n\r");
    }
    else if(strcmp(argv[1], "B") == 0)
    {
        pwm_select_char = 'B';
        pwm_chn = chn_B;
        uart_SendString("Channel B selected\n\r");
    }
    else if(strcmp(argv[1], "C") == 0)
    {
        pwm_select_char = 'C';
        pwm_chn = chn_C;
        uart_SendString("Channel C selected\n\r");
    }
    else uart_SendString("Unknown channel\n\r");

    return 0;
}

int cbk_pwm_frequency(uint8_t argc, char **argv)
{
    PWM_FrequencyHz(&pwm, str_temp);
    sprintf(str_buffer,"PWM Frequency: %s",str_temp);
    uart_SendString(str_buffer);
    return 0;
}

int cbk_duty_cycle(uint8_t argc, char **argv)
{
    PWM_DutyCycle(&pwm, pwm_chn, str_temp);
    sprintf(str_buffer,"Duty Cycle: %s",str_temp);
    uart_SendString(str_buffer);
    return 0;
}


int (*cmd_list[CMD_LIST_LEN])(uint8_t, char **) = {
    &cbk_help,
    &cbk_print_pwm_level, &cbk_inc_pwm_level, &cbk_dec_pwm_level,
    &cbk_idle_pwm_level,
    &cbk_mode, &cbk_select, &cbk_pwm_frequency, &cbk_duty_cycle
};

/* --------------------------- */
/*  Manual moode event handler */
/* --------------------------- */
void manual_Keypress()
{
    /* copy-in byte */
    char data = *UART_UDRn;

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
                PWM_Inc(&pwm, pwm_chn);
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
                PWM_Dec(&pwm, pwm_chn);
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
    uart_Init(1);
    sei();

    /* blinker */
    sethigh_1bit(DDRB, DDB7);
}

void InitPWM()
{

    /* Initialize timer object */
    TIMER_Init(&timer, 1);

    /* Configured for -90 to 90 deg with increments of 3 deg */
    PWM_TimerConfig(&pwm, &timer, SERVO_PWM);

    /* (max, min, idle, step) */
    uint16_t pwm_config[4] = {PWM_HIGH, PWM_LOW, PWM_IDLE, PWM_STEPS};
    PWM_PwmConfig(&pwm,pwm_config, chn_A);
    PWM_PwmConfig(&pwm,pwm_config, chn_B);
    PWM_PwmConfig(&pwm,pwm_config, chn_C);

    /* pick PWM12 [PIN 6] */
    pwm_select_char = 'B';
    pwm_chn = chn_B;
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
