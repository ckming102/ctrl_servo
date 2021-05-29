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

TIMER timer1;
TIMER timer4;

PWM pwm_grp[2];
volatile uint8_t pwm_select = 0;
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
        "PWM Level / Inc: %d / %d  LOW / IDLE / HIGH: %d / %d / %d  "
        "PWM Select: %c%d \n\r",
        pwm_grp[pwm_select].pwm_level[pwm_chn],
        pwm_grp[pwm_select].pwm_step[pwm_chn],
        pwm_grp[pwm_select].pwm_level_min[pwm_chn],
        pwm_grp[pwm_select].pwm_level_idle[pwm_chn],
        pwm_grp[pwm_select].pwm_level_max[pwm_chn],
        pwm_select_char,
        pwm_select
    );
    uart_SendString(str_buffer);
    return 0;
}

int cbk_inc_pwm_level(uint8_t argc, char **argv)
{
    return PWM_Inc(&pwm_grp[pwm_select], pwm_chn);
}

int cbk_dec_pwm_level(uint8_t argc, char **argv)
{
    return PWM_Dec(&pwm_grp[pwm_select], pwm_chn);
}

int cbk_idle_pwm_level(uint8_t argc, char **argv)
{
    return PWM_Idle(&pwm_grp[pwm_select], pwm_chn);
}

int cbk_mode(uint8_t argc, char **argv)
{
    if(argc < 2)
    {
        uart_SendString("Insufficient number of inputs\n\r");
        return 0;
    }
    
    if(strcmp(argv[1], "manual") == 0)
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
        for (i = 0; i < pwm_grp[pwm_select].pwm_level[pwm_chn] - PWM_LOW; i++) uart_SendByte('=');
        slider_pos = pwm_grp[pwm_select].pwm_level[pwm_chn] - PWM_LOW;
    }
    else if(strcmp(argv[1], "game") == 0)
    {
        /* change context */
        context = context_game;

        uart_SendString(
            "\n\r[GAME MODE]"
            " up/down [channel A], left/right [channel B], "
            " W/S [channel C]; enter to exit\n\r"
        );

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
    else if(strcmp(argv[1], "0") == 0)
    {
        pwm_select = 0;
        uart_SendString("PWM Group 0 selected\n\r");
    }
    else if(strcmp(argv[1], "1") == 0)
    {
        pwm_select = 1;
        uart_SendString("PWM Group 1 selected\n\r");
    }
    else uart_SendString("Unknown channel / group\n\r");

    return 0;
}

int cbk_pwm_frequency(uint8_t argc, char **argv)
{
    // The calculation is completely off. Need to finx in pwm.c
    PWM_FrequencyHz(&pwm_grp[pwm_select], str_temp);
    sprintf(str_buffer,"PWM Frequency: %s\n\r",str_temp);
    uart_SendString(str_buffer);
    return 0;
}

int cbk_duty_cycle(uint8_t argc, char **argv)
{
    // The calculation is completely off. Need to finx in pwm.c
    PWM_DutyCycle(&pwm_grp[pwm_select], pwm_chn, str_temp);
    sprintf(str_buffer,"Duty Cycle: %s\n\r",str_temp);
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
/*  Manual mode event handler  */
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
                status.bracket = FALSE;
                if((slider_pos < PWM_STEPS_INT) &&
                    (pwm_grp[pwm_select].pwm_level[pwm_chn] < pwm_grp[pwm_select].pwm_level_max[pwm_chn])
                  )
                {
                    PWM_Inc(&pwm_grp[pwm_select], pwm_chn);
                    uart_SendByte('=');
                    slider_pos++;
                }
            }
        break;

        /* down key */
        case 'B':
            if(status.bracket)
            {
                status.bracket = FALSE;
                if((slider_pos > 0) && 
                    (pwm_grp[pwm_select].pwm_level[pwm_chn] > pwm_grp[pwm_select].pwm_level_min[pwm_chn])
                )
                {
                    PWM_Dec(&pwm_grp[pwm_select], pwm_chn);
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

/* --------------------------- */
/*  Game mode event handler    */
/* --------------------------- */
void game_Keypress()
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

        /* --- Channel A0 / B1-- */

        /* right key */
        case 'C':
            if(status.bracket)
            {
                PWM_Inc(&pwm_grp[0], chn_A);
                status.bracket = FALSE;
            }
        break;

        case 'D':
            /* left key */
            if(status.bracket)
            {
                PWM_Dec(&pwm_grp[0], chn_A);
                status.bracket = FALSE;
            } else
            /* D key */
            {
                PWM_Dec(&pwm_grp[1], chn_B);
            }

        break;

        /* --- Channel B0 /B1-- */

        case 'A':
            /* up key */
            if(status.bracket)
            {
                PWM_Inc(&pwm_grp[0], chn_B);
                status.bracket = FALSE;
            } else 
            /* A key */
            {
                PWM_Inc(&pwm_grp[1], chn_B);
            }
        break;

        /* down key */
        case 'B':
            if(status.bracket)
            {
                PWM_Dec(&pwm_grp[0], chn_B);
                status.bracket = FALSE;
            }
        break;

        /* --- Channel C0 -- */

        /* W key */
        case 'W':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[0], chn_C);
            }
        break;

        /* S key */
        case 'S':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[0], chn_C);
            }
        break;

        /* w key */
        case 'w':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[0], chn_C);
            }
        break;

        /* s key */
        case 's':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[0], chn_C);
            }
        break;

        /* --- Channel A1 -- */

        /* F key */
        case 'F':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[1], chn_A);
            }
        break;

        /* R key */
        case 'R':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[1], chn_A);
            }
        break;

        /* r key */
        case 'r':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[1], chn_A);
            }
        break;

        /* f key */
        case 'f':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[1], chn_A);
            }
        break;

        /* --- Channel B1 -- */

        /* a key */
        case 'a':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[1], chn_B);
            }
        break;

        /* d key */
        case 'd':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[1], chn_B);
            }
        break;

        /* --- Channel C1 -- */

        /* > key */
        case '>':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[1], chn_C);
            }
        break;

        /* < key */
        case '<':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[1], chn_C);
            }
        break;

        /* . key */
        case '.':
            if(!status.bracket)
            {
                PWM_Inc(&pwm_grp[1], chn_C);
            }
        break;

        /* , key */
        case ',':
            if(!status.bracket)
            {
                PWM_Dec(&pwm_grp[1], chn_C);
            }
        break;

        /* enter key */
        case 13:
            context = context_cli;
            status.esc_char = FALSE;
            status.bracket = FALSE;
            uart_SendString("Exit game mode\n\r");
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

    /* Initialize timer objects */
    TIMER_Init(&timer1, 1);
    TIMER_Init(&timer4, 4);

    PWM_TimerConfig(&pwm_grp[0], &timer1, SERVO_PWM);
    PWM_TimerConfig(&pwm_grp[1], &timer4, SERVO_PWM);

    /* pwm config values (max, min, idle, step) */

    /* Configured for -90 to 90 deg with increments of 3 deg */
    uint16_t pwm_config_A0[4] = {72, 14, 72, PWM_INC};
    PWM_PwmConfig(&pwm_grp[0],pwm_config_A0, chn_A);

    uint16_t pwm_config_B0[4] = {50, 38, 39, PWM_INC};
    PWM_PwmConfig(&pwm_grp[0],pwm_config_B0, chn_B);

    uint16_t pwm_config_C0[4] = {65, 55, 55, PWM_INC};
    PWM_PwmConfig(&pwm_grp[0],pwm_config_C0, chn_C);

    uint16_t pwm_config_A1[4] = {65, 55, 55, PWM_INC};
    PWM_PwmConfig(&pwm_grp[1],pwm_config_A1, chn_A);

    uint16_t pwm_config_B1[4] = {72, 14, 40, PWM_INC};
    PWM_PwmConfig(&pwm_grp[1],pwm_config_B1, chn_B);

    uint16_t pwm_config_C1[4] = {72, 51, 72, PWM_INC};
    PWM_PwmConfig(&pwm_grp[1],pwm_config_C1, chn_C);


    /* pick PWM11 [PIN B] */
    pwm_select = 0;
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

            case context_game:
                if(status.rx_int == TRUE) game_Keypress();
            break;

            default:
            break;
        }
    }
}
