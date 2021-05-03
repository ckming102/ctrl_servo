/*==============================================================================
  Function declarations and data structures for 16 bit timers
 =============================================================================*/
#include <stdio.h>
#include "global.h"
#include "timer.h"
#include "pwm.h"
#include <util/delay.h> 

/* ------------------ */
/*  Extern variables  */
/* ------------------ */

/* ------------------ */
/*  Static variables  */
/* ------------------ */

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

/* # Set timer configuration

  Setup timer for phase-freq correct PWM output. Check that output frequency is
  20Mhz. Details of modes in pg. 145 of ATmega2560 data-sheet and Arduino pinout.

  Parameters
  ----------
  prescalar: size of applied prescalar; see table below
  inverted : 1 for inverted and 0 for un-inverted
*/
void PWM_TimerConfig(
    PWM *pwm,
    TIMER *timer,
    uint8_t prescalar,
    uint8_t inverted,
    uint16_t counter_max
)
{
    /* copy in timer and register addresses */
    pwm->timer = timer;
    pwm->OCRnx[chn_A] = timer->OCRnA;
    pwm->OCRnx[chn_B] = timer->OCRnB;
    pwm->OCRnx[chn_C] = timer->OCRnC;

    /* Set to PWM mode [TCCRnA, TCCRnB] */

    // set to PWM mode to all 3 outputs
    set_1bit(*(pwm->timer->TCCRnB), WGMn3, 1);
    set_1bit(*(pwm->timer->TCCRnB), WGMn2, 0);
    set_1bit(*(pwm->timer->TCCRnA), WGMn1, 0);
    set_1bit(*(pwm->timer->TCCRnA), WGMn0, 0);

    /* Inverted(11) vs. Uninverted(10) [TCCRnA] */
    set_1bit(*(pwm->timer->TCCRnA), COMnA1, 1);
    set_1bit(*(pwm->timer->TCCRnA), COMnA0, -inverted);

    set_1bit(*(pwm->timer->TCCRnA), COMnB1, 1);
    set_1bit(*(pwm->timer->TCCRnA), COMnB0, -inverted);

    set_1bit(*(pwm->timer->TCCRnA), COMnC1, 1);
    set_1bit(*(pwm->timer->TCCRnA), COMnC0, -inverted);

    /* Clock source and prescalar [TCCRnB] */
    /*
        prescalar     | CSn2:0       
        --------------|--------------
        1    = 0x0001 | 001  = 0x01  
        8    = 0x0010 | 010  = 0x02  
        64   = 0x0040 | 011  = 0x04  
        256  = 0x0100 | 100  = 0x08  
        1024 = 0x0400 | 101  = 0x10  
    */
    set_1bit_hex(*(pwm->timer->TCCRnB), CSn2, prescalar);
    set_1bit_hex(*(pwm->timer->TCCRnB), CSn1, prescalar);
    set_1bit_hex(*(pwm->timer->TCCRnB), CSn0, prescalar);
    switch(prescalar)
    {
        case 1:
        pwm->prescalar = 1;
        break;

        case 2:
        pwm->prescalar = 8;
        break;

        case 4:
        pwm->prescalar = 64;
        break;

        case 8:
        pwm->prescalar = 256;
        break;

        case 16:
        pwm->prescalar = 1024;
        break;

        default:
        // error, unset
        pwm->prescalar = 0;
        break;
    }

    /* Turn on all 3 output pins [DDRB, DDRE, DDRH, DDRL]
       General Setup: DDRn |= ((1<< OCnA) | (1<< OCnB) | (1<< OCnC))
    */
    switch(pwm->timer->timer_n)
    {
        case 1:
        // pins 11, 12, 13
        DDRB |= ((1 << 5) | (1 << 6 ) | (1 << 7));
        pwm->DDReg = &DDRB;
        break;

        case 3:
        // pins 5, 2, 3
        DDRE |= ((1 << 3) | (1 << 4 ) | (1 << 5));
        pwm->DDReg = &DDRE;
        break;

        case 4:
        // pins 6, 7, 8
        DDRH |= ((1 << 3) | (1 << 4 ) | (1 << 5));
        pwm->DDReg = &DDRH;
        break;

        case 5:
        // pins 46, 45, 44
        DDRL |= ((1 << 3) | (1 << 4 ) | (1 << 5));
        pwm->DDReg = &DDRL;
        break;
    }

    /* Set counter_max [20Hz for 16Mhz cpu clock] */
    pwm->counter_max = counter_max;

    /* set TOP [ICRn] */
    *(pwm->timer->ICRn) = pwm->counter_max;

   /* start from zero */
    *(pwm->timer->TCNTn) = 0x0000;
}

/*# Set pwm configuration and initial state

  Parameters
  ----------
  pwm_config[0]: max level
  pwm_config[1]: min level
  pwm_config[2]: idle level
  pwm_config[3]: increments in compare value stored in register
*/
void PWM_PwmConfig(PWM *pwm, uint16_t pwm_config[4], PWM_Channel chn_x)
{
    pwm->pwm_level_max[chn_x]  = pwm_config[0];
    pwm->pwm_level_min[chn_x]  = pwm_config[1];
    pwm->pwm_level_idle[chn_x] = pwm_config[2];
    pwm->pwm_step[chn_x]       = pwm_config[3];

    pwm->pwm_level[chn_x] = pwm->pwm_level_idle[chn_x];
    *(pwm->OCRnx[chn_x]) = pwm->pwm_level[chn_x] * pwm->pwm_step[chn_x];
}

/* # Increment duty cycle level */
int PWM_Inc(PWM *pwm, PWM_Channel chn_x)
{
    if(pwm->pwm_level[chn_x] < pwm->pwm_level_max[chn_x])
    {
        *(pwm->OCRnx[chn_x]) += pwm->pwm_step[chn_x];
        pwm->pwm_level[chn_x]++;
    }
    return 0;
}

/* # Decrement duty cycle level */
int PWM_Dec(PWM *pwm, PWM_Channel chn_x)
{
    if(pwm->pwm_level[chn_x] > pwm->pwm_level_min[chn_x])
    {
        *(pwm->OCRnx[chn_x]) -= pwm->pwm_step[chn_x];
        pwm->pwm_level[chn_x]--;
    }
    return 0;
}

/* # Calculate and return duty cycle of given PWM */
int PWM_FrequencyHz(PWM *pwm, char *str_out)
{
    uint8_t denom = 2 * pwm->counter_max * pwm->prescalar;
    if(denom <= 0)
        return -1;
    else
    {
        uint8_t freq_hz = (F_CPU - (F_CPU % denom)) / denom;
        return sprintf(str_out, "%d Hz", freq_hz);
    }
}

/* # Calculate and return duty cycle of given PWM */
int PWM_DutyCycle(PWM *pwm, PWM_Channel chn_x, char *str_out)
{
    if(*(pwm->OCRnx[chn_x]) > pwm->counter_max)
        return -1;
    else
    {
        uint8_t mod = (100 * pwm->counter_max) % *(pwm->OCRnx[chn_x]);
        uint8_t out = ((100 * pwm->counter_max) - mod) / *(pwm->OCRnx[chn_x]);
        return sprintf(str_out, "%d %%", out);
    }
}

/* # Set level back to idle
*/
int PWM_Idle(PWM * pwm, PWM_Channel chn_x)
{
    while(pwm->pwm_level[chn_x] < pwm->pwm_level_idle[chn_x])
    {
        _delay_ms(25);
        *(pwm->OCRnx[chn_x]) += pwm->pwm_step[chn_x];
        pwm->pwm_level[chn_x]++;
    }
    while(pwm->pwm_level[chn_x] > pwm->pwm_level_idle[chn_x])
    {
        _delay_ms(25);
        *(pwm->OCRnx[chn_x]) -= pwm->pwm_step[chn_x];
        pwm->pwm_level[chn_x]--;
    }
    return 0;
}

