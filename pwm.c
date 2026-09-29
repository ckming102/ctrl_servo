/*==============================================================================
  Function declarations and data structures for 16 bit timers
 =============================================================================*/
#include <stdio.h>
#include <util/atomic.h>
#include "global.h"
#include "timer.h"
#include "pwm.h"

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

  Setup timer for phase-freq correct PWM output (mode 8, TOP = ICRn). Details
  of modes in pg. 145 of ATmega2560 data-sheet and Arduino pinout. The clock is
  left stopped so the compare values can be set first; see PWM_Start().

  Parameters
  ----------
  prescalar  : clock divider; one of 1, 8, 64, 256, 1024
  inverted   : 1 for inverted and 0 for un-inverted
  counter_max: TOP value, sets the PWM frequency
*/
int PWM_TimerConfig(
    PWM *pwm,
    TIMER *timer,
    uint16_t prescalar,
    uint8_t inverted,
    uint16_t counter_max
)
{
    /* Clock source and prescalar [TCCRnB]

       CSn2:0 is a 3 bit number, not one bit per prescalar:

        prescalar | CSn2:0
        ----------|-------
        1         | 001
        8         | 010
        64        | 011
        256       | 100
        1024      | 101
    */
    switch(prescalar)
    {
        case 1:
        pwm->cs_bits = 0x01;
        break;

        case 8:
        pwm->cs_bits = 0x02;
        break;

        case 64:
        pwm->cs_bits = 0x03;
        break;

        case 256:
        pwm->cs_bits = 0x04;
        break;

        case 1024:
        pwm->cs_bits = 0x05;
        break;

        default:
        // error: unsupported prescalar
        return -1;
    }

    /* pulse width = 2 * counts * prescalar / F_CPU */
    pwm->counts_per_us_q8 =
        (uint16_t)((F_CPU / 1000000UL * 256UL) / (2UL * prescalar));
    if(counter_max == 0 || pwm->counts_per_us_q8 == 0)
        return -1;

    /* copy in timer and register addresses */
    pwm->timer = timer;
    pwm->prescalar = prescalar;
    pwm->counter_max = counter_max;
    pwm->OCRnx[chn_A] = timer->OCRnA;
    pwm->OCRnx[chn_B] = timer->OCRnB;
    pwm->OCRnx[chn_C] = timer->OCRnC;

    /* stop the clock while configuring */
    *(pwm->timer->TCCRnB) &= ~CSn_MASK;

    /* Set to PWM mode [TCCRnA, TCCRnB] */

    // set to PWM mode to all 3 outputs
    set_1bit(*(pwm->timer->TCCRnB), WGMn3, 1);
    set_1bit(*(pwm->timer->TCCRnB), WGMn2, 0);
    set_1bit(*(pwm->timer->TCCRnA), WGMn1, 0);
    set_1bit(*(pwm->timer->TCCRnA), WGMn0, 0);

    /* Inverted(11) vs. Uninverted(10) [TCCRnA] */
    set_1bit(*(pwm->timer->TCCRnA), COMnA1, 1);
    set_1bit(*(pwm->timer->TCCRnA), COMnA0, inverted);

    set_1bit(*(pwm->timer->TCCRnA), COMnB1, 1);
    set_1bit(*(pwm->timer->TCCRnA), COMnB0, inverted);

    set_1bit(*(pwm->timer->TCCRnA), COMnC1, 1);
    set_1bit(*(pwm->timer->TCCRnA), COMnC0, inverted);

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

    /* set TOP [ICRn] */
    *(pwm->timer->ICRn) = pwm->counter_max;

   /* start from zero */
    *(pwm->timer->TCNTn) = 0x0000;

    return 0;
}

/* # Start the timer clock */
void PWM_Start(PWM *pwm)
{
    *(pwm->timer->TCCRnB) = (*(pwm->timer->TCCRnB) & ~CSn_MASK) | pwm->cs_bits;
}

/* # Set compare value; clamped to TOP */
void PWM_Write(PWM *pwm, PWM_Channel chn_x, uint16_t counts)
{
    if(counts > pwm->counter_max) counts = pwm->counter_max;
    *(pwm->OCRnx[chn_x]) = counts;
}

/* # Get compare value; atomic, as the motion interrupt may be writing it */
uint16_t PWM_Read(PWM *pwm, PWM_Channel chn_x)
{
    uint16_t counts;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        counts = *(pwm->OCRnx[chn_x]);
    }
    return counts;
}

/* # Pulse width in microseconds to timer counts (rounded) */
uint16_t PWM_UsToCounts(const PWM *pwm, uint16_t us)
{
    return (uint16_t)(((uint32_t)us * pwm->counts_per_us_q8 + 128) >> 8);
}

/* # Timer counts to pulse width in microseconds (rounded) */
uint16_t PWM_CountsToUs(const PWM *pwm, uint16_t counts)
{
    return (uint16_t)((((uint32_t)counts << 8) + pwm->counts_per_us_q8 / 2)
                      / pwm->counts_per_us_q8);
}

/* # PWM frequency in hundredths of a Hz: F_CPU / (2 * N * TOP) */
uint32_t PWM_FrequencyCentiHz(const PWM *pwm)
{
    uint32_t denom = 2UL * pwm->prescalar * pwm->counter_max;
    return (F_CPU * 100UL + denom / 2) / denom;
}

/* # Write PWM frequency as a string */
int PWM_FrequencyHz(PWM *pwm, char *str_out)
{
    uint32_t centi_hz = PWM_FrequencyCentiHz(pwm);
    return sprintf(str_out, "%lu.%02lu Hz", centi_hz / 100, centi_hz % 100);
}

/* # Write duty cycle of given channel as a string: OCRnx / TOP */
int PWM_DutyCycle(PWM *pwm, PWM_Channel chn_x, char *str_out)
{
    uint32_t centi_pct =
        ((uint32_t)PWM_Read(pwm, chn_x) * 10000UL + pwm->counter_max / 2)
        / pwm->counter_max;
    return sprintf(str_out, "%lu.%02lu %%", centi_pct / 100, centi_pct % 100);
}
