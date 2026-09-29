/*==============================================================================
  Header for PWM object

    Description
    -----------
    A single 16 bit timer on the Atmega2560 can produce 3 synchronized PWM
    signals. This object configures a given timer abstraction layer to do just
    that, using mode 8: phase and frequency correct PWM with TOP = ICRn.

    In mode 8 the counter runs 0 -> TOP -> 0, so for a prescalar N:

        frequency   = F_CPU / (2 * N * TOP)
        pulse width = 2 * OCRnx * N / F_CPU
        duty cycle  = OCRnx / TOP

    The PWM object works in timer counts. PWM_UsToCounts / PWM_CountsToUs
    convert to and from pulse width in microseconds.

 =============================================================================*/
#ifndef PWM_H
#define PWM_H

#include "global.h"
#include "timer.h"

typedef enum {chn_A, chn_B, chn_C} PWM_Channel;

typedef struct PWM
{
    /* timer object reference; contains pointers to registers */
    TIMER * timer;

    /* prescalar used in timer-counter: 1, 8, 64, 256 or 1024 */
    uint16_t prescalar;

    /* clock select code CSn2:0 matching the prescalar */
    uint8_t cs_bits;

    /* register containing output pins */
    volatile uint8_t * DDReg;

    /* TOP of the counter, stored in ICRn; sets the PWM frequency */
    uint16_t counter_max;

    /* timer counts per microsecond of pulse width, times 256 */
    uint16_t counts_per_us_q8;

    /* pointers to compare registers; controls the duty cycle */
    volatile uint16_t * OCRnx[3];

} PWM;

// Configure timer for PWM on all 3 outputs; the timer is left stopped.
// Returns -1 for an unsupported prescalar or counter_max.
extern int PWM_TimerConfig(
    PWM* pwm,
    TIMER* timer,
    uint16_t prescalar,
    uint8_t inverted,
    uint16_t counter_max
);

// Start the timer clock
extern void PWM_Start(PWM * pwm);

// Set / get compare value (timer counts) of a channel
extern void PWM_Write(PWM * pwm, PWM_Channel chn_x, uint16_t counts);
extern uint16_t PWM_Read(PWM * pwm, PWM_Channel chn_x);

// Convert between pulse width in microseconds and timer counts
extern uint16_t PWM_UsToCounts(const PWM * pwm, uint16_t us);
extern uint16_t PWM_CountsToUs(const PWM * pwm, uint16_t counts);

// PWM frequency in hundredths of a Hz
extern uint32_t PWM_FrequencyCentiHz(const PWM * pwm);

// Write PWM frequency as a string, e.g. "50.00 Hz"
extern int PWM_FrequencyHz(PWM * pwm, char *str_out);

// Write duty cycle of given channel as a string, e.g. "6.40 %"
extern int PWM_DutyCycle(PWM * pwm, PWM_Channel x, char *str_out);

/* future todo's */

// // Retire or disable a PWM output
// extern int PWM_Retire(PWM * pwm, PWM_Channel chn_x);

/* preset for 50 Hz (20 ms) servo pulses with 1 us resolution at 16 MHz
   [prescalar, uninverted, max_count]:
   16 MHz / (2 * 8 * 20000) = 50 Hz, and 1 count = 2 * 8 / 16 MHz = 1 us */
#define SERVO_PWM 8, 0, 20000

#endif
