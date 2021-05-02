/*==============================================================================
  Header for PWM object

    Description
    -----------
    A single 16 bit timer on the Atmega2560 can produce 3 synchronized PWM
    signals. This object configures a given timer abstraction layer to do just
    that.

 =============================================================================*/

#include "timer.h"

#ifndef F_CPU
// Require CPU freq 16 MHz
#define F_CPU 16000000
#endif

#ifndef PWM_H
#define PWM_H

typedef enum {chn_A, chn_B, chn_C} PWM_Channel;

typedef struct PWM
{
    /* timer object reference; contains pointers to registers */
    TIMER * timer;

    /* prescalar used in timer-counter */
    uint16_t prescalar;

    /* register containing output pins */
    volatile uint8_t * DDReg;

    /* min, max values of pwm configured oscillator-counter */
    uint16_t counter_max;

    /* pointers to compare registers; controls the duty cycle */
    volatile uint16_t * OCRnx[3];

    /* state variables for pwm's; level proportional to duty cycle */
    uint16_t pwm_level[3];

    /* constraints to pwm levels */
    uint16_t pwm_level_max[3];
    uint16_t pwm_level_min[3];
    uint16_t pwm_level_idle[3];
    uint16_t pwm_step[3];

} PWM;

// Set timer configuration
extern void PWM_TimerConfig(
    PWM* pwm,
    TIMER* timer,
    uint8_t prescalar,
    uint8_t inverted,
    uint16_t counter_max
);

// Set pwm configuration and initial state
extern void PWM_PwmConfig(PWM * pwm, uint16_t pwn_config[4], PWM_Channel chn_x);

// Constructor
extern int PWM_Init(PWM * pwm, TIMER * timer);

// Increase level of specified PWM
extern int PWM_Inc(PWM * pwm, PWM_Channel chn_x);

// Decrease level of specified PWM
extern int PWM_Dec(PWM * pwm, PWM_Channel chn_x);

// Set level back to idle
extern int PWM_Idle(PWM * pwm, PWM_Channel chn_x);

// Calculate and return duty cycle of given PWM
extern int PWM_FrequencyHz(PWM * pwm, char *str_out);

// Calculate and return duty cycle of given PWM
extern int PWM_DutyCycle(PWM * pwm, PWM_Channel x, char *str_out);

/* future todo's */

// // Retire or disable a PWM output
// extern int PWM_Retire(PWM * pwm, PWM_Channel chn_x);

// // Returns string formatted configuration of PWMs
// extern int PWM_ConfigReport(PWM * pwm, char *str_out);

/* preset for 20Hz servo app [prescalar, uninverted, max_count] */
#define SERVO_PWM 0x04, 0, 0x271

#endif