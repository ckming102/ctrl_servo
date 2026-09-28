/*==============================================================================
  Function declarations and data structures for 16 bit timers
 =============================================================================*/
#include <stddef.h>
#include <avr/interrupt.h>
#include "global.h"
#include "timer.h"

/* ------------------ */
/*  Extern variables  */
/* ------------------ */

/* ------------------ */
/*  Static variables  */
/* ------------------ */

/* overflow callbacks indexed by timer number; only 1, 3, 4 and 5 are used */
static void (* volatile overflow_callback[6])(void);

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

/* # Timer constructor

   Select 16-bit timer to configure for PWM output. Choices are
   n = 1,3,4,5. See data-sheet and schematics for pin locations.
*/
int TIMER_Init(TIMER *timer, uint8_t n)
{

    /* Determine offset address */
    switch(n)
    {
        case 1:
        timer->timer_reg_loc = &TCCR1A;
        timer->TIMSKn = &TIMSK1;
        break;

        case 3:
        timer->timer_reg_loc = &TCCR3A;
        timer->TIMSKn = &TIMSK3;
        break;

        case 4:
        timer->timer_reg_loc = &TCCR4A;
        timer->TIMSKn = &TIMSK4;
        break;

        case 5:
        timer->timer_reg_loc = &TCCR5A;
        timer->TIMSKn = &TIMSK5;
        break;

        default:
        // Error: unknown timer id
        return -1;

    }
    timer->timer_n = n;

    /* Set register addresses */
    timer->TCCRnA = timer->timer_reg_loc + _TCCRnA;
    timer->TCCRnB = timer->timer_reg_loc + _TCCRnB;
    timer->TCCRnC = timer->timer_reg_loc + _TCCRnC;

    timer->ICRn  = (volatile uint16_t *)(timer->timer_reg_loc + _ICRn);
    timer->TCNTn = (volatile uint16_t *)(timer->timer_reg_loc + _TCNTn);

    timer->OCRnA = (volatile uint16_t *)(timer->timer_reg_loc + _OCRnA);
    timer->OCRnB = (volatile uint16_t *)(timer->timer_reg_loc + _OCRnB);
    timer->OCRnC = (volatile uint16_t *)(timer->timer_reg_loc + _OCRnC);

    return 0;
}

/* # Register an overflow callback and enable / disable the interrupt */
void TIMER_SetOverflowCallback(TIMER *timer, void (*callback)(void))
{
    overflow_callback[timer->timer_n] = callback;
    set_1bit(*(timer->TIMSKn), TOIEn, callback != NULL);
}

/* ---------------------------- */
/*  Overflow interrupt handlers */
/* ---------------------------- */

static inline void _RunOverflowCallback(uint8_t n)
{
    void (*callback)(void) = overflow_callback[n];
    if(callback) callback();
}

ISR(TIMER1_OVF_vect) { _RunOverflowCallback(1); }
ISR(TIMER3_OVF_vect) { _RunOverflowCallback(3); }
ISR(TIMER4_OVF_vect) { _RunOverflowCallback(4); }
ISR(TIMER5_OVF_vect) { _RunOverflowCallback(5); }
