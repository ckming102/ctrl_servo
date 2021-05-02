/*==============================================================================
  Function declarations and data structures for 16 bit timers
 =============================================================================*/
#include "timer.h"

/* ------------------ */
/*  Extern variables  */
/* ------------------ */

/* ------------------ */
/*  Static variables  */
/* ------------------ */

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
        timer->timer_reg_loc = (volatile uint8_t *) 0x80;
        break;

        case 3:
        timer->timer_reg_loc = (volatile uint8_t *) 0x90;
        break;

        case 4:
        timer->timer_reg_loc = (volatile uint8_t *) 0xA0;
        break;

        case 5:
        timer->timer_reg_loc = (volatile uint8_t *) 0x120;
        break;

        default:
        // Error: unknown timer id
        return -1;

    }
    timer->timer_n = n;

    /* Set register addresses */
    timer->TCCRnA = &_SFR_MEM8(timer->timer_reg_loc + _TCCRnA);
    timer->TCCRnB = &_SFR_MEM8(timer->timer_reg_loc + _TCCRnB);
    timer->TCCRnC = &_SFR_MEM8(timer->timer_reg_loc + _TCCRnC);

    timer->ICRn   = &_SFR_MEM16(timer->timer_reg_loc + _ICRn);
    timer->TCNTn   = &_SFR_MEM16(timer->timer_reg_loc + _TCNTn);

    timer->OCRnA = &_SFR_MEM16(timer->timer_reg_loc + _OCRnA);
    timer->OCRnB = &_SFR_MEM16(timer->timer_reg_loc + _OCRnB);
    timer->OCRnC = &_SFR_MEM16(timer->timer_reg_loc + _OCRnC);

    return 0;
}

