/*==============================================================================
  Header for 16 bit timers

    Description
    -----------
    An abstraction layer for single 16 bit timer on the Atmega2560. Data
    stucture contains the locations of registers used for configuration. An
    object constructor to select a specific timer n = 1,3,4,5.

 =============================================================================*/
#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <avr/io.h>

/* # Address offsets for registers

   All register addresses in "/lib/avr/include/avr/iomxx0_1.h" and is
   specific to the ATMega2560. Modify in future as needed.
*/

/* relative offsets of registers */
#define _TCCRnA  0x00
#define _TCCRnB  0x01
#define _TCCRnC  0x02

#define _ICRn    0x06
#define _TCNTn   0x04

#define _OCRnA   0x08
#define _OCRnB   0x0A
#define _OCRnC   0x0C

/* register bit positions */
#define COMnA1  7
#define COMnA0  6
#define COMnB1  5
#define COMnB0  4
#define COMnC1  3
#define COMnC0  2

#define WGMn3  4
#define WGMn2  3
#define WGMn1  1
#define WGMn0  0

#define ICNCn   7
#define ICESn   6

#define CSn2  2
#define CSn1  1
#define CSn0  0

#define FOCnA   7
#define FOCnB   6
#define FOCnC   5

/* -------------- */
/*  Timer Object  */
/* -------------- */

typedef struct TIMER
{
    /* General location of timer registers */
    volatile uint8_t * timer_reg_loc;

    /* Timer registers; [OCRnx and TCNTn not used] */
    volatile uint8_t * TCCRnA;
    volatile uint8_t * TCCRnB;
    volatile uint8_t * TCCRnC;

    volatile uint16_t * ICRn;
    volatile uint16_t * TCNTn;

    volatile uint16_t * OCRnA;
    volatile uint16_t * OCRnB;
    volatile uint16_t * OCRnC;

    /* # 16-bit timer number, n = 1,3,4,5 */
    uint8_t timer_n;

} TIMER;

/* Constructor */
extern int TIMER_Init(TIMER *timer, uint8_t n);

#endif