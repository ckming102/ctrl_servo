/*==============================================================================
    Useful macros shared by all modules

    Note: only macros and declarations belong in this header. Variables defined
    here would get one copy per .c file, which fails to link with -fno-common
    (the default since GCC 10).
 =============================================================================*/
#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>

/* CPU clock; normally passed in by the Makefile as -DF_CPU=... */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ---------------------*/
/*  Stand-in for bools  */
/* ---------------------*/
#define TRUE 1
#define FALSE 0

/* ---------------- */
/*  Bit Ops Macros  */
/* ---------------- */
#define flip_1bit(REG, BIT_POS) ((REG) ^= (1 << (BIT_POS)))
#define sethigh_1bit(REG, BIT_POS) ((REG) |= (1 << (BIT_POS)))
#define setlow_1bit(REG, BIT_POS) ((REG) &= ~(1 << (BIT_POS)))
#define set_1bit(REG, BIT_POS, VAL) (\
    (VAL) ? sethigh_1bit(REG, BIT_POS) : setlow_1bit(REG, BIT_POS) \
)

#endif
