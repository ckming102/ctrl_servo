/*==============================================================================
    Useful macros and global flags to keep track of state 
 =============================================================================*/
#ifndef GLOBAL_H
#define GLOBAL_H

/* ---------------------*/
/*  Stand-in for bools  */
/* ---------------------*/
    #define TRUE 1
    #define FALSE 0

/* ---------------- */
/*  Bit Ops Macros  */
/* ---------------- */
    #define flip_1bit(REG, BIT_POS) (REG ^= (1 << BIT_POS))
    #define sethigh_1bit(REG, BIT_POS) (REG |= (1 << BIT_POS))
    #define setlow_1bit(REG, BIT_POS) (REG &= ~ (1 << BIT_POS))
    #define set_1bit(REG, BIT_POS, VAL) (\
        VAL ? sethigh_1bit(REG, BIT_POS) : setlow_1bit(REG, BIT_POS) \
    )

/* ---------------------------------- */
/*  GLOBAL flags and state variables  */
/* ---------------------------------- */
    /* Global flags */
    struct GLOBAL_FLAGS {
      /* True when uart has received a string (ended with '/r') */
      unsigned char cmd_check:1;
      /* True when valid command is executing */
      unsigned char cmd_executed:1;
      /* True when RX interrupt received */
      unsigned char rx_int:1;
      /* True when escape character received */
      unsigned char esc_char:1;
      /* True when [ received */
      unsigned char bracket:1;
      /* Dummy bits to fill up a byte */
      unsigned char dummy:3;
    }; 

    volatile struct GLOBAL_FLAGS status;

    /* context */
    #define N_CONTEXT_TYPES 2
    enum context_types
    {
      context_cli, context_manual
    };

    volatile unsigned char context;

    /* error number */
    int err_no;

/* --------------------------------- */
/*  State variables for PWM control  */
/* --------------------------------- */

    unsigned char pwm_C1_level;
    unsigned char pwm_B1_level;
    unsigned char pwm_A1_level;

    unsigned char pwm_C1_slider_pos;
    unsigned char pwm_B1_slider_pos;
    unsigned char pwm_A1_slider_pos;



#endif
