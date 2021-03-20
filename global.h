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
      uint8_t cmd_check:1;
      /* True when valid command is executing */
      uint8_t cmd_executed:1;
      /* True when RX interrupt received */
      uint8_t rx_int:1;
      /* True when escape character received */
      uint8_t esc_char:1;
      /* True when [ received */
      uint8_t bracket:1;
      /* Dummy bits to fill up a byte */
      uint8_t dummy:3;
    }; 

    volatile struct GLOBAL_FLAGS status;

    /* context */
    #define N_CONTEXT_TYPES 2
    enum context_types
    {
      context_cli, context_manual
    };

    volatile uint8_t context;

    /* error number */
    int err_no;

/* --------------------------------- */
/*  State variables for PWM control  */
/* --------------------------------- */

    uint8_t pwm_C1_level;
    uint8_t pwm_B1_level;
    uint8_t pwm_A1_level;

    uint8_t pwm_C1_slider_pos;
    uint8_t pwm_B1_slider_pos;
    uint8_t pwm_A1_slider_pos;



#endif
