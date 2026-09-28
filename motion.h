/*==============================================================================
  Header for the motion layer

    Description
    -----------
    Owns the servo joints: their limits, where each one is now and where it is
    heading. A timer overflow interrupt (one per PWM period, 50 Hz) moves every
    joint a little towards its target and writes the new pulse widths, so the
    main loop never has to wait for a servo.

    Two kinds of motion:

    - Jog: MOTION_Jog() nudges one joint's target. The joint follows at up to
      MOTION_JOG_SLEW_US per tick. Used by inc/dec and the keyboard modes.

    - Move: MOTION_MoveTo() / MOTION_MoveJoint() start a coordinated move. All
      joints start and finish together and ease in / ease out (smoothstep),
      with the fastest joint peaking at the configured speed.

    All positions are pulse widths in microseconds, always kept inside each
    joint's [min, max].

 =============================================================================*/
#ifndef MOTION_H
#define MOTION_H

#include "global.h"
#include "timer.h"
#include "pwm.h"

#define N_JOINTS 6

/* max change per tick while following a jog; 40 us / 20 ms = 2000 us/s */
#define MOTION_JOG_SLEW_US 40

/* peak speed for coordinated moves, in us of pulse width per second */
#define MOTION_SPEED_MIN     10
#define MOTION_SPEED_MAX     5000
#define MOTION_SPEED_DEFAULT 1000

// Attach joint to a PWM channel with limits in us; it starts at idle_us.
// Returns -1 for an invalid joint or limits.
extern int MOTION_AddJoint(
    uint8_t joint,
    PWM *pwm,
    PWM_Channel chn_x,
    uint16_t min_us,
    uint16_t max_us,
    uint16_t idle_us
);

// Start updating joints from tick_timer's overflow interrupt, tick_hz times a second
extern void MOTION_Start(TIMER *tick_timer, uint8_t tick_hz);

// Move one joint's target by delta_us; cancels a coordinated move in progress
extern void MOTION_Jog(uint8_t joint, int16_t delta_us);

// Coordinated smooth move of all joints / one joint (others keep their targets)
extern void MOTION_MoveTo(const uint16_t goal_us[N_JOINTS]);
extern void MOTION_MoveJoint(uint8_t joint, uint16_t goal_us);

// Hold every joint where it is now
extern void MOTION_Stop(void);

// TRUE while any joint is still moving
extern uint8_t MOTION_Busy(void);

// Current commanded position and target of a joint, in us
extern uint16_t MOTION_Position(uint8_t joint);
extern uint16_t MOTION_Target(uint8_t joint);
extern void MOTION_Targets(uint16_t target_us[N_JOINTS]);

// Joint limits, in us
extern uint16_t MOTION_Min(uint8_t joint);
extern uint16_t MOTION_Max(uint8_t joint);
extern uint16_t MOTION_Idle(uint8_t joint);

// Peak speed of coordinated moves; returns -1 if out of range
extern int MOTION_SetSpeed(uint16_t us_per_s);
extern uint16_t MOTION_GetSpeed(void);

// Ticks since start (wraps), and ticks per second
extern uint16_t MOTION_Ticks(void);
extern uint8_t MOTION_TickHz(void);

#endif
