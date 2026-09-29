/*==============================================================================
  Source for the motion layer; see motion.h
 =============================================================================*/
#include <stddef.h>
#include <util/atomic.h>
#include "global.h"
#include "timer.h"
#include "pwm.h"
#include "motion.h"

/* ------------------ */
/*  Static variables  */
/* ------------------ */

/* joint configuration; pwm is NULL for joints that are not attached */
typedef struct JOINT
{
    PWM * pwm;
    PWM_Channel chn;
    uint16_t min_us;
    uint16_t max_us;
    uint16_t idle_us;
} JOINT;

static JOINT joint_cfg[N_JOINTS];

/* Shared with the tick interrupt; main loop accesses them in ATOMIC_BLOCKs */

/* commanded position, written to the PWM every tick */
static volatile uint16_t pos_us[N_JOINTS];
/* where each joint ends up */
static volatile uint16_t target_us[N_JOINTS];

/* coordinated move: pos = start + delta * smoothstep(t / ticks) */
static uint16_t move_start[N_JOINTS];
static int16_t move_delta[N_JOINTS];
static uint16_t move_ticks;
static volatile uint16_t move_t;
static volatile uint8_t move_active;

static volatile uint16_t tick_count;
static uint8_t tick_hz = 50;
static uint16_t speed_us_per_s = MOTION_SPEED_DEFAULT;

/* ---------------------- */
/*  Function definitions  */
/* ---------------------- */

static uint16_t _Clamp(uint8_t joint, int32_t us)
{
    if(us < joint_cfg[joint].min_us) return joint_cfg[joint].min_us;
    if(us > joint_cfg[joint].max_us) return joint_cfg[joint].max_us;
    return (uint16_t)us;
}

/* # Smoothstep in Q15: u^2 (3 - 2u), for u in [0, 1] = [0, 32768]

   Starts and ends with zero velocity; peak velocity is 1.5x the average.
*/
static uint16_t _Smoothstep(uint16_t u)
{
    uint32_t u2 = ((uint32_t)u * u) >> 15;
    return (uint16_t)((u2 * (3UL * 32768UL - 2UL * u)) >> 15);
}

static void _WriteJoint(uint8_t joint)
{
    PWM * pwm = joint_cfg[joint].pwm;
    PWM_Write(pwm, joint_cfg[joint].chn, PWM_UsToCounts(pwm, pos_us[joint]));
}

/* # Called from the tick timer's overflow interrupt, once per PWM period */
static void _Tick(void)
{
    uint8_t j;

    tick_count++;

    if(move_active)
    {
        move_t++;
        if(move_t >= move_ticks)
        {
            for(j = 0; j < N_JOINTS; j++) pos_us[j] = target_us[j];
            move_active = FALSE;
        }
        else
        {
            uint16_t s = _Smoothstep(((uint32_t)move_t << 15) / move_ticks);
            /* delta * s rounded to nearest, so both directions move alike */
            for(j = 0; j < N_JOINTS; j++)
                pos_us[j] = move_start[j]
                    + (int16_t)(((int32_t)move_delta[j] * s + (1L << 14)) >> 15);
        }
    }
    else
    {
        /* follow jogs at a capped rate */
        for(j = 0; j < N_JOINTS; j++)
        {
            int16_t err = (int16_t)target_us[j] - (int16_t)pos_us[j];
            if(err > MOTION_JOG_SLEW_US) err = MOTION_JOG_SLEW_US;
            else if(err < -MOTION_JOG_SLEW_US) err = -MOTION_JOG_SLEW_US;
            pos_us[j] += err;
        }
    }

    for(j = 0; j < N_JOINTS; j++)
        if(joint_cfg[j].pwm != NULL) _WriteJoint(j);
}

/* # Attach a joint to a PWM channel; the joint starts (and is driven) at idle */
int MOTION_AddJoint(
    uint8_t joint,
    PWM *pwm,
    PWM_Channel chn_x,
    uint16_t min_us,
    uint16_t max_us,
    uint16_t idle_us
)
{
    if(joint >= N_JOINTS || pwm == NULL)
        return -1;
    if(min_us > idle_us || idle_us > max_us)
        return -1;
    if(PWM_UsToCounts(pwm, max_us) > pwm->counter_max)
        return -1;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        joint_cfg[joint].pwm = pwm;
        joint_cfg[joint].chn = chn_x;
        joint_cfg[joint].min_us = min_us;
        joint_cfg[joint].max_us = max_us;
        joint_cfg[joint].idle_us = idle_us;

        pos_us[joint] = idle_us;
        target_us[joint] = idle_us;
        _WriteJoint(joint);
    }
    return 0;
}

/* # Start the tick */
void MOTION_Start(TIMER *tick_timer, uint8_t hz)
{
    if(hz > 0) tick_hz = hz;
    TIMER_SetOverflowCallback(tick_timer, &_Tick);
}

/* # Nudge one joint's target */
void MOTION_Jog(uint8_t joint, int16_t delta_us)
{
    uint8_t j;

    if(joint >= N_JOINTS || joint_cfg[joint].pwm == NULL)
        return;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        /* a jog takes over: freeze a coordinated move where it is */
        if(move_active)
        {
            for(j = 0; j < N_JOINTS; j++) target_us[j] = pos_us[j];
            move_active = FALSE;
        }
        target_us[joint] = _Clamp(joint, (int32_t)target_us[joint] + delta_us);
    }
}

/* # Coordinated smooth move of all joints */
void MOTION_MoveTo(const uint16_t goal_us[N_JOINTS])
{
    uint8_t j;
    uint16_t max_delta = 0;
    uint32_t ticks;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        for(j = 0; j < N_JOINTS; j++)
        {
            uint16_t goal;
            uint16_t delta_abs;

            if(joint_cfg[j].pwm == NULL) continue;

            goal = _Clamp(j, goal_us[j]);
            move_start[j] = pos_us[j];
            move_delta[j] = (int16_t)goal - (int16_t)pos_us[j];
            target_us[j] = goal;

            delta_abs = move_delta[j] < 0 ? -move_delta[j] : move_delta[j];
            if(delta_abs > max_delta) max_delta = delta_abs;
        }

        /* smoothstep peaks at 1.5x the average speed; size the move so the
           joint travelling furthest peaks at speed_us_per_s */
        ticks = ((uint32_t)max_delta * 3UL * tick_hz + 2UL * speed_us_per_s - 1)
                / (2UL * speed_us_per_s);
        if(ticks > 0xFFFF) ticks = 0xFFFF;

        if(ticks == 0)
        {
            /* already there */
            move_active = FALSE;
        }
        else
        {
            move_ticks = (uint16_t)ticks;
            move_t = 0;
            move_active = TRUE;
        }
    }
}

/* # Coordinated smooth move of one joint; the others keep their targets */
void MOTION_MoveJoint(uint8_t joint, uint16_t goal_us)
{
    uint16_t goal[N_JOINTS];

    if(joint >= N_JOINTS)
        return;

    MOTION_Targets(goal);
    goal[joint] = goal_us;
    MOTION_MoveTo(goal);
}

/* # Hold every joint where it is now */
void MOTION_Stop(void)
{
    uint8_t j;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        move_active = FALSE;
        for(j = 0; j < N_JOINTS; j++) target_us[j] = pos_us[j];
    }
}

uint8_t MOTION_Busy(void)
{
    uint8_t j;
    uint8_t busy = FALSE;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        busy = move_active;
        for(j = 0; j < N_JOINTS; j++)
            if(pos_us[j] != target_us[j]) busy = TRUE;
    }
    return busy;
}

uint16_t MOTION_Position(uint8_t joint)
{
    uint16_t us;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        us = pos_us[joint];
    }
    return us;
}

uint16_t MOTION_Target(uint8_t joint)
{
    uint16_t us;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        us = target_us[joint];
    }
    return us;
}

void MOTION_Targets(uint16_t target[N_JOINTS])
{
    uint8_t j;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        for(j = 0; j < N_JOINTS; j++) target[j] = target_us[j];
    }
}

uint16_t MOTION_Min(uint8_t joint) { return joint_cfg[joint].min_us; }
uint16_t MOTION_Max(uint8_t joint) { return joint_cfg[joint].max_us; }
uint16_t MOTION_Idle(uint8_t joint) { return joint_cfg[joint].idle_us; }

int MOTION_SetSpeed(uint16_t us_per_s)
{
    if(us_per_s < MOTION_SPEED_MIN || us_per_s > MOTION_SPEED_MAX)
        return -1;
    speed_us_per_s = us_per_s;
    return 0;
}

uint16_t MOTION_GetSpeed(void)
{
    return speed_us_per_s;
}

uint16_t MOTION_Ticks(void)
{
    uint16_t ticks;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ticks = tick_count;
    }
    return ticks;
}

uint8_t MOTION_TickHz(void)
{
    return tick_hz;
}
