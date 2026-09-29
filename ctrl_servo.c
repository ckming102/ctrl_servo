/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# PWM control of 6 servos (a 6 DOF arm) through a serial terminal connected
  via UART.

- Connect to uart1's rx and tx of the atmega2560 board. Interface with USB to TTL
  module and GTKTerm serial program. Configured for 16Mhz and 19.2 kbps.

- Servo pulses are 50 Hz with 1 us resolution (timer prescalar 8, TOP 20000).
  Joint positions and limits are pulse widths in us.

- motion.c moves the joints from the timer 1 overflow interrupt, so the main
  loop below only reads the terminal and never waits for a servo.

# TODO:
- Acceleration limit for jogs (they are only speed limited).

- Joint-space moves only; no inverse kinematics.
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "global.h"
#include "uart.h"
#include "cmd.h"
#include "timer.h"
#include "pwm.h"
#include "motion.h"
#include "pose.h"

/* ------------- */
/*  Joint setup  */
/* ------------- */

/* Joint j is channel (j % 3) of PWM group (j / 3):
   group 0 is timer 1 (pins 11, 12, 13), group 1 is timer 4 (pins 6, 7, 8) */
static const char * const joint_name[N_JOINTS] = {
    "A0", "B0", "C0", "A1", "B1", "C1"
};

/* Limits in us of pulse width. These are the values tuned with the old 32 us
   steps (old level x 32 us), so range and rest pose are unchanged. */
static const struct JOINT_CONFIG
{
    uint16_t min_us;
    uint16_t max_us;
    uint16_t idle_us;
} joint_config[N_JOINTS] = {
    /* A0 */ { 448, 2304, 2304},
    /* B0 */ {1216, 1600, 1280},
    /* C0 */ {1760, 2080, 1760},
    /* A1 */ {1760, 2080, 1824},
    /* B1 */ { 448, 2304, 1280},
    /* C1 */ {1632, 2304, 2304},
};

/* Servo horn angle calibration for "set <joint> <angle>deg": the old code
   mapped 448..2304 us to -90..+90 deg. This is the servo's angle, not the
   joint's; adjust for your servos. */
#define SERVO_US_AT_0_DEG   1376
#define SERVO_US_PER_90_DEG  928

/* default inc/dec/keyboard step; 32 us is one step of the old code, ~3 deg */
#define JOG_STEP_DEFAULT 32
#define JOG_STEP_MAX 500

#define SLIDER_WIDTH 40

TIMER timer1;
TIMER timer4;

PWM pwm_grp[2];

/* joint used by inc, dec, idle, duty_cycle and manual mode */
static uint8_t joint_select = 1;
static uint16_t jog_step_us = JOG_STEP_DEFAULT;

/* -------------- */
/*  Terminal I/O  */
/* -------------- */
char str_buffer[80];

// for short temp strings
char str_temp[20];

/* context */
enum context_types
{
    context_cli, context_manual, context_game
};

static uint8_t context;

/* TRUE until the first key in manual / game mode; see mode_IsExitKey() */
static uint8_t mode_fresh;

/* ---------------- */
/*  Helpers         */
/* ---------------- */

static PWM * joint_Pwm(uint8_t joint)
{
    return &pwm_grp[joint / 3];
}

static PWM_Channel joint_Channel(uint8_t joint)
{
    return (PWM_Channel)(joint % 3);
}

/* "A0".."C1" (any case) to a joint number, or -1 */
static int8_t joint_Parse(const char *str)
{
    uint8_t chn;
    uint8_t grp;

    if(strlen(str) != 2)
        return -1;

    chn = (uint8_t)(toupper((unsigned char)str[0]) - 'A');
    grp = (uint8_t)(str[1] - '0');
    if(chn > 2 || grp > 1)
        return -1;

    return grp * 3 + chn;
}

/* whole decimal number in [min, max]; returns 0 if ok */
static int parse_Number(const char *str, int32_t min, int32_t max, int32_t *out)
{
    char *end;
    long value = strtol(str, &end, 10);

    if(end == str || *end != '\0' || value < min || value > max)
        return -1;

    *out = value;
    return 0;
}

static int16_t us_ToDeg(uint16_t us)
{
    int32_t x = ((int32_t)us - SERVO_US_AT_0_DEG) * 90;
    x += (x >= 0) ? SERVO_US_PER_90_DEG / 2 : -(SERVO_US_PER_90_DEG / 2);
    return (int16_t)(x / SERVO_US_PER_90_DEG);
}

static int32_t deg_ToUs(int32_t deg)
{
    int32_t x = deg * SERVO_US_PER_90_DEG;
    x += (x >= 0) ? 45 : -45;
    return SERVO_US_AT_0_DEG + x / 90;
}

/* user input moves a joint: stop any sequence that is playing */
static void jog(uint8_t joint, int8_t dir)
{
    SEQ_Stop();
    MOTION_Jog(joint, dir * (int16_t)jog_step_us);
}

static void stop_All(void)
{
    SEQ_Stop();
    MOTION_Stop();
}

/* ---------------------------------- */
/*  Registered commands and callbacks */
/* ---------------------------------- */

int cbk_help(uint8_t argc, char **argv)
{
    cli_PrintHelp();
    return 0;
}

int cbk_status(uint8_t argc, char **argv)
{
    uint8_t j;

    snprintf(str_buffer, sizeof(str_buffer), "\n\r %-5s %6s %6s %6s %6s %6s %9s\n\r",
        "Joint", "Now", "Target", "Min", "Idle", "Max", "Angle");
    uart_SendString(str_buffer);

    for(j = 0; j < N_JOINTS; j++)
    {
        uint16_t target = MOTION_Target(j);

        snprintf(str_buffer, sizeof(str_buffer), "%c%-5s %6u %6u %6u %6u %6u %+5d deg\n\r",
            (j == joint_select) ? '*' : ' ',
            joint_name[j],
            MOTION_Position(j),
            target,
            MOTION_Min(j),
            MOTION_Idle(j),
            MOTION_Max(j),
            us_ToDeg(target)
        );
        uart_SendString(str_buffer);
    }

    snprintf(str_buffer, sizeof(str_buffer),
        "\n\r Motion: %s  Speed: %u us/s  Step: %u us  Dwell: %u ms\n\r",
        MOTION_Busy() ? "moving" : "stopped",
        MOTION_GetSpeed(), jog_step_us, SEQ_GetDwell()
    );
    uart_SendString(str_buffer);

    if(SEQ_Active())
    {
        snprintf(str_buffer, sizeof(str_buffer), " Playing: step %u of %u%s\n\r",
            SEQ_Step(), SEQ_Length(), SEQ_Looping() ? " (loop)" : "");
        uart_SendString(str_buffer);
    }
    uart_SendString("\n\r");
    return 0;
}

static void print_JogTarget(void)
{
    snprintf(str_buffer, sizeof(str_buffer), "%s target %u us\n\r",
        joint_name[joint_select], MOTION_Target(joint_select));
    uart_SendString(str_buffer);
}

int cbk_inc_pwm_level(uint8_t argc, char **argv)
{
    jog(joint_select, +1);
    print_JogTarget();
    return 0;
}

int cbk_dec_pwm_level(uint8_t argc, char **argv)
{
    jog(joint_select, -1);
    print_JogTarget();
    return 0;
}

int cbk_idle_pwm_level(uint8_t argc, char **argv)
{
    uint16_t goal[N_JOINTS];
    uint8_t j;

    if(argc == 1)
    {
        SEQ_Stop();
        MOTION_MoveJoint(joint_select, MOTION_Idle(joint_select));
        snprintf(str_buffer, sizeof(str_buffer), "%s moving to idle (%u us)\n\r",
            joint_name[joint_select], MOTION_Idle(joint_select));
        uart_SendString(str_buffer);
    }
    else if(strcmp(argv[1], "all") == 0)
    {
        SEQ_Stop();
        for(j = 0; j < N_JOINTS; j++) goal[j] = MOTION_Idle(j);
        MOTION_MoveTo(goal);
        uart_SendString("All joints moving to idle\n\r");
    }
    else uart_SendString("Usage: idle [all]\n\r");

    return 0;
}

int cbk_step(uint8_t argc, char **argv)
{
    int32_t value;

    if(argc >= 2)
    {
        if(parse_Number(argv[1], 1, JOG_STEP_MAX, &value) != 0)
        {
            uart_SendString("Step must be 1 to 500 us\n\r");
            return 0;
        }
        jog_step_us = (uint16_t)value;
    }
    snprintf(str_buffer, sizeof(str_buffer), "Step: %u us\n\r", jog_step_us);
    uart_SendString(str_buffer);
    return 0;
}

int cbk_set(uint8_t argc, char **argv)
{
    int8_t joint;
    long value;
    char *end;
    int32_t us;

    if(argc < 3)
    {
        uart_SendString("Usage: set <joint> <us>  or  set <joint> <angle>deg\n\r");
        return 0;
    }

    joint = joint_Parse(argv[1]);
    if(joint < 0)
    {
        uart_SendString("Unknown joint; use A0 B0 C0 A1 B1 C1\n\r");
        return 0;
    }

    value = strtol(argv[2], &end, 10);
    if(end == argv[2])
        us = -1;
    else if(*end == '\0')
        us = (value >= 0 && value <= 20000) ? value : -1;
    else if(strcmp(end, "deg") == 0 || strcmp(end, "d") == 0)
        us = (value >= -180 && value <= 180) ? deg_ToUs(value) : -1;
    else
        us = -1;

    if(us < 0)
    {
        uart_SendString("Bad value; e.g. set B0 1500  or  set B0 -30deg\n\r");
        return 0;
    }

    SEQ_Stop();
    MOTION_MoveJoint(joint, (uint16_t)us);

    snprintf(str_buffer, sizeof(str_buffer), "%s moving to %u us%s\n\r",
        joint_name[joint], MOTION_Target(joint),
        (MOTION_Target(joint) != us) ? " (joint limit)" : "");
    uart_SendString(str_buffer);
    return 0;
}

int cbk_speed(uint8_t argc, char **argv)
{
    int32_t value;

    if(argc >= 2)
    {
        if(parse_Number(argv[1], MOTION_SPEED_MIN, MOTION_SPEED_MAX, &value) != 0)
        {
            uart_SendString("Speed must be 10 to 5000 us/s\n\r");
            return 0;
        }
        MOTION_SetSpeed((uint16_t)value);
    }
    snprintf(str_buffer, sizeof(str_buffer), "Speed: %u us/s\n\r", MOTION_GetSpeed());
    uart_SendString(str_buffer);
    return 0;
}

int cbk_stop(uint8_t argc, char **argv)
{
    stop_All();
    uart_SendString("Stopped\n\r");
    return 0;
}

static void print_Pose(uint8_t slot, const uint16_t us[N_JOINTS])
{
    uint8_t j;

    snprintf(str_buffer, sizeof(str_buffer), " %4u ", slot);
    uart_SendString(str_buffer);
    for(j = 0; j < N_JOINTS; j++)
    {
        snprintf(str_buffer, sizeof(str_buffer), " %5u", us[j]);
        uart_SendString(str_buffer);
    }
    uart_SendString("\n\r");
}

int cbk_pose(uint8_t argc, char **argv)
{
    uint16_t us[N_JOINTS];
    int32_t slot;
    uint8_t n_saved = 0;
    uint8_t i;

    if(argc == 2 && strcmp(argv[1], "list") == 0)
    {
        uart_SendString(" Pose     A0    B0    C0    A1    B1    C1\n\r");
        for(i = 0; i < POSE_SLOTS; i++)
        {
            if(POSE_Load(i, us) == 0)
            {
                print_Pose(i, us);
                n_saved++;
            }
        }
        if(n_saved == 0) uart_SendString(" (none saved)\n\r");
        return 0;
    }

    if(argc != 3 || parse_Number(argv[2], 0, POSE_SLOTS - 1, &slot) != 0)
    {
        uart_SendString("Usage: pose list  or  pose save|go|clear <0-15>\n\r");
        return 0;
    }

    if(strcmp(argv[1], "save") == 0)
    {
        MOTION_Targets(us);
        POSE_Save((uint8_t)slot, us);
        snprintf(str_buffer, sizeof(str_buffer), "Pose %u saved\n\r", (uint8_t)slot);
    }
    else if(strcmp(argv[1], "go") == 0)
    {
        if(POSE_Load((uint8_t)slot, us) != 0)
            snprintf(str_buffer, sizeof(str_buffer), "Pose %u is empty\n\r", (uint8_t)slot);
        else
        {
            SEQ_Stop();
            MOTION_MoveTo(us);
            snprintf(str_buffer, sizeof(str_buffer), "Moving to pose %u\n\r", (uint8_t)slot);
        }
    }
    else if(strcmp(argv[1], "clear") == 0)
    {
        POSE_Clear((uint8_t)slot);
        snprintf(str_buffer, sizeof(str_buffer), "Pose %u cleared\n\r", (uint8_t)slot);
    }
    else
        snprintf(str_buffer, sizeof(str_buffer), "Unknown pose action: %s\n\r", argv[1]);

    uart_SendString(str_buffer);
    return 0;
}

int cbk_play(uint8_t argc, char **argv)
{
    uint8_t slots[SEQ_MAX_STEPS];
    uint16_t us[N_JOINTS];
    uint8_t n_slots = argc - 1;
    uint8_t loop = FALSE;
    int32_t slot;
    uint8_t i;

    if(argc >= 2 && strcmp(argv[argc - 1], "loop") == 0)
    {
        loop = TRUE;
        n_slots--;
    }

    if(n_slots == 0 || n_slots > SEQ_MAX_STEPS)
    {
        uart_SendString("Usage: play <pose> [<pose> ...] [loop]  (up to 16 poses)\n\r");
        return 0;
    }

    for(i = 0; i < n_slots; i++)
    {
        if(parse_Number(argv[i + 1], 0, POSE_SLOTS - 1, &slot) != 0)
        {
            snprintf(str_buffer, sizeof(str_buffer), "Bad pose number: %s\n\r", argv[i + 1]);
            uart_SendString(str_buffer);
            return 0;
        }
        if(POSE_Load((uint8_t)slot, us) != 0)
        {
            snprintf(str_buffer, sizeof(str_buffer), "Pose %u is empty\n\r", (uint8_t)slot);
            uart_SendString(str_buffer);
            return 0;
        }
        slots[i] = (uint8_t)slot;
    }

    if(SEQ_Start(slots, n_slots, loop) != 0)
        return 1;

    snprintf(str_buffer, sizeof(str_buffer), "Playing %u poses%s; stop or Ctrl-C to stop\n\r",
        n_slots, loop ? " in a loop" : "");
    uart_SendString(str_buffer);
    return 0;
}

int cbk_dwell(uint8_t argc, char **argv)
{
    int32_t value;

    if(argc >= 2)
    {
        if(parse_Number(argv[1], 0, 60000, &value) != 0)
        {
            uart_SendString("Dwell must be 0 to 60000 ms\n\r");
            return 0;
        }
        SEQ_SetDwell((uint16_t)value);
    }
    snprintf(str_buffer, sizeof(str_buffer), "Dwell: %u ms\n\r", SEQ_GetDwell());
    uart_SendString(str_buffer);
    return 0;
}

static void manual_Draw(void);

int cbk_mode(uint8_t argc, char **argv)
{
    if(argc < 2)
    {
        uart_SendString("Insufficient number of inputs\n\r");
        return 0;
    }

    if(strcmp(argv[1], "manual") == 0)
    {
        /* change context */
        context = context_manual;
        mode_fresh = TRUE;

        uart_SendString(
            "\n\r[MANUAL MODE] up/down: move joint, left/right: change joint,"
            " enter: exit\n\r"
        );
        manual_Draw();
    }
    else if(strcmp(argv[1], "game") == 0)
    {
        /* change context */
        context = context_game;
        mode_fresh = TRUE;

        uart_SendString(
            "\n\r[GAME MODE] enter to exit; first key of each pair increases\n\r"
            "  A0: right/left   B0: up/down   C0: w/s\n\r"
            "  A1: r/f          B1: a/d       C1: . / ,\n\r"
        );
    }
    else uart_SendString("Unknown mode\n\r");
    return 0;
}

int cbk_select(uint8_t argc, char **argv)
{
    int8_t joint;
    char c;

    if(argc < 2)
    {
        uart_SendString("Insufficient number of inputs\n\r");
        return 0;
    }

    joint = joint_Parse(argv[1]);
    c = (char)toupper((unsigned char)argv[1][0]);

    if(joint >= 0)
        joint_select = (uint8_t)joint;
    /* channel A, B or C in the current group */
    else if(argv[1][1] == '\0' && c >= 'A' && c <= 'C')
        joint_select = (joint_select / 3) * 3 + (c - 'A');
    /* group 0 or 1, same channel */
    else if(argv[1][1] == '\0' && (c == '0' || c == '1'))
        joint_select = (c - '0') * 3 + joint_select % 3;
    else
    {
        uart_SendString("Unknown channel / group\n\r");
        return 0;
    }

    snprintf(str_buffer, sizeof(str_buffer), "Joint %s selected\n\r", joint_name[joint_select]);
    uart_SendString(str_buffer);
    return 0;
}

int cbk_pwm_frequency(uint8_t argc, char **argv)
{
    uint8_t grp;

    for(grp = 0; grp < 2; grp++)
    {
        PWM_FrequencyHz(&pwm_grp[grp], str_temp);
        snprintf(str_buffer, sizeof(str_buffer), "PWM Frequency group %u: %s\n\r", grp, str_temp);
        uart_SendString(str_buffer);
    }
    return 0;
}

int cbk_duty_cycle(uint8_t argc, char **argv)
{
    PWM *pwm = joint_Pwm(joint_select);
    PWM_Channel chn = joint_Channel(joint_select);

    PWM_DutyCycle(pwm, chn, str_temp);
    snprintf(str_buffer, sizeof(str_buffer), "Duty Cycle %s: %s (%u us)\n\r",
        joint_name[joint_select], str_temp, PWM_CountsToUs(pwm, PWM_Read(pwm, chn)));
    uart_SendString(str_buffer);
    return 0;
}

static const CMD cmd_table[] = {
    {"help",       "Displays this list", &cbk_help},
    {"status",     "Positions, limits and settings of all joints", &cbk_status},
    {"select",     "<joint> | A|B|C | 0|1 : joint for inc/dec/idle/manual", &cbk_select},
    {"inc",        "Move selected joint up one step", &cbk_inc_pwm_level},
    {"dec",        "Move selected joint down one step", &cbk_dec_pwm_level},
    {"step",       "[us] : show / set the inc, dec and keyboard step", &cbk_step},
    {"set",        "<joint> <us> | <joint> <angle>deg : smooth move", &cbk_set},
    {"idle",       "[all] : smooth move of selected joint (or all) to idle", &cbk_idle_pwm_level},
    {"speed",      "[us/s] : show / set peak speed of smooth moves", &cbk_speed},
    {"stop",       "Stop all motion", &cbk_stop},
    {"pose",       "list | save|go|clear <0-15> : poses kept in EEPROM", &cbk_pose},
    {"play",       "<pose> [<pose> ...] [loop] : move through saved poses", &cbk_play},
    {"dwell",      "[ms] : show / set pause at each pose while playing", &cbk_dwell},
    {"mode",       "manual | game : keyboard control; enter to exit", &cbk_mode},
    {"frequency",  "Displays the pwm frequency in Hz", &cbk_pwm_frequency},
    {"duty_cycle", "Displays the duty cycle of the selected joint", &cbk_duty_cycle},
};

#define CMD_LIST_LEN (sizeof(cmd_table) / sizeof(cmd_table[0]))

/* ------------------------------ */
/*  Manual and game mode helpers  */
/* ------------------------------ */

/* Enter (CR) leaves a mode. A lone LF also does, except straight after
   entering, where it is the tail of the CR LF that ran the mode command. */
static uint8_t mode_IsExitKey(int16_t key)
{
    uint8_t fresh = mode_fresh;
    mode_fresh = FALSE;
    return key == '\r' || (key == '\n' && !fresh);
}

static void mode_Exit(const char *msg)
{
    context = context_cli;
    uart_SendString(msg);
    cli_Prompt();
}

/* --------------------------- */
/*  Manual mode event handler  */
/* --------------------------- */

/* redraw the slider line for the selected joint */
static void manual_Draw(void)
{
    uint8_t j = joint_select;
    uint16_t min = MOTION_Min(j);
    uint16_t max = MOTION_Max(j);
    uint16_t target = MOTION_Target(j);
    uint8_t fill = SLIDER_WIDTH;
    uint8_t i;

    if(max > min)
        fill = (uint8_t)(((uint32_t)(target - min) * SLIDER_WIDTH + (max - min) / 2)
                         / (max - min));

    uart_SendByte('\r');
    uart_SendString(joint_name[j]);
    uart_SendString(" [");
    for(i = 0; i < SLIDER_WIDTH; i++) uart_SendByte(i < fill ? '=' : ' ');
    snprintf(str_temp, sizeof(str_temp), "] %4u us", target);
    uart_SendString(str_temp);
}

void manual_Keypress(int16_t key)
{
    if(mode_IsExitKey(key))
    {
        mode_Exit("\n\r");
        return;
    }

    switch(key)
    {
        case KEY_UP:
            jog(joint_select, +1);
            manual_Draw();
        break;

        case KEY_DOWN:
            jog(joint_select, -1);
            manual_Draw();
        break;

        case KEY_RIGHT:
            joint_select = (joint_select + 1) % N_JOINTS;
            manual_Draw();
        break;

        case KEY_LEFT:
            joint_select = (joint_select + N_JOINTS - 1) % N_JOINTS;
            manual_Draw();
        break;

        default:
        break;
    }
}

/* --------------------------- */
/*  Game mode event handler    */
/* --------------------------- */

/* letters are matched in lower case, so caps lock makes no difference */
static const struct GAME_KEY
{
    int16_t key;
    uint8_t joint;
    int8_t dir;
} game_keys[] = {
    /* A0 */ {KEY_RIGHT, 0, +1}, {KEY_LEFT, 0, -1},
    /* B0 */ {KEY_UP,    1, +1}, {KEY_DOWN, 1, -1},
    /* C0 */ {'w',       2, +1}, {'s',      2, -1},
    /* A1 */ {'r',       3, +1}, {'f',      3, -1},
    /* B1 */ {'a',       4, +1}, {'d',      4, -1},
    /* C1 */ {'.',       5, +1}, {',',      5, -1},
    /* C1 */ {'>',       5, +1}, {'<',      5, -1},
};

void game_Keypress(int16_t key)
{
    uint8_t i;

    if(mode_IsExitKey(key))
    {
        mode_Exit("Exit game mode\n\r");
        return;
    }

    if(key >= 'A' && key <= 'Z') key = tolower(key);

    for(i = 0; i < sizeof(game_keys) / sizeof(game_keys[0]); i++)
    {
        if(game_keys[i].key == key)
        {
            jog(game_keys[i].joint, game_keys[i].dir);
            break;
        }
    }
}

/* ---------------- */
/*  Initialization  */
/* ---------------- */

void InitUART()
{
    /* uart; interrupts on so messages can be sent during the rest of setup */
    uart_Init(1);
    sei();
}

void InitServos()
{
    uint8_t j;

    /* Initialize timer objects */
    if(TIMER_Init(&timer1, 1) != 0 || TIMER_Init(&timer4, 4) != 0 ||
       PWM_TimerConfig(&pwm_grp[0], &timer1, SERVO_PWM) != 0 ||
       PWM_TimerConfig(&pwm_grp[1], &timer4, SERVO_PWM) != 0)
    {
        uart_SendString("PWM configuration error; servos off\n\r");
        return;
    }

    /* each joint starts at idle */
    for(j = 0; j < N_JOINTS; j++)
    {
        if(MOTION_AddJoint(j, joint_Pwm(j), joint_Channel(j),
                           joint_config[j].min_us, joint_config[j].max_us,
                           joint_config[j].idle_us) != 0)
        {
            uart_SendString("Bad limits for joint ");
            uart_SendString(joint_name[j]);
            uart_SendString("; joint off\n\r");
        }
    }

    PWM_Start(&pwm_grp[0]);
    PWM_Start(&pwm_grp[1]);

    /* one motion tick per PWM period */
    MOTION_Start(&timer1, (uint8_t)(PWM_FrequencyCentiHz(&pwm_grp[0]) / 100));
}

void InitState()
{
    context = context_cli;
    cli_Init(cmd_table, CMD_LIST_LEN);
}

/* print a message that isn't a reply to a command, then restore the screen */
static void async_Message(const char *msg)
{
    uart_SendString("\n\r");
    uart_SendString(msg);
    uart_SendString("\n\r");

    if(context == context_cli) cli_Prompt();
    else if(context == context_manual) manual_Draw();
}


/* ----------- */
/*  Main Loop  */
/* ----------- */
int main()
{
    int16_t data;
    int16_t key;

    /* hardware */
    InitUART();
    InitServos();

    /* software */
    InitState();

    uart_SendString("\n\rctrl_servo ready. Type help for commands.\n\r");
    cli_Prompt();

    /* superloop */
    while(1)
    {
        /* keys typed on the terminal */
        while((data = uart_ReadByte()) >= 0)
        {
            key = cli_DecodeKey((uint8_t)data);
            if(key == KEY_NONE)
                continue;

            /* Ctrl-C stops everything, in any mode */
            if(key == KEY_CTRL_C)
            {
                stop_All();
                uart_SendString("^C stopped\n\r");
                context = context_cli;
                cli_Reset();
                cli_Prompt();
                continue;
            }

            switch(context)
            {
                case context_cli:
                    if(cli_Keypress(key) && context == context_cli) cli_Prompt();
                break;

                case context_manual:
                    manual_Keypress(key);
                break;

                case context_game:
                    game_Keypress(key);
                break;

                default:
                break;
            }
        }

        /* pose sequence playback */
        switch(SEQ_Poll())
        {
            case SEQ_EVT_FINISHED:
                async_Message("Sequence finished");
            break;

            case SEQ_EVT_ERROR:
                async_Message("Sequence stopped: a pose was cleared");
            break;

            default:
            break;
        }
    }
}
