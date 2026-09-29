/*==============================================================================
  Simulated tests for ctrl_servo, using simavr

    Runs the firmware ELF on a simulated ATmega2560 at 16 MHz, types commands
    into UART1 and checks the replies and the timer registers that drive the
    servos.

    simavr 1.6 does not model timer mode 8 (phase and frequency correct PWM,
    TOP = ICRn). The test patches it in as a single slope PWM with TOP = ICRn
    and doubles the /8 prescalar, so the overflow interrupt fires every
    2 * 8 * 20000 cycles = 20 ms, as on the real chip. Pulse widths are
    computed from the registers with the datasheet formula:

        pulse width = 2 * OCRnx * N / F_CPU

    Build and run: make -C sim run   (or make test from the top directory)
 =============================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sim_avr.h>
#include <sim_elf.h>
#include <sim_io.h>
#include <avr_uart.h>
#include <avr_timer.h>

#define F_CPU 16000000UL

/* ATmega2560 register addresses */
#define TIMSK1 0x6F
#define TCCR1A 0x80
#define TCCR1B 0x81
#define ICR1   0x86
#define TCCR4A 0xA0
#define TCCR4B 0xA1
#define ICR4   0xA6
#define DDRB   0x24
#define DDRH   0x101

/* compare register of each joint: A0 B0 C0 on timer 1, A1 B1 C1 on timer 4 */
static const uint16_t joint_ocr[6] = {0x88, 0x8A, 0x8C, 0xA8, 0xAA, 0xAC};
static const char *joint_name[6] = {"A0", "B0", "C0", "A1", "B1", "C1"};
enum { A0, B0, C0, A1, B1, C1 };

/* idle pulse widths of the old firmware: old level x 32 us */
static const uint16_t old_idle_us[6] = {72 * 32, 40 * 32, 55 * 32, 57 * 32, 40 * 32, 72 * 32};

static avr_t *avr;
static avr_irq_t *uart_in;

static char out[1 << 18];
static size_t out_len;
static size_t out_mark;

static int checks;
static int failures;

#define CHECK(cond, ...) do { \
    checks++; \
    if(!(cond)) { \
        failures++; \
        printf("  FAIL line %d: ", __LINE__); \
        printf(__VA_ARGS__); \
        printf("\n"); \
    } \
} while(0)

/* ---------------------- */
/*  Simulator plumbing    */
/* ---------------------- */

static void uart_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    if(out_len < sizeof(out) - 1)
    {
        out[out_len++] = (char)value;
        out[out_len] = '\0';
    }
}

static void run_ms(double ms)
{
    avr_cycle_count_t end = avr->cycle + (avr_cycle_count_t)(ms * F_CPU / 1000.0);
    while(avr->cycle < end)
    {
        int state = avr_run(avr);
        if(state == cpu_Done || state == cpu_Crashed)
        {
            printf("CPU stopped (state %d) at pc 0x%x\n", state, avr->pc);
            exit(2);
        }
    }
}

static double now_ms(void)
{
    return avr->cycle * 1000.0 / F_CPU;
}

static uint8_t reg8(uint16_t addr)
{
    return avr->data[addr];
}

static uint16_t reg16(uint16_t addr)
{
    return avr->data[addr] | (avr->data[addr + 1] << 8);
}

/* prescalar selected by CSn2:0 in TCCRnB */
static unsigned prescalar(uint16_t tccrnb)
{
    static const unsigned div[8] = {0, 1, 8, 64, 256, 1024, 0, 0};
    return div[reg8(tccrnb) & 0x07];
}

/* pulse width of a joint in us, from the registers */
static double pulse_us(int joint)
{
    uint16_t tccrnb = joint < 3 ? TCCR1B : TCCR4B;
    return 2.0 * reg16(joint_ocr[joint]) * prescalar(tccrnb) * 1e6 / F_CPU;
}

static uint16_t ocr(int joint)
{
    return reg16(joint_ocr[joint]);
}

static void mark(void)
{
    out_mark = out_len;
}

static int saw(const char *text)
{
    return strstr(out + out_mark, text) != NULL;
}

static int count(const char *text)
{
    int n = 0;
    const char *p = out + out_mark;
    while((p = strstr(p, text)) != NULL) { n++; p += strlen(text); }
    return n;
}

/* type bytes one per ms (a byte takes 0.52 ms at 19.2 kbps) */
static void type_bytes(const char *s, size_t n)
{
    for(size_t i = 0; i < n; i++)
    {
        avr_raise_irq(uart_in, (uint8_t)s[i]);
        run_ms(1);
    }
}

static void type(const char *s)
{
    type_bytes(s, strlen(s));
}

/* run until the firmware has sent nothing for 15 ms (a byte takes 0.52 ms) */
static void wait_quiet(double timeout_ms)
{
    double start = now_ms();
    double last_change = start;
    size_t last_len = out_len;

    while(now_ms() - start < timeout_ms)
    {
        run_ms(1);
        if(out_len != last_len)
        {
            last_len = out_len;
            last_change = now_ms();
        }
        else if(now_ms() - last_change >= 15)
            break;
    }
}

/* type a command line and wait for the whole reply */
static void cmd(const char *line)
{
    mark();
    type(line);
    type("\r");
    wait_quiet(3000);
}

/* run until every joint stops changing (or timeout); returns ms waited */
static double settle(double timeout_ms)
{
    double start = now_ms();
    uint16_t last[6];
    int quiet = 0;

    for(int j = 0; j < 6; j++) last[j] = ocr(j);
    while(now_ms() - start < timeout_ms && quiet < 5)
    {
        int changed = 0;
        run_ms(20);
        for(int j = 0; j < 6; j++)
        {
            if(ocr(j) != last[j]) changed = 1;
            last[j] = ocr(j);
        }
        quiet = changed ? 0 : quiet + 1;
    }
    return now_ms() - start;
}

/* add phase and frequency correct PWM with TOP = ICRn (mode 8) to timers 1
   and 4; see the note at the top */
static void patch_timers(void)
{
    for(avr_io_t *io = avr->io_port; io; io = io->next)
    {
        if(strcmp(io->kind, "timer") != 0) continue;
        avr_timer_t *t = (avr_timer_t *)io;
        if(t->name == '1' || t->name == '4')
        {
            t->wgm_op[8] = (avr_timer_wgm_t)AVR_TIMER_WGM_ICPWM();
            t->cs_div[2] = 4;    /* /8, doubled for the dual slope */
        }
    }
}

/* ---------------------- */
/*  Tests                 */
/* ---------------------- */

static void test_boot(void)
{
    printf("boot and PWM registers\n");
    run_ms(100);
    CHECK(strstr(out, "ctrl_servo ready") != NULL, "no banner: [%s]", out);

    CHECK(reg8(TCCR1A) == 0xA8, "TCCR1A = 0x%02x, want 0xA8 (COM1A1|COM1B1|COM1C1, WGM11:0 = 00)", reg8(TCCR1A));
    CHECK(reg8(TCCR1B) == 0x12, "TCCR1B = 0x%02x, want 0x12 (WGM13, CS12:0 = 010 -> /8)", reg8(TCCR1B));
    CHECK(reg16(ICR1) == 20000, "ICR1 = %u, want 20000", reg16(ICR1));
    CHECK(reg8(TCCR4A) == 0xA8, "TCCR4A = 0x%02x", reg8(TCCR4A));
    CHECK(reg8(TCCR4B) == 0x12, "TCCR4B = 0x%02x", reg8(TCCR4B));
    CHECK(reg16(ICR4) == 20000, "ICR4 = %u", reg16(ICR4));
    CHECK(reg8(TIMSK1) & 0x01, "timer 1 overflow interrupt not enabled");
    CHECK((reg8(DDRB) & 0xE0) == 0xE0, "pins 11-13 not outputs, DDRB = 0x%02x", reg8(DDRB));
    CHECK((reg8(DDRH) & 0x38) == 0x38, "pins 6-8 not outputs, DDRH = 0x%02x", reg8(DDRH));

    double period_ms = 2.0 * prescalar(TCCR1B) * reg16(ICR1) * 1000.0 / F_CPU;
    CHECK(period_ms == 20.0, "PWM period %.3f ms, want 20 ms (50 Hz)", period_ms);

    for(int j = 0; j < 6; j++)
        CHECK(pulse_us(j) == old_idle_us[j], "%s idle pulse %.1f us, old firmware gave %u us",
              joint_name[j], pulse_us(j), old_idle_us[j]);
}

static void test_frequency_duty(void)
{
    printf("frequency and duty_cycle\n");
    cmd("frequency");
    CHECK(saw("group 0: 50.00 Hz") && saw("group 1: 50.00 Hz"), "got [%s]", out + out_mark);
    cmd("duty_cycle");
    CHECK(saw("Duty Cycle B0: 6.40 % (1280 us)"), "got [%s]", out + out_mark);
}

static void test_smooth_move(void)
{
    printf("set: smooth, speed limited, non-blocking\n");
    cmd("speed 1000");
    mark();
    type("set B0 1600\r");
    double start = now_ms();
    uint16_t prev = ocr(B0);
    int max_step = 0;
    int monotonic = 1;
    double arrived = -1;
    int answered_while_moving = 0;

    while(now_ms() - start < 1000)
    {
        run_ms(1);
        uint16_t v = ocr(B0);
        if(v < prev) monotonic = 0;
        if(v - prev > max_step) max_step = v - prev;
        prev = v;
        if(v == 1600 && arrived < 0) arrived = now_ms() - start;

        /* the main loop keeps answering while the joint moves */
        if(!answered_while_moving && now_ms() - start > 100 && now_ms() - start < 101)
        {
            size_t m = out_len;
            type("frequency\r");
            run_ms(40);
            answered_while_moving = strstr(out + m, "50.00 Hz") != NULL && ocr(B0) < 1600;
            prev = ocr(B0);
        }
    }
    CHECK(saw("B0 moving to 1600 us"), "got [%s]", out + out_mark);
    CHECK(monotonic, "B0 went backwards");
    /* 320 us at a 1000 us/s peak: ceil(320 * 1.5 * 50 / 1000) = 24 ticks = 480 ms */
    CHECK(arrived > 440 && arrived < 540, "arrived after %.0f ms, want ~480 ms", arrived);
    /* peak 1000 us/s = 20 us per 20 ms tick */
    CHECK(max_step <= 21, "largest step %d us per tick, want <= 21", max_step);
    CHECK(answered_while_moving, "no reply to frequency while moving");
}

static void test_limits_and_degrees(void)
{
    printf("set: limits, degrees, bad input\n");
    cmd("speed 5000");
    cmd("set B0 3000");
    CHECK(saw("B0 moving to 1600 us (joint limit)"), "got [%s]", out + out_mark);
    settle(3000);
    CHECK(ocr(B0) == 1600, "B0 = %u", ocr(B0));

    cmd("set b0 100");
    settle(3000);
    CHECK(ocr(B0) == 1216, "B0 = %u, want min 1216", ocr(B0));

    cmd("set A0 0deg");
    settle(3000);
    CHECK(ocr(A0) == 1376, "A0 0deg = %u us, want 1376", ocr(A0));
    cmd("set A0 -90deg");
    settle(3000);
    CHECK(ocr(A0) == 448, "A0 -90deg = %u us, want 448", ocr(A0));

    cmd("set XX 1500");
    CHECK(saw("Unknown joint"), "got [%s]", out + out_mark);
    cmd("set B0 abc");
    CHECK(saw("Bad value"), "got [%s]", out + out_mark);
    cmd("set B0 15x");
    CHECK(saw("Bad value"), "got [%s]", out + out_mark);
    cmd("set B0");
    CHECK(saw("Usage: set"), "got [%s]", out + out_mark);
}

static void test_select_jog(void)
{
    printf("select, inc, dec, step\n");
    cmd("idle all");
    settle(5000);
    for(int j = 0; j < 6; j++)
        CHECK(ocr(j) == old_idle_us[j], "%s = %u after idle all", joint_name[j], ocr(j));

    cmd("select b0");
    CHECK(saw("Joint B0 selected"), "got [%s]", out + out_mark);
    cmd("inc");
    CHECK(saw("B0 target 1312 us"), "got [%s]", out + out_mark);
    run_ms(50);
    CHECK(ocr(B0) == 1312, "B0 = %u after inc", ocr(B0));

    cmd("step 10");
    CHECK(saw("Step: 10 us"), "got [%s]", out + out_mark);
    cmd("dec");
    run_ms(50);
    CHECK(ocr(B0) == 1302, "B0 = %u after dec", ocr(B0));
    cmd("step 0");
    CHECK(saw("Step must be"), "got [%s]", out + out_mark);
    cmd("step 32");

    cmd("select C");
    CHECK(saw("Joint C0 selected"), "got [%s]", out + out_mark);
    cmd("select 1");
    CHECK(saw("Joint C1 selected"), "got [%s]", out + out_mark);
    cmd("select Z");
    CHECK(saw("Unknown channel / group"), "got [%s]", out + out_mark);
    cmd("select B0");

    /* idle for the selected joint */
    cmd("idle");
    CHECK(saw("B0 moving to idle (1280 us)"), "got [%s]", out + out_mark);
    settle(2000);
    CHECK(ocr(B0) == 1280, "B0 = %u after idle", ocr(B0));

    /* jog far past the limit: clamped */
    cmd("step 500");
    cmd("inc");
    settle(2000);
    CHECK(ocr(B0) == 1600, "B0 = %u, want clamp at 1600", ocr(B0));
    cmd("step 32");
    cmd("idle");
    settle(2000);
}

static void test_poses_and_play(void)
{
    printf("poses in EEPROM, coordinated moves, play\n");
    cmd("speed 1000");
    cmd("pose clear 2");
    cmd("pose clear 3");
    cmd("pose clear 5");

    cmd("pose go 5");
    CHECK(saw("Pose 5 is empty"), "got [%s]", out + out_mark);

    /* pose 2: A0 and B1 moved away from idle */
    cmd("set A0 1000");
    settle(5000);
    cmd("set B1 2000");
    settle(5000);
    cmd("pose save 2");
    CHECK(saw("Pose 2 saved"), "got [%s]", out + out_mark);

    cmd("idle all");
    settle(8000);
    cmd("pose save 3");

    cmd("pose list");
    CHECK(saw("    2   1000  1280  1760  1824  2000  2304"), "pose 2 not listed: [%s]", out + out_mark);
    CHECK(saw("    3   2304  1280  1760  1824  1280  2304"), "pose 3 not listed: [%s]", out + out_mark);

    /* coordinated: A0 moves -1304 us, B1 +720 us, and at every moment both
       have done the same fraction of their move */
    type("pose go 2\r");
    double start = now_ms();
    double a0_end = -1, b1_end = -1;
    double max_gap = 0;
    while(now_ms() - start < 5000)
    {
        run_ms(1);
        double t = now_ms() - start;
        double a0_done = (2304.0 - ocr(A0)) / 1304.0;
        double b1_done = (ocr(B1) - 1280.0) / 720.0;
        double gap = a0_done > b1_done ? a0_done - b1_done : b1_done - a0_done;
        if(gap > max_gap) max_gap = gap;
        if(a0_end < 0 && ocr(A0) == 1000) a0_end = t;
        if(b1_end < 0 && ocr(B1) == 2000) b1_end = t;
    }
    CHECK(a0_end > 0 && b1_end > 0, "pose 2 not reached: A0 %u B1 %u", ocr(A0), ocr(B1));
    CHECK(max_gap < 0.01, "joints out of step by %.1f%% of their move", max_gap * 100);
    CHECK(a0_end - b1_end <= 1 && b1_end - a0_end <= 1, "joints finished apart: A0 %.0f ms, B1 %.0f ms", a0_end, b1_end);
    /* 1304 us at a 1000 us/s peak: ceil(1304 * 1.5 * 50 / 1000) = 98 ticks */
    CHECK(a0_end > 1880 && a0_end < 1980, "pose move took %.0f ms, want ~1960 ms", a0_end);

    /* poses survive a reset (power cycle) */
    avr_reset(avr);
    out_mark = out_len;
    run_ms(100);
    CHECK(saw("ctrl_servo ready"), "no banner after reset");
    CHECK(ocr(A0) == 2304, "A0 = %u after reset, want idle", ocr(A0));
    cmd("pose list");
    CHECK(saw("    2   1000  1280  1760  1824  2000  2304"), "pose 2 lost on reset: [%s]", out + out_mark);

    /* play 2 then 3 */
    cmd("dwell 200");
    CHECK(saw("Dwell: 200 ms"), "got [%s]", out + out_mark);
    cmd("play 2 3");
    CHECK(saw("Playing 2 poses"), "got [%s]", out + out_mark);
    uint16_t a0_min = ocr(A0);
    start = now_ms();
    while(now_ms() - start < 10000 && !saw("Sequence finished"))
    {
        run_ms(10);
        if(ocr(A0) < a0_min) a0_min = ocr(A0);
    }
    CHECK(saw("Sequence finished"), "sequence did not finish: [%s]", out + out_mark);
    CHECK(a0_min == 1000, "A0 lowest %u, want 1000 (pose 2)", a0_min);
    for(int j = 0; j < 6; j++)
        CHECK(ocr(j) == old_idle_us[j], "%s = %u at end, want pose 3", joint_name[j], ocr(j));

    cmd("play 2 5");
    CHECK(saw("Pose 5 is empty"), "got [%s]", out + out_mark);
    cmd("play 99");
    CHECK(saw("Bad pose number"), "got [%s]", out + out_mark);

    /* loop until stopped */
    cmd("play 2 3 loop");
    CHECK(saw("in a loop"), "got [%s]", out + out_mark);
    run_ms(4000);
    cmd("status");
    CHECK(saw("Playing: step"), "status while playing: [%s]", out + out_mark);
    cmd("stop");
    CHECK(saw("Stopped"), "got [%s]", out + out_mark);
    uint16_t held[6];
    for(int j = 0; j < 6; j++) held[j] = ocr(j);
    run_ms(2000);
    int still = 1;
    for(int j = 0; j < 6; j++) if(ocr(j) != held[j]) still = 0;
    CHECK(still, "joints kept moving after stop");
    cmd("status");
    CHECK(!saw("Playing"), "still playing after stop");

    /* a jog takes over from a playing sequence */
    cmd("play 2 3 loop");
    run_ms(300);
    cmd("inc");
    for(int j = 0; j < 6; j++) held[j] = ocr(j);
    run_ms(1500);
    still = 1;
    for(int j = 0; j < 6; j++) if(j != B0 && ocr(j) != held[j]) still = 0;
    CHECK(still, "sequence kept running after a jog");

    cmd("idle all");
    settle(8000);
}

static void test_ctrl_c(void)
{
    printf("Ctrl-C\n");
    cmd("speed 200");
    cmd("set B1 448");
    run_ms(500);
    mark();
    type("\x03");
    run_ms(50);
    CHECK(saw("^C stopped"), "got [%s]", out + out_mark);
    uint16_t held = ocr(B1);
    run_ms(1000);
    CHECK(ocr(B1) == held && held > 448 && held < 1280, "B1 %u -> %u after Ctrl-C", held, ocr(B1));
    cmd("speed 5000");
    cmd("idle all");
    settle(5000);
}

static void test_game_mode(void)
{
    printf("game mode keys\n");
    cmd("step 32");
    cmd("mode game");
    CHECK(saw("[GAME MODE]") && saw("A1: r/f"), "got [%s]", out + out_mark);

    struct { const char *keys; int joint; int delta; } cases[] = {
        {"r", A1, +32}, {"R", A1, +32},     /* caps lock no longer reverses r/f */
        {"f", A1, -32}, {"F", A1, -32},
        {"\x1b[A", B0, +32}, {"\x1b[B", B0, -32},
        {"\x1b[C", A0, -0}, {"\x1b[D", A0, -32},   /* A0 idles at its max: right is clamped */
        {"\x1b[C", A0, +32},
        {"w", C0, +32}, {"S", C0, -32},
        {"a", B1, +32}, {"D", B1, -32},
        {".", C1, -0}, {",", C1, -32}, {">", C1, +32}, {"<", C1, -32}, {".", C1, +32},
        {"\x1bOA", B0, +32},                /* application cursor mode arrows */
        {"\x1b[5~w", C0, +32},              /* page up is ignored, w still works */
        {"\x1b[1;5B", B0, -32},             /* ctrl + down */
    };
    for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        uint16_t before = ocr(cases[i].joint);
        type(cases[i].keys);
        run_ms(60);
        int delta = (int)ocr(cases[i].joint) - (int)before;
        CHECK(delta == cases[i].delta, "key %zu (%s) moved %s by %d, want %d",
              i, cases[i].keys[0] == 0x1b ? "ESC..." : cases[i].keys,
              joint_name[cases[i].joint], delta, cases[i].delta);
    }

    mark();
    type("\r");
    wait_quiet(3000);
    CHECK(saw("Exit game mode") && saw("> "), "got [%s]", out + out_mark);
    cmd("idle all");
    settle(5000);
}

static void test_manual_mode(void)
{
    printf("manual mode slider\n");
    cmd("select B0");
    cmd("mode manual");
    /* B0 idle 1280 in 1216..1600: 64/384 of 40 cells = 7 */
    CHECK(saw("[MANUAL MODE]") && saw("\rB0 [=======                                 ] 1280 us"),
          "got [%s]", out + out_mark);

    mark();
    type("\x1b[A");
    run_ms(60);
    CHECK(saw("\rB0 [==========                              ] 1312 us"), "got [%s]", out + out_mark);
    CHECK(ocr(B0) == 1312, "B0 = %u", ocr(B0));

    mark();
    type("\x1b[C");
    run_ms(60);
    CHECK(saw("\rC0 ["), "right arrow did not switch to C0: [%s]", out + out_mark);

    mark();
    type("\x1b[D\x1b[B");
    run_ms(100);
    CHECK(saw("\rB0 [") && ocr(B0) == 1280, "left + down: B0 = %u [%s]", ocr(B0), out + out_mark);

    mark();
    type("\r");
    wait_quiet(3000);
    CHECK(saw("> "), "no prompt after leaving manual mode");
    cmd("frequency");
    CHECK(saw("50.00 Hz"), "cli not back: [%s]", out + out_mark);
}

static void test_cli(void)
{
    printf("command line\n");

    /* more words than argv can hold used to write past the end of argv */
    mark();
    for(int i = 0; i < 38; i++) type("a ");
    type("\r");
    wait_quiet(3000);
    CHECK(saw("Too many arguments"), "got [%s]", out + out_mark);
    cmd("frequency");
    CHECK(saw("50.00 Hz"), "cli broken after long line: [%s]", out + out_mark);

    /* empty enter after a command with leading spaces used to re-run it */
    cmd("  help");
    CHECK(saw("List of commands"), "leading spaces: [%s]", out + out_mark);
    cmd("");
    CHECK(!saw("List of commands") && !saw("not found"), "empty line did something: [%s]", out + out_mark);

    /* DEL and BS both erase */
    cmd("helx\x7Fp");
    CHECK(saw("List of commands"), "DEL: [%s]", out + out_mark);
    cmd("freqq\buency");
    CHECK(saw("50.00 Hz"), "BS: [%s]", out + out_mark);

    /* CR LF runs the command once */
    mark();
    type("frequency\r\n");
    wait_quiet(3000);
    CHECK(count("group 0: 50.00 Hz") == 1 && !saw("not found"), "CR LF: [%s]", out + out_mark);

    /* LF alone works as enter */
    mark();
    type("frequency\n");
    wait_quiet(3000);
    CHECK(count("group 0: 50.00 Hz") == 1, "LF: [%s]", out + out_mark);

    /* arrow keys are not typed into the line */
    cmd("\x1b[Afrequency");
    CHECK(saw("50.00 Hz") && !saw("[A"), "arrow in cli: [%s]", out + out_mark);

    cmd("bogus");
    CHECK(saw("bogus: command not found"), "got [%s]", out + out_mark);

    /* over-long line: extra characters are refused, no overflow */
    mark();
    for(int i = 0; i < 100; i++) type("x");
    type("\r");
    wait_quiet(3000);
    CHECK(saw("\a") && saw(": command not found"), "long line: [%.100s]", out + out_mark);
    cmd("frequency");
    CHECK(saw("50.00 Hz"), "cli broken after overlong line");

    /* a burst of bytes with no gaps: the RX ring buffer keeps them all */
    mark();
    const char *burst = "frequency\rfrequency\rfrequency\rfrequency\rfrequency\r";
    for(const char *p = burst; *p; p++) avr_raise_irq(uart_in, (uint8_t)*p);
    wait_quiet(3000);
    CHECK(count("group 1: 50.00 Hz") == 5, "burst: %d of 5 answered", count("group 1: 50.00 Hz"));

    cmd("status");
    CHECK(saw("*B0") && saw("Speed:") && saw("Motion: stopped"), "status: [%s]", out + out_mark);

    cmd("help");
    CHECK(saw("pose") && saw("play") && saw("Ctrl-C"), "help: [%s]", out + out_mark);
}

int main(int argc, char *argv[])
{
    elf_firmware_t firmware;
    const char *elf = argc > 1 ? argv[1] : "../ctrl_servo.elf";

    memset(&firmware, 0, sizeof(firmware));
    if(elf_read_firmware(elf, &firmware) != 0)
    {
        printf("cannot read %s\n", elf);
        return 2;
    }

    avr = avr_make_mcu_by_name("atmega2560");
    if(!avr)
    {
        printf("simavr has no atmega2560\n");
        return 2;
    }
    avr_init(avr);
    avr->frequency = F_CPU;
    avr->log = LOG_ERROR;
    avr_load_firmware(avr, &firmware);
    patch_timers();

    /* UART1: capture output, don't echo it to the console */
    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('1'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('1'), &flags);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT),
                            uart_out_hook, NULL);
    uart_in = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_INPUT);

    test_boot();
    test_frequency_duty();
    test_smooth_move();
    test_limits_and_degrees();
    test_select_jog();
    test_poses_and_play();
    test_ctrl_c();
    test_game_mode();
    test_manual_mode();
    test_cli();

    printf("\n%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
