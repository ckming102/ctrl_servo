# Control Servo

Firmware for an ATmega2560 (Arduino Mega) that drives 6 servos, a 6 DOF arm, from a serial terminal. Joints can be jogged from the keyboard, moved smoothly to a position, saved as poses in EEPROM and played back as sequences.

## Hardware

- ATmega2560 at 16 MHz.
- Terminal on UART1: TX1 is pin 18, RX1 is pin 19. Connect a USB to TTL module and use GTKTerm (or any terminal) at 19200 baud, 8N1.
- Servo pulses are 50 Hz with 1 µs resolution (timer prescalar 8, TOP 20000, phase and frequency correct PWM).

| Joint | Timer / output | Pin | Min µs | Idle µs | Max µs |
|-------|----------------|-----|--------|---------|--------|
| A0    | Timer 1 / OC1A | 11  | 448    | 2304    | 2304   |
| B0    | Timer 1 / OC1B | 12  | 1216   | 1280    | 1600   |
| C0    | Timer 1 / OC1C | 13  | 1760   | 1760    | 2080   |
| A1    | Timer 4 / OC4A | 6   | 1760   | 1824    | 2080   |
| B1    | Timer 4 / OC4B | 7   | 448    | 1280    | 2304   |
| C1    | Timer 4 / OC4C | 8   | 1632   | 2304    | 2304   |

Limits live in `joint_config` in `ctrl_servo.c`. No command can drive a joint outside them. At power-on every joint goes straight to idle.

## Build and flash

```sh
make            # ctrl_servo.hex
make size       # flash / RAM use
make program    # avrdude via /dev/ttyUSB0
make test       # simulated tests, needs simavr (apt install libsimavr-dev)
```

Tested with avr-gcc 7.3. The build uses `-fno-common`, which is the default from GCC 10, so the old multiple-definition link errors can't come back.

## Commands

| Command | What it does |
|---------|--------------|
| `help` | List commands |
| `status` | Position, target and limits of every joint, plus speed, step, dwell and playback state |
| `select <joint>` | Pick the joint for `inc`, `dec`, `idle`, `duty_cycle` and manual mode: `B0`, or `A`/`B`/`C` (channel), or `0`/`1` (group) |
| `inc`, `dec` | Move the selected joint one step |
| `step [us]` | Show or set the step used by `inc`/`dec` and the keyboard modes (default 32 µs, about 3°) |
| `set <joint> <us>` | Smooth move of one joint, e.g. `set B0 1500` |
| `set <joint> <angle>deg` | Same, in servo degrees, e.g. `set A0 -30deg` (-90..90° = 448..2304 µs) |
| `idle [all]` | Smooth move of the selected joint (or all joints) to idle |
| `speed [us/s]` | Show or set the peak speed of smooth moves (default 1000 µs/s) |
| `stop` | Stop all motion and any playback |
| `pose list` | Show saved poses |
| `pose save <n>` | Save the current joint targets as pose `n` (0-15) |
| `pose go <n>` | Smooth move to pose `n` |
| `pose clear <n>` | Erase pose `n` |
| `play <n> [<n> ...] [loop]` | Move through saved poses in order, optionally repeating |
| `dwell [ms]` | Show or set the pause at each pose while playing (default 500 ms) |
| `mode manual` | Slider for the selected joint: up/down moves it, left/right changes joint, enter exits |
| `mode game` | Every joint on its own keys (below); enter exits |
| `frequency` | PWM frequency of both timers |
| `duty_cycle` | Duty cycle and pulse width of the selected joint |

**Ctrl-C** stops all motion and returns to the command line from any mode.

Game mode keys. The first key of each pair increases the pulse width, and Caps Lock makes no difference.

| Joint | Keys |
|-------|------|
| A0 | right / left |
| B0 | up / down |
| C0 | w / s |
| A1 | r / f |
| B1 | a / d |
| C1 | . / , (or > / <) |

### Example: record and play back a motion

```
> set A0 1000
> set B1 2000
> pose save 1
> idle all
> pose save 2
> dwell 300
> play 1 2 loop
> stop
```

## How motion works

The timer 1 overflow interrupt fires once per PWM period (50 Hz). `motion.c` uses it to move each joint towards its target and write the new pulse widths, so the main loop never waits for a servo.

- **Jogs** (`inc`, `dec`, keyboard modes) change one joint's target. The joint follows at up to 40 µs per tick.
- **Smooth moves** (`set`, `idle`, `pose go`, `play`) start and stop every joint together. Each move eases in and out, and the joint with the furthest to go peaks at the set speed.
- Any jog, `set`, `idle` or `pose go` stops a sequence that is playing, so the keyboard always wins.

Poses are kept in EEPROM with a checksum, so they survive power cycles and re-flashing. A pose is clamped to the current joint limits when it is played.

## Code layout

| File | Contents |
|------|----------|
| `ctrl_servo.c` | Joint setup, commands, manual and game modes, main loop |
| `motion.c/h` | Joints, limits, 50 Hz tick, jogs and smooth moves |
| `pose.c/h` | Poses in EEPROM and sequence playback |
| `pwm.c/h` | PWM on a 16 bit timer (mode 8), pulse width conversion, frequency / duty cycle |
| `timer.c/h` | 16 bit timer registers and overflow callbacks |
| `uart.c/h` | Interrupt driven UART with RX and TX ring buffers |
| `cmd.c/h` | Command table, line editing, arrow key decoding |
| `sim/` | simavr test harness (`make test`) |

## Tests

`make test` runs the firmware in simavr on a simulated ATmega2560. It types commands into UART1 and checks the replies and the timer registers: PWM setup, pulse widths, move timing and coordination, limits, poses surviving a reset, playback, Ctrl-C, keyboard modes and command-line edge cases. simavr 1.6 does not model timer mode 8, so the harness patches it in (see the top of `sim/test_ctrl_servo.c`).

## TODO

- Acceleration limit for jogs (they are only speed limited).
- Joint-space moves only; no inverse kinematics.
