# Simulated tests of ../ctrl_servo.elf; needs simavr (apt install libsimavr-dev)
SIMAVR_INCLUDE=/usr/include/simavr
CFLAGS=-O2 -Wall -Wextra -I$(SIMAVR_INCLUDE)
LDLIBS=-lsimavr

all: test_ctrl_servo

test_ctrl_servo: test_ctrl_servo.c

run: test_ctrl_servo
	$(MAKE) -C .. ctrl_servo.elf
	./test_ctrl_servo ../ctrl_servo.elf

clean:
	rm -f test_ctrl_servo

.PHONY: all run clean
