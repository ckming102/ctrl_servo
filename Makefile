MCU=atmega2560
F_CPU=16000000UL
CFLAGS=-g -Os -std=gnu11 -Wall -Wextra -Wno-unused-parameter -fno-common -mcall-prologues -mmcu=$(MCU) -DF_CPU=$(F_CPU) -MMD -MP
LDFLAGS=-Wl,-gc-sections -Wl,-relax
CC=avr-gcc
TARGET=ctrl_servo
OBJECT_FILES=uart.o cmd.o timer.o pwm.o motion.o pose.o ctrl_servo.o

all: $(TARGET).hex

clean:
	rm -f *.o *.d *.hex *.elf

%.hex: %.elf
	avr-objcopy -R .eeprom -O ihex $< $@

$(TARGET).elf: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

size: $(TARGET).elf
	avr-size -C --mcu=$(MCU) $<

program: $(TARGET).hex
	avrdude -D -p m2560 -c stk500v2 -P /dev/ttyUSB0 -b 115200 -F -U flash:w:$(TARGET).hex

# simulated tests; needs simavr (apt install libsimavr-dev)
test: $(TARGET).elf
	$(MAKE) -C sim run

-include $(OBJECT_FILES:.o=.d)

.PHONY: all clean size program test
