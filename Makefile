MCU=atmega2560
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os
LDFLAGS=-Wl,-gc-sections -Wl,-relax
CC=avr-gcc
TARGET=ctrl_servo
OBJECT_FILES=uart.o cmd.o timer.o pwm.o ctrl_servo.o

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.elf *.hex

%.hex: %.elf
	avr-objcopy -R .eeprom -O ihex $< $@

%.elf: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

program: $(TARGET).hex
	avrdude -D -p m2560 -c stk500v2 -P /dev/ttyUSB0 -b 115200 -F -U flash:w:$(TARGET).hex