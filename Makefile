#
# Makefile for busy-wait IO tests
#
XCC     = gcc
AS	= as
LD      = ld
CFLAGS  = -c -std=c99 -fPIC -Wall -I. -I include -mcpu=arm920t -msoft-float
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -Map bin/trains.map -N -T tools/orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L lib

SRCFILES = $(wildcard src/*.c)
OBJFILES = $(patsubst src/%.c, bin/%.o, $(SRCFILES))
ASMFILES = $(patsubst src/%.c, bin/%.s, $(SRCFILES))

.PHONY = clean build

all: bin/kernel.elf

bin/%.s: src/%.c 
	$(XCC) -S $(CFLAGS) -o $@ $^

bin/%.o: src/%.s
	$(AS) $(ASFLAGS) -o $@ $^

bin/kernel.elf: $(OBJFILES) $(ASMFILES)
	$(LD) $(LDFLAGS) -o $@ bin/*.o -lbwio -lgcc

clean:
	-rm -f bin/*

build: all clean

install: all
	@echo "cp bin/kernel.elf /u/cs452/tftp/ARM/baforbes/"
	@cp bin/kernel.elf /u/cs452/tftp/ARM/baforbes/

rebuild: clean all
