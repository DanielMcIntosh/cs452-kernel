#
# Makefile for busy-wait IO tests
#
XCC     = gcc
AS	= as
LD      = ld
CFLAGS  = -c -fPIC -Wall -I. -I../include -mcpu=arm920t -msoft-float
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -Map trains.map -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../lib

SRCFILES = $(wildcard *.c)
OBJFILES = $(patsubst %.c, %.o, $(SRCFILES))
ASMFILES = $(patsubst %.c, %.s, $(SRCFILES))

.PHONY = clean build

all: kernel.elf

%.s: %.c 
	$(XCC) -S $(CFLAGS) -o $@ $^

%.o: %.s
	$(AS) $(ASFLAGS) -o $@ $^

kernel.elf: $(OBJFILES) $(ASMFILES)
	$(LD) $(LDFLAGS) -o $@ *.o -lbwio -lgcc

clean:
	-rm -f kernel.elf *.s *.o kernel.map

build: all clean

install: all
	@echo "cp kernel.elf /u/cs452/tftp/ARM/baforbes/"
	@cp kernel.elf /u/cs452/tftp/ARM/baforbes/

rebuild: clean all
