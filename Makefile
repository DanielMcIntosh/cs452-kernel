#
# Makefile for busy-wait IO tests
#
DEPDIR := .d
SRCDIR := src
BINDIR := bin
ASMDIR := asm
LIBDIR := lib
TOOLSDIR := tools
INCLUDEDIR := include
DOCSDIR := docs

DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td

$(shell mkdir -p $(DEPDIR) >/dev/null)
$(shell mkdir -p $(BINDIR) >/dev/null)
XCC     = gcc
AS	= as
LD      = ld
CFLAGS  = $(DEPFLAGS) -c -std=c99 -fPIC -Wall -I. -I $(INCLUDEDIR) -mcpu=arm920t -msoft-float
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -Map $(BINDIR)/kernel.map -N -T $(TOOLSDIR)/orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L $(LIBDIR)

SRCFILES = $(wildcard $(SRCDIR)/*.c)
SRCASM = $(wildcard $(ASMDIR)/*.s)
OBJFILES = $(patsubst $(SRCDIR)/%.c, $(BINDIR)/%.o, $(SRCFILES))
ASMFILES = $(patsubst $(SRCDIR)/%.c, $(BINDIR)/%.s, $(SRCFILES))
HANDASM = $(patsubst $(ASMDIR)/%.s, $(BINDIR)/%.o, $(SRCASM))

POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

.PHONY = clean build

all: $(BINDIR)/kernel.elf

$(BINDIR)/%.s: $(SRCDIR)/%.c $(DEPDIR)/%.d
	$(XCC) -S $(CFLAGS) -o $@ $<
	$(POSTCOMPILE)

$(BINDIR)/%.o: $(ASMDIR)/%.s
	$(AS) $(ASFLAGS) -o $@ $<

$(BINDIR)/%.o: $(SRCDIR)/%.s
	$(AS) $(ASFLAGS) -o $@ $<

$(BINDIR)/kernel.elf: $(OBJFILES) $(HANDASM) $(ASMFILES)
	$(LD) $(LDFLAGS) -o $@ $(BINDIR)/*.o -lbwio -lgcc

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d

$(DOCSDIR)/%.pdf: $(DOCSDIR)/%.tex
	pdflatex -output-directory $(DOCSDIR) $<

clean:
	-rm -f $(BINDIR)/* $(DEPDIR)/*

build: all clean

install: all
	@echo "cp $(BINDIR)/kernel.elf /u/cs452/tftp/ARM/$(shell whoami)/"
	@cp $(BINDIR)/kernel.elf /u/cs452/tftp/ARM/$(shell whoami)/

rebuild: clean all
reinstall: clean install

docs: $(DOCSDIR)/k1.pdf

include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRCS))))
