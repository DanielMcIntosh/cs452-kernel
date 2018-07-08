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
XCC     = arm-none-eabi-gcc 
AS	= arm-none-eabi-as
LD      = arm-none-eabi-ld
CFLAGS  = $(DEPFLAGS) -c -std=c11 -fPIC -Wall -Wextra -I. -I $(INCLUDEDIR) -mcpu=arm920t -msoft-float -O3
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -Map $(BINDIR)/kernel.map -N -T $(TOOLSDIR)/orex.ld "-L/u7/c7zou/! cs452/toolchain/lib/gcc/arm-none-eabi/8.1.0" -L $(LIBDIR)

SRCFILES = $(wildcard $(SRCDIR)/*.c)
SRCASM = $(wildcard $(ASMDIR)/*.s)
SRCDOCS = $(wildcard $(DOCSDIR)/*.tex)
OBJFILES = $(patsubst $(SRCDIR)/%.c, $(BINDIR)/%.o, $(SRCFILES))
ASMFILES = $(patsubst $(SRCDIR)/%.c, $(BINDIR)/%.s, $(SRCFILES))
HANDASM = $(patsubst $(ASMDIR)/%.s, $(BINDIR)/%.o, $(SRCASM))
DOCS = $(patsubst $(DOCSDIR)/%.tex, $(DOCSDIR)/%.pdf, $(SRCDOCS))

POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

.PHONY = clean build

all: $(BINDIR)/kernel.elf $(DOCS)

$(BINDIR)/%.s: $(SRCDIR)/%.c $(DEPDIR)/%.d $(INCLUDEDIR)/%.h
	$(XCC) -S $(CFLAGS) -o $@ $<
	$(POSTCOMPILE)

$(BINDIR)/%.o: $(ASMDIR)/%.s
	$(AS) $(ASFLAGS) -o $@ $<

$(BINDIR)/%.o: $(BINDIR)/%.s
	$(AS) $(ASFLAGS) -o $@ $<

$(BINDIR)/kernel.elf: $(OBJFILES) $(HANDASM) $(ASMFILES)
	$(LD) $(LDFLAGS) -o $@ $(BINDIR)/*.o -lbwio -lgcc

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d

$(DOCSDIR)/%.pdf: $(DOCSDIR)/%.tex
	pdflatex -output-directory $(DOCSDIR) $<

clean:
	-rm -f $(filter-out $(BINDIR)/track_data.o $(BINDIR)/track_data.s,$(wildcard $(BINDIR)/*)) $(DEPDIR)/* $(DOCSDIR)/*.aux $(DOCSDIR)/*.log $(DOCSDIR)/*.out
cleand:
	-rm -f $(DOCSDIR)/*.pdf $(DOCSDIR)/*.aux $(DOCSDIR)/*.log $(DOCSDIR)/*.out


build: all clean

install: all
	@echo "cp $(BINDIR)/kernel.elf /u/cs452/tftp/ARM/$(shell whoami)/"
	@cp $(BINDIR)/kernel.elf /u/cs452/tftp/ARM/$(shell whoami)/

rebuild: clean all
reinstall: clean install

docs: $(DOCS)

include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRCS))))
