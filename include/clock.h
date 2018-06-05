#ifndef CLOCK_H
#define CLOCK_H

#include <ts7200.h>

#define CYCLES_PER_MILLI 508
#define CYCLES_PER_TEN_MILLIS (CYCLES_PER_MILLI*10)
#define CYCLES_PER_HUNDRED_MILLIS (CYCLES_PER_MILLI*100)

#define NAME_CLOCK "CLK"
#define CLOCK_MH_SIZE 40

extern struct clock {
    volatile unsigned int load;
    volatile unsigned int value;
    //TODO: change to a union of int and struct of (carefully aligned) bit fields
    volatile unsigned int control;
    volatile unsigned int clear;
} *clk1, *clk2, *clk3;

extern struct debugclock {
    volatile unsigned int value_low;
    volatile unsigned char value_high;
    volatile unsigned char enable;
} __attribute__ ((packed)) *clk4;  //have to specify packed to ensure GCC doesn't word align enable

void task_clockserver();
void task_clocknotifier();

int Time();
int Delay(int ticks);
int DelayUntil(int ticks); //FIXME: pass in tid_clk

#endif
