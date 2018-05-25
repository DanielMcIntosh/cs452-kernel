#ifndef CLOCK_H
#define CLOCK_H

#include <ts7200.h>

#define CYCLES_PER_MILLI 508
#define CYCLES_PER_HUNDRED_MILLIS (CYCLES_PER_MILLI*100)

extern struct clock {
    volatile unsigned int load;
    volatile unsigned int value;
    volatile unsigned int control;
    volatile unsigned int clear;
} *clk1, *clk2, *clk3;

extern struct debugclock {
    volatile unsigned int value_low;
    volatile unsigned char value_high;
    volatile unsigned char enable;
} *clk4; 

#endif
