#ifndef CLOCK_H
#define CLOCK_H

#include <ts7200.h>
#include <circlebuffer.h>

#define CLOCK3_LOAD (int*)(TIMER3_BASE | LDR_OFFSET)
#define CLOCK3_VAL (int*)(TIMER3_BASE | VAL_OFFSET)
#define CLOCK3_CONTROL (int*)(TIMER3_BASE | CRTL_OFFSET)

#define CYCLES_PER_MILLI 508
#define CYCLES_PER_HUNDRED_MILLIS (CYCLES_PER_MILLI*100)

void clock_init();
int clock_read();
int clock_incremented(int clock_millis);
int output_time(int time_millis, struct circlebuffer* cb);

#endif
