#include <clock.h>
#include <ts7200.h>
#include <comio.h>

struct debugclock *clk4 = (struct debugclock*)TIMER4_BASE; 
struct clock *clk1 = (struct clock*)TIMER1_BASE, *clk2=(struct clock*)TIMER2_BASE, *clk3 = (struct clock*)TIMER3_BASE;
