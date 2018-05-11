#include <clock.h>
#include <ts7200.h>
#include <comio.h>

int clock_load(int load){
    *CLOCK3_LOAD = load;
    return load;
}

int clock_read() {
    int dta = *CLOCK3_VAL;
    return dta;
}

void clock_init(){
    clock_load(CYCLES_PER_HUNDRED_MILLIS);
    *CLOCK3_CONTROL |= ((ENABLE_MASK | MODE_MASK | CLKSEL_MASK) | 0xFF) ; // Enabled = 1, Mode = 1 (Periodic), Clock = 1 (508 kHz)
}

int clock_incremented(int clock_millis){
    static int prev_time = 214724899; // INT_MAX
    int r = 0;

    if (prev_time < clock_millis) {
        r= 1;
    }

    prev_time = clock_millis;

    return r;
}
