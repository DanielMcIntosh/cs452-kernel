#include "event.h"
#include "vic.h"
#include "debug.h"
#include "clock.h"

int IRQ_MAP[NUM_EVENTS] = {
    [EVENT_CLK_3] = 51
};


void event_turn_off(int event) {
    switch(event) {
        case EVENT_CLK_3:
        {
            clk3->clear = 0;
            break;
        }
        default:
        {
            PANIC("unknown event",);
        }
    }
}

