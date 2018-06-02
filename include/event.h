#ifndef EVENT_H
#define EVENT_H

typedef enum {
    EVENT_CLK_3,
    NUM_EVENTS
} Event;

extern int IRQ_MAP[NUM_EVENTS];

void event_turn_off(int event);

#endif //EVENT_H
