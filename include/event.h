#ifndef EVENT_H
#define EVENT_H

typedef enum {
    EVENT_CLK_3,
    EVENT_UART_1_SEND,
    EVENT_UART_2_SEND,
    EVENT_UART_1_RCV,
    EVENT_UART_2_RCV,
    NUM_EVENTS
} Event;

extern int IRQ_MAP[NUM_EVENTS];

int event_turn_off(int event, int * handled_event);

#endif //EVENT_H
