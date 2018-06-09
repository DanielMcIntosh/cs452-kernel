#include <event.h>
#include <vic.h>
#include <debug.h>
#include <clock.h>
#include <uart.h>
#include <ts7200.h>
#include <debug.h>

int IRQ_MAP[NUM_EVENTS] = {
    [EVENT_CLK_3] = 51,
    [EVENT_UART_1_SEND] = 52,
    [EVENT_UART_1_RCV] = 52,
    [EVENT_UART_2_SEND] = 54,
    [EVENT_UART_2_RCV] = 54
};

int event_turn_off(int event, int * handled_event) {
    int data = 0;
    switch(event) {
        case EVENT_CLK_3:
        {
            clk3->clear = 0;
            *handled_event = event;
            break;
        }
        case EVENT_UART_1_SEND:
        case EVENT_UART_1_RCV:
        {
            if (uart1->intidclr & UART_RIS_MASK) {
                 // recieve interrupt
                data = uart1->data;
                *handled_event = EVENT_UART_1_RCV;
            } else if (uart1->intidclr & UART_TIS_MASK) {
                // transmit interrupt
                uart1->ctrl &= ~TIEN_MASK; 
                *handled_event = EVENT_UART_1_SEND;
            }
            break;
        }
        case EVENT_UART_2_SEND:
        case EVENT_UART_2_RCV:
        {
            if (uart2->intidclr & UART_RIS_MASK) {
                 // recieve interrupt
                data = uart2->data;
                *handled_event = EVENT_UART_2_RCV;
            } else if (uart2->intidclr & UART_TIS_MASK) {
                // transmit interrupt
                uart2->ctrl &= ~TIEN_MASK; 
                *handled_event = EVENT_UART_2_SEND;
            }
            break;
        }
        default:
        {
            PANIC("unknown event");
        }
    }
    return data;
}

