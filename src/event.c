#include <event.h>
#include <vic.h>
#include <debug.h>
#include <clock.h>
#include <uart.h>
#include <ts7200.h>
#include <debug.h>
#include <syscall.h>

int IRQ_MAP[NUM_EVENTS] = {
    [EVENT_CLK_3] = 51,
    [EVENT_UART_1_SEND] = 52,
    [EVENT_UART_1_RCV] = 52,
    [EVENT_UART_1_MODEM] = 52, // TODO split up these interrupts
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
        case EVENT_UART_1_MODEM:
        {
            if (uart1->intidclr & UART_RIS_MASK) {
                 // recieve interrupt
                data = uart1->data;
                *handled_event = EVENT_UART_1_RCV;
            } else if (uart1->intidclr & UART_TIS_MASK) {
                // transmit interrupt
                uart1->ctrl &= ~TIEN_MASK; 
                *handled_event = EVENT_UART_1_SEND;
            } else if (uart1->intidclr & UART_MIS_MASK){
                // modem status interrupt (mis)
                uart1->intidclr = 0; // clear Modem Status interrupt
                *handled_event = EVENT_UART_1_MODEM;
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

