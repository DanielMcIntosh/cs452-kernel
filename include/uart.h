#ifndef UART_H
#define UART_H

#define NAME_UART1_SEND "UART1SEND"
#define NAME_UART1_RCV  "UART1RCV"
#define NAME_UART2_SEND "UART2SEND"
#define NAME_UART2_RCV " UART2RCV"

#define RCVQ_BUF_SIZE 50 // TODO calculate these based on stack size
#define SENDQ_BUF_SIZE 50
#define GETQ_BUF_SIZE 5 // expected that not many different tasks will want to pull from any given uart - most likely 1, no?


extern struct uart {
    volatile unsigned int data          : 8;
    volatile unsigned int               : 24;
    // err
    union {
        struct {
            volatile unsigned int framingerror  : 1;
            volatile unsigned int parityerror   : 1;
            volatile unsigned int breakerror    : 1;
            volatile unsigned int overrunerror  : 1;
            volatile unsigned int               : 28;
        } individual;
        volatile unsigned int collective        : 32;
    } err;


    volatile unsigned int linctrlhigh   : 7; // note: must write last
    volatile unsigned int               : 25;
    //lin ctrl mid
    volatile unsigned int baudratehigh  : 8;
    volatile unsigned int               : 24;
    //lin ctrl low
    volatile unsigned int baudratelow   : 8;
    volatile unsigned int               : 24;

    volatile unsigned int ctrl          : 8;
    volatile unsigned int               : 24;

    volatile unsigned int flag          : 8;
    volatile unsigned int               : 24;

    volatile unsigned int intidclr      : 4; // Interupt ID/Clear
    volatile unsigned int               : 28;

    volatile unsigned int dmactrl       : 3;
    volatile unsigned int               : 29;
} *uart1, *uart2;

int Getc(int servertid, int channel);
int Putc(int servertid, int channel, char ch);

void task_init_uart_servers();

#endif
