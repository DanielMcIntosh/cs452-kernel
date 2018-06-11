#ifndef UART_H
#define UART_H

#define NAME_UART1_SEND "U1SND"
#define NAME_UART1_RCV  "U1RCV"
#define NAME_UART2_SEND "U2SND"
#define NAME_UART2_RCV " U2RCV"

#define RCVQ_BUF_SIZE 50 // TODO calculate these based on stack size
#define TXQ_BUF_SIZE 100
#define GETQ_BUF_SIZE 5 // expected that not many different tasks will want to pull from any given uart - most likely 1, no?

#define F_UARTCLK 7372800
#define TC_BAUD 2400
#define TM_BAUD 115200

#define UART_STR_MAX 20

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
int Puts(int servertid, char *st, int len);

void task_init_uart_servers();

#endif
