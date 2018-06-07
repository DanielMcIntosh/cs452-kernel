#include <uart.h>
#include <ts7200.h>
#include <event.h>
#include <message.h>
#include <debug.h>
#include <kernel.h>
#include <syscall.h>
#include <name.h>
#include <tasks.h>
#include <circlebuffer.h>

struct uart *uart1 = (struct uart*) UART1_BASE, *uart2 = (struct uart*) UART2_BASE;

typedef enum uartrequest{
    NOTIFY_SEND,
    NOTIFY_RCV,
    GETCH,
    PUTCH
} UARTRequest;

typedef struct rcvserver{
    circlebuffer_t rcvQ;
    circlebuffer_t getQ;
} RcvServer;

typedef struct uartmessage{
    MessageType id;
    UARTRequest request;
    int argument;
} UARTMessage;

void generic_uart_rcv_notifier(int servertid, int uart){
    int event = (uart == 1? EVENT_UART_1_RCV : EVENT_UART_2_RCV);
    UARTMessage msg = {MESSAGE_UART, NOTIFY_RCV, 0};
    ReplyMessage rm = {0, 0};

    // initialize uart happens here fn (TODO)
    FOREVER {
        msg.argument = AwaitEvent(event);
        int err = Send(servertid, &msg, sizeof(msg), &rm, sizeof(rm));
        ASSERT(err >= 0, "Error sending to server");
        ASSERT(rm.ret == 0, "Error return from server");
    }

};

void task_uart1_rcv_notifier() {
    generic_uart_rcv_notifier(WhoIs(NAME_UART1_RCV), 1);
}
void task_uart2_rcv_notifier(){
    generic_uart_rcv_notifier(WhoIs(NAME_UART2_RCV), 1);
}

void generic_uart_rcv_server(int uart){
    char rcvQ_buf[RCVQ_BUF_SIZE];
    char getQ_buf[GETQ_BUF_SIZE]; 
    circlebuffer_t cb_rcv;
    cb_init(&cb_rcv, rcvQ_buf, RCVQ_BUF_SIZE);
    circlebuffer_t cb_get;
    cb_init(&cb_get, getQ_buf, GETQ_BUF_SIZE);
    RcvServer rs = {cb_rcv, cb_get};

    UARTMessage um;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    int tid, err;

    RegisterAs(uart == 1 ? NAME_UART1_RCV : NAME_UART2_RCV);
    Create(PRIORITY_NOTIFIER, (uart == 1 ? &task_uart1_rcv_notifier : &task_uart2_rcv_notifier));
    FOREVER{
        Receive(&tid, &um, sizeof(um));
        switch(um.request){
        case NOTIFY_RCV:
        {
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm)); // reply to notifier
            rm.ret = um.argument;
            if (!cb_empty(&rs.getQ)){
                err = cb_read(&rs.getQ, (char *) &tid);
                ASSERT(err == 0, "CB Read failure");
                Reply(tid, &rm, sizeof(rm));
            } else {
                cb_write(&rs.rcvQ, rm.ret);
            }
            break;
        }
        case GETCH:
        {
            if (!cb_empty(&rs.rcvQ)){
                err = cb_read(&rs.rcvQ, (char *) &rm.ret);
                ASSERT(err == 0, "CB read failure");
                Reply(tid, &rm, sizeof(rm));
            } else {
                cb_write(&rs.getQ, tid);
            }
            break;
        } 
        default:
            PANIC("UNHANDLED REQUEST TYPE")
        }
    }
}

void task_uart1rcv(){
    generic_uart_rcv_server(1);
}
void task_uart2rcv(){
    generic_uart_rcv_server(2);
}
void task_uart1send(){}
void task_uart2send(){}

void task_init_uart_servers(){
    Create(PRIORITY_WAREHOUSE, &task_uart1send);
    Create(PRIORITY_WAREHOUSE, &task_uart2send);
    Create(PRIORITY_WAREHOUSE, &task_uart1rcv);
    Create(PRIORITY_WAREHOUSE, &task_uart2rcv);
}

