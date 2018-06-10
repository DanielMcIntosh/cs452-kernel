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
    NOTIFY_MODEM,
    GETCH,
    PUTCH
} UARTRequest;


typedef enum modemstate {
    CTS_NEGATED,
    CTS_ASSERTED,
    SEND_COMPLETE 
} CTSState;

typedef struct rcvserver{
    circlebuffer_t rcvQ;
    circlebuffer_t getQ;
} RcvServer;

typedef struct sendserver{
    circlebuffer_t txQ;
    int notifier;
    CTSState cts;
} SendServer;

typedef struct uartmessage{
    MessageType id;
    UARTRequest request;
    int argument;
} UARTMessage;

static inline void generic_uart_rcv_notifier(int servertid, int uart){
    int event = (uart == 1? EVENT_UART_1_RCV : EVENT_UART_2_RCV);
    UARTMessage msg = {MESSAGE_UART, NOTIFY_RCV, 0};
    ReplyMessage rm = {0, 0};

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
    generic_uart_rcv_notifier(WhoIs(NAME_UART2_RCV), 2);
}

void generic_uart_send_notifier(int servertid, int uart){
    int event = (uart == 1? EVENT_UART_1_SEND : EVENT_UART_2_SEND);
    struct uart *u = (uart == 1 ? uart1 : uart2);
    UARTMessage msg = {MESSAGE_UART, NOTIFY_SEND, 0};
    ReplyMessage rm = {0, 0};

    FOREVER {
        msg.argument = AwaitEvent(event); 
        int err = Send(servertid, &msg, sizeof(msg), &rm, sizeof(rm));
        ASSERT(err >= 0, "Error sending to server");
        ASSERT(rm.ret == 0, "Error return from server");
        u->data = rm.ret;
    }
}

void task_uart1_modem_notifier(){
    UARTMessage msg = {MESSAGE_UART, NOTIFY_MODEM, 0};
    ReplyMessage rm = {0, 0};

    int servertid = WhoIs(NAME_UART1_SEND);
    FOREVER{
        AwaitEvent(EVENT_UART_1_MODEM);
        int err = Send(servertid, &msg, sizeof(msg), &rm, sizeof(rm));
        ASSERT(err >= 0, "Error sending to server");
        ASSERT(rm.ret == 0, "Error return from server");
    }
}

void task_uart1_send_notifier() {
    generic_uart_send_notifier(WhoIs(NAME_UART1_SEND), 1);
}
void task_uart2_send_notifier(){
    generic_uart_send_notifier(WhoIs(NAME_UART2_SEND), 2);
}

static inline void generic_uart_rcv_server(int uart){
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

static inline void generic_uart_send_server(int uart){
    char txQ_buf[TXQ_BUF_SIZE];
    circlebuffer_t cb_tx;
    cb_init(&cb_tx, txQ_buf, TXQ_BUF_SIZE);
    SendServer ss = {cb_tx, 0, CTS_ASSERTED};

    UARTMessage um;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    int tid, err;
    RegisterAs(uart == 1 ? NAME_UART1_SEND : NAME_UART2_SEND);
    if (uart == 1) Create(PRIORITY_NOTIFIER, &task_uart1_modem_notifier);
    Create(PRIORITY_NOTIFIER, (uart == 1 ? &task_uart1_send_notifier : &task_uart2_send_notifier));
    FOREVER{
        Receive(&tid, &um, sizeof(um));
        switch(um.request){
        case NOTIFY_SEND:
        {
            rm.ret = 0;
            if (!cb_empty(&ss.txQ) && (ss.cts == CTS_ASSERTED || uart == 2)){
                err = cb_read(&ss.txQ, (char *) &rm.ret);
                ASSERT(err == 0, "CB Read failure");
                Reply(tid, &rm, sizeof(rm));
                ss.cts = SEND_COMPLETE;
            } else {
                ss.notifier = tid;
            }
            break;
        }
        case NOTIFY_MODEM:
        {
            if (!cb_empty(&ss.txQ) && ss.notifier != 0 && ss.cts == CTS_NEGATED){
                err = cb_read(&ss.txQ, (char *) &rm.ret);
                ASSERT(err == 0, "CB Read failure");
                Reply(ss.notifier, &rm, sizeof(rm));
                ss.cts = SEND_COMPLETE;
            } else {
                switch (ss.cts){
                case SEND_COMPLETE:
                case CTS_ASSERTED:
                    ss.cts = CTS_NEGATED;
                    break;
                case CTS_NEGATED:
                    ss.cts = CTS_ASSERTED;
                    break;
                default:
                    PANIC("INVALID STATE");
                }
            }
        }
        case PUTCH:
        {
            if (ss.notifier != 0 && (ss.cts == CTS_ASSERTED || uart == 2)){
                rm.ret = um.argument;
                Reply(ss.notifier, &rm, sizeof(rm));
                ss.notifier = 0;
                ss.cts = SEND_COMPLETE;
            } else {
                cb_write(&ss.txQ, um.argument);
            }
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));
            break;
        } 
        default:
            PANIC("UNHANDLED REQUEST TYPE")
        }
    }
}

void task_uart1rcv(){ // FIXME: is there a cleaner way to do this?
    RegisterAs(NAME_UART1_RCV);
    generic_uart_rcv_server(1);
}
void task_uart2rcv(){
    RegisterAs(NAME_UART2_RCV);
    generic_uart_rcv_server(2);
}
void task_uart1send(){
    RegisterAs(NAME_UART1_SEND);
    generic_uart_send_server(1);
}
void task_uart2send(){
    RegisterAs(NAME_UART2_SEND);
    generic_uart_send_server(2);
}

void task_init_uart_servers(){
    for (int i = 0; i < 2; i++){
        struct uart *u = (i == 0 ? uart1 : uart2);
        int brd = F_UARTCLK / (16 * (i == 0 ? TC_BAUD : TM_BAUD)) - 1;
        int brd_hi = brd / 256;
        int brd_lo = brd % 256;

        u->baudratehigh = brd_hi;
        u->baudratelow = brd_lo;
        u->linctrlhigh = ((u->linctrlhigh) & ~FEN_MASK) | STP2_MASK; // FIFO off, 2 stop bits
        // enable
        u->ctrl |= UARTEN_MASK | RIEN_MASK | MSIEN_MASK; //(uart == 1 ? MSIEN_MASK : 0); // TODO: disable first?
        //TODO 55 NOPs here?
    }
    Create(PRIORITY_WAREHOUSE, &task_uart1send);
    Create(PRIORITY_WAREHOUSE, &task_uart1rcv);
    Create(PRIORITY_WAREHOUSE, &task_uart2send);
    Create(PRIORITY_WAREHOUSE, &task_uart2rcv);
}

int Getc(int servertid, int channel){
    UARTMessage um = {MESSAGE_UART, GETCH, 0};
    ReplyMessage rm = {0, 0};
    int r = Send(servertid, &um, sizeof(um), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

int Putc(int servertid, int channel, char ch){
    UARTMessage um = {MESSAGE_UART, PUTCH, ch};
    ReplyMessage rm = {0, 0};
    int r = Send(servertid, &um, sizeof(um), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

