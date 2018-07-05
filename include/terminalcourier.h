#include <terminal.h>
#include <message.h>
#include <circlebuffer.h>

#ifndef TERMINAL_COURIER_H
#define TERMINAL_COURIER_H

typedef struct treq {
    TerminalRequest tr;
    int arg1;
    int arg2;
} TerminalReq;

typedef struct tcourier {
    int notifiertid;
    circlebuffer_t* cb;
} TerminalCourier;

typedef struct terminalcouriermsg {
    MessageType id;
    TerminalReq req;
} TerminalCourierMessage;

typedef void (*TerminalCourierFunction)(int tid, TerminalReq* treq);

// NOTE: fn is a TerminalCourierFunction
void task_terminal_courier(int tid, int fn);
int tc_send(TerminalCourier *tc, TerminalRequest tr, int arg1, int arg2);
int tc_update_notifier(TerminalCourier *tc, int notifiertid);

#endif
