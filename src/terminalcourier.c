#include <message.h>
#include <terminal.h>
#include <terminalcourier.h>
#include <util.h>
#include <syscall.h>
#include <name.h>
#include <debug.h>

void task_terminal_courier(int snd_tid, int fn) {
    TerminalCourierFunction tcf = (TerminalCourierFunction) fn;
    int terminaltid = WhoIs(NAME_TERMINAL);
    TerminalReq treq;

    FOREVER {
        tcf(snd_tid, &treq);
        SendTerminalRequest(terminaltid, treq.tr, treq.arg1, treq.arg2);
    }
}

static void ts_exec_terminal_send(int notifiertid, circlebuffer_t * cb_terminal){
    char rq; int  a1, a2;

    ASSERT(cb_read(cb_terminal, &rq) == 0, "Failed to read from cb");
    ASSERT(cb_read_int(cb_terminal, &a1) == 0, "Failed to read from cb");
    ASSERT(cb_read_int(cb_terminal, &a2) == 0, "Failed to read from cb");
    TerminalReq tr = {rq, a1, a2};
    TerminalCourierMessage tcm = {MESSAGE_TERMINAL_COURIER, tr};
    Reply(notifiertid, &tcm, sizeof(tcm));
}

int tc_send(TerminalCourier *tc, TerminalRequest tr, int arg1, int arg2) {
    int err = cb_write(tc->cb, tr);
    if (err) return err;
    err = cb_write_int(tc->cb, arg1);
    if (err) return err;
    err = cb_write_int(tc->cb, arg2);
    if (err) return err;

    if (likely(tc->notifiertid != -1)) {
        ts_exec_terminal_send(tc->notifiertid, tc->cb);
        tc->notifiertid = -1;
    } 
    return 0;
}

int tc_update_notifier(TerminalCourier *tc, int notifiertid){
    if (!cb_empty(tc->cb)) {
        ts_exec_terminal_send(notifiertid, tc->cb);
    } else {
        tc->notifiertid = notifiertid;
    }
    return 0;
}
