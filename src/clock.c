#include <clock.h>
#include <ts7200.h>
#include <debug.h>
#include <kernel.h>
#include <message.h>
#include <name.h>
#include <syscall.h>
#include <tasks.h>
#include <err.h>
#include <event.h>
#include <minheap.h>
#include <debug.h>

struct debugclock *clk4 = (struct debugclock*)TIMER4_BASE; 
struct clock *clk1 = (struct clock*)TIMER1_BASE, *clk2=(struct clock*)TIMER2_BASE, *clk3 = (struct clock*)TIMER3_BASE;

typedef enum clockrequest{
    NOTIFIER,
    TIME,
    DELAY,
    DELAYUNTIL
} ClockRequest;
    
typedef struct clockserver {
    int ticks;
    minheap_t mh;
} ClockServer;

typedef struct clockmessage{
    MessageType id;
    ClockRequest request;
    int argument;
} ClockMessage;

void task_clockserver(){
    LOG("ClockServer init\r\n");

    entry_t mh_array[50]; // TODO: array size
    ClockServer cs = {0, {mh_array, 0, 50}};
    ClockMessage cm;
    ReplyMessage rm;
    int tid, err;
    entry_t mh_entry;
    
    RegisterAs(NAME_CLOCK);
    Create(PRIORITY_NOTIFIER, &task_clocknotifier);
    FOREVER{
        Receive(&tid, &cm, sizeof(cm));
        switch(cm.request){
        case NOTIFIER:
            LOGF("Time Server: NOTIFIER\r\n");
            cs.ticks++;
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));

            // reply to all tasks to wakeup now
            err = mh_peek_min(&cs.mh, &mh_entry);
            while (err == 0 && mh_entry.value <= cs.ticks){
                bwprintf(COM2, "ITEM: %d, VALUE: %d\r\n", mh_entry.item, mh_entry.value);
                Reply(mh_entry.item, &rm, sizeof(rm));
                mh_remove_min(&cs.mh, &mh_entry);
                err = mh_peek_min(&cs.mh, &mh_entry);
            }
            break;
        case TIME:
            LOGF("Time Server: TIME\r\n");
            rm.ret = cs.ticks;
            Reply(tid, &rm, sizeof(rm));
            break;
        case DELAY:
            LOGF("Delay: %d\r\n", cm.argument);
            err = mh_add(&cs.mh, tid, cm.argument + cs.ticks);
            ASSERT(err == 0, "MINHEAP ADD ERROR",);
            break;
        case DELAYUNTIL:
            LOGF("DELAY UNTIL: %d\r\n", cm.argument);
            err = mh_add(&cs.mh, tid, cm.argument);
            ASSERT(err == 0, "MINHEAP ADD ERROR",);
            break;
        default:
           rm.ret = ERR_NOT_IMPLEMENTED;
           Reply(tid, &rm, sizeof(rm));
        }
    }
}
void task_clocknotifier(){
    LOG("ClockNotifier init \r\n");
    int clk_tid = WhoIs(NAME_CLOCK);
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = NOTIFIER;
    ReplyMessage rm;

    // Initialize Timer
    clk3->load = CYCLES_PER_TEN_MILLIS; // ?? what precision do we need
    clk3->control |= 0xD8; // TODO disable first? unclear

    FOREVER {
        AwaitEvent(EVENT_CLK_3);
        int err = Send(clk_tid, &cm, sizeof(cm), &rm, sizeof(rm));
        ASSERT(err >= 0, "Error sending to clock server",);
        ASSERT(rm.ret == 0, "Error return from clock server",);
    }
}

int Time(int tid_clk){ // FIXME: cleanup duplicated code here?
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = TIME;
    ReplyMessage rm;
    bwputstr(COM2, "Sending Time Msg");
    int r = Send(tid_clk, &cm, sizeof(cm), &rm, sizeof(rm));
    bwputstr(COM2, "Returned from sending Time Msg");
    return (r >= 0 ? rm.ret : r);
}

int Delay(int tid_clk, int ticks){
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = DELAY;
    cm.argument = ticks;
    ReplyMessage rm;
    bwputstr(COM2, "Delay Send\r\n");
    int r = Send(tid_clk, &cm, sizeof(cm), &rm, sizeof(rm));
    bwputstr(COM2, "Delay Return\r\n");
    return (r >= 0 ? rm.ret : r);
}

int DelayUntil(int tid_clk, int ticks){
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = DELAYUNTIL;
    cm.argument = ticks;
    ReplyMessage rm;
    int r = Send(tid_clk, &cm, sizeof(cm), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}
