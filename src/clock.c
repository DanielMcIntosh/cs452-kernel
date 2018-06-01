#include <clock.h>
#include <ts7200.h>
#include <debug.h>
#include <kernel.h>
#include <message.h>
#include <name.h>
#include <syscall.h>
#include <tasks.h>
#include <err.h>

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
    //TODO: array-based minheap here probably
} ClockServer;

typedef struct clockmessage{
    MessageType id;
    ClockRequest request;
    int argument;
} ClockMessage;

void task_clockserver(){
    LOG("ClockServer init\r\n");
    ClockServer cs;
    cs.ticks = 0;
    ClockMessage cm;
    ReplyMessage rm;
    int tid;
    
    RegisterAs(NAME_CLOCK);
    Create(PRIORITY_NOTIFIER, &task_clocknotifier);
    FOREVER{
        Receive(&tid, &cm, sizeof(cm));
        switch(cm.request){
        case NOTIFIER:
            cs.ticks++;
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));
            // TODO: wakeup any tasks that need it here.
            break;
        case TIME:
            rm.ret = cs.ticks;
            Reply(tid, &rm, sizeof(rm));
            break;
        case DELAY:
        case DELAYUNTIL:
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
    FOREVER {
        AwaitEvent(0); // TODO
        int err = Send(clk_tid, &cm, sizeof(cm), &rm, sizeof(rm));
        ASSERT(err == 0, "Error sending to clock server",);
        ASSERT(rm.ret == 0, "Error return from clock server",);
    }
}

int Time(int tid_clk){ // FIXME: cleanup generated code here?
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = TIME;
    ReplyMessage rm;
    int r = Send(tid_clk, &cm, sizeof(cm), &rm, sizeof(rm));
    return (r == 0 ? rm.ret : r);
}

int Delay(int tid_clk, int ticks){
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = DELAY;
    cm.argument = ticks;
    ReplyMessage rm;
    int r = Send(tid_clk, &cm, sizeof(cm), &rm, sizeof(rm));
    return (r == 0 ? rm.ret : r);
}

int DelayUntil(int tid_clk, int ticks){
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = DELAYUNTIL;
    cm.argument = ticks;
    ReplyMessage rm;
    int r = Send(tid_clk, &cm, sizeof(cm), &rm, sizeof(rm));
    return (r == 0 ? rm.ret : r);
}
