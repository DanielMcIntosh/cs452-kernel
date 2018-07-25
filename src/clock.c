#include <clock.h>
#include <ts7200.h>
#include <debug.h>
#include <kernel.h>
#include <message.h>
#include <name.h>
#include <syscall.h>
#include <err.h>
#include <event.h>
#include <minheap.h>
#include <debug.h>
#include <terminal.h>
#include <util.h>
#include <features.h>

struct debugclock *clk4 = (struct debugclock*)TIMER4_BASE; 
struct clock *clk1 = (struct clock*)TIMER1_BASE, *clk2=(struct clock*)TIMER2_BASE, *clk3 = (struct clock*)TIMER3_BASE;

typedef enum clockrequest{
    NOTIFIER,
    TIME,
    DELAY,
    DELAYUNTIL,
    UPDATE_TIMEOUT,
    REQUEST_TIMEOUT
} ClockRequest;
    
typedef struct clockserver {
    unsigned int ticks;
    minheap_t mh;
} ClockServer;

typedef struct clockmessage{
    MessageType id;
    ClockRequest request;
    int argument;
} ClockMessage;

typedef struct timeoutmessage{
    int caller_tid;
    unsigned int ticks;
} TimeoutMessage;

static void __attribute__((noreturn)) task_timeout(int clock_tid);
static void __attribute__((noreturn)) task_clocknotifier(int clk_tid);

void __attribute__((noreturn)) task_clockserver(){
    LOG("ClockServer init\r\n");
    RegisterAs(NAME_CLOCK);

    int mytid = MyTid();
    CreateWithArgument(PRIORITY_NOTIFIER, &task_clocknotifier, mytid);
    //create task_timeout with a slightly lower priority than warehouse since
    // it's not _quite_ as important as eg. the clock server
    CreateWithArgument(PRIORITY_WAREHOUSE+1, &task_timeout, mytid);

    entry_t mh_array[CLOCK_MH_SIZE];
    ClockServer cs = {0, {mh_array, 0, CLOCK_MH_SIZE}};
    ClockMessage cm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    int tid, err;
    entry_t mh_entry;
    unsigned int timeout = INT_MAX;
    int timeout_tid = -1;
    
    FOREVER {
        Receive(&tid, &cm, sizeof(cm));
        switch(cm.request){
        case NOTIFIER:
            LOGF("Time Server: NOTIFIER\r\n");
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));
            cs.ticks++;

            // reply to all tasks to wakeup now
            err = mh_peek_min(&cs.mh, &mh_entry);
            while (err == 0 && mh_entry.value <= cs.ticks){
                LOGF("Clock Server replying to: ITEM: %d, VALUE: %d\r\n", mh_entry.item, mh_entry.value);
                Reply(mh_entry.item, &rm, sizeof(rm));
                mh_remove_min(&cs.mh, &mh_entry);
                err = mh_peek_min(&cs.mh, &mh_entry);
            }
            if (unlikely(timeout <= cs.ticks)) {
                Reply(timeout_tid, NULL, 0);
                timeout = INT_MAX;
            }
            break;
        case TIME:
            LOGF("Time Server: TIME\r\n");
            rm.ret = cs.ticks;
            Reply(tid, &rm, sizeof(rm));
            break;
        case DELAY:
            LOGF("Delay: %d\r\n", cm.argument);
            ASSERT(cm.argument > 0, "Delay %d <= 0", cm.argument);
            err = mh_add(&cs.mh, tid, cm.argument + cs.ticks);
            ASSERT(err == 0, "MINHEAP ADD ERROR");
            break;
        case DELAYUNTIL:
            ASSERT(DEBUG_CLOCK && (unsigned int) cm.argument > cs.ticks, "Delay %d <= %d cs.ticks ", cm.argument, cs.ticks);
            if ((unsigned int) cm.argument <= cs.ticks){
                err = Reply(tid, &rm, sizeof(rm));
            }
            LOGF("DELAY UNTIL: %d\r\n", cm.argument);
            err = mh_add(&cs.mh, tid, cm.argument);
            ASSERT(err == 0, "MINHEAP ADD ERROR");
            break;
        case UPDATE_TIMEOUT:
            LOGF("Update Timeout: %d\r\n", cm.argument);
            timeout = cm.argument;
            Reply(tid, NULL, 0);
            break;
        case REQUEST_TIMEOUT:
            LOGF("Request Timeout: %d\r\n", cm.argument);
            timeout_tid = tid;
            break;
        default:
           rm.ret = ERR_NOT_IMPLEMENTED;
           Reply(tid, &rm, sizeof(rm));
        }
    }
}

static void __attribute__((noreturn)) task_clocknotifier(int clk_tid){
    ReplyMessage rm = {0, 0};
    LOG("ClockNotifier init\r\n");
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = NOTIFIER;

    // Initialize Timer
    clk3->load = CYCLES_PER_TEN_MILLIS;
    clk3->control |= 0xD8; 

    FOREVER {
        AwaitEvent(EVENT_CLK_3);
        int err = Send(clk_tid, &cm, sizeof(cm), &rm, sizeof(rm));
        ASSERT(err >= 0, "Error sending to clock server");
        ASSERT(rm.ret == 0, "Error return from clock server");
    }
}

static inline int clockSend(int req, int arg){
    ClockMessage cm;
    cm.id = MESSAGE_CLOCK;
    cm.request = req;
    cm.argument = arg;
    ReplyMessage rm;
    int r = Send(WhoIs(NAME_CLOCK), &cm, sizeof(cm), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

int Time(){
    return clockSend(TIME, 0);
}

inline int Delay(int ticks){
    ASSERT(DEBUG_CLOCK && ticks > 0, "cannot delay for negative time: %d", ticks);
    if (ticks < 0) return 0;
    return clockSend(DELAY, ticks);
}

int DelayUntil(int ticks){
    return clockSend(DELAYUNTIL, ticks);
}

void __attribute__((noreturn)) task_timeout_update_courier(int timeout_srv_tid, int clock_tid) {
    ClockMessage cm = {MESSAGE_CLOCK, UPDATE_TIMEOUT, 0};
    TimeoutMessage tm = {0, 0};

    FOREVER {
        Send(timeout_srv_tid, &tm, sizeof(tm), &cm.argument, sizeof(cm.argument));
        Send(clock_tid, &cm, sizeof(cm), NULL, 0);
    }
}

void __attribute__((noreturn)) task_timeout_courier(int timeout_srv_tid, int clock_tid) {
    ClockMessage cm = {MESSAGE_CLOCK, REQUEST_TIMEOUT, 0};
    TimeoutMessage tm = {0, 0};

    FOREVER {
        Send(clock_tid, &cm, sizeof(cm), NULL, 0);
        Send(timeout_srv_tid, &tm, sizeof(tm), NULL, 0);
    }
}

static void __attribute__((noreturn)) task_timeout(int clock_tid) {
    RegisterAs(NAME_TIMEOUT);
    int mytid = MyTid();
    int rcv_courier = CreateWith2Args(PRIORITY_HIGHER, &task_timeout_courier, mytid, clock_tid);
    int snd_courier = CreateWith2Args(PRIORITY_HIGHER, &task_timeout_update_courier, mytid, clock_tid);

    entry_t mh_array[CLOCK_MH_SIZE];
    minheap_t mh = {mh_array, 0, CLOCK_MH_SIZE};
    entry_t mh_entry;

    TimeoutMessage tm;
    bool snd_ready = FALSE, min_changed = FALSE;
    int tid, err;
    FOREVER{
        Receive(&tid, &tm, sizeof(tm));
        if (tid == snd_courier) {
            //since the clock server should reply immdiately, most of the send courier's time will be spent waiting on us, so
            // we will normally be able to reply to the send courier as soon as the min changes. Thus, it is unlikely that min_changed is true
            if (unlikely(min_changed)){
                ASSERT(mh_peek_min(&mh, &mh_entry) == 0, "min heap should not be empty if the min has changed");
                err = Reply(snd_courier, &mh_entry.item, sizeof(mh_entry.item));
                ASSERT(err >= 0, "Error replying to send courier");
                snd_ready = FALSE;
                min_changed = FALSE;
            }
            else {
                snd_ready = TRUE;
            }
        } else if (tid == rcv_courier) {
            //because we reply to rcv before we reply to snd, rcv is queued first. Since the immediate next instruction in
            // rcv is a send, it is always receive-blocked on the clock server by the time snd sends a TIMEOUT_UPDATE
            Reply(tid, NULL, 0);
            mh_remove_min(&mh, &mh_entry);

            if ((min_changed = (mh_peek_min(&mh, &mh_entry) == 0)) && likely(snd_ready)) {
                err = Reply(snd_courier, &mh_entry.item, sizeof(mh_entry.item));
                ASSERT(err >= 0, "Error replying to send courier");
                snd_ready = FALSE;
                min_changed = FALSE;
            }
            //NOTE: this should be a very quick srr since the task which called Timeout(int) is very likely already receive blocked
            MessageType msg = MESSAGE_TIMEOUT;
            err = Send(mh_entry.item, &msg, sizeof(msg), NULL, 0);
            ASSERT(err >= 0, "Error sending to caller of Timeout(int)");
        } else { //from a call to Timeout(int)
            //get time immediately, then reply to unblock the caller
            unsigned int timeout = Time() + tm.ticks;
            Reply(tid, NULL, 0);

            unsigned int old_min = UINT_MAX;
            if (mh_peek_min(&mh, &mh_entry) == 0) {
                old_min = (unsigned int)mh_entry.value;
            }
            mh_add(&mh, tm.caller_tid, timeout);
            mh_peek_min(&mh, &mh_entry);
            if ((min_changed = (mh_entry.value < old_min)) && likely(snd_ready)) {
                err = Reply(snd_courier, &timeout, sizeof(timeout));
                ASSERT(err >= 0, "Error replying to send courier");
                snd_ready = FALSE;
                min_changed = FALSE;
            }
        }
    }
}

void Timeout(unsigned int ticks) {
    int mytid = MyTid(), timeout_tid = WhoIs(NAME_TIMEOUT);
    TimeoutMessage tm = {.caller_tid = mytid, .ticks = ticks};
    Send(timeout_tid, &tm, sizeof(tm), NULL, 0);
}

void __attribute__((noreturn)) task_clockprinter(int terminaltid){
    LOG("CLOCK PRINTER INIT");
    int time = Time();
    int time_since_startup_hundred_millis = 0;
    FOREVER{
        SendTerminalRequest(terminaltid, TERMINAL_TIME, time_since_startup_hundred_millis, GetValue(VALUE_IDLE));
        time += TICKS_PER_HUNDRED_MILLIS;
        DelayUntil(time);
        time_since_startup_hundred_millis++;
    }
}
