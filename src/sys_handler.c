#include <sys_handler.h>
#include <debug.h>
#include <bwio.h>
#include <err.h>
#include <tasks.h>
#include <util.h>
#include <vic.h>
#include <clock.h>
#include <event.h>

static inline void handle_create(TD *task, TaskQueue *task_ready_queue) {
    LOGF("TASK CREATE: %d, %d, %d\r\n", task_getTid(task), TD_arg(task, 0), TD_arg(task, 1));
    task->r0 = task_create(task_ready_queue, task_getTid(task), TD_arg(task, 0), TD_arg(task, 1));
}

static inline void handle_tid(TD *task) {
    task->r0 = task_getTid(task);
    LOGF("TID: %d\r\n", task->r0);
}

static inline void handle_ptid(TD *task) {
    task->r0 = task_getParentTid(task);
    LOGF("PTID: %d\r\n", task->r0);
}

static inline void handle_pass() {
    LOG("PASS called\r\n");
}

static inline void handle_exit(TD *task) {
    LOG("EXIT called\r\n");
    //don't re-queue the task, let it become a zombie task.
    //it will still be accessible by it's TID, since that gives us an index in the task pool
    task->state = STATE_ZOMBIE;
}


//////////////////////////////////////////////////////////////////////////////////////////////
//                                  MESSAGE PASSING
//////////////////////////////////////////////////////////////////////////////////////////////

static inline void handle_send(TD *task, TD *task_pool, TaskQueue *task_ready_queue){
    LOG("SEND called\r\n");

    TD *receiver = task_lookup(task_pool, TD_arg(task, 0));
    if (receiver == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }


    //reciever is already waiting for a message
    if (receiver->state == STATE_BLK_SEND)
    {
        *((int *)(TD_arg(receiver, 0))) = task_getTid(task); //set the tid of the receive caller
        if (TD_arg(task, 2) != TD_arg(receiver, 2)){
            receiver->r0 = ERR_MSG_TRUNCATED;
        } else {
            receiver->r0 = TD_arg(task, 2);
        }
        memcpy((int *)(TD_arg(receiver, 1)), (int *)(TD_arg(task, 1)), MIN(TD_arg(task, 2), TD_arg(receiver, 2)));

        receiver->state = STATE_READY;
        task->state = STATE_BLK_REPLY; // sender state will be reacted to after returning from this method
        task_react_to_state(receiver, task_ready_queue);
    }
    else {
        task->state = STATE_BLK_RECEIVE;
        // Put task into receiver queue.
        if (receiver->rcv_queue == NULL){
            receiver->rcv_queue = task;
            receiver->rcv_queue_tail = task;
        } else {
            receiver->rcv_queue_tail->rdynext = task;
            receiver->rcv_queue_tail = task;
        }
    }
}

static inline void handle_receive(TD *task) {
    LOG("RECEIVE called\r\n");
    TD *sender = task->rcv_queue;

    //someone has already sent us a message
    if (sender != NULL) {
        //pop sender from queue
        task->rcv_queue = sender->rdynext;
        if (task->rcv_queue == NULL){
            task->rcv_queue_tail = NULL;
        }

        void * msg = (void*) TD_arg(sender, 1);
        int len = TD_arg(sender, 2);

        *((int *)TD_arg(task, 0)) = task_getTid(sender); //set the tid value
        if (TD_arg(task, 2) != len) { // TODO: is this the correct behavior for when the lengths are wrong?
            task->r0 = ERR_MSG_TRUNCATED;
        } else {
            task->r0 = len;
        }
        memcpy((void *)(TD_arg(task, 1)), msg, MIN(len, TD_arg(task, 2)));

        sender->state = STATE_BLK_REPLY;
        task->state = STATE_READY;
    }
    else
    {
        task->state = STATE_BLK_SEND;
    }
}

static inline void handle_reply(TD *task, TD *task_pool, TaskQueue *task_ready_queue) {
    LOG("REPLY called\r\n");
    
    TD *sender = task_lookup(task_pool, TD_arg(task, 0));
    if (sender == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }
    if (sender->state != STATE_BLK_REPLY) {
        task->r0 = ERR_TASK_NOT_REPLY_BLOCKED;
        return;
    }

    if (TD_arg(sender, 4) != TD_arg(task, 2)) {
        task->r0 = ERR_MSG_TRUNCATED;
        sender->r0 = ERR_MSG_TRUNCATED;
    } else {
        task->r0 = 0;
        sender->r0 = TD_arg(task, 2);
    }
    memcpy((void *) TD_arg(sender, 3), (int *)TD_arg(task, 1), MIN(TD_arg(task, 2), TD_arg(sender, 4)));   

    sender->state = STATE_READY;
    task_react_to_state(sender, task_ready_queue);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//                                  INTERRUPTS
//////////////////////////////////////////////////////////////////////////////////////////////
static inline void handle_await(TD *task, TaskQueue *task_ready_queue){
    int event = TD_arg(task, 0);

    if (task_ready_queue->event_wait[event]) {
        task->r0 = ERR_EVENT_ALREADY_HAS_TASK_WAITING;
    }

    task_ready_queue->event_wait[event] = task;

    task->state = STATE_BLK_EVENT;
}

static inline void handle_interrupt(TD *task, TaskQueue *task_ready_queue){
    LOG("HANDLE INTERRUPT\r\n");
    LOGF("vic1 status: %x, vic2 status: %x\r\n", vic1->IRQStatus, vic2->IRQStatus);

    // figure out what interrupt it is
    unsigned long long IRQStatus = vic1->IRQStatus | (unsigned long long)(vic2->IRQStatus) << VIC_SIZE;
    Event event;
    for (event = 0; event < NUM_EVENTS; ++event) {
        if (0x1ULL << IRQ_MAP[event] | IRQStatus) {
            break;
        }
    }
    ASSERT(event < NUM_EVENTS, "Interrupt doesn't correspond to an event",);

    // turn off that interrupt
    event_turn_off(event);

    // unblock task waiting for that interrupt?
    TD *waiting = task_ready_queue->event_wait[event];
    if (!waiting) {
        return;
    }

    waiting->state = STATE_READY;
    //TODO set return value of waiting
    task_react_to_state(waiting, task_ready_queue);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//                                     MAIN
//////////////////////////////////////////////////////////////////////////////////////////////
Syscall handle(Syscall a, TD *task, TD *task_pool, TaskQueue *task_ready_queue) {
    LOGF("HANDLE: %d, %d\t", a, task);
    LOGF("ARGS: %d, %d, %d, %d, %d\t", TD_arg(task, 0), TD_arg(task, 1), TD_arg(task, 2), TD_arg(task, 3), TD_arg(task, 4));
    switch(a) {
        case SYSCALL_CREATE:
        {
            handle_create(task, task_ready_queue);
            break;
        }
        case SYSCALL_TID:
        {
            handle_tid(task);
            break;
        }
        case SYSCALL_PTID:
        {
            handle_ptid(task);
            break;
        }
        case SYSCALL_PASS:
        {
            handle_pass();
            break;
        }
        case SYSCALL_EXIT:
        {
            handle_exit(task);
            break;
        }
        case SYSCALL_SEND:
        {
            handle_send(task, task_pool, task_ready_queue);
            break;
        }
        case SYSCALL_RECEIVE:
        {
            handle_receive(task);
            break;
        }
        case SYSCALL_REPLY:
        {
            handle_reply(task, task_pool, task_ready_queue);
            break;
        }
        case SYSCALL_AWAIT:
        {
            handle_await(task, task_ready_queue);
            break;
        }
        case SYSCALL_INTERRUPT:
        {
            handle_interrupt(task, task_ready_queue);
            break;
        }
        default:
        {
            bwputstr(COM2, "UNKNOWN SYSCALL\r\n");
            break;
        }
    }
    task_react_to_state(task, task_ready_queue);
    return a;
}
