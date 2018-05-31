#include <sys_handler.h>
#include <debug.h>
#include <bwio.h>
#include <err.h>
#include <tasks.h>
#include <util.h>
#include <vic.h>

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
    if (receiver->state == STATE_SEND_BLOCKED)
    {
        *((int *)(TD_arg(receiver, 0))) = task_getTid(task); //set the tid of the receive caller
        if (TD_arg(task, 2) != TD_arg(receiver, 2)){
            receiver->r0 = ERR_MSG_TRUNCATED;
        } else {
            receiver->r0 = TD_arg(task, 2);
        }
        memcpy((int *)(TD_arg(receiver, 1)), (int *)(TD_arg(task, 1)), MIN(TD_arg(task, 2), TD_arg(receiver, 2)));

        receiver->state = STATE_READY;
        task->state = STATE_REPLY_BLOCKED; // sender state will be reacted to after returning from this method
        task_react_to_state(receiver, task_ready_queue);
    }
    else {
        task->state = STATE_RECEIVE_BLOCKED;
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

        sender->state = STATE_REPLY_BLOCKED;
        task->state = STATE_READY;
    }
    else
    {
        task->state = STATE_SEND_BLOCKED;
    }
}

static inline void handle_reply(TD *task, TD *task_pool, TaskQueue *task_ready_queue) {
    LOG("REPLY called\r\n");
    
    TD *sender = task_lookup(task_pool, TD_arg(task, 0));
    if (sender == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }
    if (sender->state != STATE_REPLY_BLOCKED) {
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
static inline void handle_await(TD *task){}
static inline void handle_interrupt(TD *task){
    LOG("HANDLE INTERRUPT\r\n");
    LOGF("int reg: %d\r\n", vic1->IRQStatus);
    vic1->SoftIntClear = 1;
    LOGF("int reg: %d\r\n", vic1->IRQStatus);
    bwputstr(COM2, "========================================================================================================================================================================================================================================================================================================================================================");
    // figure out what interrupt it is
    // turn off that interrupt
    // unblock task waiting for that interrupt?
}


Syscall handle(Syscall a, TD *task, TD *task_pool, TaskQueue *task_ready_queue) {
    LOGF("HANDLE: %d, %d\t", a, task);
    LOGF("ARGS: %d, %d, %d, %d, %d\t", TD_arg(task, 0), TD_arg(task, 1), TD_arg(task, 2), TD_arg(task, 3), TD_arg(task, 4))
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
            handle_await(task); // TODO
            break;
        }
        case SYSCALL_INTERRUPT:
        {
            handle_interrupt(task);
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
