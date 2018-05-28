#include "sys_handler.h"
#include "debug.h"
#include "bwio.h"
#include "err.h"
#include "tasks.h"
#include "util.h"

static inline void handle_create(TD *task, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue, int *args) {
    LOGF("TASK CREATE: %d, %d, %d\r\n", task_getTid(task), args[0], args[1]);
    task->r0 = task_create(task_ready_queues, task_ready_queue_tails, task_free_queue, task_getTid(task), args[0], args[1]);
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

static inline void handle_send(TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue, int *args) {
    LOG("SEND called\r\n");

    TD *receiver = task_lookup(task_pool, args[0]);
    if (receiver == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }


    //reciever is already waiting for a message
    if (receiver->state == STATE_SEND_BLOCKED)
    {
        LOG("Receiver is send blocked\r\n");
        *((int *)(receiver->syscall_args[0])) = task_getTid(task); //set the tid of the receive caller
        if (args[2] != receiver->syscall_args[2]){
            receiver->r0 = ERR_MSG_TRUNCATED;
        } else {
            receiver->r0 = args[2];
        }
        memcpy((int *)(receiver->syscall_args[1]), (int *)(args[1]), MIN(args[2], receiver->syscall_args[2]));

        receiver->state = STATE_READY;
        task->state = STATE_REPLY_BLOCKED; // sender state will be reacted to after returning from this method
        task_react_to_state(receiver, task_ready_queues, task_ready_queue_tails, task_free_queue);
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

static inline void handle_receive(TD *task, int *args) {
    LOG("RECEIVE called\r\n");
    TD *sender = task->rcv_queue;

    //someone has already sent us a message
    if (sender != NULL) {
        LOGF("Sender: %d\r\n", sender);

        //pop sender from queue
        task->rcv_queue = sender->rdynext;
        if (task->rcv_queue == NULL){
            task->rcv_queue_tail = NULL;
        }

        void * msg = (void*) sender->syscall_args[1];
        int len = sender->syscall_args[2];

        *((int *)args[0]) = task_getTid(sender); //set the tid value
        if (args[2] != len) { // TODO: is this the correct behavior for when the lengths are wrong?
            task->r0 = ERR_MSG_TRUNCATED;
        } else {
            task->r0 = len;
        }
        memcpy((int *)(args[1]), msg, MIN(len, args[2]));

        sender->state = STATE_REPLY_BLOCKED;
        task->state = STATE_READY;
    }
    else
    {
        task->state = STATE_SEND_BLOCKED;
    }
}

static inline void handle_reply(TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue, int *args) {
    LOG("REPLY called\r\n");
    
    TD *sender = task_lookup(task_pool, args[0]);
    if (sender == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }
    if (sender->state != STATE_REPLY_BLOCKED) {
        task->r0 = ERR_TASK_NOT_REPLY_BLOCKED;
        return;
    }

    if (sender->syscall_args[4] != args[2]) {
        task->r0 = ERR_MSG_TRUNCATED;
        sender->r0 = ERR_MSG_TRUNCATED;
    } else {
        task->r0 = 0;
        sender->r0 = args[2];
    }
    memcpy((void *) sender->syscall_args[3], (int *)(args[1]), MIN(args[2], sender->syscall_args[4]));   

    sender->state = STATE_READY;
    task_react_to_state(sender, task_ready_queues, task_ready_queue_tails, task_free_queue);
}

Syscall handle(Syscall a, TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue) {
    LOGF("HANDLE: %d, %d\t", a, task);
    int *args = task->syscall_args;
    LOGF("ARGS: %d, %d, %d, %d, %d\t", args[0], args[1], args[2], args[3], args[4]);
    switch(a) {
        case SYSCALL_CREATE:
        {
            handle_create(task, task_ready_queues, task_ready_queue_tails, task_free_queue, args);
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
            handle_send(task, task_pool, task_ready_queues, task_ready_queue_tails, task_free_queue, args);
            break;
        }
        case SYSCALL_RECEIVE:
        {
            handle_receive(task, args);
            break;
        }
        case SYSCALL_REPLY:
        {
            handle_reply(task, task_pool, task_ready_queues, task_ready_queue_tails, task_free_queue, args);
            break;
        }
        default:
        {
            bwputstr(COM2, "UNKNOWN SYSCALL\r\n");
            break;
        }
    }
    task_react_to_state(task, task_ready_queues, task_ready_queue_tails, task_free_queue);
    return a;
}
