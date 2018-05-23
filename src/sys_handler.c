#include "sys_handler.h"
#include "debug.h"
#include "bwio.h"
#include "err.h"

static inline void handle_create(TD *task, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue, int *args) {
    #if DEBUG
    bwprintf(COM2, "TASK CREATE: %d, %d, %d\r\n", task_getTid(task), args[0], args[1]);
    #endif
    task->r0 = task_create(task_ready_queues, task_ready_queue_tails, task_free_queue, task_getTid(task), args[0], args[1]);
}

static inline void handle_tid(TD *task) {
    task->r0 = task_getTid(task);
    #if DEBUG
    bwprintf(COM2, "TID: %d\r\n", task->r0);
    #endif
}

static inline void handle_ptid(TD *task) {
    task->r0 = task_getParentTid(task);
    #if DEBUG
    bwprintf(COM2, "PTID: %d\r\n", task->r0);
    #endif
}

static inline void handle_pass() {
    #if DEBUG
    bwputstr(COM2, "PASS called\r\n");
    #endif
}

static inline void handle_exit(TD *task) {
    #if DEBUG
    bwputstr(COM2, "EXIT called\r\n");
    #endif
    //don't re-queue the task, let it become a zombie task.
    //it will still be accessible by it's TID, since that gives us an index in the task pool
    task->state = STATE_ZOMBIE;
}


//////////////////////////////////////////////////////////////////////////////////////////////
//                                  MESSAGE PASSING
//////////////////////////////////////////////////////////////////////////////////////////////

static inline void handle_send(TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue, int *args) {
    #if DEBUG
    bwputstr(COM2, "SEND called\r\n");
    #endif

    TD *receiver = task_lookup(task_pool, args[0]);
    if (receiver == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }


    //reciever is already waiting for a message
    if (receiver->state == STATE_BLOCKED)
    {
        //TODO
        *((int *)(receiver->syscall_args[0])) = task_getTid(task); //set the tid of the receive caller

        receiver->state = STATE_READY;
        task_react_to_state(receiver, task_ready_queues, task_ready_queue_tails, task_free_queue);
    }
    else {
        task->state = STATE_BLOCKED;
        
        task->msg_queue = receiver->msg_queue;
        receiver->msg_queue = task;

        task->msg_snd = args[1];
        task->msg_snd_len = args[2];
        task->msg_rpy = args[3];
        task->msg_rpy_len = args[4];
    }
}

static inline void handle_receive(TD *task, int *args) {
    #if DEBUG
    bwputstr(COM2, "RECEIVE called\r\n");
    #endif
    TD *sender = task->msg_queue;

    //someone has already sent us a message
    if (sender != NULL) {
        *((int *)args[0]) = task_getTid(sender); //set the tid value

        task->msg_queue = sender->msg_queue;
        sender->msg_queue = 0;

        if (args[2] < sender->msg_snd_len) {
            task->r0 = ERR_MSG_TRUNCATED;
            memcpy((int *)(args[1]), sender->msg_snd, args[2]);
        }
        else {
            task->r0 = sender->msg_snd_len;
            memcpy((int *)(args[1]), sender->msg_snd, sender->msg_snd_len);   
        }
    }
    else
    {
        //TODO
        task->state = STATE_BLOCKED;
    }
}

static inline void handle_reply(TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue, int *args) {
    #if DEBUG
    bwputstr(COM2, "REPLY called\r\n");
    #endif
    
    TD *sender = task_lookup(task_pool, args[0]);
    if (sender == NULL) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }
    if (sender->msg_queue != NULL || sender->state != STATE_BLOCKED) {
        task->r0 = ERR_TASK_NOT_REPLY_BLOCKED;
        return;
    }

    if (sender->msg_rpy_len < args[2]) {
        task->r0 = ERR_MSG_TRUNCATED;
        sender->r0 = ERR_MSG_TRUNCATED;
        memcpy(sender->msg_rpy, args[1], sender->msg_rpy_len);
    }
    else {
        task->r0 = 0;
        sender->r0 = args[2];
        memcpy(sender->msg_rpy, args[1], args[2]);   
    }

    sender->state = STATE_READY;
    task_react_to_state(sender, task_ready_queues, task_ready_queue_tails, task_free_queue);
}

Syscall handle(Syscall a, TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue) {
    #if DEBUG
    bwprintf(COM2, "HANDLE: %d, %d\t", a, task);
    #endif
    int *args = task->syscall_args;
    #if DEBUG
    bwprintf(COM2, "ARGS: %d, %d, %d, %d, %d\t", args[0], args[1], args[2], args[3], args[4]);
    #endif
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
