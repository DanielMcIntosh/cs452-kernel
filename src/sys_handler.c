#include <sys_handler.h>
#include <debug.h>
#include <bwio.h>
#include <err.h>
#include <tasks.h>
#include <util.h>
#include <vic.h>
#include <clock.h>
#include <event.h>
#include <uart.h>

static inline void handle_create(TD *task, TaskQueue *task_ready_queue) {
    LOGF("TASK CREATE: %d, %d, %d\r\n", task_getTid(task), TD_arg(task, 0), TD_arg(task, 1));
    task->r0 = task_create(task_ready_queue, task_getTid(task), TD_arg(task, 0), TD_arg(task, 1), 0, 0);
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

static int is_deadlock(TD *sender, TD *receiver) {
    // TODO
    int k = 0;

    if (receiver->state == STATE_BLK_RECEIVE || receiver->state == STATE_BLK_REPLY) {
        TD * sndr = sender->rcv_queue;
        while (sndr != NULL) {
            if (sndr->tid == receiver->tid) return 1; //, "DEADLOCK: %d is sending to %d, but %d is currently sending to %d (in state: %d)", task->tid, receiver->tid, receiver->tid, task->tid, receiver->state);
            sndr = sndr->rdynext;
            KASSERT(k++ < 200, "definitely an infinite loop");
        }
   }
    return 0;
}

static inline void handle_send(TD *task, TaskQueue *task_ready_queue){
    LOG("SEND called\r\n");

    TD *receiver = task_lookup(task_ready_queue, TD_arg(task, 0));
    if (unlikely(receiver == NULL)) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }

    //KASSERT(!is_deadlock(task, receiver), "DEADLOCK BETWEEN %d (%d) and %d (%d)", task->tid, TD_arg(task, 1), receiver->tid, TD_arg(receiver, 1));
    if (task == NULL || is_deadlock(task, receiver)) {
        //__builtin_trap();
        //bwprintf(COM2, "F");
    }

    //reciever is already waiting for a message
    if (receiver->state == STATE_BLK_SEND)
    {
        *((int *)(TD_arg(receiver, 0))) = task_getTid(task); //set the tid of the receive caller
        if (unlikely(TD_arg(task, 2) != TD_arg(receiver, 2))){
            receiver->r0 = ERR_MSG_TRUNCATED;

            //__builtin_trap();
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
        sender->rdynext = NULL;
        if (task->rcv_queue == NULL){
            task->rcv_queue_tail = NULL;
        }

        void * msg = (void*) TD_arg(sender, 1);
        int len = TD_arg(sender, 2);

        *((int *)TD_arg(task, 0)) = task_getTid(sender); //set the tid value
        if (TD_arg(task, 2) != len) { 
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

static inline void handle_reply(TD *task, TaskQueue *task_ready_queue) {
    LOG("REPLY called\r\n");
    
    TD *sender = task_lookup(task_ready_queue, TD_arg(task, 0));
    if (unlikely(sender == NULL)) {
        task->r0 = ERR_TASK_DOES_NOT_EXIST;
        return;
    }
    if (unlikely(sender->state != STATE_BLK_REPLY)) {
        task->r0 = ERR_TASK_NOT_REPLY_BLOCKED;
        return;
    }

    if (unlikely(TD_arg(sender, 4) != TD_arg(task, 2))) {
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
        return;
    }
    if (event == EVENT_UART_2_SEND){
        uart2->ctrl |= TIEN_MASK; 
    } else if (event == EVENT_UART_1_SEND){
        uart1->ctrl |= TIEN_MASK; 
    }

    task_ready_queue->event_wait[event] = task;
    task->state = STATE_BLK_EVENT;
}

static inline void handle_interrupt(TD __attribute__((unused)) *task, TaskQueue *task_ready_queue){
    LOG("HANDLE INTERRUPT\r\n");
    // figure out what interrupt it is
    unsigned long long IRQStatus = ((unsigned long long) vic1->IRQStatus) | ((unsigned long long)(vic2->IRQStatus) << VIC_SIZE);
    Event event;
    for (event = 0; event < NUM_EVENTS; ++event) {
        if ((0x1ULL << IRQ_MAP[event]) & IRQStatus) {
            break;
        }
    }
    KASSERT(event < NUM_EVENTS, "Interrupt doesn't correspond to an event:%d,  %x|%x", event, vic1->IRQStatus, vic2->IRQStatus);
    LOGF("EVENT: %d | STATUS: %x|%x\r\n", event, vic1->IRQStatus, vic2->IRQStatus);

    // turn off that interrupt
    int handled_event = 0;
    int data = event_turn_off(event, &handled_event);
    event = (Event) handled_event;
    // event is updated to the handled event - specifically for uart send & recieve

    // unblock task waiting for that interrupt?
    TD *waiting = task_ready_queue->event_wait[event];
    task_ready_queue->event_wait[event] = NULL;
    if (!waiting) {
        return;
    }

    waiting->state = STATE_READY;
    waiting->r0 = data;
    task_react_to_state(waiting, task_ready_queue);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//                                      K4
//////////////////////////////////////////////////////////////////////////////////////////////

static inline void handle_quit() {
}

static inline void handle_enter_critical_section(TD *task) {
    task->spsr |= 0x80;
}

static inline void handle_exit_critical_section(TD *task) {
    task->spsr &= ~(0x80);
}

static inline void handle_destroy(TD *task){
    LOG("DESTROY called\r\n");
    task->state = STATE_DESTROYED;
}

static inline void handle_create_argument(TD *task, TaskQueue *task_ready_queue){
    LOGF("TASK CREATE ARG: %d, %d, %d, %d\r\n", task_getTid(task), TD_arg(task, 0), TD_arg(task, 1), TD_arg(task, 2));
    task->r0 = task_create(task_ready_queue, task_getTid(task), TD_arg(task, 0), TD_arg(task, 1), TD_arg(task, 2), 0);
}

static inline void handle_create_2_args(TD *task, TaskQueue *task_ready_queue){
    LOGF("TASK CREATE ARG: %d, %d, %d, %d, %d\r\n", task_getTid(task), TD_arg(task, 0), TD_arg(task, 1), TD_arg(task, 2), TD_arg(task, 3));
    task->r0 = task_create(task_ready_queue, task_getTid(task), TD_arg(task, 0), TD_arg(task, 1), TD_arg(task, 2), TD_arg(task, 3));
}

static inline void handle_store_value(TD * restrict task, ValueStore * restrict v){
    LOG("HANDLE STORE VALUE\r\n");
    v->values[TD_arg(task, 0)] = TD_arg(task, 1);
    task->r0 = TD_arg(task, 1);
}

static inline void handle_get_value(TD * restrict task, ValueStore * restrict v){
    LOG("HANDLE GET VALUE\r\n");
    task->r0 = v->values[TD_arg(task, 0)];
}

//////////////////////////////////////////////////////////////////////////////////////////////
//                                     MAIN
//////////////////////////////////////////////////////////////////////////////////////////////
Syscall handle(Syscall a, TD *task, TaskQueue *task_ready_queue, ValueStore * restrict values) {
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
            handle_send(task, task_ready_queue);
            break;
        }
        case SYSCALL_RECEIVE:
        {
            handle_receive(task);
            break;
        }
        case SYSCALL_REPLY:
        {
            handle_reply(task, task_ready_queue);
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
        case SYSCALL_QUIT:
        {
            handle_quit();
            break;
        }
        case SYSCALL_INTERRUPTS_OFF:
        {
            handle_enter_critical_section(task);
            break;
        }
        case SYSCALL_INTERRUPTS_ON:
        {
            handle_exit_critical_section(task);
            break;
        }
        case SYSCALL_DESTROY:
        {
            handle_destroy(task);
            break;
        }
        case SYSCALL_CREATE_ARGUMENT:
        {
            handle_create_argument(task, task_ready_queue);
            break;
        }
        case SYSCALL_CREATE_2_ARGS:
        {
            handle_create_2_args(task, task_ready_queue);
            break;
        }
        case SYSCALL_STORE_VALUE:
        {
            handle_store_value(task, values);
            break;
        }
        case SYSCALL_GET_VALUE:
        {
            handle_get_value(task, values);
            break;
        }
        default:
        {
            PANIC("UNKNOWN SYSCALL");
            break;
        }
    }
    task_react_to_state(task, task_ready_queue);
    return a;
}
