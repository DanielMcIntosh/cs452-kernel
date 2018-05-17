#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>
#include <syscall.h>

#define FOREVER for(;;)
#define STACK_SPACE_SIZE 0x800000

#define DUMPR(x) "mov r0, #1\n\tmov r1, "x"\n\t bl bwputr\n\t"

int kernel_init(){
    __asm__(
        "ldr r0, =KERNEL_ENTRY_POINT\n\t"\
        "ldr r1, =0x28\n\t"\
        "str r0, [r1]");
    return 0;
}


TD* schedule(TD **task_ready_queues, TD **task_ready_queue_tails){
    return task_nextActive(task_ready_queues, task_ready_queue_tails);
}

extern int activate(int task);
extern void KERNEL_ENTRY_POINT(void);

Syscall handle(Syscall a, TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue){
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
            #if DEBUG
            bwprintf(COM2, "TASK CREATE: %d, %d, %d\r\n", task_getTid(task), args[0], args[1]);
            #endif
            task->r0 = task_create(task_ready_queues, task_ready_queue_tails, task_free_queue, task_getTid(task), args[0], args[1]);
            break;
        }
        case SYSCALL_TID:
        {
            task->r0 = task_getTid(task);
            #if DEBUG
            bwprintf(COM2, "TID: %d\r\n", task->r0);
            #endif
            break;
        }
        case SYSCALL_PTID:
        {
            task->r0 = task_getParentTid(task);
            #if DEBUG
            bwprintf(COM2, "PTID: %d\r\n", task->r0);
            #endif
            break;
        }
        case SYSCALL_PASS:
        {
            #if DEBUG
            bwputstr(COM2, "PASS called\r\n");
            #endif
            break;
        }
        case SYSCALL_EXIT:
        {
            #if DEBUG
            bwputstr(COM2, "EXIT called\r\n");
            #endif
            //don't re-queue the task, let it become a zombie task.
            //it will still be accessible by it's TID, since that gives us an index in the task pool
            task->state = STATE_ZOMBIE;
            break;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////
        //                              NOT FULLY IMPLEMENTED
        //////////////////////////////////////////////////////////////////////////////////////////////
        case SYSCALL_SEND:
        {
            TD *receiver = task_lookup(task_pool, args[0]);
            #if DEBUG
            bwputstr(COM2, "SEND called\r\n");
            #endif
            if (receiver->state == STATE_BLOCKED)
            {
                receiver->state = STATE_READY;
                *((int *)(receiver->syscall_args[0])) = task_getTid(task); //set the tid of the receive caller
                task_react_to_state(receiver, task_ready_queues, task_ready_queue_tails, task_free_queue);
            }
            else {
                task->state = STATE_BLOCKED;
                task->msg_queue = receiver->msg_queue;
                receiver->msg_queue = task;
            }
            break;
        }
        case SYSCALL_RECEIVE:
        {
            #if DEBUG
            bwputstr(COM2, "RECEIVE called\r\n");
            #endif
            TD *sender = task->msg_queue;
            if (sender) {
                *((int *)args[0]) = task_getTid(sender); //set the tid value
                task->msg_queue = sender->msg_queue;
                sender->msg_queue = 0;
            }
            else
            {
                task->state = STATE_BLOCKED;
            }
            break;
        }
        case SYSCALL_REPLY:
        {
            #if DEBUG
            bwputstr(COM2, "REPLY called\r\n");
            #endif
            TD *sender = task_lookup(task_pool, args[0]);
            sender->state = STATE_READY;
            task_react_to_state(sender, task_ready_queues, task_ready_queue_tails, task_free_queue);
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

void utsk(){
    int tid = MyTid();
    if (tid == 4) {
        int sender;
        Receive(&sender, 9990, 550);
        bwprintf(COM2, "sender = %d\r\n", sender);
        Reply(sender, 0, 0);
    }
    else if (tid == 3) {
        Send(4, 10, 20, 30, 40);
    }
    int ptid = MyParentTID();
    bwprintf(COM2, "MyTid: %d, MyParentTid: %d\r\n", tid, ptid);
    Pass();
    bwprintf(COM2, "MyTid: %d, MyParentTid: %d\r\n", tid, ptid);
    Exit();
}

void fut(){
    int r = Create(PRIORITY_LOWEST, &utsk);
    bwprintf(COM2, "Created: %d\r\n", r);
    r = Create(PRIORITY_LOWEST, &utsk);
    bwprintf(COM2, "Created: %d\r\n", r);
    r = Create(PRIORITY_HIGHEST, &utsk);
    bwprintf(COM2, "Created: %d\r\n", r);
    r = Create(PRIORITY_HIGHEST, &utsk);
    bwprintf(COM2, "Created: %d\r\n", r);
    bwputstr(COM2, "FirstUserTask: exiting\r\n");
    Exit();
}

int main(){
    #if DEBUG
    bwputstr(COM2, "Start!");
    #endif

    kernel_init();
    char stack_space[STACK_SPACE_SIZE];

    TD task_pool[TASK_POOL_SIZE];
    TD *task_ready_queues[NUM_PRIORITIES];
    TD *task_ready_queue_tails[NUM_PRIORITIES];
    TD *task_free_queue = task_pool;

    task_init(task_pool, task_ready_queues, task_ready_queue_tails, stack_space, STACK_SPACE_SIZE);

    int err = task_create(task_ready_queues, task_ready_queue_tails, &task_free_queue, 1, 4, (int) &fut);
    if (err < 0) {
        bwprintf(COM2, "-=-=-=-=-=-=ERR = %d=-=-=-=-=-=-=-\r\n", err);
        return -1;
    }
    #if DEBUG
    bwputstr(COM2, "Task Created!\r\n");
    bwprintf(COM2, "&fut: %x\r\n", (int) &fut);
    bwprintf(COM2, "&utsk: %x\r\n", (int) &utsk);
    #endif

    TD *task = schedule(task_ready_queues, task_ready_queue_tails);
    bwprintf(COM2, "OFFSETS: lr %d, sp %d, r0 %d, spsr %d, args0 %d, arg4 %d, task %d", &(task->lr), &(task->sp), &(task->r0), &(task->spsr), &(task->syscall_args[0]), &(task->syscall_args[4]), task);
    while (task){
        #if DEBUG
        bwprintf(COM2, "Task Scheduled! Pr = %d\t", task->priority);
        bwprintf(COM2, "task = %x\t", (int) task);
        bwprintf(COM2, "task->lr = %x\r\n", task->lr);
        #endif
        int f = activate((int) task);
        #if DEBUG
        bwputc(COM2, f);
        bwputstr(COM2, "\r\n");
        #endif

        handle(f, task, task_pool, task_ready_queues, task_ready_queue_tails, &task_free_queue);
        #if DEBUG
        bwputstr(COM2, "\r\n");
        #endif
        task = schedule(task_ready_queues, task_ready_queue_tails);
    }
    #if DEBUG
    bwputstr(COM2, "Passed?");
    #endif

    return 0;
}
