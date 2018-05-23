#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>
#include <syscall.h>
#include <sys_handler.h>

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
