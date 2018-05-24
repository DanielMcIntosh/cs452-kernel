#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>
#include <syscall.h>
#include <sys_handler.h>
#include <name.h>
#include <util.h>

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

void a(){
    char c[2];
    c[1] = NULL;
    for (int i = 4; i < 200; i += 2) {
        c[0] = i;
        RegisterAs(c);
    }
    Exit();
}
void b(){
    char c[3];
    c[2] = NULL;
    for (int i = 4; i < 200; i += 2) {
        c[0] = i-1;
        c[1] = 1;
        RegisterAs(c);
    }
    Exit();
}
void c(){
    char c[2];
    c[1] = NULL;
    for (int i = 4; i < 200; i+=2) {
        c[0] = i;
        bwprintf(COM2, "%s |-> %d\r\n", c, WhoIs(c));
    }
    char d[3];
    for (int i = 4; i < 200; i += 2) {
        d[0] = i-1;
        d[1] = 1;
        bwprintf(COM2, "%s |-> %d\r\n", d, WhoIs(d));
    }
    Exit();
}

void TestHashTable(){
    Create(PRIORITY_LOWEST-2, &a);
    Create(PRIORITY_LOWEST-1, &b);
    Create(PRIORITY_LOWEST, &c);
}
void fut(){
    int r = Create(PRIORITY_WAREHOUSE, &task_nameserver);
    bwprintf(COM2, "Created: %d\r\n", r);
    r = RegisterAs("FUT");
    bwprintf(COM2, "Registered As: \"FUT\" (%d)\r\n", r);
    int iam = MyTid();
    bwprintf(COM2, "I Am: %d\r\n", iam);
    r = WhoIs("FUT");
    bwprintf(COM2, "\"FUT\" is: %d\r\n", r);
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
    #endif

    TD *task = schedule(task_ready_queues, task_ready_queue_tails);
    #if DEBUG
    bwprintf(COM2, "OFFSETS: lr %d, sp %d, r0 %d, spsr %d, args0 %d, arg4 %d, task %d", &(task->lr), &(task->sp), &(task->r0), &(task->spsr), &(task->syscall_args[0]), &(task->syscall_args[4]), task);
    #endif
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
