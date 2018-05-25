#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>
#include <syscall.h>
#include <sys_handler.h>
#include <name.h>
#include <rps.h>
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
    r = RegisterAs("FUT");
    r = Create(PRIORITY_HIGH, &task_rps);
    bwprintf(COM2, "Created RPS Server: %d\r\n", r);
    r = Create(PRIORITY_LOW, &task_rps_client);
    bwprintf(COM2, "Created RPS Client 1: %d\r\n", r);
    r = Create(PRIORITY_LOW, &task_rps_client);
    bwprintf(COM2, "Created RPS Client 2: %d\r\n", r);
    Exit();
}


int main(){
    LOG("Start!");

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
    LOG("Task Created!\r\n");
    LOGF("&fut: %x\r\n", (int) &fut);

    TD *task = schedule(task_ready_queues, task_ready_queue_tails);
    LOGF("OFFSETS: lr %d, sp %d, r0 %d, spsr %d, args0 %d, arg4 %d, task %d", &(task->lr), &(task->sp), &(task->r0), &(task->spsr), &(task->syscall_args[0]), &(task->syscall_args[4]), task);
    while (task){
        LOGF("Task Scheduled! Pr = %d\t", task->priority);
        LOGF("task = %x\t", (int) task);
        LOGF("task->lr = %x\r\n", task->lr);
        int f = activate((int) task);
        LOGC(f);
        LOG("\r\n");

        handle(f, task, task_pool, task_ready_queues, task_ready_queue_tails, &task_free_queue);
        LOG("\r\n");
        task = schedule(task_ready_queues, task_ready_queue_tails);
    }
    LOG("Kernel Exiting - No More Tasks");

    return 0;
}
