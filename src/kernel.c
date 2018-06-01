#include <debug.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>
#include <syscall.h>
#include <sys_handler.h>
#include <name.h>
#include <rps.h>
#include <vic.h>
#include <util.h>
#include <msg_metrics.h>
#include <clock.h>

extern int activate(int task);
extern void KERNEL_ENTRY_POINT(void);
extern void IRQ_ENTRY_POINT(void);

int kernel_init(){
    __asm__(
        "ldr r0, =KERNEL_ENTRY_POINT\n\t"
        "ldr r1, =0x28\n\t"
        "str r0, [r1]\n\t"
        "ldr r0, =IRQ_ENTRY_POINT\n\t"
        "ldr r1, =0x38\n\t"\
        "str r0, [r1]\n\t"
        );

    // Initialize Timer
    // Unmask timer interrupt
    vic1->IntEnable |= 1;
    return 0;
}

TD* schedule(TaskQueue *task_ready_queue){
    return task_nextActive(task_ready_queue);
}

void fut(){
    LOG("First User Task: Start\r\n");
    software_interrupt(1);
    int r = RegisterAs("FUT");
    r = Create(PRIORITY_WAREHOUSE, &task_nameserver);
    r = Create(PRIORITY_WAREHOUSE, &task_clockserver);

    /*
    r = Create(PRIORITY_HIGH, &task_msg_metrics);
    //*/
    //*
    r = Create(PRIORITY_HIGH, &task_rps);
    bwprintf(COM2, "Created RPS Server: %d\r\n", r);
    r = Create(PRIORITY_LOW, &task_rps_client);
    bwprintf(COM2, "Created RPS Client 1: %d\r\n", r);
    r = Create(PRIORITY_LOW, &task_rps_client);
    bwprintf(COM2, "Created RPS Client 2: %d\r\n", r);
    //*/
    Exit();
}


int main(){
#if CACHE
    __asm__(
        "ldr r1, %[bits]\n\t"
        "MRC p15, 0, r0, c1, c0, 0\n\t"
        "orr r0, r0, r1\n\t"
        "MCR p15, 0, r0, c1, c0, 0\n\t"
            :
            : [bits] "rim" (0x1004)
            : "r0", "r1"
        );
#endif //CACHE

    LOG("Start!");

    kernel_init();
    char stack_space[STACK_SPACE_SIZE];

    TD task_pool[TASK_POOL_SIZE];
    TaskQueue task_ready_queue;

    task_init(task_pool, &task_ready_queue, stack_space, STACK_SPACE_SIZE);

    int err = task_create(&task_ready_queue, 1, 4, (int) &fut);
    if (err < 0) {
        bwprintf(COM2, "-=-=-=-=-=-=ERR = %d=-=-=-=-=-=-=-\r\n", err);
        return -1;
    }
    LOG("Task Created!\r\n");
    LOGF("&fut: %x\r\n", (int) &fut);

    TD *task = schedule(&task_ready_queue);
    LOGF("OFFSETS: lr %d, sp %d, r0 %d, spsr %d, task %d\r\n", (int) &(task->lr)-(int)task, (int)&(task->sp)-(int)task, (int)&(task->r0)-(int)task, (int)&(task->spsr)-(int)task, task);
    int f = 0;
    while (task){
        LOGF("Task Scheduled! Pr = %d\t", task->priority);
        LOGF("task = %x\t", (int) task);
        LOGF("task->r0 = %d\t", task->r0);
        LOGF("task->lr = %x\r\n", task->lr);
        LOGF("Task Stack: \r\n");
        for (int i = 0; i < 25; i++) {
            LOGF("SP[%d] = %d\r\n", i, task->sp[i])
        }
        if (task->last_syscall == SYSCALL_INTERRUPT){
            task->lr -= 4;
        } else {
            task->sp[0] = task->r0; // TODO ?? is there a better way to do this?
        }
        f = activate((int) task);
        task->last_syscall = f;
        LOGF("SYSCALL: %d\r\n", f);

        handle(f, task, task_pool, &task_ready_queue);
        LOG("\r\n");
        task = schedule(&task_ready_queue);
    }
    LOG("Kernel Exiting - No More Tasks");

    return 0;
}
