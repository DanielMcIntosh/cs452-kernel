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
#include <event.h>
#include <clock.h>
#include <message.h>

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

    // Unmask timer interrupt
    vic2->IntEnable |= 0x1 << (IRQ_MAP[EVENT_CLK_3] - 32);


    return 0;
}

TD* schedule(TaskQueue *task_ready_queue){
    return task_nextActive(task_ready_queue);
}

#define IDLE_ITERATIONS 500000
void task_idle() {
    int i, j = 0;
    FOREVER {
        i = IDLE_ITERATIONS;
        int time_start = clk4->value_low;        
        __asm__ volatile (
            "idle_loop:\n\t"
            "subs %[i], %[i], #1\n\t"
            "mov %[j], %[i]\n\t"
            "mov %[i], %[j]\n\t"
            "mov %[j], %[i]\n\t"
            "mov %[i], %[j]\n\t"
            "bne idle_loop\n\t"
            : [i] "+r" (i), [j] "+r" (j));
        int time_end = clk4->value_low;
        int time_total = time_end - time_start;
        int percent_idle = 39320 * 100 / time_total;
        bwprintf(COM2, "\0337\033[H%d%% \0338", percent_idle);
    }
}
#undef IDLE_ITERATIONS

typedef struct ttmsg {
    int MessageType;
    int t;
    int n;
} TTMsg;

void task_timetest(){
    int tid = MyTid();
    int fut_tid = WhoIs(NAME_FUT);
    TTMsg tm;
    tm.MessageType = MESSAGE_TT;
    Send(fut_tid, &tm, sizeof(tm), &tm, sizeof(tm));
    for (int i = 0; i < tm.n; i++){
        int err = Delay(tm.t);
        ASSERT(err == 0, "Error Delaying");
        bwprintf(COM2, "%d: %d, %d/%d\r\n", tid, tm.t, i+1, tm.n);
    }
}

void fut(){
    LOG("First User Task: Start\r\n");
    Create(PRIORITY_WAREHOUSE, &task_nameserver);
    Create(PRIORITY_WAREHOUSE, &task_clockserver);
    RegisterAs(NAME_FUT);

    Create(PRIORITY_IDLE, &task_idle);

    for (int i = 0; i < 4; i++){
        Create(3 + i, &task_timetest);
    }
    TTMsg tm;
    int tid, t[] = {10, 23, 33, 71}, n[] = {20, 9, 6, 3};
    for (int i = 0; i < 4; i++){
        Receive(&tid, &tm, sizeof(tm));
        tm.t = t[i];
        tm.n = n[i];
        Reply(tid, &tm, sizeof(tm));
    }
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

        if (task->last_syscall == SYSCALL_INTERRUPT){
            task->lr -= 4;
        } else {
            task->sp[0] = task->r0; // TODO ?? is there a better way to do this?
        }
        f = activate((int) task);
        task->last_syscall = f;

        handle(f, task, task_pool, &task_ready_queue);
        LOG("\r\n");
        task = schedule(&task_ready_queue);
    }
    LOG("Kernel Exiting - No More Tasks");

    return 0;
}
