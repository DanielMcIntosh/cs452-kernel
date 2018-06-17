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
#include <uart.h>
#include <terminal.h> 
#include <command.h>
#include <sensor.h>

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
        : : :"r0", "r1");
    // Unmask interrupts
    vic2->IntEnable |= 0x1 << (IRQ_MAP[EVENT_CLK_3] - 32) | ( 0x1 << (IRQ_MAP[EVENT_UART_1_SEND] - 32)) | (0x1 << (IRQ_MAP[EVENT_UART_1_RCV] - 32)) | (0x1 << (IRQ_MAP[EVENT_UART_2_SEND] - 32)) | (0x1 << (IRQ_MAP[EVENT_UART_2_RCV] - 32));

    // enable
    uart2->ctrl &= ~UARTEN_MASK;
    uart1->ctrl &= ~UARTEN_MASK;

    return 0;
}

TD* schedule(TaskQueue *task_ready_queue){
    return task_nextActive(task_ready_queue);
}

#define IDLE_ITERATIONS 500000
void task_idle() {
    int i, j = 0;
    int movingavg = 99;
    int alpha = 60;
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
        movingavg = (CLAMP(percent_idle, 99, 0) * alpha) / 100 + (movingavg * (100-alpha) / 100);
        StoreValue(VALUE_IDLE, movingavg);
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
    StoreValue(VALUE_IDLE, 0); // init idle value
    Create(PRIORITY_WAREHOUSE, &task_nameserver);
    Create(PRIORITY_WAREHOUSE, &task_clockserver);
    init_uart_servers();
    Create(PRIORITY_IDLE, &task_idle);
    int cmdtid = Create(PRIORITY_HIGH, &task_commandserver);
    Create(PRIORITY_HIGH, &task_terminal);
    Create(PRIORITY_HIGH, &task_sensor_server);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_switch_courier, cmdtid);
}

void check_stack_size(TD *task) {
    int stack_limit = STACK_SPACE_SIZE/TASK_POOL_SIZE - 4;
    if (task_get_stack_size(task) >= stack_limit) {
        PANIC("TASK %d RAN PAST IT'S STACK LIMIT", task_getTid(task));
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

    TD task_pool[TASK_POOL_SIZE]; // fun fact: every time we run the program, addresses move back by 80!

//    bwprintf(COM2, "%d -> %d, %d -> %d\r\n", stack_space, stack_space + STACK_SPACE_SIZE, task_pool, task_pool + TASK_POOL_SIZE);

    TaskQueue task_ready_queue;

    task_init(task_pool, &task_ready_queue, stack_space, STACK_SPACE_SIZE);

    int err = task_create(&task_ready_queue, 1, 4, (int) &fut, 0);
    ASSERT(err == 0, "FATAL ERROR: Could not create First User Task");
    LOG("Task Created!\r\n");
    LOGF("&fut: %x\r\n", (int) &fut);

    TD *task = schedule(&task_ready_queue);
    LOGF("OFFSETS: lr %d, sp %d, r0 %d, spsr %d, task %d\r\n", (int) &(task->lr)-(int)task, (int)&(task->sp)-(int)task, (int)&(task->r0)-(int)task, (int)&(task->spsr)-(int)task, task);
    int f = 0;
    while (task && f != SYSCALL_QUIT){
        LOGF("Task Scheduled! Pr = %d\t", task->priority);
        LOGF("task = %x\t", (int) task);
        LOGF("task->r0 = %d\t", task->r0);
        LOGF("task->lr = %x\r\n", task->lr);

        if (task->last_syscall != SYSCALL_INTERRUPT){
            task->sp[0] = task->r0; 
        }
        f = activate((int) task);
        task->last_syscall = f;

        check_stack_size(task);

        handle(f, task, task_pool, &task_ready_queue);
        LOG("\r\n");
        task = schedule(&task_ready_queue);
    }
    LOG("Kernel Exiting - No More Tasks");

    return 0;
}
