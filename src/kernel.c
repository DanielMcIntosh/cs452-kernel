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
#include <track_state.h>

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

#define IDLE_ITERATIONS 2000000
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
        int percent_idle = 157280 * 100 / time_total;
        movingavg = MOVING_AVERAGE(percent_idle, movingavg, alpha);
        StoreValue(VALUE_IDLE, movingavg);
    }
}
#undef IDLE_ITERATIONS

void task_stack_metric_printer(int terminaltid){
    LOG("STACK METRIC PRINTER INIT");
    FOREVER{
        SendTerminalRequest(terminaltid, TERMINAL_STACK_METRICS, GetValue(VALUE_STACK_AVG), GetValue(VALUE_STACK_MAX));
        Delay(10*TICKS_PER_HUNDRED_MILLIS + 1); //add 1 just so we don't trigger exactly the same time as the clock printing
    }
}

void update_stack_size_metric(TD *task, ValueStore *value_store) {
    int stack_limit = STACK_SPACE_SIZE/TASK_POOL_SIZE - 4;
    int size = task_get_stack_size(task);
    if (size >= stack_limit) {
        PANIC("TASK %d RAN PAST IT'S STACK LIMIT", task_getTid(task));
    }

    int old_avg = value_store->values[VALUE_STACK_AVG];
    int old_max = value_store->values[VALUE_STACK_MAX];

    if (size > old_max) {
        value_store->values[VALUE_STACK_MAX] = size;
    }

    int alpha = 50;
    value_store->values[VALUE_STACK_AVG] = MOVING_AVERAGE(size, old_avg, alpha);
}

void fut(){
    LOG("First User Task: Start\r\n");
    // get any important values here
    uart2->ctrl |= UARTEN_MASK;
    bwputstr(COM2, "TRACK? [A/B] >> ");
    char track = bwgetc(COM2);
    for (volatile int i = 0; i < 55; i++);
    uart2->ctrl &= ~UARTEN_MASK;

    StoreValue(VALUE_IDLE, 0); // init idle value
    StoreValue(VALUE_STACK_AVG, 0); // init avg task stack size
    StoreValue(VALUE_STACK_MAX, 0); // init max task stack size
    Create(PRIORITY_WAREHOUSE, &task_nameserver);
    Create(PRIORITY_WAREHOUSE, &task_clockserver);
    init_uart_servers();
    Create(PRIORITY_IDLE, &task_idle);
    int cmdtid = Create(PRIORITY_HIGH, &task_commandserver);
    Create(PRIORITY_HIGH, &task_terminal);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_switch_courier, cmdtid); // This is here because it must be created after both the command server and the terminal server, but with a higher priority compared to both.
    CreateWithArgument(PRIORITY_HIGH, &task_track_state, CHAR_TO_TRACK(track));
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
    task_ready_queue.task_pool = task_pool;
    ValueStore value_store = {{0}};

    task_init(&task_ready_queue, stack_space, STACK_SPACE_SIZE);

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

        update_stack_size_metric(task, &value_store);

        handle(f, task, &task_ready_queue, &value_store);
        LOG("\r\n");
        task = schedule(&task_ready_queue);
    }
    LOG("Kernel Exiting - No More Tasks");

    return 0;
}
