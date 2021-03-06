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
#include <train_state.h>
#include <track.h>

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
    vic2->IntEnable |= (0x1U << (IRQ_MAP[EVENT_CLK_3]        - 32)) |
                       (0x1U << (IRQ_MAP[EVENT_UART_1_SEND]  - 32)) |
                       (0x1U << (IRQ_MAP[EVENT_UART_1_RCV]   - 32)) |
                       (0x1U << (IRQ_MAP[EVENT_UART_2_SEND]  - 32)) |
                       (0x1U << (IRQ_MAP[EVENT_UART_2_RCV]   - 32));

    // enable
    uart2->ctrl &= ~UARTEN_MASK;
    uart2->err.collective = 0;
    uart1->ctrl &= ~UARTEN_MASK;
    uart1->err.collective = 0;

    return 0;
}

TD* schedule(TaskQueue *task_ready_queue){
    return task_nextActive(task_ready_queue);
}

#define IDLE_ITERATIONS 2000000
void __attribute__((noreturn)) task_idle() {
    int i, j = 0;
    unsigned int movingavg = 99;
    unsigned int alpha = 60;
    FOREVER {
        i = IDLE_ITERATIONS;
        unsigned int time_start = clk4->value_low;        
        __asm__ volatile (
            "idle_loop:\n\t"
            "subs %[i], %[i], #1\n\t"
            "mov %[j], %[i]\n\t"
            "mov %[i], %[j]\n\t"
            "mov %[j], %[i]\n\t"
            "mov %[i], %[j]\n\t"
            "bne idle_loop\n\t"
            : [i] "+r" (i), [j] "+r" (j));
        unsigned int time_end = clk4->value_low;
        unsigned int time_total = time_end - time_start;
        unsigned int percent_idle = 157280 * 100 / time_total;
        movingavg = MOVING_AVERAGE(percent_idle, movingavg, alpha);
        StoreValue(VALUE_IDLE, movingavg & 0xFFFF);
    }
}
#undef IDLE_ITERATIONS

void __attribute__((noreturn)) task_stack_metric_printer(int terminaltid){
    LOG("STACK METRIC PRINTER INIT");
    FOREVER{
        SendTerminalRequest(terminaltid, TERMINAL_STACK_METRICS, GetValue(VALUE_STACK_AVG), GetValue(VALUE_STACK_MAX));
        Delay(10*TICKS_PER_HUNDRED_MILLIS + 1); //add 1 just so we don't trigger exactly the same time as the clock printing
    }
}

void update_stack_size_metric(TD * restrict task, ValueStore * restrict value_store) {
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
    init_track();

    StoreValue(VALUE_IDLE, 0); // init idle value
    StoreValue(VALUE_STACK_AVG, 0); // init avg task stack size
    StoreValue(VALUE_STACK_MAX, 0); // init max task stack size
    Create(PRIORITY_WAREHOUSE, &task_nameserver);
    Create(PRIORITY_WAREHOUSE, &task_clockserver);
    init_uart_servers();
    Create(PRIORITY_IDLE, &task_idle);
    int trackstate_tid  =   Create(             PRIORITY_HIGH+1,    &task_track_state);
    int trainstate_tid  =   CreateWithArgument( PRIORITY_HIGH+1,    &task_train_state,      trackstate_tid);
    int cmdtid          =   CreateWith2Args(    PRIORITY_HIGH,      &task_commandserver,    trackstate_tid, trainstate_tid);
    int term_tid        =   CreateWithArgument( PRIORITY_HIGH,      &task_terminal,         trackstate_tid);
    CreateWith2Args(PRIORITY_NOTIFIER, &task_switch_courier, cmdtid, term_tid); // This is here because it must be created after both the command server and the terminal server, but with a higher priority compared to both.
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
    value_store.values[VALUE_LAST_FN] = 51;
    value_store.values[VALUE_VSLF_ADDR] = (int) (value_store.values + VALUE_LAST_FN);

    task_init(&task_ready_queue, stack_space, STACK_SPACE_SIZE);

    int err = task_create(&task_ready_queue, 1, PRIORITY_INIT, (int) &fut, 0, 0);
    KASSERT(err == 0, "FATAL ERROR: Could not create First User Task");

    LOG("Task Created!\r\n");
    LOGF("&fut: %x\r\n", (int) &fut);

    TD *task = schedule(&task_ready_queue), *prev_task = NULL;
    int reentry_count = 0;
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
        value_store.values[VALUE_LAST_FN] = task->tid;
        value_store.values[VALUE_LAST_LR] = task->sp[13];
        f = activate((int) task);
        task->last_syscall = f;

        update_stack_size_metric(task, &value_store);

        handle(f, task, &task_ready_queue, &value_store);
        LOG("\r\n");

        //check for infinite loop                                                                       //20 is the TID of task_terminal
        if (task == prev_task && PRIORITY_IDLE > task->priority && task->priority > PRIORITY_WAREHOUSE && task_getTid(task) != 20) {
            ++reentry_count;
            KASSERT(reentry_count < 100, "infinite loop. tid = %d, priority = %d, lr = %x", task_getTid(task), task->priority, TD_lr(task));
        }
        else if (task->priority > PRIORITY_WAREHOUSE) {
            reentry_count = 0;
            prev_task = task;
        }
        
        task = schedule(&task_ready_queue);
    }

    LOG("Kernel Exiting - No More Tasks");
    vic2->IntEnableClear = 0xFFFFFFFF;

    bwprintf(COM2,"\r\n\n\n\n\n  \033[1;4;33;41m x -b 0x%x\033[0m  ", (int) value_store.values+VALUE_LAST_FN);
    return 0;
}
