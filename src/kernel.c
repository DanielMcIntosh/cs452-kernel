#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>

#define FOREVER for(;;)
#define STACK_SPACE_SIZE 0x1000

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

int handle(int a, TD *task){
    bwprintf(COM2, "HANDLE: %d, %d", a, task);
    switch(a) {
        case SYSCALL_CREATE:
        {
            break;
        }
        case SYSCALL_TID:
        {
            task->r0 = task_getTid(task);
            break;
        }
        case SYSCALL_PTID:
        {
            task->r0 = task_getParentTid(task);
            bwprintf(COM2, "PTID: %d", task->r0);
            break;
        }
        case SYSCALL_PASS:
        {
            bwputstr(COM2, "PASS called\r\n");
            break;
        }
        case SYSCALL_EXIT:
        {
            break;
        }
        default:
        {
            break;
        }
    }
    return a;
}

void fak(){
        bwputstr(COM2, "#f\r\n");
    FOREVER {
        bwputstr(COM2, "#believe\r\n");
        Pass();
        int tid = MyTid();
        int ptid = MyParentTID();
        bwprintf(COM2, "%d, %d\r\n", tid, ptid);
    }

}

int main(){
    bwputstr(COM2, "Start!");

    kernel_init();
    bwputstr(COM2, "Kernel Init!");

    char stack_space[STACK_SPACE_SIZE];
    bwputstr(COM2, "Stack Space!");

    TD task_pool[TASK_POOL_SIZE] = {(TD){0}};
    TD *task_ready_queues[NUM_PRIORITIES];
    TD *task_ready_queue_tails[NUM_PRIORITIES];
    TD *task_free_queue = task_pool;

    task_init(task_pool, task_ready_queues, task_ready_queue_tails, stack_space, STACK_SPACE_SIZE);
    bwputstr(COM2, "Task Init!");

    task_create(task_ready_queues, task_ready_queue_tails, &task_free_queue, 1, PRIORITY_HIGHEST, (int) &fak);
    task_create(task_ready_queues, task_ready_queue_tails, &task_free_queue, 1000, PRIORITY_HIGHEST, (int) &fak);
    bwputstr(COM2, "Task Created!\r\n");

    bwprintf(COM2, "&fak: ");
    int fak_ptr = (int) &fak;
    __asm__(
        "mov r4, %[fak_ptr]\n\t"
        DUMPR("r4")
        :
        : [fak_ptr] "r" (fak_ptr)
    );

    //FOREVER 
    for (int i = 0; i < 10; i++){
        TD *task = schedule(task_ready_queues, task_ready_queue_tails);
        bwputstr(COM2, "\r\nTask Scheduled!\r\ntask->lr = ");
        int task_lr = task->lr;
        __asm__(
            "mov r4, %[task_lr]\n\t"
            DUMPR("r4")
            :
            : [task_lr] "r" (task_lr)
        );
        bwputstr(COM2, "\r\n");
        int f = activate(task);
        bwputc(COM2, f);
        bwputstr(COM2, "\r\n");

        handle(f, task);
    }
    //*/
    bwputstr(COM2, "Passed?");

    return 0;
}
