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

int handle(int a, TD *task, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue){
    bwprintf(COM2, "HANDLE: %d, %d\t", a, task);
    switch(a) {
        case SYSCALL_CREATE:
        {
            bwprintf(COM2, "TASK CREATE: %d, %d, %d\r\n", task_getTid(task), task->syscall_arg0, task->syscall_arg1);
            task->r0 = task_create(task_ready_queues, task_ready_queue_tails, task_free_queue, task_getTid(task), task->syscall_arg0, task->syscall_arg1);
            break;
        }
        case SYSCALL_TID:
        {
            task->r0 = task_getTid(task);
            bwprintf(COM2, "TID: %d\r\n", task->r0);
            break;
        }
        case SYSCALL_PTID:
        {
            task->r0 = task_getParentTid(task);
            bwprintf(COM2, "PTID: %d\r\n", task->r0);
            break;
        }
        case SYSCALL_PASS:
        {
            bwputstr(COM2, "PASS called\r\n");
            break;
        }
        case SYSCALL_EXIT:
        {
            bwputstr(COM2, "EXIT called\r\n");
            //don't re-queue the task, let it become a zombie task.
            //it will still be accessible by it's TID, since that gives us an index in the task pool
            return a;
        }
        default:
        {
            bwputstr(COM2, "UNKNOWN SYSCALL\r\n");
            break;
        }
    }
    task_enqueue(task, task_ready_queues, task_ready_queue_tails);
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

void utsk(){
    int tid = MyTid();
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
    bwputstr(COM2, "Start!");

    kernel_init();
    char stack_space[STACK_SPACE_SIZE];

    TD task_pool[TASK_POOL_SIZE];
    TD *task_ready_queues[NUM_PRIORITIES];
    TD *task_ready_queue_tails[NUM_PRIORITIES];
    TD *task_free_queue = task_pool;

    task_init(task_pool, task_ready_queues, task_ready_queue_tails, stack_space, STACK_SPACE_SIZE);

    int err = task_create(task_ready_queues, task_ready_queue_tails, &task_free_queue, 1, 4, (int) &fut);
    if (err) {
        bwprintf(COM2, "-=-=-=-=-=-=ERR = %d=-=-=-=-=-=-=-\r\n", err);
    }
    bwputstr(COM2, "Task Created!\r\n");

    bwprintf(COM2, "&fak: %x\r\n", (int) &fak);
    bwprintf(COM2, "&fut: %x\r\n", (int) &fut);
    bwprintf(COM2, "&utsk: %x\r\n", (int) &utsk);

    TD *task = schedule(task_ready_queues, task_ready_queue_tails);
    while (task){
        bwprintf(COM2, "Task Scheduled! Pr = %d\t", task->priority);
        bwprintf(COM2, "task = %x\t", (int) task);
        bwprintf(COM2, "task->lr = %x\r\n", task->lr);
        int f = activate(task);
        bwputc(COM2, f);
        bwputstr(COM2, "\r\n");

        handle(f, task, task_ready_queues, task_ready_queue_tails, &task_free_queue);
        bwputstr(COM2, "\r\n");
        task = schedule(task_ready_queues, task_ready_queue_tails);
    }
    //*/
    bwputstr(COM2, "Passed?");

    return 0;
}
