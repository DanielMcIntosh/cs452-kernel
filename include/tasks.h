#ifndef TASKS_H
#define TASKS_H

#define TASK_COUNTER_OFFSET 8
#define TASK_POOL_SIZE (0x1 << (TASK_COUNTER_OFFSET - 1))
#define TASK_BASE_TID_MASK (TASK_POOL_SIZE - 1)

typedef enum {
    PRIORITY_HIGHEST,
    PRIORITY_WAREHOUSE,
    PRIORITY_HIGH,
    PRIORITY_MID,
    PRIORITY_LOW,
    PRIORITY_LOWEST = 6,
    PRIORITY_IDLE,
    NUM_PRIORITIES
} Priority;

typedef enum {
    STATE_READY,
    STATE_REPLY_BLOCKED,
    STATE_SEND_BLOCKED,
    STATE_RECEIVE_BLOCKED,
    STATE_ZOMBIE,
    STATE_DESTROYED,
} State;

typedef struct taskdesc {
    // DO NOT MODIFY THIS STRUCT WITHOUT A LOT OF WARNING
    
    //use task descriptors as the ready queues to avoid allocating extra memory
    struct taskdesc *rdynext;

    int tid;
    int p_tid;
    int sp_base; // for reuse
    int lr;
    // ____________DO NOT______________
    //             MODIFY
    //           THIS STRUCT 
    //     WITHOUT A LOT OF WARNING

    // The offsets are hard coded into asm/activate.s, so changing the fields WILL break the kernel.
    int sp;
    int spsr;
    int r0; //return value, since we might not be returning to this task which called 
    int syscall_args[5];

    int use_counter;

    State state;
    Priority priority;

    struct taskdesc *rcv_queue;
    struct taskdesc *rcv_queue_tail;
} TD;

typedef struct {
    TD *heads[NUM_PRIORITIES];
    TD *tails[NUM_PRIORITIES];
    TD *free_queue;
    unsigned int ready_bitfield : NUM_PRIORITIES;
} TaskQueue;

int task_init(TD *task_pool, TaskQueue *queue, char *stack_space, unsigned int stack_space_size);

int task_getTid(TD *task);
int task_getParentTid(TD *task);

TD *task_nextActive(TaskQueue *queue);
int task_react_to_state(TD *task, TaskQueue *queue);
int task_create(TaskQueue *queue, int parent_tid, Priority priority, int lr);

static inline TD *task_lookup(TD *task_pool, int tid) {
    return &(task_pool[tid & TASK_BASE_TID_MASK]);
}

#endif //TASKS_H
