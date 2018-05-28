#include "tasks.h"
#include "minheap.h"
#include "elem.h"
#include "circlebuffer.h"
#include "bwio.h"
#include "debug.h"

//////////////////////////////////////////////////////
//  HELPERS
//////////////////////////////////////////////////////
static const char LogTable256[256] = 
{
#define LT(n) n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n
    -1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
    LT(4), LT(5), LT(5), LT(6), LT(6), LT(6), LT(6),
    LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7)
};

static inline int log_2(unsigned int v) {
    register unsigned int t; // temporaries

    return (t = v >> 8) ? 8 + LogTable256[t] : LogTable256[v];
}

void insert(TaskQueue * restrict queue, TD * restrict task, Priority priority) {
    TD *tail = queue->tails[priority];
    if (!tail) {
        queue->heads[priority] = task;
    }
    else {
        tail->rdynext = task;
    }
    task->rdynext = 0;
    queue->tails[priority] = task;
}

TD *init_task(TD *task, int parent_tid, Priority priority, int lr) {
    int base_tid = task->tid & TASK_BASE_TID_MASK;
    task->tid = base_tid | (task->use_counter << TASK_COUNTER_OFFSET);
    task->use_counter++;

    task->p_tid = parent_tid;

    task->lr = lr;
    task->sp = task->sp_base;
    task->spsr = 16;

    task->state = STATE_READY;
    task->priority = priority;

    task->rcv_queue = 0;
    task->rcv_queue_tail = 0;
    return task;
}

TD *fetch_task(TaskQueue * restrict queue) {
    if (!queue->free_queue) {
        return 0;
    }
    TD * restrict task = queue->free_queue;
    queue->free_queue = task->rdynext; 

    return task;
}

//////////////////////////////////////////////////////
//  EXPOSED
//////////////////////////////////////////////////////
int task_init(TD *task_pool, TaskQueue * restrict queue, char * restrict stack_space, unsigned int stack_space_size) {
    int STACK_SPACE_PER_TASK = stack_space_size/TASK_POOL_SIZE - 4;

    for (int i = 0; i < TASK_POOL_SIZE; ++i) {
        //we write to the stack from the bottom up,
        //so start with sp_base at the end of the allocated range for this task
        stack_space += STACK_SPACE_PER_TASK;
        task_lookup(task_pool, i)->sp_base = (int) stack_space;
        task_lookup(task_pool, i)->state = STATE_DESTROYED;
        task_lookup(task_pool, i)->tid = i;
        task_lookup(task_pool, i)->use_counter = 0;
    }

    for (int i = 0; i < TASK_POOL_SIZE - 1; ++i) {
        task_lookup(task_pool, i)->rdynext = task_lookup(task_pool, i+1);
    }
    task_lookup(task_pool, TASK_POOL_SIZE-1)->rdynext = 0;

    for (int i = 0; i < NUM_PRIORITIES; ++i) {
        queue->heads[i] = 0;
        queue->tails[i] = 0;
    }
    queue->free_queue = task_pool;
    return 0;
}

int task_getTid(TD *task) {
    return task->tid;
}

int task_getParentTid(TD *task) {
    return task->p_tid;
}

TD *task_nextActive(TaskQueue * restrict queue) {
    for (int ready = 0; ready < NUM_PRIORITIES; ++ready) {
        if (queue->heads[ready]) {

            TD * restrict active = queue->heads[ready];
            queue->heads[ready] = active->rdynext;

            if (!queue->heads[ready]) {
                queue->tails[ready] = 0;
            }

            return active;
        }
    }
    return 0;
}

int task_react_to_state(TD * restrict task, TaskQueue * restrict queue) {
    switch (task->state) {
        case STATE_READY:
        {
            insert(queue, task, task->priority);
            break;
        }
        case STATE_SEND_BLOCKED:
        case STATE_RECEIVE_BLOCKED:
        case STATE_REPLY_BLOCKED:
        {
            break;
        }
        case STATE_ZOMBIE:
        {
            break;
        }
        case STATE_DESTROYED:
        {
            task->rdynext = queue->free_queue;
            queue->free_queue = task;
            break;
        }
    }
    return 0;
}

int task_create(TaskQueue * restrict queue, int parent_tid, Priority priority, int lr) {
    TD * restrict task;

    if (priority >= NUM_PRIORITIES) {
        return -1;
    }
    task = fetch_task(queue);
    if (!task) {
        return -2;
    }

    init_task(task, parent_tid, priority, lr);

    // Store registers on stack
    int task_sp = task->sp, task_sp_out;
    __asm__(
        "mov r0, %[task_sp_in]\n\t"
        "stmdb r0!, {r4-r12,r14}\n\t"
        "mov %[task_sp_out], r0"
        : [task_sp_out]"=r"(task_sp_out)
        : [task_sp_in]"r"(task_sp)
        : "r0"
    );
    task->sp = task_sp_out;
    LOGF("New task = %x\t", task);
    LOGF("lr = %x\t", task->lr);
    LOGF("sp = %x\r\n", task->sp);
    //insert the new task into the queues, the old one will be handled in task_react_to_state
    insert(queue, task, priority);

    return task_getTid(task);
}

