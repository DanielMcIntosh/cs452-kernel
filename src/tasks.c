#include <tasks.h>
#include <bwio.h>
#include <debug.h>
#include <syscall.h>
#include <event.h>
#include <util.h>

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
    task->sp = (int *) task->sp_base;
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
int task_init(TaskQueue * restrict queue, char * restrict stack_space, unsigned int stack_space_size) {
    int STACK_SPACE_PER_TASK = stack_space_size/TASK_POOL_SIZE - 4;

    for (int i = 0; i < TASK_POOL_SIZE; ++i) {
        //we write to the stack from the bottom up,
        //so start with sp_base at the end of the allocated range for this task
        stack_space += STACK_SPACE_PER_TASK;
        task_lookup(queue, i)->sp_base = (int) stack_space;
        task_lookup(queue, i)->state = STATE_DESTROYED;
        task_lookup(queue, i)->tid = i;
        task_lookup(queue, i)->use_counter = 0;
        task_lookup(queue, i)->last_syscall = SYSCALL_EXIT;
    }

    for (int i = 0; i < TASK_POOL_SIZE - 1; ++i) {
        task_lookup(queue, i)->rdynext = task_lookup(queue, i+1);
    }
    task_lookup(queue, TASK_POOL_SIZE-1)->rdynext = 0;

    for (int i = 0; i < NUM_PRIORITIES; ++i) {
        queue->heads[i] = 0;
        queue->tails[i] = 0;
    }
    queue->free_queue = queue->task_pool;
    for (int i = 0; i < NUM_EVENTS; ++i) {
        queue->event_wait[i] = 0;
    }
    return 0;
}

int task_getTid(TD *task) {
    return task->tid;
}

int task_getParentTid(TD *task) {
    return task->p_tid;
}

int task_get_stack_size(TD *task) {
    return task->sp_base - (int)task->sp;
}

TD *task_nextActive(TaskQueue * restrict queue) {
    for (int ready = 0; ready < NUM_PRIORITIES; ++ready) {
        if (queue->heads[ready]) {
            TD * restrict active = queue->heads[ready];
            queue->heads[ready] = active->rdynext;
            active->rdynext = NULL;
            if (!queue->heads[ready]) {
                queue->tails[ready] = NULL;
            }

            return active;
        }
    }
    return 0;
}

void task_react_to_state(TD * restrict task, TaskQueue * restrict queue) {
    switch (task->state) {
        case STATE_READY:
        {
            insert(queue, task, task->priority);
            break;
        }
        case STATE_BLK_SEND:
        case STATE_BLK_RECEIVE:
        case STATE_BLK_REPLY:
        case STATE_BLK_EVENT:
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
        default:
        {
            PANIC("unrecognized state");
        }
    }
}

int task_create(TaskQueue * restrict queue, int parent_tid, Priority priority, int lr, int arg0, int arg1) {
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
    task->sp -= 15; // 14 registers, 1 arg5
    int *sp = task->sp; 
    *sp++ = arg0;
    *sp++ = arg1;
    for (int cur_reg = 2; cur_reg <= 12; ++cur_reg){
        *sp++ = cur_reg;
    }
    *sp++ = (int)(&Exit);
    task->r0 = arg0;
    task->sp[1] = arg1;
    
    LOGF("New task = %x\t", task);
    LOGF("lr = %x\t", task->lr);
    LOGF("sp = %x\r\n", task->sp);
    task->rdynext = NULL;
    //insert the new task into the queues, the old one will be handled in task_react_to_state
    insert(queue, task, priority);

    return task_getTid(task);
}

