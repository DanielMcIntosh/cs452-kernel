#include "tasks.h"
#include "minheap.h"
#include "elem.h"
#include "circlebuffer.h"
#include "bwio.h"

//#define CIRCULAR

//////////////////////////////////////////////////////
//  HELPERS
//////////////////////////////////////////////////////
void insert(TD **queue_heads, TD **queue_tails, TD *task, Priority priority) {
    TD *tail = queue_tails[priority];
    if (!tail) {
        queue_heads[priority] = task;
    }
    else {
        queue_tails[priority]->rdynext = task;
    }
#ifdef CIRCULAR
    task->rdynext = queue_heads[priority];
#else
    task->rdynext = 0;
#endif
    queue_tails[priority] = task;
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

    return task;
}

int fetch_task(TD **ret, TD **free_queue) {
    if (!(*free_queue)) {
        return -2;
    }
    TD *task = *free_queue;

    //free queue is linear, so we don't worry about looping around
    *free_queue = task->rdynext; 

    *ret = task;
    return 0;
}

//////////////////////////////////////////////////////
//  EXPOSED
//////////////////////////////////////////////////////
int task_init(TD *task_pool, TD **queue_heads, TD **queue_tails, char * stack_space, unsigned int stack_space_size) {
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
        queue_heads[i] = 0;
        queue_tails[i] = 0;
    }
    return 0;
}

int task_getTid(TD *task) {
    return task->tid;
}

int task_getParentTid(TD *task) {
    return task->p_tid;
}

TD *task_nextActive(TD **queue_heads, TD **queue_tails) {
   for (int i = 0; i < NUM_PRIORITIES; ++i) {
        if (queue_heads[i]) {
            TD *active = queue_heads[i];
            queue_heads[i] = queue_heads[i]->rdynext;
#ifndef CIRCULAR
            if (!queue_heads[i]) {
                queue_tails[i] = 0;
            }
#endif
            return active;
        }
    }

    return 0;
}

int task_react_to_state(TD *task, TD **queue_heads, TD **queue_tails, TD **free_queue) {
    switch (task->state) {
        case STATE_READY:
        {
            insert(queue_heads, queue_tails, task, task->priority);
            break;
        }
        case STATE_BLOCKED:
        {
            break;
        }
        case STATE_ZOMBIE:
        {
            break;
        }
        case STATE_DESTROYED:
        {
            task->rdynext = *free_queue;
            *free_queue = task;
            break;
        }
    }
    return 0;
}

int task_create(TD **queue_heads, TD **queue_tails, TD **free_queue, int parent_tid, Priority priority, int lr) {
    TD *task;
    int err;

    if (priority >= NUM_PRIORITIES) {
        return -1;
    }
    err = fetch_task(&task, free_queue);
    if (err) {
        return err;
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
    #if DEBUG
    bwprintf(COM2, "New task = %x\t", task);
    bwprintf(COM2, "lr = %x\t", task->lr);
    bwprintf(COM2, "sp = %x\r\n", task->sp);
    #endif
    //insert the new task into the queues, the old one will be handled in task_react_to_state
    insert(queue_heads, queue_tails, task, priority);

    return task_getTid(task);
}

