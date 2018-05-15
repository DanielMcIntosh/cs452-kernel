#include "tasks.h"
#include "minheap.h"
#include "elem.h"
#include "circlebuffer.h"
#include "bwio.h"

//////////////////////////////////////////////////////
//  HELPERS
//////////////////////////////////////////////////////
void insert(TD **queue_heads, TD *task, enum Priority priority) {
    TD *head = queue_heads[priority];
    if (!head) {
        queue_heads[priority] = task;
    }
    else {
        TD *tail = head->rdyprev;

        tail->rdynext = task;
        task->rdyprev = tail;
    }

    head->rdyprev = task;
    task->rdynext = head;
}

TD *init_task(TD *task, int parent_tid, enum Priority priority, int lr) {

    static int task_counter = 0;
    int base_tid = task->tid & TASK_BASE_TID_MASK;
    task->tid = base_tid | (task_counter << TASK_COUNTER_OFFSET);

    task->p_tid = parent_tid;

    task->lr = lr;
    task->sp = task->sp_base;
    task->spsr = 16;
    //r0 needs to be handled, but might be best on the stack

    task->state = STATE_READY;
    task->priority = priority;

    return task;
}

int get_task(TD **ret, TD **free_queue) {
    if (!(*free_queue)) {
        return -2;
    }
    TD *task = *free_queue;
    *free_queue = task->rdynext; 

    //free queue doesn't actually use rdyprev, so don't bother with it
    if ((*free_queue)->rdynext == task) {
    	(*free_queue)->rdynext = 0;
    }

    *ret = task;
    return 0;
}

//////////////////////////////////////////////////////
//  EXPOSED
//////////////////////////////////////////////////////
int task_init(TD *task_pool, TD **queue_heads, char * stack_space, unsigned int stack_space_size) {
    int STACK_SPACE_PER_TASK = stack_space_size/TASK_POOL_SIZE;

    for (int i = 0; i < TASK_POOL_SIZE; ++i) {
        task_pool[i].sp_base = (int) stack_space;
        stack_space += STACK_SPACE_PER_TASK;
    }

	for (int i = 0; i < TASK_POOL_SIZE - 1; ++i) {
		task_pool[i].rdynext = &(task_pool[i+1]);
	}
	task_pool[TASK_POOL_SIZE-1].rdynext = task_pool;

	for (int i = 1; i < TASK_POOL_SIZE; ++i) {
		task_pool[i].rdyprev = &(task_pool[i-1]);
	}
	task_pool[0].rdyprev = &(task_pool[TASK_POOL_SIZE-1]);

	for (int i = 0; i < NUM_PRIORITIES; ++i) {
		queue_heads[i] = 0;
	}
	return 0;
}

int task_getTid(TD *task) {
    return task->tid;
}

int task_getParentTid(TD *task) {
    return task->p_tid;
}

TD *task_nextActive(TD **queue_heads) {
   for (int i = 0; i < NUM_PRIORITIES; ++i) {
        if (queue_heads[i]) {
            TD *active = queue_heads[i];
            queue_heads[i] = queue_heads[i]->rdynext;
            return active;
        }
    }

    return 0;
}

int task_create(TD **queue_heads, TD **free_queue, int parent_tid, enum Priority priority, int lr) {
	TD *task;
	int err;

    bwprintf(COM2, "lr = %d\r\n", lr);
	if (priority >= NUM_PRIORITIES) {
		return -1;
	}
	if ((err = get_task(&task, free_queue))) {
		return err;
	}

    init_task(task, parent_tid, priority, lr);
    bwprintf(COM2, "lr = %d\r\n", task->lr);

    // Store registers on stack
    int task_sp = task->sp;
    __asm__(
        "stmdb %[task_sp]!, {r4-r12,r14}\n\t"
        : [task_sp]"+r"(task_sp)
    );
    task->sp = task_sp;

    bwprintf(COM2, "lr = %d\r\n", task->lr);
    insert(queue_heads, task, priority);
    bwprintf(COM2, "lr = %d\r\n", task->lr);
    //*
    TD *next_task = task_nextActive(queue_heads);
    bwprintf(COM2, "lr = %d\r\n", next_task->lr);
    next_task = task_nextActive(queue_heads);
    bwprintf(COM2, "lr = %d\r\n", next_task->lr);
    //*/
    return 0;
}

