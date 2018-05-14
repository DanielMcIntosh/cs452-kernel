#include "tasks.h"
#include "minheap.h"
#include "elem.h"
#include "circlebuffer.h"

//////////////////////////////////////////////////////
//  PSEUDO-GLOBALS
//////////////////////////////////////////////////////
TD **get_queue_heads() {
    static TD *queue_heads[NUM_PRIORITIES] = {0};
    return queue_heads;
}

TD **get_free_queue() {
    static TD **free_head;
    return free_head;
}

//////////////////////////////////////////////////////
//  HELPERS
//////////////////////////////////////////////////////
void insert(TD *task, enum Priority priority) {
    TD **queue_heads = get_queue_heads();
    TD *head = queue_heads[priority];
    TD *tail = head->rdyprev;

    tail->rdynext = task;
    head->rdyprev = task;

    task->rdynext = head;
    task->rdyprev = tail;
}

TD *init_task(TD *task, int parent_tid, enum Priority priority, int lr) {

    static int task_counter = 0;
    int base_tid = task->tid & TASK_BASE_TID_MASK;
    task->tid = base_tid | (task_counter << TASK_COUNTER_OFFSET);

    task->p_tid = parent_tid;

    task->lr = lr;
    //sp needs to be handled
    //spsr, r0 need to be handled, but might be best on the stack

    task->state = STATE_READY;
    task->priority = priority;

    return task;
}

int get_task(TD **ret) {
	TD **free_queue = get_free_queue();
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
int task_init(TD *task_pool) {
	for (int i = 0; i < TASK_POOL_SIZE - 1; ++i) {
		task_pool[i].rdynext = task_pool+i+1;
	}
	task_pool[TASK_POOL_SIZE-1].rdynext = task_pool;

	for (int i = 1; i < TASK_POOL_SIZE; ++i) {
		task_pool[i].rdyprev = task_pool+i-1;
	}
	task_pool[0].rdyprev = task_pool+TASK_POOL_SIZE-1;

	*(get_free_queue()) = task_pool;

	TD **queue_heads = get_queue_heads();
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

TD *task_nextActive() {
    TD **queue_heads = get_queue_heads();
    for (int i = 0; i < NUM_PRIORITIES; ++i) {
        if (queue_heads[i]) {
            TD *active = queue_heads[i];
            queue_heads[i] = queue_heads[i]->rdynext;
            return active;
        }
    }

    return 0;
}

int task_create(int parent_tid, enum Priority priority, int lr) {
	TD *task;
	int err;

	if (priority >= NUM_PRIORITIES) {
		return -1;
	}
	if ((err = get_task(&task))) {
		return err;
	}

    init_task(task, parent_tid, priority, lr);

    insert(task, priority);

    return 0;
}

