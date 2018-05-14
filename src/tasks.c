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
    TD *tail = queue_heads[priority]->rdyprev;
    tail->rdynext = task;
    queue_heads[priority]->rdynext = task;
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

//////////////////////////////////////////////////////
//  EXPOSED
//////////////////////////////////////////////////////
int task_init() {
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
    TD *task = *(get_free_queue());
    if (!task) {
        return -2;
    }
    *(get_free_queue()) = task->rdynext;


    init_task(task, parent_tid, priority, lr);

    insert(task, priority);

    return 0;
}

