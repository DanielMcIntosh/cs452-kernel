#ifndef TASKS_H
#define TASKS_H

#define TASK_COUNTER_OFFSET 5
#define TASK_POOL_SIZE (0x1 << (TASK_COUNTER_OFFSET - 1))
#define TASK_BASE_TID_MASK (TASK_POOL_SIZE - 1)

enum Priority {
	PRIORITY_HIGHEST,
	PRIORITY_LOWEST,
	NUM_PRIORITIES
}

enum State {
	STATE_READY
}

typedef struct task-descr
{
	//use task descriptors as the ready queues to avoid allocating extra memory
	struct task-desc *rdy-next;
	struct task-desc *rdy-prev;

	int tid;
	int p_tid;

	//these are probably better on the stack, but they're here for now as a reminder
	int sp;
	int lr;
	int spsr;
	int r0; //return value, since we might not be returning to this task which called 

	State state;
	Priority priority;
} TD;

int task_init();
int task_getTid(TD *task);
int task_getParentTid(TD *task);
TD *task_nextActive();
int task_create(int parent_tid, Priority priority);

static inline TD *task_lookup(TD **task_pool, int tid) {
	return task_pool[tid & TASK_BASE_TID_MASK];
}

#endif //TASKS_H
