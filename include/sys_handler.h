#ifndef SYS_HANDLER_H
#define SYS_HANDLER_H

#include "syscall.h"

Syscall handle(Syscall a, TD *task, TD *task_pool, TD** task_ready_queues, TD** task_ready_queue_tails, TD** task_free_queue);

#endif //SYS_HANDLER_H