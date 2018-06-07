#ifndef SYS_HANDLER_H
#define SYS_HANDLER_H

#include "syscall.h"
#include "tasks.h"

Syscall handle(Syscall a, TD *task, TD *task_pool, TaskQueue *task_ready_queue);

#endif //SYS_HANDLER_H
