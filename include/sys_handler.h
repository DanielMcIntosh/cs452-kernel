#ifndef SYS_HANDLER_H
#define SYS_HANDLER_H

#include "syscall.h"
#include "tasks.h"

Syscall handle(Syscall a, TD *task, TaskQueue *task_ready_queue, ValueStore *values);

#endif //SYS_HANDLER_H
