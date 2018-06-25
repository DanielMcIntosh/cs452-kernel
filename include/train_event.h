#ifndef _TRAIN_EVENT_H_
#define _TRAIN_EVENT_H_
#include <syscall.h>
#include <util.h>

#define NAME_TRAIN_EVENT_COURRIER "trn_evt"

typedef struct {
    void (*func)(int arg0, int arg1, bool success);
    int arg0;
    int arg1;
    unsigned int timeout;
    char run_on_timeout;
} Runnable;

void task_train_event_courier();
void TrainEvent_Notify(int train_evt_courrier_tid, int sensor);

//if timeout == 0, never timeout
void RunWhen(int sensor, Runnable *to_run, Priority priority);

#endif //_TRAIN_EVENT_H_
