#include <train_event.h>
#include <kernel.h>
#include <syscall.h>
#include <clock.h>
#include <name.h>
#include <message.h>
#include <util.h>
#include <debug.h>
#include <terminal.h>

typedef enum te_request{
    QUEUE,
    UNQUEUE,
    NOTIFY
} RequestType;

typedef struct te_message{
    RequestType rq;
    int sensor;
} TrainEventMessage;

void task_train_event_courier() {
    RegisterAs(NAME_TRAIN_EVENT_COURIER);

    TrainEventMessage tm;
    int tid;

    int waiting_tid[NUM_SENSORS] = {0};

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));
        Reply(tid, NULL, 0);
        switch (tm.rq) {
            case QUEUE:
            {
                //set waiting task for sensor
                //ASSERT(waiting_tid[tm.sensor] == 0, "task already queued");
                if (waiting_tid[tm.sensor] != 0){
                    PANIC("\r\ntask not currently queued: %d --> %d\r\n", tm.sensor, waiting_tid[tm.sensor]);
                }
                ASSERT(tm.sensor < NUM_SENSORS && tm.sensor >= 0, "invalid sensor to wait on");
                waiting_tid[tm.sensor] = tid;
                break;
            }
            case UNQUEUE:
            {
                //unset waiting task for sensor
                //ASSERT(waiting_tid[tm.sensor] == tid, "task not queued");
                if (waiting_tid[tm.sensor] != tid){
                    PANIC("\r\ntask already queued: %d --> %d\r\n", tm.sensor, waiting_tid[tm.sensor]);
                }
                ASSERT(tm.sensor < NUM_SENSORS && tm.sensor >= 0, "invalid sensor to wait on");
                waiting_tid[tm.sensor] = 0;
                break;
            }
            case NOTIFY:
            {
                if (waiting_tid[tm.sensor] != 0) {
                    //send to the task waiting on sensor
                    MessageType wakeup = MESSAGE_WAKEUP;
                    Send(waiting_tid[tm.sensor], &wakeup, sizeof(wakeup), NULL, 0);
                    waiting_tid[tm.sensor] = 0;
                }
                break;
            }
        }
    }
}

void TrainEvent_Notify(int courier_tid, int sensor) {
    TrainEventMessage msg = {NOTIFY, sensor};
    Send(courier_tid, &msg, sizeof(msg), NULL, 0);
}

void task_runner(int sensor) {
    int caller_tid;
    Runnable to_run;
    Receive(&caller_tid, &to_run, sizeof(to_run));
    Reply(caller_tid, NULL, 0);

    int train_evt_courier_tid = WhoIs(NAME_TRAIN_EVENT_COURIER);
    
    //queue up to receive alert
    TrainEventMessage tm = {QUEUE, sensor};
    Send(train_evt_courier_tid, &tm, sizeof(tm), NULL, 0);

    int tid;
    MessageType type;
    if (to_run.timeout != 0) {
        Timeout(to_run.timeout);
    }
    Receive(&tid, &type, sizeof(type));
    Reply(tid, NULL, 0);

    if (to_run.run_on_timeout || type == MESSAGE_WAKEUP) {
        to_run.func(to_run.arg0, to_run.arg1, type == MESSAGE_WAKEUP);
    }

    if (type == MESSAGE_TIMEOUT) {
        tm.rq = UNQUEUE;
        Send(train_evt_courier_tid, &tm, sizeof(tm), NULL, 0);
    } else if (to_run.timeout != 0) {
        Receive(&tid, &type, sizeof(type));
        Reply(tid, NULL, 0);
    }
    Destroy();
}


//if timeout == 0, never timeout
void RunWhen(int sensor, Runnable *to_run, Priority priority) {
    int runner_tid = CreateWithArgument(priority, &task_runner, sensor);
    Send(runner_tid, to_run, sizeof(*to_run), NULL, 0);
}

