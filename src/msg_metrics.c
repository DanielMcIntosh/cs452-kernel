#include "msg_metrics.h"
#include "syscall.h"
#include "tasks.h"
#include "name.h"
#include "clock.h"
#include "bwio.h"
#include "debug.h"

//OPTIONS
#define MSG_SIZE 4

#define PRIORITY_SND PRIORITY_MID
#define PRIORITY_RCV PRIORITY_LOW

#define ITERATIONS 50000

void snd_task() {
    RegisterAs(SND_NAME);

    char msg[MSG_SIZE] = {0};
    char rsp[MSG_SIZE] = {0};

    int tid_rcv = WhoIs(RCV_NAME);

    LOG("Send task ready\r\n");
    //send 1 message just to ensure both tasks are ready
    Send(tid_rcv, msg, sizeof(msg), rsp, sizeof(rsp));
    
    int time_start = clk4->value_low;
    for (int i = 0; i < ITERATIONS; ++i) {
        Send(tid_rcv, msg, sizeof(msg), rsp, sizeof(rsp));
    }
    int time_end = clk4->value_low;
    int time_total = time_end - time_start;
    bwprintf(COM2, "Send Time: %d, Send Avg: %d\r\n", time_total, time_total/ITERATIONS);
    Exit();
}

void rcv_task() {
    RegisterAs(RCV_NAME);
    
    //see comment in task_msg_metrics
    Create(PRIORITY_SND, &snd_task);

    char msg[MSG_SIZE] = {0};
    char rsp[MSG_SIZE] = {0};
    int tid;

    LOG("Recv task ready\r\n");
    //recieve 1 message just to make sure both tasks are ready
    Receive(&tid, msg, sizeof(msg));
    Reply(tid, msg, sizeof(msg));

    int time_start = clk4->value_low;
    for (int i = 0; i < ITERATIONS; ++i) {
        Receive(&tid, msg, sizeof(msg));
        Reply(tid, rsp, sizeof(rsp));
    }
    int time_end = clk4->value_low;
    int time_total = time_end - time_start;
    bwprintf(COM2, "Recv Time: %d, Recv Avg: %d\r\n", time_total, time_total/ITERATIONS);
    Exit();
}

void task_msg_metrics() {
    LOG("In msg_metrics:\r\n");
    LOGF("MSG SIZE = %d, SND PRIORITY = %d, RCV PRIORITY = %d\r\n", MSG_SIZE, PRIORITY_SND, PRIORITY_RCV);
    //in order to ensure that rcv has registered before send queries,
    //we actually start snd_task from rcv_task
    Create(PRIORITY_RCV, &rcv_task);
    Exit();
}

