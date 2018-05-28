#ifndef MSG_METRICS
#define MSG_METRICS

#define SND_NAME "snd"
#define RCV_NAME "rcv"

//OPTIONS
#define MSG_SIZE 4
#define CACHE 1
#define PRIORITY_SND PRIORITY_MID
#define PRIORITY_RCV PRIORITY_LOW
#define ITERATIONS 50000


void task_msg_metrics();

#endif //MSG_METRICS
