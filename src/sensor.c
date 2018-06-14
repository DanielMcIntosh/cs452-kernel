#include <kernel.h>
#include <sensor.h>
#include <name.h>
#include <uart.h>
#include <message.h>
#include <syscall.h>
#include <clock.h>
#include <tasks.h>
#include <command.h>
#include <debug.h>

typedef struct sensorserver{
    char sensor_status[81];
    char sensor_data[10];
    int current_sensor_query;
    int query_complete;
} SensorServer;

typedef enum sensorrequest{
    SENSOR_READ_NOTIFY,
    SENSOR_TIMEOUT
} SensorRequest;

typedef struct sensormessage{
    MessageType type;
    SensorRequest request;
    char data;
} SensorMessage;

void task_sensor_read_notifier(int servertid){
    int gettid = WhoIs(NAME_UART1_RCV);
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_READ_NOTIFY, 0};
    ReplyMessage rm = {0, 0};
    FOREVER{
        sm.data = Getc(gettid, 1);
        Send(servertid, &sm, sizeof(sm), &rm, sizeof(rm));
    }
}

void task_sensor_timeout_notifier(int servertid){
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_TIMEOUT, 0};
    ReplyMessage rm = {0, 0};
    FOREVER{
        Send(servertid, &sm, sizeof(sm), &rm, sizeof(rm));
        Delay(rm.ret);
    }
}

void task_sensor_server(){
    // concept: 2 notifiers - a timeout notifier, and a read notifier
    // server waits for bytes to be sent from com1; if it times out, we send it again
    // each time we read sensors, we output them i guess (this is definitely hacky rn but fuck it)
    // not really happy with this design
    SensorServer ss = {{0}, {0}, 0, 0};
    SensorMessage sm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    Command c = {COMMAND_SENSOR_REQUEST, 0, 0}; // TODO courier

    int mytid = MyTid(), cmdtid = WhoIs(NAME_COMMANDSERVER), puttid = WhoIs(NAME_UART2_SEND), tid;
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_read_notifier, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_timeout_notifier, mytid);
    FOREVER{
        Receive(&tid, &sm, sizeof(sm));
        switch(sm.request){
        case SENSOR_TIMEOUT:
        {
            rm.ret = 180;
            Reply(tid, &rm, sizeof(rm));
            SendCommand(cmdtid, c);
            ss.current_sensor_query = 0;
            // TODO remember the off by 1 error from before?
            // TODO switch to individual queries
            break;
        }
        case SENSOR_READ_NOTIFY:
        {
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));
            ss.sensor_data[ss.current_sensor_query++] = sm.data; 
            if (ss.current_sensor_query == 10) {
                int i, j, k;
                for (j = 0; j < 5; j++) {
                    k = 128;
                    for (i = 1; i <= 8; i++, k >>= 1 ) {
                        if (ss.sensor_data[2*j] & k) { // ith sensor is active
                            if (!ss.sensor_status[(2 * j) * 8 + i]) {
                                Putc(puttid, 2, 'A' + j);
                                Putc(puttid, 2, '0' +  i);
                                ss.sensor_status[(2 * j) * 8 + i] = 1;
                            }
                        } else {
                            ss.sensor_status[(2 * j) * 8 + i] = 0;
                        }
                    }
                    k = 128;
                    for (i = 1; i <= 8; i++, k >>= 1 ) {
                        if (ss.sensor_data[2*j+1] & k) { // ith sensor is active
                            if (!ss.sensor_status[(2*j)*8 + i + 8]) {
                                Putc(puttid, 2, 'A' + j);
                                Putc(puttid, 2, '0' +  (8+i) / 10);
                                Putc(puttid, 2, '0' + (8 + i) % 10);
                                ss.sensor_status[(2*j)*8 + i + 8] = 1;
                            }
                        } else {
                            ss.sensor_status[(2*j)*8 + i + 8] = 0;
                        }
                    }
                }
            }
            break;
        }
        default:
        {
            PANIC("INVALID SENSOR MESSAGE");
        }
        }
    }
}
