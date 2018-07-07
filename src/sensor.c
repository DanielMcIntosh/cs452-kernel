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
#include <terminal.h>
#include <track_state.h>

typedef struct sensorserver{
    char sensor_data[2];
    int current_sensor_query;
    int current_sensor_radix;
    int query_complete;
    int courier_tid;
    int send_cm;
} SensorServer;

typedef enum sensorrequest{
    SENSOR_READ_NOTIFY,
    SENSOR_TIMEOUT,
    SENSOR_COURIER
} SensorRequest;

typedef struct sensormessage{
    MessageType type;
    SensorRequest request;
    char data;
} SensorMessage;

typedef struct couriermessage {
    MessageType type;
    Command c;
} CourierMessage;

void task_sensor_read_notifier(int servertid){
    int gettid = WhoIs(NAME_UART1_RCV);
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_READ_NOTIFY, 0};
    FOREVER{
        sm.data = Getc(gettid, 1);
        Send(servertid, &sm, sizeof(sm), NULL, 0);
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

void task_sensor_courier(int servertid){
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_COURIER, 0};
    CourierMessage cm = {0, {0, 0, {.arg2 = 0}}};
    int commandtid = WhoIs(NAME_COMMANDSERVER);
    FOREVER{
        int err = Send(servertid, &sm, sizeof(sm), &cm, sizeof(cm));
        ASSERT(err >= 0, "SEND ERROR");
        SendCommand(commandtid, cm.c);
    }
}

void task_sensor_server(){
    // concept: 2 notifiers - a timeout notifier, and a read notifier
    // server waits for bytes to be sent from com1; if it times out, we send it again
    SensorServer ss = {{0}, 0, 0, 0, 0, 1};
    SensorMessage sm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    CourierMessage cm = {MESSAGE_SENSOR_COURIER, {COMMAND_SENSOR_REQUEST, 0, {.arg2 = 0}}};

    int mytid = MyTid(), trackstatetid = WhoIs(NAME_TRACK_STATE), tid;
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_read_notifier, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_timeout_notifier, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_courier, mytid);

    int time_of_resp = 0;
    FOREVER{
        Receive(&tid, &sm, sizeof(sm));
        switch(sm.request){
        case SENSOR_COURIER:
        {
            if (ss.send_cm){
                cm.c.arg1 = ss.current_sensor_radix;
                Reply(tid, &cm, sizeof(cm));
                ss.send_cm = 0;
            } else
                ss.courier_tid = tid;
            break;
        }
        case SENSOR_TIMEOUT:
        {
            rm.ret = 50;
            Reply(tid, &rm, sizeof(rm));

            if (!ss.query_complete){
                if (ss.courier_tid != 0){
                    cm.c.arg1 = ss.current_sensor_radix;
                    Reply(ss.courier_tid, &cm, sizeof(cm));
                    ss.courier_tid = 0;
                } else 
                    ss.send_cm = 1;

                ss.current_sensor_query = 0;
            }
            ss.query_complete = 0;
            break;
        }
        case SENSOR_READ_NOTIFY:
        {
            //we want to get the time as close to when we get the first byte as possible?
            int cur_time = Time();
            Reply(tid, NULL, 0);
            ss.sensor_data[ss.current_sensor_query++] = sm.data;
            if (ss.current_sensor_query == 2) {
                SensorData s = {ss.current_sensor_radix, (ss.sensor_data[0] << 8) | ss.sensor_data[1], time_of_resp};
                NotifySensorData(trackstatetid, s);

                ss.current_sensor_query = 0;
                ss.current_sensor_radix = (ss.current_sensor_radix + 1) % 5;
                ss.query_complete = 1;
                if (ss.courier_tid != 0){
                    cm.c.arg1 = ss.current_sensor_radix;
                    Reply(ss.courier_tid, &cm, sizeof(cm));
                    ss.courier_tid = 0;
                } else 
                    ss.send_cm = 1;
            } else {
                time_of_resp = cur_time;
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
