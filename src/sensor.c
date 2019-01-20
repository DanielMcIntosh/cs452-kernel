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
#include <terminalcourier.h>
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
    SENSOR_COURIER,
    SENSOR_TERMINAL_COURIER
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

void __attribute__((noreturn)) task_sensor_read_notifier(int servertid){
    int gettid = WhoIs(NAME_UART1_RCV);
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_READ_NOTIFY, 0};
    FOREVER{
        sm.data = Getc(gettid, 1);
        Send(servertid, &sm, sizeof(sm), NULL, 0);
    }
}

void __attribute__((noreturn)) task_sensor_timeout_notifier(int servertid){
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_TIMEOUT, 0};
    ReplyMessage rm = {0, 0};
    FOREVER{
        Send(servertid, &sm, sizeof(sm), &rm, sizeof(rm));
        Delay(rm.ret);
    }
}

void __attribute__((noreturn)) task_sensor_courier(int servertid){
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_COURIER, 0};
    CourierMessage cm = {0, {0, 0, {.arg2 = 0}}};
    int commandtid = WhoIs(NAME_COMMANDSERVER);
    FOREVER{
        int err = Send(servertid, &sm, sizeof(sm), &cm, sizeof(cm));
        ASSERT(err >= 0, "SEND ERROR");
        SendCommand(commandtid, cm.c);
    }
}

int ss_notify_terminal_buffer(int sensortid, TerminalReq *treq) {
    ASSERT(treq != NULL, "null TerminalRequest output");
    TerminalCourierMessage tcm;
    SensorMessage sm = {MESSAGE_SENSOR, SENSOR_TERMINAL_COURIER, 0};
    int r = Send(sensortid, &sm, sizeof(sm), &tcm, sizeof(tcm));
    if (r < 0) return r;
    *treq = tcm.req;
    return r;
}

void __attribute__((noreturn)) task_sensor_server(int trackstatetid){
    // concept: 2 notifiers - a timeout notifier, and a read notifier
    // server waits for bytes to be sent from com1; if it times out, we send it again
    SensorServer ss = {{0}, 0, 0, 0, 0, 1};
    SensorMessage sm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    CourierMessage cm = {MESSAGE_SENSOR_COURIER, {COMMAND_SENSOR_REQUEST, 0, {.arg2 = 0}}};
    circlebuffer_t cb_terminal;
    char cb_terminal_buf[SENSOR_TERMINAL_BUFFER_SIZE];
    cb_init(&cb_terminal, cb_terminal_buf, COMMAND_TERMINAL_BUFFER_SIZE);
    TerminalCourier tc = {-1, &cb_terminal};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_terminal_courier, MyTid(), (int) &ss_notify_terminal_buffer);

    int mytid = MyTid(), tid;
    //int timeout_on = 0;
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_read_notifier, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_timeout_notifier, mytid);
    //TODO use a generic courrier to make this cleaner - see terminalcourier.c for more information on this
    CreateWithArgument(PRIORITY_NOTIFIER, &task_sensor_courier, mytid);

    int time_of_resp = 0;
    ss.send_cm = 1;
    ss.query_complete = 1;
    int cur_time = Time();
    FOREVER{
        Receive(&tid, &sm, sizeof(sm));
        switch(sm.request){
        case SENSOR_COURIER:
        {
            if (ss.send_cm){
                cm.c.arg1 = 0x85;
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
                tc_send(&tc, TERMINAL_FLAGS_SET, STATUS_FLAG_SENSOR_TIMEOUT, 0);
                //timeout_on = 1;
                if (ss.courier_tid != 0){
                    cm.c.arg1 = 0x85;
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

            Reply(tid, NULL, 0);
            if (ss.current_sensor_query == 0) {
                cur_time = Time();
            }
            ss.sensor_data[(ss.current_sensor_query++) % 2] = sm.data;
            if (ss.current_sensor_query % 2 == 0) {
                /*
                if (timeout_on) {
                    tc_send(&tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_SENSOR_TIMEOUT, 0);
                    timeout_on = 0;
                }*/
                SensorData s = {ss.current_sensor_radix, (ss.sensor_data[0] << 8) | ss.sensor_data[1], time_of_resp};
                NotifySensorData(trackstatetid, s);
                ss.current_sensor_radix = (ss.current_sensor_radix + 1) % 5;

                if (ss.current_sensor_query == 10) {
                    ss.current_sensor_query = 0;
                    ss.query_complete = 1;
                    if (ss.courier_tid != 0){
                        cm.c.arg1 = 0x85;
                        Reply(ss.courier_tid, &cm, sizeof(cm));
                        ss.courier_tid = 0;
                    } else 
                        ss.send_cm = 1;
                }
            } else {
                time_of_resp = cur_time;
            }
            break;
        }
        case SENSOR_TERMINAL_COURIER:
        {
            tc_update_notifier(&tc, tid);
            break;
        }
        default:
        {
            PANIC("INVALID SENSOR MESSAGE");
        }
        }
    }
}
