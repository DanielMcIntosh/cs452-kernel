#include <track_state.h>
#include <train.h>
#include <switch.h>
#include <sensor.h>
#include <err.h>
#include <kernel.h>
#include <message.h>
#include <syscall.h>
#include <debug.h>
#include <terminal.h>
#include <name.h>

typedef struct track{
    int track;
    Train trains[NUM_TRAINS];
    Switch switches[NUM_SWITCHES];
    Sensor sensors[NUM_SENSORS];
} TrackState;

typedef struct tsmessage{
    MessageType type;
    TrackStateRequest request;
    long long data;
} TrackStateMessage;

typedef union snsrunion{
    SensorData fields;
    long long bits;
} SensorUnion;

void init_track_state(TrackState *ts, int track){
    ts->track = track;
    Train init_train = {0, FORWARD, 0, 0, 0, 0};
    Sensor init_sensor = {{0}, SENSOR_OFF, 0};
    Switch init_switch = {SWITCH_STRAIGHT};
    for (int i = 0; i < NUM_TRAINS; i++){
        ts->trains[i] = init_train;
    }
    for (int i = 0; i < NUM_SENSORS; i++){
        ts->sensors[i] = init_sensor;
        ts->sensors[i].name[0] = 'A' + i/16;
        ts->sensors[i].name[1] = '0' + ((i%16) / 10);
        ts->sensors[i].name[2] = '0' + ((i%16) % 10);
        ts->sensors[i].name[3] = 0;
    }
    for (int i = 0; i < NUM_SWITCHES; i++){
        ts->switches[i] = init_switch;
    }
}

int NotifySensorData(int trackstatetid, SensorData data){
    SensorUnion u = { .fields = data };
    TrackStateMessage msg = {MESSAGE_TRACK_STATE, NOTIFY_SENSOR_DATA, u.bits};
    ReplyMessage rm;
    int r = Send(trackstatetid, &msg, sizeof(msg), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

void task_track_state(int track){
    RegisterAs(NAME_TRACK_STATE);
    int puttid = WhoIs(NAME_TERMINAL);

    TrackState ts;
    init_track_state(&ts, track);

    TrackStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    int tid;

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));
        Reply(tid, &rm, sizeof(rm));
        switch (tm.request){
        case (TRAIN_SPEED):
        {
            rm.ret = ts.trains[(int) tm.data].speed;
            break;
        }
        case (SWITCH):
        {
            rm.ret = ts.switches[(int) tm.data].state;
            break;
        }
        case (SENSOR):
        {
            rm.ret = ts.sensors[(int) tm.data].state;
            break;
        }

        case (NOTIFY_SENSOR_DATA):
        {
            SensorUnion u = { .bits = tm.data};
            SensorData f = u.fields;
            int k = 1 << 15;
            for (int i = 1; i <= 16; i++, k >>= 1){
                if (f.data & k) {
                    if (!(ts.sensors[16 * f.radix + i].state == SENSOR_ON)){
                        ts.sensors[16 * f.radix + i].state = SENSOR_ON;
                        SendTerminalRequest(puttid, TERMINAL_SENSOR, f.radix << 16 | i, f.time); // TODO courier
                    }
                } else {
                    ts.sensors[16 * f.radix + i].state = SENSOR_OFF;
                }
                ts.sensors[16 * f.radix + i].lastTriggeredTime = f.time;
            }
            break;
        }
        case (NOTIFY_TRAIN_SPEED):
        {
            ts.trains[(int) (tm.data >> 32)].speed = (int) (tm.data & 0xFFFF); // unpack train speed; TODO struct?
            break;
        }
        case (NOTIFY_TRAIN_DIRECTION):
        {
            ts.trains[(int) (tm.data >> 32)].direction = (int) (tm.data & 0xFFFF); // unpack train direction; TODO struct?
            break;
        }
        case (NOTIFY_SWITCH):
        {
            ts.switches[(int) (tm.data >> 32)].state = (int) (tm.data & 0xFFFF); // unpack train direction; TODO struct?
            break;
        }
        default:
        {
            PANIC("Track State Server: Unhandled request type: %d", tm.request);
        }
        }
    }
}
