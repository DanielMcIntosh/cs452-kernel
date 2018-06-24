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
#include <track_data.h>
#include <train_event.h>
#include <util.h>

typedef struct track{
    int track_number;
    Train trains[NUM_TRAINS];
    Switch switches[NUM_SWITCHES+1];
    Sensor sensors[NUM_SENSORS];
    track_node track[TRACK_MAX];
} TrackState;

typedef struct visited { // represents sensors
    long long switches: NUM_SWITCHES;
} Visited;

#define IsVisited(v, n) \
        ((v).switches & (1 << (SWCLAMP(n) - 1)))
#define Visit(v, n) \
        (v).switches |= (1 << (SWCLAMP(n) - 1));

typedef struct trackpath{
    Visited visited;
    Switch switches[NUM_SWITCHES+1];
} TrackPath;

typedef struct tsmessage{
    MessageType type;
    TrackStateRequest request;
    union {
        long long data;
        SensorData sensor_data;
        SwitchData switch_data;
    };
} TrackStateMessage;

void init_track_state(TrackState *ts, int track){
    ts->track_number = track;
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
        ts->sensors[i].name[3] = '\0';
    }
    for (int i = 0; i < NUM_SWITCHES; i++){
        ts->switches[i] = init_switch;
    }
    if (track == TRACK_A)
        init_tracka(ts->track);
    else
        init_trackb(ts->track);
}

static track_node* predict_next_sensor(TrackState *ts, track_node *last_sensor, int *distance){
    // basic plan for this: linked list search: follow state given by TrackState
    // Stop when another sensor is found

    track_node *n = last_sensor->edge[DIR_AHEAD].dest;
    *distance = last_sensor->edge[DIR_AHEAD].dist;
    while (n != NULL && n->type != NODE_SENSOR){
        switch (n->type) {
        case (NODE_BRANCH):
        {
            *distance += n->edge[STATE_TO_DIR(ts->switches[SWCLAMP(n->num)].state)].dist;
            n = n->edge[STATE_TO_DIR(ts->switches[SWCLAMP(n->num)].state)].dest;
            break;
        }
        case (NODE_ENTER):
        case (NODE_EXIT):
        case (NODE_MERGE):
        {
            *distance += n->edge[DIR_AHEAD].dist;
            n = n->edge[DIR_AHEAD].dest;
            break;
        }
        default:
        {
            PANIC("INVALID TRACK NODE TYPE: %d", n->type);
        }
        }
    }
    ASSERT(n == NULL || n->type == NODE_SENSOR, "While Loop broken early");

    return n;
}

static int find_path_between_nodes(int puttid, track_node *origin, track_node *dest, TrackPath * l, int level){
    if (origin == dest) return 1;
    if (origin == NULL) return 0;

    switch (origin->type){
    case (NODE_BRANCH):
    {
        if (IsVisited(l->visited, origin->num)){
            return 0;
        }
        Visit(l->visited, origin->num);

        if (find_path_between_nodes(puttid, origin->edge[DIR_STRAIGHT].dest, dest, l, level+1)){
            l->switches[SWCLAMP(origin->num)].state = SWITCH_STRAIGHT;
            return 1;
        } else if (find_path_between_nodes(puttid, origin->edge[DIR_CURVED].dest, dest, l, level+1)){
            l->switches[SWCLAMP(origin->num)].state = SWITCH_CURVED;
            if (origin->num >= 153){
               l->switches[SWCLAMP(SW3_COMPLEMENT(origin->num))].state = SWITCH_STRAIGHT; 
            }
            return 1;
        }
        return 0;
    }
    case (NODE_SENSOR):
    case (NODE_ENTER):
    case (NODE_EXIT):
    case (NODE_MERGE):
    {
        return find_path_between_nodes(puttid, origin->edge[DIR_AHEAD].dest, dest, l, level+1);
    }
    default:
    {
        PANIC("INVALID TRACK NODE TYPE: %d", origin->type);
    }
    }
    return 0;
}

static inline int sendTrackState(int trackstatetid, TrackStateMessage *msg){
    ReplyMessage rm;
    int r = Send(trackstatetid, msg, sizeof(*msg), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}
int NotifySensorData(int trackstatetid, SensorData data){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_SENSOR_DATA, {.sensor_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifySwitchStatus(int trackstatetid, SwitchData data){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_SWITCH, {.switch_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int GetSwitchState(int trackstatetid, int sw){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = SWITCH, {.data = sw}};
    return sendTrackState(trackstatetid, &msg);
}

int GetRoute(int trackstatetid, int sensor, RouteMessage *rom){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = ROUTE, {.data = SENSOR_TO_NODE(sensor)}};
    int r = Send(trackstatetid, &msg, sizeof(msg), rom, sizeof(*rom));
    return (r >= 0 ? 0 : -1);
}
    

void task_track_state(int track){
    RegisterAs(NAME_TRACK_STATE);
    int puttid = WhoIs(NAME_TERMINAL);
    int train_evt_courrier_tid = Create(PRIORITY_MID, &task_train_event_courier);

    TrackState ts;
    init_track_state(&ts, track);

    TrackStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    RouteMessage rom = {0, {{SWITCH_UNKNOWN}}};
    int tid;

    int alpha = 15;
    int VELOCITY_PRECISION = 10000;

    int predicted_velocity = 5 * VELOCITY_PRECISION; // TODO tie to train

    int last_sensor = 0;
    unsigned int last_sensor_time = 0;
    int last_sensor_distance = 0;

    int next_sensor = 0;
    int next_sensor_predict_time = 0;

    int last_error = 0;

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));

        switch (tm.request){
        case (TRAIN_SPEED):
        {
            rm.ret = ts.trains[(int) tm.data].speed;
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (SWITCH):
        {
            rm.ret = ts.switches[SWCLAMP(tm.data)].state;
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (SENSOR):
        {
            rm.ret = ts.sensors[(int) tm.data].state;
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (ROUTE):
        {
            track_node *d = &ts.track[(int) tm.data];
            track_node *n = &ts.track[next_sensor];

            TrackPath tp = {{0}, {{SWITCH_UNKNOWN}}}; 
            int possible = find_path_between_nodes(puttid, n, d, &tp, 0);
            if (possible) {
                rom.end_sensor = (int) tm.data; // TODO: second last sensor + distance
                for (int i = 1; i <= NUM_SWITCHES; i++){
                    if (tp.switches[i].state != ts.switches[i].state) {
                        rom.switches[i] = tp.switches[i]; // pick up differences and unnknowns
                    } else {
                        rom.switches[i].state = SWITCH_UNKNOWN;
                    }
                }
            } else {
                rom.end_sensor = -1;
                for (int i = 0; i <= NUM_SWITCHES; i++){
                    rom.switches[i].state = SWITCH_UNKNOWN; // pick up differences and unnknowns
                }
            }
            Reply(tid, &rom, sizeof(rom));
            break;
        }
        case (NOTIFY_SENSOR_DATA):
        {
            Reply(tid, &rm, sizeof(rm));
            SensorData f = tm.sensor_data;
            int k = 1 << 15;
            for (int i = 0; i <= 15; i++, k >>= 1){
                int sensor = SENSOR_PAIR_TO_SENSOR(f.radix, i);
                if (f.data & k) {
                    if (ts.sensors[sensor].state != SENSOR_ON){
                        ts.sensors[sensor].state = SENSOR_ON;
                        ts.sensors[sensor].lastTriggeredTime = f.time;

                        TrainEvent_Notify(train_evt_courrier_tid, sensor);

                        //TODO change all of this to use RunWhen instead of running here
                        //first need to be able to have multiple tasks waiting per sensor

                        SendTerminalRequest(puttid, TERMINAL_SENSOR, sensor, f.time); // TODO courier
                        // TODO: this process might eventually take too long to happen here - delegate it to another process at some point?

                        track_node *c = &(ts.track[SENSOR_TO_NODE(sensor)]); // last known train position
                        if (c->num == next_sensor) {
                            // Time is in ms, velocity is in cm/s -> error is in mm (?)
                            last_error = (next_sensor_predict_time - last_sensor_time) * predicted_velocity / VELOCITY_PRECISION;
                            if (last_error < 0) last_error *= -1;
                            unsigned int dt = f.time - last_sensor_time;
                            int new_velocity = last_sensor_distance * VELOCITY_PRECISION / dt;
                            predicted_velocity = MOVING_AVERAGE(new_velocity, predicted_velocity, alpha);
                        }
                        
                        int distance;
                        track_node *n = predict_next_sensor(&ts, c, &distance); // next predicted train position

                        next_sensor_predict_time = f.time + distance * VELOCITY_PRECISION / predicted_velocity;
                        SendTerminalRequest(puttid, TERMINAL_SENSOR_PREDICT, next_sensor_predict_time, last_error);
                        SendTerminalRequest(puttid, TERMINAL_VELOCITY_DEBUG, predicted_velocity, n->num);
                        SendTerminalRequest(puttid, TERMINAL_DISTANCE_DEBUG, distance, 0);

                        last_sensor = c->num;
                        last_sensor_time = f.time;
                        last_sensor_distance = distance;
                        next_sensor = n->num;
                    }
                } else {
                    ts.sensors[sensor].state = SENSOR_OFF;
                }

            }
            break;
        }
        case (NOTIFY_TRAIN_SPEED):
        {
            Reply(tid, &rm, sizeof(rm));
            ts.trains[(int) (tm.data >> 32)].speed = (int) (tm.data & 0xFFFF); // unpack train speed; TODO struct? (realistically, clean up all of the bit packing adventures - this is bad code)
            break;
        }
        case (NOTIFY_TRAIN_DIRECTION):
        {
            Reply(tid, &rm, sizeof(rm));
            ts.trains[(int) (tm.data >> 32)].direction = (int) (tm.data & 0xFFFF); // unpack train direction; TODO struct?
            break;
        }
        case (NOTIFY_SWITCH):
        {
            Reply(tid, &rm, sizeof(rm));
            ts.switches[SWCLAMP(tm.switch_data.sw)].state = tm.switch_data.state;
            break;
        }
        default:
        {
            PANIC("Track State Server: Unhandled request type: %d", tm.request);
        }
        }
    }
}
