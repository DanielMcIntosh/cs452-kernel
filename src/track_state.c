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
    int total_trains;
    int active_train_map[NUM_TRAINS];
    Train active_trains[MAX_CONCURRENT_TRAINS];
    Switch switches[NUM_SWITCHES+1];
    Sensor sensors[NUM_SENSORS];
    track_node track[TRACK_MAX];
} TrackState;

#define TRAIN(ts, tr) ((ts)->active_trains[(ts)->active_train_map[(tr)]])

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
    Switch merges[NUM_SWITCHES+1]; // for doing reverse distance search later
} TrackPath;

typedef struct tsmessage{
    const MessageType type;
    const TrackStateRequest request;
    union {
        int data;
        SensorData sensor_data;
        SwitchData switch_data;
        TrainData train_data;
        RouteRequest route_request;
        CalData cal_data;
        ParamData param_data;
        NewTrain new_train;
    };
} TrackStateMessage;

void init_track_state(TrackState *ts, int track){
    ts->track_number = track;
    ts->total_trains = 0;
    Train init_train = {0, FORWARD, 0, -1, -1, 0, 0};
    Sensor init_sensor = {SENSOR_OFF};
    Switch init_switch = {SWITCH_STRAIGHT};

    for (int i = 0; i < NUM_TRAINS; ++i) {
        ts->active_train_map[i] = -1;
    }

    for (int i = 0; i < MAX_CONCURRENT_TRAINS; i++){
        ts->active_trains[i] = init_train;
    }
    for (int i = 0; i < NUM_SENSORS; i++){
        ts->sensors[i] = init_sensor;
    }
    for (int i = 0; i < NUM_SWITCHES; i++){
        ts->switches[i] = init_switch;
    }
    if (track == TRACK_A)
        init_tracka(ts->track);
    else
        init_trackb(ts->track);
}

static track_node* predict_next_sensor(const Switch * restrict ts, const track_node *last_sensor, const Switch * restrict path, int * restrict distance){
    // basic plan for this: linked list search: follow state given by TrackState
    // Stop when another sensor is found

    track_node *n = last_sensor->edge[DIR_AHEAD].dest;
    *distance = last_sensor->edge[DIR_AHEAD].dist;
    track_edge *e;
    while (n != NULL && n->type != NODE_SENSOR){
        switch (n->type) {
        case (NODE_BRANCH):
        {
            //ASSERT(!(path &&path->merges[SWCLAMP(n->num)].state != SWITCH_UNKNOWN && path->switches[SWCLAMP(n->num)].state != SWITCH_UNKNOWN), "path covers branch & merge?");
            if (path && path[SWCLAMP(n->num)].state != SWITCH_UNKNOWN)
                e = &n->edge[STATE_TO_DIR(path[SWCLAMP(n->num)].state)];
            else 
                e = &n->edge[STATE_TO_DIR(ts[SWCLAMP(n->num)].state)];

            *distance += e->dist;
            n = e->dest;
            break;
        }
        case (NODE_ENTER):
        case (NODE_MERGE):
        {
            *distance += n->edge[DIR_AHEAD].dist;
            n = n->edge[DIR_AHEAD].dest;
            break;
        }
        case (NODE_EXIT):
        {
            return NULL;
        }
        default:
        {
            PANIC("in: predict_next_sensor - INVALID TRACK NODE TYPE: %d", n->type);
        }
        }
    }
    ASSERT(n == NULL || n->type == NODE_SENSOR, "While Loop broken early");

    return n;
}

static int reverse_distance_from_node(const Switch * restrict ts, const track_node *destination, int distance, int velocity, const Switch * restrict path, int * restrict wakeup_sensor, int * restrict time_after_sensor){
    ASSERT(distance > 0, "Cannot reverse find negative distance");
    const track_node * restrict next_sensor = destination->reverse;
    int tdist = 0, cdist;
    while (tdist < distance) {
        next_sensor = predict_next_sensor(ts, next_sensor, path, &cdist);
        ASSERT(next_sensor != NULL, "next sensor == null!");
        tdist += cdist;
    }
    cdist = tdist - distance; // remaining distance (in mm)
    *wakeup_sensor = next_sensor->reverse->num;
    ASSERT(*wakeup_sensor != destination->num, "WRONG");
    *time_after_sensor = (cdist * VELOCITY_PRECISION / (velocity)); // mm / (mm/10 ms) -> 10 ms 

    return 0;
}

static int forward_distance_from_node(const Switch * restrict ts, const track_node *destination, int distance, int velocity, const Switch * restrict path, int * restrict wakeup_sensor, int * restrict time_after_sensor){
    ASSERT(distance >= 0, "Cannot forward find negative distance");
    const track_node * restrict next_sensor = destination;
    int tdist = 0, last_tdist = 0, cdist;
    while (tdist < distance) {
        *wakeup_sensor = next_sensor->num;
        last_tdist = tdist;
        next_sensor = predict_next_sensor(ts, next_sensor, path, &cdist);
        tdist += cdist;
    }
    cdist = distance - last_tdist; // remaining distance (in mm)
    *time_after_sensor = (cdist * VELOCITY_PRECISION / (velocity)); // mm / (mm/10 ms) -> 10 ms 

    return 0;
}

static int find_path_between_nodes(const track_node *origin, const track_node *dest, const track_node *previous, TrackPath * restrict l, int level){
    if (origin == dest) return 1;
    if (origin == NULL || origin->type == NODE_NONE) return 0;

    switch (origin->type){
    case (NODE_BRANCH):
    {
        if (IsVisited(l->visited, origin->num)){
            return 0;
        }
        Visit(l->visited, origin->num);

        if (find_path_between_nodes(origin->edge[DIR_STRAIGHT].dest, dest, origin, l, level+1)){
            l->switches[SWCLAMP(origin->num)].state = SWITCH_STRAIGHT;
            return 1;
        } else if (find_path_between_nodes(origin->edge[DIR_CURVED].dest, dest, origin, l, level+1)){
            l->switches[SWCLAMP(origin->num)].state = SWITCH_CURVED;
            if (origin->num >= 153){
               l->switches[SWCLAMP(SW3_COMPLEMENT(origin->num))].state = SWITCH_STRAIGHT; 
            }
            return 1;
        }
        return 0;
    }
    case (NODE_MERGE):
    {
        // figure out which side of the branch we are:
        const track_node *pr = previous->reverse;
        const track_node *br = origin->reverse;
        if (br->edge[DIR_STRAIGHT].dest == pr) {
            l->merges[SWCLAMP(origin->num)].state = SWITCH_STRAIGHT;
        } else {
            l->merges[SWCLAMP(origin->num)].state = SWITCH_CURVED;
        }
        // no break on purpose: should fall through to the next case.
    }
    case (NODE_SENSOR):
    case (NODE_ENTER):
    {
        return find_path_between_nodes(origin->edge[DIR_AHEAD].dest, dest, origin, l, level+1);
    }
    case (NODE_EXIT):
    {
        return 0;
    }
    default:
    {
        PANIC("in: find_path_between_nodes - INVALID TRACK NODE TYPE: %d", origin->type);
    }
    }
    return 0;
}


static inline int get_active_train_from_sensor(TrackState *ts, const int sensor) {
    int distance;
    for (int skipped = 0; skipped < 4; ++skipped) {
        for (int cur = 0; cur < ts->total_trains; ++cur) {
            int next_sensor = ts->active_trains[cur].next_sensor;
            for (int i = 0; unlikely(i < skipped); ++i) {
                next_sensor = predict_next_sensor(ts->switches, &(ts->track[next_sensor]), NULL, &distance)->num;
            }
            if (next_sensor == sensor) {
                return cur;
            }
        }
    }
    return -1;
}

static inline int sendTrackState(int trackstatetid, const TrackStateMessage *msg){
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

int NotifyTrainSpeed(int trackstatetid, TrainData data){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_TRAIN_SPEED, {.train_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifyCalibrationResult(int trackstatetid, CalData data) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_CAL, {.cal_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifyParam(int trackstatetid, ParamData data) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_PARAM, {.param_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifyNewTrain(int trackstatetid, NewTrain data) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_NEW_TRAIN, {.new_train = data}};
    return sendTrackState(trackstatetid, &msg);
}

int GetSwitchState(int trackstatetid, int sw){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = SWITCH, {.data = sw}};
    return sendTrackState(trackstatetid, &msg);
}

int GetRoute(int trackstatetid, RouteRequest req, RouteMessage *rom){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = ROUTE, {.route_request = req}};
    int r = Send(trackstatetid, &msg, sizeof(msg), rom, sizeof(*rom));
    return (r >= 0 ? 0 : -1);
}

int GetShort(int trackstatetid, int distance, ShortMessage *sm){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = SHORT, {.data = distance}};
    int r = Send(trackstatetid, &msg, sizeof(msg), sm, sizeof(*sm));
    return (r >= 0 ? 0 : -1);
}
    
int GetTrainSpeed(int trackstatetid, int train){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = TRAIN_SPEED, {.data = train}};
    return sendTrackState(trackstatetid, &msg);
}

void task_track_state(int track){
    RegisterAs(NAME_TRACK_STATE);
    const int puttid = WhoIs(NAME_TERMINAL);
    const int train_evt_courrier_tid = Create(PRIORITY_MID, &task_train_event_courier);

    TrackState ts;
    init_track_state(&ts, track);

    TrackStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    RouteMessage rom = {0, 0, {{SWITCH_UNKNOWN}}};
    int tid;

    int predicted_velocity[NUM_SPEEDS] = 
    { 0, 0, 0, // 0->2
        VELOCITY_PRECISION, VELOCITY_PRECISION, //3, 4
        2 * VELOCITY_PRECISION, 2 * VELOCITY_PRECISION, // 5, 6
        3 * VELOCITY_PRECISION, 3 * VELOCITY_PRECISION, // 7, 8
        4 * VELOCITY_PRECISION, 5 * VELOCITY_PRECISION, // 9, 10
        5 * VELOCITY_PRECISION, 6 * VELOCITY_PRECISION, // 11, 12
        7 * VELOCITY_PRECISION, 8 * VELOCITY_PRECISION // 13, 14
    };
    int stopping_distance[NUM_SPEEDS] = 
    { 0, 10, 20,
        30, 40,
        50, 60,
        70, 80,
        90, 150,
        400, 800,
        1500, 2000
    };

    int short_speed[NUM_SHORTS] = 
    {
         0, 
         8,  8,  8,  8,  9,
        10, 10, 10, 14, 12
    };
    int short_delay[NUM_SHORTS] =  // increments of 2 cm: 2 cm -> 20 cm
    {
         0, 
         65,  80,  92, 100, 110, // i guess this is calibrated now
        120, 122, 130, 135, 141
    };

    int current_speed = 0;

    int last_sensor_time = 0;
    int last_sensor_distance = 0;

    int next_sensor_predict_time = 0;

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));

        switch (tm.request){
        case (TRAIN_SPEED):
        {
            rm.ret = TRAIN(&ts, (int) tm.data).speed;
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
            int object = tm.route_request.position.object;
            int distance_past = tm.route_request.position.distance_past;
            int tr = tm.route_request.train;
            Train *train = &(TRAIN(&ts, tr));

            current_speed = train->speed;
            int next_sensor = train->next_sensor;
            int last_sensor = train->last_sensor;

            ASSERT(current_speed != 0, "Trying to find route with speed == 0!");

            if (next_sensor < 0) {
                int __attribute__((unused)) distance;
                next_sensor = predict_next_sensor(ts.switches, &ts.track[last_sensor], NULL, &distance)->num;
            }

            const track_node *d = &ts.track[object], *n = &ts.track[next_sensor], *o = &ts.track[last_sensor];

            TrackPath tp = {{0}, {{SWITCH_UNKNOWN}}, {{SWITCH_UNKNOWN}}}; 
            int possible = find_path_between_nodes(n, d, o, &tp, 0);
            if (possible) {
                if (stopping_distance[current_speed] > distance_past) {
                    //make sure we actually have enough distance to stop in
                    int dist_needed = stopping_distance[current_speed] - distance_past;
                    const track_node *walk_node = n;
                    for (int walked_dist = 0, temp; walked_dist < dist_needed; walked_dist += temp) {
                        if (walk_node == d) {
                            walk_node = predict_next_sensor(ts.switches, walk_node, tp.switches, &temp);
                            //add to &tp a loop from d to d
                            find_path_between_nodes(walk_node, d, d, &tp, 0);
                        }
                        walk_node = predict_next_sensor(ts.switches, walk_node, tp.switches, &temp);
                    }


                    int err = reverse_distance_from_node(ts.switches, d, dist_needed, predicted_velocity[current_speed], tp.merges, &rom.end_sensor, &rom.time_after_end_sensor);
                    ASSERT(err == 0, "Reverse Distance Failed");
                } else {
                    int dist_needed = distance_past - stopping_distance[current_speed];
                    int err = forward_distance_from_node(ts.switches, d, dist_needed, predicted_velocity[current_speed], tp.switches, &rom.end_sensor, &rom.time_after_end_sensor);
                    ASSERT(err == 0, "Forward Distance Failed");
                }
                for (int i = 1; i <= NUM_SWITCHES; i++){
                    if (tp.switches[i].state != ts.switches[i].state) {
                        rom.switches[i] = tp.switches[i]; // pick up differences and unnknowns
                    } else {
                        rom.switches[i].state = SWITCH_UNKNOWN;
                    }
                }
            } else {
                PANIC("failed to find path between %d and %d", object, next_sensor);
                rom.end_sensor = -1;
                for (int i = 0; i <= NUM_SWITCHES; i++){
                    rom.switches[i].state = SWITCH_UNKNOWN; // pick up differences and unnknowns
                }
            }
            Reply(tid, &rom, sizeof(rom));
            break;
        }
        case (SHORT):
        {
            int distance = tm.data / (10 * INCREMENT_SHORT); // mm -> SHORT_INCREMENT CM
            ASSERT(distance >= 0 && distance < NUM_SHORTS, "invalid short move");
            ShortMessage sm = {short_speed[distance], short_delay[distance]};
            Reply(tid, &sm, sizeof(sm));
            break;
        }
        case (NOTIFY_SENSOR_DATA):
        {
            Reply(tid, &rm, sizeof(rm));
            SensorData f = tm.sensor_data;
            int k = 1 << 15;
            for (int i = 0; i <= 15; i++, k >>= 1){
                int sensor = SENSOR_PAIR_TO_SENSOR(f.radix, i);
                if (likely(!(f.data & k))) {
                    ts.sensors[sensor].state = SENSOR_OFF;
                }
                else if (ts.sensors[sensor].state != SENSOR_ON){
                    ts.sensors[sensor].state = SENSOR_ON;

                    TrainEvent_Notify(train_evt_courrier_tid, sensor);

                    //TODO change all of this to use RunWhen instead of running here
                    //first need to be able to have multiple tasks waiting per sensor

                    SendTerminalRequest(puttid, TERMINAL_SENSOR, sensor, f.time); // TODO courier
                    // TODO: this process might eventually take too long to happen here - delegate it to another process at some point?

                    if (unlikely(ts.total_trains <= 0)) {
                        continue;
                    }
                    int active_train = get_active_train_from_sensor(&ts, sensor);
                    ASSERT(active_train >= 0, "Could not find which train hit sensor");

                    //we haven't reset our calculations && we actually hit the sensor we expected to (and not the one after?)
                    if (ts.active_trains[active_train].last_sensor >= 0 && sensor == ts.active_trains[active_train].next_sensor) {
                        int last_error_time = (next_sensor_predict_time - f.time);
                        int last_error_dist = last_error_time * predicted_velocity[current_speed] / VELOCITY_PRECISION;
                        SendTerminalRequest(puttid, TERMINAL_SENSOR_PREDICT, last_error_time, last_error_dist);

                        // Time is in clock-ticks, velocity is in mm/(clock-tick) -> error is in units of mm
                        int dt = f.time - last_sensor_time;
                        int new_velocity = last_sensor_distance * VELOCITY_PRECISION / dt;
                        predicted_velocity[current_speed] = MOVING_AVERAGE(new_velocity, predicted_velocity[current_speed], 15);
                    }

                    int distance;
                    const track_node *c = &(ts.track[SENSOR_TO_NODE(sensor)]); // last known train position
                    const track_node *n = predict_next_sensor(ts.switches, c, NULL, &distance); // next predicted train position

                    next_sensor_predict_time = f.time + distance * VELOCITY_PRECISION / predicted_velocity[current_speed];
                    SendTerminalRequest(puttid, TERMINAL_VELOCITY_DEBUG, predicted_velocity[current_speed], n->num);
                    SendTerminalRequest(puttid, TERMINAL_DISTANCE_DEBUG, distance, 0);

                    ts.active_trains[active_train].last_sensor = sensor;
                    last_sensor_time = f.time;
                    last_sensor_distance = distance;
                    ts.active_trains[active_train].next_sensor = n->num;
                }
            }
            break;
        }
        case (NOTIFY_TRAIN_SPEED):
        {
            Reply(tid, &rm, sizeof(rm));
            TrainData data = tm.train_data;
            current_speed = data.speed;
            TRAIN(&ts, (int) data.train).speed = data.speed;
            TRAIN(&ts, (int) data.train).last_sensor = -1; // reset prediction;
            break;
        }
        case (NOTIFY_TRAIN_DIRECTION):
        {
            Reply(tid, &rm, sizeof(rm));

            break;
        }
        case (NOTIFY_SWITCH):
        {
            Reply(tid, &rm, sizeof(rm));
            SwitchData data = tm.switch_data;
            ts.switches[SWCLAMP(data.sw)].state = data.state;
            break;
        }
        case (NOTIFY_CAL):
        {
            Reply(tid, &rm, sizeof(rm));
            CalData data = tm.cal_data;

            ASSERT(data.iteration < CAL_ITERATIONS, "iteration >= CAL_ITERATIONS");

            //adjust stopping_distance
            int interval = BASE_STOP_DIST_ADJUSTMENT;
            for (int i = 0; i < data.iteration; ++i) {
                interval = interval * 4 / 5;
            }
            ASSERT(interval >= 10, "interval < 10");
            stopping_distance[data.speed] += (data.triggered ? 1 : -1) * interval;
            if (stopping_distance[data.speed] <= 0) {
                stopping_distance[data.speed] = 1;
            }
            break;
        }
        case (NOTIFY_PARAM):
        {
            Reply(tid, &rm, sizeof(rm));
            ParamData data = tm.param_data;
            int key = data.key / (10 * INCREMENT_SHORT); // mm -> SHORT_INCREMENT CM

            if (data.param == PARAM_SPEED) {
                short_speed[key] = data.value;
            } else if (data.param == PARAM_DELAY) {
                short_delay[key] = data.value;
            }
            break;
        }
        case (NOTIFY_NEW_TRAIN):
        {
            Reply(tid, &rm, sizeof(rm));
            NewTrain data = tm.new_train;

            ts.active_train_map[data.train] = ts.total_trains;
            ts.active_trains[ts.total_trains].next_sensor = data.sensor;
            ++ts.total_trains;
            break;
        }
        default:
        {
            PANIC("Track State Server: Unhandled request type: %d", tm.request);
        }
        }
    }
}
