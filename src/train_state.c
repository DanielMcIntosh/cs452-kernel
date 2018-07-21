#include <train_state.h>
#include <track_state.h>
#include <train.h>
#include <message.h>
#include <syscall.h>
#include <name.h>
#include <debug.h>
#include <clock.h>
#include <terminal.h>
#include <track.h>
#include <terminalcourier.h>
#include <command.h>
#include <circlebuffer.h>
#include <clock.h>
#include <features.h>

#define NAV_SPEED 12
#define SHORT_COEFF_PRECISION 10000
#define ZERO_ACCEL_TOLERANCE 200


#define TRAIN(ts, tr) (&((ts)->active_trains[(ts)->active_train_map[(tr)]]))
typedef struct active_route{
    Route route;
    int end_node;
    int distance_past;
    int train;
    int remaining_distance;
    int next_step_distance;
    int idx_resrv;
    int cur_pos_idx;
    int stopped;
    int reversing;
    int last_handled_sensor;
} ActiveRoute;
#define ACTIVE_ROUTE_INIT {ROUTE_INIT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#define ACTIVE_ROUTE_DONE_ACTIONS(ar) ((ar)->stopped)
//#define ACTIVE_ROUTE_DONE_ACTIONS(ar) ((ar)->stopped && !((ar)->route.rcs[(ar)->idx_resrv].a != ACTION_NONE))
#define ACTIVE_ROUTE_COMPLETE(ar) ((ACTIVE_ROUTE_DONE_ACTIONS(ar) && (ar)->stopped))
#define ACTIVE_ROUTE_SHOULD_STOP(ar, train, dist_to_next_snsr) ((dist_to_next_snsr + (train)->stopping_distance[(train)->speed] >= (ar)->remaining_distance && !(ar)->stopped))
#define ACTIVE_ROUTE_NEXT_STEP_RV_IN_RANGE(ar, train, d)  (((ar)->route.rcs[(ar)->cur_pos_idx].a == ACTION_RV) && ((d) + (train)->stopping_distance[(train)->speed] <= (ar)->next_step_distance) && (!(ar)->reversing))
#define ACTIVE_ROUTE_NEXT_STEP_RV(ar)  (((ar)->route.rcs[(ar)->cur_pos_idx].a == ACTION_RV) && (!(ar)->reversing))
#define ACTIVE_ROUTE_SHOULD_PERFORM_ACTION(ar, resrv_dist) ((ar)->next_step_distance <= (resrv_dist) && !ACTIVE_ROUTE_DONE_ACTIONS(ar))

typedef struct train_state{
    int total_trains;
    int active_train_map[NUM_TRAINS];
    Train active_trains[MAX_CONCURRENT_TRAINS];
    Reservation reservations;
    ActiveRoute active_routes[MAX_CONCURRENT_TRAINS];
} TrainState;

#define TRAIN_STATE_INIT {0, {0}, {TRAIN_INIT}, RESERVATION_INIT, {ACTIVE_ROUTE_INIT}}

typedef union delaystop{
    int data;
    struct {
        unsigned int delay: 13;
        unsigned int rv: 1;
        unsigned int stoppos: 8;
        unsigned int distance_past: 10;
    };
} DelayStop;

typedef struct stopdata {
    unsigned int train: 8;
    unsigned int stoppos: 8;
    unsigned int distance_past: 10;
} StopData;

typedef struct tsmessage{
    const MessageType type;
    const TrainStateRequest request;
    union {
        int data;
        TrainData train_data;
        CalData cal_data;
        NewTrain new_train;
        StopData stop_data;
        SensorEvent sensor_event;
        NavigateRequest nav_req;
    };
} TrainStateMessage;

static inline int sendTrainState(int trainstatetid, const TrainStateMessage *msg) {
    ReplyMessage rm;
    int r = Send(trainstatetid, msg, sizeof(*msg), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

int __attribute__((pure))  calc_reverse_time(TrainState *ts, int activetrain){
    ASSERT(ts->active_trains[activetrain].speed !=  15, "DIVISION BY ZERO");
    return 450 / (15 - ts->active_trains[activetrain].speed) + 75; // BIG TODO
}

int __attribute__((const)) calc_reverse_time_from_velocity(int velocity, int distance_to_stop, int stopping_distance) {
    if (velocity == 0) return 0;
    return (VELOCITY_PRECISION * (distance_to_stop - stopping_distance) / velocity); 
}

int __attribute__((const)) calc_stop_time_from_vi_vf_a_d_ds(int vi, int vf, int a, int d, int ds) {
    // first, calculate portion over which we'll be accelerationg
    // vf^2 = vi^2 + 2ad -> d = (vi^2 - vf^2)/(2a)
    int vf2 = vf * vf / VELOCITY_PRECISION;
    int vi2 = vi * vi / VELOCITY_PRECISION;
    int deltav_prec = (vf2 - vi2) * (ACCELERATION_PRECISION / VELOCITY_PRECISION);
    int accel_d = deltav_prec / (2 * a);
    ASSERT(accel_d >= 0, "cannot be accelerating for negative distance (%d, %d, %d, %d, %d, %d)", vi, vf, a, d, ds, accel_d);
    //  calc time to do that acceleration
    //  vf = vi + at -> t = (vf - vi) / a
    int accel_t = (vf - vi) *(ACCELERATION_PRECISION / VELOCITY_PRECISION) / a;
    ASSERT(accel_t >= 0, "cannot be accelerating for negative time (vi %d, vf %d, a %d, d %d, ds %d, ad %d, at %d)", vi, vf, a, d, ds, accel_d, accel_t);

    int const_d = d - accel_d;
    ASSERT(const_d >= 0, "cannot be at a constant speed for negative distance (%d, %d, %d, %d, %d, %d, %d, %d)", vi, vf, a, d, ds, accel_d, accel_t, const_d);
    // t = (delta_d / vf)
    int const_t = (const_d - ds) * VELOCITY_PRECISION / vf;
    ASSERT(const_t >= 0, "cannot be at a constant speed for negative time (vi %d, vf %d, a %d, d %d, ds %d, ad %d, at %d, cd %d, ct %d)", vi, vf, a, d, ds, accel_d, accel_t, const_d, const_t);
    return accel_t + const_t;
}

int __attribute__((pure)) calc_short_delay(Train *train, int dist_millis) {
    int c = train->short_delay_coeffs[0], b = train->short_delay_coeffs[1], a = train->short_delay_coeffs[2];
    int det = b*b - (4 * a * (c - (dist_millis * SHORT_COEFF_PRECISION / 10)));
    int sqrt = fastintsqrt(det);
    int time = (-b + sqrt)/ (2*a);
    ASSERT(time > 0, "invalid time returned in calc_short_delay: %d,  a = %d, b = %d, c = %d, dist_millis = %d, det = %d, sqrt = %d", time, a, b, c, dist_millis, det, sqrt);
    return time;
}

int __attribute__((const)) calc_accel_from_vi_vf_d(int vi, int vf, int d) {
    // vf^2 = vi^2 + 2ad
    // a = (vi^2 - vf^2)/(2d)
    int vf2 = vf * vf / VELOCITY_PRECISION;
    int vi2 = vi * vi / VELOCITY_PRECISION;
    int deltav_prec = (vf2 - vi2) * (ACCELERATION_PRECISION / VELOCITY_PRECISION);
    return deltav_prec / (2 * d);
}

int notify_rv_timeout(int trainstatetid, int activetrain) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RV_TIMEOUT, .data = activetrain};
    return sendTrainState(trainstatetid, &msg);
}

int notify_rv_start(int trainstatetid, StopData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RV_START, .stop_data = data};
    return sendTrainState(trainstatetid, &msg);
}

int notify_stop(int trainstatetid, StopData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_STOP, .stop_data = data};
    return sendTrainState(trainstatetid, &msg);
}

int notify_stopped(int trainstatetid, int train) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_STOPPED, .data = train};
    return sendTrainState(trainstatetid, &msg);
}

void __attribute__((noreturn)) task_notify_rv_timeout(int delay, int activetrain){
    int tid = WhoIs(NAME_TRAIN_STATE);
    Delay(delay);
    notify_rv_timeout(tid, activetrain);
    Destroy();
}

void __attribute__((noreturn)) task_delay_reaccel(int speed, int train){
    int tid = WhoIs(NAME_COMMANDSERVER);
    Delay(20);
    Command c = {COMMAND_TR, speed, .arg2=train};
    SendCommand(tid, c);
    Destroy();
}

void __attribute__((noreturn)) task_delay_stop(int data, int train){
    int tid = WhoIs(NAME_TRAIN_STATE);
    DelayStop ds = {.data = data};
    Delay(ds.delay);
    StopData sd = {.train = train, .stoppos = ds.stoppos, .distance_past = ds.distance_past};
    //PANIC("%d %d %d %d %d %d", train, ds.stoppos, ds.distance_past, sd.train, sd.stoppos, sd.distance_past);
    if (ds.rv != 0) {
        notify_rv_start(tid, sd);
    } else {
        notify_stop(tid, sd);
    }
    Destroy();
}

void __attribute__((noreturn)) task_notify_stopped(int delay, int train){
    int tid = WhoIs(NAME_TRAIN_STATE);
    Delay(delay);
    notify_stopped(tid, train);
    Destroy();
}

static inline void trainserver_begin_reverse(TrainState *ts, int activetrain, int delay, int rvstop, int distance_past) {
    DelayStop ds = {.delay = delay, .rv = 1, .stoppos = rvstop, .distance_past = distance_past};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, ts->active_trains[activetrain].num);
    ts->active_routes[activetrain].reversing = 1;
}

int NotifyTrainSpeed(int trainstatetid, TrainData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_TRAIN_SPEED, {.train_data = data}};
    return sendTrainState(trainstatetid, &msg);
}

int NotifyNewTrain(int trainstatetid, NewTrain data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_NEW_TRAIN, {.new_train = data}};
    return sendTrainState(trainstatetid, &msg);
}

int NotifySensorEvent(int trainstatetid, SensorEvent data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_SENSOR_EVENT, {.sensor_event = data}};
    return sendTrainState(trainstatetid, &msg);
}

int NotifyReservation(int trainstatetid, int data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RESERVATION, {.data = data}};
    return sendTrainState(trainstatetid, &msg);
}

int GetTrainSpeed(int trainstatetid, int train){
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = TRAIN_SPEED, {.data = train}};
    return sendTrainState(trainstatetid, &msg);
}

int GetActiveTrain(int trainstatetid, int train){
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = ACTIVE_TRAIN, {.data = train}};
    return sendTrainState(trainstatetid, &msg);
}

int NavigateTo(int trainstatetid, NavigateRequest nav_req) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NAVIGATE, {.nav_req = nav_req}};
    ReplyMessage rm;
    int r = Send(trainstatetid, &msg, sizeof(msg), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

TrackPosition GetTrainPosition(int trainstatetid, int train) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = TRAIN_POSITION, {.data = train}};
    ReplyMessage rm;
    int r = Send(trainstatetid, &msg, sizeof(msg), &rm, sizeof(rm));
    if (r < 0) {
        TrackPosition tp = {-1, -1};
        return tp;
    }
    TrackPositionUnion tpu = {.bytes = rm.ret};
    return tpu.tp;
}

void init_train_state(TrainState *ts) {
    ts->total_trains = 0;
    Train init_train = TRAIN_INIT;
    Reservation init_reservation = {0, 0};

    for (int i = 0; i < NUM_TRAINS; ++i) {
        ts->active_train_map[i] = -1;
    }

    for (int i = 0; i < MAX_CONCURRENT_TRAINS; i++) {
        ts->active_trains[i] = init_train;
        ts->active_routes[i].stopped = 1;
    }

    ts->reservations = init_reservation;
}

static inline int get_active_train_from_sensor(TrainState *ts, const int sensor, int *distance, int rev_penalty) {
    int min_dist = INT_MAX;
    int train = 0;
    const track_node *d = &track[SENSOR_TO_NODE(sensor)];
    Reservation resrv = {0, 0};
    for (int i = 0; i < ts->total_trains; ++i) {
        const track_node *n = &track[ts->active_trains[i].last_sensor];

        Route r = ROUTE_INIT;
        int cur_dist = find_path_between_nodes(&resrv, 1, rev_penalty, n, d, &r);
        //TODO notify track_state of any switches we would have to have taken, incase a switch wasn't in the expected state
        if (cur_dist < min_dist) {
            min_dist = cur_dist;
            train = i;
        }
    }
    
    *distance = min_dist;
    return train;
}

static int ar_stop(ActiveRoute*, Train*, TerminalCourier*);
static void task_train_printer(int);

static void ts_exec_step(TrainState * restrict ts, TerminalCourier * restrict tc, ActiveRoute * restrict ar, int activetrain, int cmdtid, int distance_to_stop, const char * sig) {
    // do current step:
    RouteCommand rc = ar->route.rcs[ar->idx_resrv];
    Train *train = &(ts->active_trains[activetrain]);
    const track_node *cnode = rc.a != ACTION_NONE ? rc_to_track_node(rc, sig) : NULL; // It is possible to have an ACTION_NONE when stopping
    switch (rc.a) {
        case (ACTION_STRAIGHT):
        {
            Command cmd = {COMMAND_SW, STATE_TO_CHAR(SWITCH_STRAIGHT), .arg2 = SWUNCLAMP(rc.swmr)};
            SendCommand(cmdtid, cmd);
            break;
        }
        case (ACTION_CURVED):
        {
            Command cmd = {COMMAND_SW, STATE_TO_CHAR(SWITCH_CURVED), .arg2 = SWUNCLAMP(rc.swmr)};
            SendCommand(cmdtid, cmd);
            break;
        }
        case (ACTION_RV):
        {
            //*
            if (ar->reversing) {
                ar->next_step_distance = 99999; 
                return; // TODO this isn't quite right
            }
            //*/
            ASSERT(!ar->reversing, "CANNOT EXECUTE RV WHEN ROUTE IS ALREADY REVERSING");
            //int delay = calc_reverse_time_from_velocity(train->velocity[train->speed], distance_to_stop, train->stopping_distance[train->speed]);
            int time = Time();
            int vi = Position_CalculateVelocityNow(&train->pos, time);
            int vf = train->velocity[train->speed];
            int d = distance_to_stop;
            int dstp = train->stopping_distance[train->speed];
            int a = calc_accel_from_vi_vf_d(vi, 0, dstp); // a will always be negative
            int delay = calc_stop_time_from_vi_vf_a_d_ds(vi, vf, (vi > vf ? a : -a), d, dstp);
            trainserver_begin_reverse(ts, activetrain, delay, MERGE_TO_NODE_NSC(rc.swmr), 350);
            cnode = &track[MERGE_TO_NODE_NSC(rc.swmr)];
            ar->reversing = 1;
            break;
        }
        case (ACTION_NONE):
        {
            // route is done - stopping ought happen here.
            ar_stop(ar, train, tc);
            break;
        }
        default:
        {
            PANIC("Unhandled action type in ts_exec_step: %d", rc.a);
        }
    }
    // NOTE THE ++ in the if, also not it will only happen if ar->idx_resrv < MAX_ROUTE_COMMAND
    if (ar->idx_resrv < MAX_ROUTE_COMMAND && ar->route.rcs[++ar->idx_resrv].a != ACTION_NONE) {
        RouteCommand nc = ar->route.rcs[ar->idx_resrv];
        const track_node *nnode = rc_to_track_node(nc, sig);
        ASSERT(nc.swmr >= 0, "invalid swmr: %d\r\n", nc.swmr);
        ar->next_step_distance += distance_to_on_route(&ar->route, ar->idx_resrv - 1, cnode, nnode, sig);
    } else if (!ar->stopped && !ar->reversing) {
        const track_node *nnode = &track[ar->end_node];
        ar->next_step_distance += distance_to_on_route(&ar->route, ar->idx_resrv - 1, cnode, nnode, sig);
    } else {
        ar->next_step_distance = INT_MAX - 10000;
        tc_send(tc, TERMINAL_ROUTE_DBG2, 204, 0);
    }
}

static inline int get_resrv_dist(const ActiveRoute * restrict ar, int idx, const track_node *start, int stopping_distance) {
    ASSERT_VALID_TRACK(start);
    int cur_dist = 0, total_dist = 0;
    const track_node *resrv_end = start;

    //nth-sensor + stopping_dist + next_switch
    resrv_end = nth_sensor_on_route(2,  &ar->route, &idx, resrv_end, &cur_dist, "get_resrv_dist: 1");
    total_dist += cur_dist;
    if (resrv_end == NULL) {
        return total_dist;
    }
    ASSERT_VALID_TRACK(resrv_end);

    cur_dist = stopping_distance;
    resrv_end = forward_dist_on_route(  &ar->route, &idx, resrv_end, &cur_dist, "get_resrv_dist: 2");
    total_dist += cur_dist;
    if (resrv_end == NULL) {
        return total_dist;
    }
    ASSERT_VALID_TRACK(resrv_end);

    cur_dist = 0;
    resrv_end = next_switch_on_route(   &ar->route, &idx, resrv_end, &cur_dist, "get_resrv_dist: 3");
    total_dist += cur_dist;
    if (resrv_end == NULL) {
        return total_dist;
    }
    ASSERT_VALID_TRACK(resrv_end);
    return total_dist;
}

static inline int ts_notify_terminal_buffer(int tstid, TerminalReq * restrict treq) {
    ASSERT(treq != NULL, "null TerminalRequest output");
    TrainStateMessage tm = {MESSAGE_TRAIN_STATE, TRAIN_STATE_NOTIFY_TERMINAL_COURIER, .data = 0};
    TerminalCourierMessage tcm;
    int r = Send(tstid, &tm, sizeof(tm), &tcm, sizeof(tcm));
    if (r < 0) return r;
    *treq = tcm.req;
    return r;
}

static inline void handle_navigate(TrainState * restrict ts, TerminalCourier * restrict tc, TrainStateMessage * restrict tm, int trackstate_tid, int tid) {
    int object = tm->nav_req.position.object;
    int distance_past = tm->nav_req.position.distance_past;
    int tr = tm->nav_req.train;
    Train *train = TRAIN(ts, tr);
    ReplyMessage rm = {MESSAGE_REPLY, 0};

    int min_dist, rev_penalty;
    if (train->speed == 0){
        min_dist = 0;
        rev_penalty = 400;
        train->speed = NAV_SPEED;
    } else {
        min_dist = train->stopping_distance[train->speed] - distance_past;
        // assume the distance it takes to stop is the same as that needed to return to full speed in the opposite direction
        rev_penalty = 2*train->stopping_distance[train->speed];
    }

    ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", tr, train->last_sensor);
    int time = Time();
    TrackPosition tp = Position_CalculateNow(&train->pos, NULL, time);
    RouteRequest req = {
        .reservations = ts->reservations,
        .next = tp.object, //train->last_sensor, //train->next_sensor,
        .prev = tp.object, //train->last_sensor,
        .end = object,
        .min_dist = min_dist,
        .rev_penalty = rev_penalty
    };
    Route route = ROUTE_INIT;
    int distance = GetRoute(trackstate_tid, req, &route);
    rm.ret = distance;
    Reply(tid, &rm, sizeof(rm));
    ActiveRoute ar = ACTIVE_ROUTE_INIT;

    ar.route = route;
    ar.end_node = object;
    ar.remaining_distance = distance + distance_past;
    ar.distance_past = distance_past;
    ar.stopped = 0;
    ar.last_handled_sensor = -1;
    if (route.reverse != 0) { // TODO
        //tc_send(&tc, TERMINAL_ROUTE_DBG2, 215, ts.active_train_map[tr]);
        trainserver_begin_reverse(ts, ts->active_train_map[tr], 0, train->last_sensor, 0);  // TODO reverse starts
        ar.reversing = 1;
    } else {
        CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, NAV_SPEED, train->num); // TODO number
        ar.reversing = 0;
        Position_HandleAccel(&train->pos, &ar.route, Time(), 0, calc_accel_from_vi_vf_d(0, train->velocity[NAV_SPEED], 2 * train->stopping_distance[NAV_SPEED]));
    }
    ASSERT(0 <= ts->active_train_map[tr] && ts->active_train_map[tr] < MAX_CONCURRENT_TRAINS, "Invalid active train: %d", ts->active_train_map[tr]);
    ts->active_routes[ts->active_train_map[tr]] = ar;

    for (int i = 0; i < MAX_ROUTE_COMMAND && ar.route.rcs[i].a != ACTION_NONE; i++){
        tc_send(tc, TERMINAL_ROUTE_DBG, ar.route.rcs[i].swmr, ar.route.rcs[i].a);
    }
    //tc_send(&tc, TERMINAL_ROUTE_DBG2, 211, route.reverse);
    //tc_send(&tc, TERMINAL_ROUTE_DBG2, 214, ar.reversing);
    tc_send(tc, TERMINAL_FLAGS_SET, STATUS_FLAG_FINDING, 0);
}

static void train_on_sensor_event(TrainState *restrict ts, Train * restrict train, TerminalCourier * restrict tc, int sensor, int event_time, int distance, int tr) {
    if (train->last_sensor < 0) {
        return;
    }
    if (train->velocity[train->speed] != 0) {
        int predicted_time = train->last_sensor_time + distance * VELOCITY_PRECISION / train->velocity[train->speed];
        int error_time = (predicted_time - event_time);
        int error_dist = error_time * train->velocity[train->speed] / VELOCITY_PRECISION;
        tc_send(tc, TERMINAL_SENSOR_PREDICT, error_time, error_dist);

    }
    // Time is in clock-ticks, velocity is in mm/(clock-tick) -> error is in units of mm
    int dt = event_time - train->last_sensor_time;
    ASSERT(dt != 0, "division by 0 @ dt");
    int new_velocity = distance * VELOCITY_PRECISION / dt;
    int old_velocity = train->pos.v; // TODO this is such a fucking shitty hack fuck
    train->velocity[train->speed] = MOVING_AVERAGE(new_velocity, train->velocity[train->speed], 15);

    // Calculate acceleration over the last track section:
    int acceleration = (new_velocity - old_velocity) / dt;
    ActiveRoute *ar = &ts->active_routes[ts->active_train_map[tr]];
    if (new_velocity > train->velocity[train->speed] && train->pos.state == PSTATE_ACCEL) {
        Position_HandleConstVelo(&train->pos, &ar->route, event_time, train->velocity[train->speed]);
        tc_send(tc, TERMINAL_ROUTE_DBG2, 500, ABS(acceleration));
    }
    tc_send(tc, TERMINAL_ROUTE_DBG2, 501, new_velocity);
    tc_send(tc, TERMINAL_ROUTE_DBG2, 502, old_velocity);

    tc_send(tc, TERMINAL_VELOCITY_DEBUG, train->velocity[train->speed], acceleration);
    train->last_sensor = sensor;
    train->last_sensor_time = event_time;
    ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", tr, train->last_sensor);
}

static inline void activeroute_recalculate_distances(ActiveRoute * restrict ar, TrainState * restrict ts, TerminalCourier * restrict tc, int sensor, int distance) {
    ASSERT(distance != 0, "distance between last sensor pair shouldn't be 0");
    // recalculate next step distance and remaining distance every time
    if (ACTIVE_ROUTE_DONE_ACTIONS(ar)) {
        ar->next_step_distance = 0;
    } else {
        ASSERT(ar->idx_resrv >= ar->cur_pos_idx, "should have reserved ahead of cur_pos_idx, idx_resrv = %d, cur_pos_idx = %d, sensor = %d", ar->idx_resrv, ar->cur_pos_idx, sensor);
        ar->next_step_distance = distance_to_on_route(&ar->route, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], rc_to_track_node(ar->route.rcs[ar->idx_resrv], "ar next step recalculate rc2tn"), "ar next step recalculate");
        //have to delay initialization of next step distance until we hit the next sensor, since that's where the route actually starts
        if (ar->idx_resrv == 0 && RESERVE_TRACK) {
            bool success = reserve_track(&ar->route, ar->idx_resrv, &track[SENSOR_TO_NODE(sensor)], rc_to_track_node(ar->route.rcs[0], "init reservations"), &ts->reservations);
            if (unlikely(!success)) {
                PANIC("FIRST step already reservered!");
            }
        }
    }
    ar->remaining_distance = distance_to_on_route(&ar->route, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], &track[ar->end_node], "ar distance recalculate") + ar->distance_past;
    tc_send(tc, TERMINAL_ROUTE_DBG2, 207, ar->remaining_distance);
}

static int activeroute_distance_to_next_stop(ActiveRoute *ar, track_node *cnode, int *idx) {
    const track_node *n = cnode;
    int distance = 0;
    const track_edge *e;
    while (ar->route.rcs[*idx].a != ACTION_NONE && ar->route.rcs[*idx].a != ACTION_RV){
        e = next_edge_on_route(&(ar->route), idx, n, "Distance to next stop");
        ASSERT(e != NULL, "Should not be able to get null edge without ACTION_NONE or ACTION_RV");
        distance += e->dist;
        n = e->dest;
    }
    if (ar->route.rcs[*idx].a == ACTION_RV) {
        distance += 350; // TODO 
    }
    return distance;
}

static int ar_stop(ActiveRoute * restrict ar, Train * restrict train, TerminalCourier * restrict tc) {
    // Figure out how long we need to wait:
    //static int k = 0;
    //int stopdelay = calc_reverse_time_from_velocity(train->velocity[train->speed], ar->remaining_distance, train->stopping_distance[train->speed]);
    //if (stopdelay < 0) stopdelay = 0;
    int time = Time();
    int vi = Position_CalculateVelocityNow(&train->pos, time);
    int vf = train->velocity[train->speed];
    int d = ar->remaining_distance;
    int dstp = train->stopping_distance[train->speed];
    int a = calc_accel_from_vi_vf_d(vi, 0, dstp); // a will always be negative
    int stopdelay = calc_stop_time_from_vi_vf_a_d_ds(vi, vf, (vi > vf ? a : -a), d, dstp);
    ASSERT(stopdelay >= 0, "cannot delay negative time (vi %d vf %d a %d d %d ds %d delay %d", vi, vf, a, d, dstp, stopdelay);
    ////ASSERT(train->pos.state != PSTATE_CONST_VELO, "state shouldn't be const velo (vi %d vf %d a %d d %d ds %d delay %d", vi, vf, a, d, dstp, stopdelay);
    DelayStop ds = {.delay = stopdelay, .rv = 0, .stoppos = ar->end_node, .distance_past = ar->distance_past};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, train->num);
    ar->stopped = 1;
    tc_send(tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
    //ASSERT(FALSE, "%d %d %d %d %d %d %d", ar->remaining_distance, train->stopping_distance[train->speed], train->velocity[train->speed], ds.delay, ds.rv, ds.stoppos, ds.distance_past);
    return 999999;
}

static inline bool ar_perform_action(ActiveRoute *restrict ar, TrainState * restrict ts, TerminalCourier * restrict tc, const track_node ** resrv_start, int tr, int cmdtid) {
        const track_node *resrv_end;
        bool resrv_successful = TRUE;

        if (ar->idx_resrv < MAX_ROUTE_COMMAND && ar->route.rcs[ar->idx_resrv+1].a != ACTION_NONE) {
            resrv_end = rc_to_track_node(ar->route.rcs[ar->idx_resrv+1], "resrv_end");
        }
        else {
            resrv_end = &track[ar->end_node];
        }
        //todo will fail after first sensor right now because we've already reserved some of this
        //Therefore, we currently ignore whether we actually could reserve track
        if (RESERVE_TRACK) {
            reserve_track(&ar->route, ar->idx_resrv, *resrv_start, resrv_end, &ts->reservations);
            if (!resrv_successful) {
                return resrv_successful;
            }
            *resrv_start = resrv_end;
        }

        int nxt_step_rv_in_range = ACTIVE_ROUTE_NEXT_STEP_RV(ar);
        ts_exec_step(ts, tc, ar, tr, cmdtid, ar->next_step_distance + (nxt_step_rv_in_range ? 350 : 0), "action loop");
        return resrv_successful;
}

static void ar_short_move(TrainState * restrict ts, ActiveRoute * restrict ar, Train * restrict train, TerminalCourier * restrict tc, int distance, int shortmoveidx, int activetrain, int cmdtid) {
    // basic idea: do all of the steps between now and the end of the short move.
    // then, start the train moving at 14
    // start the server to delay stopping the train
    // if the short move is a reverse, trigger the reverse start after the delay as well.
    
    int k = 0;
    if (ar->route.rcs[ar->idx_resrv].a != ACTION_NONE) { // no more reservations exist
        const track_node *resrv_end =  rc_to_track_node(ar->route.rcs[ar->idx_resrv], "resrv_start short move");
        while (ar->idx_resrv < shortmoveidx){
            bool resrv_successful = ar_perform_action(ar, ts, tc, &resrv_end, train->num, cmdtid);
            // resrv_end is updated by this call
            if (!resrv_successful) 
                break;
            ASSERT(k++ < 20, "probably infinite loop");
        }
    }

    ar->idx_resrv++; // skip the idx of the short move
    CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, 14, train->num);
    train->speed = 14;
    int shortdelay = calc_short_delay(train, distance);

    DelayStop ds = {.delay = shortdelay, .rv = 0, .stoppos = 0, .distance_past = 350}; // TODO
    if (ar->route.rcs[shortmoveidx].a == ACTION_RV) {
        ts->active_routes[activetrain].reversing = 1;
        ds.rv = 1;
        ds.stoppos = SWITCH_TO_NODE_NSC(ar->route.rcs[ar->idx_resrv].swmr);
    } else {
        tc_send(tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
        ds.stoppos = ar->end_node;
        ds.distance_past = ar->distance_past;
        ar->stopped = 1;
    }
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, train->num);
}

static void activeroute_exec_steps(ActiveRoute * restrict ar, TrainState * restrict ts, TerminalCourier * restrict tc, int resrv_dist, int tr, int cmdtid) {
    const track_node *resrv_end = rc_to_track_node(ar->route.rcs[ar->idx_resrv], "resrv_start");
    //we assign resrv_end to resrv_start first thing in the loop
    tc_send(tc, TERMINAL_ROUTE_DBG2, 260, resrv_end->num);
    // Perform any actions we need to do:
    int k = 0;
    while (ACTIVE_ROUTE_SHOULD_PERFORM_ACTION(ar, resrv_dist))  { // must perform next actio due to proximity directly or bc the reverse will take a while
        bool resrv_successful = ar_perform_action(ar, ts, tc, &resrv_end, tr, cmdtid);
        // rsrv_end is updated by this call
        if (!resrv_successful) 
            break;
        ASSERT(k++ < 20, "infinite loop of execs");
    }
    tc_send(tc, TERMINAL_PRINT_RESRV1, ts->reservations.bits_low & 0xFFFFFFFF, (ts->reservations.bits_low >> 32) & 0xFFFFFFFF);
    tc_send(tc, TERMINAL_PRINT_RESRV2, ts->reservations.bits_high & 0xFFFFFFFF, (ts->reservations.bits_high >> 32) & 0xFFFFFFFF);
}

static inline void activeroute_on_sensor_event(ActiveRoute * restrict ar, Train * restrict train, TrainState * restrict ts, TerminalCourier * restrict tc, int sensor, int distance, int tr, int cmdtid){
    int dist_to_next_snsr = 0;
    if (ACTIVE_ROUTE_COMPLETE(ar) || ar->reversing)
        return;

    // update cur_pos_idx
    while (ar->last_handled_sensor != -1 && ar->last_handled_sensor != sensor){
        ar->last_handled_sensor =  next_sensor_on_route(&ar->route, &ar->cur_pos_idx, &track[SENSOR_TO_NODE(ar->last_handled_sensor)], &dist_to_next_snsr, "update cur_pos_idx")->num;
    }
    ar->last_handled_sensor = sensor;

    activeroute_recalculate_distances(ar, ts, tc, sensor, distance);

    int resrv_dist = get_resrv_dist(ar, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], train->stopping_distance[train->speed]);

    //ASSERT(resrv_dist > ar->next_step_distance, "Resrv dist too small: resrv_dist = %d, sensor = %s, idx_resrv = %d, next_step_distance = %d", resrv_dist, track[SENSOR_TO_NODE(sensor)].name, ar->idx_resrv, ar->next_step_distance);

    /*if (ACTIVE_ROUTE_SHOULD_STOP(ar, train, dist_to_next_snsr)) {
        resrv_dist = ar_stop(ar, train, tc);
    }//*/

    if (!ACTIVE_ROUTE_DONE_ACTIONS(ar)) {
        activeroute_exec_steps(ar, ts, tc, resrv_dist, tr, cmdtid);
    }
}

static inline void handle_sensor_event(TrainState * restrict ts, TerminalCourier * restrict tc, TrainStateMessage * restrict tm, int cmdtid, int tid) {
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    Reply(tid, &rm, sizeof(rm)); 
    if (unlikely(ts->total_trains <= 0))
        return;

    int sensor = tm->sensor_event.sensor, event_time = tm->sensor_event.time, distance;
    int tr = get_active_train_from_sensor(ts, sensor, &distance, 1600); // TODO REV PENALTY
    ASSERT(tr >= 0, "Could not find which train hit sensor");

    Train *train = &(ts->active_trains[tr]);
    ActiveRoute *ar = &(ts->active_routes[tr]);

    train_on_sensor_event(ts, train, tc, sensor, event_time, distance, tr);
    activeroute_on_sensor_event(ar, train, ts, tc, sensor, distance, tr, cmdtid);

    Position_HandleSensorHit(&train->pos, &track[SENSOR_TO_NODE(sensor)], event_time, ar->cur_pos_idx);

    train->last_sensor = sensor;
    train->last_sensor_time = event_time;
    ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", train->num, train->last_sensor);
}

static inline void add_new_train(TrainState * restrict ts, NewTrain data) {
    ASSERT(ts->total_trains >= 0, "Invalid total trains: %d", ts->total_trains);

    ts->active_train_map[data.train] = ts->total_trains;
    ts->active_trains[ts->total_trains].last_sensor = data.sensor;
    ts->active_trains[ts->total_trains].pos.last_known_node = &track[SENSOR_TO_NODE(data.sensor)];
    ts->active_trains[ts->total_trains].pos.state = PSTATE_STOPPED;
        ts->active_trains[ts->total_trains].pos.millis_off_last_node = 0; // TODO
    ts->active_trains[ts->total_trains].pos.last_update_time = Time();
    ts->active_trains[ts->total_trains].pos.swdir = 0;
    ts->active_trains[ts->total_trains].pos.last_route_idx = 0;
    ts->active_trains[ts->total_trains].pos.v = 0;
    //ts->active_trains[ts->total_trains].pos.a = 0;
    ts->active_trains[ts->total_trains].pos.stop_end_pos = NULL;
    ts->active_trains[ts->total_trains].pos.millis_off_stop_end = 0;

    //ts->active_trains[ts->total_trains].short_delay_coeffs[0] = 126.3030 * SHORT_COEFF_PRECISION;
    //ts->active_trains[ts->total_trains].short_delay_coeffs[0] = 5.9912 * SHORT_COEFF_PRECISION;
    //ts->active_trains[ts->total_trains].short_delay_coeffs[0] = -0.0394 * SHORT_COEFF_PRECISION;
    
    ts->active_trains[ts->total_trains].short_delay_coeffs[0] = 76.5 * SHORT_COEFF_PRECISION;
    ts->active_trains[ts->total_trains].short_delay_coeffs[1] =(int) ( -0.695 * SHORT_COEFF_PRECISION);
    ts->active_trains[ts->total_trains].short_delay_coeffs[2] = (int) (0.0019 * SHORT_COEFF_PRECISION); // these are from time -> distance; to calculate later, we invert the function
    
    if (data.train == 58) {
        ts->active_trains[ts->total_trains].stopping_distance[14] = 1150;
        ts->active_trains[ts->total_trains].stopping_distance[13] = 800;
        ts->active_trains[ts->total_trains].stopping_distance[12] = 740;
        ts->active_trains[ts->total_trains].stopping_distance[11] = 560;
        ts->active_trains[ts->total_trains].stopping_distance[10] = 390;
        
        ts->active_trains[ts->total_trains].velocity[14] = 59000;
        ts->active_trains[ts->total_trains].velocity[13] = 54000;
        ts->active_trains[ts->total_trains].velocity[12] = 47250;
        ts->active_trains[ts->total_trains].velocity[11] = 41000;
        ts->active_trains[ts->total_trains].velocity[10] = 33650;
    } else if (data.train == 78) {
        ts->active_trains[ts->total_trains].stopping_distance[14] = 950;
        ts->active_trains[ts->total_trains].stopping_distance[13] = 740;
        ts->active_trains[ts->total_trains].stopping_distance[12] = 560;
        ts->active_trains[ts->total_trains].stopping_distance[11] = 430;
        ts->active_trains[ts->total_trains].stopping_distance[10] = 340;

        ts->active_trains[ts->total_trains].velocity[14] = 59000;
        ts->active_trains[ts->total_trains].velocity[13] = 54000;
        ts->active_trains[ts->total_trains].velocity[12] = 47250;
        ts->active_trains[ts->total_trains].velocity[11] = 41000;
        ts->active_trains[ts->total_trains].velocity[10] = 33650;
    } else {
        ts->active_trains[ts->total_trains].stopping_distance[14] = 1000;
        ts->active_trains[ts->total_trains].stopping_distance[13] = 860;
        ts->active_trains[ts->total_trains].stopping_distance[12] = 600;
        ts->active_trains[ts->total_trains].stopping_distance[11] = 470;
        ts->active_trains[ts->total_trains].stopping_distance[10] = 360;

        ts->active_trains[ts->total_trains].velocity[14] = 59000;
        ts->active_trains[ts->total_trains].velocity[13] = 54000;
        ts->active_trains[ts->total_trains].velocity[12] = 47250;
        ts->active_trains[ts->total_trains].velocity[11] = 41000;
        ts->active_trains[ts->total_trains].velocity[10] = 33650;
    }


    ts->active_trains[ts->total_trains].num = data.train;
    ++ts->total_trains;

    CreateWithArgument(PRIORITY_LOW, &task_train_printer, data.train);
}

static void __attribute__((noreturn)) task_train_printer(int train){
    int trainstate_tid = WhoIs(NAME_TRAIN_STATE);
    int terminal_tid = WhoIs(NAME_TERMINAL);
    FOREVER {
        TrackPosition tp = GetTrainPosition(trainstate_tid, train);
        SendTerminalRequest(terminal_tid, TERMINAL_POS_DBG, tp.object, tp.distance_past);
        Delay(50); // 500 ms delay
    }
}

void __attribute__((noreturn)) task_train_state(int trackstate_tid) {
    RegisterAs(NAME_TRAIN_STATE);
    int cmdtid = WhoIs(NAME_COMMANDSERVER);

    circlebuffer_t cb_terminal;
    char cb_terminal_buf[TRAIN_STATE_TERMINAL_BUFFER_SIZE];
    cb_init(&cb_terminal, cb_terminal_buf, TRAIN_STATE_TERMINAL_BUFFER_SIZE);
    TerminalCourier tc = {-1, &cb_terminal};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_terminal_courier, MyTid(), (int) &ts_notify_terminal_buffer);

    TrainState ts = TRAIN_STATE_INIT;

    init_train_state(&ts);

    TrainStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    int tid;

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));

        switch (tm.request) {
        case (TRAIN_SPEED):
        {
            rm.ret = TRAIN(&ts, (int) tm.data)->speed;
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (TRAIN_POSITION):
        {
            int tr = (int) tm.data;
            Train * train = TRAIN(&ts, tr);
            ActiveRoute *ar = &ts.active_routes[ts.active_train_map[tr]];

            TrackPositionUnion tpu = {.tp = Position_CalculateNow(&train->pos, &ar->route, Time())};
            rm.ret = tpu.bytes;
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (ACTIVE_TRAIN):
        {
            rm.ret = ts.active_train_map[tm.data];
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (NAVIGATE):
        {
            handle_navigate(&ts, &tc, &tm, trackstate_tid, tid);
            break;
        }
        case (NOTIFY_SENSOR_EVENT):
        {
            handle_sensor_event(&ts, &tc, &tm, cmdtid, tid);
            break;
        }
        case (NOTIFY_TRAIN_SPEED):
        {
            Reply(tid, &rm, sizeof(rm));
            TrainData data = tm.train_data;
            TRAIN(&ts, (int) data.train)->speed = data.speed;
            break;
        }
        case (NOTIFY_TRAIN_DIRECTION):
        {
            Reply(tid, &rm, sizeof(rm));

            break;
        }
        case (NOTIFY_NEW_TRAIN):
        {
            Reply(tid, &rm, sizeof(rm));
            NewTrain data = tm.new_train;
            add_new_train(&ts, data);
            break;
        }
        case (NOTIFY_RESERVATION):
        {
            Reply(tid, &rm, sizeof(rm));
            if (tm.data < 64) {
                ts.reservations.bits_low ^= 0x1ULL << tm.data;
            } else {
                ts.reservations.bits_high ^= 0x1ULL << (tm.data - 64);
            }
            tc_send(&tc, TERMINAL_PRINT_RESRV1, ts.reservations.bits_low & 0xFFFFFFFFULL, (ts.reservations.bits_low >> 32) & 0xFFFFFFFFULL);
            tc_send(&tc, TERMINAL_PRINT_RESRV2, ts.reservations.bits_high & 0xFFFFFFFFULL, (ts.reservations.bits_high >> 32) & 0xFFFFFFFFULL);
            break;
        }
        case (NOTIFY_RV_TIMEOUT):
        {
            Reply(tid, &rm, sizeof(rm));
            /* Train is now stopped. So, we need to do several things:
             * 1. Send speed 15 command (reverse)
             * 2. Dispatch worker to send a speed command after a delay (two speed commands right after each other are bad I think)
            //*/

            int activetrain = tm.data;
            ActiveRoute *ar = &(ts.active_routes[activetrain]);
            Train *train = &(ts.active_trains[activetrain]);

            Position_HandleStop(&train->pos, ts.active_routes[activetrain].cur_pos_idx + 1, Time());
            ASSERT(train->pos.state == PSTATE_STOPPED, "cannot reverse when not yet stopped, %d | %d", train->pos.state, activetrain);
            Position_Reverse(&train->pos);

            Command c = {COMMAND_TR, 15, {.arg2 = train->num}};
            SendCommand(cmdtid, c);
            // find current node (using stopping distance past the merge we just reversed at?
            // also, find the distance we are past said current node (same deal)
            // then, find the distance from that to the next stop on the route.
            // then, use that to decide if we need to use a short move or a regular move
             
            int time = Time();
            TrackPosition tp = Position_CalculateNow(&train->pos, &ts.active_routes[activetrain].route, time);
            track_node *rvn = &track[tp.object];
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 134, TRACK_NODE_TO_INDEX(rvn));
            int idx = ar->cur_pos_idx + 1;
            int dist = activeroute_distance_to_next_stop(ar, rvn, &idx); // TODO: curpos is still the one from before the reverse - ought not it to be moved too? not sure how, given last_handled_sensor
            // I think this will break if there are two short moves in a row
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 133, dist);

            if (dist < 1000 && ALLOW_SHORTS){
                if (ar->route.rcs[idx].a != ACTION_RV)  // TODO idx sanitization
                    ar->reversing = 0;
                ar_short_move(&ts, ar, train, &tc, dist, idx, activetrain, cmdtid);
            } else {
                ar->reversing = 0;
                CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, NAV_SPEED, train->num); // TODO number
                train->speed = NAV_SPEED;
            }

            Position_HandleAccel(&train->pos, &ar->route, time, 0, calc_accel_from_vi_vf_d(0, train->velocity[train->speed], 2 * train->stopping_distance[train->speed]));
            tc_send(&tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_REVERSING, 0);
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 206, ar->route.rcs[ar->idx_resrv].swmr);
            break;
        }
        case (NOTIFY_RV_START):
        {
            Reply(tid, &rm, sizeof(rm));
            StopData sd = tm.stop_data;
            int train = sd.train, stoppos = sd.stoppos, distance = sd.distance_past;

            int activetrain = ts.active_train_map[train];
            Train *tr = TRAIN(&ts, train);
            tc_send(&tc, TERMINAL_FLAGS_SET, STATUS_FLAG_REVERSING, 0);
            Command c = {COMMAND_TR, 0, .arg2 = train};
            SendCommand(cmdtid, c);
            // figure out end position:
            int time = Time();
            ASSERT(tr->speed == NAV_SPEED, "incorrect speed");
            //PANIC("%d %s %d", stoppos, track[stoppos].name, distance);
            Position_HandleBeginStop(&tr->pos, &ts.active_routes[activetrain].route, time, 
                    &track[stoppos], distance, 
                    calc_accel_from_vi_vf_d(tr->velocity[tr->speed], 0, tr->stopping_distance[tr->speed]));

            tc_send(&tc, TERMINAL_ROUTE_DBG2, 212, train);
            CreateWith2Args(PRIORITY_LOW, &task_notify_rv_timeout, calc_reverse_time(&ts, activetrain), activetrain);
            ts.active_trains[activetrain].speed = 0;
            break;
        }
        case (NOTIFY_STOP):
        {
            Reply(tid, &rm, sizeof(rm));
            StopData sd = tm.stop_data;
            int train = sd.train, stoppos = sd.stoppos, distance = sd.distance_past;
            int activetrain = ts.active_train_map[train];
            Train *tr = TRAIN(&ts, train);
            ActiveRoute *ar = &ts.active_routes[activetrain];
            Command c = {COMMAND_TR, 0, .arg2=train};
            SendCommand(cmdtid, c);
            Position_HandleBeginStop(&tr->pos, &ar->route, Time(), 
                    &track[stoppos], distance,
                    calc_accel_from_vi_vf_d(tr->velocity[tr->speed], 0, tr->stopping_distance[tr->speed]));

            CreateWith2Args(PRIORITY_LOW, &task_notify_stopped, calc_reverse_time(&ts, activetrain), train);
            break;
        }
        case (NOTIFY_STOPPED):
        {
            Reply(tid, &rm, sizeof(rm));
            int train = tm.data;
            int activetrain = ts.active_train_map[train];
            Train *tr = TRAIN(&ts, train);
            Position_HandleStop(&tr->pos, ts.active_routes[activetrain].cur_pos_idx, Time());
            break;
        }
        case (TRAIN_STATE_NOTIFY_TERMINAL_COURIER):
        {
            tc_update_notifier(&tc, tid);
            break;
        }
        default:
        {
            PANIC("Train State Server: Unhandled request type: %d", tm.request);
        }
        }
    }
}

