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
#define ZERO_ACCEL_TOLERANCE 2500
#define TRAIN_PRINTER_DELAY 30

#define DS_TO_DA(x) (10 * (x) / 3) //((30 * (x) / 7))

#define TRAIN(ts, tr) (&((ts)->active_trains[(ts)->active_train_map[(tr)]]))
#define ACTIVE_ROUTE(ts, tr) (&((ts)->active_routes[(ts)->active_train_map[(tr)]]))

typedef enum reversing_state {
    REV_NOT_REVERSING,
    REV_BEFORE_MERGE,
    REV_AFTER_MERGE
} ReversingState;

typedef enum stopping_state {
    STOP_NOT_STOPPING,
    STOP_DELAY_ACTIVE,
    STOP_STOPPING,
    STOP_STOPPED
} StoppingState;

typedef struct active_route{
    int id;
    Route route;
    int end_node;
    int distance_past;
    int train;
    int remaining_distance;
    int next_step_distance;
    int idx_resrv;
    int cur_pos_idx;
    StoppingState stop_state;
    ReversingState rev_state;
    int last_handled_sensor;
} ActiveRoute;
#define ACTIVE_ROUTE_INIT {0, ROUTE_INIT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#define ACTIVE_ROUTE_DONE_ACTIONS(ar) ((ar)->stop_state >= STOP_DELAY_ACTIVE)
#define ACTIVE_ROUTE_COMPLETE(ar) (ACTIVE_ROUTE_DONE_ACTIONS(ar) && (ar)->stop_state == STOP_STOPPED)

#define ACTIVE_ROUTE_NEXT_STEP_RV_IN_RANGE(ar, train, d)  (((ar)->route.rcs[(ar)->cur_pos_idx].a == ACTION_RV) && ((d) + (train)->stopping_distance[(train)->speed] <= (ar)->next_step_distance) && ((ar)->rev_state == REV_NOT_REVERSING))
#define ACTIVE_ROUTE_NEXT_STEP_RV(ar)  ((ar)->route.rcs[(ar)->cur_pos_idx].a == ACTION_RV)
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
    unsigned int id: 6;
} StopData;

typedef union trainid {
    struct {
        unsigned int train: 8;
        unsigned int id: 6;
    };
    int bytes;
} TrainID;

typedef struct tsmessage{
    const MessageType type;
    const TrainStateRequest request;
    union {
        int data;
        TrainData train_data;
        CalData cal_data;
        NewTrain new_train;
        StopData stop_data;
        TrainID train_id;
        SensorEvent sensor_event;
        NavigateRequest nav_req;
    };
} TrainStateMessage;

int notify_rv_timeout(int trainstatetid, TrainID data);
int notify_rv_start(int trainstatetid, StopData data);
int notify_stop(int trainstatetid, StopData data);
int notify_stopped(int trainstatetid, TrainID data);

static inline void dump_stop_data(TerminalCourier * restrict tc, Position * restrict pos, int delay, int vi, int vf, int a, int d, int dstp, int time, int radix){
#if !(DEBUG_STOP_DISTANCE)
    return;
#endif
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 1, delay);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 2, vi);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 3, vf);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 4, a);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 5, d);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 6, dstp);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 7, pos->state);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 8, time);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 9, pos->last_update_time);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 10, pos->v);
    tc_send(tc, TERMINAL_ROUTE_DBG2, radix * 100 + 11, pos->a);
}

static inline int sendTrainState(int trainstatetid, const TrainStateMessage *msg) {
    ReplyMessage rm;
    int r = Send(trainstatetid, msg, sizeof(*msg), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

int __attribute__((pure, warn_unused_result))  calc_stop_time(Train *train){
    ASSERT(train->speed !=  15, "DIVISION BY ZERO");
    // dx = v_avg * t
    // -> t = 2 * dx / (vf + vi)
    return (2 * train->stopping_distance[train->speed] * VELOCITY_PRECISION / (train->velocity[train->speed]));
}

int __attribute__((const, warn_unused_result)) calc_reverse_time_from_velocity(int velocity, int distance_to_stop, int stopping_distance) {
    if (velocity == 0) return 0;
    return (VELOCITY_PRECISION * (distance_to_stop - stopping_distance) / velocity); 
}

int __attribute__((const, warn_unused_result)) calc_stop_delay_from_vi_vf_a_d_ds(int vi, int vf, int a, int d, int ds) {
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

    //if (d < (accel_d + ds)) return accel_t;

    int const_d = d - accel_d;
    //ASSERT(const_d >= 0, "cannot be at a constant speed for negative distance (%d, %d, %d, %d, %d, %d, %d, %d)", vi, vf, a, d, ds, accel_d, accel_t, const_d);
    // t = (delta_d / vf)
    int const_t = (const_d - ds) * VELOCITY_PRECISION / vf;
    //ASSERT(const_t >= 0, "cannot be at a constant speed for negative time (vi %d, vf %d, a %d, d %d, ds %d, ad %d, at %d, cd %d, ct %d)", vi, vf, a, d, ds, accel_d, accel_t, const_d, const_t);
    return accel_t + const_t;
}

int __attribute__((pure, warn_unused_result)) calc_short_delay(Train *train, int dist_millis) {
    int c = train->short_delay_coeffs[0], b = train->short_delay_coeffs[1], a = train->short_delay_coeffs[2];
    int det = b*b - (4 * a * (c - (dist_millis * SHORT_COEFF_PRECISION / 10)));
    int sqrt = fastintsqrt(det);
    int time = (-b + sqrt)/ (2*a);
    ASSERT(time > 0, "invalid time returned in calc_short_delay: %d,  a = %d, b = %d, c = %d, dist_millis = %d, det = %d, sqrt = %d", time, a, b, c, dist_millis, det, sqrt);
    return time;
}

int __attribute__((const, warn_unused_result)) calc_accel_from_vi_vf_d(int vi, int vf, int d) {
    // vf^2 = vi^2 + 2ad
    // a = (vi^2 - vf^2)/(2d)
    float vff = ((float) vf) / VELOCITY_PRECISION;
    float vif = ((float) vi) / VELOCITY_PRECISION;
    float vf2 = vff * vff;
    float vi2 = vif * vif;
    //ASSERT(vf2 > vf && vi2 > vi, "Overflow: %d %d -> %d %d", vf, vi, vf2, vi2);
    int deltav_prec = (int) ((vf2 - vi2) * ACCELERATION_PRECISION);
    return deltav_prec / (2 * d);
}

void __attribute__((noreturn)) task_notify_rv_timeout(int delay, int trid){
    int tid = WhoIs(NAME_TRAIN_STATE);
    TrainID ti = {.bytes = trid};
    Delay(delay);
    notify_rv_timeout(tid, ti);
    Destroy();
}

void __attribute__((noreturn)) task_delay_reaccel(int speed, int train){
    int tid = WhoIs(NAME_COMMANDSERVER);
    Delay(20);
    Command c = {COMMAND_TR, speed, .arg2=train};
    SendCommand(tid, c);
    Destroy();
}

void __attribute__((noreturn)) task_delay_stop(int data, int trid){
    int tid = WhoIs(NAME_TRAIN_STATE);
    TrainID ti = {.bytes = trid};
    DelayStop ds = {.data = data};
    Delay(ds.delay);
    StopData sd = {.train = ti.train, .stoppos = ds.stoppos, .distance_past = ds.distance_past, .id = ti.id};
    //PANIC("%d %d %d %d %d %d", train, ds.stoppos, ds.distance_past, sd.train, sd.stoppos, sd.distance_past);
    if (ds.rv != 0) {
        notify_rv_start(tid, sd);
    } else {
        notify_stop(tid, sd);
    }
    Destroy();
}

void __attribute__((noreturn)) task_notify_stopped(int delay, int trid){
    int tid = WhoIs(NAME_TRAIN_STATE);
    TrainID ti = {.bytes = trid};
    Delay(delay);
    notify_stopped(tid, ti);
    Destroy();
}

static inline void __attribute__((nonnull)) trainserver_begin_reverse(ActiveRoute * restrict ar, Train * restrict train, int delay, int rvstop, int distance_past) {
    DelayStop ds = {.delay = delay, .rv = 1, .stoppos = rvstop, .distance_past = distance_past};
    TrainID ti = {.train = train->num, .id = ar->id};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, ti.bytes);
    ar->rev_state = REV_BEFORE_MERGE;
}

int notify_rv_start(int trainstatetid, StopData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RV_START, .stop_data = data};
    return sendTrainState(trainstatetid, &msg);
}

int notify_rv_timeout(int trainstatetid, TrainID ti) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RV_TIMEOUT, .train_id = ti};
    return sendTrainState(trainstatetid, &msg);
}

int notify_stop(int trainstatetid, StopData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_STOP, .stop_data = data};
    return sendTrainState(trainstatetid, &msg);
}

int notify_stopped(int trainstatetid, TrainID ti) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_STOPPED, .train_id = ti};
    return sendTrainState(trainstatetid, &msg);
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

int NotifyRandomRoute(int trainstatetid, int train) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RANDOM_ROUTE, {.data = train}};
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

void __attribute__((nonnull)) init_train_state(TrainState *ts) {
    ts->total_trains = 0;
    Train init_train = TRAIN_INIT;
    Reservation init_reservation = RESERVATION_INIT;

    for (int i = 0; i < NUM_TRAINS; ++i) {
        ts->active_train_map[i] = -1;
    }

    for (int i = 0; i < MAX_CONCURRENT_TRAINS; i++) {
        ts->active_trains[i] = init_train;
        ts->active_routes[i].stop_state = STOP_STOPPED;
        ts->active_routes[i].end_node = -1;
        ts->active_routes[i].last_handled_sensor = -1;
    }

    ts->reservations = init_reservation;
}

static inline int __attribute__((nonnull, warn_unused_result)) get_active_train_from_sensor(TrainState *ts, const track_node *sensor_node, int *distance, int rev_penalty, Route* route_to_sensor) {
    int min_dist = MAX_SENSOR_ATTRIB_LEN + 1;
    int train = 0;
    Route min_route = ROUTE_INIT;
    for (int i = 0; i < ts->total_trains; ++i) {
        const track_node *n = &track[ts->active_trains[i].last_sensor];
        ASSERT(n >= track && n <= track + TRACK_MAX, "invalid track node for last sensor: %d %d %d", (int) n, ts->active_trains[i].num, ts->active_trains[i].last_sensor);

        Route r = ROUTE_INIT;
        int cur_dist = find_path_between_nodes(NULL, 1, min_dist, rev_penalty, n, sensor_node, &r);
        //TODO notify track_state of any switches we would have to have taken, incase a switch wasn't in the expected state
        if (cur_dist > 0 && cur_dist < min_dist) {
            min_dist = cur_dist;
            min_route = r;
            train = i;
        }
    }
    
    *distance = min_dist;
    *route_to_sensor = min_route;
    return train;
}

static int ar_stop(TerminalCourier*, ActiveRoute*, Train*) __attribute__((nonnull));
static int activeroute_distance_to_next_stop(ActiveRoute *ar, track_node *cnode, int *idx);
static void ar_short_move(TerminalCourier * restrict tc, ActiveRoute * restrict ar, MyReservation * my_reserv, Train * restrict train, int distance, int shortmoveidx, int cmdtid);
static void activeroute_exec_steps(TerminalCourier * restrict tc, ActiveRoute * restrict ar, MyReservation * my_reserv, Train *train, int resrv_dist, int cmdtid, bool can_stop);
static void task_train_printer(int);

static void ts_exec_stop(ActiveRoute * restrict ar, Train * restrict train, int stoppos, int distance, int cmdtid){
    Command c = {COMMAND_TR, 0, .arg2=train->num};
    SendCommand(cmdtid, c);
    Position_HandleBeginStop(&train->pos, &ar->route, Time(), 
            &track[stoppos], distance,
            calc_accel_from_vi_vf_d(train->velocity[train->speed], 0, train->stopping_distance[train->speed]));
    TrainID ti = {.train = train->num, .id = ar->id};
    CreateWith2Args(PRIORITY_LOW, &task_notify_stopped, calc_stop_time(train), ti.bytes);
}

static void __attribute__((nonnull)) ts_exec_step(TerminalCourier * restrict tc, ActiveRoute * restrict ar, Train * restrict train, int cmdtid, int distance_to_stop, const char * sig) {
    // do current step:
    RouteCommand rc = ar->route.rcs[ar->idx_resrv];
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
            if (ar->rev_state != REV_NOT_REVERSING) {
                ar->next_step_distance = 99999;
                return; // TODO this isn't quite right
            }
            //*/
            ASSERT(ar->rev_state == REV_NOT_REVERSING, "CANNOT EXECUTE RV WHEN ROUTE IS ALREADY REVERSING");
            //int delay = calc_reverse_time_from_velocity(train->velocity[train->speed], distance_to_stop, train->stopping_distance[train->speed]);
            int time = Time();
            int vi = Position_CalculateVelocityNow(&train->pos, time);
            int vf = train->velocity[train->speed];
            int d = distance_to_stop;
            int dstp = train->stopping_distance[train->speed];
            int a = calc_accel_from_vi_vf_d(vi, 0, dstp); // a will always be negative
            int delay = calc_stop_delay_from_vi_vf_a_d_ds(vi, vf, (vi > vf ? a : -a), d, dstp);
            dump_stop_data(tc, &train->pos, delay, vi, vf, a, d, dstp, time, 8);
            trainserver_begin_reverse(ar, train, delay, MERGE_TO_NODE_NSC(rc.swmr), 350);
            cnode = &track[MERGE_TO_NODE_NSC(rc.swmr)];
            ar->rev_state = REV_BEFORE_MERGE;
            break;
        }
        case (ACTION_NONE):
        {
            // route is done - stopping ought happen here.
            ar_stop(tc, ar, train);
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
        bool on_route;
        ar->next_step_distance += distance_to_on_route(&ar->route, ar->idx_resrv - 1, cnode, nnode, &on_route, sig);
        ASSERT(on_route, "next step must be on route when execing");
    } else if (ar->stop_state == STOP_NOT_STOPPING && ar->rev_state == REV_NOT_REVERSING) {
        const track_node *nnode = &track[ar->end_node];
        bool on_route;
        ar->next_step_distance += distance_to_on_route(&ar->route, ar->idx_resrv - 1, cnode, nnode, &on_route, sig);
        ASSERT(on_route, "next step must be on route when execing");
    } else {
        ar->next_step_distance = INT_MAX - 10000;
        tc_send(tc, TERMINAL_ROUTE_DBG2, 204, 0);
    }
}

static inline int __attribute__((nonnull, warn_unused_result)) get_resrv_dist(const ActiveRoute * restrict ar, int idx, const track_node *start, int stopping_distance) {
    ASSERT_VALID_TRACK(start);
    int cur_dist = 0, total_dist = 0;
    const track_node *resrv_end = start;
    bool on_route = TRUE;

    //nth-sensor + stopping_dist + next_switch
    resrv_end = nth_sensor_on_route(2,  &ar->route, &idx, resrv_end, &cur_dist, &on_route, "get_resrv_dist: 1");
    total_dist += cur_dist;
    if (resrv_end == NULL || !on_route) {
        return total_dist;
    }
    ASSERT_VALID_TRACK(resrv_end);

    cur_dist = stopping_distance;
    resrv_end = forward_dist_on_route(  &ar->route, &idx, resrv_end, &cur_dist, &on_route, "get_resrv_dist: 2");
    total_dist += cur_dist;
    if (resrv_end == NULL || !on_route) {
        return total_dist;
    }
    ASSERT_VALID_TRACK(resrv_end);

    cur_dist = 0;
    resrv_end = next_switch_on_route(   &ar->route, &idx, resrv_end, &cur_dist, &on_route, "get_resrv_dist: 3");
    total_dist += cur_dist;
    if (resrv_end == NULL || !on_route) {
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

static inline void __attribute__((nonnull)) handle_navigate(TerminalCourier * restrict tc, TrainState *ts, TrainStateMessage * restrict tm, int trackstate_tid, int tid, int cmdtid) {
    static int ar_id = 1;
    int object = tm->nav_req.position.object;
    int distance_past = tm->nav_req.position.distance_past;
    int tr = tm->nav_req.train;
    int a_tr = ts->active_train_map[tr];
    ASSERT(0 <= a_tr && a_tr < MAX_CONCURRENT_TRAINS, "Invalid active train: %d", ts->active_train_map[tr]);
    Train *train = TRAIN(ts, tr);
    ActiveRoute *ar_old = ACTIVE_ROUTE(ts, tr);
    ReplyMessage rm = {MESSAGE_REPLY, 0};

    ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", tr, train->last_sensor);

    int min_dist = 0, rev_penalty = 2 * train->stopping_distance[NAV_SPEED];

    int time = Time();
    TrackPosition tp = Position_CalculateNow(&train->pos, NULL, time);
    RouteRequest req = {
        .prev = tp.object, //train->last_sensor,
        .end = object,
        .min_dist = min_dist,
        .rev_penalty = rev_penalty
    };
    reservation_to_blockage(&req.blockages, &ts->reservations, ts->active_train_map[tr]);

    ActiveRoute ar_new = ACTIVE_ROUTE_INIT;
    int distance = GetRoute(trackstate_tid, req, &ar_new.route, &ar_new.cur_pos_idx);

    if (tid >= 0) {
        rm.ret = distance;
        Reply(tid, &rm, sizeof(rm));
    }

    ar_new.id = ar_id++; // each AR has a unique id.
    ar_new.end_node = ar_new.route.reverse_dest ? TRACK_NODE_TO_INDEX(track[object].reverse) : object;
    ar_new.remaining_distance = distance + distance_past;
    ar_new.distance_past = distance_past;
    ar_new.stop_state = STOP_NOT_STOPPING;
    //ar_new.last_handled_sensor = tp.object;
    ar_new.last_handled_sensor = ar_old->last_handled_sensor >= 0 ? ar_old->last_handled_sensor : tp.object;

    MyReservation my_reserv;
    reservation_to_my_reservation(&my_reserv, &(ts->reservations), ts->active_train_map[tr]);
    int resrv_dist = get_resrv_dist(&ar_new, 0, &track[object], train->stopping_distance[train->speed]);
    activeroute_exec_steps(tc, &ar_new, &my_reserv, train, resrv_dist,  cmdtid, TRUE); // TODO this will stop though, no?
    terminal_set_reservations(tc, &(ts->reservations.blkges[a_tr]), a_tr);


    track_node * cn = &track[tp.object];
    if (ar_new.route.reverse) {
        //ASSERT(train->speed == 0, "Should not be able to get a reverse without 0 speed (%d)", train->speed);
        Command c = {COMMAND_TR, 15, .arg2 = train->num};
        SendCommand(cmdtid, c);
        Position_Reverse(&train->pos);
        cn = cn->reverse;
    } 

    int idx = 0;
    int dist = activeroute_distance_to_next_stop(&ar_new, cn, &idx);
    tc_send(tc, TERMINAL_ROUTE_DBG2, 603, dist);
    if (dist < 1000 && ALLOW_SHORTS){
        ar_short_move(tc, &ar_new, &my_reserv, train, dist, idx, cmdtid);
        ar_new.stop_state = STOP_DELAY_ACTIVE;
    } else {
        CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, NAV_SPEED, train->num); // TODO number
        train->speed = NAV_SPEED;
    }
    ar_new.rev_state = REV_NOT_REVERSING;
    int a = calc_accel_from_vi_vf_d(train->velocity[train->speed], 0, DS_TO_DA(train->stopping_distance[train->speed]));
    ASSERT(a <= 0, "A ought be negative: %d %d %d", a, train->velocity[NAV_SPEED], DS_TO_DA(train->stopping_distance[train->speed]));
    Position_HandleAccel(&train->pos, &ar_new.route, Time(), 0, -1 * a, train->velocity[train->speed]);
    *ar_old = ar_new;

    for (int i = 0; i < MAX_ROUTE_COMMAND && ar_new.route.rcs[i].a != ACTION_NONE; i++){
        tc_send(tc, TERMINAL_ROUTE_DBG, ar_new.route.rcs[i].swmr, ar_new.route.rcs[i].a);
    }
    tc_send(tc, TERMINAL_ROUTE_DBG2, 301, ar_new.cur_pos_idx);
    //tc_send(&tc, TERMINAL_ROUTE_DBG2, 211, route.reverse);
    tc_send(tc, TERMINAL_ROUTE_DBG2, 21, ar_new.remaining_distance);
    tc_send(tc, TERMINAL_FLAGS_SET, STATUS_FLAG_FINDING, 0);
}

static inline void __attribute__((nonnull)) handle_random_route(TerminalCourier * restrict tc, TrainState *ts, Train * restrict train, int trackstate_tid, int cmdtid) {
    //first. pick a random node:
    int node = clk4->value_low % SWITCH_MAX;
    NavigateRequest nr = {.position = {.object = node, .distance_past = 0}, .train = train->num};
    TrainStateMessage tm = {.nav_req = nr};

    handle_navigate(tc, ts, &tm, trackstate_tid, -1, cmdtid);
}

static inline void __attribute__((nonnull)) train_on_sensor_event(TerminalCourier * restrict tc, ActiveRoute * restrict ar, Train * restrict train, int event_time, int distance) {
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
    int old_velocity = train->pos.v;

    if (train->pos.state == PSTATE_CONST_VELO) {
        train->velocity[train->speed] = MOVING_AVERAGE(new_velocity, train->velocity[train->speed], 15);
    }

    // Calculate acceleration over the last track section:
    int acceleration = (new_velocity - old_velocity) / dt;
    if (ABS(new_velocity - train->velocity[train->speed]) < ZERO_ACCEL_TOLERANCE && train->pos.state == PSTATE_ACCEL) {
        Position_HandleConstVelo(&train->pos, &ar->route, event_time, train->velocity[train->speed]);
        tc_send(tc, TERMINAL_ROUTE_DBG2, 500, ABS(acceleration));
    }

    tc_send(tc, TERMINAL_VELOCITY_DEBUG, train->velocity[train->speed], acceleration);
}

//NOTE: called both when a sensor is hit, and when we reach the end of a route
//SORT OF SKETCHY because the only reason we can do this is that ACTIVE_ROUTE_NEXT_STEP_RV() returns false at the end of a route
static inline void __attribute__((nonnull)) free_track_behind_train(ActiveRoute * restrict ar, MyReservation * restrict my_reserv, Train * restrict train, const track_node *free_end, Blockage * restrict result, const char *sig) {
    const track_node *free_start = &track[SENSOR_TO_NODE(train->last_sensor)];
    //ASSERT(free_start != free_end, "hit same sensor twice?! free_start = %s, free_end = %s, train = %d", free_start->name, free_end->name, train->num);

    if (ACTIVE_ROUTE_NEXT_STEP_RV(ar) && ar->rev_state == REV_BEFORE_MERGE) {
        //TODO handle off route problems
        bool on_route;

        //REMINDER: as per above comment, THIS WILL BREAK if it runs at the end of a route, since there is no next merge/switch
        const track_node *merge = rc_to_track_node(ar->route.rcs[ar->cur_pos_idx], sig);
        int dist_to_merge = distance_to_on_route(&ar->route, ar->cur_pos_idx, free_start, merge, &on_route, sig);
        int dist_to_snsr = get_dist_to_nxt_sensor(&ar->route, ar->cur_pos_idx, free_start, &on_route, sig);
        if (dist_to_merge < dist_to_snsr) { // TODO second part of && is a big hack
            free_end = merge;
            ar->rev_state = REV_AFTER_MERGE;
        }
    }

    //TODO: free when we're past the sensor, not when we hit it
    //OR, reserve end node, and the node after
    free_track(&ar->route, ar->cur_pos_idx, free_start, free_end, my_reserv, result, sig);
}

static inline bool __attribute__((nonnull)) update_cur_pos_idx(ActiveRoute * restrict ar, int sensor) {
    int dummy;
    bool on_route = TRUE;
    const track_node *cur_node = &track[SENSOR_TO_NODE(ar->last_handled_sensor)];
    while (cur_node != NULL && cur_node->num != sensor && on_route){
        cur_node = next_sensor_on_route(&ar->route, &ar->cur_pos_idx, cur_node, &on_route, &dummy, "update cur_pos_idx");
    }
    ar->last_handled_sensor = sensor;
    return on_route;
}

static inline void __attribute__((nonnull)) activeroute_recalculate_distances(TerminalCourier * restrict tc, ActiveRoute * restrict ar, int sensor, int distance) {
    ASSERT(distance != 0, "distance between last sensor pair shouldn't be 0");
    // recalculate next step distance and remaining distance every time
    if (ACTIVE_ROUTE_DONE_ACTIONS(ar)) {
        ar->next_step_distance = 0;
    } else {
        ASSERT(ar->idx_resrv >= ar->cur_pos_idx, "should have reserved ahead of cur_pos_idx, idx_resrv = %d, cur_pos_idx = %d, sensor = %d", ar->idx_resrv, ar->cur_pos_idx, sensor);
        bool on_route = TRUE;
        if (ar->route.rcs[ar->idx_resrv].a != ACTION_NONE)
            ar->next_step_distance = distance_to_on_route(&ar->route, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], rc_to_track_node(ar->route.rcs[ar->idx_resrv], "ar next step recalculate rc2tn"), &on_route, "ar next step recalculate");
        else 
            ar->next_step_distance = distance_to_on_route(&ar->route, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], &track[ar->end_node], &on_route, "ar next step recalculate");

        ASSERT(on_route, "recalculate distance cannot be off route: %d (%s -> %s@ %d)", on_route, track[SENSOR_TO_NODE(sensor)].name, 
                (ar->route.rcs[ar->idx_resrv].a == ACTION_NONE ? 
                    track[ar->end_node].name : 
                    track[SWITCH_TO_NODE_NSC(ar->route.rcs[ar->idx_resrv].swmr)].name), ar->idx_resrv);
    }
    bool on_route = TRUE;
    ar->remaining_distance = distance_to_on_route(&ar->route, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], &track[ar->end_node], &on_route, "ar distance recalculate") + ar->distance_past;
    ASSERT(on_route, "should be on_route to find end");
    tc_send(tc, TERMINAL_ROUTE_DBG2, 207, ar->remaining_distance);
}


static int __attribute__((nonnull, warn_unused_result)) activeroute_distance_to_next_stop(ActiveRoute *ar, track_node *cnode, int *idx) {
    const track_node *n = cnode;
    int distance = 0;
    const track_edge *e;
    bool on_route = TRUE;
    while (ar->route.rcs[*idx].a != ACTION_NONE && ar->route.rcs[*idx].a != ACTION_RV) {
        e = next_edge_on_route(&(ar->route), idx, n, &on_route, "Distance to next stop");
        ASSERT(e != NULL && on_route, "Should not be able to get null edge without ACTION_NONE or ACTION_RV");
        distance += e->dist;
        n = e->dest;
    }
    if (ar->route.rcs[*idx].a == ACTION_RV) {
        distance += distance_to_on_route(&ar->route, *idx, n, &track[SWITCH_TO_NODE_NSC(ar->route.rcs[*idx].swmr)], &on_route, "distance to next stop (rv)")  + 350; // TODO 
    } else {
        distance += distance_to_on_route(&ar->route, *idx, n, &track[ar->end_node], &on_route, "distance to next stop");
    }
    return distance;
}

static inline int __attribute__((nonnull)) ar_stop(TerminalCourier * restrict tc, ActiveRoute * restrict ar, Train * restrict train) {
    // Figure out how long we need to wait:
    //static int k = 0;
    //int stopdelay = calc_reverse_time_from_velocity(train->velocity[train->speed], ar->remaining_distance, train->stopping_distance[train->speed]);
    //if (stopdelay < 0) stopdelay = 0;
    int time = Time();
    int vi = Position_CalculateVelocityNow(&train->pos, time);
    int vf = train->velocity[train->speed];
    ASSERT(vi <= vf, "Vi cannot be less than vf: %d, %d", vi, vf);
    int d = ar->remaining_distance;
    int dstp = train->stopping_distance[train->speed];
    int a = -calc_accel_from_vi_vf_d(vf, 0, DS_TO_DA(dstp)); // a will always be negative
    int stopdelay = calc_stop_delay_from_vi_vf_a_d_ds(vi, vf, a, d, dstp);
    dump_stop_data(tc, &train->pos, stopdelay, vi, vf, a, d, dstp, time, 7);
    ASSERT(stopdelay >= 0, "cannot delay negative time (vi %d vf %d a %d d %d ds %d delay %d", vi, vf, a, d, dstp, stopdelay);
    ////ASSERT(train->pos.state != PSTATE_CONST_VELO, "state shouldn't be const velo (vi %d vf %d a %d d %d ds %d delay %d", vi, vf, a, d, dstp, stopdelay);
    DelayStop ds = {.delay = stopdelay, .rv = 0, .stoppos = ar->end_node, .distance_past = ar->distance_past};
    TrainID ti = {.train = train->num, .id = ar->id};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, ti.bytes);
    ar->stop_state = STOP_DELAY_ACTIVE;
    tc_send(tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
    //ASSERT(FALSE, "%d %d %d %d %d %d %d", ar->remaining_distance, train->stopping_distance[train->speed], train->velocity[train->speed], ds.delay, ds.rv, ds.stoppos, ds.distance_past);
    return 999999;
}

static inline bool __attribute__((nonnull)) ar_perform_action(TerminalCourier * restrict tc, ActiveRoute *restrict ar, MyReservation * restrict my_reserv, Train * restrict train, const track_node ** resrv_start, int cmdtid) {
    const track_node *resrv_end;
    bool resrv_successful = TRUE;

    if (ar->idx_resrv+1 < MAX_ROUTE_COMMAND && ar->route.rcs[ar->idx_resrv+1].a != ACTION_NONE) {
        resrv_end = rc_to_track_node(ar->route.rcs[ar->idx_resrv+1], "resrv_end");
    }
    else {
        resrv_end = &track[ar->end_node];
    }
    
    if (RESERVE_TRACK) {
        resrv_successful = reserve_track(&ar->route, ar->idx_resrv, *resrv_start, resrv_end, my_reserv, "main reserve_track");
        if (!resrv_successful) {
            return FALSE;
        }
        *resrv_start = resrv_end;
    }

    int nxt_step_rv_in_range = ACTIVE_ROUTE_NEXT_STEP_RV(ar) && (ar->rev_state == REV_NOT_REVERSING);
    ts_exec_step(tc, ar, train, cmdtid, ar->next_step_distance + (nxt_step_rv_in_range ? 350 : 0), "action loop");
    return resrv_successful;
}

static void ar_short_move(TerminalCourier * restrict tc, ActiveRoute * restrict ar, MyReservation * restrict my_reserv, Train * restrict train, int distance, int shortmoveidx, int cmdtid) {
    // basic idea: do all of the steps between now and the end of the short move.
    // then, start the train moving at 14
    // start the server to delay stopping the train
    // if the short move is a reverse, trigger the reverse start after the delay as well.
    
    if (ar->route.rcs[ar->idx_resrv].a != ACTION_NONE) { // no more reservations exist
        const track_node *resrv_end =  rc_to_track_node(ar->route.rcs[ar->idx_resrv], "resrv_start short move");
        while( ar->idx_resrv < shortmoveidx) {
            bool resrv_successful = ar_perform_action(tc, ar, my_reserv, train, &resrv_end, cmdtid);
            // resrv_end is updated by this call
            if (!resrv_successful) 
                break;
        }
    }

    ar->idx_resrv++; // skip the idx of the short move
    CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, 14, train->num);
    train->speed = 14;
    int shortdelay = calc_short_delay(train, distance);
    tc_send(tc, TERMINAL_ROUTE_DBG2, 601, shortdelay);
    tc_send(tc, TERMINAL_ROUTE_DBG2, 602, distance);

    DelayStop ds = {.delay = shortdelay, .rv = 0, .stoppos = 0, .distance_past = 350}; // TODO
    if (ar->route.rcs[shortmoveidx].a == ACTION_RV) {
        ar->rev_state = REV_BEFORE_MERGE;
        ds.rv = 1;
        ds.stoppos = SWITCH_TO_NODE_NSC(ar->route.rcs[ar->idx_resrv].swmr);
    } else {
        tc_send(tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
        ds.stoppos = ar->end_node;
        ds.distance_past = ar->distance_past;
        ar->stop_state = STOP_DELAY_ACTIVE;
    }
    TrainID ti = {.train = train->num, .id = ar->id};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, ti.bytes);
}

static void activeroute_exec_steps(TerminalCourier * restrict tc, ActiveRoute * restrict ar, MyReservation * restrict my_reserv, Train * restrict train, int resrv_dist, int cmdtid, bool in_startup) {
    const track_node *resrv_start;
    if (in_startup) {
        resrv_start = &track[SENSOR_TO_NODE(train->last_sensor)];
    } else if (unlikely(ar->route.rcs[ar->idx_resrv].a == ACTION_NONE)) {
        //if our next step is the stop at the end of the route, don't need to reserve anything
        //this works because reserving between ar->end_node and ar->end_node won't reserve anything
        resrv_start = &track[ar->end_node];
    } else {
        resrv_start = rc_to_track_node(ar->route.rcs[ar->idx_resrv], "resrv_start");
    }

    tc_send(tc, TERMINAL_ROUTE_DBG2, 260, resrv_start->num);
    // Perform any actions we need to do:
    while (ACTIVE_ROUTE_SHOULD_PERFORM_ACTION(ar, resrv_dist)){  // must perform next actio due to proximity directly or bc the reverse will take a while
        ASSERT_VALID_TRACK(resrv_start);
        //can't send stop command before we've even started accelerating
        if (((ar->idx_resrv >= MAX_ROUTE_COMMAND || ar->route.rcs[ar->idx_resrv].a == ACTION_NONE || ar->route.rcs[ar->idx_resrv].a == ACTION_RV)) && in_startup)
            break;
        // resrv_start is updated by this call
        bool resrv_successful = ar_perform_action(tc, ar, my_reserv, train, &resrv_start, cmdtid);
        if (!resrv_successful) {
            break;
        }
    }
}

static bool activeroute_off_route(UNUSED TerminalCourier * restrict tc, TrainState *ts, ActiveRoute * restrict ar, MyReservation* restrict my_reserv, Train * restrict train, UNUSED int sensor, UNUSED int distance, UNUSED int cmdtid, int trackstate_tid){
    if (ar->rev_state == REV_AFTER_MERGE) return FALSE; // We're off route, but we don't mind

    tc_send(tc, TERMINAL_ROUTE_DBG2, 999, 999);

    free_all_reservations(my_reserv); // drop all current reservations (we aren't even on them anymore)

    // restart navigation.
    NavigateRequest nr  = {.position = {.object = ar->end_node, .distance_past = ar->distance_past}, .train = train->num};
    TrainStateMessage tm = {.nav_req = nr};
    handle_navigate(tc, ts, &tm, trackstate_tid, -1, cmdtid);
    
    /*
    FdistReq frq = {SENSOR_TO_NODE(sensor), 0};
    TrackPosition fdist = GetFdist(trackstate_tid, frq);
    ASSERT(0 <= fdist.object && fdist.object <= TRACK_MAX, "invalid object during off-route emergency stop: %d", fdist.object);

    if (ar->stop_state < STOP_STOPPING) {
        // a valid case is hitting a sensor after becoming off route - should not stop twice.
        ts_exec_stop(ar, train, fdist.object, fdist.distance_past, cmdtid);
        ar->stop_state = STOP_STOPPING;
    }
    */

    //ASSERT(FALSE, "Off route, but not currently reversing");
    return FALSE;
}

static inline void activeroute_on_sensor_event(TerminalCourier * restrict tc, ActiveRoute * restrict ar, MyReservation * restrict my_reserv, Train * restrict train, int sensor, int distance, int cmdtid){
    tc_send(tc, TERMINAL_ROUTE_DBG2, 279, ar->stop_state);
    if (ACTIVE_ROUTE_COMPLETE(ar))
        return;

    // update cur_pos_idx
    bool on_route = update_cur_pos_idx(ar, sensor);
    ASSERT(on_route, "should not be off route when updating cur_pos_idx");
    ar->last_handled_sensor = sensor;

    tc_send(tc, TERMINAL_ROUTE_DBG2, 271, ar->rev_state);
    if (ar->rev_state != REV_NOT_REVERSING)
        return;

    activeroute_recalculate_distances(tc, ar, sensor, distance);

    int resrv_dist = get_resrv_dist(ar, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], train->stopping_distance[train->speed]);


    /*
    ASSERT(resrv_dist > ar->next_step_distance, "Resrv dist too small: resrv_dist = %d, sensor = %s, idx_resrv = %d, next_step_distance = %d", resrv_dist, track[SENSOR_TO_NODE(sensor)].name, ar->idx_resrv, ar->next_step_distance);
#define ACTIVE_ROUTE_SHOULD_STOP(ar, train, dist_to_next_snsr) ((dist_to_next_snsr + (train)->stopping_distance[(train)->speed] >= (ar)->remaining_distance && !((ar)->stop_state == STOP_DELAY_ACTIVE)))
      if (ACTIVE_ROUTE_SHOULD_STOP(ar, train, dist_to_next_snsr)) {
        resrv_dist = ar_stop(tc, ar, train);
    }//*/

    tc_send(tc, TERMINAL_ROUTE_DBG2, 272, ACTIVE_ROUTE_DONE_ACTIONS(ar));
    if (!ACTIVE_ROUTE_DONE_ACTIONS(ar)) {
        activeroute_exec_steps(tc, ar, my_reserv, train, resrv_dist, cmdtid, FALSE);
    }
}

static inline void __attribute__((nonnull)) handle_sensor_event(TerminalCourier * restrict tc, TrainState *ts, TrainStateMessage * restrict tm, int cmdtid, int tid, int trackstate_tid) {
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    Reply(tid, &rm, sizeof(rm)); 
    if (unlikely(ts->total_trains <= 0))
        return;

    int sensor = tm->sensor_event.sensor, event_time = tm->sensor_event.time, distance;
    const track_node *sensor_node = &track[SENSOR_TO_NODE(sensor)];
    Route route_to_sensor = ROUTE_INIT;
    int tr = get_active_train_from_sensor(ts, sensor_node, &distance, 1600, &route_to_sensor); // TODO REV PENALTY
    ASSERT(tr >= 0, "Could not find which train hit sensor");

    Train *train = &(ts->active_trains[tr]);
    ActiveRoute *ar = &(ts->active_routes[tr]);
    MyReservation my_reserv;
    reservation_to_my_reservation(&my_reserv, &(ts->reservations), tr);

    train_on_sensor_event(tc, ar, train, event_time, distance);

    // now, we need to determine if we're actually on the right route
    bool continue_actions = TRUE;
    bool on_route = compare_route(&ar->route, ar->cur_pos_idx, &route_to_sensor);
    if (!on_route) {
        Position_HandleSensorHit(&train->pos, sensor_node, event_time, ar->cur_pos_idx); // TODO hack
        continue_actions = activeroute_off_route(tc, ts, ar, &my_reserv, train, sensor, distance, cmdtid, trackstate_tid);
    }

    if (continue_actions) {
        //we've been given a nav command at some point
        //TODO handle problem when we give a route command an there is a switch between the last sensor and the next
        if (RESERVE_TRACK && likely(ar->end_node >= 0) && ar->rev_state != REV_AFTER_MERGE) {
            Blockage freed;
            free_track_behind_train(ar, &my_reserv, train, sensor_node, &freed, "sensor_event free_track");
            terminal_unset_reservations(tc, &freed);
        }

        //    update_cur_pos_idx(ar, sensor);
        activeroute_on_sensor_event(tc, ar, &my_reserv, train, sensor, distance, cmdtid);
    }

    Position_HandleSensorHit(&train->pos, sensor_node, event_time, ar->cur_pos_idx);

    train->last_sensor = sensor;
    train->last_sensor_time = event_time;
    ASSERT(0 <= train->last_sensor && train->last_sensor <= TRACK_MAX, "invalid last sensor for train %d: %d", train->num, train->last_sensor);

    terminal_set_reservations(tc, &(ts->reservations.blkges[tr]), tr);
}

static inline void __attribute__((nonnull)) add_new_train(TrainState *ts, NewTrain data) {
    ASSERT(ts->total_trains >= 0, "Invalid total trains: %d", ts->total_trains);

    ts->active_train_map[data.train] = ts->total_trains;
    Train *train = &(ts->active_trains[ts->total_trains]);
    train->last_sensor = data.sensor;
    train->pos.last_known_node = &track[SENSOR_TO_NODE(data.sensor)];
    train->pos.state = PSTATE_STOPPED;
        train->pos.millis_off_last_node = 0; // TODO
    train->pos.last_update_time = Time();
    train->pos.swdir = 0;
    train->pos.last_route_idx = 0;
    train->pos.v = 0;
    train->pos.a = 0;
    train->pos.stop_end_pos = NULL;
    train->pos.millis_off_stop_end = 0;

    //train->short_delay_coeffs[0] = 126.3030 * SHORT_COEFF_PRECISION;
    //train->short_delay_coeffs[0] = 5.9912 * SHORT_COEFF_PRECISION;
    //train->short_delay_coeffs[0] = -0.0394 * SHORT_COEFF_PRECISION;
    
    train->short_delay_coeffs[0] = 76.5 * SHORT_COEFF_PRECISION;
    train->short_delay_coeffs[1] =(int) ( -0.695 * SHORT_COEFF_PRECISION);
    train->short_delay_coeffs[2] = (int) (0.0019 * SHORT_COEFF_PRECISION); // these are from time -> distance; to calculate later, we invert the function
    
    if (data.train == 58) {
        train->stopping_distance[14] = 1150;
        train->stopping_distance[13] = 800;
        train->stopping_distance[12] = 740;
        train->stopping_distance[11] = 560;
        train->stopping_distance[10] = 390;
        
        train->velocity[14] = 59000;
        train->velocity[13] = 54000;
        train->velocity[12] = 47250;
        train->velocity[11] = 41000;
        train->velocity[10] = 33650;
    } else if (data.train == 78) {
        train->stopping_distance[14] = 950;
        train->stopping_distance[13] = 740;
        train->stopping_distance[12] = 560;
        train->stopping_distance[11] = 430;
        train->stopping_distance[10] = 340;

        train->velocity[14] = 59000;
        train->velocity[13] = 54000;
        train->velocity[12] = 47250;
        train->velocity[11] = 41000;
        train->velocity[10] = 33650;
    } else {
        train->stopping_distance[14] = 1000;
        train->stopping_distance[13] = 860;
        train->stopping_distance[12] = 600;
        train->stopping_distance[11] = 470;
        train->stopping_distance[10] = 360;

        train->velocity[14] = 59000;
        train->velocity[13] = 54000;
        train->velocity[12] = 47250;
        train->velocity[11] = 41000;
        train->velocity[10] = 33650;
    }


    train->num = data.train;
    ++ts->total_trains;

    CreateWithArgument(PRIORITY_LOW, &task_train_printer, data.train);
}

static void __attribute__((noreturn)) task_train_printer(int train){
    int trainstate_tid = WhoIs(NAME_TRAIN_STATE);
    int terminal_tid = WhoIs(NAME_TERMINAL);
    FOREVER {
        TrackPosition tp = GetTrainPosition(trainstate_tid, train);
        SendTerminalRequest(terminal_tid, TERMINAL_POS_DBG, tp.object, tp.distance_past);
        Delay(TRAIN_PRINTER_DELAY); // 500 ms delay
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
            ActiveRoute *ar = ACTIVE_ROUTE(&ts, tr);

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
            handle_navigate(&tc, &ts, &tm, trackstate_tid, tid, cmdtid);
            break;
        }
        case (NOTIFY_SENSOR_EVENT):
        {
            handle_sensor_event(&tc, &ts, &tm, cmdtid, tid, trackstate_tid);
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
            int object = tm.data;
            //user's reservations show up as the reservations of another train
            //MAX_CONCURRENT_TRAINS is an ID that won't be used
            int active_train = MAX_CONCURRENT_TRAINS;
            MyReservation my_reserv;
            reservation_to_my_reservation(&my_reserv, &(ts.reservations), active_train);

            bool success = reserve_track(NULL, 0, &track[object], &track[object], &my_reserv, "drop track");
            if (likely(success)) {
                terminal_set_reservations(&tc, my_reserv.mine, MAX_CONCURRENT_TRAINS);
            } else {
                Blockage freed;
                const track_node *next = track[object].edge[DIR_AHEAD].dest;
                free_track(NULL, 0, &track[object], next, &my_reserv, &freed, "drop command free_track");
                terminal_unset_reservations(&tc, &freed);
            }
            break;
        }
        case (NOTIFY_RV_TIMEOUT):
        {
            Reply(tid, &rm, sizeof(rm));
            /* Train is now stopped. So, we need to do several things:
             * 1. Send speed 15 command (reverse)
             * 2. Dispatch worker to send a speed command after a delay (two speed commands right after each other are bad I think)
            //*/

            TrainID ti = tm.train_id;
            int tr = ti.train, id = ti.id;
            ActiveRoute *ar = ACTIVE_ROUTE(&ts, tr);
            Train *train = TRAIN(&ts, tr);
            ASSERT(ar != NULL && train != NULL, "invalid train");
            if (ar->id != id){ // outdated route
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 500, ar->id);
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 501, id);
                break;
            }

            Position_HandleStop(&train->pos, ar->cur_pos_idx + 1, Time());
            ASSERT(train->pos.state == PSTATE_STOPPED, "cannot reverse when not yet stopped, %d | %d", train->pos.state, train->num);
            Position_Reverse(&train->pos);

            Command c = {COMMAND_TR, 15, {.arg2 = train->num}};
            SendCommand(cmdtid, c);
            // find current node (using stopping distance past the merge we just reversed at?
            // also, find the distance we are past said current node (same deal)
            // then, find the distance from that to the next stop on the route.
            // then, use that to decide if we need to use a short move or a regular move
             
            int time = Time();
            TrackPosition tp = Position_CalculateNow(&train->pos, &ar->route, time);
            ASSERT(tp.object >= 0 && tp.object <= TRACK_MAX, "bad object %d", tp.object);
            track_node *rvn = &track[tp.object];
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 134, TRACK_NODE_TO_INDEX(rvn));
            int idx = ar->cur_pos_idx + 1;
            int dist = activeroute_distance_to_next_stop(ar, rvn, &idx);
            // I think this will break if there are two short moves in a row
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 133, dist);

            if (dist < 1000 && ALLOW_SHORTS){
                if (ar->route.rcs[idx].a != ACTION_RV) {  // TODO idx sanitization
                    ar->rev_state = REV_NOT_REVERSING;
                }
                MyReservation my_reserv;
                reservation_to_my_reservation(&my_reserv, &(ts.reservations), ts.active_train_map[tr]);
                ar_short_move(&tc, ar, &my_reserv, train, dist, idx, cmdtid);
            } else {
                ar->rev_state = REV_NOT_REVERSING;
                CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, NAV_SPEED, train->num); // TODO number
                train->speed = NAV_SPEED;
            }

            Position_HandleAccel(&train->pos, &ar->route, time, 0, calc_accel_from_vi_vf_d(0, train->velocity[12], DS_TO_DA(train->stopping_distance[12])), train->velocity[train->speed]);
            tc_send(&tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_REVERSING, 0);
            //ASSERT(ar->route.rcs[ar->idx_resrv].swmr => 0 && ar->route.rcs[ar->idx_resrv].swmr <= 22, "invalid route swmr: %d %d %d", ar->idx_resrv, ar->route.rcs[ar->idx_resrv].swmr, ar->route.rcs[ar->idx_resrv].a);
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 206, ar->route.rcs[ar->idx_resrv].swmr);
            break;
        }
        case (NOTIFY_RV_START):
        {
            // rv delay has started
            Reply(tid, &rm, sizeof(rm));
            StopData sd = tm.stop_data;
            int tr = sd.train, stoppos = sd.stoppos, distance = sd.distance_past, id = sd.id;

            Train *train = TRAIN(&ts, tr);
            ActiveRoute *ar = ACTIVE_ROUTE(&ts, tr);
            if (ar->id != id) { // outdated ar
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 500, ar->id);
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 501, id);
                break;
            }

            tc_send(&tc, TERMINAL_ROUTE_DBG2, 278, ar->stop_state);
            if (ar->stop_state >= STOP_STOPPING) // it is entirely possible that a train could emergency stop while they have a regular stop delay running, in which case it shouldn't try to re-stop
                break;

            tc_send(&tc, TERMINAL_FLAGS_SET, STATUS_FLAG_REVERSING, 0);
            ASSERT(train != NULL, "NULL train: %d", tr);
            Command c = {COMMAND_TR, 0, .arg2 = tr};
            SendCommand(cmdtid, c);
            // figure out end position:
            int time = Time();
            //ASSERT(train->speed == NAV_SPEED, "incorrect speed");
            //PANIC("%d %s %d", stoppos, track[stoppos].name, distance);
            Position_HandleBeginStop(&train->pos, &ar->route, time, 
                    &track[stoppos], distance, 
                    calc_accel_from_vi_vf_d(train->velocity[train->speed], 0, train->stopping_distance[train->speed]));


            TrainID ti = {.train = tr, .id = ar->id};
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 212, ti.train);
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 213, ti.id);
            CreateWith2Args(PRIORITY_LOW, &task_notify_rv_timeout, calc_stop_time(train), ti.bytes);
            //train->speed = 0;
            break;
        }
        case (NOTIFY_STOP):
        {
            Reply(tid, &rm, sizeof(rm));
            StopData sd = tm.stop_data;
            int tr = sd.train, stoppos = sd.stoppos, distance = sd.distance_past, id = sd.id;
            Train *train = TRAIN(&ts, tr);
            ActiveRoute *ar = ACTIVE_ROUTE(&ts, tr);
            if (ar->id != id) {
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 500, ar->id);
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 501, id);
                break;
            }
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 277, ar->stop_state);
            if (ar->stop_state < STOP_STOPPING){ // it is entirely possible that a train could emergency stop while they have a regular stop delay running, in which case it shouldn't try to re-stop
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 274, ar->stop_state);
                ar->stop_state = STOP_STOPPING;
                ts_exec_stop(ar, train, stoppos, distance, cmdtid);
            }
            break;
        }
        case (NOTIFY_STOPPED):
        {
            Reply(tid, &rm, sizeof(rm));
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 272, 0);
            TrainID ti = tm.train_id;
            int tr = ti.train, id = ti.id;
            Train *train = TRAIN(&ts, tr);
            ActiveRoute *ar = ACTIVE_ROUTE(&ts, tr);
            if (ar->id != id) {
                break;
            }

            MyReservation my_reserv;
            reservation_to_my_reservation(&my_reserv, &(ts.reservations), ts.active_train_map[tr]);

            Blockage freed;
            free_track_behind_train(ar, &my_reserv, train, train->pos.stop_end_pos, &freed, "stopped free_track");
            terminal_unset_reservations(&tc, &freed);

            Position_HandleStop(&train->pos, ar->cur_pos_idx, Time());
            train->speed = 0;
            ar->stop_state = STOP_STOPPED;

            if (train->random_routing) {
                handle_random_route(&tc, &ts, train, trackstate_tid, cmdtid);
            }
            break;
        }
        case (NOTIFY_RANDOM_ROUTE):
        {
            Reply(tid, &rm, sizeof(rm));
            int tr = tm.data;
            Train * train = TRAIN(&ts, tr);
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 0, tr);
            train->random_routing = TRUE;

            handle_random_route(&tc, &ts, train, trackstate_tid, cmdtid);
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

