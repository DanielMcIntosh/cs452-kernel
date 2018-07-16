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

#define NAV_SPEED 12

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

#define ACTIVE_ROUTE_DONE_ACTIONS(ar) (ar->route.rcs[ar->idx_resrv].a == ACTION_NONE)
#define ACTIVE_ROUTE_COMPLETE(ar) ((ACTIVE_ROUTE_DONE_ACTIONS(ar) && ar->stopped))
#define ACTIVE_ROUTE_INIT {ROUTE_INIT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define ACTIVE_ROUTE_SHOULD_STOP(ar, train, dist_to_next_snsr) ((dist_to_next_snsr + train->stopping_distance[train->speed] >= (ar)->remaining_distance && !(ar)->stopped))
#define ACTIVE_ROUTE_NEXT_STEP_RV_IN_RANGE(ar, train, d)  ((ar->route.rcs[ar->cur_pos_idx].a == ACTION_RV) && (d + train->stopping_distance[train->speed] <= ar->next_step_distance) && (!ar->reversing))
#define ACTIVE_ROUTE_NEXT_STEP_RV(ar)  ((ar->route.rcs[ar->cur_pos_idx].a == ACTION_RV) && (!ar->reversing))
#define ACTIVE_ROUTE_SHOULD_PERFORM_ACTION(ar, resrv_dist, nxt_step_rv_in_range) ((ar->next_step_distance <= resrv_dist || nxt_step_rv_in_range) && !ACTIVE_ROUTE_DONE_ACTIONS(ar))


typedef struct train_state{
    int total_trains;
    int active_train_map[NUM_TRAINS];
    Train active_trains[MAX_CONCURRENT_TRAINS];
    Reservation reservations;
    ActiveRoute active_routes[MAX_CONCURRENT_TRAINS];
} TrainState;

#define TRAIN_STATE_INIT {0, {0}, {TRAIN_INIT}, RESERVATION_INIT, {ACTIVE_ROUTE_INIT}}

typedef struct tsmessage{
    const MessageType type;
    const TrainStateRequest request;
    union {
        int data;
        TrainData train_data;
        CalData cal_data;
        NewTrain new_train;
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

int __attribute__((pure)) calc_reverse_time_from_velocity(int velocity, int distance_to_stop, int stopping_distance) {
    if (velocity == 0) return 0;
    return (VELOCITY_PRECISION * (distance_to_stop - stopping_distance) / velocity); 
}

int notify_rv_timeout(int trainstatetid, int activetrain) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RV_TIMEOUT, .data = activetrain};
    return sendTrainState(trainstatetid, &msg);
}

int notify_rv_start(int trainstatetid, int train) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_RV_START, .data = train};
    return sendTrainState(trainstatetid, &msg);
}

void task_notify_rv_timeout(int delay, int activetrain){
    int tid = WhoIs(NAME_TRAIN_STATE);
    Delay(delay);
    notify_rv_timeout(tid, activetrain);
    Destroy();
}

void task_delay_reaccel(int speed, int train){
    int tid = WhoIs(NAME_COMMANDSERVER);
    Delay(20);
    Command c = {COMMAND_TR, speed, .arg2=train};
    SendCommand(tid, c);
    Destroy();
}

typedef union delaystop{
    int data;
    struct {
        int delay: 16;
        int rv: 2;
    };

} DelayStop;

void task_delay_stop(int data, int train){
    int tid = WhoIs(NAME_TRAIN_STATE);
    int cmdtid = WhoIs(NAME_COMMANDSERVER);
    DelayStop ds = {.data = data};
    Delay(ds.delay);
    Command c = {COMMAND_TR, 0, .arg2=train};
    SendCommand(cmdtid, c);
    if (ds.rv) {
        notify_rv_start(tid, train);
    }
    Destroy();
}

static inline void trainserver_begin_reverse(TrainState *ts, int activetrain, int delay) {
    DelayStop ds = {.delay = delay, .rv = 1};
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

void init_train_state(TrainState *ts) {
    ts->total_trains = 0;
    Train init_train = {0, 0, FORWARD, {0}, {0}, -1, 0, -1, 0, 0};
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
        int distance = find_path_between_nodes(&resrv, 1, rev_penalty, n, d, &r);
        //TODO notify track_state of any switches we would have to have taken, incase a switch wasn't in the expected state
        if (distance < min_dist) {
            min_dist = distance;
            train = i;
        }
    }
    
    *distance = min_dist;
    return train;
}

static int ar_stop(ActiveRoute*, Train*, TerminalCourier*);

static void ts_exec_step(TrainState * restrict ts, TerminalCourier * restrict tc, ActiveRoute * restrict ar, int activetrain, int cmdtid, int distance_to_stop, char * sig) {
    // do current step:
    RouteCommand rc = ar->route.rcs[ar->idx_resrv];
    Train *train = &(ts->active_trains[activetrain]);
    const track_node *cnode = rc_to_track_node(rc, sig);
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
            Train *train = &(ts->active_trains[activetrain]);
            int delay = calc_reverse_time_from_velocity(train->velocity[train->speed], distance_to_stop, train->stopping_distance[train->speed]);
            trainserver_begin_reverse(ts, activetrain, delay);
            cnode = &track[MERGE_TO_NODE(rc.swmr)];
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
        ar->next_step_distance += distance_to_on_route(&ar->route, ar->idx_resrv - 1, cnode, nnode, sig);
    } else {
        ar->next_step_distance = INT_MAX - 10000;
        tc_send(tc, TERMINAL_ROUTE_DBG2, 204, 0);
    }
}

static inline int get_resrv_dist(const ActiveRoute * restrict ar, int idx, const track_node *start, int stopping_distance) {
    ASSERT(start != NULL, "start node is null! idx = %d, stopping_distance = %d", idx, stopping_distance);
    int cur_dist = 0, total_dist;
    const track_node *resrv_end = start;

    //nth-sensor + stopping_dist + next_switch
    resrv_end = nth_sensor_on_route(2,  &ar->route, &idx, resrv_end, &cur_dist, "get_resrv_dist: 1");
    total_dist += cur_dist;
    if (resrv_end == NULL) {
        return total_dist;
    }

    cur_dist = stopping_distance;
    resrv_end = forward_dist_on_route(  &ar->route, &idx, resrv_end, &cur_dist, "get_resrv_dist: 2");
    total_dist += cur_dist;
    if (resrv_end == NULL) {
        return total_dist;
    }

    cur_dist = 0;
    resrv_end = next_switch_on_route(   &ar->route, &idx, resrv_end, &cur_dist, "get_resrv_dist: 3");
    total_dist += cur_dist;
    if (resrv_end == NULL) {
        return total_dist;
    }
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
    RouteRequest req = {
        .reservations = ts->reservations, // TODO let a train run over its own reservations
        .next = train->last_sensor, //train->next_sensor,
        .prev = train->last_sensor,
        .end = object,
        .min_dist = min_dist,
        .rev_penalty = rev_penalty
    };
    Route route = ROUTE_INIT;
    int distance = GetRoute(trackstate_tid, req, &route);
    rm.ret = distance;
    Reply(tid, &rm, sizeof(rm));
    ActiveRoute ar = ACTIVE_ROUTE_INIT;
    if (route.reverse != 0) {
        //tc_send(&tc, TERMINAL_ROUTE_DBG2, 215, ts.active_train_map[tr]);
        trainserver_begin_reverse(ts, ts->active_train_map[tr], 0);
        ar.reversing = 1;
    } else {
        CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, NAV_SPEED, train->num); // TODO number
        ar.reversing = 0;
    }
    ar.route = route;
    ar.end_node = object;
    ar.remaining_distance = distance + distance_past;
    ar.distance_past = distance_past;
    ar.stopped = 0;
    ar.last_handled_sensor = -1;
    ASSERT(ts->active_train_map[tr] >= 0 && ts->active_train_map[tr] < MAX_CONCURRENT_TRAINS, "Invalid active train: %d", ts->active_train_map[tr]);
    ts->active_routes[ts->active_train_map[tr]] = ar;
    for (int i = 0; i < MAX_ROUTE_COMMAND && ar.route.rcs[i].a != ACTION_NONE; i++){
        tc_send(tc, TERMINAL_ROUTE_DBG, ar.route.rcs[i].swmr, ar.route.rcs[i].a);
    }
    //tc_send(&tc, TERMINAL_ROUTE_DBG2, 211, route.reverse);
    //tc_send(&tc, TERMINAL_ROUTE_DBG2, 214, ar.reversing);
    tc_send(tc, TERMINAL_FLAGS_SET, STATUS_FLAG_FINDING, 0);
}

static inline void train_on_sensor_event(Train * restrict train, TerminalCourier * restrict tc, int event_time, int distance) {
    if (train->last_sensor < 0) {
        return;
    }
    //we haven't reset our calculations && we actually hit the sensor we expected to (and not the one after?)
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
    train->velocity[train->speed] = MOVING_AVERAGE(new_velocity, train->velocity[train->speed], 15);

    tc_send(tc, TERMINAL_VELOCITY_DEBUG, train->velocity[train->speed], distance);
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
        if (ar->idx_resrv == 0) {
            bool success = reserve_track(&ar->route, ar->idx_resrv, &track[SENSOR_TO_NODE(sensor)], rc_to_track_node(ar->route.rcs[0], "init reservations"), &ts->reservations);
            if (unlikely(!success)) {
                PANIC("FIRST step already reservered!");
            }
        }
    }
    ar->remaining_distance = distance_to_on_route(&ar->route, ar->cur_pos_idx, &track[SENSOR_TO_NODE(sensor)], &track[ar->end_node], "ar distance recalculate") + ar->distance_past;
    tc_send(tc, TERMINAL_ROUTE_DBG2, 207, ar->remaining_distance);
}

static inline int ar_stop(ActiveRoute * restrict ar, Train * restrict train, TerminalCourier * restrict tc) {
    // Figure out how long we need to wait:
    int stopdelay = calc_reverse_time_from_velocity(train->velocity[train->speed], ar->remaining_distance, train->stopping_distance[train->speed]);
    DelayStop ds = {.delay = stopdelay, .rv = 0};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_delay_stop, ds.data, train->num);
    ar->stopped = 1;
    tc_send(tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
    return 999999;
}

static inline void activeroute_exec_steps(ActiveRoute * restrict ar, TrainState * restrict ts, TerminalCourier * restrict tc, int resrv_dist, int tr, int cmdtid) {
    const track_node *resrv_start;
    //we assign resrv_end to resrv_start first thing in the loop
    const track_node *resrv_end = rc_to_track_node(ar->route.rcs[ar->idx_resrv], "resrv_start");
    tc_send(tc, TERMINAL_ROUTE_DBG2, 260, resrv_end->num);

    // Perform any actions we need to do:
    int nxt_step_rv_in_range = ACTIVE_ROUTE_NEXT_STEP_RV(ar), k = 0;
    while (ACTIVE_ROUTE_SHOULD_PERFORM_ACTION(ar, resrv_dist, nxt_step_rv_in_range))  { // must perform next actio due to proximity directly or bc the reverse will take a while
        resrv_start = resrv_end;

        ts_exec_step(ts, tc, ar, tr, cmdtid, ar->next_step_distance + (nxt_step_rv_in_range ? 350 : 0), "action loop");

        resrv_end = rc_to_track_node(ar->route.rcs[ar->idx_resrv], "resrv_end");
        //todo will fail after first sensor right now because we've already reserved some of this
        //Therefore, we currently ignore whether we actually could reserve track
        bool __attribute__((unused)) resrv_successful = reserve_track(&ar->route, ar->idx_resrv, resrv_start, resrv_end, &ts->reservations);
        //bool free_successful = free_track(&ar->route, ar->cur_pos_idx, SENSOR_TO_NODE(train->last_sensor), SENSOR_TO_NODE(sensor), &ts.reservations);

        nxt_step_rv_in_range = ACTIVE_ROUTE_NEXT_STEP_RV(ar);
        ASSERT(k++ < 20, "infinite loop of execs");
    }

    tc_send(tc, TERMINAL_ROUTE_DBG2, 261, resrv_end->num);
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

    if (ACTIVE_ROUTE_SHOULD_STOP(ar, train, dist_to_next_snsr))
        resrv_dist = ar_stop(ar, train, tc);

    if (!ACTIVE_ROUTE_DONE_ACTIONS(ar))
        activeroute_exec_steps(ar, ts, tc, resrv_dist, tr, cmdtid);
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

    train_on_sensor_event(train, tc, event_time, distance);
    activeroute_on_sensor_event(ar, train, ts, tc, sensor, distance, tr, cmdtid);

    train->last_sensor = sensor;
    train->last_sensor_time = event_time;
    ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", tr, train->last_sensor);
}

static inline void add_new_train(TrainState * restrict ts, NewTrain data) {
    ASSERT(ts->total_trains >= 0, "Invalid total trains: %d", ts->total_trains);

    ts->active_train_map[data.train] = ts->total_trains;
    ts->active_trains[ts->total_trains].last_sensor = data.sensor;
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
}

void task_train_state(int trackstate_tid) {
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
            Command c = {COMMAND_TR, 15, {.arg2 = train->num}};
            SendCommand(cmdtid, c);

            ar->reversing = 0;
            CreateWith2Args(PRIORITY_LOW, &task_delay_reaccel, NAV_SPEED, train->num); // TODO number
            train->speed = NAV_SPEED;
            tc_send(&tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_REVERSING, 0);
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 206, ar->route.rcs[ar->idx_resrv].swmr);
            break;
        }
        case (NOTIFY_RV_START):
        {
            Reply(tid, &rm, sizeof(rm));
            int train = tm.data;
            int activetrain = ts.active_train_map[train];
            tc_send(&tc, TERMINAL_FLAGS_SET, STATUS_FLAG_REVERSING, 0);
            Command c = {COMMAND_TR, 0, .arg2 = train};
            //tc_send(&tc, TERMINAL_ROUTE_DBG2, 212, train);
            SendCommand(cmdtid, c);
            CreateWith2Args(PRIORITY_NOTIFIER, &task_notify_rv_timeout, calc_reverse_time(&ts, activetrain), activetrain);
            ts.active_trains[activetrain].speed = 0;
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

