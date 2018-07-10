#include <train_state.h>
#include <track_state.h>
#include <train.h>
#include <message.h>
#include <syscall.h>
#include <name.h>
#include <debug.h>
#include <terminal.h>
#include <track.h>
#include <terminalcourier.h>
#include <command.h>
#include <circlebuffer.h>

#define TRAIN(ts, tr) (&((ts)->active_trains[(ts)->active_train_map[(tr)]]))
typedef struct active_route{
    Route route;
    int train;
    int remaining_distance;
    int next_step_distance;
    int idx;
    int stopped;
} ActiveRoute;

#define ACTIVE_ROUTE_INIT {ROUTE_INIT, 0, 0, 0, 0, 1}
#define ACTIVE_ROUTE_COMPLETE(ar) (ar->route.rcs[ar->idx].a == ACTION_NONE && ar->stopped)

typedef struct train_state{
    int total_trains;
    int active_train_map[NUM_TRAINS];
    Train active_trains[MAX_CONCURRENT_TRAINS];
    Reservation reservations;
    // at the moment: 1 active route, but there can be 1 per train later
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

int NotifyTrainSpeed(int trainstatetid, TrainData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_TRAIN_SPEED, {.train_data = data}};
    return sendTrainState(trainstatetid, &msg);
}

int NotifyCalibrationResult(int trainstatetid, CalData data) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NOTIFY_CAL, {.cal_data = data}};
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
    Train init_train = {0, 0, FORWARD, {0}, -1, 0, -1, 0, 0};
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

static inline int get_active_train_from_sensor(TrainState *ts, const int sensor, int *distance) {
    int min_dist = INT_MAX;
    int train = 0;
    const track_node *d = &track[SENSOR_TO_NODE(sensor)];
    Reservation resrv = {0, 0};
    for (int i = 0; i < ts->total_trains; ++i) {
        const track_node *n = &track[ts->active_trains[i].last_sensor];

        Route r = ROUTE_INIT;
        int distance = find_path_between_nodes(&resrv, 1, 99999, n, d, &r);
        //TODO notify track_state of any switches we would have to have taken, incase a switch wasn't in the expected state
        if (distance < min_dist) {
            min_dist = distance;
            train = i;
        }
    }
    
    *distance = min_dist;
    return train;
}

static void ts_exec_step(TerminalCourier * restrict tc, ActiveRoute * restrict ar, int cmdtid) {
    // do current step:
    RouteCommand rc = ar->route.rcs[ar->idx];
    track_node *cnode = NULL;
    switch (rc.a) {
        case (ACTION_STRAIGHT):
        {
            Command cmd = {COMMAND_SW, STATE_TO_CHAR(SWITCH_STRAIGHT), .arg2 = SWUNCLAMP(rc.swmr)};
            SendCommand(cmdtid, cmd);
            cnode = &track[SWITCH_TO_NODE(rc.swmr-1)];
            break;
        }
        case (ACTION_CURVED):
        {
            Command cmd = {COMMAND_SW, STATE_TO_CHAR(SWITCH_CURVED), .arg2 = SWUNCLAMP(rc.swmr)};
            SendCommand(cmdtid, cmd);
            cnode = &track[SWITCH_TO_NODE(rc.swmr-1)];
            break;
        }
        case (ACTION_RV):
        {
            //TODO
            cnode = &track[MERGE_TO_NODE(rc.swmr-1)];
            break;
        }
        default:
        {
            PANIC("Unhandled action type in ts_exec_step: %d", rc.a);
        }
    }
    // If the route is over, set next step distance to intmax
    if (ar->idx < MAX_ROUTE_COMMAND && ar->route.rcs[ar->idx+1].a != ACTION_NONE) {
        RouteCommand nc = ar->route.rcs[ar->idx+1];
        tc_send(tc, TERMINAL_ROUTE_DBG2, 204, nc.swmr);
        ar->next_step_distance += distance_to_on_route(&ar->route, ar->idx, cnode, rc_to_track_node(nc));
        ar->idx++;
    } else {
        ar->next_step_distance = 99999;
        tc_send(tc, TERMINAL_ROUTE_DBG2, 204, 0);
    }
}

static int ts_notify_terminal_buffer(int tstid, TerminalReq *treq) {
    ASSERT(treq != NULL, "null TerminalRequest output");
    TrainStateMessage tm = {MESSAGE_TRAIN_STATE, TRAIN_STATE_NOTIFY_TERMINAL_COURIER, .data = 0};
    TerminalCourierMessage tcm;
    int r = Send(tstid, &tm, sizeof(tm), &tcm, sizeof(tcm));
    if (r < 0) return r;
    *treq = tcm.req;
    return r;
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

    int stopping_distance[NUM_SPEEDS] =
    { 0, 10, 20,
        30, 40,
        50, 60,
        70, 80,
        90, 150,
        400, 800,
        1500, 2000
    };


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
            int object = tm.nav_req.position.object;
            int distance_past = tm.nav_req.position.distance_past;
            int tr = tm.nav_req.train;
            Train *train = TRAIN(&ts, tr);

            int min_dist, rev_penalty;
            if (train->speed == 0){
                min_dist = 0;
                rev_penalty = 0;
                train->speed = 10;
            } else {
                min_dist = stopping_distance[train->speed] - distance_past;
                // assume the distance it takes to stop is the same as that needed to return to full speed in the opposite direction
                rev_penalty = 2*stopping_distance[train->speed];
            }

            ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", tr, train->last_sensor);
            RouteRequest req = {
                .reservations = ts.reservations, // TODO let a train run over its own reservations
                .dir = train->direction,
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
            if (route.reverse) {
                Command c = {COMMAND_RV, tr, .arg2 = 0};
                SendCommand(cmdtid, c);
            }
            ActiveRoute ar = ACTIVE_ROUTE_INIT;
            //for (int i = 0; i < MAX_ROUTE_COMMAND && ar.idx
            ar.route = route;
            ar.remaining_distance = distance;
            ar.next_step_distance = distance_to_on_route(&ar.route, ar.idx, &track[SENSOR_TO_NODE(train->last_sensor)], rc_to_track_node(route.rcs[0])); // TODO
            ar.stopped = 0;
            ASSERT(ts.active_train_map[tr] >= 0 && ts.active_train_map[tr] < MAX_CONCURRENT_TRAINS, "Invalid active train: %d", ts.active_train_map[tr]);
            ts.active_routes[ts.active_train_map[tr]] = ar;
            for (int i = 0; i < MAX_ROUTE_COMMAND && ar.route.rcs[i].a != ACTION_NONE; i++){
                tc_send(&tc, TERMINAL_ROUTE_DBG, ar.route.rcs[i].swmr, ar.route.rcs[i].a);
            }
            tc_send(&tc, TERMINAL_ROUTE_DBG2, ar.remaining_distance, ar.next_step_distance);
            tc_send(&tc, TERMINAL_FLAGS_SET, STATUS_FLAG_FINDING, 0);
            break;
        }
        case (NOTIFY_SENSOR_EVENT):
        {
            Reply(tid, &rm, sizeof(rm));
            int sensor = tm.sensor_event.sensor;
            int event_time = tm.sensor_event.time;

            if (unlikely(ts.total_trains <= 0)) {
                tc_send(&tc, TERMINAL_ROUTE_DBG2, -1, ts.total_trains);
                continue;
            }
            int distance;
            int tr = get_active_train_from_sensor(&ts, sensor, &distance);
            ASSERT(tr >= 0, "Could not find which train hit sensor");
            //tc_send(&tc, TERMINAL_ROUTE_DBG2, 202, tr);

            Train *train = &(ts.active_trains[tr]);
            ActiveRoute *ar = &(ts.active_routes[tr]);

            //we haven't reset our calculations && we actually hit the sensor we expected to (and not the one after?)
            if (train->last_sensor >= 0) {
                if (train->velocity[train->speed] != 0) {
                    int predicted_time = train->last_sensor_time + distance * VELOCITY_PRECISION / train->velocity[train->speed];
                    int error_time = (predicted_time - event_time);
                    int error_dist = error_time * train->velocity[train->speed] / VELOCITY_PRECISION;
                    tc_send(&tc, TERMINAL_SENSOR_PREDICT, error_time, error_dist);
                }
                // Time is in clock-ticks, velocity is in mm/(clock-tick) -> error is in units of mm
                int dt = event_time - train->last_sensor_time;
                int new_velocity = distance * VELOCITY_PRECISION / dt;
                train->velocity[train->speed] = MOVING_AVERAGE(new_velocity, train->velocity[train->speed], 15);
            }

            tc_send(&tc, TERMINAL_VELOCITY_DEBUG, train->velocity[train->speed], distance);
            train->last_sensor = sensor;
            train->last_sensor_time = event_time;
            ASSERT(train->last_sensor <= TRACK_MAX && train->last_sensor >= 0, "invalid last sensor for train %d: %d", tr, train->last_sensor);

            //tc_send(&tc, TERMINAL_ROUTE_DBG2, ar->remaining_distance, ar->next_step_distance);
            //tc_send(&tc, TERMINAL_ROUTE_DBG2, 203, ts.total_trains);
            if (!ACTIVE_ROUTE_COMPLETE(ar)){
                tc_send(&tc, TERMINAL_ROUTE_DBG2, 207, ar->remaining_distance);

                int idx = ar->idx, tmp = 0;
                const track_node *resrv_end = &track[SENSOR_TO_NODE(sensor)];
                //nth-sensor + stopping_dist + next_switch
                resrv_end = nth_sensor_on_route(2, &ar->route, &idx, resrv_end, &tmp);
                tmp = stopping_distance[train->speed];
                resrv_end = forward_dist_on_route(  &ar->route, &idx, resrv_end, &tmp);
                resrv_end = next_switch_on_route(   &ar->route, &idx, resrv_end, &tmp);

                //todo will fail after first sensor right now because we've already reserved some of this
                bool resrv_successful = reserve_track(&ar->route, ar->idx, &track[SENSOR_TO_NODE(sensor)], resrv_end, &ts.reservations);

                // Perform any actions we need to do:
                ar->remaining_distance -= distance;
                ar->next_step_distance -= distance;

                if (!resrv_successful || (ar->remaining_distance <= stopping_distance[train->speed] && !ar->stopped)) {
                    Command stop = {COMMAND_TR, 0, .arg2 = train->num}; 
                    SendCommand(cmdtid, stop);
                    ar->stopped = 1;
                    tc_send(&tc, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
                }

                if (resrv_successful) {
                    while (ar->next_step_distance <= 1000) { // TODO (distance to next sensor)
                        tc_send(&tc, TERMINAL_ROUTE_DBG2, 206, ar->next_step_distance);
                        ts_exec_step(&tc, ar, cmdtid);
                    }
                }
            }
            
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
        case (NOTIFY_NEW_TRAIN):
        {
            Reply(tid, &rm, sizeof(rm));
            NewTrain data = tm.new_train;
            ASSERT(ts.total_trains >= 0, "Invalid total trains: %d", ts.total_trains);

            ts.active_train_map[data.train] = ts.total_trains;
            ts.active_trains[ts.total_trains].last_sensor = data.sensor;
            ts.active_trains[ts.total_trains].num = data.train;
            ++ts.total_trains;
            tc_send(&tc, TERMINAL_ECHO, '0' + ts.total_trains, 0);
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
