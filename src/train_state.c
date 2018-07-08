#include <train_state.h>
#include <track_state.h>
#include <train.h>
#include <message.h>
#include <syscall.h>
#include <name.h>
#include <debug.h>
#include <terminal.h>
#include <track_data.h>

#define TRAIN(ts, tr) (&((ts)->active_trains[(ts)->active_train_map[(tr)]]))

typedef struct train_state{
    int total_trains;
    int active_train_map[NUM_TRAINS];
    Train active_trains[MAX_CONCURRENT_TRAINS];
    Reservation reservations;
} TrainState;

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

int NavigateTo(int trainstatetid, NavigateRequest nav_req, NavigateResult *res) {
    TrainStateMessage msg = {.type = MESSAGE_TRAIN_STATE, .request = NAVIGATE, {.nav_req = nav_req}};
    int r = Send(trainstatetid, &msg, sizeof(msg), res, sizeof(*res));
    return (r >= 0 ? 0 : -1);
}

void init_train_state(TrainState *ts) {
    ts->total_trains = 0;
    Train init_train = {0, FORWARD, {0}, -1, 0, -1, 0, 0};
    Reservation init_reservation = {0, 0};

    for (int i = 0; i < NUM_TRAINS; ++i) {
        ts->active_train_map[i] = -1;
    }

    for (int i = 0; i < MAX_CONCURRENT_TRAINS; i++) {
        ts->active_trains[i] = init_train;
    }

    ts->reservations = init_reservation;
}

static inline int get_active_train_from_sensor(TrainState *ts, const int sensor) {
    int distance;
    for (int skipped = 0; skipped < 4; ++skipped) { // TODO instead of this - build auxiliary sensor array?
        for (int cur = 0; cur < ts->total_trains; ++cur) {
            int next_sensor = ts->active_trains[cur].next_sensor;
            /*
            for (int i = 0; unlikely(i < skipped); ++i) {
                next_sensor = predict_next_sensor(ts->switches, &(ts->track[next_sensor]), NULL, &distance)->num;
            }
            //*/
            if (next_sensor == sensor) {
                return cur;
            }
        }
    }
    return -1;
}

void task_train_state(int trackstate_tid) {
    RegisterAs(NAME_TRAIN_STATE);
    const int puttid = WhoIs(NAME_TERMINAL);

    TrainState ts;
    init_train_state(&ts);

    TrainStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    NavigateResult nav_res = {0, 0, 0, {}};
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

            RouteRequest req = {
                .reservations = ts.reservations,
                .dir = train->direction,
                .next = train->next_sensor,
                .prev = train->last_sensor,
                .end = object,
                .min_dist = min_dist,
                .rev_penalty = rev_penalty
            };
            RouteResult route_res = {FORWARD, 0, 0, {}};
            GetRoute(trackstate_tid, req, &route_res);

            for (int i = 1; i <= NUM_SWITCHES; i++) {
                if (route_res.switches[i].set_state) {
                    nav_res.switches[i] = route_res.switches[i].state; // pick up differences and unnknowns
                } else {
                    nav_res.switches[i] = SWITCH_UNKNOWN;
                }
            }
            nav_res.end_sensor = route_res.end_sensor;
            nav_res.time_after_end_sensor = route_res.dist_after_end_sensor * VELOCITY_PRECISION / train->velocity[train->speed];
            nav_res.speed = (route_res.dir == train->direction) ? CURRENT_SPEED : -1 * train->speed;

            Reply(tid, &nav_res, sizeof(nav_res));
            break;
        }
        case (NOTIFY_SENSOR_EVENT):
        {
            Reply(tid, &rm, sizeof(rm));
            int sensor = tm.sensor_event.sensor;
            int event_time = tm.sensor_event.time;

            if (unlikely(ts.total_trains <= 0)) {
                continue;
            }
            int tr = get_active_train_from_sensor(&ts, sensor);
            ASSERT(tr >= 0, "Could not find which train hit sensor");

            Train *train = &(ts.active_trains[tr]);

            //we haven't reset our calculations && we actually hit the sensor we expected to (and not the one after?)
            if (train->last_sensor >= 0 && sensor == train->next_sensor) {
                int last_error_time = (train->next_sensor_predict_time - event_time);
                int last_error_dist = last_error_time * train->velocity[train->speed] / VELOCITY_PRECISION;
                SendTerminalRequest(puttid, TERMINAL_SENSOR_PREDICT, last_error_time, last_error_dist);

                // Time is in clock-ticks, velocity is in mm/(clock-tick) -> error is in units of mm
                int dt = event_time - train->last_sensor_time;
                int new_velocity = train->last_sensor_dist * VELOCITY_PRECISION / dt;
                train->velocity[train->speed] = MOVING_AVERAGE(new_velocity, train->velocity[train->speed], 15);
            }

            /*
            int distance;
            const track_node *c = &(ts.track[SENSOR_TO_NODE(sensor)]); // last known train position
            const track_node *n = predict_next_sensor(ts.switches, c, NULL, &distance); // next predicted train position

            train->next_sensor_predict_time = event_time + distance * VELOCITY_PRECISION / train->velocity[train->speed];
            SendTerminalRequest(puttid, TERMINAL_VELOCITY_DEBUG, train->velocity[train->speed], n->num);
            SendTerminalRequest(puttid, TERMINAL_DISTANCE_DEBUG, distance, 0);


            train->next_sensor = n->num;
            train->last_sensor_dist = distance;
            //*/
            train->last_sensor = sensor;
            train->last_sensor_time = event_time;
            break;
        }
        case (NOTIFY_TRAIN_SPEED):
        {
            Reply(tid, &rm, sizeof(rm));
            TrainData data = tm.train_data;
            TRAIN(&ts, (int) data.train)->speed = data.speed;
            TRAIN(&ts, (int) data.train)->last_sensor = -1; // reset prediction;
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

            ts.active_train_map[data.train] = ts.total_trains;
            ts.active_trains[ts.total_trains].next_sensor = data.sensor;
            ++ts.total_trains;
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
        default:
        {
            PANIC("Track State Server: Unhandled request type: %d", tm.request);
        }
        }
    }
}
