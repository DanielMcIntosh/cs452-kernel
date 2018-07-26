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
#include <track.h>
#include <train_event.h>
#include <train_state.h>
#include <util.h>
#include <minheap.h>
#include <terminalcourier.h>
#include <circlebuffer.h>

typedef struct track{
    SwitchState switches[NUM_SWITCHES+1];
    Sensor sensors[NUM_SENSORS];
} TrackState;

typedef struct visited {
    long long switches: NUM_SWITCHES;
} Visited;

#define IsVisited(v, n) \
        ((v).switches & (1 << (SWCLAMP(n) - 1)))
#define Visit(v, n) \
        (v).switches |= (1 << (SWCLAMP(n) - 1));

typedef struct tsmessage{
    const MessageType type;
    const TrackStateRequest request;
    union {
        int data;
        SensorData sensor_data;
        SwitchData switch_data;
        RouteRequest route_request;
        ParamData param_data;
        FdistReq fdist_request;
    };
} TrackStateMessage;

void init_track_state(TrackState *ts) {
    Sensor init_sensor = {SENSOR_OFF};

   for (int i = 0; i < NUM_SENSORS; i++) {
        ts->sensors[i] = init_sensor;
    }
    for (int i = 0; i < NUM_SWITCHES; i++) {
        ts->switches[i] = SWITCH_STRAIGHT;
    }
}

static const track_edge * __attribute__((nonnull(1, 2))) predict_next_edge(const SwitchState * restrict ts, const track_node *prev, Route * restrict path, int * restrict start_idx) {
    const track_edge *e;
    switch (prev->type) {
    case (NODE_BRANCH):
    {
        SwitchState state = ts[SWCLAMP(prev->num)];
        e = &prev->edge[STATE_TO_DIR(state)];
        if (path != NULL) {
            path->rcs[*start_idx].a = (state == SWITCH_STRAIGHT) ? ACTION_STRAIGHT : ACTION_CURVED;
            path->rcs[*start_idx].swmr = SWCLAMP(prev->num);
            (*start_idx)++;
        }
        break;
    }
    case (NODE_SENSOR):
    case (NODE_ENTER):
    case (NODE_MERGE):
    {
        e = &prev->edge[DIR_AHEAD];
        break;
    }
    case (NODE_EXIT):
    {
        e = NULL;
        break;
    }
    default:
    {
        PANIC("in: predict_next_edge - INVALID TRACK NODE TYPE: %d", prev->type);
    }
    }

    return e;
}

static const track_node * __attribute__((nonnull)) predict_next_sensor(const SwitchState * restrict ts, const track_node *prev, Route * restrict path, int * restrict start_idx) {
    ASSERT_VALID_TRACK(prev);
    // basic plan for this: linked list search: follow state given by TrackState
    // Stop when another sensor is found

    const track_node *n = prev;
    
    const track_edge *e;
    //do-while because we don't want to return prev if prev is already a sensor
    do {
        e = predict_next_edge(ts, n, path, start_idx);
        if (e == NULL) break;
        n = e->dest;
    } while (n != NULL && n->type != NODE_SENSOR);
    if (n == NULL || n->type != NODE_SENSOR) n = prev;

    return n;
}
/*
static int reverse_distance_from_node(const Switch * restrict ts, const track_node *destination, int distance, int velocity, const Switch * restrict path, int * restrict wakeup_sensor, int * restrict time_after_sensor) {
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
    *dist_after_sensor = cdist;
    return 0;
}

*/
static int forward_distance_from_node(const SwitchState * restrict ts, const track_node *destination, int distance, int * restrict wakeup_sensor, int * restrict dist_after_sensor) {
    ASSERT(distance >= 0, "Cannot forward find negative distance");
    ASSERT_VALID_TRACK(destination);
    const track_node * restrict next_sensor = destination;
    const track_edge *e;
    *wakeup_sensor = TRACK_NODE_TO_INDEX(next_sensor);
    int tdist = 0, last_tdist = 0, cdist;
    while (tdist < distance) {
        *wakeup_sensor = TRACK_NODE_TO_INDEX(next_sensor);
        last_tdist = tdist;
        e = predict_next_edge(ts, next_sensor, NULL, NULL);
        ASSERT(e != NULL, "NULL edge predicted ahead");
        tdist += e->dist;
        next_sensor = e->dest;
        ASSERT_VALID_TRACK(next_sensor);
    }
    cdist = distance - last_tdist; // remaining distance (in mm)
    *dist_after_sensor = cdist;
    return 0;
}
//*/

static inline int sendTrackState(int trackstatetid, const TrackStateMessage *msg) {
    ReplyMessage rm;
    int r = Send(trackstatetid, msg, sizeof(*msg), &rm, sizeof(rm));
    return (r >= 0 ? rm.ret : r);
}

int NotifySensorData(int trackstatetid, SensorData data) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_SENSOR_DATA, {.sensor_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifySwitchStatus(int trackstatetid, SwitchData data) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_SWITCH, {.switch_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifyParam(int trackstatetid, ParamData data) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_PARAM, {.param_data = data}};
    return sendTrackState(trackstatetid, &msg);
}

int NotifyTerminalCourier(int trackstatetid, TerminalReq *treq) {
    ASSERT(treq != NULL, "null TerminalRequest return");
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = NOTIFY_TERMINAL_COURIER, {.data = 0}};
    TerminalCourierMessage tcm;
    int r = Send(trackstatetid, &msg, sizeof(msg), &tcm, sizeof(tcm));
    if (r < 0) return r;
    *treq = tcm.req;
    return r;
}
    
int GetSwitchState(int trackstatetid, int sw) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = SWITCH, {.data = sw}};
    return sendTrackState(trackstatetid, &msg);
}

int GetRoute(int trackstatetid, RouteRequest req, Route *res, int *start_idx) {
    RouteResult rom = ROUTE_RESULT_INIT;
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = ROUTE, {.route_request = req}};
    int r = Send(trackstatetid, &msg, sizeof(msg), &rom, sizeof(rom));
    if (r < 0) return -1;
    *res = rom.route;
    *start_idx = rom.start_idx;
    return rom.distance;
}

int GetShort(int trackstatetid, int distance, ShortMessage *sm) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = SHORT, {.data = distance}};
    int r = Send(trackstatetid, &msg, sizeof(msg), sm, sizeof(*sm));
    return (r >= 0 ? 0 : -1);
}

TrackPosition GetFdist(int trackstatetid, FdistReq fdr){
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = FDIST, {.fdist_request = fdr}};
    ReplyMessage rm;
    int r = Send(trackstatetid, &msg, sizeof(msg), &rm, sizeof(rm));
    TrackPosition tp = {0,0};
    if (r < 0) return tp;
    TrackPositionUnion tpu = {.bytes = rm.ret};
    return tpu.tp;
}

void __attribute__((noreturn)) task_track_state() {
    RegisterAs(NAME_TRACK_STATE);
    const int train_evt_courier_tid = Create(PRIORITY_MID, &task_train_event_courier);

    TrackState ts;
    init_track_state(&ts);

    circlebuffer_t cb_terminal;
    char cb_terminal_buf[TRACK_STATE_TERMINAL_BUFFER_SIZE];
    cb_init(&cb_terminal, cb_terminal_buf, TRACK_STATE_TERMINAL_BUFFER_SIZE);
    TerminalCourier tc = { -1, &cb_terminal};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_terminal_courier, MyTid(), (int) &NotifyTerminalCourier);

    TrackStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    RouteResult route_res = ROUTE_RESULT_INIT;
    int tid;

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
        120, 122, 130, 135, 141 // TODO fix the whole short move framework
    };

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));

        switch (tm.request) {
        case (SWITCH):
        {
            rm.ret = ts.switches[SWCLAMP(tm.data)];
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
            int prev = tm.route_request.prev;
            int end = tm.route_request.end;
            int min_dist = tm.route_request.min_dist;
            int rev_penalty = tm.route_request.rev_penalty;
            Blockage blockages = tm.route_request.blockages;

            Route r = ROUTE_INIT;
            int start_idx = 0;
            const track_node *next = predict_next_sensor(ts.switches, &track[prev], &r, &start_idx);
            ASSERT_VALID_TRACK(next);
            tc_send(&tc, TERMINAL_ROUTE_DBG2, 208, next->num);
            const track_node *dest = &track[end];

            int distance = find_path_between_nodes(&blockages, min_dist, INT_MAX - 20, rev_penalty, next, dest, &r);
            ASSERT(0 <= distance && distance < INT_MAX, "could not find route from %s to %s (starting at %s, start_idx = %d) (rev_penalty = %d)", track[prev].name, dest->name, next->name, start_idx, rev_penalty);

            if (likely(0 <= distance && distance < INT_MAX)) {
                route_res.route = r;
                route_res.distance = distance;
                route_res.start_idx = start_idx;
            } else {
                route_res.distance = distance;
                for (int i = 0; i < MAX_ROUTE_COMMAND; i++) {
                    route_res.route.rcs[i].swmr = SWITCH_NONE;
                }
            }

            Reply(tid, &route_res, sizeof(route_res));
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
        case (FDIST):
        {
            FdistReq fdr = tm.fdist_request;
            int node = 0, distance = 0;
            //PANIC("WHY __-__ %s (%d), %d", track[fdr.cnode].name, fdr.cnode, fdr.distance);
            StoreValue(VALUE_LAST_FN, 2);
            int err = forward_distance_from_node(ts.switches, &track[fdr.cnode], fdr.distance, &node, &distance);
            if (err < 0) {
                rm.ret = -1;
                Reply(tid, &rm, sizeof(rm));
                break;
            }
            TrackPositionUnion tpu = {.tp = {node, distance}};
            rm.ret = tpu.bytes;
            Reply(tid, &rm, sizeof(rm));
            break;
        }
        case (NOTIFY_SENSOR_DATA):
        {
            Reply(tid, &rm, sizeof(rm));
            SensorData f = tm.sensor_data;
            int k = 1 << 15;
            for (int i = 0; i <= 15; i++, k >>= 1) {
                int sensor = SENSOR_PAIR_TO_SENSOR(f.radix, i);
                if (likely(!(f.data & k))) {
                    ts.sensors[sensor].state = SENSOR_OFF;
                } else if (ts.sensors[sensor].state != SENSOR_ON) {
                    ts.sensors[sensor].state = SENSOR_ON;
                    tc_send(&tc, TERMINAL_SENSOR, sensor, f.time);

                    TrainEvent_Notify(train_evt_courier_tid, sensor);
                    SensorEvent sse = {.sensor = sensor, .time = f.time};
                    NotifySensorEvent(WhoIs(NAME_TRAIN_STATE), sse);
                }
            }
            break;
        }
        case (NOTIFY_SWITCH):
        {
            Reply(tid, &rm, sizeof(rm));
            SwitchData data = tm.switch_data;
            ts.switches[SWCLAMP(data.sw)] = data.state;
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
        case (NOTIFY_TERMINAL_COURIER):
        {
            tc_update_notifier(&tc, tid);
            break;
        }
        default:
        {
            PANIC("Track State Server: Unhandled request type: %d", tm.request);
        }
        }
    }
}

