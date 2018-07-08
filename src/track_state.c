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
#include <minheap.h>
#include <terminalcourier.h>
#include <circlebuffer.h>

typedef struct track{
    int track_number;
    SwitchState switches[NUM_SWITCHES+1];
    Sensor sensors[NUM_SENSORS];
    track_node track[TRACK_MAX];
} TrackState;

typedef struct visited {
    long long switches: NUM_SWITCHES;
} Visited;

#define IsVisited(v, n) \
        ((v).switches & (1 << (SWCLAMP(n) - 1)))
#define Visit(v, n) \
        (v).switches |= (1 << (SWCLAMP(n) - 1));

typedef struct trackpath{
    Visited visited;
    Direction dir;
    SwitchState switches[NUM_SWITCHES+1];
    SwitchState merges[NUM_SWITCHES+1];
} TrackPath;

typedef struct tsmessage{
    const MessageType type;
    const TrackStateRequest request;
    union {
        int data;
        SensorData sensor_data;
        SwitchData switch_data;
        RouteRequest route_request;
        ParamData param_data;
    };
} TrackStateMessage;

typedef struct bfsnode {
    TrackPath tp;
    const track_node *current_node;
    const track_node *previous_node;
    struct bfsnode *next;
} BFSNode;

static BFSNode* q_pop(BFSNode** freeQ, BFSNode** freeQTail){
    ASSERT(freeQ != NULL, "Cannot pop from empty free queue");
    BFSNode *ret = *freeQ;
    *freeQ = (*freeQ)->next;
    if (*freeQ == NULL)
        *freeQTail = NULL;

    ret->next = NULL;
    return ret;
}

static void q_add(BFSNode** freeQ, BFSNode** freeQTail, BFSNode *node){
    if (*freeQTail == NULL) {
        *freeQ = node;
        *freeQTail = node;
        return;
    }

    (*freeQTail)->next = node;
    *freeQTail = node;
}

void init_track_state(TrackState *ts, int track) {
    ts->track_number = track;
    Sensor init_sensor = {SENSOR_OFF};

   for (int i = 0; i < NUM_SENSORS; i++) {
        ts->sensors[i] = init_sensor;
    }
    for (int i = 0; i < NUM_SWITCHES; i++) {
        ts->switches[i] = SWITCH_STRAIGHT;
    }
    if (track == TRACK_A)
        init_tracka(ts->track);
    else
        init_trackb(ts->track);
}

static track_node* predict_next_sensor(const SwitchState * restrict ts, const track_node *last_sensor, SwitchState * restrict path, int * restrict distance) {
    // basic plan for this: linked list search: follow state given by TrackState
    // Stop when another sensor is found

    track_node *n = last_sensor->edge[DIR_AHEAD].dest;
    *distance = last_sensor->edge[DIR_AHEAD].dist;
    track_edge *e;
    while (n != NULL && n->type != NODE_SENSOR) {
        switch (n->type) {
        case (NODE_BRANCH):
        {
            //ASSERT(!(path &&path->merges[SWCLAMP(n->num)].state != SWITCH_UNKNOWN && path->switches[SWCLAMP(n->num)].state != SWITCH_UNKNOWN), "path covers branch & merge?");
            if (path && path[SWCLAMP(n->num)] != SWITCH_UNKNOWN) {
                e = &n->edge[STATE_TO_DIR(path[SWCLAMP(n->num)])];
            }
            else {
                e = &n->edge[STATE_TO_DIR(ts[SWCLAMP(n->num)])];
                path[SWCLAMP(n->num)] = ts[SWCLAMP(n->num)];
            }

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
static int reverse_distance_from_node(const SwitchState * restrict ts, const track_node *destination, int distance, SwitchState * restrict path, int * restrict wakeup_sensor, int * restrict dist_after_sensor) {
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

static int forward_distance_from_node(const SwitchState * restrict ts, const track_node *destination, int distance, SwitchState * restrict path, int * restrict wakeup_sensor, int * restrict dist_after_sensor) {
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
    *dist_after_sensor = cdist;
    return 0;
}

static int find_path_between_nodes(const Reservation * restrict reservations, int min_dist, int rev_penalty, const track_node *origin, const track_node *dest, const track_node *previous, TrackPath * restrict l) {
    entry_t mh_array[BFS_MH_SIZE];
    minheap_t mh = {mh_array, 0, BFS_MH_SIZE};
    entry_t entry;
    // IDEA: minheap contains pointers to some struct. That struct contains a TrackPath and some other data I guess? We allocate those structs in a big array on the stack here.
    // Keep a free queue of those structs so we can free them whenever we drop a node?
    BFSNode bfsnodes[BFS_MH_SIZE];

    BFSNode* freeQ = &bfsnodes[0];
    BFSNode* freeQTail = &bfsnodes[BFS_MH_SIZE-1];

    for (int i = 0; i < BFS_MH_SIZE; i++) {
        bfsnodes[i].next = (i < BFS_MH_SIZE - 1 ? &(bfsnodes[i+1]) : NULL);
        bfsnodes[i].tp.visited.switches = 0ll;
        for (int i = 0; i < NUM_SWITCHES + 1; i++) {
            bfsnodes[i].tp.switches[i] = SWITCH_UNKNOWN;
            bfsnodes[i].tp.merges[i] = SWITCH_UNKNOWN;
        }
    }

    BFSNode * fw = q_pop(&freeQ, &freeQTail);
    fw->tp = *l;
    fw->tp.visited.switches = 0;
    fw->current_node = origin;
    fw->previous_node = previous;
    mh_add(&mh, (int) fw, 0);
    BFSNode * rv = q_pop(&freeQ, &freeQTail);
    fw->tp = *l;
    rv->tp.dir = !(l->dir);
    fw->tp.visited.switches = 0;
    rv->current_node = origin->reverse;
    rv->previous_node = previous; // TODO get actual reverse?
    mh_add(&mh, (int) rv, rev_penalty);

    bfsnodes[BFS_MH_SIZE-1].next = NULL;
    int k = 0;

    //int tid = WhoIs(NAME_TERMINAL);

    while (mh_remove_min(&mh, &entry) == 0){
        ASSERT(k++ <= 10000, "probably an infinite loop");
        BFSNode *bn = (BFSNode*) entry.item;
        int distance = entry.value;
        TrackPath tp = bn->tp;
        const track_node *cn = bn->current_node;
        const track_node *pn = bn->previous_node;
        q_add(&freeQ, &freeQTail, bn);
        ASSERT( !(tp.switches[SWCLAMP(153)] == SWITCH_CURVED && tp.switches[SWCLAMP(154)] == SWITCH_CURVED) &&
                !(tp.switches[SWCLAMP(156)] == SWITCH_CURVED && tp.switches[SWCLAMP(155)] == SWITCH_CURVED), "CC");

        ASSERT(cn != NULL && cn->type != NODE_NONE, "ERROR");
        if (unlikely(cn == dest) && distance > ((tp.dir == l->dir) ? min_dist : min_dist + rev_penalty)) { // found shortest path > min_dist
            *l = tp;
            return distance;
        } else if ((0x1ULL << (TRACK_NODE_TO_INDEX(cn) % 64)) & ((TRACK_NODE_TO_INDEX(cn) < 64) ? reservations->bits_low : reservations->bits_high)) { // TODO allow trains to use their own reserved track
            continue;
        } 
        // continue the search

        if (cn->type == NODE_BRANCH) {
            if (IsVisited(tp.visited, cn->num)) {
                continue;
            }
            Visit(tp.visited, cn->num);

            BFSNode * straight = q_pop(&freeQ, &freeQTail);
            straight->current_node = cn->edge[DIR_STRAIGHT].dest;
            straight->previous_node = cn;
            tp.switches[SWCLAMP(cn->num)] = SWITCH_STRAIGHT;
            memcpy(&straight->tp, &tp, sizeof(TrackPath));
            mh_add(&mh, (unsigned long int) straight, distance + cn->edge[DIR_STRAIGHT].dist);

            BFSNode * curved = q_pop(&freeQ, &freeQTail);
            curved->current_node = cn->edge[DIR_CURVED].dest;
            curved->previous_node = cn;
            tp.switches[SWCLAMP(cn->num)] = SWITCH_CURVED;
            if (cn->num >= 153){
               tp.switches[SWCLAMP(SW3_COMPLEMENT(cn->num))] = SWITCH_STRAIGHT; 
               Visit(tp.visited, SW3_COMPLEMENT(cn->num));
            }
            memcpy(&curved->tp, &tp, sizeof(TrackPath));
            mh_add(&mh, (unsigned long int) curved, distance + cn->edge[DIR_CURVED].dist);
        }
        if (cn->type == NODE_MERGE) {
            // figure out which side of the branch we are:
            const track_node *pr = pn->reverse;
            const track_node *br = cn->reverse;
            if (br->edge[DIR_STRAIGHT].dest == pr) {
                tp.merges[SWCLAMP(cn->num)] = SWITCH_STRAIGHT;
            } else {
                tp.merges[SWCLAMP(cn->num)] = SWITCH_CURVED;
            }
        }
        if (cn->type == NODE_MERGE || cn->type == NODE_SENSOR || cn->type == NODE_ENTER) {
            BFSNode * ahead = q_pop(&freeQ, &freeQTail);
            ahead->current_node = cn->edge[DIR_AHEAD].dest;
            ahead->previous_node = cn;
            memcpy(&ahead->tp, &tp, sizeof(TrackPath));
            mh_add(&mh, (unsigned long int) ahead, distance + cn->edge[DIR_AHEAD].dist);
        }
    }

    return -1;
}

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

int GetRoute(int trackstatetid, RouteRequest req, RouteResult *res) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = ROUTE, {.route_request = req}};
    int r = Send(trackstatetid, &msg, sizeof(msg), res, sizeof(*res));
    return (r >= 0 ? 0 : -1);
}

int GetShort(int trackstatetid, int distance, ShortMessage *sm) {
    TrackStateMessage msg = {.type = MESSAGE_TRACK_STATE, .request = SHORT, {.data = distance}};
    int r = Send(trackstatetid, &msg, sizeof(msg), sm, sizeof(*sm));
    return (r >= 0 ? 0 : -1);
}

void task_track_state(int track) {
    RegisterAs(NAME_TRACK_STATE);
    const int train_evt_courier_tid = Create(PRIORITY_MID, &task_train_event_courier);

    TrackState ts;
    init_track_state(&ts, track);

    circlebuffer_t cb_terminal;
    char cb_terminal_buf[TRACK_STATE_TERMINAL_BUFFER_SIZE];
    cb_init(&cb_terminal, cb_terminal_buf, TRACK_STATE_TERMINAL_BUFFER_SIZE);
    TerminalCourier tc = { -1, &cb_terminal};
    CreateWith2Args(PRIORITY_NOTIFIER, &task_terminal_courier, MyTid(), (int) &NotifyTerminalCourier);

    TrackStateMessage tm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    RouteResult route_res = {TRUE, 0, 0, {{SWITCH_UNKNOWN, FALSE}}};
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
            Direction dir = tm.route_request.dir;
            int next = tm.route_request.next;
            int prev = tm.route_request.prev;
            int end = tm.route_request.end;
            int min_dist = tm.route_request.min_dist;
            int rev_penalty = tm.route_request.rev_penalty;
            Reservation reservations = tm.route_request.reservations;

            if (next < 0) {
                int __attribute__((unused)) distance;
                next = predict_next_sensor(ts.switches, &ts.track[prev], NULL, &distance)->num;
            }

            const track_node *n = &ts.track[next], *o = &ts.track[prev], *d = &ts.track[end];

            TrackPath tp = {{0}, dir, {}, {}};
            int distance = find_path_between_nodes(&reservations, min_dist, rev_penalty, n, d, o, &tp);
            if (unlikely(distance < 0)) {
                PANIC("failed to find path between %d and %d", next, end);
            }

            if (min_dist > 0) {
                int err = reverse_distance_from_node(ts.switches, d, min_dist, tp.merges, &route_res.end_sensor, &route_res.dist_after_end_sensor);
                ASSERT(err == 0, "Reverse Distance Failed");
            } else {
                min_dist *= -1;
                int err = forward_distance_from_node(ts.switches, d, min_dist, tp.switches, &route_res.end_sensor, &route_res.dist_after_end_sensor);
                ASSERT(err == 0, "Forward Distance Failed");
            }

            for (int i = 1; i <= NUM_SWITCHES; i++) {
                route_res.switches[i].state = tp.switches[i];
                if (tp.switches[i] != ts.switches[i]) {
                    route_res.switches[i].set_state = TRUE;
                }
            }

            route_res.dir = tp.dir;

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
