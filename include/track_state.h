#include <kernel.h>
#include <switch.h>
#include <message.h>
#include <util.h>
#include <train_state.h>
#include <train.h>
#include <track.h>

#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#define NAME_TRACK_STATE "trk_st"
#define TRACK_A 1
#define TRACK_B 0

#define PARAM_SPEED 'S'
#define PARAM_DELAY 'D'

#define CHAR_TO_TRACK(c) ((c) == 'A' ? TRACK_A : TRACK_B)

#define SENSOR_TO_NODE(s) (s)
#define SWITCH_TO_NODE(s) (80 + 2 * (SWCLAMP(s) - 1))
#define SWITCH_TO_NODE_NSC(s) (80 + 2 * (s - 1))
#define MERGE_TO_NODE(m) (81 + 2 * (SWCLAMP(m) - 1))
#define MERGE_TO_NODE_NSC(m) (81 + 2 * (m - 1))
#define ENTER_TO_NODE(n) (124 + 2 * (n))
#define EXIT_TO_NODE(n) (125 + 2 * (n))

// TODO is there a nicer way to do this?
#define TRACK_NODE_TO_INDEX(n) \
    (n->type == NODE_SENSOR ? SENSOR_TO_NODE(n->num) : \
     (n->type == NODE_BRANCH ? SWITCH_TO_NODE(n->num) : \
      (n->type == NODE_MERGE ? MERGE_TO_NODE(n->num) : \
       (n->type == NODE_ENTER ? ENTER_TO_NODE(n->num) : \
        (n->type == NODE_EXIT ? EXIT_TO_NODE(n->num) : -1)))))

#define MAX_SHORT 20
#define MIN_SHORT 2
#define INCREMENT_SHORT 2
#define NUM_SHORTS ((MAX_SHORT / INCREMENT_SHORT) + 1)

#define BFS_MH_SIZE 80

#define TRACK_STATE_TERMINAL_BUFFER_SIZE 300

typedef struct sensordata {
    const unsigned int radix: 4;
    const unsigned int data: 16;
    const int time: 32;
} __attribute__((packed)) SensorData;

typedef struct switchdata{
    const SwitchState state;
    const unsigned int sw;
} __attribute__((packed)) SwitchData;

typedef struct routerequest{
    const Reservation reservations;
    const unsigned int next : 8;
    const unsigned int prev : 8;
    const unsigned int end : 8;
    const int min_dist : 16;
    const int rev_penalty : 16;
} __attribute__((packed)) RouteRequest;

typedef struct paramdata{
    const int key: 16;
    const int param: 16;
    const int value;
} __attribute__((packed)) ParamData;

typedef struct routeresult{
    MessageType type;
    Route route;
    int distance;
} RouteResult;

#define ROUTE_RESULT_INIT {MESSAGE_ROUTE, ROUTE_INIT, 0}

typedef struct shortmessage{
    const int speed;
    const int delay;
} ShortMessage;

typedef enum trkstrequest{
    SWITCH,
    SENSOR,
    ROUTE,
    SHORT,

    NOTIFY_SENSOR_DATA,
    NOTIFY_SWITCH,
    NOTIFY_PARAM,
    NOTIFY_TERMINAL_COURIER,

    NUM_TRACK_STATE_REQUESTS
} TrackStateRequest;

void __attribute__((noreturn)) task_track_state();

int NotifySensorData(int trackstatetid, SensorData data);
int NotifySwitchStatus(int trackstatetid, SwitchData data);
int NotifyParam(int trackstatetid, ParamData data);

int GetSwitchState(int trackstatetid, int sw);
int GetRoute(int trackstatetid, RouteRequest req, Route *res);
int GetShort(int trackstatetid, int distance, ShortMessage *sm);

#endif
