#include <kernel.h>
#include <switch.h>
#include <message.h>
#include <util.h>
#include <train_state.h>
#include <train.h>

#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#define NAME_TRACK_STATE "trk_st"
#define TRACK_A 1
#define TRACK_B 0

#define PARAM_SPEED 'S'
#define PARAM_DELAY 'D'

#define CHAR_TO_TRACK(c) ((c) == 'A' ? TRACK_A : TRACK_B)

#define SENSOR_TO_NODE(s) (s)
#define SWITCH_TO_NODE(s) (80 + 2 * (s))
#define MERGE_TO_NODE(m) (81 + 2 * (m))
#define ENTER_TO_NODE(n) (124 + 2 * (n))
#define EXIT_TO_NODE(n) (125 + 2 * (n))

// TODO is there a nicer way to do this?
#define TRACK_NODE_TO_INDEX(n) \
    (n->type == NODE_SENSOR ? SENSOR_TO_NODE(n->num) : \
     (n->type == NODE_BRANCH ? SWITCH_TO_NODE(SWCLAMP(n->num)) : \
      (n->type == NODE_MERGE ? MERGE_TO_NODE(SWCLAMP(n->num)) : \
       (n->type == NODE_ENTER ? ENTER_TO_NODE(n->num) : \
        (n->type == NODE_EXIT ? EXIT_TO_NODE(n->num) : -1)))))

#define MAX_SHORT 20
#define MIN_SHORT 2
#define INCREMENT_SHORT 2
#define NUM_SHORTS ((MAX_SHORT / INCREMENT_SHORT) + 1)

#define BFS_MH_SIZE 80

#define TRACK_STATE_TERMINAL_BUFFER_SIZE 300

#define MAX_ROUTE_COMMAND 15
// 18 -> 126 bits which is the min dist to a power of 2 for a while I think - this is totally changeably but routes probably aren't more than 18 switches long?

typedef enum action {
    ACTION_NONE = 0, // default
    ACTION_STRAIGHT = 1,
    ACTION_CURVED = 2,
    ACTION_RV = 3
} __attribute__((packed)) Action; // packed -> use min possible type to store this enum

typedef struct routecommand{
    unsigned int swmr: 5; // 22 switches/merges -> need 5 bytes to represent, | 10 extra values
    Action a: 2; // 4 actions -> need 2 bits to represent 
    // note: reverse only happens after a merge - so ACTION_REVERSE implies swmr refers to a merge; otherwise swmr is a switch
    int packing: 1; // could let the compiler do it, but i feel better this way
}__attribute__((packed)) RouteCommand;

#define SWITCH_NONE 31 
// SWITCH_NONE is the maximum 5 bit unsigned int

typedef struct route {
    RouteCommand rcs[MAX_ROUTE_COMMAND];
    int reverse: 1; int packing: 7;
} Route;

#define ROUTE_INIT {{{0, 0, 0}}, 0, 0}

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
    const Direction dir : 8;
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
} RouteResult;

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

void task_track_state();

int NotifySensorData(int trackstatetid, SensorData data);
int NotifySwitchStatus(int trackstatetid, SwitchData data);
int NotifyParam(int trackstatetid, ParamData data);

int GetSwitchState(int trackstatetid, int sw);
int GetActiveTrain(int trackstatetid, int train);
int GetRoute(int trackstatetid, RouteRequest req);
int GetShort(int trackstatetid, int distance, ShortMessage *sm);

#endif
