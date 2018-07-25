#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#include <route.h>
#include <message.h>
#include <switch.h>
#include <track_position.h>
#include <reservations.h>

#define NAME_TRACK_STATE "trk_st"
#define TRACK_A 1
#define TRACK_B 0

#define PARAM_SPEED 'S'
#define PARAM_DELAY 'D'

#define CHAR_TO_TRACK(c) ((c) == 'A' ? TRACK_A : TRACK_B)

#define MAX_SHORT 20
#define MIN_SHORT 2
#define INCREMENT_SHORT 2
#define NUM_SHORTS ((MAX_SHORT / INCREMENT_SHORT) + 1)

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
    Blockage blockages;
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

typedef struct fdistreq {
    unsigned int cnode: 8;
    unsigned int distance: 16;
} __attribute__((packed)) FdistReq;

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
    FDIST,

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

int GetSwitchState(int trackstatetid, int sw) __attribute__((warn_unused_result));
int GetRoute(int trackstatetid, RouteRequest req, Route *res) __attribute__((nonnull));
int GetShort(int trackstatetid, int distance, ShortMessage *sm) __attribute__((nonnull));
TrackPosition GetFdist(int trackstatetid, FdistReq fdr) __attribute__((warn_unused_result));

#endif
