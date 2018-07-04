#include <kernel.h>
#include <switch.h>
#include <util.h>

#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#define NUM_SPEEDS 15

#define NAME_TRACK_STATE "state"
#define TRACK_A 1
#define TRACK_B 0

#define PARAM_SPEED 'S'
#define PARAM_DELAY 'D'

#define CHAR_TO_TRACK(c) ((c) == 'A' ? TRACK_A : TRACK_B)
#define VELOCITY_PRECISION 10000
#define CAL_ITERATIONS 8
#define BASE_STOP_DIST_ADJUSTMENT 70

//TODO move these:
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

#define BFS_MH_SIZE 100

typedef struct position {
    const int object : 16;
    const int distance_past : 16;
} TrackPosition;

typedef struct sensordata {
    const unsigned int radix: 4;
    const unsigned int data: 16;
    const int time: 32;
} __attribute__((packed)) SensorData;

typedef struct switchdata{
    const unsigned int state: 4;
    const unsigned int sw: 32;
} __attribute__((packed)) SwitchData;

typedef struct traindata {
    const unsigned int speed;
    const unsigned int train;
} __attribute__((packed)) TrainData;

typedef struct routerequest{
    const TrackPosition position;
    const int train;
} __attribute__((packed)) RouteRequest; // TODO move these structs into the c file.

typedef struct caldata{
    const int iteration : 16;
    const int speed : 16;
    const bool triggered;
} __attribute__((packed)) CalData;

typedef struct paramdata{
    const int key: 16;
    const int param: 16;
    const int value;
} __attribute__((packed)) ParamData;

typedef struct newtrain{
    const int train;
    const int sensor;
} __attribute__((packed)) NewTrain; // TODO move these structs into the c file.

typedef struct routemessage{ // TODO MessageType
    int end_sensor;
    int time_after_end_sensor;
    Switch switches[NUM_SWITCHES+1];
} RouteMessage;

typedef struct shortmessage{
    const int speed;
    const int delay;
} ShortMessage;

typedef enum tsrequest{
    TRAIN_SPEED, // TODO other requests
    SWITCH,
    SENSOR,
    ACTIVE_TRAIN,
    ROUTE,
    SHORT,

    NOTIFY_SENSOR_DATA,
    NOTIFY_TRAIN_SPEED,
    NOTIFY_TRAIN_DIRECTION,
    NOTIFY_SWITCH,
    NOTIFY_CAL,
    NOTIFY_PARAM,
    NOTIFY_NEW_TRAIN,
    NOTIFY_RESERVATION,

    NUM_TRACK_STATE_REQUESTS
} TrackStateRequest;

void task_track_state();

int NotifySensorData(int trackstatetid, SensorData data);
int NotifySwitchStatus(int trackstatetid, SwitchData data);
int NotifyTrainSpeed(int trackstatetid, TrainData data);
int NotifyCalibrationResult(int trackstatetid, CalData data);
int NotifyParam(int trackstatetid, ParamData data);
int NotifyNewTrain(int trackstatetid, NewTrain data);
int NotifyReservation(int trackstatetid, int data);

int GetTrainSpeed(int trackstatetid, int train);
int GetSwitchState(int trackstatetid, int sw);
int GetActiveTrain(int trackstatetid, int train);
int GetRoute(int trackstatetid, RouteRequest req, RouteMessage *rom);
int GetShort(int trackstatetid, int distance, ShortMessage *sm);

#endif
