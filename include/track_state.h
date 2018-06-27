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
#define SWITCH_TO_NODE(s) (80 + 2 * (s))
#define MERGE_TO_NODE(m) (81 + 2 * (m))
#define ENTER_TO_NODE(n) (124 + 2 * (n))
#define EXIT_TO_NODE(n) (125 + 2 * (n))

#define NUM_SHORTS 10

typedef struct sensordata {
    unsigned int radix: 4;
    unsigned int data: 16;
    int time: 32;
} __attribute__((packed)) SensorData;

typedef struct switchdata{
    unsigned int state: 4;
    unsigned int sw: 32;
} __attribute__((packed)) SwitchData;

typedef struct traindata {
    unsigned int speed;
    unsigned int train;
} __attribute__((packed)) TrainData;

typedef struct routerequest{
    int object: 16;
    int distance_past: 16;
} __attribute__((packed)) RouteRequest; // TODO move these structs into the c file.

typedef struct caldata{
    int iteration : 16;
    int speed : 16;
    bool triggered;
} __attribute__((packed)) CalData;

typedef struct paramdata{
    int key: 16;
    int param: 16;
    int value;
} __attribute__((packed)) ParamData;

typedef struct routemessage{ // TODO MessageType
    int end_sensor;
    int time_after_end_sensor;
    Switch switches[NUM_SWITCHES+1];
} RouteMessage;

typedef struct shortmessage{
    int speed;
    int delay;
} ShortMessage;

typedef enum tsrequest{
    TRAIN_SPEED, // TODO other requests
    SWITCH,
    SENSOR,
    ROUTE,
    SHORT,

    NOTIFY_SENSOR_DATA,
    NOTIFY_TRAIN_SPEED,
    NOTIFY_TRAIN_DIRECTION,
    NOTIFY_SWITCH,
    NOTIFY_CAL,
    NOTIFY_PARAM,

    NUM_TRACK_STATE_REQUESTS
} TrackStateRequest;

void task_track_state();

int NotifySensorData(int trackstatetid, SensorData data);
int NotifySwitchStatus(int trackstatetid, SwitchData data);
int NotifyTrainSpeed(int trackstatetid, TrainData data);
int NotifyCalibrationResult(int trackstatetid, CalData data);
int NotifyParam(int trackstatetid, ParamData data);

int GetSwitchState(int trackstatetid, int sw);
int GetRoute(int trackstatetid, RouteRequest req, RouteMessage *rom);
int GetShort(int trackstatetid, int distance, ShortMessage *sm);
int GetTrainSpeed(int trackstatetid, int train);

#endif
