#include <kernel.h>
#include <switch.h>

#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#define NUM_SPEEDS 15

#define NAME_TRACK_STATE "state"
#define TRACK_A 1
#define TRACK_B 0

#define CHAR_TO_TRACK(c) ((c) == 'A' ? TRACK_A : TRACK_B)
#define VELOCITY_PRECISION 10000
#define STOPPING_DISTANCE 100

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
    unsigned int speed: 4;
    unsigned int train: 8;
} __attribute__((packed)) TrainData;

typedef struct routemessage{
    int end_sensor;
    int time_after_end_sensor;
    Switch switches[NUM_SWITCHES+1];
} RouteMessage;

typedef enum tsrequest{
    TRAIN_SPEED, // TODO other requests
    SWITCH,
    SENSOR,
    ROUTE,

    NOTIFY_SENSOR_DATA,
    NOTIFY_TRAIN_SPEED,
    NOTIFY_TRAIN_DIRECTION,
    NOTIFY_SWITCH,

    NUM_TRACK_STATE_REQUESTS
} TrackStateRequest;

void task_track_state();

void requestTrackState(); // TODO

int NotifySensorData(int trackstatetid, SensorData data);
int NotifySwitchStatus(int trackstatetid, SwitchData data);
int NotifyTrainSpeed(int trackstatetid, TrainData data);
int GetSwitchState(int trackstatetid, int sw);
int GetRoute(int trackstatetid, int sensor, RouteMessage *rom);
int GetTrainSpeed(int trackstatetid, int train);

#endif
