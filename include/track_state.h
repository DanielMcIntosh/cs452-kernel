#ifndef TRACK_STATE_H
#define TRACK_STATE_H

#define NAME_TRACK_STATE "state"
#define TRACK_A 0
#define TRACK_B 0

typedef struct sensordata {
    int radix: 4;
    unsigned int data: 16;
    int time: 32;
} __attribute__((packed)) SensorData;

typedef enum tsrequest{
    TRAIN_SPEED, // TODO other requests
    SWITCH,
    SENSOR,

    NOTIFY_SENSOR_DATA,
    NOTIFY_TRAIN_SPEED,
    NOTIFY_TRAIN_DIRECTION,
    NOTIFY_SWITCH,

    NUM_TRACK_STATE_REQUESTS
} TrackStateRequest;

void task_track_state();

void requestTrackState(); // TODO

void notifySensorData(int trackstatetid, SensorData data); // TODO

#endif
