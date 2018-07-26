#include "util.h"
#include "switch.h"
#include "track_position.h"

#ifndef _TRAIN_STATE_H_
#define _TRAIN_STATE_H_

#define NAME_TRAIN_STATE "trn_st"

#define VELOCITY_PRECISION 10000
#define ACCELERATION_PRECISION 1000000
#define CAL_ITERATIONS 8
#define BASE_STOP_DIST_ADJUSTMENT 70

#define TRAIN_STATE_TERMINAL_BUFFER_SIZE 1000

typedef struct traindata {
    const unsigned int speed;
    const unsigned int train;
} __attribute__((packed)) TrainData;

typedef struct caldata{
    const int iteration : 16;
    const int speed : 16;
    const bool triggered;
} __attribute__((packed)) CalData;

typedef struct newtrain{
    const int train;
    const int sensor;
} __attribute__((packed)) NewTrain; // TODO move these structs into the c file.

typedef struct sensor_event {
    const int sensor;
    const int time;
} __attribute__((packed)) SensorEvent; // TODO move these structs into the c file.

typedef struct navigaterequest{
    const TrackPosition position;
    const int train;
} __attribute__((packed)) NavigateRequest;

typedef enum trnstrequest{
    TRAIN_SPEED, 
    ACTIVE_TRAIN,
    NAVIGATE,
    TRAIN_POSITION,

    NOTIFY_SENSOR_EVENT,
    NOTIFY_TRAIN_SPEED,
    NOTIFY_TRAIN_DIRECTION,
    NOTIFY_NEW_TRAIN,
    NOTIFY_RESERVATION,
    NOTIFY_RV_TIMEOUT,
    NOTIFY_RV_START,
    NOTIFY_STOP,
    NOTIFY_STOPPED,
    NOTIFY_RANDOM_ROUTE,
    TRAIN_STATE_NOTIFY_TERMINAL_COURIER,

    NUM_TRAIN_STATE_REQUESTS
} TrainStateRequest;

void __attribute__((noreturn)) task_train_state();

int NotifyTrainSpeed(int trainstatetid, TrainData data);
int NotifyNewTrain(int trainstatetid, NewTrain data);
int NotifySensorEvent(int trainstatetid, SensorEvent data);
int NotifyReservation(int trainstatetid, int data);
int NotifyRandomRoute(int trainstatetid, int train);

int GetTrainSpeed(int trainstatetid, int train) __attribute__((warn_unused_result));
int GetActiveTrain(int trainstatetid, int train) __attribute__((warn_unused_result));
int NavigateTo(int trainstatetid, NavigateRequest nav_req);
TrackPosition GetTrainPosition(int trainstatetid, int train);

#endif //_TRAIN_STATE_H_
