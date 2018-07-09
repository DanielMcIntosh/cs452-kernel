#include "util.h"
#include "switch.h"

#ifndef _TRAIN_STATE_H_
#define _TRAIN_STATE_H_

#define NAME_TRAIN_STATE "trn_st"

#define VELOCITY_PRECISION 10000
#define CAL_ITERATIONS 8
#define BASE_STOP_DIST_ADJUSTMENT 70

#define TRAIN_STATE_TERMINAL_BUFFER_SIZE 1000

typedef struct position {
    const int object : 16;
    const int distance_past : 16;
} TrackPosition;

typedef struct reservation {
    //actually only need 124, but round up to a power of 8 (and conviniently 32)
    long long bits_low : 64;
    long long bits_high : 64;    
} __attribute__((packed)) Reservation;

#define RESERVATION_INIT {0, 0}

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

    NOTIFY_SENSOR_EVENT,
    NOTIFY_TRAIN_SPEED,
    NOTIFY_TRAIN_DIRECTION,
    NOTIFY_CAL,
    NOTIFY_NEW_TRAIN,
    NOTIFY_RESERVATION,
    TRAIN_STATE_NOTIFY_TERMINAL_COURIER,

    NUM_TRAIN_STATE_REQUESTS
} TrainStateRequest;

void task_train_state();

int NotifyTrainSpeed(int trainstatetid, TrainData data);
int NotifyCalibrationResult(int trainstatetid, CalData data);
int NotifyNewTrain(int trainstatetid, NewTrain data);
int NotifySensorEvent(int trainstatetid, SensorEvent data);
int NotifyReservation(int trainstatetid, int data);

int GetTrainSpeed(int trainstatetid, int train);
int GetActiveTrain(int trainstatetid, int train);
int NavigateTo(int trainstatetid, NavigateRequest nav_req);

#endif //_TRAIN_STATE_H_
