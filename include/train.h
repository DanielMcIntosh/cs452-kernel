#ifndef TRAIN_H
#define TRAIN_H

#define NUM_SPEEDS 15

#define MAX_CONCURRENT_TRAINS 5
#define NUM_TRAINS 80

typedef enum direction{
    FORWARD,
    BACKWARD
} Direction;

typedef struct trainposition{
    int sensor;
    int distance;
} TrainPosition;

typedef struct train{
    int speed;
    Direction direction;
    int velocity[NUM_SPEEDS];
    int next_sensor;
    int next_sensor_predict_time;
    int last_sensor;
    int last_sensor_time; 
    int last_sensor_dist;
} Train;

#endif
