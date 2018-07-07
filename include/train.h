#ifndef TRAIN_H
#define TRAIN_H

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
    int velocity;
    int next_sensor;
    int next_sensor_predict_time;
    int last_sensor;
    int last_sensor_time; 
    int last_sensor_dist;
} Train;

#endif
