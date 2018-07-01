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
    int last_sensor;
    int last_sensor_time; // TODO when we do sensor attribution
    int last_sensor_dist;
} Train;

void predict_train_position(int train, TrainPosition * position); // TODO

#endif
