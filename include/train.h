#ifndef TRAIN_H
#define TRAIN_H

#define NUM_SPEEDS 15

#define MAX_CONCURRENT_TRAINS 5
#define NUM_TRAINS 80

//train body is ~62cm long, round up to 65 just in case
#define TRAIN_LENGTH 650

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
    int num;
    Direction direction;
    int velocity[NUM_SPEEDS];
    int stopping_distance[NUM_SPEEDS];
    int short_delay_coeffs[3];
    int next_sensor;
    int next_sensor_predict_time;
    int last_sensor;
    int last_sensor_time; 
    int last_sensor_dist;
} Train;

#define TRAIN_INIT {0, 0,  0, {0}, {0}, {0, 0, 0}, -1, 0, -1, 0, 0}

#endif
