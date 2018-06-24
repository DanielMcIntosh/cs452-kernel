#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_PAIR_TO_SENSOR(r, s) (16 * (r) + (s))
#define SENSOR_GET_RADIX(s) ((s) / 16)
#define SENSOR_GET_NUM(s) ((s) % 16)

typedef enum sensorstate{
    SENSOR_OFF,
    SENSOR_ON
} SensorState;

typedef struct sensor {
    char name[4]; // TODO use TrackData? 
    SensorState state;
    int lastTriggeredTime;
} Sensor;

void task_sensor_server();

#endif
