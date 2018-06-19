#ifndef SENSOR_H
#define SENSOR_H

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
