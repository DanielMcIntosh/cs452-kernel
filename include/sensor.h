#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_PAIR_TO_SENSOR(r, s) (16 * (r) + (s))
#define SENSOR_GET_RADIX(s) ((s) / 16)
#define SENSOR_GET_NUM(s) ((s) % 16)

#define SENSOR_TERMINAL_BUFFER_SIZE 21

typedef enum sensorstate{
    SENSOR_OFF,
    SENSOR_ON
} SensorState;

typedef struct sensor {
    SensorState state;
} Sensor;

void task_sensor_server();

#endif
