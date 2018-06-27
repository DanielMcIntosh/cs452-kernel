#ifndef COMMAND_H
#define COMMAND_H

#define NAME_COMMANDSERVER "CMDS"
// 2 arguments for each of the 22 switches + 2 extra as buffer space
#define SWITCHQ_BUF_SIZE (2 *(22 + 2))

/*
typedef enum direction{
    FORWARD,
    BACKWARD
} Direction;
*/

typedef enum cmdtype{
    // PARSED COMMANDS
    COMMAND_GO,
    COMMAND_TR,
    COMMAND_RV,
    COMMAND_SW,
    COMMAND_QUIT,
    COMMAND_INV,
    COMMAND_ROUTE,
    COMMAND_CAL,
    COMMAND_MOVE,
    COMMAND_PARAM,
    NO_COMMAND,
    // PARTIAL COMMANDS
    COMMAND_NOTIFY_RV_ACCEL,
    COMMAND_NOTIFY_RV_REVERSE,
    COMMAND_NOTIFY_SOLENOID_TIMER,
    COMMAND_NOTIFY_COURIER,
    COMMAND_SENSOR_REQUEST,
    INVALID_COMMAND
} CommandType;

typedef struct command{
    CommandType type;
    int arg1;
    union {
        int arg2: 32;
        struct {
            int smallarg1: 16;
            int smallarg2: 16;
        } __attribute__((packed));
    };
} Command;

int SendCommand(int servertid, Command c);

void task_commandserver();
void task_switch_courier(int cmdtid);
#endif
