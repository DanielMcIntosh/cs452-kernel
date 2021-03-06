#ifndef COMMAND_H
#define COMMAND_H

#define NAME_COMMANDSERVER "CMDS"
// 2 arguments for each of the 22 switches + 2 extra as buffer space
#define SWITCHQ_BUF_SIZE (2 *(22 + 2))
#define COMMAND_TERMINAL_BUFFER_SIZE 50

typedef enum cmdtype{
    // PARSED COMMANDS
    COMMAND_GO,
    COMMAND_TR,
    COMMAND_RV,
    COMMAND_SW,
    COMMAND_QUIT,
    COMMAND_INV,
    COMMAND_ROUTE,
    COMMAND_MOVE,
    COMMAND_PARAM,
    COMMAND_ADD,
    COMMAND_RESERVE,
    COMMAND_FUNC,
    COMMAND_RANDOM_ROUTE,
    NO_COMMAND,
    // PARTIAL COMMANDS
    COMMAND_NOTIFY_RV_ACCEL,
    COMMAND_NOTIFY_RV_REVERSE,
    COMMAND_NOTIFY_SOLENOID_TIMER,
    COMMAND_NOTIFY_COURIER,
    COMMAND_SENSOR_REQUEST,
    COMMAND_NOTIFY_TERMINAL_COURIER,
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

void __attribute__((noreturn)) task_commandserver();
void __attribute__((noreturn)) task_switch_courier(int cmdtid, int term_tid);
#endif
