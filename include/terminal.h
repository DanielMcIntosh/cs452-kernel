#ifndef TERMINAL_H
#define TERMINAL_H

#define L(x) x, sizeof(x)

#define CB_INPUT_BUF_SIZE 20
#define CB_TERMINAL_BUF_SIZE 500

#define STR_BACKSPACE "\033H \033H"
#define STR_NEWLINE "\r\n"

#define TERMINAL_INPUT_BASE_LINE 2
#define TERMINAL_INPUT_BASE_COL 12
#define TERMINAL_INPUT_MAX_LINE 26
#define TERMINAL_INPUT_MAX_COL 32

#define SENSOR_COL_BASE 34
#define SENSOR_LINE_BASE 2
#define SENSOR_LINE_MAX 24

#define NAME_TERMINAL "term"

typedef enum terminalrequest {
    TERMINAL_ECHO,
    TERMINAL_BACKSPACE,
    TERMINAL_NEWLINE,
    TERMINAL_SENSOR,
    TERMINAL_SWITCH,
    TERMINAL_TIME,
    TERMINAL_STACK_METRICS,
    TERMINAL_SENSOR_PREDICT,
    TERMINAL_VELOCITY_DEBUG,
    TERMINAL_DISTANCE_DEBUG,

    TERMINAL_NOTIFY_COURIER,
    NUM_TERMINAL_REQUESTS
} TerminalRequest;

int SendTerminalRequest(int terminaltid, TerminalRequest rq, int arg1, int arg2);

void task_terminal();
#endif
