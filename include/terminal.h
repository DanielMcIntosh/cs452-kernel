#ifndef TERMINAL_H
#define TERMINAL_H

#define L(x) x, sizeof(x)

#define CB_INPUT_BUF_SIZE 20
#define CB_TERMINAL_BUF_SIZE 2000

#define STR_BACKSPACE "\033H \033H"
#define STR_NEWLINE "\r\n"

#define TERMINAL_INPUT_BASE_LINE 17
#define TERMINAL_INPUT_BASE_COL 12
#define TERMINAL_INPUT_MAX_LINE 47
#define TERMINAL_INPUT_MAX_COL 32

#define TERMINAL_DEBUG_BASE_LINE 50
#define TERMINAL_DEBUG_MAX_LINE 55

#define SENSOR_COL_BASE 60
#define SENSOR_LINE_BASE 3 
#define SENSOR_LINE_MAX 16 

#define NAME_TERMINAL "term"

#define FLAG(s) "\033[2m"S(s)
#define STYLED_FLAG_STRING FLAG(I)FLAG(F)FLAG(C)FLAG(R)FLAG(M)FLAG(S)"\033[m"

typedef enum status_flag{
    STATUS_FLAG_INVALID         = (1 << 0),
    STATUS_FLAG_FINDING         = (1 << 1),
    STATUS_FLAG_CALIBRATING     = (1 << 2),
    STATUS_FLAG_REVERSING       = (1 << 3),
    STATUS_FLAG_MOVING          = (1 << 4),
    STATUS_FLAG_SENSOR_TIMEOUT  = (1 << 5),
    //NOT A REAL FLAG - used to determine when to stop iterating through flags
    STATUS_FLAG_END             = (1 << 6)
} StatusFlag;

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
    TERMINAL_FLAGS_SET,
    TERMINAL_FLAGS_UNSET,
    TERMINAL_ROUTE_DBG,
    TERMINAL_ROUTE_DBG2,

    TERMINAL_NOTIFY_COURIER,
    NUM_TERMINAL_REQUESTS
} TerminalRequest;

int SendTerminalRequest(int terminaltid, TerminalRequest rq, int arg1, int arg2);

void task_terminal();
#endif
