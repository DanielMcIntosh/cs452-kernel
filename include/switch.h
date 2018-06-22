#ifndef SWITCH_H
#define SWITCH_H

typedef enum switchstate{
    SWITCH_UNKNOWN,
    SWITCH_STRAIGHT,
    SWITCH_CURVED
} SwitchState;

typedef struct swtch{
    SwitchState state;
} Switch;

#endif


