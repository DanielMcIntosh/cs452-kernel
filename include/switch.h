#include "util.h"

#ifndef SWITCH_H
#define SWITCH_H

#define NUM_SWITCHES 22

#define STATE_TO_CHAR(s) ((s) == SWITCH_UNKNOWN ? '?' : ((s) == SWITCH_STRAIGHT ? 'S' : 'C'))
#define INV_STATE_TO_CHAR(s) ((s) == SWITCH_UNKNOWN ? '?' : ((s) == SWITCH_STRAIGHT ? 'C' : 'S'))
#define CHAR_TO_SW_STATE(c) ((c) == '?' ? SWITCH_UNKNOWN : ((c) == 'S' ? SWITCH_STRAIGHT : SWITCH_CURVED))

#define STATE_TO_DIR(s) ((s) == SWITCH_STRAIGHT ? DIR_STRAIGHT : DIR_CURVED)

#define SWCLAMP(c) ((c) > 32 ? (c) - 134 : (c))
#define SWUNCLAMP(c) ((c) > 32 ? (c) + 134 : (c))
#define SW3_COMPLEMENT(c) ((((c) - 1) ^ 1) + 1)

typedef enum switchstate{
    SWITCH_UNKNOWN = 0,
    SWITCH_STRAIGHT,
    SWITCH_CURVED
} SwitchState;

typedef struct swtch{
    bool set_state : 16;
    SwitchState state;
} Switch;

#endif


