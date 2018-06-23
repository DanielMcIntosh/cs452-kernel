#ifndef UTIL_H
#define UTIL_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

void memswap(void * a, void * b, unsigned int sz);
void * memcpy(void * dest, const void* src, unsigned int sz);

#ifndef NULL
#define NULL 0
#endif

#ifndef MAX
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifndef MIN
#define MIN(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifndef CLAMP
#define CLAMP(X, Y, Z) (((X) > (Y)) ? (Y) : (((X) < (Z)) ? (Z) : (X)))
#endif

#ifndef MOVING_AVERAGE
#define MOVING_AVERAGE(N, O, A) ((A)*(N)/100 + (100-(A))*(O)/100)
#endif

#define STATE_TO_CHAR(s) ((s) == SWITCH_STRAIGHT ? 'S' : ((s) == SWITCH_UNKNOWN ? '?' : 'C'))
#define INV_STATE_TO_CHAR(s) ((s) == SWITCH_STRAIGHT ? 'C' : ((s) == SWITCH_UNKNOWN ? '?' : 'S'))
#define CHAR_TO_TRACK(c) ((c) == 'A' ? TRACK_A : TRACK_B)
#define SENSOR_TO_NODE(r, s) (16 * (r) + (s) - 1)
#define STATE_TO_DIR(s) ((s) == SWITCH_STRAIGHT ? DIR_STRAIGHT : DIR_CURVED)
#define SWCLAMP(c) ((c) > 18 ? (c) - 134 : (c))
#define SW3_COMPLEMENT(c) ((((c) - 1) ^ 1) + 1)

#endif
