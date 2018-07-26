#ifndef FEATURES_H
#define FEATURES_H

#define RESERVE_TRACK TRUE

#if RESERVE_TRACK
 #define TPR1 TERMINAL_PRINT_RESRV1
 #define TPR2 TERMINAL_PRINT_RESRV2
#else
 #define TPR1 TERMINAL_JUST_REPLY
 #define TPR2 TERMINAL_JUST_REPLY
#endif

#define ALLOW_REVERSE_START TRUE
#define ALLOW_REVERSE_ENROUTE TRUE

#define ALLOW_SHORTS TRUE

#define DEBUG_CLOCK FALSE

#endif
