#ifndef __RESERVATIONS_H__
#define __RESERVATIONS_H__

#include "track_node.h"
#include "route.h"
#include "util.h"
#include "constants.h"

typedef struct blockage {
    //actually only need 124, but round up to a power of 8 (and conviniently 32)
    unsigned long long bits_low : 64;
    unsigned long long bits_high : 64;    
} __attribute__((packed)) Blockage;
#define BLOCKAGE_INIT {0, 0}

typedef struct reservation {
    //the extra +1 is for user reserved track from the 'drop' command
    Blockage blkges[MAX_CONCURRENT_TRAINS+1];
    Blockage total;
} __attribute__((packed)) Reservation;
#define RESERVATION_INIT {{}, BLOCKAGE_INIT}

typedef struct myreservation {
    Blockage *mine;
    Blockage *total;
} __attribute__((packed)) MyReservation;
#define RESERVATION_INIT {{}, BLOCKAGE_INIT}

#define TRACK_BLOCKED(blockages, cn) (blockages != NULL && ((0x1ULL << (TRACK_NODE_TO_INDEX(cn) % 64)) & ((TRACK_NODE_TO_INDEX(cn) < 64) ? blockages->bits_low : blockages->bits_high)))


//EXclusive of start, but INclusive of end. When start == end, reserves end (and therefore start)
__attribute__((warn_unused_result, nonnull (3, 4, 6)))
bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, const MyReservation * restrict my_reserv, const char * restrict sig);
//INclusive of start, but EXclusive of end. When start == end, doesn't free anything
__attribute__((nonnull (3, 4, 6)))
void free_track(const Route *route, int idx, const track_node *start, const track_node *end, const MyReservation * restrict my_reserv, Blockage * restrict result, const char *sig);

void my_reservation_to_blockage(Blockage * restrict blockages, const MyReservation * restrict my_reserv);
void reservation_to_my_reservation(MyReservation * restrict my_reserv, Reservation * restrict reservations, int active_train);
void reservation_to_blockage(Blockage * restrict blockages, const Reservation * restrict reservations, int active_train);

#endif //__RESERVATIONS_H__
