#ifndef __RESERVATIONS_H__
#define __RESERVATIONS_H__

#include "track_node.h"
#include "route.h"
#include "util.h"

typedef struct reservation {
    //actually only need 124, but round up to a power of 8 (and conviniently 32)
    unsigned long long bits_low : 64;
    unsigned long long bits_high : 64;
} __attribute__((packed)) Reservation;
#define RESERVATION_INIT {0, 0}

typedef struct blockage {
    //actually only need 124, but round up to a power of 8 (and conviniently 32)
    unsigned long long bits_low : 64;
    unsigned long long bits_high : 64;    
} __attribute__((packed)) Blockage;
#define BLOCKAGE_INIT {0, 0}

#define TRACK_BLOCKED(blockages, cn) (blockages != NULL && ((0x1ULL << (TRACK_NODE_TO_INDEX(cn) % 64)) & ((TRACK_NODE_TO_INDEX(cn) < 64) ? blockages->bits_low : blockages->bits_high)))


//EXclusive of start, but INclusive of end. When start == end, reserves end (and therefore start)
bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations, const char * restrict sig) __attribute__((warn_unused_result, nonnull (3, 4, 5, 6)));
//INclusive of start, but EXclusive of end. When start == end, doesn't free anything
void free_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations, Blockage * restrict result, const char *sig) __attribute__((nonnull (3, 4, 5, 6)));

void reservation_to_blockage(Blockage * restrict blockages, const Reservation * restrict reservations);

#endif //__RESERVATIONS_H__
