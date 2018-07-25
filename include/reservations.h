#ifndef __RESERVATIONS_H__
#define __RESERVATIONS_H__

#include "track_node.h"
#include "route.h"
#include "util.h"
#include "constants.h"

typedef struct blockage {
    union {
        struct {
            //actually only need 124, but round up to a power of 8 (and conviniently 32)
            unsigned long long bits_high : 64;
            unsigned long long bits_low : 64;
        } __attribute__((packed, scalar_storage_order("big-endian")));
        struct {
            //actually only need 124, but round up to a power of 8 (and conviniently 32)
            unsigned int word4 : 32;
            unsigned int word3 : 32;
            unsigned int word2 : 32;
            unsigned int word1 : 32;
        } __attribute__((packed, scalar_storage_order("big-endian")));
        struct {
            unsigned int padding : (128 - (2 * NUM_SWITCHES + 5 * SENSORS_PER_GROUP));
            unsigned int mr : NUM_SWITCHES;
            unsigned int br : NUM_SWITCHES;
            unsigned int e : SENSORS_PER_GROUP;
            unsigned int d : SENSORS_PER_GROUP;
            unsigned int c : SENSORS_PER_GROUP;
            unsigned int b : SENSORS_PER_GROUP;
            unsigned int a : SENSORS_PER_GROUP;
        } __attribute__((packed, scalar_storage_order("big-endian")));
    };
} __attribute__((packed, designated_init)) Blockage;
#define BLOCKAGE_INIT {.bits_high = 0, .bits_low = 0}

typedef struct reservation {
    //the extra +1 is for user reserved track from the 'drop' command
    Blockage blkges[MAX_CONCURRENT_TRAINS+1];
    Blockage total;
} __attribute__((packed, designated_init)) Reservation;
#define RESERVATION_INIT {.blkges = {}, .total = BLOCKAGE_INIT}

typedef struct myreservation {
    Blockage *mine;
    Blockage *total;
} __attribute__((packed, designated_init)) MyReservation;
#define MY_RESERVATION_INIT {NULL, NULL}

#define TRACK_BLOCKED(blockages, cn) (blockages != NULL && ((0x1ULL << (TRACK_NODE_TO_INDEX(cn) % 64)) & ((TRACK_NODE_TO_INDEX(cn) < 64) ? blockages->bits_low : blockages->bits_high)))


//EXclusive of start, but INclusive of end. When start == end, reserves end (and therefore start)
__attribute__((warn_unused_result, nonnull (3, 4, 6)))
bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, const MyReservation *my_reserv, const char * restrict sig);
//INclusive of start, but EXclusive of end. When start == end, doesn't free anything
__attribute__((nonnull (3, 4, 6)))
void free_track(const Route *route, int idx, const track_node *start, const track_node *end, const MyReservation *my_reserv, Blockage *result, const char * restrict sig);

void my_reservation_to_blockage(Blockage *blockages, const MyReservation *my_reserv);
void reservation_to_my_reservation(MyReservation *my_reserv, Reservation *reservations, int active_train);
void reservation_to_blockage(Blockage *blockages, const Reservation *reservations, int active_train);

#endif //__RESERVATIONS_H__
