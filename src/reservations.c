#include "reservations.h"
#include "switch.h"
#include "track.h"
#include "util.h"
#include "debug.h"
#include "syscall.h"

static inline void add_to_mask(const track_node *n, Reservation * restrict mask) {
    int ind = TRACK_NODE_TO_INDEX(n);
    if (ind < 64) {
        mask->bits_low |= 0x1ULL << ind;
    }
    else {
        if (n->type == NODE_BRANCH) {
            ind = SWCLAMP(n->num) - 1 + 80;
        }
        else if (n->type == NODE_MERGE) {
            ind = (SWCLAMP(n->num) - 1) + 80 + NUM_SWITCHES;
        }
        ind -= 64;
        ASSERT(ind >= 0, "can't have a negative shift! ind = %d, n = %s", ind, n->name);
        mask->bits_high |= 0x1ULL << ind;
    }
}

/*
static inline void remove_from_mask(const track_node *n, Reservation * restrict mask) {
    int ind = TRACK_NODE_TO_INDEX(n);
    if (ind < 64) {
        mask->bits_low &= ~(0x1ULL << ind);
    }
    else {
        if (n->type == NODE_BRANCH) {
            ind = SWCLAMP(n->num) - 1 + 80;
        }
        else if (n->type == NODE_MERGE) {
            ind = (SWCLAMP(n->num) - 1) + 80 + NUM_SWITCHES;
        }
        ind -= 64;
        mask->bits_high &= ~(0x1ULL << ind);
    }
}
//*/

//EXclusive of start, but INclusive of end
bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(start, sig);
    //in theory should handle end == null just fine by reserving to the switch after the end of the route, but put this here anyways
    ASSERT_VALID_TRACK_SIG(end, sig);

    Reservation mask = RESERVATION_INIT;
    add_to_mask(end, &mask);

    const track_node *n = start;
    const track_edge *e;
    while (n != end && n != NULL) {
        e = next_edge_on_route(route, &idx, n, sig);
        if (e == NULL) {
            return FALSE;
        }
        ASSERT(e->dest != NULL, "about to add NULL to mask! start = %s, end = %s, n = %s, @ %s", start->name, end->name, n->name, sig)
        n = e->dest;

        add_to_mask(n, &mask);
    }
    ASSERT(n == end, "While Loop broken early");

    if ((reservations->bits_low & mask.bits_low) || (reservations->bits_high & mask.bits_high)) {
        return FALSE;
    }

    reservations->bits_low |= mask.bits_low;
    reservations->bits_high |= mask.bits_high;

    return TRUE;
}

//INclusive of start, but EXclusive of end
void free_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations, const char *sig) {
    ASSERT_VALID_TRACK(start);
    //in theory should handle end == null just fine by freeing to the switch after the end of the route, but put this here anyways
    ASSERT_VALID_TRACK(end);

    Reservation mask = RESERVATION_INIT;
    const track_node *n = start;
    const track_edge *e;
    while (n != end && n != NULL) {
        add_to_mask(n, &mask);
        e = next_edge_on_route(route, &idx, n, sig);
        ASSERT(e != NULL, "tried to free track past end of given route. start = %s, end = %s, n = %s, idx = %d", start->name, end->name, n->name, idx);
        n = e->dest;
    }
    ASSERT(n == end, "While Loop broken early");

    /*
    ASSERT(((reservations->bits_low  & mask.bits_low ) == mask.bits_low ) &&
           ((reservations->bits_high & mask.bits_high) == mask.bits_high), "Tried to free unreserved track!"
            " mask_low = %d %d, mask_high = %d %d, resrv_low = %d %d, resrv_high = %d %d",
            (int)(mask.bits_low           & 0xFFFFFFFF), (int)((mask.bits_low  >> 32)          & 0xFFFFFFFF),
            (int)(mask.bits_high          & 0xFFFFFFFF), (int)((mask.bits_high >> 32)          & 0xFFFFFFFF),
            (int)(reservations->bits_low  & 0xFFFFFFFF), (int)((reservations->bits_low  >> 32) & 0xFFFFFFFF),
            (int)(reservations->bits_high & 0xFFFFFFFF), (int)((reservations->bits_high >> 32) & 0xFFFFFFFF));
    //*/

    reservations->bits_low &= ~(mask.bits_low);
    reservations->bits_high &= ~(mask.bits_high);
}

//TODO
void reservation_to_blockage(Blockage * restrict blockages, const Reservation * restrict reservations) {
    memcpy(blockages, reservations, sizeof(*reservations));
}
