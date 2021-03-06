#include "reservations.h"
#include "switch.h"
#include "track.h"
#include "util.h"
#include "debug.h"
#include "syscall.h"

#define CHECK_OWNERSHIP(total, mine) \
    ASSERT(((total)->bits_low & (mine)->bits_low) == (mine)->bits_low, "total reservations doesn't contain my reservations! mine_low = 0x%x %x, total_low = 0x%x %x", \
                                                                        (mine) ->word2, (mine) ->word1, \
                                                                        (total)->word2, (total)->word1); \
    ASSERT(((total)->bits_high & (mine)->bits_high) == (mine)->bits_high, "total reservations doesn't contain my reservations! mine_high = 0x%x %x, total_high = 0x%x %x", \
                                                                        (mine) ->word2, (mine) ->word1, \
                                                                        (total)->word2, (total)->word1);

static inline int __attribute__((warn_unused_result, nonnull)) node_to_blkge_ind(const track_node *n) {
    int ind = TRACK_NODE_TO_INDEX(n);
    if (n->type == NODE_BRANCH) {
        ind = SWCLAMP(n->num) - 1 + 80;
    }
    else if (n->type == NODE_MERGE) {
        ind = (SWCLAMP(n->num) - 1) + 80 + NUM_SWITCHES;
    }
    return ind;
}

bool is_track_blocked(const Blockage * restrict blkges, const track_node *node) {
    int ind = node_to_blkge_ind(node);
    return (0x1ULL << (ind % 64)) & ((ind < 64) ? blkges->bits_low : blkges->bits_high);
}

static inline void add_to_mask(const track_node *n, Blockage *mask) {
    int ind = node_to_blkge_ind(n);
    int ind_rv = node_to_blkge_ind(n->reverse);
    if (ind < 64) {
        mask->bits_low |= 0x1ULL << ind;
        mask->bits_low |= 0x1ULL << ind_rv;
    }
    else {
        ind -= 64;
        ind_rv -= 64;
        ASSERT(0 <= ind && ind < 64, "can't have a negative shift! ind = %d, n = %s", ind, n->name);
        ASSERT(0 <= ind_rv && ind_rv < 64, "can't have a negative shift! ind_rv = %d, n = %s", ind_rv, n->name);
        mask->bits_high |= 0x1ULL << ind;
        mask->bits_high |= 0x1ULL << ind_rv;
    }
}

/*
static inline void remove_from_mask(const track_node *n, Blockage *mask) {
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

inline void reservation_to_my_reservation(MyReservation *my_reserv, Reservation *reservations, int active_train) {
    my_reserv->mine = &(reservations->blkges[active_train]);
    my_reserv->total = &(reservations->total);
}

inline void my_reservation_to_blockage(Blockage *blockages, const MyReservation *my_reserv) {
    Blockage *mine = my_reserv->mine;
    Blockage *total = my_reserv->total;
    CHECK_OWNERSHIP(total, mine);

    memcpy(blockages, total, sizeof(*blockages));
    blockages->bits_low ^= mine->bits_low;
    blockages->bits_high ^= mine->bits_high;
}

inline void reservation_to_blockage(Blockage *blockages, const Reservation *reservations, int active_train) {
    MyReservation tmp;
    //cast to remove const qualifier.
    //SAFE ONLY BECAUSE my_reservation_to_blockage doesn't write to tmp->mine or tmp->total
    reservation_to_my_reservation(&tmp, (Reservation *)reservations, active_train);
    my_reservation_to_blockage(blockages, &tmp);
}

static inline bool can_resrv(const MyReservation *my_reserv, Blockage *mask) {
    Blockage blockages;
    my_reservation_to_blockage(&blockages, my_reserv);

    return !((blockages.bits_low & mask->bits_low) || (blockages.bits_high & mask->bits_high));
}

static inline void set_resrv(const MyReservation *my_reserv, Blockage *mask) {
    my_reserv->mine->bits_low   |= mask->bits_low;
    my_reserv->mine->bits_high  |= mask->bits_high;

    my_reserv->total->bits_low  |= mask->bits_low;
    my_reserv->total->bits_high |= mask->bits_high;
}

static inline void unset_resrv(const MyReservation *my_reserv, Blockage *mask) {
    my_reserv->mine->bits_low   &= ~(mask->bits_low);
    my_reserv->mine->bits_high  &= ~(mask->bits_high);

    my_reserv->total->bits_low  &= ~(mask->bits_low);
    my_reserv->total->bits_high &= ~(mask->bits_high);
}

void free_all_reservations(const MyReservation *my_reserv) {
    my_reserv->total->bits_low ^= my_reserv->mine->bits_low;
    my_reserv->total->bits_high ^= my_reserv->mine->bits_high;

    my_reserv->mine->bits_low = 0;
    my_reserv->mine->bits_high = 0;
}

//EXclusive of start, but INclusive of end
const track_node *reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, const MyReservation *my_reserv, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(start, sig);
    //in theory should handle end == null just fine by reserving to the switch after the end of the route, but put this here anyways
    ASSERT_VALID_TRACK_SIG(end, sig);

    Blockage blockages = BLOCKAGE_INIT, mask = BLOCKAGE_INIT;
    my_reservation_to_blockage(&blockages, my_reserv);
    add_to_mask(end, &mask);

    const track_node *n = start;
    const track_edge *e;
    while (n != end && n != NULL) {
        //TODO handle off route problems
        bool on_route;
        e = next_edge_on_route(route, &idx, n, &on_route, sig);
        if (e == NULL || e->dest == NULL) {
            return NULL;
        }
        ASSERT(e->dest != NULL, "about to add NULL to mask! start = %s, end = %s, n = %s, @ %s", start->name, end->name, n->name, sig)

        if (is_track_blocked(&blockages, e->dest)) {
            break;
        }

        n = e->dest;
        add_to_mask(n, &mask);
    }

    ASSERT(can_resrv(my_reserv, &mask), "should be able to reserve track, as we checked in the loop");

    set_resrv(my_reserv, &mask);
    return n;
}

//INclusive of start, but EXclusive of end
void free_track(const Route *route, int idx, const track_node *start, const track_node *end, const MyReservation *my_reserv, Blockage *result, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(start, sig);
    //in theory should handle end == null just fine by freeing to the switch after the end of the route, but put this here anyways
    ASSERT_VALID_TRACK_SIG(end, sig);

    Blockage mask = BLOCKAGE_INIT;
    const track_node *n = start;
    const track_edge *e;
    bool on_route;
    while (n != end && n != NULL) {
        add_to_mask(n, &mask);
        //TODO handle off route problems
        e = next_edge_on_route(route, &idx, n, &on_route, sig);
        //ASSERT(e != NULL || !on_route, "tried to free track past end of given route. start = %s, end = %s, n = %s, idx = %d @ %s", start->name, end->name, n->name, idx, sig);
        //if we turn off asserts, just break out of the loop when this happens
        if (e == NULL) {
            break;
        }
        n = e->dest;
    }

    //this doesn't work because we don't own the last_sensor when a new train is added
    //CHECK_OWNERSHIP(my_reserv->total, &mask);

    unset_resrv(my_reserv, &mask);

    memcpy(result, &mask, sizeof(*result));
}

