#include "track.h"
#include "track_data.h"
#include "track_state.h"
#include "syscall.h"
#include "debug.h"
#include "minheap.h"

track_node track[TRACK_MAX];

//called from FUT
void init_track() {
    // get track:
    *((int *) 0x8001004c) &= ~(7); // reading the current track off the MAC address (which apparently works? - thnx jennifer)
    if (*((unsigned char *) 0x80010055) == 0xc5) {
        init_tracka(track);
        StoreValue(VALUE_TRACK_NAME, 'A');
    } else {
        init_trackb(track);
        StoreValue(VALUE_TRACK_NAME, 'B');
    }
}


typedef struct bfsnode {
    Route r;
    unsigned int idx: 8;
    const track_node *current_node;
    struct bfsnode *next;
} BFSNode;

static BFSNode* q_pop(BFSNode** freeQ, BFSNode** freeQTail){
    ASSERT(freeQ != NULL, "Cannot pop from empty free queue");
    BFSNode *ret = *freeQ;
    *freeQ = (*freeQ)->next;
    if (*freeQ == NULL)
        *freeQTail = NULL;

    ret->next = NULL;
    return ret;
}

static void q_add(BFSNode** freeQ, BFSNode** freeQTail, BFSNode *node){
    if (*freeQTail == NULL) {
        *freeQ = node;
        *freeQTail = node;
        return;
    }

    (*freeQTail)->next = node;
    *freeQTail = node;
}

int find_path_between_nodes(const Reservation * restrict reservations, int min_dist, int rev_penalty, const track_node *origin, const track_node *dest, Route * restrict r) {
    entry_t mh_array[BFS_MH_SIZE];
    minheap_t mh = {mh_array, 0, BFS_MH_SIZE};
    entry_t entry;
    // IDEA: minheap contains pointers to some struct. That struct contains a TrackPath and some other data I guess? We allocate those structs in a big array on the stack here.
    // Keep a free queue of those structs so we can free them whenever we drop a node?
    BFSNode bfsnodes[BFS_MH_SIZE] = {{{{{0, 0, 0}}, 0, 0}, 0, 0, 0}};

    BFSNode* freeQ = &bfsnodes[0];
    BFSNode* freeQTail = &bfsnodes[BFS_MH_SIZE-1];

    for (int i = 0; i < BFS_MH_SIZE; i++) {
        bfsnodes[i].next = (i < BFS_MH_SIZE - 1 ? &(bfsnodes[i+1]) : NULL);
    }

    BFSNode * fw = q_pop(&freeQ, &freeQTail);
    fw->r = *r;
    fw->current_node = origin;
    fw->idx = 0;
    mh_add(&mh, (int) fw, 0);
    BFSNode * rv = q_pop(&freeQ, &freeQTail);
    rv->r = *r;
    rv->r.reverse = 1;
    rv->current_node = origin->reverse;
    rv->idx = 0;
    //mh_add(&mh, (int) rv, rev_penalty);

    bfsnodes[BFS_MH_SIZE-1].next = NULL;
    int k = 0;

    while (mh_remove_min(&mh, &entry) == 0){
        ASSERT(k++ <= 10000, "probably an infinite loop");
        BFSNode *bn = (BFSNode*) entry.item;
        int distance = entry.value;
        Route route = bn->r;
        int idx = bn->idx;
        ASSERT(idx < MAX_ROUTE_COMMAND, "Too many route commands in a route from %d to %d (%d, %d)", origin->num, dest->num, k, rev_penalty);
        //if (idx >= MAX_ROUTE_COMMAND) continue;
        
        const track_node *cn = bn->current_node;
        q_add(&freeQ, &freeQTail, bn);
        if (unlikely(cn == dest) && distance > (!route.reverse ? min_dist : (min_dist + rev_penalty))) { // found shortest path > min_dist
            for (int i = idx+1; i < MAX_ROUTE_COMMAND; i++){
                route.rcs[i].swmr = 0;
                route.rcs[i].a = ACTION_NONE;
            }
            *r = route;
            return distance;
        } else if ((0x1ULL << (TRACK_NODE_TO_INDEX(cn) % 64)) & ((TRACK_NODE_TO_INDEX(cn) < 64) ? reservations->bits_low : reservations->bits_high)) { // TODO allow trains to use their own reserved track
            continue;
        } 
        // continue the search

        if (cn->type == NODE_BRANCH) {
            // Can go either direction on a branch
            BFSNode * straight = q_pop(&freeQ, &freeQTail);
            straight->current_node = cn->edge[DIR_STRAIGHT].dest;
            memcpy(&straight->r, &route, sizeof(Route));
            straight->r.rcs[idx].swmr = SWCLAMP(cn->num);
            straight->r.rcs[idx].a = ACTION_STRAIGHT;
            straight->idx=idx+1;
            mh_add(&mh, (unsigned long int) straight, distance + cn->edge[DIR_STRAIGHT].dist);

            BFSNode * curved = q_pop(&freeQ, &freeQTail);
            curved->current_node = cn->edge[DIR_CURVED].dest;
            memcpy(&curved->r, &route, sizeof(Route));
            curved->r.rcs[idx].swmr = SWCLAMP(cn->num);
            curved->r.rcs[idx].a = ACTION_CURVED;
            curved->idx=idx+1;
            mh_add(&mh, (unsigned long int) curved, distance + cn->edge[DIR_CURVED].dist);
        }
        if (cn->type == NODE_MERGE && FALSE) {
            // Can reverse after hitting a merge:
            BFSNode * reverse = q_pop(&freeQ, &freeQTail);
            reverse->current_node = cn->reverse;
            memcpy(&reverse->r, &route, sizeof(Route));
            reverse->r.rcs[idx].swmr = SWCLAMP(cn->num);
            reverse->r.rcs[idx].a = ACTION_RV;
            reverse->idx=idx+1;
            mh_add(&mh, (unsigned long int) reverse, distance + rev_penalty);
        }
        if (cn->type == NODE_MERGE || cn->type == NODE_SENSOR || cn->type == NODE_ENTER) {
            // Can go straight on merges, sensors, and enters.
            BFSNode * ahead = q_pop(&freeQ, &freeQTail);
            ahead->current_node = cn->edge[DIR_AHEAD].dest;
            ahead->idx = idx;
            memcpy(&ahead->r, &route, sizeof(Route));
            mh_add(&mh, (unsigned long int) ahead, distance + cn->edge[DIR_AHEAD].dist);
            ASSERT(distance + cn->edge[DIR_AHEAD].dist > 0, "fuck");
        }
    }

    return -1;
}

const track_node* rc_to_track_node(RouteCommand rc) {
    switch (rc.a) {
        case (ACTION_CURVED):
        case (ACTION_STRAIGHT):
        {
            return &track[SWITCH_TO_NODE(rc.swmr)];
        }
        case (ACTION_RV):
        {
            return &track[MERGE_TO_NODE(rc.swmr)];
        }
        case (ACTION_NONE):
        {
            PANIC("NO");
            return NULL;
        }
        default:
        {
            PANIC("Unhandled action type in rc_to_track_node: %d", rc.a);
        }
    }
}

inline const track_edge *next_edge_on_route(const Route *route, int * restrict idx, const track_node *n) {
    ASSERT(track <= n && n <= track+TRACK_MAX, "invalid n: %d, [%d -> %d]", n, track, track+TRACK_MAX);
    switch (n->type) {
    case (NODE_BRANCH):
    {
        ASSERT(route->rcs[*idx].swmr == SWCLAMP(n->num), "Incorrect switch in path at idx %d: %d(%s), should be %d(%s).", *idx, route->rcs[*idx].swmr, track[SWITCH_TO_NODE(route->rcs[*idx].swmr)].name, SWCLAMP(n->num), n->name);
        ASSERT(route->rcs[*idx].a != ACTION_NONE, "Action None on route (idx: %d, node: %s)", *idx, n->name);
        RouteCommand rc = route->rcs[(*idx)++];
        return &(n->edge[(rc.a == ACTION_STRAIGHT) ? DIR_STRAIGHT : DIR_CURVED]);
        break;
    }
    case (NODE_ENTER):
    case (NODE_MERGE):
    case (NODE_SENSOR):
    //return a pointer to an edge with all 0 values in the case of exit
    case (NODE_EXIT):
    {
         // TODO proper next sensor predicting w/ reversing
        return &(n->edge[DIR_AHEAD]);
    }
    default:
    {
        PANIC("in: distance_to_on_route - INVALID TRACK NODE TYPE: %d", n->type);
    }
    }
}

static inline const track_node *next_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, node_type type) {
    const track_node *n = prev;
    *distance = 0;
    const track_edge *e;
    do {
        e = next_edge_on_route(route, idx, n);

        *distance += e->dist;
        n = e->dest;
    } while (n != NULL && n->type != type);

    return n;
}

inline const track_node *next_sensor_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance) {
    ASSERT(track <= prev && prev <= track+TRACK_MAX, "invalid prev: %d, [%d -> %d]", prev, track, track+TRACK_MAX);
    return next_on_route(route, idx, prev, distance, NODE_SENSOR);
}
inline const track_node *next_switch_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance) {
    ASSERT(track <= prev && prev <= track+TRACK_MAX, "invalid prev: %d, [%d -> %d]", prev, track, track+TRACK_MAX);
    return next_on_route(route, idx, prev, distance, NODE_BRANCH);
}

//TODO distance is overwritten by successive calls to next_sensor_on_route, so it isn't actually the resulting distance
const track_node *nth_sensor_on_route(int n, const Route *route, int * restrict idx, const track_node *prev, int * restrict distance) {
    ASSERT(track <= prev && prev <= track+TRACK_MAX, "invalid prev: %d, [%d -> %d]", prev, track, track+TRACK_MAX);
    int cur_dist = 0;
    for (int i = 0; i < n && likely(prev != NULL); ++i) {
        prev = next_sensor_on_route(route, idx, prev, &cur_dist);
        *distance += cur_dist;
    }
    return prev;
}

const track_node *forward_dist_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance) {
    ASSERT(track <= prev && prev <= track+TRACK_MAX, "INVALID prev: %d, [%d -> %d]", prev, track, track+TRACK_MAX);
    int cur_dist = 0;
    const track_edge *e;
    while (cur_dist < *distance && prev != NULL) {
        e = next_edge_on_route(route, idx, prev);

        cur_dist += e->dist;
        prev = e->dest;
    }
    *distance = cur_dist;
    return prev;
}

int distance_to_on_route(const Route *route, int idx, const track_node *from, const track_node *to) {
    ASSERT(track <= to && to <= track+TRACK_MAX, "INVALID to: %d, [%d -> %d]", to, track, track+TRACK_MAX);
    ASSERT(track <= from && from <= track+TRACK_MAX, "INVALID from: %d, [%d -> %d]", from, track, track+TRACK_MAX);
    const track_node *n = from;
    int distance = 0;
    const track_edge *e;
    while (n != to && n != NULL) {
        e = next_edge_on_route(route, &idx, n);

        distance += e->dist;
        n = e->dest;
    }
    ASSERT(n == to, "While Loop broken early");

    return (unlikely(n == NULL)) ? -1 : distance;
}

bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations) {
    Reservation mask = RESERVATION_INIT;
    const track_node *n = start;
    ASSERT(track <= n && n <= track+TRACK_MAX, "INVALID n: %d, [%d -> %d]", n, track, track+TRACK_MAX);
    while (n != end && n != NULL) {
        int ind = TRACK_NODE_TO_INDEX(n);
        if (ind < 64) {
            mask.bits_low |= 0x1ULL << ind;
        }
        else {
            mask.bits_high |= 0x1ULL << ind;
        }

        n = next_edge_on_route(route, &idx, n)->dest;
    }
    ASSERT(n == end, "While Loop broken early");

    if ((reservations->bits_low & mask.bits_low) || (reservations->bits_low & mask.bits_low)) {
        return FALSE;
    }

    reservations->bits_low |= mask.bits_low;
    reservations->bits_high |= mask.bits_high;

    return TRUE;
}

