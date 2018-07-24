#include <track.h>
#include <track_data.h>
#include <track_state.h> 
#include <syscall.h>
#include <debug.h>
#include <minheap.h>
#include <features.h>

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
    ASSERT(*freeQ != NULL, "Cannot pop from empty free queue");
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

static inline void bfs_add_node(minheap_t *mh, BFSNode **freeQ, BFSNode **freeQTail, Route *route, int idx, int distance, int cnnum, const track_node *dest, Action a, int dist){
    BFSNode * straight = q_pop(freeQ, freeQTail);
    straight->current_node = dest;
    memcpy(&straight->r, route, sizeof(Route));
    if (a != ACTION_NONE) {
        straight->r.rcs[idx].swmr = SWCLAMP(cnnum);
        straight->r.rcs[idx].a = a;
        idx++;
    }
    straight->idx=idx;
    mh_add(mh, (unsigned long int) straight, distance + dist);
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
    if (ALLOW_REVERSE_START) {
        BFSNode * rv = q_pop(&freeQ, &freeQTail);
        rv->r = *r;
        rv->r.reverse = 1;
        rv->current_node = origin->reverse;
        rv->idx = 0;
        mh_add(&mh, (int) rv, rev_penalty);
    }
    bfsnodes[BFS_MH_SIZE-1].next = NULL;
    int k = 0;

    while (mh_remove_min(&mh, &entry) == 0){
        Pass();
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
            for (int i = idx; i < MAX_ROUTE_COMMAND; i++){
                route.rcs[i].swmr = 0;
                route.rcs[i].a = ACTION_NONE;
            }
            *r = route;
            return distance;
        } else if (TRACK_RESERVED(reservations, cn)) { // TODO allow trains to use their own reserved track
            continue;
        } 
        // continue the search
        //

        if (cn->type == NODE_BRANCH) {
            // Can go either direction on a branch
            bfs_add_node(&mh, &freeQ, &freeQTail, &route, idx, distance, cn->num, cn->edge[DIR_STRAIGHT].dest, ACTION_STRAIGHT, cn->edge[DIR_STRAIGHT].dist);
            bfs_add_node(&mh, &freeQ, &freeQTail, &route, idx, distance, cn->num, cn->edge[DIR_CURVED].dest, ACTION_CURVED, cn->edge[DIR_CURVED].dist);
        }
        if (cn->type == NODE_MERGE && ALLOW_REVERSE_ENROUTE) {
            // Can reverse after hitting a merge:
            const track_node *rev;
            if (likely(cn->num <= 18)) {
                rev = cn->reverse;
            }
            else {
                // if we're in the middle, our next node is not cn->reverse, but SW3_COMPLEMENT(cn->num)
                rev = track[MERGE_TO_NODE(SW3_COMPLEMENT(cn->num))].reverse;
            }
            bfs_add_node(&mh, &freeQ, &freeQTail, &route, idx, distance, cn->num, rev, ACTION_RV, rev_penalty);
        }
        if (cn->type == NODE_MERGE || cn->type == NODE_SENSOR || cn->type == NODE_ENTER) {
            // Can go straight on merges, sensors, and enters.
            bfs_add_node(&mh, &freeQ, &freeQTail, &route, idx, distance, cn->num, cn->edge[DIR_AHEAD].dest, ACTION_NONE, cn->edge[DIR_AHEAD].dist);
        }
    }

    return -1;
}

const track_node* rc_to_track_node(RouteCommand rc, const char * restrict sig) {
    switch (rc.a) {
        case (ACTION_CURVED):
        case (ACTION_STRAIGHT):
        {
            return &track[SWITCH_TO_NODE_NSC(rc.swmr)];
        }
        case (ACTION_RV):
        {
            return &track[MERGE_TO_NODE(rc.swmr)];
        }
        case (ACTION_NONE):
        {
            ASSERT(FALSE, "rc_to_track_node: action NONE @ %s", sig);
            return NULL;
        }
        default:
        {
            PANIC("Unhandled action type in rc_to_track_node: %d @ %s", rc.a, sig);
        }
    }
}

inline const track_edge *next_edge_on_route(const Route *route, int * restrict idx, const track_node *n,  bool* on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(n, sig);
    ASSERT(0 <= *idx && *idx < MAX_ROUTE_COMMAND, "invalid idx: %d @ %s", *idx, sig);
    switch (n->type) {
    case (NODE_BRANCH):
    {
        if(unlikely(route->rcs[*idx].a == ACTION_NONE)) {
            return NULL; // End of route
        }
        if (route->rcs[*idx].swmr != SWCLAMP(n->num)) {
            *on_route = FALSE;
            return NULL;
        }//, "Incorrect switch in path at idx %d: %d(%s), should be %d(%s) @ %s", *idx, route->rcs[*idx].swmr, track[SWITCH_TO_NODE(route->rcs[*idx].swmr)].name, SWCLAMP(n->num), n->name, sig);

        RouteCommand rc = route->rcs[(*idx)++];
        return &(n->edge[(rc.a == ACTION_STRAIGHT) ? DIR_STRAIGHT : DIR_CURVED]);
        break;
    }
    case (NODE_MERGE):
    {
        if (route->rcs[*idx].swmr == SWCLAMP(n->num) && route->rcs[*idx].a == ACTION_RV) {
            *idx+=1;
            // TODO BIG HACK
            //if (n->edge[DIR_AHEAD].dist < 300) {
            //    return n->edge[DIR_AHEAD].dest->edge[DIR_AHEAD].reverse;
            //}
            return n->edge[DIR_AHEAD].reverse;
        }
        FALLTHROUGH;
    }
    case (NODE_ENTER):
    case (NODE_SENSOR):
    //return a pointer to an edge with all 0 values in the case of exit
    case (NODE_EXIT):
    {
        return &(n->edge[DIR_AHEAD]);
    }
    default:
    {
        PANIC("in: next_edge_on_route - INVALID TRACK NODE TYPE: %d @ %s", n->type, sig);
    }
    }
}

static inline const track_node *next_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, node_type type, bool * on_route,  const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    const track_node *n = prev;
    *distance = 0;
    const track_edge *e;
    do {
        e = next_edge_on_route(route, idx, n, on_route, sig);
        if (e == NULL) return NULL;
        *distance += e->dist;
        n = e->dest;
    } while (n != NULL && n->type != type);

    return n;
}

inline const track_node *next_sensor_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, bool * on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    return next_on_route(route, idx, prev, distance, NODE_SENSOR, on_route, sig);
}
inline const track_node *next_switch_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, bool * on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    return next_on_route(route, idx, prev, distance, NODE_BRANCH, on_route, sig);
}

//TODO distance is overwritten by successive calls to next_sensor_on_route, so it isn't actually the resulting distance
const track_node *nth_sensor_on_route(int n, const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, bool * on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    int cur_dist = 0;
    for (int i = 0; i < n && likely(prev != NULL); ++i) {
        prev = next_sensor_on_route(route, idx, prev, &cur_dist, on_route, sig);
        *distance += cur_dist;
    }
    return prev;
}

const track_node *forward_dist_on_route_no_extra(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, bool *on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    int cur_dist = 0;
    int idx_old = *idx;
    const track_edge *e;
    while (cur_dist < *distance && prev != NULL) {
        e = next_edge_on_route(route, idx, prev, on_route, sig);
        if (e == NULL) {
            *distance = cur_dist; // off route?

            return NULL;
        } else if (cur_dist + e->dist > *distance) {
            *idx = idx_old;
            break; // TODO cleaner way to do this?
        }
        cur_dist += e->dist;
        prev = e->dest;
        idx_old = *idx;
    }
    *distance -= cur_dist;
    return prev;
}

const track_node *forward_dist_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, bool *on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    int cur_dist = *distance;
    prev = forward_dist_on_route_no_extra(route, idx, prev, &cur_dist, on_route, sig);
    // cur_dist is the amount of distance remaining between prev and the goal distance.
    if (prev != NULL) {
        const track_edge *e = next_edge_on_route(route, idx, prev, on_route, sig);
        if (!on_route)
            return prev;
        *distance += (e->dist - cur_dist); // the amount added minus the amount missing
        prev = e->dest;
    } else {
        *distance -= cur_dist;
    }
    return prev;
}

int distance_to_on_route(const Route *route, int idx, const track_node *from, const track_node *to, bool *on_route, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(to, sig);
    ASSERT_VALID_TRACK_SIG(from, sig);
    const track_node *n = from;
    int distance = 0;
    const track_edge *e;
    while (n != to && n != NULL && *on_route) {
        e = next_edge_on_route(route, &idx, n, on_route, sig);
        ASSERT(e != NULL, "Null edge on route: from %s, to %s, n: %s, idx: %d @ %s", from->name, to->name, n->name, idx, sig);
        distance += e->dist;
        n = e->dest;
    }
    ASSERT(n == to, "While Loop broken early");

    return (unlikely(n == NULL)) ? -1 : distance;
}

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
            ind = (SWCLAMP(n->num) - 1) + 80 + 22;
        }
        ind -= 64;
        mask->bits_high |= 0x1ULL << ind;
    }
}

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
            ind = (SWCLAMP(n->num) - 1) + 80 + 22;
        }
        ind -= 64;
        mask->bits_high &= ~(0x1ULL << ind);
    }
}

bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations) {
    ASSERT_VALID_TRACK(start);
    ASSERT_VALID_TRACK(end);

    Reservation mask = RESERVATION_INIT;
    add_to_mask(end, &mask);

    const track_node *n = start;
    bool on_route = TRUE;
    while (n != end && n != NULL && on_route) {
        n = next_edge_on_route(route, &idx, n, &on_route, "reserve_track")->dest;

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
