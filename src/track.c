#include <track.h>
#include <track_data.h>
#include <track_state.h> 
#include <syscall.h>
#include <debug.h>
#include <minheap.h>
#include <features.h>
#include <reservations.h>
#include <constants.h>

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
    ASSERT(freeQ != NULL && *freeQ != NULL, "Cannot pop from empty free queue");
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

int find_path_between_nodes(const Blockage * restrict blockages, int min_dist, int max_dist, int rev_penalty, const track_node *origin, const track_node *dest, Route * restrict r) {
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
    mh_add(&mh, (unsigned long int) fw, 0);
    if (ALLOW_REVERSE_START) {
        BFSNode * rv = q_pop(&freeQ, &freeQTail);
        rv->r = *r;
        rv->r.reverse = 1;
        rv->current_node = origin->reverse;
        rv->idx = 0;
        mh_add(&mh, (unsigned long int) rv, rev_penalty);
    }
    bfsnodes[BFS_MH_SIZE-1].next = NULL;
    int k = 0;

    while (mh_remove_min(&mh, &entry) == 0){
        ASSERT(k++ <= 10000, "probably an infinite loop");
        BFSNode *bn = (BFSNode*) entry.item;
        int distance = entry.value;
        Route route = bn->r;
        int idx = bn->idx;
        if (idx >= MAX_ROUTE_COMMAND || distance > max_dist) {
            return INT_MAX;
        }
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
        } else if (TRACK_BLOCKED(blockages, cn)) { // TODO allow trains to use their own reserved track
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

inline const track_edge *next_edge_on_route(const Route *route, int * restrict idx, const track_node *n, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(n, sig);
    ASSERT(0 <= *idx && *idx < MAX_ROUTE_COMMAND, "invalid idx: %d @ %s", *idx, sig);
    switch (n->type) {
    case (NODE_BRANCH):
    {
        if(unlikely(route->rcs[*idx].a == ACTION_NONE)) {
            return NULL; // End of route
        }
        ASSERT(route->rcs[*idx].swmr == SWCLAMP(n->num), "Incorrect switch in path at idx %d: %d(%s), should be %d(%s) @ %s", *idx, route->rcs[*idx].swmr, track[SWITCH_TO_NODE(route->rcs[*idx].swmr)].name, SWCLAMP(n->num), n->name, sig);

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

static inline const track_node *next_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, node_type type, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    const track_node *n = prev;
    *distance = 0;
    const track_edge *e;
    do {
        e = next_edge_on_route(route, idx, n, sig);
        if (e == NULL) return NULL;
        *distance += e->dist;
        n = e->dest;
    } while (n != NULL && n->type != type);

    return n;
}

inline const track_node *next_sensor_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    return next_on_route(route, idx, prev, distance, NODE_SENSOR, sig);
}
inline const track_node *next_switch_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    return next_on_route(route, idx, prev, distance, NODE_BRANCH, sig);
}

//TODO distance is overwritten by successive calls to next_sensor_on_route, so it isn't actually the resulting distance
const track_node *nth_sensor_on_route(int n, const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    int cur_dist = 0;
    for (int i = 0; i < n && likely(prev != NULL); ++i) {
        prev = next_sensor_on_route(route, idx, prev, &cur_dist, sig);
        *distance += cur_dist;
    }
    return prev;
}

const track_node *forward_dist_on_route_no_extra(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    int cur_dist = 0;
    int idx_old = *idx;
    const track_edge *e;
    while (cur_dist < *distance && prev != NULL) {
        e = next_edge_on_route(route, idx, prev, sig);
        if (e == NULL) {
            *distance = cur_dist;
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

const track_node *forward_dist_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(prev, sig);
    int cur_dist = *distance;
    prev = forward_dist_on_route_no_extra(route, idx, prev, &cur_dist, sig);
    // cur_dist is the amount of distance remaining between prev and the goal distance.
    if (prev != NULL) {
        const track_edge *e = next_edge_on_route(route, idx, prev, sig);
        *distance += (e->dist - cur_dist); // the amount added minus the amount missing
        prev = e->dest;
    } else {
        *distance -= cur_dist;
    }
    return prev;
}

int distance_to_on_route(const Route *route, int idx, const track_node *from, const track_node *to, const char * restrict sig) {
    ASSERT_VALID_TRACK_SIG(from, sig);
    //in theory should handle to == null just fine by returning the distance to the switch after the end of the route, but put this here anyways
    ASSERT_VALID_TRACK_SIG(to, sig);
    const track_node *n = from;
    int distance = 0;
    const track_edge *e;
    while (n != to && n != NULL) {
        e = next_edge_on_route(route, &idx, n, sig);
        ASSERT(e != NULL, "Null edge on route: from %s, to %s, n: %s, idx: %d @ %s", from->name, to->name, n->name, idx, sig);
        distance += e->dist;
        n = e->dest;
    }
    ASSERT(n == to, "While Loop broken early");

    return (unlikely(n == NULL)) ? -1 : distance;
}

int get_dist_to_nxt_sensor(const Route *route, int idx, const track_node *cur_sensor, const char * restrict sig) {
    int res;
    next_sensor_on_route(route, &idx, cur_sensor, &res, sig);
    return res;
}
