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
    mh_add(&mh, (int) rv, rev_penalty);

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
        } else if (cn == dest) {
            PANIC("YIKES %d | %d | %d | %d | %d | %d", distance, route.reverse, min_dist, rev_penalty, k, (!route.reverse ? min_dist : min_dist + rev_penalty));
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
            memcpy(&ahead->r, &route, sizeof(Route));
            mh_add(&mh, (unsigned long int) ahead, distance + cn->edge[DIR_AHEAD].dist);
            ASSERT(distance + cn->edge[DIR_AHEAD].dist > 0, "fuck");
        }
    }

    return -1;
}


