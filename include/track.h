#ifndef _TRACK_H_
#define _TRACK_H_

#include "track_data.h"
#include "train_state.h"

extern track_node track[TRACK_MAX];

#define MAX_ROUTE_COMMAND 15
// 18 -> 126 bits which is the min dist to a power of 2 for a while I think - this is totally changeably but routes probably aren't more than 18 switches long?

typedef enum action {
    ACTION_NONE = 0, // default
    ACTION_STRAIGHT = 1,
    ACTION_CURVED = 2,
    ACTION_RV = 3
} __attribute__((packed)) Action; // packed -> use min possible type to store this enum

typedef struct routecommand{
    unsigned int swmr: 5; // 22 switches/merges -> need 5 bytes to represent, | 10 extra values
    Action a: 2; // 4 actions -> need 2 bits to represent 
    // note: reverse only happens after a merge - so ACTION_REVERSE implies swmr refers to a merge; otherwise swmr is a switch
    int packing: 1; // could let the compiler do it, but i feel better this way
}__attribute__((packed)) RouteCommand;

#define SWITCH_NONE 31 
// SWITCH_NONE is the maximum 5 bit unsigned int

typedef struct route {
    RouteCommand rcs[MAX_ROUTE_COMMAND];
    unsigned int reverse: 1; int packing: 7;
} Route;

#define ROUTE_INIT {{{0, 0, 0}}, 0, 0}

void init_track();
int find_path_between_nodes(const Reservation * restrict reservations, int min_dist, int rev_penalty, const track_node *origin, const track_node *dest, Route * restrict r);

const track_node *rc_to_track_node(RouteCommand rc, char * restrict sig);
const track_edge *next_edge_on_route(const Route *route, int * restrict ind, const track_node *n, char * restrict sig);

const track_node *next_sensor_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, char * restrict sig);
const track_node *next_switch_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, char * restrict sig);
const track_node *nth_sensor_on_route(int n, const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, char * restrict sig);

const track_node *forward_dist_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, char * restrict sig);

int distance_to_on_route(const Route *route, int idx, const track_node *from, const track_node *to, char * restrict sig);
bool reserve_track(const Route *route, int idx, const track_node *start, const track_node *end, Reservation * restrict reservations);

#endif //_TRACK_H_

