#ifndef __ROUTE_H__
#define __ROUTE_H__

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

typedef struct route {
    RouteCommand rcs[MAX_ROUTE_COMMAND];
    unsigned int reverse: 1; int packing: 7;
} Route;
#define ROUTE_INIT {{{0, 0, 0}}, 0, 0}

#endif //__ROUTE_H__
