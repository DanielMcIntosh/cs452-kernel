#ifndef _TRACK_H_
#define _TRACK_H_

#include "track_node.h"
#include "track_data.h"
#include "route.h"
#include "reservations.h"
#include "switch.h"

#define SENSOR_TO_NODE(s) (s)
#define SWITCH_TO_NODE(s) (80 + 2 * (SWCLAMP(s) - 1))
#define SWITCH_TO_NODE_NSC(s) (80 + 2 * (s - 1))
#define MERGE_TO_NODE(m) (81 + 2 * (SWCLAMP(m) - 1))
#define MERGE_TO_NODE_NSC(m) (81 + 2 * (m - 1))
#define ENTER_TO_NODE(n) (124 + 2 * (n))
#define EXIT_TO_NODE(n) (125 + 2 * (n))

// TODO is there a nicer way to do this?
#define TRACK_NODE_TO_INDEX(n) \
    (n->type == NODE_SENSOR ? SENSOR_TO_NODE(n->num)  : \
     (n->type == NODE_BRANCH ? SWITCH_TO_NODE(n->num) : \
      (n->type == NODE_MERGE ? MERGE_TO_NODE(n->num) : \
       (n->type == NODE_ENTER ? ENTER_TO_NODE(n->num) : \
        (n->type == NODE_EXIT ? EXIT_TO_NODE(n->num) : -1)))))


extern track_node track[TRACK_MAX];
#define ASSERT_VALID_TRACK(n) ASSERT(track <= (n) && (n) <= track+TRACK_MAX, "INVALID "S(n)": %d, [%d -> %d]", (int)(n), (int)track, (int)track+TRACK_MAX)
#define ASSERT_VALID_TRACK_SIG(n, sig) ASSERT(track <= (n) && (n) <= track+TRACK_MAX, "INVALID "S(n)": %d, [%d -> %d] @ %s", (int)(n), (int)track, (int)track+TRACK_MAX, (sig))

#define SWITCH_NONE 31 
// SWITCH_NONE is the maximum 5 bit unsigned int

void init_track();
 __attribute__((nonnull (5, 6, 7)))
int find_path_between_nodes(const Blockage * restrict blockages, int min_dist, int max_dist, int rev_penalty, const track_node *origin, const track_node *dest, Route * restrict r);

const track_node *rc_to_track_node(RouteCommand rc, const char * restrict sig) __attribute__((warn_unused_result));
const track_edge *next_edge_on_route(const Route *route, int * restrict ind, const track_node *n, const char * restrict sig) __attribute__((nonnull));

const track_node *next_sensor_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) __attribute__((nonnull));
const track_node *next_switch_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) __attribute__((nonnull));
const track_node *nth_sensor_on_route(int n, const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) __attribute__((nonnull));

const track_node *forward_dist_on_route_no_extra(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) __attribute__((nonnull));
const track_node *forward_dist_on_route(const Route *route, int * restrict idx, const track_node *prev, int * restrict distance, const char * restrict sig) __attribute__((nonnull));

int distance_to_on_route(const Route *route, int idx, const track_node *from, const track_node *to, const char * restrict sig) __attribute__((warn_unused_result, nonnull));

int get_dist_to_nxt_sensor(const Route *route, int idx, const track_node *cur_sensor, const char * restrict sig) __attribute__((warn_unused_result, nonnull));

#endif //_TRACK_H_

