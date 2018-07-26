#include <train_state.h>
#include <track_state.h>
#include <switch.h>
#include <debug.h>
#include <syscall.h>
#include <track.h>
#include <position.h>
#include <name.h>

/* Notes:
 * This file uses the kinematics equations associated to motion with constant acceleration.
 * It could instead use the kinematics equations associated to motion with constant jerk
 *
 * TODO tomorrow: convert to floats, use an approximation of command delay
 */

static int position_calc_distance_with_max_velo(Position *p, int dt) {
    int distance = p->millis_off_last_node;
    if (p->state == PSTATE_STOPPED) return distance;
    if (p->v == p->v_max) {
        distance += p->v * dt / VELOCITY_PRECISION;
    } else {
        // so we aren't stopped, and we aren't at the maximum velocity. So, we must be accelerating or decelerating
        ASSERT(p->state != PSTATE_CONST_VELO && p->state != PSTATE_STOPPED, "Cannot be a constant velocity without v = v_max: state %d v %d vm %d a %d dt %d", p->state, p->v, p->v_max, p->a, dt);
        ASSERT(
                (p->state == PSTATE_ACCEL && p->a > 0) ||
                (p->state == PSTATE_DECEL && p->a < 0),
                "State does not match p->a sign: %d, %d", p->state, p->a);
        // first, we need to figure out how much of dt we need to get to max velo
        // vf = vi + at -> t =(vf - vi)/a
        // Note that when decelerating, vf = 0, but v_max not zero (it's the starting velocity) so we get -vi/a
        int accel_t = MIN(
                (((p->state == PSTATE_ACCEL ? p->v_max : 0) - p->v) * ACCELERATION_PRECISION) / (p->a * VELOCITY_PRECISION), 
                dt) ;
        int accel_d = p->v * accel_t / VELOCITY_PRECISION + (p->a * accel_t / ACCELERATION_PRECISION) * accel_t / 2;
        int vm = p->v + accel_t * p->a * VELOCITY_PRECISION / ACCELERATION_PRECISION;

        // then, do the rest at const velo
        int const_t = dt - accel_t;
        int const_d = vm * const_t / VELOCITY_PRECISION;
        distance += accel_d + const_d;
    }

    return distance;
}
void Position_HandleSensorHit(Position* p, const track_node *snsr, int time, int new_route_idx){
    p->last_known_node = snsr;
    p->millis_off_last_node = 0;
    if (p->state == PSTATE_ACCEL) {
        p->v = MIN(p->v + (time - p->last_update_time) * p->a * VELOCITY_PRECISION / ACCELERATION_PRECISION, p->v_max);
    } else if (p->state == PSTATE_DECEL) {
        p->v = MIN(p->v - (time - p->last_update_time) * p->a * VELOCITY_PRECISION / ACCELERATION_PRECISION, p->v_max);
    }
    p->last_update_time = time;
    p->last_route_idx = new_route_idx;
}

void Position_HandleStop(Position *p, int idx_new, int time) {
    ASSERT(p->state == PSTATE_DECEL, "Cannot stop without first decelerating (%d).", p->state);
    //int dt = time - p->last_update_time;
    //int distance = p->millis_off_last_node + p->v * dt + (1 / 2) * p->a * dt * dt;
    p->state = PSTATE_STOPPED;
    int distance = p->millis_off_stop_end;
    FdistReq frq = {TRACK_NODE_TO_INDEX(p->stop_end_pos), distance};
    TrackPosition fdist = GetFdist(WhoIs(NAME_TRACK_STATE), frq);
    ASSERT(0 <= fdist.object && fdist.object <= TRACK_MAX, "invalid object");
    p->last_known_node = &track[fdist.object];
    //PANIC("::: %s (%d) + %d -> %s + %d", p->stop_end_pos->name, TRACK_NODE_TO_INDEX(p->stop_end_pos), distance, p->last_known_node->name, fdist.distance_past);
    p->millis_off_last_node = fdist.distance_past;
    p->last_update_time = time;
    p->last_route_idx = idx_new;
    p->v = 0;
    p->v_max = 0;
    p->a = 0;
}

void Position_HandleBeginStop(Position *p, const Route *r, int time, const track_node * stop_end_pos, int millis_off_stop_end, int a){
    Position_HandleDecel(p, r, time, p->v, a);
    p->stop_end_pos = stop_end_pos;
    p->millis_off_stop_end = millis_off_stop_end;
}

void position_handle_accdec(Position *p, const Route *r, int time, int current_velocity, int a, int v_max, PositionState ps) {
    // when a decel starts, we first need to figure out what the most recent predicted node is
    int dt = time - p->last_update_time;
    int distance = position_calc_distance_with_max_velo(p, dt);
    int d = distance;
    // This should update idx.

    int idx = p->last_route_idx;
    bool on_route = TRUE;
    const track_node *tn = forward_dist_on_route_no_extra(r, &idx, p->last_known_node, &distance, &on_route, "handle accel/decel");
    if (tn == NULL) {
        // Decelerating off route - might be lost
        FdistReq frq = {TRACK_NODE_TO_INDEX(p->last_known_node), distance};
        TrackPosition fdist = GetFdist(WhoIs(NAME_TRACK_STATE), frq);
        tn = &track[fdist.object];
        distance = fdist.distance_past;
    }
    ASSERT(tn != NULL || !on_route, 
            "accelerating off route: %d, %s, %d, %d -> %d, %d -> %d, %d - %d = %d, |%d, %d, %d|, :%d -> %d@ ", 
            (int) p->last_known_node, p->last_known_node->name, distance,
            p->last_route_idx, idx,
            p->state, ps,
            time, p->last_update_time, dt,
            p->v, p->v_max, p->a,
            p->millis_off_last_node, d
            );
    p->state = ps;
    p->last_known_node  = tn;
    p->last_route_idx = idx;
    p->millis_off_last_node = distance;
    p->last_update_time = time;
    p->v = current_velocity;
    p->v_max = v_max;
    p->a = a;
}

void Position_HandleAccel(Position *p, const Route *r, int time, int current_velocity, int a, int v_max) {
    ASSERT(a > 0, "accel must be positive to accel, %d, %d", a, a / ACCELERATION_PRECISION);
    return position_handle_accdec(p, r, time, current_velocity, a, v_max, PSTATE_ACCEL);
}

void Position_HandleDecel(Position *p, const Route *r, int time, int current_velocity, int a) {
    ASSERT(a < 0, "accel must be negative to decel: pv %d, cv %d, a %d", p->v, current_velocity, a);
    return position_handle_accdec(p, r, time, current_velocity, a, current_velocity, PSTATE_DECEL);
}

void Position_HandleConstVelo(Position *p, const Route *r, int time, int current_velocity) {
    return position_handle_accdec(p, r, time, current_velocity, 0, current_velocity, PSTATE_CONST_VELO);
}

TrackPosition Position_CalculateNow(Position *p, const Route *r, int time) {
    int dt = time - p->last_update_time;
    ASSERT(dt >= 0, "negative dt");
    int distance = position_calc_distance_with_max_velo(p, dt);

    ASSERT(p != NULL && p->last_known_node != NULL, "null p or last node: %d/%d", (int) p, (int) (p ? p->last_known_node : NULL));
    int idx = p->last_route_idx, object = TRACK_NODE_TO_INDEX(p->last_known_node);
    // TODO this will totally break if your position predicts ahead too far
    if (r != NULL && !(p->state == PSTATE_STOPPED)) {
        bool on_route = TRUE;
        const track_node *tn = forward_dist_on_route_no_extra(r, &idx, p->last_known_node, &distance, &on_route, "position calculateion");
        //ASSERT(tn != NULL, "Null TrackNode");
        if (on_route){
            ASSERT_VALID_TRACK(tn);
            object = TRACK_NODE_TO_INDEX(tn);
        }
        else {
            // we past the end of the route
            /*
            FdistReq frq = {TRACK_NODE_TO_INDEX(p->last_known_node), distance};
            TrackPosition fdist = GetFdist(WhoIs(NAME_TRACK_STATE), frq);
            p->last_known_node = &track[fdist.object];
            //*/
            ASSERT_VALID_TRACK(p->last_known_node);
            object = TRACK_NODE_TO_INDEX(p->last_known_node);
        }
    }
    TrackPosition tp = {.object = object, .distance_past = distance};
    return tp;
}

void Position_Reverse(Position *p){
    p->millis_off_last_node *= -1;
    p->last_known_node = p->last_known_node->reverse;
}

int __attribute__((pure)) Position_CalculateVelocityNow(Position *p, int time) {
    ASSERT(time > p->last_update_time, "Can't calculate velocity retrospectively");
    int velo = p->v + ((time - p->last_update_time) * p->a * VELOCITY_PRECISION) / ACCELERATION_PRECISION;
    ASSERT(velo >= 0, "Cannot have negative velocity: velo %d v %d a %d lut %d t %d", velo, p->v, p->a, p->last_update_time, time);
    return MIN(velo, p->v_max);
}
