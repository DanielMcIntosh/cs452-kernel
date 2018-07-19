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
 */

void Position_HandleSensorHit(Position* p, track_node *snsr, int time, int new_route_idx){
    p->last_known_node = snsr;
    p->millis_off_last_node = 0;
    /*
    if (p->state == PSTATE_ACCEL) {
        p->v = p->v + (time - p->last_update_time) * p->a;
    } else if (p->state == PSTATE_DECEL) {
        p->v = p->v - (time - p->last_update_time) * p->a;
    }
    */
    p->last_update_time = time;
    p->last_route_idx = new_route_idx;
}

void Position_HandleStop(Position *p, int idx_new, int time) {
    ASSERT(p->state == PSTATE_DECEL, "Cannot stop without first decelerating.");
    //int dt = time - p->last_update_time;
    //int distance = p->millis_off_last_node + p->v * dt + (1 / 2) * p->a * dt * dt;
    p->state = PSTATE_STOPPED;
    int distance = p->millis_off_stop_end;
    FdistReq frq = {TRACK_NODE_TO_INDEX(p->stop_end_pos), distance};
    TrackPosition fdist = GetFdist(WhoIs(NAME_TRACK_STATE), frq);
    p->last_known_node = &track[fdist.object];
    //PANIC("::: %s (%d) + %d -> %s + %d", p->stop_end_pos->name, TRACK_NODE_TO_INDEX(p->stop_end_pos), distance, p->last_known_node->name, fdist.distance_past);
    p->millis_off_last_node = fdist.distance_past;
    p->last_update_time = time;
    p->last_route_idx = idx_new;
    p->v = 0;
    //p->a = 0;
}

void Position_HandleBeginStop(Position *p, const Route *r, int time, const track_node * stop_end_pos, int millis_off_stop_end){
    Position_HandleDecel(p, r, time, p->v, 0);
    p->stop_end_pos = stop_end_pos;
    p->millis_off_stop_end = millis_off_stop_end;
}

void position_handle_accdec(Position *p, const Route *r, int time, int current_velocity, int a, PositionState ps) {
    p->state = ps;
    // when a decel starts, we first need to figure out what the most recent predicted node is
    int dt = time - p->last_update_time;
    int distance = p->millis_off_last_node + p->v * dt / VELOCITY_PRECISION;// + (1 / 2) * p->a * dt * dt;
    
    // This should update idx.
    p->last_known_node = forward_dist_on_route_no_extra(r, &p->last_route_idx, p->last_known_node, &distance, "handle accel/decel");
    p->millis_off_last_node = distance;
    p->last_update_time = time;
    p->v = current_velocity;
    //p->a = a;
}

void Position_HandleAccel(Position *p, const Route *r, int time, int current_velocity, int a) {
    return position_handle_accdec(p, r, time, current_velocity, a, PSTATE_ACCEL);
}

void Position_HandleDecel(Position *p, const Route *r, int time, int current_velocity, int a) {
    return position_handle_accdec(p, r, time, current_velocity, a, PSTATE_DECEL);
}

void Position_HandleConstVelo(Position *p, Route *r, int time, int current_velocity) {
    return position_handle_accdec(p, r, time, current_velocity, 0, PSTATE_CONST_VELO);
}

TrackPosition Position_CalculateNow(Position *p, const Route *r, int time) {
    int distance = p->millis_off_last_node + ((p->state != PSTATE_STOPPED) ? p->v * (time - p->last_update_time) / VELOCITY_PRECISION : 0);
    int idx = p->last_route_idx, object = TRACK_NODE_TO_INDEX(p->last_known_node);
    // TODO this will totally break if your position predicts ahead too far
    if (r != NULL && !(p->state == PSTATE_STOPPED)) {
        const track_node *tn = forward_dist_on_route_no_extra(r, &idx, p->last_known_node, &distance, "position calculateion");
        //ASSERT(tn != NULL, "Null TrackNode");
        object = TRACK_NODE_TO_INDEX(tn);
    }
    TrackPosition tp = {.object = object, .distance_past = distance};
    return tp;
}

void Position_Reverse(Position *p){
    p->millis_off_last_node *= -1;
    p->last_known_node = p->last_known_node->reverse;
}
