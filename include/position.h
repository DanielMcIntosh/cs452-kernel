#include <switch.h>
#include <route.h>
#include <track_node.h>
#include <track_position.h>

#ifndef POSITION_H
#define POSITION_H

typedef enum pstate {
    PSTATE_STOPPED,
    PSTATE_CONST_VELO,
    PSTATE_ACCEL,
    PSTATE_DECEL
} PositionState;

typedef struct tposition {
    PositionState state;
    const track_node *last_known_node;
    int millis_off_last_node;
    int last_update_time;
    SwitchState swdir;
    int last_route_idx;
    int v;
    int a;
    int v_max;
    const track_node *stop_end_pos;
    int millis_off_stop_end;
} Position;
#define POSITION_INIT {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

void Position_HandleStop(Position *p, int idx, int time);
void Position_HandleAccel(Position *p, const Route *r, int time, int current_velocity, int a, int v_max);
void Position_HandleDecel(Position *p, const Route *r, int time, int current_velocity, int a);
void Position_HandleBeginStop(Position *p, const Route *r, int time, const track_node * stop_end_pos, int millis_off_stop_end, int a);
void Position_HandleConstVelo(Position *p, const Route *r, int time, int current_velocity);
void Position_HandleSensorHit(Position* p, const track_node *snsr, int time, int new_route_idx);
void Position_Reverse(Position *p);
TrackPosition Position_CalculateNow(Position *p, const Route *r, int time);
int Position_CalculateVelocityNow(Position *p, int time);

#endif
