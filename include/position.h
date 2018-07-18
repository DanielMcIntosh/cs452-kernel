#include <switch.h>
#include <track.h>
#include <train_state.h>

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
    //int a;
    const track_node *stop_end_pos;
    int millis_off_stop_end;
} Position;

#define POSITION_INIT {0, 0, 0, 0, 0, 0, 0, 0, 0}

void Position_HandleStop(Position *p, Route *r, int idx, int time);
void Position_HandleAccel(Position *p, const Route *r, int time, int current_velocity, int a);
void Position_HandleDecel(Position *p, const Route *r, int time, int current_velocity, int a);
void Position_HandleBeginStop(Position *p, const Route *r, int time, const track_node * stop_end_pos, int millis_off_stop_end);
void Position_HandleConstVelo(Position *p, Route *r, int time, int current_velocity);
void Position_HandleSensorHit(Position* p, track_node *snsr, int time, int new_route_idx);
TrackPosition Position_CalculateNow(Position *p, const Route *r, int time);

#endif
