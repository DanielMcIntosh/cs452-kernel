#ifndef __TRACK_POSITION_H__
#define __TRACK_POSITION_H__

typedef struct position {
    const int object : 16;
    const int distance_past : 16;
} TrackPosition;

typedef union tps {
    TrackPosition tp;
    int bytes;
} TrackPositionUnion;


#endif //__TRACK_POSITION_H__
