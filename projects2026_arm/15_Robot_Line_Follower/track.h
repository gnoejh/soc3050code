/*
 * track.h - the tracks: a closed centreline of tape on a white floor
 *
 * A track is written as TURTLE commands - "go straight 600 mm", "turn left
 * 180 degrees on a 400 mm radius" - and track_build() walks them into a
 * polyline of points in millimetres.  Arcs are cut into chords short enough
 * that the polyline never strays more than 1 mm from the true arc, so a
 * sensor cannot tell the difference.
 *
 * Pure C, no hardware: the same file builds into the firmware and host/sitl.c.
 */
#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>

#define TRACK_COUNT     4
#define TRACK_MAX_PTS   128
#define TAPE_HALF_MM    9.5f     /* 19 mm electrical tape                      */

typedef struct { int16_t x, y; uint16_t s; } track_pt_t;   /* mm; s = distance along */

typedef struct {
    int        id;               /* 0..TRACK_COUNT-1                             */
    int        n;                /* points; segment i runs pt[i] -> pt[i+1]      */
    track_pt_t pt[TRACK_MAX_PTS];
    uint8_t    gap[(TRACK_MAX_PTS + 7) / 8];   /* bit i: segment i has no tape   */
    uint16_t   length;           /* mm, once round                               */
    int16_t    xmin, xmax, ymin, ymax;
    float      x0, y0, th0;      /* the start line: position and heading         */
    float      close_err;        /* mm the turtle missed its start by (before the
                                    last point was snapped onto the first)       */
    uint8_t    crossing;         /* the track crosses itself                     */
} track_t;

extern track_t track;

int         track_build(int id);          /* returns the number of points     */
const char *track_name(int id);           /* "OVAL" ...                       */
const char *track_level(int id);          /* "easy" ...                       */

static inline int track_is_gap(int seg)
{
    return (track.gap[seg >> 3] >> (seg & 7)) & 1;
}

/* Distance from (px, py) to segment `seg`, in mm.  *t gets how far along the
 * segment the nearest point is (0..1); *side gets +1 if the point is to the
 * LEFT of the direction of travel, -1 to the right. */
float track_seg_dist(int seg, float px, float py, float *t, int *side);

#endif
