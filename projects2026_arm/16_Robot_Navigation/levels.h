/*
 * levels.h - the built-in arenas
 *
 * levels.c is the ONE copy of every map.  host/planner.py reads the same
 * file (it parses the string literals), so the Python planner and the robot
 * can never disagree about what level 3 looks like.
 */
#ifndef LEVELS_H
#define LEVELS_H

#include "sim.h"

#define N_LEVELS  5

/* Each row is GRID_W characters, TOP row (y = GRID_H - 1) first:
 *   '#' wall    '.' floor    'S' robot start (facing +x)    'G' goal
 *   'D' a door: floor at the start, a wall once the robot comes near it */
typedef struct {
    const char *name;
    const char *rows[GRID_H];
} level_t;

extern const level_t level_table[N_LEVELS];

#endif
