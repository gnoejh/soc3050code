/*
 * astar.h - A* on the robot's own 32 x 14 grid, integers only
 *
 * The planner knows nothing about walls or robots: it searches a COSTMAP, one
 * byte per cell, that nav.c builds from what the robot has mapped so far:
 *
 *   ASTAR_LETHAL (255)   never enter
 *   0..254               extra cost to ENTER this cell, on top of the step
 *
 * A step costs 10 straight and 14 diagonal (10 x sqrt 2, rounded), so every
 * cost is an integer and the heuristic - the "octile" distance, the exact
 * cost of an empty grid - never overestimates.  A diagonal step may not cut
 * the corner of a lethal cell.
 *
 * host/planner.py implements the same search with the same costs and the same
 * tie-break (lowest f, then lowest h, then lowest cell index), so the two
 * produce the SAME path, cell for cell, not merely the same cost.
 */
#ifndef ASTAR_H
#define ASTAR_H

#include <stdint.h>
#include "sim.h"

#define ASTAR_LETHAL   255u
#define ASTAR_NONE     0xFFFFu
#define ASTAR_STRAIGHT 10u
#define ASTAR_DIAG     14u
#define ASTAR_HEAP     160u       /* open-list slots, <= 255 (hpos[] is a byte);
                                     the host test measures the real peak      */

/* Cell index: x + y * GRID_W.  GRID_W is 32, so x = i & 31 and y = i >> 5. */
#define CELL(x, y)   ((uint16_t)((x) + (y) * GRID_W))
#define CELL_X(i)    ((int)((i) & (GRID_W - 1)))
#define CELL_Y(i)    ((int)((i) >> 5))

typedef struct {
    uint16_t cost;          /* total path cost, or ASTAR_NONE: no path       */
    uint16_t len;           /* cells in path[], start and goal included      */
    uint16_t expanded;      /* nodes taken off the open list                 */
    uint16_t open_peak;     /* the open list's high-water mark               */
    uint8_t  overflow;      /* 1: gave up because the open list was full     */
} astar_result_t;

/* Plan from `start` to `goal` across `costmap` (GRID_N bytes).  Writes up to
 * `path_max` cell indices, start first, into `path`.  Returns 1 if a path was
 * found, 0 if none exists (result.cost == ASTAR_NONE). */
int astar_plan(const uint8_t *costmap, uint16_t start, uint16_t goal,
               uint16_t *path, uint16_t path_max, astar_result_t *result);

/* RAM the search uses, all static: reported by the `stats` command. */
uint32_t astar_ram_bytes(void);

#endif
