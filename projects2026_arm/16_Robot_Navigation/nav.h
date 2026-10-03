/*
 * nav.h - the robot's software: estimate, map, plan, follow, stay safe
 *
 * Everything here runs inside control_step() (sim.h), which sees only a
 * sensors_t.  The functions below are the MISSION interface - what an
 * operator may tell the robot (where it starts, where to go, how to behave)
 * and what it may report back.  None of them reveals the world.
 *
 * Pure C.  The harness (Main.c on the board, host/sitl.c on a PC) must
 * provide nav_clock(), the stopwatch the planner is timed with.
 */
#ifndef NAV_H
#define NAV_H

#include <stdint.h>
#include "sim.h"

enum { NAV_IDLE, NAV_CALIB, NAV_RUN, NAV_ARRIVED, NAV_NOPATH };
enum { MAP_UNKNOWN, MAP_FREE, MAP_OCC };
enum { HEAD_GYRO, HEAD_WHEELS };

#define NAV_PATH_MAX   128        /* cells; a longer plan is followed in parts */

typedef struct {                  /* the knobs the labs turn                  */
    int16_t speed_mm_s;           /* cruise speed (the board's knob)          */
    uint8_t inflate;              /* 0, 1 or 2 cells of keep-away cost         */
    uint8_t reactive;             /* 1: the safety layer may override the plan */
    uint8_t heading;              /* HEAD_GYRO or HEAD_WHEELS                  */
    uint8_t gyro_cal;             /* 1: measure the gyro's bias before moving  */
} nav_params_t;

typedef struct {
    uint16_t plans;               /* A* runs                                    */
    uint16_t replans;             /* ... of which forced by something new       */
    uint16_t last_expanded, peak_expanded, peak_open;
    uint32_t last_plan_clk, peak_plan_clk;   /* nav_clock() units               */
    uint16_t path_cost, path_len;
    uint16_t brakes;              /* control steps the safety layer cut speed   */
    uint16_t backups;             /* bumper reflexes                            */
    uint16_t vetoes;              /* cells the safety layer barred the planner from */
    uint16_t recoveries;          /* "no path" that a second look disproved      */
    uint16_t overflows;           /* searches that ran out of open-list slots     */
    uint32_t run_ms;              /* start to arrival                           */
    int32_t  gyro_bias_urad_s;    /* what calibration measured                  */
} nav_stats_t;

/* ---- mission ---------------------------------------------------------------- */
void nav_reset(int start_x, int start_y);      /* forget the map; you are here,  */
                                               /* facing +x                      */
void nav_set_goal(int x, int y);
void nav_get_goal(int *x, int *y);
void nav_start(void);                          /* calibrate 1 s, then go         */
int  nav_mode(void);
const char *nav_mode_name(int mode);
nav_params_t *nav_params(void);

/* ---- reports, for the display and telemetry ---------------------------------- */
const nav_stats_t *nav_stats(void);
int  nav_cell(int x, int y);                   /* MAP_UNKNOWN / FREE / OCC        */
const uint16_t *nav_path(uint16_t *len, uint16_t *progress);
void nav_pose(float *x, float *y, float *th);  /* the ESTIMATE                    */
uint32_t nav_ram_bytes(void);                  /* map + costmap + path + A*       */

/* ---- the costmap: what A* searches (astar.h) ------------------------------------
 * `cls(x, y)` says what a cell is (MAP_*).  Occupied cells become lethal; the
 * `inflate` cells around them get a keep-away cost; unknown cells a small one.
 * The robot passes its own map; the host test passes the TRUE map to find the
 * optimal path; host/planner.py does the same arithmetic in Python. */
#define COST_UNKNOWN   3
#define COST_INFLATE1  12          /* a cell touching a wall                 */
#define COST_INFLATE2  4           /* two cells from a wall                  */
void costmap_build(int (*cls)(int x, int y), uint8_t inflate, uint8_t *out);
const uint8_t *nav_costmap(void);              /* the robot's, rebuilt now        */

/* ---- for the shell's "map" dump, which host/planner.py checks ---------------- */
int      nav_plan_now(void);                   /* re-plan from here; 1 = a path   */
uint16_t nav_plan_start(void);                 /* the cell that plan started in   */
int      nav_vetoes(uint16_t *cells, int max); /* cells barred by the safety layer */

uint32_t nav_clock(void);                      /* provided by the harness         */

#endif
