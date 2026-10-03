/*
 * world.h - the simulated arena and the robot's true body
 *
 * THE ROBOT'S SOFTWARE MUST NOT INCLUDE THIS FILE.  nav.c sees the world only
 * through sensors_t (sim.h).  Main.c and host/sitl.c - the harness - include
 * both, which is their whole job.
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>
#include "sim.h"
#include "levels.h"

#define LEVEL_CUSTOM  N_LEVELS          /* levels 0..4 built in, 5 = from $MAP */

typedef struct {
    /* the truth */
    float    x, y, th;            /* pose: mm, mm, rad (CCW from +x)          */
    float    vl, vr;              /* wheel ground speeds after the motor lag  */
    uint32_t t_ms;                /* simulated time                            */
    float    odometer_mm;         /* how far the body really travelled         */
    /* events */
    uint8_t  bumper;              /* touching a wall now                       */
    uint16_t collisions;          /* bumper rising edges: the penalty count    */
    uint8_t  door_closed;
    /* the level */
    int      level;               /* 0..4, or LEVEL_CUSTOM                     */
    int      start_x, start_y;    /* cells                                     */
    int      goal_x, goal_y;
} world_t;

const world_t *world(void);

/* Load a level (0..N_LEVELS-1) or LEVEL_CUSTOM; robot at the start, stopped. */
void world_load(int level);

/* Advance WORLD_MS of physics under these wheel commands. */
void world_step(const actuators_t *cmd);

/* What the sensors read now.  Call every CONTROL_MS. */
void world_sense(sensors_t *s);

/* The true map: 1 = wall. Outside the grid counts as wall. */
int  world_occ(int x, int y);

/* The custom level, as $MAP frames build it.  `bits` has x = 0 in bit 31,
 * so the hex digits read left to right like a printed map. */
void world_custom_row(int y, uint32_t bits);
void world_custom_start(int x, int y);
void world_custom_goal(int x, int y);
uint32_t world_custom_rowbits(int y);

/* Test hooks for the labs and the host test. */
void world_set_seed(uint32_t seed);

#endif
