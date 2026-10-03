/*
 * world.h - the sumo ring and two robots: physics and sensor synthesis
 *
 * The WORLD half of software-in-the-loop.  It owns the truth - where each
 * robot is, how fast it moves, who touches whom - and turns that truth into
 * the readings each robot's sensors would give (robot_view_t, strategy.h).
 * Strategies never see this file's structs.
 *
 * Units: metres, seconds, kilograms, radians; x right, y up, angles
 * counter-clockwise from +x.  Float, because the physics is easier to read
 * that way and lesson 10 measured what float costs on this chip; every
 * square root and trig call goes through fmath.h.
 *
 * Pure C: no chip header.  The firmware and host/league.c compile the same
 * file, and given the same seed both should produce the same bits.
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>
#include "strategy.h"

/* ---- the ring: a mini-sumo dohyo ----------------------------------------- */
#define RING_R      0.385f      /* 77 cm diameter                              */
#define LINE_W      0.025f      /* the white border, inside RING_R             */

/* ---- the robot: every robot is the same machine ---------------------------- */
#define ROBOT_R     0.055f      /* drawn and collided as a circle              */
#define ROBOT_M     0.50f       /* kg - the mini-sumo weight limit             */
#define ROBOT_I     (0.5f * ROBOT_M * ROBOT_R * ROBOT_R)   /* a uniform disc   */
#define HALF_TRACK  0.045f      /* wheel to centre                             */
#define V_MAX       0.80f       /* m/s, wheel speed with no load at 100 %      */
#define F_STALL     3.0f        /* N per wheel at 0 speed and 100 %            */
#define MU_LONG     1.0f        /* tyre grip along the wheel                   */
#define MU_LAT      0.6f        /* ... and sideways: a tyre slides sideways first */
#define MU_BODY     0.3f        /* robot-on-robot friction                     */
#define RESTITUTION 0.1f        /* almost dead collisions                      */
#define GRAVITY     9.81f

#define SENSOR_SPREAD 0.5236f   /* 30 degrees: left/right distance sensors    */
#define SENSOR_CONE   0.1309f   /* 7.5 degree half-angle of each cone          */
#define EDGE_POS      0.040f    /* corner sensors at (+-40, +-40) mm           */

#define DT          0.005f      /* physics step: 5 ms, 200 Hz                  */

typedef struct {
    float x, y, th;             /* pose                                        */
    float vx, vy, w;            /* velocity, angular velocity                  */
    float enc_l, enc_r;         /* wheel travel, metres                        */
    float drive_l, drive_r;     /* commanded, -1..1                            */
    uint8_t bump;               /* BUMP_* since the last sense                 */
    uint8_t slip;               /* bit 0 left, bit 1 right: wheel slipping now */
    uint32_t rng;               /* this robot's sensor-noise generator         */
} body_t;

typedef struct {
    body_t  b[2];
    uint8_t touching;           /* in contact this step                        */
    uint32_t steps;
} world_t;

void world_place(world_t *w, int i, float x, float y, float th, uint32_t seed);
void world_drive(world_t *w, int i, const robot_cmd_t *cmd);   /* latch command */
void world_step(world_t *w);                                    /* advance DT    */
void world_sense(world_t *w, int i, robot_view_t *v);   /* fills sensor fields,
                                                           clears bump        */
float world_radius(const world_t *w, int i);            /* centre to ring centre */
float w_sin(float a);                                   /* fmath.h's, out of line */
float w_cos(float a);

/* xorshift32: the one random generator, shared by world and referee. */
static inline uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;  x ^= x >> 17;  x ^= x << 5;
    return *s = x;
}

#endif
