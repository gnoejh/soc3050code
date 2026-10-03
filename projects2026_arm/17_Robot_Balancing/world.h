/*
 * world.h - a simulated two-wheeled balancing robot
 *
 * The plant.  A wheeled inverted pendulum - a cart-pole whose "cart" is two
 * wheels driven through a DC motor - integrated at 1 kHz with its full
 * nonlinear equations, plus the sensors a real robot has: a gyro and an
 * accelerometer (with noise, bias, a mounting error, and the accelerometer
 * fooled by the robot's own acceleration) and a wheel encoder.
 *
 * Pure C, float, no registers: it runs inside the firmware as the "world"
 * task, and on a PC inside host/sitl.c.  The controller never includes this
 * file - it sees only sensors_t (control.h).
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>
#include "control.h"

typedef struct {
    /* the true state - the controller never sees these */
    float x, xd;          /* wheel axle position (m) and speed (m/s)          */
    float th, thd;        /* body tilt from vertical (rad, + = leaning forward) and rate */
    float xdd, thdd;      /* last accelerations, for the accelerometer         */
    float t;              /* simulated time, s                                 */

    /* what the world is being asked to do */
    float u;              /* voltage applied after the supply clipped it, V   */
    float tau;            /* total motor torque on the wheels, N m             */
    float payload;        /* kg at PAY_H, from the knob                        */
    float m, l, inertia;  /* body + payload: mass, CoM height, inertia about CoM */
    float push_force;     /* N, held while push_left > 0                       */
    float push_left;      /* s                                                 */
    float vbat;           /* supply voltage: the clip on |u|  (Lab: saturation) */
    float noise;          /* sensor noise scale: 0 clean, 1 nominal.  world_init
                             keeps payload, vbat and noise, so SET noise = 1
                             BEFORE the first world_init or the sensors are perfect */
    float mount_c, mount_s; /* cos and sin of IMU_TILT, the IMU's mounting error */
    uint8_t fallen;       /* |tilt| went past FALL_ANGLE                        */
    uint32_t rng;         /* xorshift32 state: the same seed, the same run      */
    uint32_t saturated;   /* steps on which the supply clipped the command     */
} world_t;

void  world_init(world_t *w, uint32_t seed, float tilt0);   /* standing, tilted tilt0 rad */
void  world_step(world_t *w, float volts);                  /* advance WORLD_DT           */
void  world_sense(world_t *w, float v_cmd, sensors_t *out); /* read the sensors now       */
void  world_push(world_t *w, float impulse_Ns);             /* a shove at the body's CoM  */
void  world_set_payload(world_t *w, float kg);

#endif
