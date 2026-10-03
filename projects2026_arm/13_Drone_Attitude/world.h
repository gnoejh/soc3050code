/*
 * world.h - a quadcopter on a gimbal, simulated inside the firmware
 *
 * SOC3050 lesson 13.  The "world" is the drone we do not have.  world_step()
 * advances its physics by WORLD_DT and world_sense() produces what an IMU
 * bolted to it would report - with noise, a drifting gyro bias and motor
 * vibration, because a perfect sensor teaches nothing about filtering.
 *
 * The model (slide "The Plant"):
 *   - a rigid X-quad, mass 0.5 kg, arms 0.12 m, rotating freely about its
 *     centre of mass: the classic ATTITUDE TEST STAND, a drone on a gimbal.
 *     It cannot climb, fall or drift sideways; position is lesson 14's job.
 *   - four motors, thrust = command x 4 N, each a first-order lag of 30 ms,
 *     command clamped to 0..1;
 *   - rotor drag torque = 0.016 m x thrust, which is what yaws a quad;
 *   - Euler's equations, I w' = tau - w x (I w) - c w, integrated with a
 *     quaternion so no attitude is singular (a flipped drone is just upside
 *     down, not a division by zero).
 *
 * Pure C and float: compiles unchanged into the firmware and into host/sitl.c.
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>
#include "control.h"          /* sensors_t: what the IMU part of it fills  */

#define WORLD_HZ    500.0f
#define WORLD_DT    (1.0f / WORLD_HZ)

typedef struct {
    /* physical parameters - change them and the controller must cope */
    float mass, arm, ixx, iyy, izz;
    float t_max;            /* N per motor at command 1                */
    float k_drag;           /* yaw torque per newton of thrust, m       */
    float tau_motor;        /* s                                        */
    float c_damp;           /* rotational air damping, N m s            */
    /* sensor quality */
    float gyro_noise;       /* rad/s, 1 sigma                           */
    float gyro_walk;        /* bias random walk, rad/s per sqrt(s)      */
    float accel_noise;      /* m/s^2, 1 sigma                           */
    float vib_accel;        /* m/s^2 of vibration per unit mean motor   */
    float dlpf;             /* sensor low-pass blend per step, 0..1     */
    /* state */
    float q[4];             /* attitude quaternion w, x, y, z           */
    float w[3];             /* body rates p, q, r             rad/s     */
    float thrust[4];        /* each motor's thrust now        N         */
    float bias[3];          /* gyro bias now                  rad/s     */
    float t;                /* simulated seconds                        */
    uint32_t vib_phase;     /* 2^32 = one turn                          */
    float k_motor, inv_tmax, inv_i[3];  /* derived by world_init()      */
    float gyro_f[3], accel_f[3];   /* the sensor's own low-pass filter  */
    uint32_t rng;
    uint32_t seq;
    uint8_t  locked;        /* 1 = someone is holding it: rates forced 0 */
    uint8_t  crashed;       /* latched when it tips past 90 degrees      */
} world_t;

void  world_init(world_t *w, uint32_t seed);
void  world_reset_attitude(world_t *w);                /* level, still      */
void  world_step(world_t *w, const float motor_cmd[4]);
void  world_sense(world_t *w, sensors_t *s);           /* gyro, accel, seq  */
void  world_kick(world_t *w, float dp, float dq);      /* the gust: rad/s   */
void  world_euler(const world_t *w, float *roll, float *pitch, float *yaw);

#endif
