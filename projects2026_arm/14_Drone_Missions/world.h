/*
 * world.h - the simulated drone and its weather, inside the firmware
 *
 * There is no drone in Wokwi, so the firmware carries one: a world task
 * steps this model at WORLD_HZ and the flight computer only ever sees it
 * through sensors_t (control.h) - noisy, late and incomplete, as a real
 * flight computer sees a real aircraft.
 *
 * The model, one line each (slides 2-6):
 *   - a point mass in 3D, pushed by a thrust vector tilted by roll and pitch;
 *   - the attitude and thrust FOLLOW their commands with first-order lags
 *     (tau 80 ms and 50 ms) - lesson 13's inner loop, abstracted;
 *   - linear rotor drag on the velocity relative to the AIR;
 *   - wind = steady + gusts, the gusts a first-order filtered random process
 *     from a deterministic generator, so every run with one seed is the same;
 *   - a battery that drains faster the harder the rotors work;
 *   - GPS at 10 Hz, 200 ms late, with slowly wandering error; a noisy baro.
 *
 * Pure C: no registers, so host/sitl.c flies the very same file.
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>
#include "control.h"

#define WORLD_HZ        100u
#define WORLD_DT        (1.0f / (float)WORLD_HZ)
#define GPS_DIV         5u          /* one fix every 5 control steps = 10 Hz  */
#define GPS_DELAY       10u         /* control steps: 10 x 20 ms = 200 ms     */

typedef struct {
    /* truth - only the simulator knows these */
    float px, py, pz;               /* m                                      */
    float vx, vy, vz;               /* m/s                                    */
    float ax, ay, az;               /* m/s^2, the last step's                 */
    float roll, pitch, yaw;         /* rad, actual                            */
    float yaw_rate;                 /* rad/s, actual                          */
    float thrust;                   /* m/s^2, actual                          */
    float wind_speed, wind_dir;     /* steady part: m/s, rad (blowing TOWARD) */
    float gust_x, gust_y;           /* m/s                                    */
    float wind_x, wind_y;           /* m/s, steady + gust, this step          */
    float battery;                  /* percent                                */
    float drain;                    /* percent per second at hover            */
    float max_impact;               /* worst touchdown speed seen, m/s        */
    float tau_att;                  /* s, lesson 13 abstracted: attitude lag   */
    uint32_t t_ms;
    uint32_t rng;                   /* xorshift32 state                       */
    uint8_t  on_ground;
    /* sensor state */
    float gps_err_x, gps_err_y;     /* slowly wandering GPS error, m          */
    float hist_x[GPS_DELAY + 1], hist_y[GPS_DELAY + 1];
    uint8_t  hist_i;
    uint32_t senses;
} world_t;

void  world_init(world_t *w, uint32_t seed);
void  world_set_wind(world_t *w, float speed, float dir_rad);
void  world_step(world_t *w, const actuators_t *a, float dt);  /* at WORLD_HZ */
void  world_sense(world_t *w, sensors_t *s);                    /* at CTRL_HZ  */

#endif
