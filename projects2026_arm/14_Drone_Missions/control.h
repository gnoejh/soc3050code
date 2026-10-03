/*
 * control.h - the contract between the plant and the flight computer
 *
 * Part 3's two-tier rule, low-level half: whatever produces the sensor
 * readings (a world model inside this firmware, a host test, one day a real
 * airframe) and whatever consumes the actuator commands meet in exactly one
 * function:
 *
 *     void control_step(const sensors_t *in, actuators_t *out);
 *
 * called at CTRL_HZ.  Lesson 13 defines the same function for the attitude
 * loops; this lesson's is one level up - it commands TILT and THRUST and
 * assumes lesson 13's loops deliver them (slide 3 says why that is allowed).
 *
 * Pure C, no hardware: the same header compiles in host/sitl.c.
 *
 * Frame: x = east, y = north, z = up, metres, origin at the launch point
 * ("home").  Yaw is measured from +x (east), counter-clockwise, radians.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define CTRL_HZ     50u
#define CTRL_DT     (1.0f / (float)CTRL_HZ)
#define GRAVITY     9.81f

typedef struct {
    uint32_t t_ms;               /* time of this sample                          */
    float    acc_x, acc_y, acc_z;/* linear acceleration, WORLD frame, gravity
                                    removed, m/s^2 - what lesson 13's attitude
                                    estimate lets you compute from the IMU     */
    float    yaw;                /* heading, rad (a compass, in effect)          */
    float    baro_alt;           /* barometric altitude, m, noisy, every step    */
    uint8_t  gps_new;            /* 1 on the step a GPS fix arrives (10 Hz)      */
    float    gps_x, gps_y;       /* that fix, m - and it is 200 ms OLD           */
    float    battery;            /* percent remaining                            */
} sensors_t;

typedef struct {
    uint8_t  armed;              /* 0: motors off                                */
    float    thrust;             /* specific thrust, m/s^2: 9.81 hovers          */
    float    roll, pitch;        /* commanded body tilt, rad: lesson 13's
                                    attitude loop is assumed to track these     */
    float    yaw_rate;           /* rad/s                                        */
} actuators_t;

void control_step(const sensors_t *in, actuators_t *out);

#endif
