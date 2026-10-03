/*
 * params.h - the balancing robot, in numbers.  ONE SOURCE OF TRUTH.
 *
 * Three programs read this file:
 *   world.c        the simulated robot inside the firmware (and on the host)
 *   control.c      the controllers' timing and limits
 *   host/lqr.py    linearises the SAME model and computes the LQR gain K
 *
 * lqr.py does not include it - it is Python - it PARSES it: every line of the
 * form  "#define NAME expression"  below is evaluated in order, so an
 * expression may use names defined above it.  Keep to that shape: one define
 * per line, numbers (an `f` suffix is fine), + - * / and parentheses.
 * Change a mass here, re-run lqr.py, paste the new K: the plant, the firmware
 * and the gain design can never disagree about the robot.
 *
 * Units are SI throughout: kg, m, s, rad, V, A, N.
 *
 * The robot: a small two-wheeled balancer about 25 cm tall - a body (battery,
 * board, motors) on an axle with two wheels.  The numbers are of the size of
 * the hobby kits sold for exactly this exercise; they are not one product's.
 */
#ifndef PARAMS_H
#define PARAMS_H

/* ---- the body (everything above the axle) ------------------------------ */
#define BOT_MB        0.80f     /* body mass, kg                                  */
#define BOT_L         0.10f     /* axle -> body centre of mass, m                 */
#define BOT_IB        0.0045f   /* body inertia about its own centre of mass      */

/* ---- the wheels (both together) ---------------------------------------- */
#define BOT_MW        0.10f     /* both wheels, kg                                */
#define BOT_R         0.035f    /* wheel radius, m                                */
#define BOT_IW        (0.5f * BOT_MW * BOT_R * BOT_R)   /* two solid discs       */
#define BOT_BROLL     0.05f     /* rolling friction on the ground, N per m/s      */

/* ---- the motors (two, geared; figures referred to the WHEEL shaft) ----- */
#define MOT_N         2.0f      /* how many                                       */
#define MOT_K         0.20f     /* torque constant N m/A = back-EMF constant V s/rad */
#define MOT_RES       4.0f      /* winding resistance, ohm                        */
#define MOT_B         0.002f    /* gearbox viscous friction, N m per rad/s        */
#define VBAT          7.4f      /* supply: a 2-cell lithium pack.  |u| <= VBAT    */

/* ---- the payload the knob adds (Lab: robustness) ----------------------- */
#define PAY_H         0.20f     /* height of the payload above the axle, m        */
#define PAY_MAX       0.60f     /* knob fully clockwise, kg                       */

/* ---- sensors ------------------------------------------------------------ */
#define IMU_H         0.08f     /* height of the IMU above the axle, m            */
#define IMU_TILT      0.0175f   /* IMU mounted 1 degree off true, rad             */
#define GYRO_BIAS     0.005f    /* gyro zero-rate offset, rad/s (0.3 deg/s)       */
#define GYRO_NOISE    0.010f    /* gyro noise, rad/s rms per sample               */
#define ACC_NOISE     0.30f     /* accelerometer noise, m/s^2 rms per sample      */
#define ENC_CPR       1440.0f   /* encoder counts per wheel revolution            */

/* ---- the world ---------------------------------------------------------- */
#define GRAV          9.81f
#define FALL_ANGLE    0.785f    /* 45 degrees: past this it is on the floor       */
#define PUSH_TIME     0.020f    /* a push is a force held this long, s            */

/* ---- timing ------------------------------------------------------------- */
#define WORLD_DT      0.001f    /* physics step: 1 kHz                            */
#define CTRL_DT       0.005f    /* controller step: 200 Hz - lqr.py designs for this */

/* ---- LQR weights (lqr.py reads these; the firmware does not) ----------- */
/* Q penalises the state [x, x_dot, theta, theta_dot]; R the voltage.  Lab
 * Part 4 changes these, re-runs lqr.py and pastes the new K into control.c. */
#define LQR_QX        30.0f     /* position error, per m^2                        */
#define LQR_QV        2.0f      /* speed error, per (m/s)^2                       */
#define LQR_QTH       50.0f     /* tilt, per rad^2                                */
#define LQR_QW        0.5f      /* tilt rate, per (rad/s)^2                       */
#define LQR_R         1.0f      /* voltage, per V^2                               */

#endif
