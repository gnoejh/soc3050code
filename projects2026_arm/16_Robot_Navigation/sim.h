/*
 * sim.h - the Software-In-The-Loop contract for lesson 16
 *
 * Two halves of this program must never see each other's variables:
 *
 *   world.c   THE WORLD.  It knows the true map, the true pose, the true
 *             wheel sizes.  Every 20 ms it moves the robot and, every 40 ms,
 *             synthesises what the robot's sensors would read.
 *
 *   nav.c     THE ROBOT.  It knows only what arrives in a sensors_t, and it
 *             answers only with an actuators_t.  It starts not knowing the
 *             map, and its idea of where it is drifts.
 *
 * The one function between them is
 *
 *       void control_step(const sensors_t *in, actuators_t *out);
 *
 * Replace world.c by real motor drivers, encoders and rangefinders and nav.c
 * does not change - that is what "software in the loop" buys.  The units are
 * the ones real hardware produces: millimetres, encoder ticks, milliradians
 * per second, integers.  A real encoder does not hand you a float.
 *
 * Everything in this file is the robot's DATASHEET: nominal values the
 * robot's software is allowed to know.  The world uses slightly different
 * true values (world.c, "the robot as built"), which is where drift comes from.
 *
 * Pure C: no stm32c031xx.h anywhere below, so host/sitl.c compiles it on a PC.
 */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>

/* ---- the arena ------------------------------------------------------------
 * A grid of 10 cm cells, 32 x 14 - a 3.2 m x 1.4 m arena.  Drawn 4 px per
 * cell it is 128 x 56 pixels: the OLED, with one 8-pixel text line spare.
 *
 * Cells are (x, y) with y UP, like the metric frame: cell (0, 0) is the
 * BOTTOM-left.  A printed map lists its top row (y = 13) first. */
#define GRID_W        32
#define GRID_H        14
#define GRID_N        (GRID_W * GRID_H)          /* 448 cells */
#define CELL_MM       100

/* ---- the robot, nominal --------------------------------------------------- */
#define ROBOT_R_MM    35          /* body radius: a 7 cm round robot          */
#define WHEELBASE_MM  60          /* distance between the two wheels          */
#define TICKS_PER_M   3600        /* encoder ticks per metre of wheel travel   */
#define SPEED_MAX_MM  400         /* wheel speed limit, mm/s                   */

/* ---- range sensors -------------------------------------------------------
 * Five narrow-beam rangefinders (think VL53L0X or an IR triangulation sensor)
 * at these angles from the robot's heading, left positive. */
#define N_RANGE       5
#define RANGE_MAX_MM  600
#define RANGE_ANGLES_DEG  { 60, 30, 0, -30, -60 }
#define RANGE_NONE    RANGE_MAX_MM  /* "no echo" reads as the maximum         */

/* ---- one sensor sample: everything the robot may know --------------------- */
typedef struct {
    uint32_t t_ms;                /* time of the sample                        */
    int32_t  enc_l, enc_r;        /* cumulative encoder ticks, each wheel      */
    int32_t  gyro_mrad_s;         /* yaw rate, mrad/s, CCW positive: has BIAS  */
    uint16_t range_mm[N_RANGE];   /* RANGE_ANGLES_DEG order                    */
    uint8_t  bumper;              /* 1 while the body is pressed against a wall */
} sensors_t;

/* ---- one command: everything the robot may do ---------------------------- */
typedef struct {
    int16_t  wheel_l_mm_s;        /* wheel speed set-points, +- SPEED_MAX_MM   */
    int16_t  wheel_r_mm_s;
} actuators_t;

/* The robot's whole software, called every CONTROL_MS with the newest sample. */
#define CONTROL_MS    40
#define WORLD_MS      20
void control_step(const sensors_t *in, actuators_t *out);

#endif
