/*
 * world.h - the robot's world: physics, sensors and the race judge
 *
 * Everything the controller is NOT allowed to see lives here: where the robot
 * really is, how fast it is really sliding, how far it really is from the
 * line.  The firmware runs it as the top-priority RTOS task; host/sitl.c runs
 * the very same file on a PC.  Pure C, float, fmath.h - no hardware.
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdint.h>
#include "control.h"

#define WORLD_HZ      200u          /* physics steps per second              */
#define WORLD_DT      (1.0f / (float)WORLD_HZ)

/* ---- the robot, as built (change them and the race changes) ------------ */
#define ROBOT_TRACK_MM   100.0f     /* distance between the wheels           */
#define BAR_AHEAD_MM     70.0f      /* sensor bar, ahead of the axle         */
#define WHEEL_R_MM       16.0f      /* wheel radius: encoder ticks -> mm     */
#define MOTOR_TAU_S      0.040f     /* wheel speed lags its command by this  */
#define MOTOR_ACC_MMS2   8000.0f    /* and cannot change faster than this    */
#define TYRE_K           60.0f      /* how hard the tyres pull the body into
                                       line with where it is pointing, 1/s   */
#define MU_G_MMS2        6867.0f    /* friction limit: 0.7 g.  The circle    */

#define DNF_WHITE_MS     1000u      /* no sensor over tape for this long     */
#define DNF_FAR_MM       150.0f     /* robot centre this far from the tape   */

enum { W_READY = 0, W_RUN = 1, W_DNF = 2 };
enum { DNF_NONE = 0, DNF_LOST = 1, DNF_FAR = 2 };

typedef struct {
    /* truth */
    float    x, y, th;           /* axle centre (mm) and heading (rad)          */
    float    vx, vy;             /* velocity over the ground, mm/s              */
    float    wl, wr;             /* wheel surface speeds, mm/s                  */
    float    dist_l, dist_r;     /* mm each wheel has turned through (encoders) */
    float    cov[LINE_SENSORS];  /* how much of each sensor's spot is on tape   */
    uint8_t  slipping;           /* the friction circle is saturated now        */
    uint32_t slip_ms;            /* total time spent sliding this run           */
    uint32_t t_ms;               /* world time: steps * 5 ms                    */
    uint16_t cand;               /* segments that passed the box test, last step */

    /* the judge */
    uint8_t  state, dnf;         /* W_READY / W_RUN / W_DNF and why             */
    int      seg;                /* the segment of centreline the robot is on   */
    float    s;                  /* distance along the track, mm                */
    float    err;                /* TRUE lateral error of the robot centre, mm  */
    float    max_err;            /* largest |err| this run                      */
    float    err2;               /* sum of err^2, for an RMS                    */
    uint32_t err_n;
    uint8_t  halfway;            /* passed half distance on this lap            */
    uint16_t laps;
    uint32_t lap_start, last_lap, best_lap;  /* ms; 0 = none yet                */
    uint32_t white_ms;           /* how long no sensor has seen tape            */

    /* the sensor model - the shell's noise / ambient / fail commands */
    int16_t  noise;              /* about one standard deviation, counts        */
    int16_t  ambient;            /* added to every reading, counts              */
    int8_t   failed;             /* this sensor reads 0 forever; -1 = none      */
    uint32_t rng;
} world_t;

extern world_t world;

void world_reset(int track_id);  /* build the track, park the robot on the start */
void world_start(void);          /* lights out: the clock runs                   */
void world_step(const actuators_t *cmd);     /* one 5 ms step                    */
void world_sense(sensors_t *s);  /* what the sensors read now                    */

#endif
