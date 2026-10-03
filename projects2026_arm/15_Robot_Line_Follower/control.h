/*
 * control.h - the contract between a robot's controller and its world
 *             SOC3050 lesson 15, and the shape lessons 13-18 share
 *
 *     void control_step(const sensors_t *in, actuators_t *out);
 *
 * The controller sees ONLY what a real line follower's firmware would see -
 * seven reflectance readings, two encoder counts, the clock and the speed
 * knob - and writes ONLY two wheel-speed commands.  It never sees the
 * robot's position, the track, or the true error.  That is the whole point
 * of Software-In-The-Loop: the same control.c could be linked against real
 * sensor drivers on a real chassis and not know the difference.
 *
 * On the Nucleo the "world" is world.c, running as the highest-priority RTOS
 * task.  On a PC it is the same world.c, driven by host/sitl.c.  The
 * controller cannot tell which.
 *
 * Integers on purpose: sensors arrive as ADC counts and motor drivers take
 * integer set-points.  What the controller does inside is its own business
 * (control.c uses float; lesson 10 measures what that costs).
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define LINE_SENSORS     7       /* the sensor bar                               */
#define SENSOR_PITCH_MM  13      /* centre to centre; the bar spans +-39 mm      */
#define CONTROL_HZ       100u    /* control_step() is called this often          */
#define WHEEL_MAX_MMS    2000    /* motor limit: commands are clamped to this    */
#define ENC_TICKS_REV    360     /* encoder ticks per wheel revolution           */

typedef struct {
    /* line[i]: reflectance 0..1000 - 1000 is black tape, ~100 is white floor.
     * i = 0 is the RIGHTMOST sensor, i = 6 the leftmost, looking forward;
     * sensor i sits (i - 3) * 13 mm to the left of the robot's centreline.  */
    uint16_t line[LINE_SENSORS];
    int32_t  enc_left, enc_right;   /* wheel encoder counts since reset         */
    int16_t  knob;                  /* the operator's speed setting, 0..1000    */
    uint32_t t_ms;                  /* time of this reading                     */
} sensors_t;

typedef struct {
    int16_t left, right;            /* wheel speed commands, mm/s, +-2000       */
} actuators_t;

void control_reset(void);                                   /* before a run */
void control_step(const sensors_t *in, actuators_t *out);   /* at 100 Hz    */

/* ---- the knobs a student turns (the shell's kp/ki/kd/df/slow commands) --- */
typedef struct {
    float kp;        /* (mm/s of wheel difference) per mm of line error        */
    float ki;        /* per mm*s                                               */
    float kd;        /* per mm/s                                               */
    float dfilt;     /* derivative low-pass, 0..1: 1 = no filter               */
    float slow;      /* corner slow-down: speed falls by this fraction at the
                        edge of the bar                                        */
    float thr;       /* a reading above this counts as "some tape"             */
} ctl_gains_t;

extern ctl_gains_t ctl_gains;

/* ---- what the controller believed, for telemetry only -------------------- */
typedef struct {
    float   pos;     /* estimated line position, mm, + = line is to the LEFT   */
    float   d;       /* filtered derivative of pos, mm/s                       */
    uint8_t lost;    /* 0 on the line; 1 coasting (a gap?); 2 searching        */
    uint8_t cross;   /* 1 if this step looked like a crossing                  */
} ctl_state_t;

extern ctl_state_t ctl_state;

#endif
