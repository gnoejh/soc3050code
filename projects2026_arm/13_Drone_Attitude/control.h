/*
 * control.h - the contract between a flight controller and the world
 *
 * SOC3050 lesson 13.  This header is the whole interface.  The controller
 * (control.c, the student's code) sees ONLY a sensors_t and writes ONLY an
 * actuators_t, once per control period:
 *
 *     void control_step(const sensors_t *in, actuators_t *out);
 *
 * On a real drone `in` is filled from an IMU and a radio receiver and `out`
 * goes to four ESCs.  In this lesson `in` is filled by world.c's physics model
 * - or by a real MPU6050 - and `out` drives the model's motors.  The
 * controller cannot tell which.  That is Software-In-The-Loop (SITL), and it
 * is why the SAME control.c links into the firmware AND into host/sitl.c.
 *
 * Pure C: no stm32c031xx.h, no RTOS, nothing but <stdint.h>.
 *
 * Frames and signs (aerospace convention, "FRD"):
 *   body x forward, y right, z DOWN.
 *   roll  > 0  right side down        p = roll rate  (about x)
 *   pitch > 0  nose up                q = pitch rate (about y)
 *   yaw   > 0  nose to the right      r = yaw rate   (about z)
 *   Level and still, the accelerometer reads (0, 0, -9.81): it measures the
 *   force holding it up (specific force), which points up, i.e. -z.
 *
 * Motors, seen from above, nose up the page (an "X" quad):
 *
 *        M0 FL (CW)    M1 FR (CCW)
 *                 \  /
 *                  ><
 *                 /  \
 *        M3 RL (CCW)   M2 RR (CW)
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* ---- in: everything the flight controller is allowed to know ------------ */
typedef struct {
    float    gyro[3];       /* body rates p, q, r                 rad/s      */
    float    accel[3];      /* specific force, body frame         m/s^2      */
    uint32_t seq;           /* +1 per new IMU sample; unchanged = stale      */
    uint8_t  imu_ok;        /* 0 = the driver says the IMU failed            */
    /* the pilot - a radio receiver is a sensor too, from here */
    float    rc_roll;       /* roll  setpoint                     rad        */
    float    rc_pitch;      /* pitch setpoint                     rad        */
    float    rc_yaw_rate;   /* yaw-rate setpoint                  rad/s      */
    float    rc_throttle;   /* collective thrust                  0..1       */
    uint8_t  rc_arm;        /* arm switch                         0/1        */
} sensors_t;

/* ---- out: everything the flight controller is allowed to do -------------- */
typedef struct {
    float    motor[4];      /* M0..M3 command, 0..1 (0 = stopped)            */
    uint8_t  armed;
} actuators_t;

/* ---- why the controller is (or went) disarmed -------------------------- */
enum {
    FS_NONE = 0,       /* disarmed because nobody armed it                   */
    FS_TILT,           /* |roll| or |pitch| estimate > CTRL_TILT_LIMIT        */
    FS_SENSOR,         /* imu_ok = 0, a non-finite or absurd reading         */
    FS_STALE,          /* seq did not change for CTRL_STALE_STEPS steps       */
    FS_THROTTLE        /* arm refused: throttle not low                      */
};

/* ---- tuning: written by Main.c (the knob) or the host test ------------- */
typedef struct {
    float alpha;          /* complementary filter: 1 = gyro only, 0 = accel only */
    float angle_kp;       /* angle loop: rate setpoint per rad of angle error    */
    float rate_kp;        /* rate loop, per rad/s of rate error  (roll, pitch)   */
    float rate_ki;
    float rate_kd;
    float yaw_kp, yaw_ki;
    float kp_scale;       /* the knob: multiplies rate_kp (x0.125 .. x8)           */
    float max_rate;       /* angle loop output clamp            rad/s           */
    float max_out;        /* per-axis rate-loop output clamp    motor units     */
    float i_limit;        /* integrator clamp                   motor units     */
    uint8_t anti_windup;  /* 1 = stop integrating while the output saturates    */
    int8_t  mix_roll_sign;/* +1 correct; -1 is Lab Part 6's broken mixer        */
} ctrl_tune_t;

/* ---- what the controller thinks - for the display and telemetry -------- */
typedef struct {
    float   roll, pitch;          /* estimate                    rad */
    float   roll_acc, pitch_acc;  /* accelerometer-only tilt     rad */
    float   roll_sp, pitch_sp;    /* angle setpoints             rad */
    float   p_sp, q_sp;           /* rate setpoints              rad/s */
    float   i_roll, i_pitch;      /* integrators                 motor units */
    uint8_t armed, fail;          /* FS_* of the last disarm or refusal */
    uint8_t saturated;            /* a motor hit 0 or 1 this step */
    uint32_t steps;
    uint32_t fails;               /* +1 at every failsafe or refusal  */
} ctrl_status_t;

#define CTRL_HZ           250.0f          /* control_step() is called at this rate */
#define CTRL_DT           (1.0f / CTRL_HZ)
#define CTRL_TILT_LIMIT   1.0472f         /* 60 degrees: failsafe                  */
#define CTRL_ARM_TILT     0.4363f         /* 25 degrees: refuse to arm beyond this */
#define CTRL_ARM_THROTTLE 0.05f           /* arm only below this throttle          */
#define CTRL_STALE_STEPS  5u              /* 20 ms with no new IMU sample          */
#define CTRL_SETTLE_STEPS 125u            /* 0.5 s of good samples before arming   */

extern ctrl_tune_t ctrl_tune;             /* defaults set by control_init()        */

void control_init(void);                  /* tuning to defaults, state to disarmed */
void control_reset(void);                 /* state only: estimate level, disarmed  */
void control_step(const sensors_t *in, actuators_t *out);
void control_status(ctrl_status_t *st);   /* a copy of the controller's view       */

#endif
