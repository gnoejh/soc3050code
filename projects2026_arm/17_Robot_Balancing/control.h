/*
 * control.h - the contract between the robot and its brain
 *
 *     void control_step(const sensors_t *in, actuators_t *out);
 *
 * This is the Software-In-The-Loop (SITL) boundary every application lesson
 * shares.  On one side, the WORLD (world.c): a simulated robot that turns a
 * motor voltage into motion and motion into sensor readings.  On the other,
 * the CONTROLLER (control.c): it sees ONLY what a real robot's firmware would
 * see - a gyro, an accelerometer, an encoder count, a command - and answers
 * with a voltage.  It never reads the true tilt.  Replace world.c with an
 * MPU6050 driver, an encoder timer and a motor PWM, and control.c does not
 * change by one line.  That is the point of the boundary, and it is how
 * ArduPilot and PX4 are developed.
 *
 * Pure C, no registers: the same file compiles into the firmware and into
 * host/sitl.c, which is how the numbers in this lesson were measured.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* What the robot's own sensors say - and nothing else. */
typedef struct {
    float   gyro;      /* pitch rate from the IMU's gyro, rad/s - with bias and noise  */
    float   acc_f;     /* accelerometer, along the body's forward axis, m/s^2          */
    float   acc_u;     /* accelerometer, along the body's up axis, m/s^2               */
    int32_t enc;       /* wheel encoder, counts: the wheel's angle RELATIVE TO THE BODY */
    float   v_cmd;     /* the driver's speed command (joystick), m/s                   */
} sensors_t;

/* What the controller may change. */
typedef struct {
    float   volts;     /* motor voltage, both motors; the H-bridge clips it at +-VBAT */
} actuators_t;

enum { CTRL_PID_TILT = 0, CTRL_CASCADE = 1, CTRL_LQR = 2, CTRL_MODES = 3 };

/* What the controller believes - for the display and telemetry ONLY.  The
 * controller computes it from sensors_t; nothing outside reads it to decide
 * anything. */
typedef struct {
    float th;          /* tilt estimate (complementary filter), rad          */
    float th_acc;      /* the accelerometer's tilt on its own, rad           */
    float x, xd;       /* wheel position and speed estimate, m and m/s       */
    float x_ref;       /* where it is trying to be, m                        */
    float v_ref;       /* how fast it is trying to go: v_cmd, slew-limited   */
    float th_ref;      /* the cascade's tilt set-point, rad                  */
} estimate_t;

void        control_init(void);                 /* zero filters, integrators   */
void        control_step(const sensors_t *in, actuators_t *out);
void        control_set_mode(int mode);         /* also resets the integrators */
int         control_mode(void);
const char *control_name(int mode);
const estimate_t *control_estimate(void);

/* Lab knobs.  The complementary filter's alpha, 0..1: 1 = gyro only,
 * 0 = accelerometer only.  Default 0.98. */
void        control_set_alpha(float a);
float       control_alpha(void);

/* The gain lqr.py printed, so the firmware can show it and the host test can
 * check it against a fresh run of lqr.py. */
extern const float LQR_K[4];

#endif
