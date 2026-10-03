/*
 * control.c - three ways to keep a robot upright, and how it knows its tilt
 *
 * Called every CTRL_DT (5 ms, 200 Hz) with the sensors, and nothing else.
 * Every call does the same two things:
 *
 *   1. ESTIMATE  what the robot cannot measure directly: its tilt (a
 *                complementary filter of gyro and accelerometer) and where
 *                the axle is (the encoder, corrected for the tilt).
 *   2. CONTROL   turn that estimate into a motor voltage, by one of
 *
 *        PID tilt   balance by tilt alone.  It stays up - and wanders off.
 *        cascade    an outer loop on position and speed chooses the tilt
 *                   to hold; the inner tilt loop holds it.  Two PIDs.
 *        LQR        one line: u = -K x on the whole state at once, with K
 *                   computed offline by host/lqr.py from the same model.
 *
 * Pure C and float.  No registers, no world.h: what it knows of the robot
 * comes through sensors_t, exactly as on real hardware.
 */
#include "control.h"
#include "params.h"
#include "fmath.h"

/* ---- the LQR gain, pasted from host/lqr.py --------------------------------
 * Q = diag(30, 2, 50, 0.5), R = 1, CTRL_DT = 5 ms, no payload.
 * Re-run  python host/lqr.py  after changing params.h, and paste its last
 * line here.  host/run.sh runs  lqr.py --check  so a stale K fails the test. */
const float LQR_K[4] = { -5.1239f, -15.255f, -21.262f, -2.4627f };

/* ---- PID gains (tuned by hand against host/sitl.c - Lab Part 2) --------
 * Each is #ifndef-guarded so the host can try other values without editing
 * this file:  gcc -DCAS_KVX=0.4f ...  (the firmware build never sets them). */
#ifndef TILT_KP
#define TILT_KP     150.0f     /* V per rad of tilt: high, to delay the run-away   */
#endif
#ifndef TILT_KI
#define TILT_KI     50.0f      /* V per rad*s                                      */
#endif
#ifndef TILT_KD
#define TILT_KD     4.0f       /* V per rad/s                                      */
#endif
#ifndef CAS_KP
#define CAS_KP      25.0f      /* inner loop: V per rad of tilt error              */
#endif
#ifndef CAS_KD
#define CAS_KD      2.0f       /* inner loop: V per rad/s                          */
#endif
#ifndef CAS_KPX
#define CAS_KPX     0.20f      /* rad of lean per m of position error              */
#endif
#ifndef CAS_KVX
#define CAS_KVX     0.60f      /* rad of lean per m/s of speed error               */
#endif
#ifndef CAS_LEAN
#define CAS_LEAN    0.30f      /* never ask for more than ~17 degrees of lean      */
#endif

#define CMD_ACCEL   0.5f       /* the joystick's speed command slews at 0.5 m/s^2  */
#define TRIP_ANGLE  0.87f      /* 50 degrees: it has fallen - cut the motors       */
#define XD_FILTER   0.25f      /* speed low-pass: new = old + 0.25 (raw - old)     */

static int        mode = CTRL_LQR;
static float      alpha = 0.98f;
static estimate_t est;
static float      integ;              /* the PID's integral of tilt error, rad*s */
static float      x_prev;
static uint8_t    started;

void control_init(void)
{
    est = (estimate_t){0};
    integ = x_prev = 0.0f;
    started = 0;
}

void control_set_mode(int m)       { mode = (m >= 0 && m < CTRL_MODES) ? m : CTRL_LQR; integ = 0.0f; }
int  control_mode(void)            { return mode; }
const estimate_t *control_estimate(void) { return &est; }
void control_set_alpha(float a)    { alpha = f_clamp(a, 0.0f, 1.0f); }
float control_alpha(void)          { return alpha; }

const char *control_name(int m)
{
    static const char *const names[CTRL_MODES] = { "PID tilt", "cascade", "LQR" };
    return (m >= 0 && m < CTRL_MODES) ? names[m] : "?";
}

/* ============================================================================
 *  1. Estimation
 * ============================================================================ */
static void estimate(const sensors_t *in)
{
    /* The accelerometer's idea of the tilt: the direction of gravity in the
     * body's axes.  Right on average, wrong whenever the robot accelerates,
     * and noisy. */
    est.th_acc = f_atan2(-in->acc_f, in->acc_u);

    /* The gyro's idea: integrate the rate.  Smooth and right over short
     * times, but any bias integrates into an error that grows forever.
     *
     * The COMPLEMENTARY FILTER trusts each where it is good: mostly the gyro
     * (alpha), corrected a little toward the accelerometer every step.  With
     * alpha = 0.98 at 200 Hz the crossover is a time constant of
     * dt * alpha / (1 - alpha) = 0.245 s. */
    if (!started) {
        est.th = est.th_acc;                       /* first call: no history */
    } else {
        est.th = alpha * (est.th + in->gyro * CTRL_DT) + (1.0f - alpha) * est.th_acc;
    }

    /* Position.  The encoder sits on the motor, which is bolted to the body,
     * so it counts the wheel's turn RELATIVE TO THE BODY.  The wheel's turn
     * over the ground is that plus the body's tilt. */
    float wheel = (float)in->enc * (F_2PI / ENC_CPR) + est.th;
    est.x = BOT_R * wheel;
    if (!started) { x_prev = est.x; }
    /* Speed by differencing.  One encoder count in 5 ms is 0.03 m/s, so the
     * raw difference is lumpy; a first-order low-pass smooths it. */
    float raw = (est.x - x_prev) * (1.0f / CTRL_DT);
    x_prev = est.x;
    est.xd += XD_FILTER * (raw - est.xd);
    started = 1;
}

/* ============================================================================
 *  2. Control
 * ============================================================================ */
void control_step(const sensors_t *in, actuators_t *out)
{
    estimate(in);

    /* The driver's command, made gentle.  A stick slammed forward asks for
     * 0.4 m/s NOW; a balancing robot can only get there by leaning first, and
     * a step would make it lean hard.  So the target speed ramps at CMD_ACCEL,
     * and the target position integrates the target speed. */
    float dv = in->v_cmd - est.v_ref, lim = CMD_ACCEL * CTRL_DT;
    est.v_ref += f_clamp(dv, -lim, lim);
    est.x_ref += est.v_ref * CTRL_DT;

    /* FAILSAFE before any control law: a robot lying on the floor with its
     * wheels at full power is how people get hurt.  Past 50 degrees there is
     * nothing left to save - cut the motors and forget the integral. */
    if (est.th > TRIP_ANGLE || est.th < -TRIP_ANGLE) {
        integ = 0.0f;
        out->volts = 0.0f;
        return;
    }

    float u;
    switch (mode) {
    case CTRL_PID_TILT:
        /* Keep the tilt at zero.  Nothing here knows where the wheels are. */
        integ = f_clamp(integ + est.th * CTRL_DT, -0.1f, 0.1f);
        u = TILT_KP * est.th + TILT_KI * integ + TILT_KD * in->gyro;
        est.th_ref = 0.0f;
        break;

    case CTRL_CASCADE: {
        /* Outer loop: to move forward you must first LEAN forward.  So the
         * position and speed errors choose a lean... */
        float lean = CAS_KPX * (est.x_ref - est.x) + CAS_KVX * (est.v_ref - est.xd);
        est.th_ref = f_clamp(lean, -CAS_LEAN, CAS_LEAN);
        /* ...and the inner loop, a PD on the tilt ERROR, holds that lean. */
        u = CAS_KP * (est.th - est.th_ref) + CAS_KD * in->gyro;
        break;
    }

    default: /* CTRL_LQR */
        /* u = -K x.  Four multiplies.  All the cleverness is in K. */
        est.th_ref = 0.0f;
        u = -(LQR_K[0] * (est.x - est.x_ref)
            + LQR_K[1] * (est.xd - est.v_ref)
            + LQR_K[2] * est.th
            + LQR_K[3] * in->gyro);
        break;
    }

    out->volts = f_clamp(u, -VBAT, VBAT);
}
