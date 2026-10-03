/*
 * control.c - the flight controller.   SOC3050 lesson 13
 *
 * THIS is the student's file.  It sees a sensors_t, writes an actuators_t,
 * and knows nothing else - not whether the drone is simulated, not what time
 * it is (it is called at exactly CTRL_HZ and counts on it).
 *
 *   gyro  ----------------+
 *                         v
 *   accel -> tilt --> [complementary filter] -> roll, pitch estimate
 *                                                     |
 *   stick -> angle setpoint -> [angle P] -> rate setpoint -> [rate PID] -> torque
 *                                                                           |
 *   throttle -------------------------------------------------> [X mixer] -> 4 motors
 *
 * plus the part every real flight controller is mostly made of: the rules for
 * when NOT to fly (arming, failsafes, saturation).
 *
 * Pure C, float (the M0+ emulates it - slide "The Budget" counts the cost).
 */
#include "control.h"
#include "fmath.h"

ctrl_tune_t ctrl_tune;

/* ---- the controller's memory -------------------------------------------- */
static struct {
    float    roll, pitch;           /* estimate, rad                          */
    float    roll_acc, pitch_acc;
    float    roll_sp, pitch_sp, p_sp, q_sp;
    float    i_roll, i_pitch, i_yaw;
    float    d_roll, d_pitch;       /* filtered D terms                       */
    float    p_prev, q_prev;
    uint32_t last_seq, stale;
    uint32_t good;                  /* consecutive trustworthy samples        */
    uint32_t steps, fails;
    uint8_t  started;               /* estimate initialised from accel        */
    uint8_t  armed, fail, need_release, saturated;
} s;

void control_reset(void)
{
    s.roll = s.pitch = s.roll_acc = s.pitch_acc = 0.0f;
    s.roll_sp = s.pitch_sp = s.p_sp = s.q_sp = 0.0f;
    s.i_roll = s.i_pitch = s.i_yaw = 0.0f;
    s.d_roll = s.d_pitch = s.p_prev = s.q_prev = 0.0f;
    s.last_seq = 0;  s.stale = 0;  s.steps = 0;  s.good = 0;   /* fails is NOT reset: it only counts up */
    s.started = 0;
    s.armed = 0;  s.fail = FS_NONE;  s.need_release = 0;  s.saturated = 0;
}

void control_init(void)
{
    ctrl_tune.alpha       = 0.98f;   /* filter time constant 0.98*4ms/0.02 = 0.2 s */
    ctrl_tune.angle_kp    = 9.0f;    /* 10 deg off -> 90 deg/s back                 */
    ctrl_tune.rate_kp     = 0.12f;   /* 1 rad/s off -> 12 % differential thrust     */
    ctrl_tune.rate_ki     = 0.15f;
    ctrl_tune.rate_kd     = 0.0008f;
    ctrl_tune.yaw_kp      = 0.15f;
    ctrl_tune.yaw_ki      = 0.05f;
    ctrl_tune.kp_scale    = 1.0f;
    ctrl_tune.max_rate    = 3.5f;    /* 200 deg/s                                   */
    ctrl_tune.max_out     = 0.25f;
    ctrl_tune.i_limit     = 0.10f;
    ctrl_tune.anti_windup = 1;
    ctrl_tune.mix_roll_sign = 1;
    control_reset();
}

/* A reading is plausible if it is finite and inside what an MPU6050 could
 * report at its widest ranges (2000 deg/s, 16 g).  NaN fails every compare,
 * so "!(x < lim)" catches it as well. */
static int plausible(const sensors_t *in)
{
    for (int i = 0; i < 3; i++) {
        if (!(f_abs(in->gyro[i])  < 35.0f))  { return 0; }
        if (!(f_abs(in->accel[i]) < 157.0f)) { return 0; }
    }
    return 1;
}

static void disarm(uint8_t why)
{
    s.armed = 0;
    s.fail = why;
    if (why != FS_NONE) { s.need_release = 1; s.fails++; }   /* switch off before re-arming */
}

/* ---- one rate-loop axis: PID on the rate error ------------------------- */
static float rate_pid(float sp, float meas, float *meas_prev, float *integ,
                      float *dfilt, int motors_saturated)
{
    const ctrl_tune_t *t = &ctrl_tune;
    float k = t->kp_scale;
    float e = sp - meas;

    /* D on the MEASUREMENT, not the error: a step in the setpoint would make
     * d(error)/dt a spike - "derivative kick".  Low-passed, because the gyro
     * is noisy and differentiation amplifies noise. */
    float d = -(meas - *meas_prev) * CTRL_HZ;
    *meas_prev = meas;
    *dfilt += 0.5f * (d - *dfilt);

    float u_free = k * t->rate_kp * e + t->rate_kd * *dfilt + *integ;
    float u = f_clamp(u_free, -t->max_out, t->max_out);

    /* Anti-windup: while the output is pinned, more integral cannot help and
     * will only have to be unwound later.  So stop integrating. */
    if (!t->anti_windup) {
        *integ += t->rate_ki * e * CTRL_DT;
    } else if (u == u_free && !motors_saturated) {
        *integ = f_clamp(*integ + t->rate_ki * e * CTRL_DT, -t->i_limit, t->i_limit);
    }
    return u;
}

void control_step(const sensors_t *in, actuators_t *out)
{
    const ctrl_tune_t *t = &ctrl_tune;
    s.steps++;

    /* ---- 1. Is the input to be trusted? ------------------------------- */
    uint8_t bad = FS_NONE;
    if (!in->imu_ok || !plausible(in)) { bad = FS_SENSOR; }
    if (in->seq == s.last_seq) {
        if (++s.stale >= CTRL_STALE_STEPS) { bad = FS_STALE; }
    } else {
        s.stale = 0;
        s.last_seq = in->seq;
    }

    s.good = (bad == FS_NONE) ? s.good + 1u : 0u;

    /* ---- 2. Estimate attitude: the complementary filter ----------------- */
    if (bad == FS_NONE) {
        float ax = in->accel[0], ay = in->accel[1], az = in->accel[2];
        s.roll_acc  = f_atan2(-ay, -az);                       /* from gravity  */
        s.pitch_acc = f_atan2(ax, f_sqrt(ay * ay + az * az));
        if (!s.started) {                    /* first sample: trust the accel */
            s.roll = s.roll_acc;  s.pitch = s.pitch_acc;  s.started = 1;
        }
        /* Gyro: integrate the rate (small-angle: roll' = p, pitch' = q). */
        float a = t->alpha;
        s.roll  = a * (s.roll  + in->gyro[0] * CTRL_DT) + (1.0f - a) * s.roll_acc;
        s.pitch = a * (s.pitch + in->gyro[1] * CTRL_DT) + (1.0f - a) * s.pitch_acc;
    }

    /* ---- 3. Arm, disarm, failsafe --------------------------------------- */
    float tilt = f_abs(s.roll) > f_abs(s.pitch) ? f_abs(s.roll) : f_abs(s.pitch);
    if (!in->rc_arm) {
        if (s.armed) { disarm(FS_NONE); }
        s.need_release = 0;
    } else if (!s.armed) {
        /* Wait for CTRL_SETTLE_STEPS good samples first: right after power-up
         * the estimate (and the sensor's own filter) has not caught up with
         * the truth, and a level-looking estimate would arm a tilted drone. */
        if (!s.need_release && s.good >= CTRL_SETTLE_STEPS) {
            if (in->rc_throttle >= CTRL_ARM_THROTTLE) { disarm(FS_THROTTLE); }
            else if (bad != FS_NONE)                  { disarm(bad); }
            else if (tilt > CTRL_ARM_TILT)            { disarm(FS_TILT); }
            else {
                s.armed = 1;  s.fail = FS_NONE;
                s.i_roll = s.i_pitch = s.i_yaw = 0.0f;
            }
        }
    } else {
        if (bad != FS_NONE)              { disarm(bad); }
        else if (tilt > CTRL_TILT_LIMIT) { disarm(FS_TILT); }
    }

    if (!s.armed || in->rc_throttle < CTRL_ARM_THROTTLE) {
        /* Disarmed, or armed on the ground: motors follow the throttle (or
         * stop), and nothing integrates - an integrator that winds up while
         * the drone sits on the grass flips it on take-off. */
        float m = s.armed ? f_clamp(in->rc_throttle, 0.0f, 1.0f) : 0.0f;
        for (int i = 0; i < 4; i++) { out->motor[i] = m; }
        out->armed = s.armed;
        s.i_roll = s.i_pitch = s.i_yaw = 0.0f;
        s.roll_sp = in->rc_roll;  s.pitch_sp = in->rc_pitch;
        s.p_sp = s.q_sp = 0.0f;
        s.saturated = 0;
        return;
    }

    /* ---- 4. Angle loop (outer, P): angle error -> rate setpoint ---------- */
    s.roll_sp  = in->rc_roll;
    s.pitch_sp = in->rc_pitch;
    s.p_sp = f_clamp(t->angle_kp * (s.roll_sp  - s.roll),  -t->max_rate, t->max_rate);
    s.q_sp = f_clamp(t->angle_kp * (s.pitch_sp - s.pitch), -t->max_rate, t->max_rate);

    /* ---- 5. Rate loop (inner, PID): rate error -> torque command --------- */
    float u_r = rate_pid(s.p_sp, in->gyro[0], &s.p_prev, &s.i_roll,  &s.d_roll,  s.saturated);
    float u_p = rate_pid(s.q_sp, in->gyro[1], &s.q_prev, &s.i_pitch, &s.d_pitch, s.saturated);

    float e_y = in->rc_yaw_rate - in->gyro[2];
    float u_y = f_clamp(t->yaw_kp * e_y + s.i_yaw, -0.1f, 0.1f);
    if (!s.saturated) { s.i_yaw = f_clamp(s.i_yaw + t->yaw_ki * e_y * CTRL_DT, -0.05f, 0.05f); }

    /* ---- 6. The X mixer: three torques and a thrust -> four motors ------- */
    float th = in->rc_throttle;
    float r  = u_r * (float)t->mix_roll_sign;
    float m[4];
    m[0] = th + r + u_p - u_y;      /* FL, CW   */
    m[1] = th - r + u_p + u_y;      /* FR, CCW  */
    m[2] = th - r - u_p - u_y;      /* RR, CW   */
    m[3] = th + r - u_p + u_y;      /* RL, CCW  */

    s.saturated = 0;
    for (int i = 0; i < 4; i++) {
        if (m[i] < 0.0f || m[i] > 1.0f) { s.saturated = 1; }
        out->motor[i] = f_clamp(m[i], 0.0f, 1.0f);
    }
    out->armed = 1;
}

void control_status(ctrl_status_t *st)
{
    st->roll = s.roll;            st->pitch = s.pitch;
    st->roll_acc = s.roll_acc;    st->pitch_acc = s.pitch_acc;
    st->roll_sp = s.roll_sp;      st->pitch_sp = s.pitch_sp;
    st->p_sp = s.p_sp;            st->q_sp = s.q_sp;
    st->i_roll = s.i_roll;        st->i_pitch = s.i_pitch;
    st->armed = s.armed;          st->fail = s.fail;
    st->saturated = s.saturated;  st->steps = s.steps;  st->fails = s.fails;
}
