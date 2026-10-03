/*
 * control.c - THE STUDENT'S FILE: a PID line follower
 *
 * It sees seven numbers, two encoder counts and a knob (control.h), and
 * answers with two wheel speeds.  Four steps, every 10 ms:
 *
 *   1. WHERE IS THE LINE?  A weighted average of the sensor positions,
 *      weighted by how dark each one reads: one number, in mm, + = left.
 *   2. HOW HARD TO TURN?   PID on that number.  P steers toward the line;
 *      D damps the swing, because the robot has mass and the motors lag;
 *      I removes a steady offset (a bent chassis, a weak motor).
 *   3. HOW FAST?           The knob sets the speed on a straight.  Far off
 *      the line usually means "in a corner", so slow down in proportion -
 *      the CORNER SLOW-DOWN.  The friction circle (world.c) punishes
 *      anyone who does not.
 *   4. LINE LOST?          If the line was near the middle when it vanished,
 *      it is probably a GAP: keep going straight for a moment.  If it was at
 *      one edge, the robot overshot a corner: spin toward that side.
 *
 * The gains below were tuned on the host (host/sitl.c) and are the "default
 * gains" every measured number in the README refers to.  Lab Part 3 asks
 * you to beat them.
 */
#include "control.h"
#include "fmath.h"

ctl_gains_t ctl_gains = {
    .kp    = 20.0f,
    .ki    = 0.0f,
    .kd    = 1.00f,
    .dfilt = 0.5f,
    .slow  = 0.5f,
    .thr   = 300.0f,
};

ctl_state_t ctl_state;

#define HALF_BAR_MM   ((float)((LINE_SENSORS - 1) / 2 * SENSOR_PITCH_MM))   /* 39 */
#define MIN_WEIGHT    60.0f     /* less tape than this in total: line lost      */
#define CROSS_COUNT   5         /* this many sensors dark at once: a crossing   */
#define GAP_POS_MM    12.0f     /* lost while the line was this near centre: a gap */
#define COAST_MS      400u      /* drive straight this long through a gap       */
#define SEARCH_MMS    500.0f    /* wheel speed while spinning to find the line  */
#define I_LIMIT       300.0f    /* the I term may add at most this, mm/s        */

static float    e_prev, d_filt, i_acc, last_pos;
static uint32_t t_prev, lost_ms;
static uint8_t  have_prev;

void control_reset(void)
{
    e_prev = d_filt = i_acc = last_pos = 0.0f;
    t_prev = lost_ms = 0u;
    have_prev = 0u;
    ctl_state.pos = ctl_state.d = 0.0f;
    ctl_state.lost = ctl_state.cross = 0u;
}

static int16_t clamp_mms(float v)
{
    return (int16_t)f_clamp(v, -(float)WHEEL_MAX_MMS, (float)WHEEL_MAX_MMS);
}

void control_step(const sensors_t *in, actuators_t *out)
{
    const ctl_gains_t *g = &ctl_gains;

    /* time since the last call - from the sensor timestamps, not assumed */
    float dt = 1.0f / (float)CONTROL_HZ;
    if (have_prev && in->t_ms > t_prev && in->t_ms - t_prev < 100u) {
        dt = (float)(in->t_ms - t_prev) * 0.001f;
    }
    t_prev = in->t_ms;

    /* 1. where is the line? */
    float sw = 0.0f, swy = 0.0f;
    int   dark = 0;
    for (int i = 0; i < LINE_SENSORS; i++) {
        float w = (float)in->line[i] - g->thr;
        if (w <= 0.0f) { continue; }
        float y = (float)((i - (LINE_SENSORS - 1) / 2) * SENSOR_PITCH_MM);
        sw  += w;
        swy += w * y;
        if (in->line[i] > 600u) { dark++; }
    }
    float base = (float)in->knob * 2.0f;                /* knob 0..1000 -> 0..2000 mm/s */

    /* 4 (first, because it decides whether 1-3 apply). Line lost? */
    if (sw < MIN_WEIGHT) {
        lost_ms += (uint32_t)(dt * 1000.0f + 0.5f);
        if (f_abs(last_pos) < GAP_POS_MM && lost_ms <= COAST_MS) {
            ctl_state.lost = 1;                         /* a gap: straight on   */
            out->left = out->right = clamp_mms(0.7f * base);
        } else {
            ctl_state.lost = 2;                         /* overshot: spin back  */
            float turn = last_pos >= 0.0f ? SEARCH_MMS : -SEARCH_MMS;
            out->left  = clamp_mms(-turn);
            out->right = clamp_mms(turn);
        }
        have_prev = 0u;                                 /* D restarts afresh    */
        return;
    }
    lost_ms = 0u;
    ctl_state.lost = 0u;

    float pos;
    if (dark >= CROSS_COUNT) {
        /* Most of the bar is dark: a CROSSING, not a line.  The average says
         * "centre", which is the right answer - but do not let the jump
         * into D or I.  Straight on, at the last steering. */
        ctl_state.cross = 1u;
        pos = e_prev;
    } else {
        ctl_state.cross = 0u;
        pos = swy / sw;
    }
    last_pos = pos;

    /* 2. PID.  e is the error; we want the line in the middle, at 0. */
    float e = pos;
    float d_raw = have_prev ? (e - e_prev) / dt : 0.0f;
    d_filt += g->dfilt * (d_raw - d_filt);               /* first-order low-pass */
    if (g->ki != 0.0f) {
        float lim = I_LIMIT / f_abs(g->ki);              /* anti-windup */
        i_acc = f_clamp(i_acc + e * dt, -lim, lim);
    }
    e_prev = e;
    have_prev = 1u;
    float turn = g->kp * e + g->ki * i_acc + g->kd * d_filt;

    /* 3. how fast: slow down as the line slides toward the edge of the bar */
    float v = base * (1.0f - g->slow * f_clamp(f_abs(e) / HALF_BAR_MM, 0.0f, 1.0f));

    out->left  = clamp_mms(v - turn);
    out->right = clamp_mms(v + turn);
    ctl_state.pos = pos;
    ctl_state.d   = d_filt;
}
