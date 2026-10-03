/*
 * host/sitl.c - Software-In-The-Loop test of lesson 13's flight controller
 *
 * Links the SAME world.c and control.c the firmware links, and flies them on
 * the PC at the firmware's rates: world 500 Hz, control 250 Hz.  No Wokwi, no
 * board, no RTOS - just the two pure-C halves of the lesson and a loop.
 *
 * It measures, prints a table, and exits non-zero if any check fails:
 *   1  roll and pitch step responses  (rise, overshoot, settling)
 *   2  gust rejection                 (peak tilt, recovery time)
 *   3  failsafes                      (throttle-high arm, tilted arm, tilt,
 *                                      NaN, stale sensor)
 *   4  estimator accuracy vs alpha    (gyro bias + motor vibration)
 *   5  rate-loop gain sweep           (the knob, 0.25x .. 4x)
 *   6  anti-windup on vs off          (held for 2 s, then released)
 *   7  a mixer with the roll sign flipped
 *
 * Build and run:  host/run.sh   or   host\run.bat
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../world.h"
#include "../control.h"

#define DEG (180.0f / 3.14159265f)
#define RAD (3.14159265f / 180.0f)
#define SEED 12345u

static int failures = 0, checks = 0;

static void check(int ok, const char *what)
{
    checks++;
    if (!ok) { failures++; printf("    ** FAIL: %s\n", what); }
}

static const char *fs_name(int f)
{
    static const char *n[] = { "none", "TILT", "SENSOR", "STALE", "THROTTLE" };
    return (f >= 0 && f <= 4) ? n[f] : "?";
}

/* ---- the harness: one world, one controller, a pilot -------------------- */
typedef struct {
    world_t     w;
    sensors_t   in;
    actuators_t out;
    uint32_t    n;          /* world steps so far          */
    int         freeze_seq; /* 1 = the IMU driver has died */
} sim_t;

/* `sitl score ...` flies YOUR gains instead of control_init()'s defaults */
static int         use_tune = 0;
static ctrl_tune_t tune_override;

static void sim_init(sim_t *s, uint32_t seed)
{
    memset(s, 0, sizeof *s);
    world_init(&s->w, seed);
    control_init();
    if (use_tune) { ctrl_tune = tune_override; }
    world_sense(&s->w, &s->in);
}

/* One world step (2 ms); every second one, a control step (4 ms).  The
 * firmware's tasks do exactly this, with the RTOS keeping the time. */
static void sim_tick(sim_t *s)
{
    world_step(&s->w, s->out.motor);
    if (!s->freeze_seq) { world_sense(&s->w, &s->in); }
    if ((++s->n & 1u) == 0) { control_step(&s->in, &s->out); }
}

static float true_axis(const sim_t *s, int axis)
{
    float r, p;
    world_euler(&s->w, &r, &p, 0);
    return axis == 0 ? r : p;
}

static float true_tilt(const sim_t *s)
{
    float r = fabsf(true_axis(s, 0)), p = fabsf(true_axis(s, 1));
    return r > p ? r : p;
}

/* Arm with the throttle low, then bring it up to 0.35 over half a second and
 * hold 1 s - what Main.c's pilot logic does when you press A. */
static int sim_takeoff(sim_t *s)
{
    s->in.rc_arm = 1;  s->in.rc_throttle = 0.0f;
    for (int i = 0; i < 1000 && !s->out.armed; i++) { sim_tick(s); }
    for (int i = 0; i < 250; i++) { s->in.rc_throttle = 0.35f * (float)i / 250.0f; sim_tick(s); }
    s->in.rc_throttle = 0.35f;
    for (int i = 0; i < 500; i++) { sim_tick(s); }
    return s->out.armed;
}

/* ---- 1 and 5: a step response -------------------------------------------- */
typedef struct { float rise, over, settle, err, osc; int crashed, armed, fail; } step_t;

static step_t step_response(int axis, float step_deg, float scale)
{
    sim_t s;
    sim_init(&s, SEED);
    ctrl_tune.kp_scale = scale;
    step_t r = { -1, 0, -1, 0, 0, 0, 0, 0 };
    if (!sim_takeoff(&s)) { return r; }

    const float target = step_deg * RAD;
    if (axis == 0) { s.in.rc_roll = target; } else { s.in.rc_pitch = target; }
    float t10 = -1, t90 = -1, peak = 0, last_out = 0, sumsq = 0;
    int nsq = 0;
    const float band = 0.05f * fabsf(target);              /* +-5 %: 1 deg */
    for (int i = 0; i < 1500; i++) {                         /* 3 s          */
        sim_tick(&s);
        float t = (float)(i + 1) * WORLD_DT;
        float y = true_axis(&s, axis);
        float frac = y / target;
        if (t10 < 0 && frac >= 0.1f) { t10 = t; }
        if (t90 < 0 && frac >= 0.9f) { t90 = t; }
        if (frac > peak) { peak = frac; }
        if (fabsf(y - target) > band) { last_out = t; }
        if (t > 2.0f) { float w = s.w.w[axis]; sumsq += w * w; nsq++; }
        if (s.w.crashed) { r.crashed = 1; }
    }
    r.rise   = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1;
    r.over   = peak > 1.0f ? (peak - 1.0f) * 100.0f : 0.0f;
    r.settle = last_out;
    r.err    = (true_axis(&s, axis) - target) * DEG;
    r.osc    = sqrtf(sumsq / (float)(nsq ? nsq : 1)) * DEG;
    r.armed  = s.out.armed;
    ctrl_status_t st;
    control_status(&st);
    r.fail   = st.fail;
    ctrl_tune.kp_scale = 1.0f;
    return r;
}

static void print_step(const char *name, step_t r)
{
    printf("  %-22s %7.3f %9.1f %9.3f %8.2f %8.2f   %s%s\n", name, r.rise, r.over,
           r.settle, r.err, r.osc, r.armed ? "armed" : "DISARMED",
           r.crashed ? ", CRASHED" : "");
}

/* ---- 2: gust --------------------------------------------------------------- */
static void test_gust(void)
{
    sim_t s;
    sim_init(&s, SEED);
    sim_takeoff(&s);
    world_kick(&s.w, 4.0f, -2.0f);         /* 229 deg/s roll, -115 deg/s pitch */
    float peak = 0, recover = -1;
    for (int i = 0; i < 1000; i++) {       /* 2 s */
        sim_tick(&s);
        float t = (float)(i + 1) * WORLD_DT, tilt = true_tilt(&s);
        if (tilt > peak) { peak = tilt; }
        if (tilt > 3.0f * RAD) { recover = t; }
    }
    printf("\n2. Gust (button B): kick p = +4 rad/s, q = -2 rad/s while hovering level\n");
    printf("  peak tilt %.1f deg, back inside 3 deg after %.3f s, %s%s\n", peak * DEG,
           recover, s.out.armed ? "still armed" : "DISARMED", s.w.crashed ? ", CRASHED" : "");
    check(peak < 30.0f * RAD, "gust: peak tilt under 30 deg");
    check(recover >= 0 && recover < 1.0f, "gust: recovered within 1 s");
    check(s.out.armed && !s.w.crashed, "gust: still armed, not crashed");
}

/* ---- 3: failsafes ----------------------------------------------------------- */
static void test_failsafes(void)
{
    sim_t s;
    ctrl_status_t st;
    printf("\n3. Failsafes\n");
    printf("  %-34s %-9s %-9s %s\n", "case", "armed", "reason", "time");

    /* a. arm with the throttle up: refused */
    sim_init(&s, SEED);
    s.in.rc_arm = 1;  s.in.rc_throttle = 0.5f;
    for (int i = 0; i < 1000; i++) { sim_tick(&s); }
    control_status(&st);
    printf("  %-34s %-9s %-9s\n", "arm with throttle 0.5", st.armed ? "yes" : "no", fs_name(st.fail));
    check(!st.armed && st.fail == FS_THROTTLE, "refuse to arm with throttle high");

    /* b. arm while tilted 30 deg (held by hand): refused */
    sim_init(&s, SEED);
    s.w.q[0] = cosf(15.0f * RAD);  s.w.q[1] = sinf(15.0f * RAD);
    s.w.locked = 1;
    s.in.rc_arm = 1;
    for (int i = 0; i < 1000; i++) { sim_tick(&s); }
    control_status(&st);
    printf("  %-34s %-9s %-9s\n", "arm while tilted 30 deg", st.armed ? "yes" : "no", fs_name(st.fail));
    check(!st.armed && st.fail == FS_TILT, "refuse to arm when tilted");

    /* c. a hard hit: tilt passes 60 deg -> disarm */
    sim_init(&s, SEED);
    sim_takeoff(&s);
    world_kick(&s.w, 15.0f, 0.0f);
    float t_fs = -1, tilt_at = 0;
    for (int i = 0; i < 500; i++) {
        sim_tick(&s);
        if (t_fs < 0 && !s.out.armed) { t_fs = (float)(i + 1) * WORLD_DT; tilt_at = true_tilt(&s); }
    }
    control_status(&st);
    printf("  %-34s %-9s %-9s %.0f ms after the hit (true tilt %.0f deg)\n",
           "kick 15 rad/s (860 deg/s)", st.armed ? "yes" : "no", fs_name(st.fail), t_fs * 1000.0f, tilt_at * DEG);
    check(!st.armed && st.fail == FS_TILT && t_fs > 0 && t_fs < 0.2f, "tilt failsafe within 200 ms");

    /* d. one NaN from the gyro */
    sim_init(&s, SEED);
    sim_takeoff(&s);
    s.in.gyro[0] = NAN;
    s.freeze_seq = 1;  s.in.seq++;     /* deliver it as a fresh sample */
    s.n = 1;                           /* next tick runs the controller */
    sim_tick(&s);
    control_status(&st);
    printf("  %-34s %-9s %-9s first control step\n", "gyro reads NaN once", st.armed ? "yes" : "no", fs_name(st.fail));
    check(!st.armed && st.fail == FS_SENSOR, "NaN disarms immediately");

    /* e. the IMU stops updating */
    sim_init(&s, SEED);
    sim_takeoff(&s);
    s.freeze_seq = 1;
    float t_st = -1;
    for (int i = 0; i < 100; i++) {
        sim_tick(&s);
        if (t_st < 0 && !s.out.armed) { t_st = (float)(i + 1) * WORLD_DT; }
    }
    control_status(&st);
    printf("  %-34s %-9s %-9s %.0f ms after the last sample\n", "IMU sample counter freezes",
           st.armed ? "yes" : "no", fs_name(st.fail), t_st * 1000.0f);
    check(!st.armed && st.fail == FS_STALE && t_st > 0 && t_st <= 0.030f, "stale IMU disarms within 30 ms");

    /* f. re-arm needs the switch released first */
    s.freeze_seq = 0;
    for (int i = 0; i < 300; i++) { s.in.rc_throttle = 0.0f; sim_tick(&s); }
    int still = s.out.armed;
    s.in.rc_arm = 0;  for (int i = 0; i < 10; i++) { sim_tick(&s); }
    s.in.rc_arm = 1;  for (int i = 0; i < 10; i++) { sim_tick(&s); }
    printf("  %-34s %-9s\n", "after failsafe: switch held on", still ? "yes" : "no");
    printf("  %-34s %-9s\n", "after failsafe: off, then on", s.out.armed ? "yes" : "no");
    check(!still && s.out.armed, "re-arm only after the switch is cycled");
}

/* ---- 4: the estimator, swept over alpha ------------------------------------ */
static void test_alpha(void)
{
    static const float alphas[] = { 0.0f, 0.90f, 0.98f, 0.995f, 1.0f };
    float rms_at[5], max_at[5];
    printf("\n4. Estimator vs alpha: 30 s hover at 0 deg, gyro bias +0.8/-0.6 deg/s, vibration on\n");
    printf("  %-7s %13s %13s %13s %14s   %s\n", "alpha", "est err RMS", "est err max",
           "true tilt max", "est noise RMS", "outcome");
    for (int k = 0; k < 5; k++) {
        sim_t s;
        sim_init(&s, SEED);
        sim_takeoff(&s);
        ctrl_tune.alpha = alphas[k];
        double se = 0, sn = 0, prev_est = 0;
        float emax = 0, tmax = 0;
        int n = 0, crash_t = -1;
        ctrl_status_t st;
        for (int i = 0; i < 15000; i++) {                 /* 30 s */
            sim_tick(&s);
            if (s.n & 1u) { continue; }                   /* sample after each control step */
            control_status(&st);
            float e = st.roll - true_axis(&s, 0);
            se += (double)e * e;
            sn += (st.roll - prev_est) * (st.roll - prev_est);
            prev_est = st.roll;
            if (fabsf(e) > emax) { emax = fabsf(e); }
            if (true_tilt(&s) > tmax) { tmax = true_tilt(&s); }
            if ((s.w.crashed || !s.out.armed) && crash_t < 0) { crash_t = i; }
            n++;
        }
        rms_at[k] = (float)sqrt(se / n) * DEG;
        max_at[k] = emax * DEG;
        char outcome[48];
        if (crash_t >= 0) {
            snprintf(outcome, sizeof outcome, "%s at %.1f s", s.w.crashed ? "CRASHED" : "failsafe",
                     (float)crash_t * WORLD_DT);
        } else {
            snprintf(outcome, sizeof outcome, "flew 30 s");
        }
        /* est noise: RMS change of the estimate per control step - how much
         * the horizon jitters, independent of whether it is right */
        printf("  %-7.3f %10.2f deg %10.2f deg %10.1f deg %10.3f deg   %s\n", alphas[k], rms_at[k],
               max_at[k], tmax * DEG, (float)sqrt(sn / n) * DEG, outcome);
        if (k == 2) {
            check(rms_at[k] < 1.0f && max_at[k] < 3.0f && crash_t < 0, "alpha 0.98: error RMS < 1 deg, max < 3 deg");
        }
        ctrl_tune.alpha = 0.98f;
    }
    check(max_at[4] > 5.0f * max_at[2], "alpha 1 (gyro only) drifts far more than 0.98");
    check(rms_at[0] > 2.0f * rms_at[2], "alpha 0 (accel only) is noisier than 0.98");
}

/* ---- 6: anti-windup ---------------------------------------------------------- */
static float windup_run(int aw, float *t_back, int *fail)
{
    sim_t s;
    sim_init(&s, SEED);
    ctrl_tune.anti_windup = (uint8_t)aw;
    sim_takeoff(&s);
    s.w.locked = 1;                         /* a hand holds it level...     */
    s.in.rc_roll = 30.0f * RAD;             /* ...while the pilot asks 30   */
    for (int i = 0; i < 1000; i++) { sim_tick(&s); }    /* 2 s */
    s.w.locked = 0;                         /* let go                        */
    float peak = 0;
    *t_back = -1;
    for (int i = 0; i < 1500; i++) {
        sim_tick(&s);
        float r = true_axis(&s, 0);
        if (r > peak) { peak = r; }
        if (fabsf(r - 30.0f * RAD) > 2.0f * RAD) { *t_back = (float)(i + 1) * WORLD_DT; }
    }
    ctrl_status_t st;
    control_status(&st);
    *fail = st.armed ? -1 : st.fail;
    ctrl_tune.anti_windup = 1;
    return peak;
}

static void test_windup(void)
{
    float t_on, t_off;
    int f_on, f_off;
    float p_on  = windup_run(1, &t_on, &f_on);
    float p_off = windup_run(0, &t_off, &f_off);
    printf("\n6. Anti-windup: held level 2 s with a 30 deg roll setpoint, then released\n");
    printf("  %-16s %10s %16s   %s\n", "anti-windup", "peak roll", "within 2 deg by", "end state");
    printf("  %-16s %6.1f deg %14.3f s   %s\n", "on",  p_on  * DEG, t_on,  f_on  < 0 ? "armed" : fs_name(f_on));
    printf("  %-16s %6.1f deg %14.3f s   %s\n", "off", p_off * DEG, t_off, f_off < 0 ? "armed" : fs_name(f_off));
    check(p_on < 36.0f * RAD && f_on < 0, "with anti-windup: overshoot under 6 deg, still armed");
    check(p_off > p_on + 10.0f * RAD || f_off >= 0, "without anti-windup: far worse");
}

/* ---- 7: a broken mixer ---------------------------------------------------- */
static void test_mixer(void)
{
    sim_t s;
    sim_init(&s, SEED);
    sim_takeoff(&s);                      /* take off with the good mixer      */
    ctrl_tune.mix_roll_sign = -1;         /* ...then flip one sign in flight   */
    float t_end = -1;
    for (int i = 0; i < 1500; i++) {
        sim_tick(&s);
        if (t_end < 0 && (!s.out.armed || s.w.crashed)) { t_end = (float)(i + 1) * WORLD_DT; }
    }
    ctrl_status_t st;
    control_status(&st);
    printf("\n7. Mixer with the roll sign flipped (Lab Part 6)\n");
    printf("  lost control: failsafe %s after %.3f s%s\n", fs_name(st.fail), t_end,
           s.w.crashed ? ", then crashed" : "");
    check(t_end > 0 && t_end < 1.5f, "a flipped mixer sign is caught within 1.5 s");
    ctrl_tune.mix_roll_sign = 1;
}

/* ---- the tuning contest (Lab Part 1) ----------------------------------------
 *   sitl score ANGLE_KP RATE_KP [RATE_KI [RATE_KD [ALPHA]]]
 * Score = roll settle + pitch settle (s), lower is better.  Disqualified if
 * either overshoots more than 5 %, buzzes (rate RMS > 3 deg/s after 2 s), a
 * gust tilts it past 15 deg, or anything disarms or crashes. */
static float gust_peak(int *ok)
{
    sim_t s;
    sim_init(&s, SEED);
    *ok = sim_takeoff(&s);
    world_kick(&s.w, 4.0f, -2.0f);
    float peak = 0;
    for (int i = 0; i < 1000; i++) {
        sim_tick(&s);
        if (true_tilt(&s) > peak) { peak = true_tilt(&s); }
    }
    if (!s.out.armed || s.w.crashed) { *ok = 0; }
    return peak * DEG;
}

static int score(int argc, char **argv)
{
    control_init();
    tune_override = ctrl_tune;
    tune_override.angle_kp = (float)atof(argv[2]);
    tune_override.rate_kp  = (float)atof(argv[3]);
    if (argc > 4) { tune_override.rate_ki = (float)atof(argv[4]); }
    if (argc > 5) { tune_override.rate_kd = (float)atof(argv[5]); }
    if (argc > 6) { tune_override.alpha   = (float)atof(argv[6]); }
    use_tune = 1;
    printf("angle_kp %.3f  rate_kp %.4f  rate_ki %.3f  rate_kd %.5f  alpha %.3f\n",
           tune_override.angle_kp, tune_override.rate_kp, tune_override.rate_ki,
           tune_override.rate_kd, tune_override.alpha);
    step_t r = step_response(0, 20.0f, 1.0f), p = step_response(1, 20.0f, 1.0f);
    int g_ok;
    float g = gust_peak(&g_ok);
    printf("  roll : rise %.3f s  overshoot %5.1f %%  settle %.3f s  rate RMS %.2f deg/s\n", r.rise, r.over, r.settle, r.osc);
    printf("  pitch: rise %.3f s  overshoot %5.1f %%  settle %.3f s  rate RMS %.2f deg/s\n", p.rise, p.over, p.settle, p.osc);
    printf("  gust : peak %.1f deg\n", g);
    const char *why = 0;
    if (!r.armed || !p.armed || r.crashed || p.crashed || !g_ok) { why = "disarmed or crashed"; }
    else if (r.over > 5.0f || p.over > 5.0f)                      { why = "overshoot > 5 %"; }
    else if (r.osc > 3.0f || p.osc > 3.0f)                        { why = "buzzing: rate RMS > 3 deg/s"; }
    else if (g > 15.0f)                                           { why = "gust peak > 15 deg"; }
    if (why) { printf("DISQUALIFIED: %s\n", why); return 1; }
    printf("SCORE %.3f s   (roll + pitch settling; default gains score 0.620)\n", r.settle + p.settle);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 4 && strcmp(argv[1], "score") == 0) { return score(argc, argv); }
    printf("SOC3050 lesson 13 - SITL: world.c + control.c on the host\n");
    printf("world %.0f Hz, control %.0f Hz, seed %u\n\n", WORLD_HZ, CTRL_HZ, SEED);

    printf("1. Step responses, 20 deg, true attitude (5 %% band = 1 deg)\n");
    printf("  %-22s %7s %9s %9s %8s %8s   %s\n", "case", "rise s", "overshoot", "settle s",
           "err deg", "osc d/s", "end");
    step_t r = step_response(0, 20.0f, 1.0f);
    print_step("roll  +20", r);
    check(r.rise > 0 && r.rise < 0.35f, "roll rise < 0.35 s");
    check(r.over < 15.0f, "roll overshoot < 15 %");
    check(r.settle < 1.0f && r.armed && !r.crashed, "roll settles < 1 s");
    step_t p = step_response(1, 20.0f, 1.0f);
    print_step("pitch +20", p);
    check(p.rise > 0 && p.rise < 0.35f, "pitch rise < 0.35 s");
    check(p.over < 15.0f, "pitch overshoot < 15 %");
    check(p.settle < 1.0f && p.armed && !p.crashed, "pitch settles < 1 s");
    step_t m = step_response(0, -20.0f, 1.0f);
    print_step("roll  -20", m);
    check(m.settle < 1.0f && m.armed, "roll -20 settles < 1 s");

    test_gust();
    test_failsafes();
    test_alpha();

    printf("\n5. Gain sweep: the knob multiplies the rate loop's Kp (roll +20 deg)\n");
    printf("  %-22s %7s %9s %9s %8s %8s   %s\n", "kp_scale", "rise s", "overshoot", "settle s",
           "err deg", "osc d/s", "end");
    static const float scales[] = { 0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f };
    step_t sw[7];
    for (int k = 0; k < 7; k++) {
        char name[24];
        snprintf(name, sizeof name, "x%.2f", scales[k]);
        sw[k] = step_response(0, 20.0f, scales[k]);
        print_step(name, sw[k]);
    }
    check(sw[0].settle > sw[3].settle, "too-low Kp settles slower than the default");
    check(sw[6].osc > 3.0f * sw[3].osc || !sw[6].armed || sw[6].crashed,
          "too-high Kp oscillates (rate RMS > 3x default) or fails");

    test_windup();
    test_mixer();

    printf("\n%s: %d of %d checks failed\n", failures ? "FAILED" : "PASSED", failures, checks);
    return failures ? 1 : 0;
}
