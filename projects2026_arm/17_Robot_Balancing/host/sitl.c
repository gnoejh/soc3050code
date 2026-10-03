/*
 * sitl.c - software-in-the-loop test of lesson 17's controllers, on a PC
 *
 * Compiles the firmware's OWN world.c, control.c and view.c with the PC's gcc
 * and runs them exactly as the firmware's tasks do: physics every 1 ms,
 * sensors -> control_step() -> volts every 5 ms.  Same seed, same run.
 *
 * For each of the three controllers, with sensor noise ON:
 *   recover   from a 6 degree tilt: settling time, peak voltage
 *   push      the largest shove it survives (forward, at the body's CoM)
 *   drift     how far it wanders in 30 s with no command
 *   payload   the heaviest load on top (PAY_H high) it survives with a small push
 *   drive     follow a 0.3 m/s joystick command for 5 s: distance covered
 * then a comparison table, and some checks that make the exit status
 * non-zero if a controller that is supposed to work does not.
 *
 *   run.sh   (Git Bash)   or   run.bat   (cmd)
 *   ./sitl --trace MODE [push|drive|tilt]   a 10 ms CSV trace of one run
 *                    (MODE 0 PID tilt, 1 cascade, 2 LQR), for plotting
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../params.h"
#include "../world.h"
#include "../control.h"
#include "../view.h"
#include "oled.h"

#define DEG(r) ((r) * 57.29578f)

typedef struct {
    int   mode;
    float secs, tilt0;
    float push_t, push_J;
    float payload, noise, alpha, vbat;
    float vcmd;              /* joystick speed command, m/s, from t = 1 s     */
    uint32_t seed;
} scen_t;

typedef struct {
    int   fell;
    float t_fall;
    float settle;            /* time after the disturbance until it STAYS settled */
    float max_th;            /* rad, true tilt                                  */
    float x_end, max_x;      /* m                                               */
    float peak_u;            /* V asked for                                     */
    float sat_frac;          /* fraction of steps the supply clipped            */
    float th_est_err;        /* rms (estimate - truth), rad                     */
    float x_ref;             /* where the controller was aiming at the end, m   */
} result_t;

static scen_t base(int mode)
{
    scen_t s = { mode, 10.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.98f, VBAT, 0.0f, 12345u };
    return s;
}

static result_t run(const scen_t *s, FILE *trace)
{
    world_t w = {0};
    w.noise = s->noise;
    w.vbat  = s->vbat;
    world_set_payload(&w, s->payload);
    world_init(&w, s->seed, s->tilt0);
    control_init();
    control_set_mode(s->mode);
    control_set_alpha(s->alpha);

    result_t r = {0};
    r.settle = -1.0f;
    float t0 = s->push_t > 0.0f ? s->push_t : 0.0f;   /* settle is measured from here */
    float last_unsettled = t0;
    double err2 = 0.0;
    int nerr = 0;
    actuators_t a = {0};
    sensors_t sn;
    int steps = (int)(s->secs / WORLD_DT + 0.5f);
    int ratio = (int)(CTRL_DT / WORLD_DT + 0.5f);
    int pushed = 0;

    for (int k = 0; k < steps; k++) {
        float t = (float)k * WORLD_DT;
        if (!pushed && s->push_t > 0.0f && t >= s->push_t) { world_push(&w, s->push_J); pushed = 1; }
        if (k % ratio == 0) {
            float v = (s->vcmd != 0.0f && t >= 1.0f) ? s->vcmd : 0.0f;
            world_sense(&w, v, &sn);
            control_step(&sn, &a);
            if (fabsf(a.volts) > r.peak_u) { r.peak_u = fabsf(a.volts); }
            float e = control_estimate()->th - w.th;
            if (!w.fallen) { err2 += (double)e * e; nerr++; }
            if (trace && k % 10 == 0) {
                fprintf(trace, "%.3f,%.3f,%.3f,%.4f,%.4f,%.3f\n", t, DEG(w.th),
                        DEG(control_estimate()->th), w.x, w.xd, a.volts);
            }
        }
        world_step(&w, a.volts);
        if (w.fallen) { r.fell = 1; r.t_fall = t; break; }
        if (fabsf(w.th) > r.max_th) { r.max_th = fabsf(w.th); }
        if (fabsf(w.x) > r.max_x)   { r.max_x = fabsf(w.x); }
        if (t >= t0 && (fabsf(w.th) > 0.0175f || fabsf(w.xd) > 0.05f)) { last_unsettled = t; }
    }
    r.x_end = w.x;
    r.x_ref = control_estimate()->x_ref;
    r.sat_frac = (float)w.saturated / (float)steps;
    r.th_est_err = nerr ? (float)sqrt(err2 / nerr) : 0.0f;
    if (!r.fell && last_unsettled < s->secs - 2.0f) { r.settle = last_unsettled - t0; }
    return r;
}

/* the largest push it survives: scan up in 0.02 N s steps until the first fall */
static float max_push(int mode, float noise, float *settle_small)
{
    float best = 0.0f;
    for (float J = 0.02f; J <= 3.0f; J += 0.02f) {
        scen_t s = base(mode);
        s.secs = 6.0f; s.push_t = 3.0f; s.push_J = J; s.noise = noise;   /* 3 s to recover */
        result_t r = run(&s, 0);
        if (r.fell) { break; }
        best = J;
    }
    scen_t s = base(mode);
    s.secs = 10.0f; s.push_t = 3.0f; s.push_J = 0.10f; s.noise = noise;
    *settle_small = run(&s, 0).settle;
    return best;
}

/* the heaviest payload it carries through a 0.1 N s push: 3 s, push, 3 s */
static float max_payload(int mode)
{
    float best = -1.0f;
    for (float m = 0.0f; m <= 2.001f; m += 0.02f) {
        scen_t s = base(mode);
        s.secs = 6.0f; s.push_t = 3.0f; s.push_J = 0.10f; s.payload = m;
        if (run(&s, 0).fell) { break; }
        best = m;
    }
    return best;
}

/* ---- the OLED drawing, run on the PC: oled.c compiles here too --------- */
int i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    (void)addr; (void)w; (void)wn; (void)r; (void)rn;
    return 0;                                   /* a bus that always says yes */
}

static int check_view(void)
{
    view_t v = {0};
    v.mode = "LQR"; v.th = 0.2f; v.x = 0.1f; v.xd = 0.05f; v.u = 1.5f; v.payload = 0.3f;
    for (int i = 0; i < 60; i++) { view_history(&v, 0.3f * sinf((float)i / 5.0f)); }
    view_draw(&v);
    int lit = 0;
    for (int y = 0; y < OLED_H; y++) for (int x = 0; x < OLED_W; x++) { lit += oled_get(x, y); }
    printf("\nOLED frame drawn on the PC by view.c - all 128 x 64 pixels, one character\n"
           "per column and two rows (' = top lit, , = bottom lit, : = both):\n");
    for (int y = 0; y < OLED_H; y += 2) {
        putchar('|');
        for (int x = 0; x < OLED_W; x++) {
            int t = oled_get(x, y), b = oled_get(x, y + 1);
            putchar(t && b ? ':' : t ? '\'' : b ? ',' : ' ');
        }
        printf("|\n");
    }
    printf("  %d pixels lit\n", lit);
    return lit > 200;
}

int main(int argc, char **argv)
{
    if (argc >= 3 && strcmp(argv[1], "--trace") == 0) {
        scen_t s = base(atoi(argv[2]));               /* 0 PID tilt, 1 cascade, 2 LQR */
        s.secs = 8.0f;
        if (argc >= 4 && strcmp(argv[3], "drive") == 0) { s.vcmd = 0.30f; }
        else if (argc >= 4 && strcmp(argv[3], "tilt") == 0) { s.tilt0 = 0.1047f; }
        else { s.push_t = 2.0f; s.push_J = 0.25f; }
        printf("t,tilt_deg,tilt_est_deg,x_m,xd_mps,volts\n");
        run(&s, stdout);
        return 0;
    }

    int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL: " __VA_ARGS__); printf("\n"); fails++; } } while (0)

    printf("SOC3050 lesson 17 - balancing robot, software in the loop\n");
    printf("world %d Hz, controller %d Hz, sensor noise ON (gyro %.3f rad/s, accel %.2f m/s^2 rms),\n"
           "gyro bias %.3f rad/s, IMU mounted %.1f deg off, encoder %d counts/rev, supply %.1f V\n\n",
           (int)(1.0f / WORLD_DT + 0.5f), (int)(1.0f / CTRL_DT + 0.5f), GYRO_NOISE, ACC_NOISE,
           GYRO_BIAS, DEG(IMU_TILT), (int)ENC_CPR, VBAT);
    printf("LQR K = [%.4f %.4f %.4f %.4f]\n\n", LQR_K[0], LQR_K[1], LQR_K[2], LQR_K[3]);

    struct { float settle6, peak6, push, settle_push, drift30, maxx30, payload, drive, drive_ref, est; int fell30; float tfall30; } m[CTRL_MODES];

    for (int c = 0; c < CTRL_MODES; c++) {
        printf("--- %s ---\n", control_name(c));
        scen_t s = base(c);
        s.tilt0 = 0.1047f; s.secs = 10.0f;                         /* 6 degrees */
        result_t r = run(&s, 0);
        m[c].settle6 = r.fell ? -1.0f : r.settle;
        m[c].peak6 = r.peak_u;
        printf("  recover from 6 deg : %s, settled in %.2f s, peak %.2f V, max tilt %.1f deg, est. error %.2f deg rms\n",
               r.fell ? "FELL" : "upright", r.settle, r.peak_u, DEG(r.max_th), DEG(r.th_est_err));
        m[c].est = DEG(r.th_est_err);
        if (r.fell) { printf("                       (fell at t = %.1f s)\n", r.t_fall); }
        if (c == CTRL_PID_TILT) {        /* expected to run away - but only after balancing a while */
            CHECK(!r.fell || r.t_fall > 3.0f, "PID tilt did not even balance for 3 s");
        } else {
            CHECK(!r.fell, "%s fell recovering from 6 degrees", control_name(c));
        }

        s = base(c); s.secs = 30.0f;
        r = run(&s, 0);
        m[c].fell30 = r.fell; m[c].tfall30 = r.t_fall;
        m[c].drift30 = r.x_end; m[c].maxx30 = r.max_x;
        printf("  30 s, no command   : %s, x at end %+.3f m, furthest %.3f m\n",
               r.fell ? "FELL" : "upright", r.x_end, r.max_x);
        if (r.fell) { printf("                       (fell at t = %.1f s)\n", r.t_fall); }

        m[c].push = max_push(c, 1.0f, &m[c].settle_push);
        printf("  largest push       : %.2f N s survived (%.2f m/s to the body);"
               " 0.10 N s settles in %.2f s\n", m[c].push, m[c].push / BOT_MB, m[c].settle_push);

        m[c].payload = max_payload(c);
        printf("  largest payload    : %.2f kg at %.0f cm, through a 0.10 N s push\n",
               m[c].payload, PAY_H * 100.0f);

        s = base(c); s.secs = 6.0f; s.vcmd = 0.30f;
        r = run(&s, 0);
        m[c].drive = r.fell ? -9.0f : r.x_end;
        m[c].drive_ref = r.x_ref;
        printf("  drive 0.30 m/s 5 s : %s, covered %.2f m; the target moved %.2f m\n\n",
               r.fell ? "FELL" : "upright", r.x_end, r.x_ref);
    }

    printf("=== comparison, measured on the host (noise on, seed 12345) ===\n");
    printf("%-10s %9s %7s %9s %11s %11s %11s %9s\n", "controller", "settle 6d", "peak V",
           "push N s", "30 s drift", "payload kg", "drive 5 s", "tilt err");
    for (int c = 0; c < CTRL_MODES; c++) {
        char drift[20], settle[20], pay[20], drive[20];
        if (m[c].fell30) { snprintf(drift, sizeof drift, "fell @%.1fs", m[c].tfall30); }
        else             { snprintf(drift, sizeof drift, "%+.2f m", m[c].drift30); }
        if (m[c].settle6 < 0.0f) { snprintf(settle, sizeof settle, "never"); }
        else                     { snprintf(settle, sizeof settle, "%.2f s", m[c].settle6); }
        if (m[c].payload < 0.0f) { snprintf(pay, sizeof pay, "none"); }
        else                     { snprintf(pay, sizeof pay, "%.2f", m[c].payload); }
        if (m[c].drive < -5.0f)  { snprintf(drive, sizeof drive, "fell"); }
        else                     { snprintf(drive, sizeof drive, "%+.2f m", m[c].drive); }
        printf("%-10s %9s %7.2f %9.2f %11s %11s %11s %5.2f deg\n", control_name(c),
               settle, m[c].peak6, m[c].push, drift, pay, drive, m[c].est);
    }

    /* ---- determinism: the same seed must give the same run, bit for bit */
    scen_t s = base(CTRL_LQR); s.push_t = 2.0f; s.push_J = 0.3f;
    result_t a = run(&s, 0), b = run(&s, 0);
    int same = memcmp(&a, &b, sizeof a) == 0;
    printf("\ndeterminism: two identical runs %s\n", same ? "agree bit for bit" : "DIFFER");
    CHECK(same, "same seed, different run");

    /* ---- sensor noise x3 and the complementary filter switched off ----- */
    printf("\nLab predictions, LQR (largest push, N s):\n");
    float dummy;
    float p3 = max_push(CTRL_LQR, 3.0f, &dummy);
    printf("  noise x3                       : %.2f\n", p3);
    scen_t g = base(CTRL_LQR); g.secs = 30.0f; g.alpha = 1.0f;
    result_t rg = run(&g, 0);
    printf("  alpha = 1 (gyro only), 30 s    : %s%s, est. error %.1f deg rms\n",
           rg.fell ? "FELL" : "upright", rg.fell ? "" : "", DEG(rg.th_est_err));
    if (rg.fell) { printf("                                   fell at %.1f s\n", rg.t_fall); }
    scen_t q = base(CTRL_LQR); q.secs = 10.0f; q.alpha = 0.0f;
    result_t rq = run(&q, 0);
    printf("  alpha = 0 (accel only), 10 s   : %s, est. error %.1f deg rms\n",
           rq.fell ? "FELL" : "upright", DEG(rq.th_est_err));
    for (float vb = VBAT; vb >= 1.0f; vb -= 0.4f) {
        scen_t v = base(CTRL_LQR); v.vbat = vb; v.secs = 8.0f; v.push_t = 3.0f; v.push_J = 0.2f;
        if (run(&v, 0).fell) { printf("  supply cut until a 0.20 N s push wins: falls at %.1f V\n", vb); break; }
    }

    int view_ok = check_view();
    CHECK(view_ok, "view.c drew almost nothing");

    CHECK(!m[CTRL_LQR].fell30 && fabsf(m[CTRL_LQR].drift30) < 0.3f, "LQR did not hold position for 30 s");
    CHECK(!m[CTRL_CASCADE].fell30 && fabsf(m[CTRL_CASCADE].drift30) < 0.3f, "cascade did not hold position for 30 s");
    CHECK(m[CTRL_LQR].push >= 0.10f, "LQR survives less than a 0.10 N s push");
    CHECK(m[CTRL_LQR].payload >= PAY_MAX, "LQR cannot carry the knob's full payload");
    CHECK(m[CTRL_LQR].drive > 1.2f, "LQR did not follow the drive command");

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails ? 1 : 0;
}
