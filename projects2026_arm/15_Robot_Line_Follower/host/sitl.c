/*
 * host/sitl.c - lesson 15's Software-In-The-Loop test, on a PC
 *
 * Links the SAME world.c, track.c, control.c and view.c the firmware links,
 * plus _lib/oled.c with the I2C call stubbed out, and runs races with no
 * chip, no RTOS and no Wokwi.  The schedule matches the firmware's: the world
 * steps every 5 ms, the controller runs on every second step (100 Hz) and
 * sees the sensors the world produced on the step before.
 *
 *   run.bat / run.sh             all the tests below; exit 1 on any failure
 *   sitl race T KNOB [KP KD KI DF NOISE AMBIENT THR]  one race, 5 laps, lap by lap
 *   sitl ascii T                 just the OLED frame, as text
 *
 * The tests:
 *   1. every track closes (the turtle comes home)
 *   2. default gains, default speed (knob 500 = 1000 mm/s): 5 laps of every
 *      track, no DNF allowed
 *   3. speed sweep: the fastest knob setting that still finishes 3 laps
 *   4. P only vs PD vs PID on S-BENDS
 *   5. sensor noise vs the D term, with and without its filter
 *   6. a dead sensor
 *   7. determinism: the same race twice gives the same numbers
 *   8. the OLED frame, as text, for the slides
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../control.h"
#include "../world.h"
#include "../track.h"
#include "../view.h"
#include "oled.h"
#include "i2c.h"

/* The panel is not there: every I2C write "succeeds". */
int i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    (void)addr; (void)w; (void)wn; (void)r; (void)rn;
    return 0;
}

#define MAXLAPS 8

static int g_ambient;                          /* race mode only: argv[9] */

typedef struct {
    int      dnf, laps;
    uint32_t lap[MAXLAPS], best, t_ms, slip_ms;
    float    max_err, rms, chatter;
} result_t;

static ctl_gains_t defaults;                   /* copied from control.c at start */

static result_t race(int trk, int knob, ctl_gains_t g, int noise, int fail,
                     int laps_wanted, uint32_t limit_ms)
{
    result_t r;
    memset(&r, 0, sizeof r);
    world.noise = (int16_t)noise;
    world.ambient = (int16_t)g_ambient;
    world.failed = (int8_t)fail;
    world.rng = 0x2545F491u;                   /* same noise every run */
    world_reset(trk);
    ctl_gains = g;
    control_reset();
    world_start();

    sensors_t s;
    actuators_t a = { 0, 0 };
    world_sense(&s);
    s.knob = (int16_t)knob;
    float prev_turn = 0.0f, ch2 = 0.0f;
    uint32_t nch = 0, step = 0;
    while (world.state == W_RUN && world.laps < laps_wanted && world.t_ms < limit_ms) {
        if ((step++ & 1u) == 0u) {
            control_step(&s, &a);
            float turn = (float)(a.right - a.left);
            ch2 += (turn - prev_turn) * (turn - prev_turn);
            prev_turn = turn;
            nch++;
        }
        uint16_t before = world.laps;
        world_step(&a);
        world_sense(&s);
        s.knob = (int16_t)knob;
        if (world.laps != before && before < MAXLAPS) { r.lap[before] = world.last_lap; }
    }
    r.dnf = world.state == W_DNF ? world.dnf : (world.laps < laps_wanted ? 9 : 0);
    r.laps = world.laps;
    r.best = world.best_lap;
    r.t_ms = world.t_ms;
    r.slip_ms = world.slip_ms;
    r.max_err = world.max_err;
    r.rms = world.err_n ? (float)__builtin_sqrt(world.err2 / (float)world.err_n) : 0.0f;
    r.chatter = nch ? (float)__builtin_sqrt(ch2 / (float)nch) : 0.0f;
    return r;
}

static const char *why(int d)
{
    return d == 0 ? "-" : d == DNF_LOST ? "DNF lost" : d == DNF_FAR ? "DNF off" : "timeout";
}

static void print_ms(uint32_t ms) { printf("%3lu.%02lu", (unsigned long)(ms / 1000u), (unsigned long)(ms % 1000u / 10u)); }

static void ascii_frame(int trk)
{
    /* a race in progress, drawn by view.c into oled.c's framebuffer */
    world.noise = 15; world.ambient = 0; world.failed = -1; world.rng = 0x2545F491u;
    world_reset(trk);
    ctl_gains = defaults;
    control_reset();
    world_start();
    sensors_t s; actuators_t a = { 0, 0 };
    world_sense(&s); s.knob = 500;
    for (uint32_t k = 0; world.t_ms < 9000u && world.state == W_RUN; k++) {
        if ((k & 1u) == 0u) { control_step(&s, &a); }
        world_step(&a); world_sense(&s); s.knob = 500;
    }
    view_track();
    view_robot(world.x, world.y, world.th);
    view_hud(&world, &s, 0, 500);
    /* two pixel rows per text line: ' ' none, '\'' top, '.' bottom, ':' both */
    printf("+");
    for (int x = 0; x < OLED_W; x++) { putchar('-'); }
    printf("+\n");
    for (int y = 0; y < OLED_H; y += 2) {
        putchar('|');
        for (int x = 0; x < OLED_W; x++) {
            int t = (oled_page(y / 8)[x] >> (y % 8)) & 1, b = (oled_page((y + 1) / 8)[x] >> ((y + 1) % 8)) & 1;
            putchar(t && b ? ':' : t ? '\'' : b ? '.' : ' ');
        }
        printf("|\n");
    }
    printf("+");
    for (int x = 0; x < OLED_W; x++) { putchar('-'); }
    printf("+\n");
}

int main(int argc, char **argv)
{
    defaults = ctl_gains;
    int fails = 0;

    if (argc >= 4 && strcmp(argv[1], "race") == 0) {
        ctl_gains_t g = defaults;
        int trk = atoi(argv[2]) - 1, knob = atoi(argv[3]), noise = 15;
        if (argc > 4) { g.kp = (float)atof(argv[4]); }
        if (argc > 5) { g.kd = (float)atof(argv[5]); }
        if (argc > 6) { g.ki = (float)atof(argv[6]); }
        if (argc > 7) { g.dfilt = (float)atof(argv[7]); }
        if (argc > 8) { noise = atoi(argv[8]); }
        if (argc > 9) { g_ambient = atoi(argv[9]); }
        if (argc > 10) { g.thr = (float)atof(argv[10]); }
        result_t r = race(trk, knob, g, noise, -1, 5, 120000u);
        printf("%s knob %d: laps %d", track_name(trk), knob, r.laps);
        for (int i = 0; i < r.laps && i < MAXLAPS; i++) { printf(" "); print_ms(r.lap[i]); }
        printf("  max %.1f rms %.1f mm  slip %lu ms  chatter %.0f  %s\n", r.max_err, r.rms,
               (unsigned long)r.slip_ms, r.chatter, why(r.dnf));
        return r.dnf ? 1 : 0;
    }
    if (argc >= 3 && strcmp(argv[1], "ascii") == 0) { ascii_frame(atoi(argv[2]) - 1); return 0; }

    printf("=== SOC3050 lesson 15 - line follower, Software-In-The-Loop on the host ===\n");
    printf("default gains: kp %.2f  ki %.2f  kd %.3f  dfilt %.2f  slow %.2f  thr %.0f\n\n",
           defaults.kp, defaults.ki, defaults.kd, defaults.dfilt, defaults.slow, defaults.thr);

    /* 1 ---------------------------------------------------------------- */
    printf("1. Tracks: built from turtle commands, closure checked\n");
    printf("   track      level    points  length mm  box mm       closure miss\n");
    for (int t = 0; t < TRACK_COUNT; t++) {
        track_build(t);
        int bad = track.n >= TRACK_MAX_PTS || track.close_err > 2.0f;
        printf("   %d %-8s %-7s  %4d    %6u     %4dx%-4d    %.2f mm %s\n", t + 1, track_name(t), track_level(t),
               track.n, (unsigned)track.length, track.xmax - track.xmin, track.ymax - track.ymin,
               track.close_err, bad ? "FAIL" : "ok");
        fails += bad;
    }

    /* 2 ---------------------------------------------------------------- */
    printf("\n2. Default gains, knob 500 (1000 mm/s), noise 15: five laps each\n");
    printf("   track      lap 1   lap 2   lap 3   lap 4   lap 5    best   max err  rms err  slip ms  result\n");
    for (int t = 0; t < TRACK_COUNT; t++) {
        result_t r = race(t, 500, defaults, 15, -1, 5, 120000u);
        printf("   %-9s", track_name(t));
        for (int i = 0; i < 5; i++) { printf(" "); if (i < r.laps) { print_ms(r.lap[i]); } else { printf("     - "); } printf(" "); }
        printf(" "); print_ms(r.best);
        printf("  %5.1f mm %5.1f mm  %6lu   %s\n", r.max_err, r.rms, (unsigned long)r.slip_ms, why(r.dnf));
        if (r.dnf) { fails++; }
    }

    /* 3 ---------------------------------------------------------------- */
    printf("\n3. Speed sweep: knob 300..1000 in steps of 25, 3 laps, noise 15.\n");
    printf("   The fastest knob setting that still finishes - with and without the corner slow-down\n");
    printf("   track      slow  fastest knob (mm/s)  best lap  max err   first failure\n");
    for (int t = 0; t < TRACK_COUNT; t++) {
        for (int sl = 0; sl < 2; sl++) {
            ctl_gains_t g = defaults;
            if (sl) { g.slow = 0.0f; }
            int ok_knob = -1, bad_knob = -1; result_t ok, bad;
            memset(&ok, 0, sizeof ok); memset(&bad, 0, sizeof bad);
            for (int k = 300; k <= 1000; k += 25) {
                result_t r = race(t, k, g, 15, -1, 3, 90000u);
                if (r.dnf) { bad = r; bad_knob = k; break; }
                ok = r; ok_knob = k;
            }
            printf("   %-9s  %.2f  %4d  (%4d)          ", sl ? "" : track_name(t), g.slow, ok_knob, ok_knob * 2);
            print_ms(ok.best);
            printf("  %5.1f mm  ", ok.max_err);
            if (bad_knob > 0) { printf("knob %d: %s after %d laps\n", bad_knob, why(bad.dnf), bad.laps); }
            else              { printf("none up to 1000\n"); }
            if (!sl && ok_knob < 500) { fails++; }
        }
    }

    /* 4 ---------------------------------------------------------------- */
    printf("\n4. P vs PD vs PID on S-BENDS, knob 700 (1400 mm/s), noise 15, 5 laps\n");
    printf("   controller           kp    kd    ki    best lap  max err  rms err  chatter  result\n");
    {
        struct { const char *name; float kp, kd, ki; } c[] = {
            { "P only (default kp)", defaults.kp, 0.0f, 0.0f },
            { "P only, kp halved  ", defaults.kp * 0.5f, 0.0f, 0.0f },
            { "PD (default)       ", defaults.kp, defaults.kd, 0.0f },
            { "PID                ", defaults.kp, defaults.kd, 2.0f },
        };
        result_t res[4];
        for (int i = 0; i < 4; i++) {
            ctl_gains_t g = defaults; g.kp = c[i].kp; g.kd = c[i].kd; g.ki = c[i].ki;
            res[i] = race(1, 700, g, 15, -1, 5, 120000u);
            printf("   %s %5.1f %5.2f %5.1f   ", c[i].name, c[i].kp, c[i].kd, c[i].ki);
            print_ms(res[i].best);
            printf("  %5.1f mm %5.1f mm  %5.0f   %s (%d laps)\n", res[i].max_err, res[i].rms, res[i].chatter,
                   why(res[i].dnf), res[i].laps);
        }
        /* the claim the slides make: P alone is worse than PD */
        int p_worse = res[0].dnf || res[0].rms > 1.3f * res[2].rms;
        printf("   P-only is %s than PD: %s\n", p_worse ? "worse" : "NOT worse", p_worse ? "ok" : "FAIL");
        fails += !p_worse || res[2].dnf;
    }

    /* 5 ---------------------------------------------------------------- */
    printf("\n5. Sensor noise vs the D term (S-BENDS, knob 700, 5 laps)\n");
    printf("   chatter = RMS change of (right - left) command between control steps: the motors' buzz\n");
    printf("   noise  dfilt   best lap  max err  rms err  chatter mm/s  result\n");
    {
        const int   nz[3] = { 0, 15, 80 };
        const float df[3] = { 1.0f, 0.5f, 0.25f };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ctl_gains_t g = defaults; g.dfilt = df[j];
                result_t r = race(1, 700, g, nz[i], -1, 5, 120000u);
                printf("   %4d   %.2f    ", nz[i], df[j]);
                print_ms(r.best);
                printf("  %5.1f mm %5.1f mm   %6.0f      %s\n", r.max_err, r.rms, r.chatter, why(r.dnf));
            }
        }
    }

    /* 6 ---------------------------------------------------------------- */
    printf("\n6. A dead sensor (reads 0 forever), OVAL and S-BENDS, knob 500\n");
    printf("   dead     OVAL best  max err  result      S-BENDS best  max err  result\n");
    for (int f = -1; f < LINE_SENSORS; f += (f < 0 ? 1 : 3)) {
        result_t a = race(0, 500, defaults, 15, f, 5, 120000u);
        result_t b = race(1, 500, defaults, 15, f, 5, 120000u);
        if (f < 0) { printf("   none     "); } else { printf("   s%d       ", f); }
        print_ms(a.best); printf("   %5.1f mm  %-10s  ", a.max_err, why(a.dnf));
        print_ms(b.best); printf("     %5.1f mm  %s\n", b.max_err, why(b.dnf));
    }

    /* 7 ---------------------------------------------------------------- */
    {
        result_t a = race(3, 500, defaults, 15, -1, 3, 90000u);
        result_t b = race(3, 500, defaults, 15, -1, 3, 90000u);
        int same = memcmp(&a, &b, sizeof a) == 0;
        printf("\n7. Determinism: FIGURE-8 raced twice: %s\n", same ? "identical, ok" : "DIFFERENT - FAIL");
        fails += !same;
    }

    /* 8 ---------------------------------------------------------------- */
    printf("\n8. The OLED after 9 s on HAIRPINS, drawn by view.c + oled.c\n");
    ascii_frame(2);
    printf("   bytes in the framebuffer: 1024; the firmware sends only dirty pages\n");

    /* 9 ---------------------------------------------------------------- */
    printf("\n9. What the shortcuts save (default gains, knob 500, 3 laps)\n");
    printf("   track      segments  box-test survivors per step  OLED bytes per 10 Hz frame\n");
    printf("                         avg    max                  avg     (a full frame: 1032)\n");
    for (int t = 0; t < TRACK_COUNT; t++) {
        world.noise = 15; world.ambient = 0; world.failed = -1; world.rng = 0x2545F491u;
        world_reset(t);
        ctl_gains = defaults;
        control_reset();
        world_start();
        view_track();
        oled_flush();
        uint32_t b0 = oled_bytes_sent(), frames = 0, csum = 0, cmax = 0, steps = 0;
        sensors_t s; actuators_t a = { 0, 0 };
        world_sense(&s); s.knob = 500;
        for (uint32_t k = 0; world.state == W_RUN && world.laps < 3; k++) {
            if ((k & 1u) == 0u) { control_step(&s, &a); }
            world_step(&a); world_sense(&s); s.knob = 500;
            csum += world.cand; steps++;
            if (world.cand > cmax) { cmax = world.cand; }
            if (k % 20u == 19u) {                  /* every 100 ms, as task_display */
                view_robot(world.x, world.y, world.th);
                view_hud(&world, &s, 0, 500);
                oled_flush();
                frames++;
            }
        }
        printf("   %-9s    %3d        %4.1f   %3lu                    %5.0f\n", track_name(t), track.n - 1,
               (double)csum / steps, (unsigned long)cmax, (double)(oled_bytes_sent() - b0) / frames);
    }

    printf("\n%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
