/*
 * sitl.c - software-in-the-loop test of lesson 14, on the PC
 *
 * "SITL" is what PX4 and ArduPilot call this: the real flight code, compiled
 * for the PC, flying a simulated aircraft.  Here the flight code (flight.c),
 * the protocol (mavlite.c), the screen (ui.c) and the aircraft (world.c) are
 * the very files the firmware compiles - only Main.c's RTOS tasks are
 * replaced, by the loop below, which calls them in the same order and at the
 * same rates: world 100 Hz, control 50 Hz, telemetry 5 Hz.
 *
 *   sitl                       every flight test; exit 1 if any fails
 *   sitl --parse F OUT         feed F (mission.py's frames) to the parser,
 *                              write the mission it loaded to OUT as $MIS
 *                              frames, and check that corrupted frames are
 *                              all refused
 *
 * Writes out/<name>.log - the same $POS/$TRU/$EVT/$MIS telemetry the board
 * prints - so mission.py --score can score a flight nobody had to watch.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "world.h"
#include "flight.h"
#include "mavlite.h"
#include "ui.h"
#include "oled.h"
#include "i2c.h"

/* The OLED's bus, on a PC: nowhere.  oled.c draws into RAM regardless. */
int i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    (void)addr; (void)w; (void)wn; (void)r; (void)rn;
    return 0;
}

static world_t    W;
static actuators_t act;
static FILE      *tlog;
static unsigned   ticks;                 /* control steps since reset        */
static int        failures;
static int        draw_ui;               /* 1: draw the OLED at 10 Hz, as the board does */

/* ---- what one flight measured ------------------------------------------ */
typedef struct {
    int    landed, mission_done, fence_rtl, bat_rtl, batcrit_land;
    float  t_start, t_end;               /* s: mission START .. touchdown    */
    float  max_xt, sum_xt2;  unsigned n_xt;
    float  wp_miss[FC_MAX_WP];           /* closest approach, 3D, m          */
    float  wp_hmiss[FC_MAX_WP];          /* the same, horizontal only        */
    float  gps_sum2;
    float  land_err, impact, max_r, bat_end;
    float  est_sum2;  unsigned n_est;  float est_max;
    float  hold_sum2, hold_max;  unsigned n_hold;
} result_t;

static result_t R;

static void logs(const char *s) { if (tlog) { fputs(s, tlog); } }

static void drain_events(void)
{
    fc_event_t e;
    char b[128];
    while (fc_event_pop(&e)) {
        if (ml_evt(b, sizeof b, &e)) { logs(b); }
        if (e.kind == EV_MISSION) {
            R.t_start = e.t_ms / 1000.0f;
            for (uint8_t i = 0; i < fc_get()->n_wp; i++) {
                if (ml_mis(b, sizeof b, fc_get(), i)) { logs(b); }
            }
        }
        if (e.kind == EV_MODE) {
            if (e.a == M_RTL  && e.b == R_FENCE)        { R.fence_rtl = 1; }
            if (e.a == M_RTL  && e.b == R_BATTERY)      { R.bat_rtl = 1; }
            if (e.a == M_RTL  && e.b == R_MISSION_DONE) { R.mission_done = 1; }
            if (e.a == M_LAND && e.b == R_BATTERY_CRIT) { R.batcrit_land = 1; }
            if (e.a == M_DISARMED && e.b == R_LANDED)   { R.landed = 1; R.t_end = e.t_ms / 1000.0f; }
        }
    }
}

/* Horizontal distance from p to the segment a-b. */
static float seg_dist(float px, float py, float ax, float ay, float bx, float by)
{
    float lx = bx - ax, ly = by - ay, l2 = lx * lx + ly * ly;
    float t = l2 > 1e-6f ? ((px - ax) * lx + (py - ay) * ly) / l2 : 0.0f;
    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }
    float dx = px - (ax + t * lx), dy = py - (ay + t * ly);
    return sqrtf(dx * dx + dy * dy);
}

/* One control period: two world steps, sense, control - Main.c's order. */
static void tick(void)
{
    world_step(&W, &act, WORLD_DT);
    world_step(&W, &act, WORLD_DT);
    sensors_t s;
    world_sense(&W, &s);
    control_step(&s, &act);
    ticks++;

    fc_t *f = fc_get();
    if (f->mode == M_MISSION) {                       /* cross-track, against the PLAN */
        unsigned k = f->cur;
        float ax = k ? f->wp[k - 1].x : 0.0f, ay = k ? f->wp[k - 1].y : 0.0f;
        float xt = seg_dist(W.px, W.py, ax, ay, f->wp[k].x, f->wp[k].y);
        if (xt > R.max_xt) { R.max_xt = xt; }
        R.sum_xt2 += xt * xt;  R.n_xt++;
    }
    for (unsigned i = 0; i < f->n_wp; i++) {
        float dx = W.px - f->wp[i].x, dy = W.py - f->wp[i].y, dz = W.pz - f->wp[i].z;
        float d = sqrtf(dx * dx + dy * dy + dz * dz);
        if (d < R.wp_miss[i]) { R.wp_miss[i] = d; }
        float h = sqrtf(dx * dx + dy * dy);
        if (h < R.wp_hmiss[i]) { R.wp_hmiss[i] = h; }
    }
    if (f->mode != M_DISARMED) {
        float ex = f->px - W.px, ey = f->py - W.py, e = sqrtf(ex * ex + ey * ey);
        R.est_sum2 += e * e;  R.n_est++;
        R.gps_sum2 += W.gps_err_x * W.gps_err_x + W.gps_err_y * W.gps_err_y;
        if (e > R.est_max) { R.est_max = e; }
        float r = sqrtf(W.px * W.px + W.py * W.py);
        if (r > R.max_r) { R.max_r = r; }
    }
    if (ticks % 10u == 0u) {                          /* 5 Hz telemetry      */
        char b[128];
        if (ml_pos(b, sizeof b, f)) { logs(b); }
        float wind = sqrtf(W.wind_x * W.wind_x + W.wind_y * W.wind_y);
        if (ml_tru(b, sizeof b, W.t_ms, W.px, W.py, W.pz, wind)) { logs(b); }
    }
    if (draw_ui && ticks % 5u == 0u) {             /* the UI task, 10 Hz  */
        ui_view_t v;
        ui_from_fc(&v, fc_get(), W.wind_speed);
        ui_draw(&v);
    }
    drain_events();
}

static void run_for(float seconds) { for (unsigned n = (unsigned)(seconds * CTRL_HZ); n; n--) { tick(); } }

static int run_until_disarmed(float limit_s)
{
    for (unsigned n = (unsigned)(limit_s * CTRL_HZ); n; n--) {
        tick();
        if (fc_get()->mode == M_DISARMED) { return 1; }
    }
    return 0;
}

static void reset(uint32_t seed, float wind, float bat0, float drain, const char *log)
{
    world_init(&W, seed);
    world_set_wind(&W, wind, 0.52f);
    W.battery = bat0;
    W.drain = drain;
    fc_reset();
    act = (actuators_t){ 0 };
    ticks = 0;
    memset(&R, 0, sizeof R);
    for (unsigned i = 0; i < FC_MAX_WP; i++) { R.wp_miss[i] = R.wp_hmiss[i] = 1e9f; }
    if (tlog) { fclose(tlog); tlog = 0; }
    if (log) {
        char path[96];
        snprintf(path, sizeof path, "out/%s.log", log);
        tlog = fopen(path, "w");
    }
    run_for(2.0f);                                    /* GPS lock, pre-arm     */
}

static void finish(void)
{
    R.land_err = sqrtf(W.px * W.px + W.py * W.py);
    R.impact   = W.max_impact;
    R.bat_end  = W.battery;
    if (tlog) { fclose(tlog); tlog = 0; }
}

static void check(int ok, const char *what)
{
    printf("    %-58s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) { failures++; }
}

static float wp_worst(void)
{
    float m = 0.0f;
    for (unsigned i = 0; i < fc_get()->n_wp; i++) { if (R.wp_miss[i] > m) { m = R.wp_miss[i]; } }
    return m;
}

static void print_mission(const char *title)
{
    printf("  %s\n", title);
    printf("    completion (START -> landed)      %6.1f s\n", R.t_end - R.t_start);
    printf("    cross-track error, max / RMS      %6.2f / %.2f m\n", R.max_xt,
           R.n_xt ? sqrtf(R.sum_xt2 / R.n_xt) : 0.0f);
    printf("    waypoint miss (closest approach)  ");
    for (unsigned i = 0; i < fc_get()->n_wp; i++) { printf("%.2f ", R.wp_miss[i]); }
    printf("m\n");
    printf("      ... horizontal only             ");
    for (unsigned i = 0; i < fc_get()->n_wp; i++) { printf("%.2f ", R.wp_hmiss[i]); }
    printf("m\n");
    printf("    landing error from home           %6.2f m\n", R.land_err);
    printf("    touchdown speed                   %6.2f m/s\n", R.impact);
    printf("    estimate error, RMS / max         %6.2f / %.2f m\n",
           R.n_est ? sqrtf(R.est_sum2 / R.n_est) : 0.0f, R.est_max);
    printf("    GPS's own slow error, RMS         %6.2f m\n", R.n_est ? sqrtf(R.gps_sum2 / R.n_est) : 0.0f);
    printf("    battery left                      %6.1f %%\n", R.bat_end);
}

/* ---- the flights ------------------------------------------------------- */
static result_t fly_default(uint32_t seed, float wind, int line, const char *log)
{
    reset(seed, wind, 100.0f, 0.35f, log);
    fc_get()->p.line = (float)line;
    fc_mission_default();
    int rc = fc_mission_start();
    if (rc != FC_OK) { printf("    mission refused: %s\n", fc_error_name(rc)); }
    run_until_disarmed(400.0f);
    finish();
    return R;
}

/* Take off, settle 15 s, then hold for 30 s and split the error three ways.
 * ki < 0 keeps the default KI_VEL; otherwise it is the value x1000. */
typedef struct { float true_rms, true_max, seen_rms, est_rms, mean_off, tilt_deg, wind_est; } hold_t;

static int    apply_params(int argc, char **argv);   /* NAME=VALUE ..., below */
static int    hold_argc;                             /* sitl --hold: your gains */
static char **hold_argv;

static hold_t hold_test(uint32_t seed, float wind, int32_t ki, float tau, const char *log)
{
    hold_t h = { 0 };
    reset(seed, wind, 100.0f, 0.35f, log);
    if (ki >= 0) { (void)fc_param_set("KI_VEL", ki); }
    if (hold_argc) { (void)apply_params(hold_argc, hold_argv); }
    if (tau > 0.0f) { W.tau_att = tau; }
    (void)fc_arm();
    run_for(15.0f);
    fc_t *f = fc_get();
    double st = 0, ss = 0, se = 0, mx = 0, my = 0;
    unsigned n = 30u * CTRL_HZ;
    for (unsigned k = 0; k < n; k++) {
        tick();
        float tx = W.px - f->spx, ty = W.py - f->spy, t = sqrtf(tx * tx + ty * ty);
        float sx = f->px - f->spx, sy = f->py - f->spy;
        float ex = W.px - f->px,   ey = W.py - f->py;
        st += t * t;  ss += sx * sx + sy * sy;  se += ex * ex + ey * ey;
        mx += tx;  my += ty;
        if (t > h.true_max) { h.true_max = t; }
    }
    h.true_rms = (float)sqrt(st / n);
    h.seen_rms = (float)sqrt(ss / n);
    h.est_rms  = (float)sqrt(se / n);
    h.mean_off = (float)sqrt((mx / n) * (mx / n) + (my / n) * (my / n));
    h.tilt_deg = 57.3f * sqrtf(W.roll * W.roll + W.pitch * W.pitch);
    /* In a steady hover, drag = DRAG_XY x wind, and the integral supplies it. */
    h.wind_est = sqrtf(f->ix * f->ix + f->iy * f->iy) * f->p.ki_vel / 0.35f;
    (void)fc_set_mode(M_LAND, R_CMD);
    run_until_disarmed(60.0f);
    finish();
    return h;
}

static void dump_oled(const char *path)
{
    FILE *o = fopen(path, "w");
    for (int y = 0; y < OLED_H; y++) {
        char row[OLED_W + 2];
        for (int x = 0; x < OLED_W; x++) { row[x] = oled_get(x, y) ? '#' : '.'; }
        row[OLED_W] = '\n';  row[OLED_W + 1] = '\0';
        if (o) { fputs(row, o); }
    }
    if (o) { fclose(o); }
}

static int flight_tests(void)
{
    printf("== lesson 14 SITL: flight tests (measured on the host) ==\n\n");

    /* 1. the default mission, still air */
    fly_default(1, 0.0f, 1, "nowind");
    print_mission("default mission, no wind:");
    result_t calm = R;
    check(calm.mission_done && calm.landed, "mission completed and landed");
    check(wp_worst() < 2.0f, "every waypoint within 2.0 m: 1.0 accept + GPS error");
    check(calm.land_err < 1.0f, "landed within 1.0 m of home");
    check(calm.impact < 1.0f, "touchdown slower than 1.0 m/s");
    printf("\n");

    /* 2. the same mission in a strong wind - with a screenshot at 40 s */
    reset(1, 8.0f, 100.0f, 0.35f, "wind");
    fc_mission_default();
    (void)fc_mission_start();
    draw_ui = 1;
    run_for(40.0f);
    dump_oled("out/oled_wind.txt");                 /* what the OLED shows now */
    draw_ui = 0;
    run_until_disarmed(400.0f);
    finish();
    print_mission("default mission, wind 8 m/s + gusts:");
    result_t windy = R;
    float windy_wp = wp_worst();
    check(windy.mission_done && windy.landed, "mission completed and landed");
    check(wp_worst() < 2.0f, "every waypoint within 2.0 m: 1.0 accept + GPS error");
    check(windy.max_xt < 3.0f, "cross-track error under 3.0 m");
    check(windy.land_err < 1.5f, "landed within 1.5 m of home");
    check(windy.impact < 1.0f, "touchdown slower than 1.0 m/s");
    printf("    screen at 40 s written to out/oled_wind.txt\n\n");

    /* 3. aim-at-the-waypoint against follow-the-line, same wind, same seed */
    fly_default(1, 8.0f, 0, 0);
    float xt_aim = R.max_xt, rms_aim = R.n_xt ? sqrtf(R.sum_xt2 / R.n_xt) : 0.0f;
    float t_aim = R.t_end - R.t_start;
    printf("  LINE=0 (aim at the waypoint), wind 8 m/s:\n");
    printf("    cross-track error, max / RMS      %6.2f / %.2f m   (LINE=1: %.2f / %.2f)\n",
           xt_aim, rms_aim, windy.max_xt, windy.n_xt ? sqrtf(windy.sum_xt2 / windy.n_xt) : 0.0f);
    printf("    completion                        %6.1f s     (LINE=1: %.1f s)\n",
           t_aim, windy.t_end - windy.t_start);
    check(windy.max_xt < xt_aim, "following the line beats aiming at the waypoint");
    printf("\n");

    /* 4. determinism: same seed, same flight, to the last bit */
    fly_default(1, 8.0f, 1, 0);
    check(R.max_xt == windy.max_xt && R.t_end == windy.t_end && R.land_err == windy.land_err,
          "same seed twice: bit-identical flight");
    printf("\n");

    /* 5. position hold in wind - and what each part of the error is */
    printf("  position hold, 30 s, wind 8 m/s + gusts:\n");
    hold_t h = hold_test(7, 8.0f, -1, 0.0f, "hold");
    printf("    true error  (truth - setpoint), RMS / max   %5.2f / %.2f m\n", h.true_rms, h.true_max);
    printf("    seen error  (estimate - setpoint), RMS      %5.2f m   <- what tuning can fix\n", h.seen_rms);
    printf("    blind error (truth - estimate), RMS         %5.2f m   <- what the GPS allows\n", h.est_rms);
    printf("    mean offset from the setpoint               %5.2f m\n", h.mean_off);
    printf("    tilt into the wind                          %5.1f deg\n", h.tilt_deg);
    printf("    wind read from the integrator               %5.1f m/s (true steady wind 8.0)\n", h.wind_est);
    check(h.true_rms < 1.5f, "hold: true RMS error under 1.5 m");
    hold_t h0 = hold_test(7, 8.0f, 0, 0.0f, 0);
    printf("  the same with KI_VEL = 0 (no integrator):\n");
    printf("    true error RMS %.2f m, mean offset %.2f m - blown downwind\n", h0.true_rms, h0.mean_off);
    check(h0.mean_off > 2.0f * h.mean_off, "without the integral the wind wins: offset more than doubles");
    printf("\n");

    /* 5b. time-scale separation: slow lesson 13's attitude loop down and
     *     watch the position loop that assumed it was fast */
    printf("  time-scale separation - hold in 8 m/s wind vs the attitude lag:\n");
    printf("    tau_att   bandwidth   ratio to velocity loop   seen RMS   true RMS\n");
    static const float taus[] = { 0.04f, 0.08f, 0.20f, 0.40f, 0.80f };
    float seen[5];
    for (int i = 0; i < 5; i++) {
        hold_t ht = hold_test(7, 8.0f, -1, taus[i], 0);
        seen[i] = ht.seen_rms;
        printf("    %4.0f ms   %5.1f rad/s   %6.1f x                %6.2f m   %6.2f m%s\n",
               taus[i] * 1000.0f, 1.0f / taus[i], (1.0f / taus[i]) / 2.0f,
               ht.seen_rms, ht.true_rms, taus[i] == 0.08f ? "   <- this lesson" : "");
    }
    check(seen[1] < 1.2f * seen[0], "at 80 ms the lag costs little: within 20% of 40 ms");
    check(seen[4] > 2.0f * seen[1], "at 800 ms the abstraction breaks: error more than doubles");
    printf("\n");

    /* 6. failsafe: geofence */
    printf("  failsafe - geofence (joystick pushes east at 3 m/s):\n");
    reset(3, 3.0f, 100.0f, 0.35f, "fence");
    (void)fc_arm();
    run_for(10.0f);
    fc_nudge(3.0f, 0.0f);
    unsigned n;
    for (n = 0; n < 60u * CTRL_HZ && fc_get()->mode == M_HOLD; n++) { tick(); }
    fc_nudge(0.0f, 0.0f);
    float t_breach = W.t_ms / 1000.0f;
    run_until_disarmed(200.0f);
    finish();
    printf("    RTL at t = %.1f s; farthest from home %.1f m (fence %.0f m)\n",
           t_breach, R.max_r, fc_get()->p.fence_r);
    printf("    landing error from home           %6.2f m\n", R.land_err);
    check(R.fence_rtl, "fence breach -> RTL (reason FENCE)");
    check(R.landed && R.land_err < 1.5f, "came home and landed within 1.5 m");
    check(R.max_r < fc_get()->p.fence_r + 5.0f, "overshoot past the fence under 5 m");
    printf("\n");

    /* 7. failsafe: low battery during the mission */
    printf("  failsafe - low battery (start at 32%%, default mission, wind 4 m/s):\n");
    reset(4, 4.0f, 32.0f, 0.35f, "battery");
    fc_mission_default();
    int rc = fc_mission_start();
    run_until_disarmed(400.0f);
    finish();
    printf("    start %s; RTL on battery: %s; mission finished: %s\n", fc_error_name(rc),
           R.bat_rtl ? "yes" : "no", R.mission_done ? "yes" : "no");
    printf("    landed with %.1f %% left, %.2f m from home\n", R.bat_end, R.land_err);
    check(R.bat_rtl && !R.mission_done, "battery < 25% -> RTL before the mission ends");
    check(R.landed && R.land_err < 1.5f && R.bat_end > 10.0f, "landed at home above 10%");
    printf("\n");

    /* 8. failsafe: critical battery - a cell sags to 9 % at 25 s, mid-mission
     *    ($PARAM,BAT,9 does the same on the board): no time to get home */
    printf("  failsafe - critical battery (drops to 9%% at 25 s, wind 4 m/s):\n");
    reset(5, 4.0f, 100.0f, 0.35f, "batcrit");
    fc_mission_default();
    (void)fc_mission_start();
    run_for(25.0f);
    W.battery = 9.0f;
    float bx = W.px, by = W.py;
    run_until_disarmed(400.0f);
    finish();
    float moved = sqrtf((W.px - bx) * (W.px - bx) + (W.py - by) * (W.py - by));
    printf("    LAND in place: %s; landed %.1f m from where it was, %.1f m from home\n",
           R.batcrit_land ? "yes" : "no", moved, R.land_err);
    printf("    touchdown %.2f m/s with %.1f %% left\n", R.impact, R.bat_end);
    check(R.batcrit_land && R.landed && !R.bat_rtl, "battery < 10% -> LAND in place (no RTL), landed");
    check(moved < 5.0f, "landed within 5 m of where the failsafe fired");
    check(R.impact < 1.0f && R.bat_end > 0.0f, "soft touchdown with charge left");
    printf("\n");

    /* 9. a mission is checked before it flies */
    printf("  mission validation, on the ground:\n");
    reset(6, 0.0f, 100.0f, 0.35f, 0);
    check(fc_mission_start() == FC_E_EMPTY, "no waypoints              -> EMPTY");
    (void)fc_wp_set(0, 10.0f, 0.0f, 5.0f);
    (void)fc_wp_set(2, 10.0f, 10.0f, 5.0f);
    check(fc_mission_start() == FC_E_EMPTY, "waypoint 1 missing        -> EMPTY");
    (void)fc_wp_set(1, 60.0f, 0.0f, 5.0f);
    check(fc_mission_start() == FC_E_FENCE, "waypoint 60 m out         -> FENCE");
    (void)fc_wp_set(1, 20.0f, 0.0f, 5.0f);
    check(fc_mission_start() == FC_OK && fc_get()->mode == M_TAKEOFF, "fixed                     -> OK, TAKEOFF");

    /* the expectations file mission.py --score --expect compares against */
    FILE *e = fopen("out/wind.expect", "w");
    if (e) {
        fprintf(e, "time %.1f\nmax_xt %.2f\nwp_worst %.2f\nland %.2f\n",
                windy.t_end - windy.t_start, windy.max_xt, windy_wp, windy.land_err);
        fclose(e);
    }

    printf("\n%s: %d check(s) failed\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}

/* ---- the parser round trip -------------------------------------------- */
static int sim_param(const char *name, int32_t v)
{
    /* the same simulator settings, and ranges, as Main.c's sim_param() */
    if      (strcmp(name, "WIND")  == 0) { if (v < 0 || v > 150)    { return FC_E_RANGE; } world_set_wind(&W, v * 0.1f, W.wind_dir); }
    else if (strcmp(name, "WDIR")  == 0) { if (v < -360 || v > 360) { return FC_E_RANGE; } W.wind_dir = v * 0.0174533f; }
    else if (strcmp(name, "BAT")   == 0) { if (v < 0 || v > 100)    { return FC_E_RANGE; } W.battery = (float)v; }
    else if (strcmp(name, "DRAIN") == 0) { if (v < 0 || v > 1000)   { return FC_E_RANGE; } W.drain = v * 0.01f; }
    else if (strcmp(name, "TAU")   == 0) { if (v < 20 || v > 1000)  { return FC_E_RANGE; } W.tau_att = v * 0.001f; }
    else { return FC_E_PARAM; }
    return FC_OK;
}

static int parse_test(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    if (!fi || !fo) { printf("cannot open %s / %s\n", in, out); return 1; }
    reset(9, 0.0f, 100.0f, 0.35f, 0);                 /* 2 s: GPS lock, so START can arm */
    char line[160], reply[256];
    int frames = 0, ok = 0, refused = 0, bad = 0;
    while (fgets(line, sizeof line, fi)) {
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] != '$') { continue; }
        frames++;
        /* corrupt a copy first: flip one bit in the body - it must bounce */
        char evil[160];
        strcpy(evil, line);
        evil[2] ^= 0x01;
        if (ml_handle(evil, reply, sizeof reply, sim_param) != ML_BAD) { bad++; }
        /* then the real thing */
        ml_handle(line, reply, sizeof reply, sim_param);
        printf("    %-34s -> %s", line, reply);
        if (strstr(reply, ",OK*")) { ok++; } else { refused++; }
    }
    fclose(fi);
    for (uint8_t i = 0; i < fc_get()->n_wp; i++) {
        char b[96];
        if (ml_mis(b, sizeof b, fc_get(), i)) { fputs(b, fo); }
    }
    fclose(fo);
    printf("  %d frames: %d acknowledged OK, %d refused; %d of %d one-bit corruptions got through\n",
           frames, ok, refused, bad, frames);

    /* Well-formed frames with bad CONTENT: every one must be refused. */
    static const char *const evil[] = {
        "WP,1,12abc,0,50", "WP,16,0,0,50", "WP,1,0,0", "WP,1,0,0,50,9", "WP,1,99999,0,50",
        "MODE,FLY", "FLY", "PARAM,KP_POS,99999", "PARAM,NOPE,1", "PARAM,TILT,x",
        "WP,1,0000000000000000000000000000000000000000000000000000000000001,0,50",
    };
    int caught = 0, n_evil = (int)(sizeof evil / sizeof evil[0]);
    for (int i = 0; i < n_evil; i++) {
        char fr[128];
        ml_frame(fr, sizeof fr, "%s", evil[i]);
        fr[strcspn(fr, "\n")] = '\0';
        ml_handle(fr, reply, sizeof reply, sim_param);
        if (strstr(reply, ",ERR,")) { caught++; } else { printf("    ACCEPTED: %s -> %s", fr, reply); }
        if (i < 3 || i == n_evil - 1) { printf("    %-34.34s -> %s", fr, reply); }
    }
    printf("  %d of %d malformed-but-checksummed frames refused\n", caught, n_evil);
    if (caught != n_evil) { bad++; }
    printf("  mission loaded: %u waypoints, mode now %s\n",
           (unsigned)fc_get()->n_wp, fc_mode_name(fc_get()->mode));
    return (frames > 0 && refused == 0 && bad == 0) ? 0 : 1;
}

/* NAME=VALUE ... -> fc_param_set(), or the simulator's own (WIND, BAT, TAU ...) */
static int apply_params(int argc, char **argv)
{
    for (int i = 0; i < argc; i++) {                  
        char name[24];
        long v;
        if (sscanf(argv[i], "%23[^=]=%ld", name, &v) != 2) { printf("not NAME=VALUE: %s\n", argv[i]); return 1; }
        int rc = fc_param_set(name, (int32_t)v);
        if (rc == FC_E_PARAM) { rc = sim_param(name, (int32_t)v); }
        printf("  %s = %ld: %s\n", name, v, fc_error_name(rc));
        if (rc != FC_OK) { return 1; }
    }
    return 0;
}

/* ---- fly YOUR mission, with YOUR parameters, on the PC (Lab Parts 5-7) ---- */
static int fly(float wind, const char *frames, int argc, char **argv)
{
    reset(1, wind, 100.0f, 0.35f, "fly");
    char line[160], reply[256];
    FILE *fi = fopen(frames, "r");
    if (!fi) { printf("cannot open %s\n", frames); return 1; }
    while (fgets(line, sizeof line, fi)) {             /* mission.py's frames     */
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] != '$') { continue; }
        ml_handle(line, reply, sizeof reply, sim_param);
        if (strstr(reply, ",ERR,")) { printf("  %s -> %s", line, reply); }
    }
    fclose(fi);
    if (apply_params(argc, argv) != 0) { return 1; }   /* after the file, so they win */
    if (fc_get()->mode == M_DISARMED) {
        int rc = fc_mission_start();
        if (rc != FC_OK) { printf("  mission refused: %s\n", fc_error_name(rc)); return 1; }
    }
    int down = run_until_disarmed(900.0f);
    finish();
    print_mission(down ? "your flight:" : "your flight (STILL FLYING after 900 s):");
    printf("  log: out/fly.log  ->  python mission.py --score out/fly.log --race\n");
    return down && R.landed ? 0 : 1;
}

int main(int argc, char **argv)
{
    if (argc == 4 && strcmp(argv[1], "--parse") == 0) { return parse_test(argv[2], argv[3]); }
    if (argc >= 4 && strcmp(argv[1], "--fly") == 0) {
        return fly((float)atof(argv[2]), argv[3], argc - 4, argv + 4);
    }
    if (argc >= 3 && strcmp(argv[1], "--hold") == 0) {    /* Lab Part 3 */
        hold_argc = argc - 3;  hold_argv = argv + 3;
        hold_t h = hold_test(7, (float)atof(argv[2]), -1, 0.0f, "hold");
        printf("  hold 30 s, wind %s m/s:  seen RMS %.2f m   blind %.2f m   true %.2f m (max %.2f)\n",
               argv[2], h.seen_rms, h.est_rms, h.true_rms, h.true_max);
        printf("  mean offset %.2f m, tilt %.1f deg; log out/hold.log\n", h.mean_off, h.tilt_deg);
        return 0;
    }
    if (argc > 1) {
        printf("usage: sitl                                 every test\n"
               "       sitl --parse FRAMES OUT              parser round trip\n"
               "       sitl --fly WIND FRAMES [NAME=V ...]  fly mission.py's frames in WIND m/s\n"
               "       sitl --hold WIND [NAME=V ...]        30 s position hold with your gains\n");
        return 1;
    }
    return flight_tests();
}
