/*
 * flight.c - SOC3050 lesson 14: the flight computer
 *
 * One call per 20 ms, control_step(), does four things in order:
 *
 *   1. ESTIMATE  where am I?   accelerometer prediction, corrected by a GPS
 *                              fix that is 200 ms old (slides 8-10)
 *   2. PROTECT   am I allowed  geofence and battery failsafes (slide 16)
 *                to be here?
 *   3. DECIDE    where should  the mode: TAKEOFF, HOLD, MISSION, RTL, LAND
 *                I be going?   turn into a velocity setpoint (slides 12-15)
 *   4. CONTROL   get there     velocity PI -> acceleration -> tilt + thrust
 *                              (slide 11)
 *
 * The flight computer never sees world.c's truth - only sensors_t.  That
 * discipline is what lets the same file fly a real airframe.
 *
 * Pure C; floats, using fmath.h for the trigonometry (lesson 10 measures what
 * a float costs on this core; slide 21 measures what this loop costs).
 */
#include <stddef.h>
#include <string.h>
#include "flight.h"
#include "fmath.h"
#include "fm.h"

static fc_t fc;

fc_t *fc_get(void) { return &fc; }

/* ============================================================================
 *  Names - for telemetry, the OLED and $ACK
 * ============================================================================ */
static const char *const mode_names[M_COUNT] =
    { "DISARMED", "TAKEOFF", "HOLD", "MISSION", "RTL", "LAND" };
static const char *const reason_names[R_COUNT] =
    { "CMD", "BUTTON", "TKOFF", "DONE", "HOME", "LANDED", "FENCE", "BAT", "BATCRIT" };
static const char *const error_names[FC_E_COUNT] =
    { "OK", "RANGE", "FENCE", "EMPTY", "STATE", "PREARM", "PARAM", "CHECKSUM", "UNKNOWN" };

const char *fc_mode_name(uint8_t m)   { return m < M_COUNT ? mode_names[m] : "?"; }
const char *fc_reason_name(uint8_t r) { return r < R_COUNT ? reason_names[r] : "?"; }
const char *fc_error_name(int e)      { return (e >= 0 && e < FC_E_COUNT) ? error_names[e] : "?"; }

/* ============================================================================
 *  Parameters: $PARAM,NAME,value sets one, as an integer in the unit shown
 * ============================================================================ */
typedef struct {
    const char *name;
    uint16_t    offset;     /* into fc_params_t                                */
    float       unit;       /* float value = integer x unit                    */
    int32_t     lo, hi;     /* allowed integer range                           */
} param_def_t;

#define P(n, field, unit, lo, hi) { n, (uint16_t)offsetof(fc_params_t, field), unit, lo, hi }
static const param_def_t params[] = {
    P("KP_POS",   kp_pos,    0.001f,     0,  5000),   /* x1000 */
    P("KP_VEL",   kp_vel,    0.001f,     0, 10000),
    P("KI_VEL",   ki_vel,    0.001f,     0,  5000),
    P("KP_Z",     kp_z,      0.001f,     0,  5000),
    P("KV_Z",     kv_z,      0.001f,     0, 10000),
    P("KI_Z",     ki_z,      0.001f,     0,  5000),
    P("CRUISE",   cruise,    0.01f,     50,  1200),   /* cm/s */
    P("HOLD_V",   hold_vmax, 0.01f,     50,  1000),   /* cm/s */
    P("TILT",     tilt_max,  0.0174533f, 5,    40),   /* degrees */
    P("ACCEPT",   accept,    0.01f,     20,  1000),   /* cm */
    P("FENCE_R",  fence_r,   1.0f,      10,   200),   /* m */
    P("FENCE_ALT",fence_alt, 1.0f,       5,   120),   /* m */
    P("RTL_ALT",  rtl_alt,   0.1f,      20,   500),   /* dm */
    P("TKOFF",    tkoff_alt, 0.1f,      10,   300),   /* dm */
    P("BAT_LOW",  bat_low,   1.0f,       0,    90),   /* % */
    P("BAT_CRIT", bat_crit,  1.0f,       0,    90),   /* % */
    P("LINE",     line,      1.0f,       0,     1),
    P("EST_A",    est_a,     0.001f,     0,  1000),   /* x1000 */
    P("EST_B",    est_b,     0.001f,     0,  1000),
};
#define N_PARAMS (sizeof params / sizeof params[0])

static float *param_ptr(const param_def_t *d) { return (float *)((char *)&fc.p + d->offset); }

static const param_def_t *param_find(const char *name)
{
    for (unsigned i = 0; i < N_PARAMS; i++) {
        if (strcmp(params[i].name, name) == 0) { return &params[i]; }
    }
    return 0;
}

int fc_param_set(const char *name, int32_t v)
{
    const param_def_t *d = param_find(name);
    if (!d) { return FC_E_PARAM; }
    if (v < d->lo || v > d->hi) { return FC_E_RANGE; }
    *param_ptr(d) = (float)v * d->unit;
    return FC_OK;
}

int fc_param_get(const char *name, int32_t *v)
{
    const param_def_t *d = param_find(name);
    if (!d) { return FC_E_PARAM; }
    float x = *param_ptr(d) / d->unit;
    *v = (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
    return FC_OK;
}

const char *fc_param_name(unsigned i) { return i < N_PARAMS ? params[i].name : 0; }

/* ============================================================================
 *  Events and mode changes
 * ============================================================================ */
static void push(uint8_t kind, uint8_t a, uint8_t b)
{
    uint8_t next = (uint8_t)((fc.ev_head + 1u) % FC_EVENTS);
    if (next == fc.ev_tail) { fc.ev_lost++; return; }        /* full: count it */
    fc.ev[fc.ev_head] = (fc_event_t){ fc.t_ms, kind, a, b };
    fc.ev_head = next;
}

int fc_event_pop(fc_event_t *e)
{
    if (fc.ev_tail == fc.ev_head) { return 0; }
    *e = fc.ev[fc.ev_tail];
    fc.ev_tail = (uint8_t)((fc.ev_tail + 1u) % FC_EVENTS);
    return 1;
}

/* Every mode change goes through here, so every one is logged and every
 * mode starts from a defined setpoint - never from whatever the last one
 * happened to leave behind. */
static void enter(uint8_t mode, uint8_t reason)
{
    fc.mode = mode;  fc.reason = reason;  fc.t_mode = fc.t_ms;  fc.t_still = fc.t_ms;
    fc.spx = fc.px;  fc.spy = fc.py;  fc.spz = fc.pz;          /* "stay here"     */
    switch (mode) {
    case M_DISARMED:
        fc.ix = fc.iy = fc.iz = 0.0f;
        fc.mission_active = 0;
        break;
    case M_TAKEOFF:
        fc.ix = fc.iy = fc.iz = 0.0f;
        fc.spz = fc.p.tkoff_alt;
        break;
    case M_MISSION:
        fc.leg_a = (vec3_t){ fc.px, fc.py, fc.pz };           /* first leg: from here */
        break;
    case M_RTL: {
        float z = fc.pz > fc.p.rtl_alt ? fc.pz : fc.p.rtl_alt; /* never descend to go home */
        if (z > fc.p.fence_alt - 2.0f) { z = fc.p.fence_alt - 2.0f; }
        fc.spz = z;
        fc.rtl_phase = 0;
        break;
    }
    case M_LAND:
        if (reason == R_HOME) { fc.spx = 0.0f; fc.spy = 0.0f; } /* land ON home    */
        break;
    default:
        break;
    }
    push(EV_MODE, mode, reason);
}

/* ============================================================================
 *  Commands - from the link task ($-frames), the buttons, or a host test
 * ============================================================================ */
static int prearm_ok(void)
{
    /* Real autopilots refuse to arm without a GPS lock and a healthy battery.
     * Ten fixes is one second of GPS: enough for the estimate to settle. */
    return fc.gps_fixes >= 10u && fc.battery >= fc.p.bat_low + 5.0f;
}

int fc_arm(void)
{
    if (fc.mode != M_DISARMED) { return FC_E_STATE; }
    if (!prearm_ok())          { return FC_E_PREARM; }
    enter(M_TAKEOFF, R_CMD);
    return FC_OK;
}

int fc_disarm(void)
{
    if (fc.mode != M_DISARMED && fc.pz > 0.3f) { return FC_E_STATE; }  /* not in the air */
    enter(M_DISARMED, R_CMD);
    return FC_OK;
}

int fc_set_mode(uint8_t mode, uint8_t reason)
{
    if (fc.mode == M_DISARMED) { return FC_E_STATE; }   /* arm first          */
    switch (mode) {
    case M_HOLD: case M_RTL: case M_LAND:
        break;
    case M_MISSION:                                      /* resume             */
        if (!fc.mission_active || fc.cur >= fc.n_wp) { return FC_E_EMPTY; }
        break;
    default:
        return FC_E_RANGE;
    }
    if (mode != fc.mode) { enter(mode, reason); }
    return FC_OK;
}

int fc_wp_set(uint8_t idx, float x, float y, float z)
{
    if (idx >= FC_MAX_WP)       { return FC_E_RANGE; }
    if (fc.mode == M_MISSION)   { return FC_E_STATE; }  /* not mid-flight      */
    fc.wp[idx] = (vec3_t){ x, y, z };
    fc.wp_set |= (uint16_t)(1u << idx);
    if (idx + 1u > fc.n_wp) { fc.n_wp = (uint8_t)(idx + 1u); }
    fc.mission_active = 0;                              /* changed: re-START   */
    return FC_OK;
}

void fc_mission_clear(void)
{
    fc.wp_set = 0;  fc.n_wp = 0;  fc.cur = 0;  fc.mission_active = 0;
    if (fc.mode == M_MISSION) { enter(M_HOLD, R_CMD); }
}

int fc_mission_start(void)
{
    /* Check the whole mission BEFORE flying any of it: no gaps, every point
     * inside the fence and above the ground.  A mission that would trip the
     * geofence halfway is refused here, on the ground. */
    if (fc.n_wp == 0u) { return FC_E_EMPTY; }
    for (uint8_t i = 0; i < fc.n_wp; i++) {
        if (!(fc.wp_set & (1u << i))) { return FC_E_EMPTY; }
        const vec3_t *w = &fc.wp[i];
        if (w->x * w->x + w->y * w->y > fc.p.fence_r * fc.p.fence_r) { return FC_E_FENCE; }
        if (w->z < 1.0f || w->z > fc.p.fence_alt)                      { return FC_E_FENCE; }
    }
    if (fc.mode == M_DISARMED) {
        if (!prearm_ok()) { return FC_E_PREARM; }
        fc.cur = 0;
        enter(M_TAKEOFF, R_CMD);
        fc.mission_active = 1;            /* TAKEOFF hands over to MISSION    */
    } else {
        fc.cur = 0;
        fc.mission_active = 1;
        if (fc.mode != M_TAKEOFF) { enter(M_MISSION, R_CMD); }
    }
    push(EV_MISSION, fc.n_wp, 0);
    return FC_OK;
}

/* The mission button A flies when no host has sent one: five waypoints, a
 * climb and a descent, legs at every angle to the wind.  host/default.mission
 * is the same mission as a text file. */
void fc_mission_default(void)
{
    static const int16_t dm[][3] = {          /* x, y, alt in decimetres       */
        {  200,    0,  60 }, {  250, 200,  80 }, {    0, 300, 100 },
        { -200,  150,  80 }, { -150, -100, 60 },
    };
    fc_mission_clear();
    for (uint8_t i = 0; i < sizeof dm / sizeof dm[0]; i++) {
        (void)fc_wp_set(i, dm[i][0] * 0.1f, dm[i][1] * 0.1f, dm[i][2] * 0.1f);
    }
}

void fc_nudge(float vx, float vy) { fc.nudge_x = vx;  fc.nudge_y = vy; }

void fc_reset(void)
{
    memset(&fc, 0, sizeof fc);
    fc.p = (fc_params_t){
        .kp_pos = 1.0f, .kp_vel = 2.0f, .ki_vel = 0.6f,
        .kp_z = 1.2f, .kv_z = 3.0f, .ki_z = 1.0f,
        .cruise = 4.0f, .hold_vmax = 3.0f, .tilt_max = 0.436f,  /* 25 degrees */
        .accept = 1.0f, .fence_r = 50.0f, .fence_alt = 30.0f,
        .rtl_alt = 10.0f, .tkoff_alt = 5.0f,
        .bat_low = 25.0f, .bat_crit = 10.0f,
        .line = 1.0f, .est_a = 0.25f, .est_b = 0.04f,
    };
    fc.mode = M_DISARMED;
    fc.battery = 100.0f;
}

/* ============================================================================
 *  1. ESTIMATE - an alpha-beta filter with the accelerometer as its model
 * ============================================================================ */
#define GPS_LAG_STEPS 10u                 /* must match world.c's GPS_DELAY   */
#define GPS_PERIOD_S  0.1f                /* a fix every 100 ms               */

static void estimate(const sensors_t *s)
{
    const float dt = CTRL_DT;

    /* Predict: integrate the accelerometer.  Alone this drifts - a bias of
     * 0.03 m/s^2 is 1.5 m off after 10 s - which is what the GPS is for. */
    fc.vx += s->acc_x * dt;   fc.vy += s->acc_y * dt;
    fc.px += fc.vx * dt;      fc.py += fc.vy * dt;
    fc.hx[fc.hi] = fc.px;     fc.hy[fc.hi] = fc.py;           /* remember it      */
    uint8_t now = fc.hi;
    fc.hi = (uint8_t)((fc.hi + 1u) % FC_HIST);

    if (s->gps_new) {
        if (fc.gps_fixes == 0u) {                           /* first fix: believe it */
            fc.px = s->gps_x;  fc.py = s->gps_y;  fc.vx = fc.vy = 0.0f;
            for (unsigned k = 0; k < FC_HIST; k++) { fc.hx[k] = fc.px; fc.hy[k] = fc.py; }
        } else {
            /* The fix says where we were 200 ms ago.  Compare it with where we
             * THOUGHT we were 200 ms ago - not with now.  Comparing with now
             * blames the GPS for the distance flown in the last 200 ms. */
            uint8_t then = (uint8_t)((now + FC_HIST - GPS_LAG_STEPS) % FC_HIST);
            float rx = s->gps_x - fc.hx[then];
            float ry = s->gps_y - fc.hy[then];
            float a = fc.p.est_a, b = fc.p.est_b / GPS_PERIOD_S;
            fc.px += a * rx;   fc.py += a * ry;
            fc.vx += b * rx;   fc.vy += b * ry;
            /* ...and correct the remembered past by the same amount, or the
             * next fix is compared with history that never heard of this one. */
            for (unsigned k = 0; k < FC_HIST; k++) { fc.hx[k] += a * rx; fc.hy[k] += a * ry; }
        }
        if (fc.gps_fixes < 255u) { fc.gps_fixes++; }
    }

    /* Altitude: the same idea with no delay - a complementary filter.  The
     * accelerometer is trusted for fast changes, the baro for the average. */
    float r = s->baro_alt - fc.pz;
    fc.vz += (s->acc_z + 4.0f * r) * dt;                    /* omega^2   = 4     */
    fc.pz += (fc.vz    + 2.8f * r) * dt;                    /* 2 zeta w  = 2.8   */
    fc.yaw = s->yaw;
}

/* ============================================================================
 *  2. PROTECT - failsafes outrank every command
 * ============================================================================ */
static void failsafes(void)
{
    uint8_t m = fc.mode;
    if (m == M_DISARMED) { return; }
    if (fc.battery < fc.p.bat_crit && m != M_LAND) {
        enter(M_LAND, R_BATTERY_CRIT);                       /* no time to get home */
        return;
    }
    if (m == M_TAKEOFF || m == M_HOLD || m == M_MISSION) {
        if (fc.battery < fc.p.bat_low) { enter(M_RTL, R_BATTERY); return; }
        float r2 = fc.px * fc.px + fc.py * fc.py;
        if (r2 > fc.p.fence_r * fc.p.fence_r || fc.pz > fc.p.fence_alt) {
            enter(M_RTL, R_FENCE);
        }
    }
}

/* ============================================================================
 *  3. DECIDE - each mode becomes a velocity setpoint
 * ============================================================================ */
static float clampf(float x, float lim) { return f_clamp(x, -lim, lim); }

/* Fly to the position setpoint, no faster than vmax. */
__attribute__((noinline)) static void hold_xy(float *vx, float *vy, float vmax)
{
    float ex = fc.spx - fc.px, ey = fc.spy - fc.py;
    *vx = fc.p.kp_pos * ex;  *vy = fc.p.kp_pos * ey;
    float v = m_sqrt(*vx * *vx + *vy * *vy);
    if (v > vmax) { *vx *= vmax / v;  *vy *= vmax / v; }
}

__attribute__((noinline)) static float alt_to(float z, float v_up, float v_down)
{
    return f_clamp(fc.p.kp_z * (z - fc.pz), -v_down, v_up);
}

/* Fly the leg from A to B.  Returns the horizontal distance to B.
 *
 * LINE = 1: split the error into ALONG the leg and ACROSS it.  The along part
 *           is limited to the cruise speed; the across part gets its own full
 *           correction.  The drone stays on the line in a crosswind.
 * LINE = 0: aim straight at B and limit the whole vector.  Simpler, and the
 *           wind pushes the drone off the line while it re-aims (slide 13). */
static float leg(vec3_t a, vec3_t b, float *vx, float *vy)
{
    float ex = b.x - fc.px, ey = b.y - fc.py;
    float d  = m_sqrt(ex * ex + ey * ey);
    float lx = b.x - a.x, ly = b.y - a.y;
    float len = m_sqrt(lx * lx + ly * ly);
    if (len > 2.0f) { fc.yaw_sp = m_atan2(ly, lx); }       /* face along the leg */

    if (fc.p.line > 0.5f && len > 0.5f) {
        float tx = lx / len, ty = ly / len;                 /* along             */
        float along  = ex * tx + ey * ty;
        float across = ex * -ty + ey * tx;                  /* left of the line  */
        float va = clampf(fc.p.kp_pos * along,  fc.p.cruise);
        float vc = clampf(fc.p.kp_pos * across, fc.p.cruise);
        *vx = va * tx - vc * ty;
        *vy = va * ty + vc * tx;
    } else {
        fc.spx = b.x;  fc.spy = b.y;
        hold_xy(vx, vy, fc.p.cruise);
    }
    return d;
}

static void decide(float *vsx, float *vsy, float *vsz)
{
    const fc_params_t *p = &fc.p;
    *vsx = *vsy = *vsz = 0.0f;

    switch (fc.mode) {
    case M_TAKEOFF:
        hold_xy(vsx, vsy, p->hold_vmax);
        *vsz = alt_to(fc.spz, 2.0f, 1.2f);
        if (fc.pz > fc.spz - 0.3f) {
            if (fc.mission_active) { enter(M_MISSION, R_TKOFF_DONE); }
            else                   { enter(M_HOLD,    R_TKOFF_DONE); }
        }
        break;

    case M_HOLD:
        fc.spx += fc.nudge_x * CTRL_DT;                      /* the joystick moves */
        fc.spy += fc.nudge_y * CTRL_DT;                      /* the setpoint, not  */
        hold_xy(vsx, vsy, p->hold_vmax);                     /* the drone          */
        *vsz = alt_to(fc.spz, 2.0f, 1.2f);
        break;

    case M_MISSION: {
        vec3_t b = fc.wp[fc.cur];
        float d = leg(fc.leg_a, b, vsx, vsy);
        *vsz = alt_to(b.z, 2.0f, 1.2f);
        if (d < p->accept && f_abs(b.z - fc.pz) < 0.7f) {    /* reached           */
            push(EV_WP, fc.cur, 0);
            fc.leg_a = b;                                    /* next leg starts AT
                                                                the waypoint, not
                                                                where we are     */
            if (++fc.cur >= fc.n_wp) {
                fc.mission_active = 0;
                enter(M_RTL, R_MISSION_DONE);
            }
        }
        break;
    }

    case M_RTL:
        if (fc.rtl_phase == 0) {                             /* climb first        */
            hold_xy(vsx, vsy, p->hold_vmax);
            *vsz = alt_to(fc.spz, 2.0f, 1.2f);
            if (f_abs(fc.pz - fc.spz) < 0.4f) {
                fc.rtl_phase = 1;
                fc.leg_a = (vec3_t){ fc.px, fc.py, fc.spz };
            }
        } else {                                             /* then go home       */
            float d = leg(fc.leg_a, (vec3_t){ 0.0f, 0.0f, fc.spz }, vsx, vsy);
            *vsz = alt_to(fc.spz, 2.0f, 1.2f);
            if (d < p->accept) { enter(M_LAND, R_HOME); }
        }
        break;

    case M_LAND:
        hold_xy(vsx, vsy, p->hold_vmax);
        *vsz = fc.pz > 2.5f ? -1.0f : -0.4f;                 /* slow near the ground */
        /* Landed = low, and not moving vertically, for 1.5 s.  On the ground
         * the integrator keeps lowering the thrust, so it stays down. */
        if (!(fc.pz < 0.5f && f_abs(fc.vz) < 0.3f)) { fc.t_still = fc.t_ms; }
        if (fc.t_ms - fc.t_still > 1500u) { enter(M_DISARMED, R_LANDED); }
        break;

    default:
        break;
    }
}

/* ============================================================================
 *  4. CONTROL - velocity PI, then acceleration -> tilt and thrust
 * ============================================================================ */
void control_step(const sensors_t *s, actuators_t *o)
{
    const fc_params_t *p = &fc.p;
    const float dt = CTRL_DT;

    fc.t_ms    = s->t_ms;
    fc.battery = s->battery;
    estimate(s);
    failsafes();

    float vsx, vsy, vsz;
    decide(&vsx, &vsy, &vsz);
    fc.vsp_x = vsx;  fc.vsp_y = vsy;

    if (fc.mode == M_DISARMED) {
        *o = (actuators_t){ 0 };
        fc.out = *o;
        fc.yaw_sp = fc.yaw;
        return;
    }

    /* Velocity PI.  The integral term is the interesting one: in a steady
     * wind, the only way to have zero velocity error is a constant tilt INTO
     * the wind - and the integral is what finds it.  Read ki*ix in a hover
     * and you are reading the wind (Lab Part 3). */
    float a_max = GRAVITY * m_sin(p->tilt_max) / m_cos(p->tilt_max);
    float ex = vsx - fc.vx, ey = vsy - fc.vy, ez = vsz - fc.vz;
    if (fc.pz > 0.5f && p->ki_vel > 0.0f) {                  /* only when flying   */
        float ilim = 0.8f * a_max / p->ki_vel;               /* anti-windup        */
        fc.ix = clampf(fc.ix + ex * dt, ilim);
        fc.iy = clampf(fc.iy + ey * dt, ilim);
    }
    if (p->ki_z > 0.0f) { fc.iz = clampf(fc.iz + ez * dt, 3.0f / p->ki_z); }

    float ax = p->kp_vel * ex + p->ki_vel * fc.ix;
    float ay = p->kp_vel * ey + p->ki_vel * fc.iy;
    float az = f_clamp(p->kv_z * ez + p->ki_z * fc.iz, -0.5f * GRAVITY, 0.6f * GRAVITY);

    float a = m_sqrt(ax * ax + ay * ay);                     /* tilt limit         */
    if (a > a_max) { ax *= a_max / a;  ay *= a_max / a; }

    /* World acceleration -> the body's forward/right, then -> angles.
     * The thrust vector must point along (a_fwd, a_right, g + az). */
    float cy = m_cos(fc.yaw), sy = m_sin(fc.yaw);
    float a_fwd   = ax * cy + ay * sy;
    float a_right = ax * sy - ay * cy;
    float a_up    = GRAVITY + az;
    o->armed    = 1;
    o->thrust   = m_sqrt(a_fwd * a_fwd + a_right * a_right + a_up * a_up);
    o->pitch    = m_atan2(a_fwd, a_up);
    o->roll     = m_atan2(a_right, m_sqrt(a_fwd * a_fwd + a_up * a_up));
    o->yaw_rate = clampf(2.0f * m_wrap(fc.yaw_sp - fc.yaw), 1.2f);
    fc.out = *o;
}
