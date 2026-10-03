/*
 * nav.c - the robot's software, in five layers
 *
 *            sensors_t
 *                |
 *   1 ESTIMATE   encoders + gyro  ->  where do I think I am?   (dead reckoning)
 *   2 MAP        rangefinders     ->  which cells are wall?    (occupancy grid)
 *   3 PLAN       map + goal       ->  a path of cells          (A*, deliberative)
 *   4 FOLLOW     path + pose      ->  speed and turn rate      (pure pursuit)
 *   5 PROTECT    rangefinders     ->  brake, back off          (reactive)
 *                |
 *            actuators_t
 *
 * Layers 3-4 are DELIBERATIVE: slow, clever, and only as good as the map.
 * Layer 5 is REACTIVE: dumb, fast, and looks at this instant's sensors only.
 * It may override everything above it.  Robots have been built this way
 * since Brooks's subsumption architecture (1986) and Gat's three-layer
 * architecture (1998); it is how a car's emergency brake sits under the
 * driver.
 *
 * This file never includes world.h.  It cannot see the true map or pose.
 */
#include <string.h>
#include "fmath.h"
#include "mathx.h"
#include "nav.h"
#include "astar.h"

/* Small helpers, kept out of line so each exists once.  (Measured: -Os was
 * already not inlining them - forcing it saved nothing.  Measure first.) */
#define NOINLINE __attribute__((noinline))

/* ---- tuning ----------------------------------------------------------------- */
#define CAL_SAMPLES     50        /* 2 s of gyro at rest = its bias            */
#define HIT_ADD         3         /* occupancy evidence for "a beam ended here" */
#define MISS_SUB        1         /*   ... and "a beam passed through here"     */
#define LO_MIN         (-4)
#define LO_MAX          6
#define LO_OCC          2         /* >= this: wall                             */
#define LO_FREE        (-1)       /* <= this: floor                            */
#define LOOKAHEAD_MM    220.0f    /* pure pursuit's carrot distance            */
#define TURN_MAX        4.0f      /* rad/s                                     */
#define ARRIVE_MM       120.0f
#define OFFPATH_MM      220.0f
#define BRAKE_MARGIN_MM 25.0f     /* the reactive layer stops this far off     */
#define BRAKE_DECEL     1500.0f   /* mm/s^2: stopping distance v^2 / 2a        */
#define SLOW_BAND_MM    150.0f
#define BACKUP_MS       400u
#define STUCK_MS        1500u
#define N_VETO          4         /* cells the reactive layer has refused      */
#define VETO_MS         20000u    /*   ... and for how long                    */

static const int8_t sensor_deg[N_RANGE] = RANGE_ANGLES_DEG;

static nav_params_t P = { 250, 1, 1, HEAD_GYRO, 1 };
static nav_stats_t  S;

static int8_t   lo[GRID_N];                 /* the map: evidence per cell      */
static uint8_t  cost[GRID_N];               /* the costmap A* searches         */
static uint16_t path[NAV_PATH_MAX];
static uint16_t path_len, path_i;

static uint8_t  mode;
static int      goal_x = -1, goal_y = -1;
static float    goal_mx, goal_my;           /* the goal cell's centre, mm      */
static float    sens_c[N_RANGE], sens_s[N_RANGE];   /* each beam's cos, sin    */
static float    ex, ey, eth;                /* the ESTIMATED pose              */
static int32_t  prev_l, prev_r;
static uint8_t  have_prev, need_plan;
static uint32_t last_t, t0, backup_until, blocked_since;
static int32_t  cal_sum;
static uint16_t cal_n;
static float    gyro_bias;                  /* rad/s                           */
static uint16_t veto_cell[N_VETO];          /* the safety layer's "not there"  */
static uint32_t veto_until[N_VETO];
static uint8_t  veto_next;
static uint16_t plan_start;                 /* where the last search began     */

/* ================================================================== mission == */
void nav_reset(int start_x, int start_y)
{
    memset(lo, 0, sizeof lo);               /* every cell UNKNOWN              */
    memset(&S, 0, sizeof S);
    path_len = path_i = 0;
    mode = NAV_IDLE;
    ex  = (float)(start_x * CELL_MM + CELL_MM / 2);
    ey  = (float)(start_y * CELL_MM + CELL_MM / 2);
    eth = 0.0f;
    have_prev = 0;  need_plan = 0;  cal_sum = 0;  cal_n = 0;  gyro_bias = 0.0f;
    backup_until = blocked_since = 0;
    memset(veto_until, 0, sizeof veto_until);
    for (int i = 0; i < N_RANGE; i++) {       /* the beams' fixed angles, once */
        float a = (float)sensor_deg[i] * (F_PI / 180.0f);
        sens_c[i] = m_cos(a);
        sens_s[i] = m_sin(a);
    }
}

void nav_set_goal(int x, int y)
{
    if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) { return; }
    goal_x = x;  goal_y = y;
    goal_mx = (float)(x * CELL_MM + CELL_MM / 2);
    goal_my = (float)(y * CELL_MM + CELL_MM / 2);
    if (mode == NAV_RUN) { need_plan = 1; }
}
void nav_get_goal(int *x, int *y) { *x = goal_x;  *y = goal_y; }

void nav_start(void)
{
    if (goal_x < 0) { return; }
    mode = NAV_CALIB;
    cal_sum = 0;  cal_n = 0;
}

int nav_mode(void) { return mode; }
const char *nav_mode_name(int m)
{
    static const char *const n[] = { "IDLE", "CALIB", "RUN", "ARRIVED", "NOPATH" };
    return (m >= 0 && m <= NAV_NOPATH) ? n[m] : "?";
}
nav_params_t *nav_params(void) { return &P; }
const nav_stats_t *nav_stats(void) { return &S; }

int nav_cell(int x, int y)
{
    if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) { return MAP_OCC; }
    int8_t v = lo[CELL(x, y)];
    return v >= LO_OCC ? MAP_OCC : (v <= LO_FREE ? MAP_FREE : MAP_UNKNOWN);
}

const uint16_t *nav_path(uint16_t *len, uint16_t *progress)
{
    *len = path_len;  *progress = path_i;
    return path;
}

void nav_pose(float *x, float *y, float *th) { *x = ex;  *y = ey;  *th = eth; }

uint32_t nav_ram_bytes(void)
{
    return sizeof lo + sizeof cost + sizeof path + astar_ram_bytes()
         + sizeof veto_cell + sizeof veto_until;
}

/* ============================================================ 1 ESTIMATE ==
 * Dead reckoning.  Distance comes from the wheels.  Heading comes from the
 * gyro by default - because a 1.5% difference between two wheels turns
 * into 0.25 rad of heading error per metre (1.5% x 1000 mm / 60 mm), while
 * a calibrated gyro drifts by its leftover bias, about 0.001 rad/s. */
static void estimate(const sensors_t *in, float dt)
{
    float dl = (float)(in->enc_l - prev_l) * (1000.0f / TICKS_PER_M);
    float dr = (float)(in->enc_r - prev_r) * (1000.0f / TICKS_PER_M);
    prev_l = in->enc_l;  prev_r = in->enc_r;

    float ds  = 0.5f * (dl + dr);
    float dth = (P.heading == HEAD_WHEELS)
              ? (dr - dl) / (float)WHEELBASE_MM
              : ((float)in->gyro_mrad_s * 0.001f - gyro_bias) * dt;
    float mid = eth + 0.5f * dth;
    ex += ds * m_cos(mid);
    ey += ds * m_sin(mid);
    eth = m_wrap(eth + dth);
}

/* ================================================================ 2 MAP ==
 * Each beam is evidence twice over: every cell it passed through is
 * probably floor, and the cell where it stopped is probably wall.  Evidence
 * is a small signed count per cell (a log-odds occupancy grid in integers):
 * a hit adds 3, a pass subtracts 1, clamped to -4..+6.  One hit makes a wall;
 * three passes un-make it - so the door that shuts in level 5 shows up after
 * one or two echoes, and a ghost from a noisy reading fades. */
static NOINLINE void add(int x, int y, int d)
{
    if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) { return; }
    int v = lo[CELL(x, y)] + d;
    lo[CELL(x, y)] = (int8_t)(v < LO_MIN ? LO_MIN : (v > LO_MAX ? LO_MAX : v));
}

/* Evidence for the cell `d` mm along a ray from (ox, oy) in direction (c, s). */
static NOINLINE void add_at(float ox, float oy, float c, float s, float d, int ev)
{
    add((int)((ox + d * c) * (1.0f / CELL_MM)), (int)((oy + d * s) * (1.0f / CELL_MM)), ev);
}

static void map_beam(float a, uint16_t range)
{
    float c = m_cos(a), s = m_sin(a);
    float ox = ex + ROBOT_R_MM * c, oy = ey + ROBOT_R_MM * s;
    int hit = range < RANGE_NONE;
    /* no echo: the floor is clear out to SOMEWHERE - but a lost echo also
     * reads "nothing", so only trust it part of the way */
    float free_to = hit ? (float)range - 30.0f : (float)RANGE_MAX_MM - 150.0f;
    int lastx = -99, lasty = -99;
    for (float d = 0.0f; d < free_to; d += 50.0f) {   /* each cell once per beam */
        int x = (int)((ox + d * c) * (1.0f / CELL_MM)), y = (int)((oy + d * s) * (1.0f / CELL_MM));
        if (x != lastx || y != lasty) { add(x, y, -MISS_SUB);  lastx = x;  lasty = y; }
    }
    if (hit) { add_at(ox, oy, c, s, (float)range + 30.0f, HIT_ADD); }   /* 30 mm into the wall */
}

static void map_update(const sensors_t *in)
{
    for (int i = 0; i < N_RANGE; i++) {
        map_beam(eth + (float)sensor_deg[i] * (F_PI / 180.0f), in->range_mm[i]);
    }
    /* where the robot is, there is no wall */
    int x = (int)(ex / CELL_MM), y = (int)(ey / CELL_MM);
    if (x >= 0 && y >= 0 && x < GRID_W && y < GRID_H && lo[CELL(x, y)] > LO_FREE) {
        lo[CELL(x, y)] = LO_FREE;
    }
}

/* =============================================================== 3 PLAN == */
void costmap_build(int (*cls)(int x, int y), uint8_t inflate, uint8_t *out)
{
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            int c = cls(x, y);
            out[CELL(x, y)] = c == MAP_OCC ? ASTAR_LETHAL : (c == MAP_UNKNOWN ? COST_UNKNOWN : 0);
        }
    }
    if (inflate == 0u) { return; }
    /* keep-away cost: the highest that any nearby wall imposes (Chebyshev ring) */
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            if (out[CELL(x, y)] != ASTAR_LETHAL) { continue; }
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= GRID_W || ny >= GRID_H) { continue; }
                    int ring = (dx < 0 ? -dx : dx) > (dy < 0 ? -dy : dy) ? (dx < 0 ? -dx : dx) : (dy < 0 ? -dy : dy);
                    if (ring == 0 || ring > inflate) { continue; }
                    uint8_t *o = &out[CELL(nx, ny)];
                    if (*o == ASTAR_LETHAL) { continue; }
                    uint8_t base = (uint8_t)(cls(nx, ny) == MAP_UNKNOWN ? COST_UNKNOWN : 0);
                    uint8_t want = (uint8_t)(base + (ring == 1 ? COST_INFLATE1 : COST_INFLATE2));
                    if (*o < want) { *o = want; }
                }
            }
        }
    }
}

/* The cell I think I am in - clamped, because a lost robot's estimate can
 * wander off the grid, and an index off the end of cost[] is a HardFault. */
static int my_cell(void)
{
    int x = (int)(ex / CELL_MM), y = (int)(ey / CELL_MM);
    x = x < 0 ? 0 : (x >= GRID_W ? GRID_W - 1 : x);
    y = y < 0 ? 0 : (y >= GRID_H ? GRID_H - 1 : y);
    return CELL(x, y);
}

const uint8_t *nav_costmap(void)
{
    costmap_build(nav_cell, P.inflate, cost);
    return cost;
}

static int search(void)
{
    nav_costmap();
    for (int i = 0; i < N_VETO; i++) {   /* cells the reactive layer refused */
        if (last_t < veto_until[i]) { cost[veto_cell[i]] = ASTAR_LETHAL; }
    }
    uint16_t start = (uint16_t)my_cell();
    cost[start] = 0;                     /* wherever I am, I can leave from */
    plan_start = start;
    astar_result_t r;
    uint32_t t = nav_clock();
    int ok = astar_plan(cost, start, CELL(goal_x, goal_y), path, NAV_PATH_MAX, &r);
    S.last_plan_clk = nav_clock() - t;
    if (S.last_plan_clk > S.peak_plan_clk) { S.peak_plan_clk = S.last_plan_clk; }
    S.plans++;
    S.last_expanded = r.expanded;
    if (r.expanded  > S.peak_expanded) { S.peak_expanded = r.expanded; }
    if (r.open_peak > S.peak_open)     { S.peak_open = r.open_peak; }
    S.path_cost = r.cost;
    if (r.overflow) { S.overflows++; }
    path_len = ok ? r.len : 0;
    return ok;
}

/* Plan - and if there is no path, do not believe it at once.  A map built
 * from noisy beams and a drifting pose has false walls in it, and the
 * safety layer's vetoes can close a corridor.  So, like a ROS navigation
 * stack's "recovery behaviours": first forget the vetoes, then forget every
 * wall seen only once or twice, and try again each time.  Only a goal that
 * is STILL unreachable is reported as NOPATH. */
static void plan(int forced)
{
    if (forced) { S.replans++; }
    int ok = search();
    if (!ok) {
        S.recoveries++;
        memset(veto_until, 0, sizeof veto_until);
        ok = search();
    }
    if (!ok) {
        for (uint16_t i = 0; i < GRID_N; i++) {
            if (lo[i] >= LO_OCC && lo[i] < LO_MAX - 1) { lo[i] = 0; }   /* weak wall -> unknown */
        }
        ok = search();
    }
    S.path_len = path_len;
    path_i = 0;
    need_plan = 0;
    if (!ok) { mode = NAV_NOPATH; }
}

/* For the shell's "map" command: plan now, from here, so the dump that
 * follows shows a path and the map it was planned on together. */
int nav_plan_now(void)
{
    if (goal_x < 0) { return 0; }
    uint8_t was = mode;
    plan(1);
    if (was != NAV_RUN) { mode = was; }         /* a report must not end a run */
    return path_len > 0u;
}

uint16_t nav_plan_start(void) { return plan_start; }

int nav_vetoes(uint16_t *cells, int max)
{
    int n = 0;
    for (int i = 0; i < N_VETO && n < max; i++) {
        if (last_t < veto_until[i]) { cells[n++] = veto_cell[i]; }
    }
    return n;
}

static NOINLINE float cx_mm(uint16_t c) { return (float)(CELL_X(c) * CELL_MM + CELL_MM / 2); }
static NOINLINE float cy_mm(uint16_t c) { return (float)(CELL_Y(c) * CELL_MM + CELL_MM / 2); }
static NOINLINE float dist2(float x, float y) { return (x - ex) * (x - ex) + (y - ey) * (y - ey); }

/* Is the plan still good?  It is not if a wall has appeared on it, or if
 * the robot has wandered away from it (a backup, a brake, a bad estimate). */
static int plan_invalid(void)
{
    for (uint16_t k = path_i; k < path_len; k++) {
        int x = CELL_X(path[k]), y = CELL_Y(path[k]);
        if (nav_cell(x, y) == MAP_OCC) { return 1; }
        if (k > path_i) {                 /* a diagonal step whose corner is now a wall */
            int px = CELL_X(path[k - 1u]), py = CELL_Y(path[k - 1u]);
            if (px != x && py != y && (nav_cell(px, y) == MAP_OCC || nav_cell(x, py) == MAP_OCC)) { return 1; }
        }
    }
    float best = 1e9f;
    for (uint16_t k = path_i; k < path_len && k < path_i + 6u; k++) {
        float d = dist2(cx_mm(path[k]), cy_mm(path[k]));
        if (d < best) { best = d; }
    }
    return best > OFFPATH_MM * OFFPATH_MM;
}

/* ============================================================= 4 FOLLOW ==
 * Pure pursuit: pick the path point one LOOKAHEAD ahead (the "carrot"),
 * and drive the circular arc that passes through it.  The arc's curvature
 * is 2 sin(alpha) / L, where alpha is the carrot's bearing - one line of
 * geometry, and it is how most ground robots and many self-driving stacks
 * still follow a path. */
/* Could the body drive straight from here to (tx, ty) without touching a
 * cell the map says is wall?  Checks the centre line and both edges of the
 * body every 25 mm. */
static NOINLINE int in_sight(float tx, float ty)
{
    float dx = tx - ex, dy = ty - ey;
    float L = m_sqrt(dx * dx + dy * dy);
    if (L < 1.0f) { return 1; }
    float ux = dx / L, uy = dy / L;
    for (float d = 0.0f; d <= L; d += 25.0f) {
        for (int side = -1; side <= 1; side++) {
            float px = ex + d * ux - (float)side * ROBOT_R_MM * uy;
            float py = ey + d * uy + (float)side * ROBOT_R_MM * ux;
            if (px < 0.0f || py < 0.0f) { return 0; }
            if (nav_cell((int)(px / CELL_MM), (int)(py / CELL_MM)) == MAP_OCC) { return 0; }
        }
    }
    return 1;
}

static void follow(float *v, float *w)
{
    /* progress: the nearest of the next few path cells */
    float best = 1e9f;
    uint16_t bi = path_i;
    for (uint16_t k = path_i; k < path_len && k < path_i + 6u; k++) {
        float d = dist2(cx_mm(path[k]), cy_mm(path[k]));
        if (d < best) { best = d;  bi = k; }
    }
    path_i = bi;

    /* the carrot: the first path cell a LOOKAHEAD away - but never one the
     * robot cannot see in a straight line, or pure pursuit happily cuts a
     * corner through the wall the path was bending around */
    uint16_t k = path_i;
    while (k + 1u < path_len && dist2(cx_mm(path[k]), cy_mm(path[k])) < LOOKAHEAD_MM * LOOKAHEAD_MM
           && in_sight(cx_mm(path[k + 1u]), cy_mm(path[k + 1u]))) { k++; }
    if (k == path_i && k + 1u < path_len && dist2(cx_mm(path[k]), cy_mm(path[k])) < 40.0f * 40.0f) {
        k++;      /* sitting on the carrot: chasing it would only spin on noise */
    }
    float tx = cx_mm(path[k]), ty = cy_mm(path[k]);
    float dx = tx - ex, dy = ty - ey;
    float L = m_sqrt(dx * dx + dy * dy);
    float alpha = m_wrap(m_atan2(dy, dx) - eth);

    if (f_abs(alpha) > 1.0f) {                /* facing away: turn on the spot */
        *v = 0.0f;
        *w = alpha > 0.0f ? TURN_MAX * 0.6f : -TURN_MAX * 0.6f;
        return;
    }
    float speed = (float)P.speed_mm_s * (1.0f - 0.6f * f_abs(alpha));
    float to_goal = m_sqrt(dist2(goal_mx, goal_my));
    if (speed > 60.0f + 1.5f * to_goal) { speed = 60.0f + 1.5f * to_goal; }   /* ease in */
    *v = speed;
    *w = f_clamp(L > 1.0f ? 2.0f * speed * m_sin(alpha) / L : 0.0f, -TURN_MAX, TURN_MAX);
}

/* ============================================================ 5 PROTECT ==
 * The reactive layer trusts nothing but this instant's beams.  For each echo
 * it asks: is that point inside the strip the body will sweep if it drives
 * straight on?  If so, how far ahead?  Then it caps the speed so the robot
 * can stop in time: v^2 / 2a, plus a margin.  It never looks at the map, so
 * a bad map or a bad pose estimate cannot fool it. */
static float clear_ahead(const sensors_t *in)
{
    float nearest = 1e9f;
    for (int i = 0; i < N_RANGE; i++) {
        if (in->range_mm[i] >= RANGE_NONE) { continue; }
        float r = ROBOT_R_MM + (float)in->range_mm[i];
        float fwd = r * sens_c[i], side = r * sens_s[i];
        if (fwd > 0.0f && f_abs(side) < ROBOT_R_MM + 8.0f) {
            float gap = fwd - ROBOT_R_MM;
            if (gap < nearest) { nearest = gap; }
        }
    }
    return nearest;
}

static int protect(const sensors_t *in, float *v, float *w)
{
    if (in->bumper && in->t_ms >= backup_until) {      /* reflex: always on */
        backup_until = in->t_ms + BACKUP_MS;
        S.backups++;
        /* a bump is a wall the beams missed: put it just in front of the body */
        add_at(ex, ey, m_cos(eth), m_sin(eth), ROBOT_R_MM + 30.0f, HIT_ADD);
    }
    if (in->t_ms < backup_until) {
        *v = -120.0f;  *w = 0.0f;
        need_plan = 1;                                 /* re-plan after it */
        return 1;
    }
    if (!P.reactive || *v <= 0.0f) { blocked_since = 0; return 0; }

    float gap  = clear_ahead(in);
    float stop = BRAKE_MARGIN_MM + (*v) * (*v) / (2.0f * BRAKE_DECEL);
    if (gap < stop) {
        *v = 0.0f;                                     /* turning in place is safe */
        S.brakes++;
        if (!blocked_since) { blocked_since = in->t_ms ? in->t_ms : 1u; }
        if (in->t_ms - blocked_since > STUCK_MS) {
            /* The two layers disagree: the plan says "through there", the
             * beams say "too close".  Neither will give way, so the robot
             * would sit here for ever.  The reactive layer wins, and tells
             * the planner: the next cell on your path is out of bounds for
             * a while.  Re-plan around it. */
            if (path_i + 1u < path_len) {
                veto_cell[veto_next]  = path[path_i + 1u];
                veto_until[veto_next] = in->t_ms + VETO_MS;
                veto_next = (uint8_t)((veto_next + 1u) % N_VETO);
                S.vetoes++;
            }
            need_plan = 1;
            blocked_since = 0;
        }
        return 1;
    }
    blocked_since = 0;
    if (gap < stop + SLOW_BAND_MM) {
        *v *= (gap - stop) / SLOW_BAND_MM;
        S.brakes++;
        return 1;
    }
    return 0;
}

/* ======================================================== control_step == */
void control_step(const sensors_t *in, actuators_t *out)
{
    float dt = have_prev ? (float)(in->t_ms - last_t) * 0.001f : CONTROL_MS * 0.001f;
    last_t = in->t_ms;
    if (!have_prev) { prev_l = in->enc_l;  prev_r = in->enc_r;  have_prev = 1; }

    float v = 0.0f, w = 0.0f;

    if (mode == NAV_CALIB) {                  /* standing still: the gyro's reading IS its bias */
        prev_l = in->enc_l;  prev_r = in->enc_r;
        cal_sum += in->gyro_mrad_s;
        if (++cal_n >= CAL_SAMPLES) {
            gyro_bias = P.gyro_cal ? (float)cal_sum / (float)cal_n * 0.001f : 0.0f;
            S.gyro_bias_urad_s = (int32_t)(gyro_bias * 1e6f);
            mode = NAV_RUN;
            t0 = in->t_ms;
            need_plan = 1;
        }
    } else if (mode == NAV_RUN) {
        estimate(in, dt);
    } else {
        prev_l = in->enc_l;  prev_r = in->enc_r;   /* parked: nothing moves */
    }

    map_update(in);

    if (mode == NAV_RUN) {
        if (dist2(goal_mx, goal_my) < ARRIVE_MM * ARRIVE_MM) {
            mode = NAV_ARRIVED;
            S.run_ms = in->t_ms - t0;
            path_i = path_len ? (uint16_t)(path_len - 1u) : 0u;
        } else {
            if (in->t_ms >= backup_until) {
                if (need_plan || path_len == 0u) {
                    plan(S.plans > 0u);
                } else if (plan_invalid() || (path_i + 1u >= path_len && path[path_len - 1u] != CELL(goal_x, goal_y))) {
                    plan(1);
                }
            }
            if (mode == NAV_RUN && path_len > 0u) { follow(&v, &w); }
            protect(in, &v, &w);
        }
    }

    /* (v, w) -> two wheels; if either is over the limit, scale both */
    float l = v - w * (0.5f * WHEELBASE_MM), r = v + w * (0.5f * WHEELBASE_MM);
    float m = f_abs(l) > f_abs(r) ? f_abs(l) : f_abs(r);
    if (m > SPEED_MAX_MM) { l *= SPEED_MAX_MM / m;  r *= SPEED_MAX_MM / m; }
    out->wheel_l_mm_s = (int16_t)l;
    out->wheel_r_mm_s = (int16_t)r;
}
