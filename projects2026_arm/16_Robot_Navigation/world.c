/*
 * world.c - the arena, the robot's true body, and its sensors
 *
 * This is the half of the program that would be REALITY on a real robot.
 * Here it is about 250 lines of C running on the same chip as the robot's
 * software, in its own RTOS task.  Everything in it is deliberately a little
 * wrong, the way reality is:
 *
 *   the motors lag            a wheel reaches its set-point in ~0.1 s, not at once
 *   the wheels are not        left 1.0% bigger than the datasheet, right 0.5%
 *     the size it says          smaller, wheelbase 61.5 mm not 60: odometry drifts
 *   the gyro has a bias       4 mrad/s, plus noise: integrate it and it walks
 *   the rangefinders are      a ray marched in 25 mm steps (it can slip past a
 *     narrow and noisy          corner), +-(3 mm + 1.5%) noise, 1 echo in 64 lost
 *   walls stop the body       but not the wheels: the encoders keep counting
 *
 * Coordinates: millimetres, x right, y UP, heading th in radians CCW from +x.
 * Cell (cx, cy) covers x in [cx*100, cx*100+100), y likewise.
 *
 * Pure C.  Floats are fine here: lesson 10 measured a soft-float multiply at
 * ~50-100 cycles, and this runs 50 times a second.
 */
#include <string.h>
#include "fmath.h"
#include "mathx.h"
#include "world.h"
#include "levels.h"

/* ---- the robot as built (the software believes sim.h's nominal values) ---- */
#define SCALE_L        1.010f     /* true wheel size / datasheet wheel size     */
#define SCALE_R        0.995f
#define WHEELBASE_TRUE 61.5f      /* mm                                          */
#define MOTOR_TAU_S    0.10f      /* first-order lag of each wheel               */
#define SLIP           0.01f      /* random +-1% per step in what the wheels log */
#define GYRO_BIAS      0.004f     /* rad/s                                       */
#define GYRO_NOISE     0.002f     /* rad/s, one sigma                            */
#define RAY_STEP_MM    25.0f      /* the beam is a line sampled this often       */
#define RANGE_SIGMA_MM 3.0f
#define RANGE_SIGMA_K  0.015f     /* plus 1.5% of the distance                   */
#define DROPOUT_MASK   63u        /* 1 in 64 readings: no echo                   */
#define DOOR_TRIGGER_MM 250.0f    /* the door shuts when the robot is this close */

static world_t  W;
static uint32_t occ[GRID_H];      /* bit (31 - x) of occ[y]: a wall              */
static uint32_t door[GRID_H];     /* the cells that become wall                  */
static int      door_x, door_y;   /* the door's centre, mm (0: no door)          */

static uint32_t custom[GRID_H];
static int      cs_x = 2, cs_y = 7, cg_x = 29, cg_y = 7;
static uint8_t  custom_ready;

static float    enc_l, enc_r;     /* fractional encoder ticks                    */
static float    th_total, th_at_sense;   /* unwrapped heading, for the gyro      */
static uint32_t t_at_sense;
static uint32_t rng_state = 0x16u;

const world_t *world(void) { return &W; }

/* ---- noise ---------------------------------------------------------------- */
static uint32_t rng(void)                  /* xorshift32: fast and repeatable   */
{
    uint32_t x = rng_state;
    x ^= x << 13;  x ^= x >> 17;  x ^= x << 5;
    return rng_state = x;
}

static float uniform(void) { return (float)(rng() >> 8) * (1.0f / 16777216.0f); }

static float gauss(void)                   /* sum of 4 uniforms: sigma 1        */
{
    return (uniform() + uniform() + uniform() + uniform() - 2.0f) * 1.7320508f;
}

void world_set_seed(uint32_t seed) { rng_state = seed ? seed : 1u; }

/* ---- the map ----------------------------------------------------------------- */
static uint32_t bit(int x) { return 1u << (31 - x); }

int world_occ(int x, int y)
{
    if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) { return 1; }
    return (occ[y] & bit(x)) != 0u;
}

static int occ_mm(float px, float py)
{
    if (px < 0.0f || py < 0.0f) { return 1; }
    return world_occ((int)px / CELL_MM, (int)py / CELL_MM);
}

/* Does a circle of the robot's radius at (px, py) overlap any wall cell? */
static int collides(float px, float py)
{
    int x0 = (int)((px - ROBOT_R_MM) / CELL_MM) - 1, x1 = (int)((px + ROBOT_R_MM) / CELL_MM);
    int y0 = (int)((py - ROBOT_R_MM) / CELL_MM) - 1, y1 = (int)((py + ROBOT_R_MM) / CELL_MM);
    for (int cy = y0; cy <= y1; cy++) {
        for (int cx = x0; cx <= x1; cx++) {
            if (!world_occ(cx, cy)) { continue; }
            /* nearest point of the cell's square to the centre */
            float nx = f_clamp(px, (float)(cx * CELL_MM), (float)(cx * CELL_MM + CELL_MM));
            float ny = f_clamp(py, (float)(cy * CELL_MM), (float)(cy * CELL_MM + CELL_MM));
            float dx = px - nx, dy = py - ny;
            if (dx * dx + dy * dy < (float)(ROBOT_R_MM * ROBOT_R_MM)) { return 1; }
        }
    }
    return 0;
}

static void load_rows(const char *const rows[GRID_H])
{
    memset(occ, 0, sizeof occ);
    memset(door, 0, sizeof door);
    int nd = 0;
    door_x = door_y = 0;
    for (int r = 0; r < GRID_H; r++) {
        int y = GRID_H - 1 - r;                    /* text is printed top row first */
        for (int x = 0; x < GRID_W; x++) {
            char c = rows[r][x];
            if (c == '#') { occ[y] |= bit(x); }
            if (c == 'D') { door[y] |= bit(x); door_x += x * CELL_MM + 50; door_y += y * CELL_MM + 50; nd++; }
            if (c == 'S') { W.start_x = x; W.start_y = y; }
            if (c == 'G') { W.goal_x = x;  W.goal_y = y; }
        }
    }
    if (nd) { door_x /= nd; door_y /= nd; }
}

static void custom_default(void)
{
    for (int y = 0; y < GRID_H; y++) {
        custom[y] = (y == 0 || y == GRID_H - 1) ? 0xFFFFFFFFu : (bit(0) | bit(GRID_W - 1));
    }
    custom_ready = 1;
}

void world_custom_row(int y, uint32_t bits)
{
    if (!custom_ready) { custom_default(); }
    if (y >= 0 && y < GRID_H) { custom[y] = bits; }
}
void world_custom_start(int x, int y) { cs_x = x; cs_y = y; }
void world_custom_goal(int x, int y)  { cg_x = x; cg_y = y; }
uint32_t world_custom_rowbits(int y)
{
    if (!custom_ready) { custom_default(); }
    return (y >= 0 && y < GRID_H) ? custom[y] : 0u;
}

void world_load(int level)
{
    memset(&W, 0, sizeof W);
    if (level >= 0 && level < N_LEVELS) {
        load_rows(level_table[level].rows);
    } else {
        level = LEVEL_CUSTOM;
        if (!custom_ready) { custom_default(); }
        memcpy(occ, custom, sizeof occ);
        memset(door, 0, sizeof door);
        W.start_x = cs_x;  W.start_y = cs_y;  W.goal_x = cg_x;  W.goal_y = cg_y;
    }
    W.level = level;
    W.x  = (float)(W.start_x * CELL_MM + CELL_MM / 2);
    W.y  = (float)(W.start_y * CELL_MM + CELL_MM / 2);
    W.th = 0.0f;
    enc_l = enc_r = 0.0f;
    th_total = th_at_sense = 0.0f;
    t_at_sense = 0;
}

/* ---- physics: one WORLD_MS step ---------------------------------------------- */
void world_step(const actuators_t *cmd)
{
    const float dt = WORLD_MS * 0.001f;

    /* 1. the motors: each wheel's speed chases its set-point with a lag */
    float cl = f_clamp((float)cmd->wheel_l_mm_s, -SPEED_MAX_MM, SPEED_MAX_MM);
    float cr = f_clamp((float)cmd->wheel_r_mm_s, -SPEED_MAX_MM, SPEED_MAX_MM);
    W.vl += (cl - W.vl) * (dt / MOTOR_TAU_S);
    W.vr += (cr - W.vr) * (dt / MOTOR_TAU_S);

    /* 2. the encoders count wheel ROTATION - the commanded kind of distance -
     *    with a little slip noise, whether or not the body moves */
    enc_l += W.vl * dt * (1.0f + SLIP * gauss()) * (TICKS_PER_M / 1000.0f);
    enc_r += W.vr * dt * (1.0f + SLIP * gauss()) * (TICKS_PER_M / 1000.0f);

    /* 3. the body moves by the TRUE wheel sizes and wheelbase */
    float gl = W.vl * SCALE_L, gr = W.vr * SCALE_R;
    float v = 0.5f * (gl + gr), w = (gr - gl) / WHEELBASE_TRUE;
    float dth = w * dt;
    float mid = W.th + 0.5f * dth;
    float nx = W.x + v * dt * m_cos(mid), ny = W.y + v * dt * m_sin(mid);
    W.th = m_wrap(W.th + dth);
    th_total += dth;

    /* 4. walls: try the move, then slide along one axis, else stay */
    int hit = collides(nx, ny);
    if (hit) {
        if      (!collides(nx, W.y)) { ny = W.y; }
        else if (!collides(W.x, ny)) { nx = W.x; }
        else                         { nx = W.x; ny = W.y; }
    }
    float mx = nx - W.x, my = ny - W.y;
    W.odometer_mm += m_sqrt(mx * mx + my * my);
    W.x = nx;  W.y = ny;
    if (hit && !W.bumper) { W.collisions++; }
    W.bumper = (uint8_t)hit;
    W.t_ms  += WORLD_MS;

    /* 5. level 5's door shuts when the robot gets close - after it has planned */
    if (!W.door_closed && door_x > 0) {
        float dx = W.x - (float)door_x, dy = W.y - (float)door_y;
        if (dx * dx + dy * dy < DOOR_TRIGGER_MM * DOOR_TRIGGER_MM) {
            for (int y = 0; y < GRID_H; y++) { occ[y] |= door[y]; }
            W.door_closed = 1;
        }
    }
}

/* ---- the rangefinder: march along the beam until it is inside a wall ---------- */
static float cast(float ox, float oy, float c, float s)
{
    float d = 0.0f;
    for (; d <= (float)RANGE_MAX_MM; d += RAY_STEP_MM) {
        if (occ_mm(ox + d * c, oy + d * s)) { break; }
    }
    if (d > (float)RANGE_MAX_MM) { return (float)RANGE_NONE; }
    /* found a wall between d - step and d: halve the bracket three times.
     * A wall that the coarse march stepped OVER (a corner thinner than one
     * step along the beam) is never seen at all - real narrow beams miss
     * corners too. */
    float lo = d - RAY_STEP_MM, hi = d;
    if (lo < 0.0f) { return 0.0f; }
    for (int i = 0; i < 3; i++) {
        float m = 0.5f * (lo + hi);
        if (occ_mm(ox + m * c, oy + m * s)) { hi = m; } else { lo = m; }
    }
    return 0.5f * (lo + hi);
}

void world_sense(sensors_t *s)
{
    static const int8_t ang_deg[N_RANGE] = RANGE_ANGLES_DEG;

    s->t_ms   = W.t_ms;
    s->enc_l  = (int32_t)enc_l;
    s->enc_r  = (int32_t)enc_r;
    s->bumper = W.bumper;

    /* gyro: the true average turn rate since the last sample, plus bias, noise */
    float dts = (float)(W.t_ms - t_at_sense) * 0.001f;
    float rate = dts > 0.0f ? (th_total - th_at_sense) / dts : 0.0f;
    rate += GYRO_BIAS + GYRO_NOISE * gauss();
    s->gyro_mrad_s = (int32_t)(rate * 1000.0f + (rate >= 0.0f ? 0.5f : -0.5f));
    th_at_sense = th_total;
    t_at_sense  = W.t_ms;

    for (int i = 0; i < N_RANGE; i++) {
        float a = W.th + (float)ang_deg[i] * (F_PI / 180.0f);
        float c = m_cos(a), sn = m_sin(a);
        float d = cast(W.x + ROBOT_R_MM * c, W.y + ROBOT_R_MM * sn, c, sn);
        if (d < (float)RANGE_NONE) {
            d += (RANGE_SIGMA_MM + RANGE_SIGMA_K * d) * gauss();
            d = f_clamp(d, 0.0f, (float)(RANGE_MAX_MM - 1));
        }
        if ((rng() & DROPOUT_MASK) == 0u) { d = (float)RANGE_NONE; }   /* lost echo */
        s->range_mm[i] = (uint16_t)d;
    }
}
