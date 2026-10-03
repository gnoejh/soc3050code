/*
 * world.c - a differential-drive robot on a taped floor, in 5 ms steps
 *
 * THE MODEL, top to bottom - each line is one block of world_step():
 *
 *   1. MOTORS.   Each wheel's speed chases its command with a 40 ms lag, and
 *                cannot change faster than 8 m/s^2.  A real motor has
 *                inertia; a controller that assumes it does not will overshoot.
 *   2. STEERING. The wheels set the turn rate: omega = (wr - wl) / track.
 *   3. TYRES.    The body's velocity over the ground is pulled toward where
 *                the robot points - but by at most 0.7 g.  That limit is the
 *                FRICTION CIRCLE: braking, accelerating and cornering all
 *                share it.  Ask for v^2/R more than 0.7 g in a corner and the
 *                robot keeps pointing into the turn while it slides OUT of it,
 *                off the line.  That is what makes speed a gamble.
 *   4. ENCODERS. Count WHEEL rotation, not ground travel - so while sliding
 *                they lie, exactly as real ones do.
 *   5. SENSORS.  Each of seven downward-looking sensors sees a 4 mm spot.
 *                Its reading rises smoothly as the spot slides onto the
 *                19 mm tape, plus ambient light, plus noise.
 *   6. JUDGE.    Which piece of track is the robot on, how far along, how
 *                far off?  Laps, best lap, and DNF.
 */
#include "world.h"
#include "track.h"
#include "fmath.h"

world_t world;

#define SPOT_MM      4.0f        /* each sensor sees a spot this big           */
#define WHITE        100.0f      /* counts over white floor                    */
#define BLACK        900.0f      /* counts over tape                           */
#define TICKS_PER_MM ((float)ENC_TICKS_REV / (2.0f * F_PI * WHEEL_R_MM))

/* ---- a little noise: xorshift32, then a sum of three uniforms ----------- */
static uint32_t rnd(void)
{
    uint32_t x = world.rng;
    x ^= x << 13;  x ^= x >> 17;  x ^= x << 5;
    world.rng = x;
    return x;
}

static float gauss(void)          /* mean 0, standard deviation ~1, |x| <= 3 */
{
    int32_t sum = (int32_t)(rnd() & 0xFFFu) + (int32_t)(rnd() & 0xFFFu)
                + (int32_t)(rnd() & 0xFFFu) - 3 * 2048;
    return (float)sum * (2.0f / 4096.0f);
}

void world_reset(int track_id)
{
    int16_t noise = world.noise, ambient = world.ambient;
    int8_t  failed = world.failed;
    uint32_t seed = world.rng ? world.rng : 0x2545F491u;

    track_build(track_id);
    world_t w = {0};
    w.x  = track.x0;
    w.y  = track.y0;
    w.th = track.th0;
    w.noise = noise;  w.ambient = ambient;  w.failed = failed;  w.rng = seed;
    w.state = W_READY;
    world = w;
}

void world_start(void)
{
    world.state     = W_RUN;
    world.lap_start = world.t_ms;
}

/* ---- the judge: where along the track, how far off ---------------------- */
static void locate(void)
{
    /* Search a few segments either side of the last one, not the whole
     * track: cheaper, and on the FIGURE-8 it keeps the judge on the branch
     * the robot is really on when the two cross. */
    int   nseg = track.n - 1, best = world.seg, side = 1, bside = 1;
    float t, bt = 0.0f, bd = 1e9f;
    for (int k = -4; k <= 6; k++) {
        int i = (world.seg + k + nseg) % nseg;
        float d = track_seg_dist(i, world.x, world.y, &t, &side);
        if (d < bd) { bd = d; best = i; bt = t; bside = side; }
    }
    world.seg = best;
    float s0 = (float)track.pt[best].s, s1 = (float)track.pt[best + 1].s;
    world.s   = s0 + bt * (s1 - s0);
    world.err = bd * (float)bside;
}

/* ---- the sensors: coverage of each spot by tape ------------------------- */
static void cover(void)
{
    float c = f_cos(world.th), s = f_sin(world.th);
    float bx = world.x + BAR_AHEAD_MM * c, by = world.y + BAR_AHEAD_MM * s;

    /* Which segments could any sensor be over?  A cheap integer box test
     * first; the exact distance only for the few that pass.  On a crossing
     * BOTH branches pass, so every sensor sees the cross line. */
    int cand[24], nc = 0;
    const int reach = 39 + 10 + 6 + 2;            /* half bar + tape + spot + slack */
    int ibx = (int)bx, iby = (int)by;
    for (int i = 0; i < track.n - 1 && nc < 24; i++) {
        if (track_is_gap(i)) { continue; }
        int ax = track.pt[i].x, ay = track.pt[i].y, cx = track.pt[i + 1].x, cy = track.pt[i + 1].y;
        int lox = ax < cx ? ax : cx, hix = ax < cx ? cx : ax;
        int loy = ay < cy ? ay : cy, hiy = ay < cy ? cy : ay;
        if (ibx + reach < lox || ibx - reach > hix || iby + reach < loy || iby - reach > hiy) { continue; }
        cand[nc++] = i;
    }
    world.cand = (uint16_t)nc;
    for (int k = 0; k < LINE_SENSORS; k++) {
        float off = (float)((k - (LINE_SENSORS - 1) / 2) * SENSOR_PITCH_MM);   /* + = left */
        float px = bx - off * s, py = by + off * c;
        float dmin = 1e9f, t;
        int side;
        for (int j = 0; j < nc; j++) {
            float d = track_seg_dist(cand[j], px, py, &t, &side);
            if (d < dmin) { dmin = d; }
        }
        /* 1 with the spot wholly on tape, 0 wholly off, a ramp between */
        world.cov[k] = f_clamp((TAPE_HALF_MM + SPOT_MM - dmin) / (2.0f * SPOT_MM), 0.0f, 1.0f);
    }
}

void world_step(const actuators_t *cmd)
{
    const float dt = WORLD_DT;
    world.t_ms += 1000u / WORLD_HZ;

    /* 1. motors - only while racing; parked otherwise */
    float ul = 0.0f, ur = 0.0f;
    if (world.state == W_RUN) {
        ul = f_clamp((float)cmd->left,  -WHEEL_MAX_MMS, WHEEL_MAX_MMS);
        ur = f_clamp((float)cmd->right, -WHEEL_MAX_MMS, WHEEL_MAX_MMS);
    }
    float dl = f_clamp((ul - world.wl) * (dt / MOTOR_TAU_S), -MOTOR_ACC_MMS2 * dt, MOTOR_ACC_MMS2 * dt);
    float dr = f_clamp((ur - world.wr) * (dt / MOTOR_TAU_S), -MOTOR_ACC_MMS2 * dt, MOTOR_ACC_MMS2 * dt);
    world.wl += dl;
    world.wr += dr;

    /* 2. steering: the wheels turn the body */
    float v     = 0.5f * (world.wl + world.wr);
    float omega = (world.wr - world.wl) / ROBOT_TRACK_MM;
    world.th    = f_wrap(world.th + omega * dt);

    /* 3. tyres: pull the ground velocity toward the heading - within 0.7 g */
    float c = f_cos(world.th), s = f_sin(world.th);
    float ax = TYRE_K * (v * c - world.vx), ay = TYRE_K * (v * s - world.vy);
    float a2 = ax * ax + ay * ay;
    world.slipping = (uint8_t)(a2 > MU_G_MMS2 * MU_G_MMS2);
    if (world.slipping) {
        float k = MU_G_MMS2 / f_sqrt(a2);          /* on the circle's edge */
        ax *= k;  ay *= k;
        if (world.state == W_RUN) { world.slip_ms += 1000u / WORLD_HZ; }
    }
    world.vx += ax * dt;
    world.vy += ay * dt;
    world.x  += world.vx * dt;
    world.y  += world.vy * dt;

    /* 4. encoders: what the wheels turned, slip or no slip */
    world.dist_l += world.wl * dt;
    world.dist_r += world.wr * dt;

    /* 5. what the sensors would see */
    cover();

    /* 6. the judge */
    locate();
    if (world.state != W_RUN) { return; }

    float ae = f_abs(world.err);
    if (ae > world.max_err) { world.max_err = ae; }
    world.err2 += world.err * world.err;
    world.err_n++;

    int on_tape = 0;
    for (int k = 0; k < LINE_SENSORS; k++) { if (world.cov[k] > 0.5f) { on_tape = 1; } }
    world.white_ms = on_tape ? 0u : world.white_ms + 1000u / WORLD_HZ;

    float L = (float)track.length;
    if (world.s > 0.4f * L && world.s < 0.6f * L) { world.halfway = 1; }
    if (world.halfway && world.s < 0.1f * L && world.seg < (track.n - 1) / 2) {
        /* crossed the start line going forward, having been round */
        uint32_t lap = world.t_ms - world.lap_start;
        world.lap_start = world.t_ms;
        world.last_lap  = lap;
        if (world.best_lap == 0u || lap < world.best_lap) { world.best_lap = lap; }
        world.laps++;
        world.halfway = 0;
    }

    if (world.white_ms > DNF_WHITE_MS) { world.state = W_DNF; world.dnf = DNF_LOST; }
    if (ae > DNF_FAR_MM)               { world.state = W_DNF; world.dnf = DNF_FAR; }
}

void world_sense(sensors_t *out)
{
    for (int k = 0; k < LINE_SENSORS; k++) {
        float r = WHITE + (BLACK - WHITE) * world.cov[k] + (float)world.ambient;
        if (world.noise) { r += (float)world.noise * gauss(); }
        r = f_clamp(r, 0.0f, 1000.0f);
        if (k == world.failed) { r = 0.0f; }          /* a dead sensor reads 0 */
        out->line[k] = (uint16_t)r;
    }
    out->enc_left  = (int32_t)(world.dist_l * TICKS_PER_MM);
    out->enc_right = (int32_t)(world.dist_r * TICKS_PER_MM);
    out->t_ms      = world.t_ms;
}
