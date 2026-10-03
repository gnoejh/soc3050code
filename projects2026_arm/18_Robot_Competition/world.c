/*
 * world.c - the sumo ring and two robots: physics and sensor synthesis
 *
 * One step of the world, every 5 ms of match time:
 *
 *   1. MOTORS   each wheel's motor pushes on the ground with a force that
 *               falls as the wheel speeds up (a DC motor's torque-speed line)
 *               - but never more than the tyre's grip, mu * N.  More than
 *               that and the wheel SLIPS: the encoder spins, the robot does not.
 *   2. TYRES    sideways, a tyre resists sliding up to MU_LAT * m * g - less
 *               than its forward grip.  This one number is why a push from
 *               the SIDE beats a push from the FRONT.
 *   3. CONTACT  two circles that overlap exchange an impulse along the line
 *               between their centres (they stop closing), plus friction
 *               along the surface (a glancing hit turns the other robot).
 *   4. MOVE     integrate position and heading.
 *
 * Nothing here knows who is "winning".  Pushing power, angle and grip decide
 * that, the way they do on a real dohyo.
 */
#include "world.h"
#include "fmath.h"

#define N_WHEEL   (ROBOT_M * GRAVITY * 0.5f)      /* normal force per wheel */

/* fmath.h's functions are `static inline`: every call site gets its own copy
 * of the polynomial.  One out-of-line copy each saves flash; the result is
 * bit-for-bit the same.  scene.c uses them too. */
float w_sin(float a) { return f_sin(a); }
float w_cos(float a) { return f_cos(a); }

void world_place(world_t *w, int i, float x, float y, float th, uint32_t seed)
{
    body_t *b = &w->b[i];
    b->x = x;  b->y = y;  b->th = f_wrap(th);
    b->vx = b->vy = b->w = 0.0f;
    b->enc_l = b->enc_r = 0.0f;
    b->drive_l = b->drive_r = 0.0f;
    b->bump = 0;  b->slip = 0;
    b->rng = seed ? seed : 0x9E3779B9u;           /* xorshift must not be 0 */
    w->touching = 0;
    w->steps = 0;
}

void world_drive(world_t *w, int i, const robot_cmd_t *cmd)
{
    float l = (float)cmd->left  * 0.01f;
    float r = (float)cmd->right * 0.01f;
    w->b[i].drive_l = f_clamp(l, -1.0f, 1.0f);
    w->b[i].drive_r = f_clamp(r, -1.0f, 1.0f);
}

/* ---- 1 + 2: one wheel's traction force; updates its encoder -------------- */
static float wheel(float drive, float v_ground, float *enc, uint8_t *slip, uint8_t bit)
{
    const float grip = MU_LONG * N_WHEEL;
    float f = F_STALL * (drive - v_ground / V_MAX);     /* the motor's line   */
    float v_wheel = v_ground;
    if (f > grip || f < -grip) {                        /* more than the tyre */
        f = f > 0.0f ? grip : -grip;                    /* can hold: it slips */
        v_wheel = V_MAX * (drive - f / F_STALL);        /* and spins at the   */
        *slip |= bit;                                   /* speed where motor  */
    }                                                   /* force = grip       */
    *enc += v_wheel * DT;
    return f;
}

static void drive_body(body_t *b)
{
    float c = w_cos(b->th), s = w_sin(b->th);
    float v_long =  b->vx * c + b->vy * s;              /* along the wheels   */
    float v_lat  = -b->vx * s + b->vy * c;              /* sideways           */

    b->slip = 0;
    float fl = wheel(b->drive_l, v_long - b->w * HALF_TRACK, &b->enc_l, &b->slip, 1u);
    float fr = wheel(b->drive_r, v_long + b->w * HALF_TRACK, &b->enc_r, &b->slip, 2u);

    v_long += (fl + fr) * (DT / ROBOT_M);
    b->w   += (fr - fl) * HALF_TRACK * (DT / ROBOT_I);

    /* Sideways the tyres stop the slide - up to their limit each step. */
    const float lat_max = MU_LAT * GRAVITY * DT;        /* delta-v per step   */
    v_lat -= f_clamp(v_lat, -lat_max, lat_max);

    b->vx = v_long * c - v_lat * s;
    b->vy = v_long * s + v_lat * c;
}

/* Which side of robot `b` faces direction (nx, ny)? */
static uint8_t bump_side(const body_t *b, float nx, float ny)
{
    float c = w_cos(b->th), s = w_sin(b->th);
    float lx =  nx * c + ny * s;                        /* in the robot frame */
    float ly = -nx * s + ny * c;
    if (lx >  0.7071f) { return BUMP_FRONT; }
    if (lx < -0.7071f) { return BUMP_BACK; }
    return ly > 0.0f ? BUMP_LEFT : BUMP_RIGHT;
}

/* ---- 3: circle-circle contact --------------------------------------------- */
static void contact(world_t *w)
{
    body_t *a = &w->b[0], *b = &w->b[1];
    float dx = b->x - a->x, dy = b->y - a->y;
    float d2 = dx * dx + dy * dy;
    const float reach = 2.0f * ROBOT_R;
    w->touching = 0;
    if (d2 >= reach * reach) { return; }

    float d = f_sqrt(d2);
    float nx = 1.0f, ny = 0.0f;                         /* a -> b              */
    if (d > 1e-6f) { nx = dx / d;  ny = dy / d; }
    w->touching = 1;
    a->bump |= bump_side(a, nx, ny);
    b->bump |= bump_side(b, -nx, -ny);

    /* Velocities of the two surface points that touch. */
    float avx = a->vx - a->w * (ny * ROBOT_R), avy = a->vy + a->w * (nx * ROBOT_R);
    float bvx = b->vx - b->w * (-ny * ROBOT_R), bvy = b->vy + b->w * (-nx * ROBOT_R);
    float rvx = bvx - avx, rvy = bvy - avy;
    float vn  = rvx * nx + rvy * ny;

    if (vn < 0.0f) {                                    /* closing: collide   */
        float jn = -(1.0f + RESTITUTION) * vn / (2.0f / ROBOT_M);
        a->vx -= jn * nx / ROBOT_M;  a->vy -= jn * ny / ROBOT_M;
        b->vx += jn * nx / ROBOT_M;  b->vy += jn * ny / ROBOT_M;

        /* Friction along the surface, limited by Coulomb: |jt| <= mu jn.
         * For a disc, a tangential impulse at the rim also spins it. */
        float tx = -ny, ty = nx;
        float vt = rvx * tx + rvy * ty;
        float k  = 2.0f / ROBOT_M + 2.0f * ROBOT_R * ROBOT_R / ROBOT_I;
        float jt = f_clamp(-vt / k, -MU_BODY * jn, MU_BODY * jn);
        a->vx -= jt * tx / ROBOT_M;  a->vy -= jt * ty / ROBOT_M;
        b->vx += jt * tx / ROBOT_M;  b->vy += jt * ty / ROBOT_M;
        a->w  -= ROBOT_R * jt / ROBOT_I;   /* r_a x t = +R, r_b x t = +R too */
        b->w  -= ROBOT_R * jt / ROBOT_I;
    }

    /* Do not let them sink into each other: split the overlap. */
    float half = 0.5f * (reach - d);
    a->x -= nx * half;  a->y -= ny * half;
    b->x += nx * half;  b->y += ny * half;
}

/* ---- the step ---------------------------------------------------------------- */
void world_step(world_t *w)
{
    drive_body(&w->b[0]);
    drive_body(&w->b[1]);
    contact(w);
    for (int i = 0; i < 2; i++) {                       /* 4: move            */
        body_t *b = &w->b[i];
        b->x  += b->vx * DT;
        b->y  += b->vy * DT;
        b->th  = f_wrap(b->th + b->w * DT);
    }
    w->steps++;
}

float world_radius(const world_t *w, int i)
{
    const body_t *b = &w->b[i];
    return f_sqrt(b->x * b->x + b->y * b->y);
}

/* ---- sensors: the truth, as this robot's hardware would report it ---------- */

/* Uniform noise in [-1, 1), from this robot's own generator. */
static float noise(body_t *b)
{
    return (float)(int32_t)(xorshift32(&b->rng) >> 8) * (1.0f / 8388608.0f) - 1.0f;
}

static uint16_t distance(body_t *me, const body_t *it, float axis)
{
    float c = w_cos(me->th), s = w_sin(me->th);
    float ox = me->x + ROBOT_R * c, oy = me->y + ROBOT_R * s;   /* on the nose */
    float dx = it->x - ox, dy = it->y - oy;
    float dc = f_sqrt(dx * dx + dy * dy);
    float r  = noise(me);                     /* drawn every time: the sequence
                                                 must not depend on what is seen */
    if (dc - ROBOT_R > DIST_RANGE_MM * 0.001f) { return DIST_NONE; }

    float a  = me->th + axis;
    float ax = w_cos(a), ay = w_sin(a);
    float along = dx * ax + dy * ay, across = ax * dy - ay * dx;
    float off = f_abs(f_atan2(across, along));          /* axis to its centre */
    float half = F_PI_2;                                /* its angular radius */
    if (dc > ROBOT_R) { half = f_atan2(ROBOT_R, f_sqrt(dc * dc - ROBOT_R * ROBOT_R)); }
    if (off > SENSOR_CONE + half) { return DIST_NONE; }

    if (r > 0.94f) { return DIST_NONE; }                /* 3% of reads: a miss */
    float mm = (dc - ROBOT_R) * 1000.0f;
    mm += r * (8.0f + 0.03f * mm);                      /* +-8 mm +-3%         */
    if (mm < 0.0f) { mm = 0.0f; }
    if (mm > (float)DIST_RANGE_MM) { return DIST_NONE; }
    return (uint16_t)mm;
}

static uint8_t edge(const body_t *b)
{
    static const float corner[4][2] = {
        { EDGE_POS,  EDGE_POS }, { EDGE_POS, -EDGE_POS },   /* FL, FR */
        {-EDGE_POS,  EDGE_POS }, {-EDGE_POS, -EDGE_POS },   /* BL, BR */
    };
    const float in2 = (RING_R - LINE_W) * (RING_R - LINE_W), out2 = RING_R * RING_R;
    float c = w_cos(b->th), s = w_sin(b->th);
    uint8_t bits = 0;
    for (int k = 0; k < 4; k++) {
        float x = b->x + corner[k][0] * c - corner[k][1] * s;
        float y = b->y + corner[k][0] * s + corner[k][1] * c;
        float r2 = x * x + y * y;
        /* White only ON the border.  Past RING_R there is no dohyo under the
         * sensor and it reads like black - a robot fast enough to cross the
         * line between two reads never sees it. */
        if (r2 >= in2 && r2 <= out2) { bits |= (uint8_t)(1u << k); }
    }
    return bits;
}

void world_sense(world_t *w, int i, robot_view_t *v)
{
    body_t *me = &w->b[i];
    const body_t *it = &w->b[1 - i];
    v->dist_mm[DIST_LEFT]   = distance(me, it,  SENSOR_SPREAD);
    v->dist_mm[DIST_CENTRE] = distance(me, it,  0.0f);
    v->dist_mm[DIST_RIGHT]  = distance(me, it, -SENSOR_SPREAD);
    v->edge  = edge(me);
    v->bump  = me->bump;
    me->bump = 0;
    v->enc_l = (int32_t)(me->enc_l * 1000.0f);
    v->enc_r = (int32_t)(me->enc_r * 1000.0f);
}
