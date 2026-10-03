/*
 * world.c - the drone, the air and the sensors.  See world.h for the model.
 *
 * Every constant below is a modelling choice, stated so you can change it.
 * They are typical of a 1-2 kg quadcopter, not of any particular one.
 */
#include "world.h"
#include "fmath.h"
#include "fm.h"

#define TAU_ATT     0.08f    /* s - lesson 13's attitude loop, as a lag        */
#define TAU_THRUST  0.05f    /* s - motor + propeller spin-up                  */
#define TAU_YAW     0.10f    /* s                                              */
#define DRAG_XY     0.35f    /* 1/s - rotor drag: m/s^2 per m/s of AIRSPEED    */
#define DRAG_Z      0.50f    /* 1/s                                            */
#define THRUST_MAX  (2.0f * GRAVITY)   /* thrust-to-weight 2: a sporty camera drone */
#define GUST_TAU    2.0f     /* s - how long a gust lasts                      */
#define GUST_FRAC   0.30f    /* gust strength as a fraction of the steady wind */
#define GPS_WANDER  0.40f    /* m - the slowly wandering part of GPS error     */
#define GPS_TAU     5.0f     /* s - how slowly it wanders                      */
#define GPS_WHITE   0.10f    /* m - the jumpy part                             */
#define BARO_NOISE  0.20f    /* m                                              */
#define ACC_NOISE   0.20f    /* m/s^2                                          */
#define ACC_BIAS_X  0.03f    /* m/s^2 - a slightly mis-calibrated IMU          */
#define ACC_BIAS_Y  (-0.02f)
#define YAW_NOISE   0.01f    /* rad                                            */

/* ---- a deterministic random generator ---------------------------------
 * xorshift32 (Marsaglia 2003): three shifts and three XORs, period 2^32 - 1.
 * Deterministic on purpose: one seed is one exact flight, on the board and on
 * the PC, so a change in the result is a change in YOUR code, not luck. */
static uint32_t rnd(world_t *w)
{
    uint32_t x = w->rng;
    x ^= x << 13;  x ^= x >> 17;  x ^= x << 5;
    w->rng = x;
    return x;
}

static float uni(world_t *w)                    /* uniform, -1 .. 1           */
{
    return (float)((int32_t)(rnd(w) >> 8) - (1 << 23)) * (1.0f / (float)(1 << 23));
}

__attribute__((noinline)) static float gauss(world_t *w)                /* mean 0, std dev 1 (approx.) */
{
    /* Three uniforms of variance 1/3 each sum to variance 1 - and by the
     * central limit theorem the sum is already close to bell-shaped.
     * No logf(), no sqrtf(): cheap on a chip with no FPU. */
    return uni(w) + uni(w) + uni(w);
}

void world_init(world_t *w, uint32_t seed)
{
    *w = (world_t){ 0 };
    w->rng       = seed ? seed : 0x2545F491u;   /* xorshift must not start at 0 */
    w->battery   = 100.0f;
    w->drain     = 0.35f;                       /* %/s at hover: ~4.8 min      */
    w->tau_att   = TAU_ATT;
    w->wind_dir  = 0.52f;                       /* toward ENE, 30 degrees      */
    w->on_ground = 1;
}

void world_set_wind(world_t *w, float speed, float dir_rad)
{
    w->wind_speed = speed < 0.0f ? 0.0f : speed;
    w->wind_dir   = dir_rad;
}

void world_step(world_t *w, const actuators_t *a, float dt)
{
    /* ---- 1. the inner loops, as lags: actual chases commanded ---------- */
    float t_cmd = (a->armed && w->battery > 0.0f) ? f_clamp(a->thrust, 0.0f, THRUST_MAX) : 0.0f;
    float r_cmd = a->armed ? f_clamp(a->roll,  -0.7f, 0.7f) : 0.0f;
    float p_cmd = a->armed ? f_clamp(a->pitch, -0.7f, 0.7f) : 0.0f;
    float y_cmd = a->armed ? a->yaw_rate : 0.0f;
    w->thrust   += (t_cmd - w->thrust)   * dt / TAU_THRUST;
    w->roll     += (r_cmd - w->roll)     * dt / w->tau_att;
    w->pitch    += (p_cmd - w->pitch)    * dt / w->tau_att;
    w->yaw_rate += (y_cmd - w->yaw_rate) * dt / TAU_YAW;
    w->yaw       = m_wrap(w->yaw + w->yaw_rate * dt);

    /* ---- 2. the air: steady wind + gusts ------------------------------- */
    /* Each gust component is an Ornstein-Uhlenbeck process: it decays toward
     * zero with time constant GUST_TAU and is kicked by white noise, scaled
     * so its standard deviation settles at GUST_FRAC x the steady wind.     */
    float sigma = GUST_FRAC * w->wind_speed;
    float kick  = sigma * m_sqrt(2.0f * dt / GUST_TAU);
    w->gust_x += -w->gust_x * dt / GUST_TAU + kick * gauss(w);
    w->gust_y += -w->gust_y * dt / GUST_TAU + kick * gauss(w);
    w->wind_x  = w->wind_speed * m_cos(w->wind_dir) + w->gust_x;
    w->wind_y  = w->wind_speed * m_sin(w->wind_dir) + w->gust_y;

    /* ---- 3. forces: the thrust vector, tilted, then drag --------------- */
    float sr = m_sin(w->roll),  cr = m_cos(w->roll);
    float sp = m_sin(w->pitch), cp = m_cos(w->pitch);
    float sy = m_sin(w->yaw),   cy = m_cos(w->yaw);
    float a_fwd   = w->thrust * sp * cr;          /* along the nose            */
    float a_right = w->thrust * sr;               /* out of the right side     */
    float a_up    = w->thrust * cp * cr;
    /* body -> world: forward is (cos yaw, sin yaw), right is (sin yaw, -cos yaw) */
    float ax = a_fwd * cy + a_right * sy - DRAG_XY * (w->vx - w->wind_x);
    float ay = a_fwd * sy - a_right * cy - DRAG_XY * (w->vy - w->wind_y);
    float az = a_up - GRAVITY - DRAG_Z * w->vz;

    /* ---- 4. the ground ------------------------------------------------- */
    if (w->on_ground) {
        if (az <= 0.0f) {                         /* resting: the ground pushes back */
            ax = ay = az = 0.0f;
            w->vx = w->vy = w->vz = 0.0f;
        } else {
            w->on_ground = 0;                     /* lift-off                   */
        }
    }

    /* ---- 5. integrate: semi-implicit Euler (velocity first) ------------ */
    w->vx += ax * dt;  w->vy += ay * dt;  w->vz += az * dt;
    w->px += w->vx * dt;  w->py += w->vy * dt;  w->pz += w->vz * dt;
    if (w->pz <= 0.0f && !w->on_ground) {         /* touchdown                  */
        float impact = -w->vz;
        if (impact > w->max_impact) { w->max_impact = impact; }
        w->pz = 0.0f;
        w->vx = w->vy = w->vz = 0.0f;
        w->on_ground = 1;
    }
    w->ax = ax;  w->ay = ay;  w->az = az;

    /* ---- 6. the battery: power grows as thrust^1.5 (momentum theory) --- */
    float load = w->thrust / GRAVITY;
    w->battery -= w->drain * load * m_sqrt(load) * dt;
    if (w->battery < 0.0f) { w->battery = 0.0f; }

    w->t_ms += (uint32_t)(dt * 1000.0f + 0.5f);
}

void world_sense(world_t *w, sensors_t *s)
{
    float dt = CTRL_DT;
    s->t_ms     = w->t_ms;
    s->acc_x    = w->ax + ACC_BIAS_X + ACC_NOISE * gauss(w);
    s->acc_y    = w->ay + ACC_BIAS_Y + ACC_NOISE * gauss(w);
    s->acc_z    = w->az + ACC_NOISE * gauss(w);
    s->yaw      = m_wrap(w->yaw + YAW_NOISE * gauss(w));
    s->baro_alt = w->pz + BARO_NOISE * gauss(w);
    s->battery  = w->battery;

    /* GPS: remember where we truly are now, report where we were 200 ms ago */
    w->hist_x[w->hist_i] = w->px;
    w->hist_y[w->hist_i] = w->py;
    w->hist_i = (uint8_t)((w->hist_i + 1u) % (GPS_DELAY + 1u));   /* now the OLDEST */
    float kick = GPS_WANDER * m_sqrt(2.0f * dt / GPS_TAU);
    w->gps_err_x += -w->gps_err_x * dt / GPS_TAU + kick * gauss(w);
    w->gps_err_y += -w->gps_err_y * dt / GPS_TAU + kick * gauss(w);

    s->gps_new = (uint8_t)(w->senses % GPS_DIV == 0u && w->senses >= GPS_DELAY);
    if (s->gps_new) {
        s->gps_x = w->hist_x[w->hist_i] + w->gps_err_x + GPS_WHITE * gauss(w);
        s->gps_y = w->hist_y[w->hist_i] + w->gps_err_y + GPS_WHITE * gauss(w);
    }
    w->senses++;
}
