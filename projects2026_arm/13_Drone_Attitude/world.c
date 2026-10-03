/*
 * world.c - the drone we do not have.   SOC3050 lesson 13
 *
 * See world.h for the model.  Everything here is float, using fmath.h's
 * f_sin/f_atan2/f_sqrt rather than libm, and nothing here touches hardware.
 * The firmware runs world_step() 500 times a second in the highest-priority
 * task; host/sitl.c runs it as fast as the PC can.  Same code, same numbers.
 */
#include "world.h"
#include "fmath.h"

#define G 9.80665f

/* sin(2 pi i / 64) in Q15, for the motor vibration (generated, not typed) */
static const int16_t sin64[64] = {
         0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
     23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
     32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
     23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
         0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
    -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
    -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
    -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212
};

/* ---- a tiny deterministic noise source ----------------------------------
 * xorshift32: three shifts and three XORs.  Seeded, so a host run is exactly
 * repeatable - a test that fails "sometimes" is worth nothing.
 *
 * A bell curve from INTEGERS: the four bytes of one random word are four
 * independent uniforms 0..255; their sum has mean 510 and standard deviation
 * sqrt(4 x (256^2 - 1) / 12) = 147.8, and is already close to Gaussian
 * (central limit theorem).  One int->float conversion and one multiply.
 * The first draft summed four FLOAT uniforms: lesson 10's M0+ model said
 * world_sense() cost 38 000 cycles, most of it here (slide "The Budget"). */
static float gauss(world_t *w)                  /* mean 0, sigma ~1 */
{
    uint32_t x = w->rng;
    x ^= x << 13;  x ^= x >> 17;  x ^= x << 5;
    w->rng = x;
    int32_t sum = (int32_t)((x & 0xFFu) + ((x >> 8) & 0xFFu) + ((x >> 16) & 0xFFu) + (x >> 24));
    return (float)(sum - 510) * (1.0f / 147.8f);
}

/* Derived constants: divisions done once here, not 500 times a second.
 * A soft-float divide costs several multiplies on the M0+. */
static void derive(world_t *w)
{
    w->k_motor = WORLD_DT / w->tau_motor;
    w->inv_tmax = 1.0f / w->t_max;
    w->inv_i[0] = 1.0f / w->ixx;
    w->inv_i[1] = 1.0f / w->iyy;
    w->inv_i[2] = 1.0f / w->izz;
}

void world_reset_attitude(world_t *w)
{
    w->q[0] = 1.0f;  w->q[1] = w->q[2] = w->q[3] = 0.0f;
    w->w[0] = w->w[1] = w->w[2] = 0.0f;
    for (int i = 0; i < 4; i++) { w->thrust[i] = 0.0f; }
    w->crashed = 0;
}

void world_init(world_t *w, uint32_t seed)
{
    w->mass      = 0.5f;
    w->arm       = 0.12f;           /* centre to motor                       */
    w->ixx       = 0.004f;          /* kg m^2                                */
    w->iyy       = 0.004f;
    w->izz       = 0.007f;
    w->t_max     = 4.0f;            /* 4 x 4 N = 16 N: thrust/weight 3.3     */
    w->k_drag    = 0.016f;
    w->tau_motor = 0.030f;
    w->c_damp    = 0.0005f;

    w->gyro_noise  = 0.005f;        /* 0.3 deg/s                             */
    w->gyro_walk   = 0.0005f;
    w->accel_noise = 0.3f;          /* 0.03 g                                */
    w->vib_accel   = 6.0f;          /* ~0.19 g of shake at hover (mean 0.31) */
    /* The MPU6050's digital low-pass filter, DLPF_CFG = 3: 44 Hz, which
     * is what Main.c sets on the real chip.  Modelled as a first-order filter
     * with tau = 1 / (2 pi 44 Hz) = 3.6 ms, stepped at 2 ms: 2 / (3.6 + 2). */
    w->dlpf        = 0.357f;

    w->bias[0] =  0.0140f;          /* +0.8 deg/s: a typical MEMS offset     */
    w->bias[1] = -0.0105f;          /* -0.6 deg/s                            */
    w->bias[2] =  0.0070f;          /* +0.4 deg/s                            */

    w->t = 0.0f;
    w->vib_phase = 0u;
    w->rng = seed ? seed : 0x13579BDFu;
    w->seq = 0;
    w->locked = 0;
    derive(w);
    for (int i = 0; i < 3; i++) { w->gyro_f[i] = w->accel_f[i] = 0.0f; }
    w->accel_f[2] = -G;
    world_reset_attitude(w);
}

/* ---- one physics step ---------------------------------------------------- */
void world_step(world_t *w, const float motor_cmd[4])
{
    const float dt = WORLD_DT;

    /* 1. Motors: the command is clamped - a real ESC cannot do -10 % or 130 % -
     *    and thrust follows it with a 30 ms first-order lag. */
    for (int i = 0; i < 4; i++) {
        float target = f_clamp(motor_cmd[i], 0.0f, 1.0f) * w->t_max;
        w->thrust[i] += (target - w->thrust[i]) * w->k_motor;
    }
    const float *T = w->thrust;

    /* 2. Torques from the X geometry.  Each motor sits at (+-d, +-d) with
     *    d = arm / sqrt(2).  Left motors (M0, M3) lift the left side: roll +.
     *    Front motors (M0, M1) lift the nose: pitch +.  The CCW props (M1, M3)
     *    push the body clockwise seen from above: yaw +. */
    const float d = w->arm * 0.70710678f;
    float tx = d * (T[0] + T[3] - T[1] - T[2]);
    float ty = d * (T[0] + T[1] - T[2] - T[3]);
    float tz = w->k_drag * (T[1] + T[3] - T[0] - T[2]);

    /* 3. Euler's equations for a rigid body: I w' = tau - w x (I w) - c w */
    float p = w->w[0], q = w->w[1], r = w->w[2];
    float pd = (tx - (w->izz - w->iyy) * q * r - w->c_damp * p) * w->inv_i[0];
    float qd = (ty - (w->ixx - w->izz) * r * p - w->c_damp * q) * w->inv_i[1];
    float rd = (tz - (w->iyy - w->ixx) * p * q - w->c_damp * r) * w->inv_i[2];
    p += pd * dt;  q += qd * dt;  r += rd * dt;
    if (w->locked) { p = q = r = 0.0f; }      /* a hand holds it still */
    w->w[0] = p;  w->w[1] = q;  w->w[2] = r;

    /* 4. Kinematics: q' = 1/2 q (x) (0, w).  Then renormalise with one
     *    Newton step, n = 1.5 - 0.5|q|^2, which needs no square root. */
    float qw = w->q[0], qx = w->q[1], qy = w->q[2], qz = w->q[3];
    float h = 0.5f * dt;
    float nw = qw + h * (-qx * p - qy * q - qz * r);
    float nx = qx + h * ( qw * p + qy * r - qz * q);
    float ny = qy + h * ( qw * q - qx * r + qz * p);
    float nz = qz + h * ( qw * r + qx * q - qy * p);
    float n = 1.5f - 0.5f * (nw * nw + nx * nx + ny * ny + nz * nz);
    w->q[0] = nw * n;  w->q[1] = nx * n;  w->q[2] = ny * n;  w->q[3] = nz * n;

    /* 5. Tipped past 90 degrees: body z points up.  On a real field, a crash. */
    float zz = 1.0f - 2.0f * (w->q[1] * w->q[1] + w->q[2] * w->q[2]);
    if (zz < 0.0f) { w->crashed = 1; }

    w->t += dt;
}

/* ---- what an IMU on it would say ---------------------------------------- */
void world_sense(world_t *w, sensors_t *s)
{
    float qw = w->q[0], qx = w->q[1], qy = w->q[2], qz = w->q[3];

    /* Gravity in the body frame is the third row of the rotation matrix.
     * The accelerometer measures the force HOLDING it against gravity, so it
     * reads minus that: level, (0, 0, -g). */
    float gx = 2.0f * (qx * qz - qw * qy);
    float gy = 2.0f * (qy * qz + qw * qx);
    float gz = 1.0f - 2.0f * (qx * qx + qy * qy);

    /* Motor vibration: a shake whose frequency and size follow the motors.
     * Its phase is a 32-bit INTEGER that wraps by itself (2^32 = one turn),
     * and sin/cos come from a 64-entry table - lesson 10's fixed point,
     * used where precision does not matter: it is noise.  The first draft
     * called f_sin and f_cos here, and the M0+ model measured world_sense()
     * at 38 000 cycles (slide "The Budget"). */
    float mean = 0.25f * (w->thrust[0] + w->thrust[1] + w->thrust[2] + w->thrust[3]) * w->inv_tmax;
    w->vib_phase += (uint32_t)((50.0f + 150.0f * mean) * 8589934.592f);   /* 2^32 / 500 Hz */
    uint32_t k = w->vib_phase >> 26;                                       /* 0..63         */
    float vs = (float)sin64[k] * (1.0f / 32767.0f);
    float vc = (float)sin64[(k + 16u) & 63u] * (1.0f / 32767.0f);          /* cos = sin + 90 */
    float va = w->vib_accel * mean;

    float a[3];
    a[0] = -G * gx + va * vs        + w->accel_noise * gauss(w);
    a[1] = -G * gy + va * vc        + w->accel_noise * gauss(w);
    a[2] = -G * gz + va * 2.0f * vs * vc + w->accel_noise * gauss(w);

    /* Gyro: the truth, plus a bias that wanders slowly, plus white noise.
     * Both then pass through the chip's low-pass filter - smoother, and
     * a few milliseconds LATE, which is what limits how hard the rate loop
     * can push (slide "Too Little, Too Much"). */
    const float walk = w->gyro_walk * 0.04472f;     /* sqrt(0.002 s) */
    for (int i = 0; i < 3; i++) {
        w->bias[i] += walk * gauss(w);
        float g = w->w[i] + w->bias[i] + w->gyro_noise * gauss(w);
        w->gyro_f[i]  += w->dlpf * (g    - w->gyro_f[i]);
        w->accel_f[i] += w->dlpf * (a[i] - w->accel_f[i]);
        s->gyro[i]  = w->gyro_f[i];
        s->accel[i] = w->accel_f[i];
    }
    s->imu_ok = 1;
    s->seq = ++w->seq;
}

void world_kick(world_t *w, float dp, float dq)
{
    w->w[0] += dp;
    w->w[1] += dq;
}

void world_euler(const world_t *w, float *roll, float *pitch, float *yaw)
{
    float qw = w->q[0], qx = w->q[1], qy = w->q[2], qz = w->q[3];
    float r20 = 2.0f * (qx * qz - qw * qy);
    float r21 = 2.0f * (qy * qz + qw * qx);
    float r22 = 1.0f - 2.0f * (qx * qx + qy * qy);
    float r10 = 2.0f * (qx * qy + qw * qz);
    float r00 = 1.0f - 2.0f * (qy * qy + qz * qz);
    float s = f_clamp(-r20, -1.0f, 1.0f);
    if (roll)  { *roll  = f_atan2(r21, r22); }
    if (pitch) { *pitch = f_atan2(s, f_sqrt(1.0f - s * s)); }     /* asin */
    if (yaw)   { *yaw   = f_atan2(r10, r00); }
}
