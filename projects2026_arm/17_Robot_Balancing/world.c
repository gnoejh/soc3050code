/*
 * world.c - the balancing robot's physics and sensors
 *
 * ---------------------------------------------------------------------------
 *  THE MODEL (Slide 4 draws it)
 * ---------------------------------------------------------------------------
 * Two coordinates: x, how far the axle has rolled, and theta, the body's tilt
 * from vertical (positive = leaning forward, toward +x).  The wheels roll
 * without slipping, so the wheel's own angle is x / r.
 *
 * Lagrange's equations for a body (mass M, CoM at height L, inertia I) on an
 * axle with wheels (mass mw, inertia Iw), driven by a torque tau that acts
 * between the body and the wheels:
 *
 *   (M + mw + Iw/r^2) x''  +  M L cos(th) th''  =  tau/r + M L sin(th) th'^2 - b x' + F
 *   M L cos(th)       x''  +  (I + M L^2)  th'' =  M g L sin(th) - tau + F L cos(th)
 *
 * Read the second line: gravity tips the body over (M g L sin th), the motor's
 * reaction torque pushes it back (-tau), and ACCELERATING THE WHEELS FORWARD
 * (x'' > 0) swings the body back upright.  That coupling is the only handle a
 * balancing robot has: to stop falling forward it must drive forward.
 *
 * F is a push, applied horizontally at the body's centre of mass.
 *
 * The motor: a DC motor is a voltage source driving a resistor against its own
 * back-EMF.  The current, and so the torque, is
 *
 *   tau = n * k * (u - k * w_rel) / R  -  n * b_m * w_rel,   w_rel = x'/r - th'
 *
 * w_rel is the wheel's speed RELATIVE TO THE BODY - the motor is bolted to the
 * body.  Two consequences a controller must live with: the faster the wheels
 * already turn, the less torque a volt buys (back-EMF); and u can never
 * exceed the battery, VBAT.
 *
 * Every step solves those two equations for x'' and th'' (a 2x2 linear
 * system) and integrates with SEMI-IMPLICIT EULER: speeds first, then
 * positions from the NEW speeds.  Plain Euler would add energy every step and
 * an undriven pendulum would swing higher and higher; this ordering does not.
 * At 1 kHz against a falling time of ~0.1 s it is accurate to well under a
 * percent, and it costs one evaluation of the equations instead of RK2's two
 * - on a chip with no FPU, that is half the world's CPU time.
 */
#include "world.h"
#include "params.h"
#include "fmath.h"

/* ---- noise --------------------------------------------------------------
 * xorshift32 (Marsaglia, 2003): three shifts and three XORs, a period of
 * 2^32 - 1.  Seeded, so a run can be repeated exactly - the host test depends
 * on that. */
static uint32_t rnd(world_t *w)
{
    uint32_t v = w->rng;
    v ^= v << 13;  v ^= v >> 17;  v ^= v << 5;
    return w->rng = v;
}

/* Roughly Gaussian, mean 0, standard deviation 1: the sum of four uniform
 * numbers (central limit theorem, cheaply).  Each uniform on [-1, 1] has
 * variance 1/3, four of them 4/3, so scale by sqrt(3/4). */
static float gauss(world_t *w)
{
    float s = 0.0f;
    for (int i = 0; i < 4; i++) {
        s += (float)(int32_t)(rnd(w) >> 8) * (1.0f / 8388608.0f) - 1.0f;
    }
    return s * 0.8660254f;
}

void world_set_payload(world_t *w, float kg)
{
    /* A mass on top moves the centre of mass UP and adds inertia: the
     * parallel-axis theorem for both pieces about their joint CoM. */
    if (kg < 0.0f) { kg = 0.0f; }
    float m = BOT_MB + kg;
    float l = (BOT_MB * BOT_L + kg * PAY_H) / m;
    float d1 = BOT_L - l, d2 = PAY_H - l;
    w->payload = kg;
    w->m = m;
    w->l = l;
    w->inertia = BOT_IB + BOT_MB * d1 * d1 + kg * d2 * d2;
}

void world_init(world_t *w, uint32_t seed, float tilt0)
{
    float pay = w->payload, vb = w->vbat, nz = w->noise;   /* survive a reset */
    *w = (world_t){0};
    w->rng   = seed ? seed : 1u;
    w->th    = tilt0;
    w->vbat  = vb > 0.0f ? vb : VBAT;
    w->noise = nz;
    w->mount_c = f_cos(IMU_TILT);
    w->mount_s = f_sin(IMU_TILT);
    world_set_payload(w, pay);
}

void world_push(world_t *w, float impulse_Ns)
{
    w->push_force = impulse_Ns / PUSH_TIME;
    w->push_left  = PUSH_TIME;
}

void world_step(world_t *w, float volts)
{
    w->t += WORLD_DT;
    if (w->fallen) {               /* lying on the floor: nothing moves until A */
        w->u = w->tau = 0.0f;
        return;
    }

    /* the H-bridge cannot give more than the battery has */
    float u = volts;
    if (u >  w->vbat) { u =  w->vbat; w->saturated++; }
    if (u < -w->vbat) { u = -w->vbat; w->saturated++; }
    w->u = u;

    float s = f_sin(w->th), c = f_cos(w->th);
    float w_rel = w->xd * (1.0f / BOT_R) - w->thd;
    float tau = MOT_N * (MOT_K * (u - MOT_K * w_rel) / MOT_RES - MOT_B * w_rel);
    w->tau = tau;

    float F = 0.0f;
    if (w->push_left > 0.0f) { F = w->push_force; w->push_left -= WORLD_DT; }

    float m = w->m, l = w->l;
    float a11 = m + BOT_MW + BOT_IW / (BOT_R * BOT_R);
    float a12 = m * l * c;
    float a22 = w->inertia + m * l * l;
    float b1  = tau * (1.0f / BOT_R) + m * l * s * w->thd * w->thd - BOT_BROLL * w->xd + F;
    float b2  = m * GRAV * l * s - tau + F * l * c;
    float det = a11 * a22 - a12 * a12;          /* > 0 always: a mass matrix */

    w->xdd  = (a22 * b1 - a12 * b2) / det;
    w->thdd = (a11 * b2 - a12 * b1) / det;

    /* semi-implicit Euler: speeds, then positions from the new speeds */
    w->xd  += w->xdd  * WORLD_DT;
    w->thd += w->thdd * WORLD_DT;
    w->x   += w->xd   * WORLD_DT;
    w->th  += w->thd  * WORLD_DT;

    if (w->th > FALL_ANGLE || w->th < -FALL_ANGLE) {
        w->fallen = 1;
        w->th  = w->th > 0.0f ? 1.40f : -1.40f;   /* on its face (80 degrees: a bumper) */
        w->xd  = w->thd = w->xdd = w->thdd = 0.0f;
        w->push_left = 0.0f;
    }
}

void world_sense(world_t *w, float v_cmd, sensors_t *out)
{
    float s = f_sin(w->th), c = f_cos(w->th);
    float nz = w->noise;

    /* GYRO: the true rate, plus a constant offset, plus noise. */
    out->gyro = w->thd + GYRO_BIAS + nz * GYRO_NOISE * gauss(w);

    /* ACCELEROMETER: it measures SPECIFIC FORCE - its own acceleration minus
     * gravity - in the body's axes.  Standing still it reads gravity, which
     * tells it the tilt.  Accelerating, it cannot tell the two apart: driving
     * forward at a looks exactly like leaning back by about a/g.  That is why
     * it cannot be used alone (Slide 8). */
    float ax = w->xdd + IMU_H * (c * w->thdd - s * w->thd * w->thd);   /* sensor point, world frame */
    float ay = -IMU_H * (s * w->thdd + c * w->thd * w->thd);
    float fx = ax, fy = ay + GRAV;                                     /* minus gravity (0, -g)    */
    float af = fx * c - fy * s;                                        /* into body axes           */
    float au = fx * s + fy * c;
    /* mounted IMU_TILT off true: rotate the reading by that much */
    out->acc_f = af * w->mount_c - au * w->mount_s + nz * ACC_NOISE * gauss(w);
    out->acc_u = au * w->mount_c + af * w->mount_s + nz * ACC_NOISE * gauss(w);

    /* ENCODER: on the motor, so it counts the wheel's turn RELATIVE TO THE
     * BODY.  A whole count, rounded down - quantisation is part of the sensor. */
    float rev = (w->x * (1.0f / BOT_R) - w->th) * (ENC_CPR / F_2PI);
    int32_t n = (int32_t)rev;
    if ((float)n > rev) { n--; }                 /* floor, for negative angles too */
    out->enc = n;

    out->v_cmd = v_cmd;
}
