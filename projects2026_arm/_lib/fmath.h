/*
 * fmath.h - small, fast float maths for a chip with no FPU   (header only)
 *
 * The Cortex-M0+ has no floating-point unit, so every float + and * is a
 * library call of 50-100 cycles (lesson 10 measures them).  The C library's
 * sinf() and cosf() are exact to the last bit and range-reduce any argument
 * up to 1e38, which costs about 4 KB of flash - an eighth of this chip.  A
 * control loop needs neither: its angles are small and 1e-4 is plenty.
 *
 * These are good to about 1e-5 over the ranges stated (measured on a PC
 * against the C library), cost a dozen multiplies, and take a few hundred
 * bytes.  Lesson 10 measures both versions; lessons 13-18 use these.  Swapping
 * sinf/cosf for f_sin/f_cos in one test build saved 3936 bytes of flash.
 *
 *   f_wrap(a)       any angle -> [-pi, pi]   (|a| < 1e6)
 *   f_sin, f_cos    any angle                max error 4e-6 (after wrap)
 *   f_atan2(y, x)   full circle              max error 2e-6 rad
 *   f_sqrt(x)       x >= 0                   relative error 1e-7 (3 Newton steps)
 *   f_clamp, f_abs
 */
#ifndef FMATH_H
#define FMATH_H

#include <stdint.h>

#define F_PI     3.14159265f
#define F_2PI    6.28318531f
#define F_PI_2   1.57079633f

static inline float f_abs(float x) { return x < 0.0f ? -x : x; }

static inline float f_clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float f_wrap(float a)
{
    if (a > F_PI || a < -F_PI) {
        int32_t k = (int32_t)(a / F_2PI + (a >= 0.0f ? 0.5f : -0.5f));
        a -= (float)k * F_2PI;
    }
    return a;
}

/* sin on [-pi/2, pi/2]: the Taylor series to x^9, in Horner form.  The first
 * term left out is x^11/11!, at most 3.6e-6 at the ends of the interval. */
static inline float f_sin_core(float x)
{
    float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f
             + x2 * (-1.0f / 5040.0f + x2 * (1.0f / 362880.0f)))));
}

static inline float f_sin(float a)
{
    a = f_wrap(a);
    if (a > F_PI_2)  { a = F_PI - a; }        /* sin(pi - a) = sin(a) */
    if (a < -F_PI_2) { a = -F_PI - a; }
    return f_sin_core(a);
}

static inline float f_cos(float a) { return f_sin(a + F_PI_2); }

/* atan on [-1, 1], then octant folding. */
static inline float f_atan_core(float z)
{
    float z2 = z * z;
    return z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f
             + z2 * (-0.11643287f + z2 * (0.05265332f + z2 * (-0.01172120f))))));
}

static inline float f_atan2(float y, float x)
{
    if (x == 0.0f && y == 0.0f) { return 0.0f; }
    float ax = f_abs(x), ay = f_abs(y), a;
    if (ax >= ay) { a = f_atan_core(ay / ax); }
    else          { a = F_PI_2 - f_atan_core(ax / ay); }
    if (x < 0.0f) { a = F_PI - a; }
    return y < 0.0f ? -a : a;
}

static inline float f_sqrt(float x)
{
    if (x <= 0.0f) { return 0.0f; }
    union { float f; uint32_t u; } v = { x };
    v.u = 0x1FBD1DF5u + (v.u >> 1);           /* exponent halved: a good first guess */
    float r = v.f;
    r = 0.5f * (r + x / r);                   /* Newton: each step doubles the digits */
    r = 0.5f * (r + x / r);
    r = 0.5f * (r + x / r);
    return r;
}

#endif
