/*
 * fix.h - fixed-point arithmetic for a chip with no FPU   (SOC3050 lesson 10)
 *
 * Pure C: no stm32 header, no hardware.  host/test_fix.c compiles this file
 * with the PC's gcc and checks every function against double precision.
 *
 * A fixed-point number is an ordinary integer with an agreed binary point.
 * "Qm.n" means m integer bits (sign included) and n fraction bits:
 *
 *   Q16.16  int32_t   value = raw / 65536    range -32768 .. +32767.99998
 *                                            step  1.5e-5
 *   Q15     int16_t   value = raw / 32768    range -1 .. +0.99997
 *   (Q1.15)                                  step  3.1e-5
 *
 * Adding two numbers in the same Q format is plain integer addition.
 * Multiplying is where the work is: the product of two Qn numbers has 2n
 * fraction bits, so it must be shifted back down by n - and it needs twice
 * the bits while it is being formed:
 *
 *   Q15   x Q15   = Q30   fits in 32 bits   -> one MULS (1 cycle on the M0+)
 *   Q16.16 x Q16.16 = Q32.32  needs 64 bits -> a call to __aeabi_lmul
 *
 * That one fact decides most of this lesson's leaderboard.
 *
 * Three hazards, and how each function below handles them:
 *   overflow     the true answer does not fit: we SATURATE (clamp to the
 *                largest value) instead of wrapping round to a large
 *                negative number.  A wrapped control output flips a motor.
 *   rounding     shifting right truncates toward minus infinity; adding half
 *                an LSB first makes it round to nearest.  Truncation biases
 *                every result down by half a step - a slow drift in a filter.
 *   -1 x -1      in Q15, -1 is representable and +1 is not, so the one
 *                product -1 * -1 overflows.  q15_mul() catches it.
 */
#ifndef FIX_H
#define FIX_H

#include <stdint.h>

typedef int32_t q16_t;              /* Q16.16 */
typedef int16_t q15_t;              /* Q1.15  */

#define Q16_ONE     65536
#define Q16_MAX     INT32_MAX
#define Q16_MIN     INT32_MIN
#define Q15_ONE_ISH 32767           /* +1 itself does not exist in Q15 */
#define Q15_MAX     INT16_MAX
#define Q15_MIN     INT16_MIN

/* Build constants at compile time from a decimal: Q16(0.5) == 32768.
 * The compiler folds the float maths away; no float code reaches the chip. */
#define Q16(x)  ((q16_t)((x) * 65536.0 + ((x) >= 0 ? 0.5 : -0.5)))
#define Q15(x)  ((q15_t)((x) * 32768.0 + ((x) >= 0 ? 0.5 : -0.5)))

/* ---- saturation helpers -------------------------------------------------- */
static inline q16_t q16_sat64(int64_t v)
{
    return v > Q16_MAX ? Q16_MAX : (v < Q16_MIN ? Q16_MIN : (q16_t)v);
}

static inline q15_t q15_sat32(int32_t v)
{
    return v > Q15_MAX ? Q15_MAX : (v < Q15_MIN ? Q15_MIN : (q15_t)v);
}

/* ---- Q16.16 --------------------------------------------------------------- */
static inline q16_t q16_add(q16_t a, q16_t b)        /* saturating */
{
    return q16_sat64((int64_t)a + b);
}

/* 32 x 32 -> 64-bit product, round to nearest, shift back to Q16.16,
 * saturate.  On the M0+ the (int64_t) multiply is a call to __aeabi_lmul:
 * the M0+'s MULS keeps only the low 32 bits of a product. */
static inline q16_t q16_mul(q16_t a, q16_t b)
{
    int64_t p = (int64_t)a * b;
    p += 1 << 15;                                    /* + half an LSB: round */
    return q16_sat64(p >> 16);
}

/* (a << 16) / b, in 64 bits.  Division is the slowest thing in this file:
 * a 64-by-64-bit software divide (__aeabi_ldivmod).  Divide by a constant?
 * Multiply by its reciprocal instead - lab Part 5. */
static inline q16_t q16_div(q16_t a, q16_t b)
{
    if (b == 0) { return a >= 0 ? Q16_MAX : Q16_MIN; }
    return q16_sat64(((int64_t)a * 65536) / b);
}

static inline int32_t q16_to_int(q16_t a)            /* round to nearest */
{
    return (a + (1 << 15)) >> 16;
}

/* ---- Q15 ------------------------------------------------------------------ */
static inline q15_t q15_add(q15_t a, q15_t b)        /* saturating */
{
    return q15_sat32((int32_t)a + b);
}

/* What a beginner writes: no saturation.  Kept so the lab can break it. */
static inline q15_t q15_add_wrap(q15_t a, q15_t b)
{
    return (q15_t)(uint16_t)((uint16_t)a + (uint16_t)b);
}

/* Q15 x Q15 = Q30 in 32 bits: ONE multiply instruction.  Round, shift back
 * by 15, and saturate the single overflowing case, -1 x -1 = +1. */
static inline q15_t q15_mul(q15_t a, q15_t b)
{
    int32_t p = (int32_t)a * b;
    p += 1 << 14;
    return q15_sat32(p >> 15);
}

/* ---- in fix.c ------------------------------------------------------------- */

/* sin of a "binary angle": a full turn is 65536, so the angle wraps for free
 * when the uint16_t overflows.  Quarter-wave table of 65 entries plus linear
 * interpolation - 130 bytes of flash.  Max error measured on the host: see
 * host/test_fix.c and the README. */
q15_t q15_sin(uint16_t angle);
static inline q15_t q15_cos(uint16_t angle) { return q15_sin((uint16_t)(angle + 16384u)); }

#endif
