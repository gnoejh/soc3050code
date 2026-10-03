/*
 * bench.c - the benchmark arena: one job, several number formats   (lesson 10)
 *
 * Pure C.  Read bench.h first.
 *
 * Each workload below is written the obvious way for its number format - the
 * way you would write it in a real program - and nothing more.  The race is
 * between FORMATS, not between clever and lazy code; making one faster is lab
 * Part 8.
 *
 * Why every function is noinline: Main.c and host/m0sim.py both call these
 * through a function pointer and time the call.  Inlined into a caller, a
 * workload would have no address to call and no boundary to time.
 */
#include <math.h>              /* sinf - the C library's, for the comparison */
#include "fmath.h"             /* f_sin - _lib/fmath.h, the course's own     */
#include "bench.h"

#define NOINLINE __attribute__((noinline))
#define K(i)     ((i) & (BENCH_N - 1u))

float    in_fa[BENCH_N], in_fb[BENCH_N], in_fang[BENCH_N];
int32_t  in_ia[BENCH_N], in_ib[BENCH_N];
q16_t    in_qa[BENCH_N], in_qb[BENCH_N];
q15_t    in_sa[BENCH_N], in_sb[BENCH_N];
uint16_t in_ang[BENCH_N];

/* Rotation and controller constants: set by bench_seed(), so they are
 * variables the compiler cannot fold away. */
static float  rot_cf, rot_sf, pid_kp, pid_ki, pid_dt;
static q16_t  rot_cq, rot_sq, pid_kpq, pid_kiq, pid_dtq;
static q15_t  rot_c15, rot_s15;

/* The bit pattern of a float, so it can be folded into a checksum without
 * any float arithmetic (an XOR of the bits costs one instruction). */
static inline uint32_t fbits(float f)
{
    union { float f; uint32_t u; } v = { f };
    return v.u;
}

static uint32_t xs;                               /* xorshift32 state */
static uint32_t rnd(void)
{
    xs ^= xs << 13; xs ^= xs >> 17; xs ^= xs << 5;
    return xs;
}

void bench_seed(uint32_t seed)
{
    xs = seed ? seed : 0x12345678u;
    for (uint32_t k = 0; k < BENCH_N; k++) {
        /* a and b in [-2, +2], b kept away from 0 so division is defined */
        int32_t ra = (int32_t)(rnd() % 4001u) - 2000;
        int32_t rb = (int32_t)(rnd() % 4001u) - 2000;
        if (rb > -20 && rb < 20) { rb = 500; }
        in_fa[k] = (float)ra / 1000.0f;
        in_fb[k] = (float)rb / 1000.0f;
        in_qa[k] = (q16_t)(ra * 65536 / 1000);        /* same values, Q16.16 */
        in_qb[k] = (q16_t)(rb * 65536 / 1000);
        /* Q15 holds -1..+1, so a and b are scaled by 1/4 into -0.5..+0.5:
         * then even a rotated point (up to 1.41x longer) cannot overflow. */
        in_sa[k] = (q15_t)(ra * 8191 / 1000);
        in_sb[k] = (q15_t)(rb * 8191 / 1000);

        /* integers like a 12-bit ADC reading, centred: -2048..2047 */
        in_ia[k] = (int32_t)(rnd() & 4095u) - 2048;
        in_ib[k] = (int32_t)(rnd() & 4095u) - 2048;
        if (in_ib[k] == 0) { in_ib[k] = 1; }

        /* angles: a binary angle for Q15, the same angle in radians for float */
        in_ang[k]  = (uint16_t)rnd();
        in_fang[k] = (float)((int32_t)in_ang[k] - 32768) * (F_2PI / 65536.0f);
    }
    /* rotate by about 30 degrees; a PI controller with kp 0.8, ki 2, 100 Hz */
    rot_cf = 0.8660254f;  rot_sf = 0.5f;
    rot_cq = Q16(0.8660254); rot_sq = Q16(0.5);
    rot_c15 = Q15(0.8660254); rot_s15 = Q15(0.5);
    pid_kp = 0.8f; pid_ki = 2.0f; pid_dt = 0.01f;
    pid_kpq = Q16(0.8); pid_kiq = Q16(2.0); pid_dtq = Q16(0.01);
}

/* ============================================================================
 *  The 16-tap FIR: a Hann-windowed low-pass, coefficients summing to 1.
 *  Generated from w[j] = sin^2(pi (j+1) / 17), normalised.  Because every
 *  coefficient is positive and they sum to 1, |output| <= max |input|: the
 *  Q15 accumulator can never overflow.  Change that and it can (lab Part 4).
 * ============================================================================ */
const float fir_h_f[FIR_TAPS] = {
    0.0039722f, 0.0153524f, 0.0326036f, 0.0533960f, 0.0749214f, 0.0942726f,
    0.1088363f, 0.1166455f, 0.1166455f, 0.1088363f, 0.0942726f, 0.0749214f,
    0.0533960f, 0.0326036f, 0.0153524f, 0.0039722f,
};
const q15_t fir_h_q15[FIR_TAPS] = {             /* round(h * 32768): sum 32766 */
    130, 503, 1068, 1750, 2455, 3089, 3566, 3822,
    3822, 3566, 3089, 2455, 1750, 1068, 503, 130,
};
const int32_t fir_h_int[FIR_TAPS] = {           /* round(h * 256): sum 256 */
    1, 4, 8, 14, 19, 24, 28, 30, 30, 28, 24, 19, 14, 8, 4, 1,
};

float fir_float(const float *x, uint32_t k)
{
    float acc = 0.0f;
    for (uint32_t j = 0; j < FIR_TAPS; j++) { acc += fir_h_f[j] * x[K(k + j)]; }
    return acc;                                   /* 16 fmul + 16 fadd calls */
}

q15_t fir_q15(const q15_t *x, uint32_t k)
{
    int32_t acc = 0;                              /* Q30: 16 products, no overflow */
    for (uint32_t j = 0; j < FIR_TAPS; j++) { acc += (int32_t)fir_h_q15[j] * x[K(k + j)]; }
    return q15_sat32((acc + (1 << 14)) >> 15);   /* round once, at the end */
}

int32_t fir_int(const int32_t *x, uint32_t k)
{
    int32_t acc = 0;
    for (uint32_t j = 0; j < FIR_TAPS; j++) { acc += fir_h_int[j] * x[K(k + j)]; }
    return (acc + 128) >> 8;                      /* coefficients were x256 */
}

/* ============================================================================
 *  The workloads.  n iterations each; k walks the 64-entry arrays.
 * ============================================================================ */

/* The baseline: the loop, one load, one add.  Every other row's cost INCLUDES
 * this, so Main.c subtracts it to get the cost of the operation alone. */
static NOINLINE uint32_t b_loop(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)in_ia[K(i)]; }
    return acc;
}

/* ---- add ----------------------------------------------------------------- */
static NOINLINE uint32_t b_add_int(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)(in_ia[K(i)] + in_ib[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_add_q16(uint32_t n)       /* saturating: 64-bit add */
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)q16_add(in_qa[K(i)], in_qb[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_add_f(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc ^= fbits(in_fa[K(i)] + in_fb[K(i)]); }
    return acc;
}

/* ---- mul ----------------------------------------------------------------- */
static NOINLINE uint32_t b_mul_int(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)(in_ia[K(i)] * in_ib[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_mul_q15(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)q15_mul(in_sa[K(i)], in_sb[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_mul_q16(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)q16_mul(in_qa[K(i)], in_qb[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_mul_f(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc ^= fbits(in_fa[K(i)] * in_fb[K(i)]); }
    return acc;
}

/* ---- div: the M0+ has no divide instruction at all ------------------------ */
static NOINLINE uint32_t b_div_int(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)(in_ia[K(i)] / in_ib[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_div_q16(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)q16_div(in_qa[K(i)], in_qb[K(i)]); }
    return acc;
}
static NOINLINE uint32_t b_div_f(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc ^= fbits(in_fa[K(i)] / in_fb[K(i)]); }
    return acc;
}

/* ---- sin ----------------------------------------------------------------- */
static NOINLINE uint32_t b_sin_libm(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc ^= fbits(sinf(in_fang[K(i)])); }
    return acc;
}
static NOINLINE uint32_t b_sin_fmath(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc ^= fbits(f_sin(in_fang[K(i)])); }
    return acc;
}
static NOINLINE uint32_t b_sin_q15(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)q15_sin(in_ang[K(i)]); }
    return acc;
}

/* ---- rotate a 2D point: 4 multiplies, 2 adds ------------------------------ */
static NOINLINE uint32_t b_rot_f(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) {
        float x = in_fa[K(i)], y = in_fb[K(i)];
        acc ^= fbits(rot_cf * x - rot_sf * y) ^ fbits(rot_sf * x + rot_cf * y);
    }
    return acc;
}
static NOINLINE uint32_t b_rot_q16(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) {
        q16_t x = in_qa[K(i)], y = in_qb[K(i)];
        acc += (uint32_t)q16_add(q16_mul(rot_cq, x), -q16_mul(rot_sq, y));
        acc += (uint32_t)q16_add(q16_mul(rot_sq, x),  q16_mul(rot_cq, y));
    }
    return acc;
}
static NOINLINE uint32_t b_rot_q15(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) {
        q15_t x = in_sa[K(i)], y = in_sb[K(i)];
        acc += (uint32_t)q15_add(q15_mul(rot_c15, x), (q15_t)-q15_mul(rot_s15, y));
        acc += (uint32_t)q15_add(q15_mul(rot_s15, x),  q15_mul(rot_c15, y));
    }
    return acc;
}

/* ---- one step of a PI controller, with anti-windup clamp ------------------ */
static NOINLINE uint32_t b_pid_f(uint32_t n)
{
    uint32_t acc = 0;
    float integ = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float err = in_fa[K(i)];
        integ = f_clamp(integ + err * pid_dt, -1.0f, 1.0f);
        acc ^= fbits(pid_kp * err + pid_ki * integ);
    }
    return acc;
}
static NOINLINE uint32_t b_pid_q16(uint32_t n)
{
    uint32_t acc = 0;
    q16_t integ = 0;
    for (uint32_t i = 0; i < n; i++) {
        q16_t err = in_qa[K(i)];
        integ = q16_add(integ, q16_mul(err, pid_dtq));
        if (integ >  Q16_ONE) { integ =  Q16_ONE; }
        if (integ < -Q16_ONE) { integ = -Q16_ONE; }
        acc += (uint32_t)q16_add(q16_mul(pid_kpq, err), q16_mul(pid_kiq, integ));
    }
    return acc;
}

/* ---- 16-tap FIR: one output per iteration -------------------------------- */
static NOINLINE uint32_t b_fir_f(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc ^= fbits(fir_float(in_fa, i)); }
    return acc;
}
static NOINLINE uint32_t b_fir_q15(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)fir_q15(in_sa, i); }
    return acc;
}
static NOINLINE uint32_t b_fir_int(uint32_t n)
{
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; i++) { acc += (uint32_t)fir_int(in_ia, i); }
    return acc;
}

const bench_t bench_table[] = {
    { "loop",  "baseline", b_loop      },
    { "add",   "int32",    b_add_int   },
    { "add",   "Q16.16",   b_add_q16   },
    { "add",   "float",    b_add_f     },
    { "mul",   "int32",    b_mul_int   },
    { "mul",   "Q15",      b_mul_q15   },
    { "mul",   "Q16.16",   b_mul_q16   },
    { "mul",   "float",    b_mul_f     },
    { "div",   "int32",    b_div_int   },
    { "div",   "Q16.16",   b_div_q16   },
    { "div",   "float",    b_div_f     },
    { "sin",   "Q15 table",b_sin_q15   },
    { "sin",   "f_sin",    b_sin_fmath },
    { "sin",   "libm sinf",b_sin_libm  },
    { "rot2d", "Q15",      b_rot_q15   },
    { "rot2d", "Q16.16",   b_rot_q16   },
    { "rot2d", "float",    b_rot_f     },
    { "pi",    "Q16.16",   b_pid_q16   },
    { "pi",    "float",    b_pid_f     },
    { "fir16", "int32",    b_fir_int   },
    { "fir16", "Q15",      b_fir_q15   },
    { "fir16", "float",    b_fir_f     },
};
const uint32_t bench_count = sizeof bench_table / sizeof bench_table[0];
