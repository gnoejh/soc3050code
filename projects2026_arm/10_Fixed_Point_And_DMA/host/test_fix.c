/*
 * test_fix.c - host test for lesson 10's fixed-point library and benchmarks
 *
 * Compiles the lesson's own fix.c and bench.c with the PC's gcc and checks
 * them against double precision:
 *
 *   1. accuracy   every format of every arena job, error in real units
 *   2. rounding   round-to-nearest vs truncation: the bias, measured
 *   3. overflow   the cases that break naive fixed point, and that ours survive
 *   4. checksums  each workload's result with the arena's seed, so host/m0sim.py
 *                 can prove the Cortex-M0+ model computed the same answers
 *
 * Exit status 0 = every check passed; 1 = at least one failed.
 * Build and run:  host/run.bat  or  bash host/run.sh
 */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "fix.h"
#include "bench.h"
#include "fmath.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;

static void check(int ok, const char *what)
{
    printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) { failures++; }
}

static double q16d(q16_t v) { return v / 65536.0; }
static double q15d(q15_t v) { return v / 32768.0; }

static uint32_t lcg = 1u;
static int32_t rnd_range(int32_t lo, int32_t hi)        /* inclusive */
{
    lcg = lcg * 1664525u + 1013904223u;
    return lo + (int32_t)((lcg >> 8) % (uint32_t)(hi - lo + 1));
}

/* ---- 1. accuracy of every arena job, per format --------------------------- */
typedef struct { const char *job, *fmt; double max_err, step; } acc_t;
static acc_t acc[32];
static int   nacc;

static void record(const char *job, const char *fmt, double max_err, double step)
{
    acc[nacc].job = job; acc[nacc].fmt = fmt; acc[nacc].max_err = max_err; acc[nacc].step = step;
    nacc++;
}

static void accuracy(void)
{
    bench_seed(2026u);
    double e;

    /* mul */
    e = 0; for (unsigned k = 0; k < BENCH_N; k++) {
        double r = q15d(in_sa[k]) * q15d(in_sb[k]);
        e = fmax(e, fabs(q15d(q15_mul(in_sa[k], in_sb[k])) - r)); }
    record("mul", "Q15", e, 1.0 / 32768);
    e = 0; for (unsigned k = 0; k < BENCH_N; k++) {
        double r = q16d(in_qa[k]) * q16d(in_qb[k]);
        e = fmax(e, fabs(q16d(q16_mul(in_qa[k], in_qb[k])) - r)); }
    record("mul", "Q16.16", e, 1.0 / 65536);
    e = 0; for (unsigned k = 0; k < BENCH_N; k++) {
        double r = (double)in_fa[k] * in_fb[k];
        e = fmax(e, fabs((double)(in_fa[k] * in_fb[k]) - r)); }
    record("mul", "float", e, 0);

    /* div */
    e = 0; for (unsigned k = 0; k < BENCH_N; k++) {
        double r = q16d(in_qa[k]) / q16d(in_qb[k]);
        e = fmax(e, fabs(q16d(q16_div(in_qa[k], in_qb[k])) - r)); }
    record("div", "Q16.16", e, 1.0 / 65536);
    e = 0; for (unsigned k = 0; k < BENCH_N; k++) {
        double r = (double)in_fa[k] / in_fb[k];
        e = fmax(e, fabs((double)(in_fa[k] / in_fb[k]) - r)); }
    record("div", "float", e, 0);

    /* sin: a sweep of the whole circle, not just 64 points */
    double eq = 0, ef = 0, el = 0;
    for (uint32_t a = 0; a < 65536u; a++) {
        double rad = (double)((int32_t)a - 32768) * (2.0 * M_PI / 65536.0);
        double r   = sin(rad);
        uint16_t ang = (uint16_t)(a - 32768u);              /* same angle, binary */
        eq = fmax(eq, fabs(q15d(q15_sin(ang)) - r));
        ef = fmax(ef, fabs((double)f_sin((float)rad) - r));
        el = fmax(el, fabs((double)sinf((float)rad) - r));
    }
    record("sin", "Q15 table", eq, 1.0 / 32768);
    record("sin", "f_sin", ef, 0);
    record("sin", "libm sinf", el, 0);

    /* rot2d: rotate by 30 degrees */
    double c = cos(M_PI / 6), s = sin(M_PI / 6);
    double e15 = 0, e16 = 0, eff = 0;
    for (unsigned k = 0; k < BENCH_N; k++) {
        double x, y;
        x = q15d(in_sa[k]); y = q15d(in_sb[k]);
        q15_t c15 = Q15(0.8660254), s15 = Q15(0.5);
        double rx = c * x - s * y, ry = s * x + c * y;
        q15_t qx = q15_add(q15_mul(c15, in_sa[k]), (q15_t)-q15_mul(s15, in_sb[k]));
        q15_t qy = q15_add(q15_mul(s15, in_sa[k]), q15_mul(c15, in_sb[k]));
        /* rx can exceed Q15's +-1 range: |x|,|y| < 0.5 keeps it inside, by design */
        e15 = fmax(e15, fmax(fabs(q15d(qx) - rx), fabs(q15d(qy) - ry)));

        x = q16d(in_qa[k]); y = q16d(in_qb[k]);
        rx = c * x - s * y; ry = s * x + c * y;
        q16_t cq = Q16(0.8660254), sq = Q16(0.5);
        q16_t px = q16_add(q16_mul(cq, in_qa[k]), -q16_mul(sq, in_qb[k]));
        q16_t py = q16_add(q16_mul(sq, in_qa[k]),  q16_mul(cq, in_qb[k]));
        e16 = fmax(e16, fmax(fabs(q16d(px) - rx), fabs(q16d(py) - ry)));

        x = in_fa[k]; y = in_fb[k];
        rx = c * x - s * y; ry = s * x + c * y;
        float fx = 0.8660254f * in_fa[k] - 0.5f * in_fb[k];
        float fy = 0.5f * in_fa[k] + 0.8660254f * in_fb[k];
        eff = fmax(eff, fmax(fabs(fx - rx), fabs(fy - ry)));
    }
    record("rot2d", "Q15", e15, 1.0 / 32768);
    record("rot2d", "Q16.16", e16, 1.0 / 65536);
    record("rot2d", "float", eff, 0);

    /* pi: 64 steps of a PI controller, error of the output vs double */
    {
        double integ = 0; q16_t iq = 0; float iff = 0; double eq16 = 0, ef32 = 0;
        for (unsigned k = 0; k < BENCH_N; k++) {
            double err = q16d(in_qa[k]);
            integ = fmin(1.0, fmax(-1.0, integ + err * 0.01));
            double out = 0.8 * err + 2.0 * integ;
            iq = q16_add(iq, q16_mul(in_qa[k], Q16(0.01)));
            if (iq > Q16_ONE) { iq = Q16_ONE; }
            if (iq < -Q16_ONE) { iq = -Q16_ONE; }
            q16_t oq = q16_add(q16_mul(Q16(0.8), in_qa[k]), q16_mul(Q16(2.0), iq));
            eq16 = fmax(eq16, fabs(q16d(oq) - out));
        }
        integ = 0;
        for (unsigned k = 0; k < BENCH_N; k++) {
            double err = in_fa[k];
            integ = fmin(1.0, fmax(-1.0, integ + err * 0.01));
            double out = 0.8 * err + 2.0 * integ;
            iff = f_clamp(iff + in_fa[k] * 0.01f, -1.0f, 1.0f);
            float of = 0.8f * in_fa[k] + 2.0f * iff;
            ef32 = fmax(ef32, fabs(of - out));
        }
        record("pi", "Q16.16", eq16, 1.0 / 65536);
        record("pi", "float", ef32, 0);
    }

    /* fir16: against double with the ideal (unrounded) coefficients */
    {
        double h[FIR_TAPS], hs = 0;
        for (unsigned j = 0; j < FIR_TAPS; j++) { h[j] = pow(sin(M_PI * (j + 1) / 17.0), 2); hs += h[j]; }
        for (unsigned j = 0; j < FIR_TAPS; j++) { h[j] /= hs; }
        double ei = 0, e15f = 0, eflt = 0;
        for (unsigned k = 0; k < BENCH_N; k++) {
            double ri = 0, r15 = 0, rf = 0;
            for (unsigned j = 0; j < FIR_TAPS; j++) {
                unsigned m = (k + j) & (BENCH_N - 1);
                ri += h[j] * in_ia[m]; r15 += h[j] * q15d(in_sa[m]); rf += h[j] * in_fa[m];
            }
            /* int: in ADC counts out of 4096; express as a fraction of full scale */
            ei   = fmax(ei,   fabs(fir_int(in_ia, k) - ri) / 2048.0);
            e15f = fmax(e15f, fabs(q15d(fir_q15(in_sa, k)) - r15));
            eflt = fmax(eflt, fabs((double)fir_float(in_fa, k) - rf) / 2.0);   /* fa spans +-2 */
        }
        record("fir16", "int32", ei, 1.0 / 2048);
        record("fir16", "Q15", e15f, 1.0 / 32768);
        record("fir16", "float", eflt, 0);
    }

    printf("\n1. ACCURACY vs double (seed 2026; sin swept over all 65536 angles)\n");
    printf("   job    format      max error      in LSBs of the format\n");
    for (int i = 0; i < nacc; i++) {
        printf("   %-6s %-10s  %.3e", acc[i].job, acc[i].fmt, acc[i].max_err);
        if (acc[i].step > 0) { printf("    %6.2f\n", acc[i].max_err / acc[i].step); }
        else                 { printf("    (float: ~24-bit mantissa)\n"); }
    }
    printf("   (fir16 int32 error is a fraction of ADC full scale; float fir16 per unit of input range)\n\n");

    for (int i = 0; i < nacc; i++) {
        if (strcmp(acc[i].fmt, "Q15") == 0 && strcmp(acc[i].job, "mul") == 0)
            check(acc[i].max_err <= 0.5 / 32768 + 1e-12, "q15_mul within half an LSB (rounded)");
        if (strcmp(acc[i].fmt, "Q16.16") == 0 && strcmp(acc[i].job, "mul") == 0)
            check(acc[i].max_err <= 0.5 / 65536 + 1e-12, "q16_mul within half an LSB (rounded)");
        if (strcmp(acc[i].fmt, "Q16.16") == 0 && strcmp(acc[i].job, "div") == 0)
            check(acc[i].max_err <= 1.0 / 65536, "q16_div within one LSB (truncated)");
        if (strcmp(acc[i].fmt, "Q15 table") == 0)
            check(acc[i].max_err < 1.5e-4, "q15_sin max error < 1.5e-4 over the full circle");
        if (strcmp(acc[i].fmt, "f_sin") == 0)
            check(acc[i].max_err < 1e-5, "f_sin max error < 1e-5 (fmath.h's claim)");
        if (strcmp(acc[i].job, "fir16") == 0 && strcmp(acc[i].fmt, "Q15") == 0)
            check(acc[i].max_err < 4.0 / 32768, "fir16 Q15 within 4 LSB of the ideal filter");
        if (strcmp(acc[i].job, "rot2d") == 0 && strcmp(acc[i].fmt, "Q15") == 0)
            check(acc[i].max_err < 3.0 / 32768, "rot2d Q15 within 3 LSB");
    }
}

/* ---- 2. rounding: the bias of truncation ---------------------------------- */
static void rounding(void)
{
    printf("\n2. ROUNDING: 1 000 000 random Q15 products, mean error\n");
    double sum_r = 0, sum_t = 0;
    for (int i = 0; i < 1000000; i++) {
        q15_t a = (q15_t)rnd_range(-32768, 32767), b = (q15_t)rnd_range(-32768, 32767);
        if (a == -32768 && b == -32768) { continue; }
        double r = q15d(a) * q15d(b);
        q15_t rounded   = q15_mul(a, b);
        q15_t truncated = (q15_t)(((int32_t)a * b) >> 15);
        sum_r += q15d(rounded) - r;
        sum_t += q15d(truncated) - r;
    }
    double mr = sum_r / 1e6 * 32768, mt = sum_t / 1e6 * 32768;
    printf("   round to nearest  mean error %+.4f LSB\n", mr);
    printf("   truncate (>> 15)  mean error %+.4f LSB   <- always low: a drift\n", mt);
    check(fabs(mr) < 0.01, "rounded products have no bias");
    check(mt < -0.45 && mt > -0.55, "truncated products are biased by -1/2 LSB");
}

/* ---- 3. overflow ----------------------------------------------------------- */
static void overflow(void)
{
    printf("\n3. OVERFLOW: where naive fixed point breaks\n");
    q15_t m = q15_mul(Q15_MIN, Q15_MIN);
    int32_t naive = ((int32_t)Q15_MIN * Q15_MIN) >> 15;          /* 32768: does not fit */
    printf("   Q15  -1 x -1:      naive (int16)%ld = %d    q15_mul = %d (+0.99997)\n",
           (long)naive, (int)(int16_t)(uint16_t)naive, (int)m);
    check(m == Q15_MAX, "q15_mul(-1, -1) saturates to +32767");

    q15_t a = Q15(0.75), b = Q15(0.5);
    q15_t w = q15_add_wrap(a, b), s = q15_add(a, b);
    printf("   Q15  0.75 + 0.5:   wrapping %+.5f   saturating %+.5f\n", q15d(w), q15d(s));
    check(w < 0, "wrapping add turns +1.25 into a NEGATIVE number");
    check(s == Q15_MAX, "saturating add clamps to +0.99997");

    q16_t big = q16_mul(Q16(300.0), Q16(300.0));                 /* 90000 > 32767 */
    printf("   Q16  300 x 300:    q16_mul = %.3f (true 90000; the format stops at 32768)\n", q16d(big));
    check(big == Q16_MAX, "q16_mul saturates on overflow");

    q16_t d0 = q16_div(Q16(1.0), 0);
    check(d0 == Q16_MAX, "q16_div by zero returns the largest value, not a crash");

    /* the FIR's guard: a gain-4 filter overflows a 32-bit Q30 accumulator */
    int32_t x = Q15_MAX;
    uint32_t acc32 = 0; int64_t acc64 = 0;
    for (unsigned j = 0; j < FIR_TAPS; j++) {
        int32_t h4 = fir_h_q15[j] * 4;                           /* taps sum to ~4.0 */
        acc32 += (uint32_t)(h4 * x);                             /* wraps like the M0+ */
        acc64 += (int64_t)h4 * x;
    }
    printf("   FIR  gain 4, full-scale input: 32-bit sum %+.4f, 64-bit sum %+.4f\n",
           (int32_t)acc32 / 1073741824.0, acc64 / 1073741824.0);
    check((int32_t)acc32 < 0 && acc64 > 0, "gain > 2 overflows the Q30 accumulator (sign flips)");
}

/* ---- 4. checksums ----------------------------------------------------------- */
static void checksums(void)
{
    bench_seed(2026u);
    printf("\n4. CHECKSUMS (seed 2026, n = 64) for host/m0sim.py --expect\n");
    for (uint32_t i = 0; i < bench_count; i++) {
        printf("CHK %s|%s|%08lX\n", bench_table[i].work, bench_table[i].fmt,
               (unsigned long)bench_table[i].fn(BENCH_N));
    }
}

int main(void)
{
    printf("== lesson 10 host test: fix.c + bench.c vs double ==\n");
    accuracy();
    rounding();
    overflow();
    checksums();
    printf("\n%s: %d check(s) failed\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
