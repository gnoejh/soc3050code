/*
 * bench.h - the benchmark arena's workloads   (SOC3050 lesson 10)
 *
 * Pure C - no stm32 header - so the same file runs in three places:
 *   the firmware      Main.c times each workload with SysTick
 *   host/test_fix.c   the PC checks each format's ACCURACY against double
 *   host/m0sim.py     an instruction-level Cortex-M0+ model runs the very
 *                     functions in Main.elf and counts their CYCLES
 *
 * Every workload has the same shape: uint32_t fn(uint32_t n) runs n
 * iterations over the 64-entry input arrays and returns a checksum of every
 * result.  The checksum is what stops the compiler deleting the work (an
 * unused result is dead code), and it lets the host compare the M0+ model's
 * answers with the PC's bit for bit.
 */
#ifndef BENCH_H
#define BENCH_H

#include <stdint.h>
#include "fix.h"

#define BENCH_N     64u        /* entries in each input array; a power of 2 */
#define FIR_TAPS    16u

typedef uint32_t (*bench_fn)(uint32_t n);

typedef struct {
    const char *work;          /* "mul", "fir16" ...                         */
    const char *fmt;           /* "float", "Q16.16" ...                      */
    bench_fn    fn;
} bench_t;

extern const bench_t  bench_table[];
extern const uint32_t bench_count;

/* Fill the input arrays from a pseudo-random seed.  Main.c seeds from the
 * joystick, so the numbers are not known at compile time. */
void bench_seed(uint32_t seed);

/* The inputs, exposed so the host test can compute the double reference. */
extern float    in_fa[BENCH_N], in_fb[BENCH_N], in_fang[BENCH_N];
extern int32_t  in_ia[BENCH_N], in_ib[BENCH_N];
extern q16_t    in_qa[BENCH_N], in_qb[BENCH_N];
extern q15_t    in_sa[BENCH_N], in_sb[BENCH_N];
extern uint16_t in_ang[BENCH_N];

/* The 16-tap low-pass filter in three number formats.  `x` is a 64-entry
 * ring of samples; the output uses x[k], x[k+1] ... x[k+15] (mod 64). */
extern const float   fir_h_f[FIR_TAPS];
extern const q15_t   fir_h_q15[FIR_TAPS];
extern const int32_t fir_h_int[FIR_TAPS];          /* sums to 256 */

float   fir_float(const float *x, uint32_t k);
q15_t   fir_q15(const q15_t *x, uint32_t k);
int32_t fir_int(const int32_t *x, uint32_t k);

#endif
