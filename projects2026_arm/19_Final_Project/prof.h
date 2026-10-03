/*
 * prof.h - per-task timing, measured on the target, for the timing budget
 *
 * TIM14 free-runs at 1 MHz: one count per microsecond, wrapping every
 * 65.536 ms.  A task stamps the counter when it wakes and again when its
 * work is done; the 16-bit difference is correct across one wrap, so any
 * span under 65 ms measures exactly.
 *
 * What it measures is RESPONSE time - wake to done, INCLUDING any time spent
 * preempted by higher-priority tasks and interrupts.  That is the number a
 * deadline cares about.  It is an upper bound on the task's own execution
 * time, not the execution time itself; the CPU% column of `stats` (counted
 * by the kernel's tick) is the other half of the picture.
 */
#ifndef PROF_H
#define PROF_H

#include <stdint.h>

typedef struct {
    const char *name;
    uint32_t period_ms;     /* the task's design rate                       */
    uint32_t runs;
    uint32_t resp_sum_us;   /* for the average                              */
    uint16_t resp_max_us;   /* worst wake-to-done seen                      */
    uint16_t late_max_ms;   /* worst lateness of the wake-up itself         */
    uint32_t overruns;      /* runs whose response exceeded the period      */
    uint16_t t0;            /* private: stamp at wake                       */
    volatile uint8_t clear; /* set by the shell; the task itself clears     */
} prof_t;

void     prof_init(void);                         /* TIM14 at 1 MHz        */
uint16_t prof_us(void);                           /* the counter, now      */

/* Call right after os_delay_until() returns; `release_ms` is the tick the
 * task was due (its `last`).  Call prof_end() when the work is done. */
void prof_begin(prof_t *p, uint32_t release_ms, uint32_t now_ms);
void prof_end(prof_t *p);
/* From ANOTHER task (the shell): ask for the counters to be zeroed.  The
 * owning task does it at its next prof_begin(), so every counter keeps one
 * writer - the same rule as health.h's beats. */
static inline void prof_request_clear(prof_t *p) { p->clear = 1u; }

#endif
