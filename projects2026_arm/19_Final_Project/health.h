/*
 * health.h - "is every task still alive?"  The decision behind the watchdog.
 *
 * A hardware watchdog (wdog.c) resets the chip unless somebody refreshes it
 * in time.  The naive design refreshes it from one task - which proves only
 * that THAT task runs.  Here every periodic task beats its own counter each
 * time round its loop, and a low-priority health task refreshes the watchdog
 * only if EVERY expected task has beaten since it last looked.  One task
 * stuck, starved or deadlocked and the refreshes stop.
 *
 * Why counters and not a shared bitmap of "I'm alive" bits?  `seen |= bit`
 * is a read-modify-write: a task preempted between its load and its store
 * writes back a stale word and erases the bit a higher-priority task set in
 * between (lesson 03, slide 7).  A counter per task has exactly ONE writer -
 * its own task - and the health task only reads it.  The BITMAP still exists:
 * it is the result, `missing`, built by the one task that owns it.
 *
 * Pure C: no registers, no RTOS.  host/test_app.c drives it with a fake clock.
 */
#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

#define HEALTH_MAX  8u

enum { HEALTH_FEED = 0,      /* everyone beat: refresh the watchdog          */
       HEALTH_STARVE = 1,    /* someone missing: do NOT refresh              */
       HEALTH_FORCE = 2 };   /* still running long after the hardware should
                                have reset us: no hardware watchdog here     */

typedef struct {
    volatile uint32_t beats[HEALTH_MAX]; /* beats[i]: written ONLY by task i  */
    uint32_t prev[HEALTH_MAX];           /* health task's copy at last look   */
    uint32_t n;                          /* tasks being watched               */
    uint32_t missing;                    /* bit i: task i did not beat        */
    uint32_t last_feed_ms;
    uint32_t force_after_ms;             /* > the hardware timeout            */
    uint32_t feeds, starved_checks;
} health_t;

void health_init(health_t *h, uint32_t n_tasks, uint32_t force_after_ms, uint32_t now_ms);

/* From task i, once per loop.  One aligned 32-bit store: atomic on M0+. */
static inline void health_beat(health_t *h, uint32_t i) { h->beats[i]++; }

/* From the health task, periodically.  Returns HEALTH_FEED/STARVE/FORCE. */
int  health_check(health_t *h, uint32_t now_ms);

#endif
