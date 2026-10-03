/*
 * health.c - heartbeat bookkeeping for the watchdog.  See health.h.
 */
#include "health.h"

void health_init(health_t *h, uint32_t n_tasks, uint32_t force_after_ms, uint32_t now_ms)
{
    h->n = n_tasks > HEALTH_MAX ? HEALTH_MAX : n_tasks;
    for (uint32_t i = 0; i < HEALTH_MAX; i++) { h->beats[i] = 0; h->prev[i] = 0; }
    h->missing        = 0;
    h->last_feed_ms   = now_ms;
    h->force_after_ms = force_after_ms;
    h->feeds          = 0;
    h->starved_checks = 0;
}

int health_check(health_t *h, uint32_t now_ms)
{
    uint32_t missing = 0;
    for (uint32_t i = 0; i < h->n; i++) {
        uint32_t b = h->beats[i];            /* one read: the value we judge  */
        if (b == h->prev[i]) { missing |= 1u << i; }
        h->prev[i] = b;
    }
    h->missing = missing;

    if (missing == 0u) {
        h->last_feed_ms = now_ms;
        h->feeds++;
        return HEALTH_FEED;
    }
    h->starved_checks++;
    /* Unsigned subtraction: correct across the 49-day wrap of a ms counter. */
    if (now_ms - h->last_feed_ms >= h->force_after_ms) { return HEALTH_FORCE; }
    return HEALTH_STARVE;
}
