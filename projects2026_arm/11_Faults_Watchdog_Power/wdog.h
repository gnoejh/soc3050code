/*
 * wdog.h - three watchdogs, and feeding them honestly    (SOC3050 lesson 11)
 *
 * The top half is pure arithmetic and logic - static inline, no device
 * header - so host/test.c compiles it and simulates every feeding policy.
 * The bottom half is the driver, in wdog.c.
 *
 * THREE DOGS, because the simulator models only some of them.  Wokwi's
 * Nucleo-C031C6 page lists IWDG as NOT implemented and WWDG as "Implemented,
 * not tested yet" (docs.wokwi.com/parts/board-st-nucleo-c031c6):
 *
 *   SOFT   a counter in the SysTick interrupt; at zero it calls
 *          NVIC_SystemReset().  Works everywhere, including Wokwi.  Armed at
 *          boot.  Weakness: it is software - a hang with interrupts OFF, or a
 *          stuck interrupt handler, stops it too.
 *   WWDG   the window watchdog: PCLK-clocked, 7-bit counter.  Real hardware,
 *          and the one Wokwi claims to model.  `arm wwdg`.
 *   IWDG   the independent watchdog: its own ~32 kHz LSI oscillator, so it
 *          survives a dead main clock.  The one to ship on silicon.  NOT
 *          modelled by Wokwi: armed there, it never bites.  `arm iwdg`.
 *
 * Once armed, neither hardware dog can be stopped except by a reset (ST's
 * HAL: IWDG - "the LSI is forced ON and both cannot be disabled"; WWDG -
 * "Once enabled the WWDG cannot be disabled except by a system reset").
 */
#ifndef WDOG_H
#define WDOG_H

#include <stdint.h>

#define LSI_HZ         32000u      /* nominal: the HAL quotes "@32KHz (LSI)"    */
#define IWDG_RL_MAX    0xFFFu      /* RLR is 12 bits                            */
#define WDT_MS_DEFAULT 1000u       /* boot setting of the soft dog; Lab Part 5  */

/* ---- IWDG timeout arithmetic -------------------------------------------------
 * The counter is clocked at LSI / div, div = 4 << PR (PR 0..6: /4 .. /256).
 * It is reloaded with RLR and counts RLR, RLR-1, ... 0: (RLR + 1) ticks.
 *
 *      timeout = (RLR + 1) * div / LSI
 *
 * Check: PR 0, RLR 0 -> 4/32000 s = 125 us; PR 6, RLR 4095 -> 32.768 s.
 * Exactly the HAL's "~125us / ~32.7s". */
static inline uint32_t iwdg_div(uint32_t pr) { return 4u << (pr > 6u ? 6u : pr); }

static inline uint32_t iwdg_timeout_us(uint32_t pr, uint32_t rlr, uint32_t lsi_hz)
{
    return (uint32_t)(((uint64_t)(rlr + 1u) * iwdg_div(pr) * 1000000u) / lsi_hz);
}

/* The SMALLEST prescaler that reaches `ms` - the finest resolution - with
 * the timeout rounded UP, never down: a watchdog that bites early is worse
 * than one a tick late.  Returns 0, or -1 if `ms` is out of range. */
static inline int iwdg_pick(uint32_t ms, uint32_t lsi_hz, uint32_t *pr, uint32_t *rlr)
{
    for (uint32_t p = 0; p <= 6u; p++) {
        uint64_t num = (uint64_t)ms * lsi_hz;            /* ticks = ms*lsi/(1000*div) */
        uint64_t den = 1000u * (uint64_t)iwdg_div(p);
        uint64_t ticks = (num + den - 1u) / den;         /* round up                  */
        if (ticks == 0u) { ticks = 1u; }
        if (ticks - 1u <= IWDG_RL_MAX) { *pr = p; *rlr = (uint32_t)(ticks - 1u); return 0; }
    }
    return -1;
}

/* ---- WWDG timeout arithmetic -------------------------------------------------
 * ST's stm32c0xx_hal_wwdg.c:
 *      WWDG clock (Hz) = PCLK1 / (4096 * Prescaler),  Prescaler = 2^WDGTB
 *      WWDG timeout (mS) = 1000 * (T[5;0] + 1) / WWDG clock (Hz)
 * The counter T[6:0] resets the chip when it rolls from 0x40 to 0x3F, so it
 * is loaded with 0x40 + ticks - 1.  At PCLK = 48 MHz the longest is
 * 64 * 4096 * 128 / 48 MHz = 699 ms - the WWDG cannot do a 1 s timeout. */
static inline uint32_t wwdg_timeout_us(uint32_t tb, uint32_t t, uint32_t pclk)
{
    return (uint32_t)(((uint64_t)((t & 0x3Fu) + 1u) * 4096u * (1u << tb) * 1000000u) / pclk);
}

/* Smallest WDGTB that reaches `ms` (rounded up).  If `ms` is beyond the
 * longest timeout, gives the longest and returns 1; else 0. */
static inline int wwdg_pick(uint32_t ms, uint32_t pclk, uint32_t *tb, uint32_t *t)
{
    for (uint32_t b = 0; b <= 7u; b++) {
        uint64_t den = 1000ull * 4096u * (1u << b);
        uint64_t ticks = ((uint64_t)ms * pclk + den - 1u) / den;
        if (ticks == 0u) { ticks = 1u; }
        if (ticks <= 64u) { *tb = b; *t = 0x40u + (uint32_t)ticks - 1u; return 0; }
    }
    *tb = 7u; *t = 0x7Fu;
    return 1;
}

/* ---- the soft dog ------------------------------------------------------------ */
typedef struct { uint32_t left, reload; uint8_t armed; } softdog_t;

static inline void soft_feed(volatile softdog_t *d) { d->left = d->reload; }

/* One millisecond passes.  1 = expired: reset now. */
static inline int soft_tick(volatile softdog_t *d)
{
    if (!d->armed) { return 0; }
    if (d->left > 0u) { d->left--; }
    return d->left == 0u;
}

/* ---- the heartbeat bitmap -----------------------------------------------------
 * The mistake this prevents: feeding the watchdog from a timer interrupt, or
 * from the top of the main loop.  Both keep feeding while one job is stuck,
 * so the watchdog never sees the failure it exists to catch.
 *
 * Instead every job sets its own bit each time it completes a cycle, and the
 * watchdog is fed only when ALL expected bits are in - then the bits clear and
 * the round starts again.  One stuck job, no more feeding, reset.  */
typedef struct {
    uint32_t expected;     /* one bit per job that must check in           */
    uint32_t seen;         /* bits in since the last feed                  */
} heartbeat_t;

static inline void hb_checkin(volatile heartbeat_t *h, uint32_t bit) { h->seen |= bit; }

/* 1 = everyone checked in: feed now (and start a new round). */
static inline int hb_ready(volatile heartbeat_t *h)
{
    if ((h->seen & h->expected) != h->expected) { return 0; }
    h->seen = 0;
    return 1;
}

/* The bits still missing - which job is late. */
static inline uint32_t hb_missing(const volatile heartbeat_t *h)
{
    return h->expected & ~h->seen;
}

/* ---- the driver (wdog.c) ---------------------------------------------------- */
enum { DOG_SOFT = 1u << 0, DOG_WWDG = 1u << 1, DOG_IWDG = 1u << 2 };

int      dog_arm(uint32_t which, uint32_t ms);  /* arm (or retune) one dog; 0 ok */
int      dog_set_ms(uint32_t ms);               /* retune every armed dog        */
void     dog_feed(void);                        /* feed every armed dog          */
uint32_t dog_armed(void);                       /* DOG_* bits                    */
uint32_t dog_ms(uint32_t which);                /* timeout actually programmed   */
int      dog_soft_tick(void);                   /* SysTick: 1 = soft dog expired */

#endif
