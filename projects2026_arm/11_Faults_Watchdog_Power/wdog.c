/*
 * wdog.c - three watchdog drivers behind one feed()      (SOC3050 lesson 11)
 *
 * See wdog.h for why there are three, and which one the simulator models.
 *
 * IWDG: no enable BIT.  Everything goes through the key register KR, and each
 * key is a 16-bit pattern a runaway program is unlikely to write by accident
 * (values from ST's stm32c0xx_hal_iwdg.h):
 *
 *      0xCCCC   start         - and force the LSI on; cannot be undone
 *      0x5555   unlock        - PR, RLR and WINR become writable
 *      0xAAAA   refresh       - reload the counter from RLR ("feed", "kick")
 *
 * The order below is ST's own HAL_IWDG_Init(): start, unlock, write PR and
 * RLR, wait until SR says both reached the LSI domain, then refresh.
 *
 * WWDG: ST's HAL_WWDG_Init() writes CR = WDGA | counter, then CFR = EWI |
 * prescaler | window.  Window W = 0x7F here: the counter never exceeds 0x7F,
 * so a refresh is never "too early" and the window feature is off.  Lab
 * Part 7 turns it on.
 */

#include "stm32c031xx.h"
#include "wdog.h"

#define KEY_START   0xCCCCu
#define KEY_UNLOCK  0x5555u
#define KEY_FEED    0xAAAAu

static volatile softdog_t soft;
static uint32_t armed, iwdg_ms_, wwdg_ms_, wwdg_reload = 0x7Fu;

static int iwdg_start(uint32_t ms)
{
    uint32_t pr, rlr;
    if (iwdg_pick(ms, LSI_HZ, &pr, &rlr) != 0) { return -1; }

    /* A watchdog that keeps counting while GDB holds the core at a breakpoint
     * resets the chip under your debugger.  DBG_IWDG_STOP freezes it while the
     * core is halted.  (Behind the DBG clock gate.  Wokwi does not model DBG
     * either - which is why this happens only when you arm the IWDG.) */
    RCC->APBENR1 |= RCC_APBENR1_DBGEN;
    DBG->APBFZ1  |= DBG_APB_FZ1_DBG_IWDG_STOP;

    IWDG->KR  = KEY_START;
    IWDG->KR  = KEY_UNLOCK;
    IWDG->PR  = pr;
    IWDG->RLR = rlr;

    /* PR and RLR cross into the slow LSI domain; SR's PVU/RVU stay set until
     * they have.  ST waits up to ~6 LSI periods x 256 (48 ms).  Bounded here
     * too - every wait in this course has a limit (lesson 09, slide 9). */
    for (uint32_t i = 0; i < 2000000u; i++) {
        if (!(IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU))) { break; }
    }
    IWDG->KR = KEY_FEED;
    iwdg_ms_ = iwdg_timeout_us(pr, rlr, LSI_HZ) / 1000u;
    return 0;
}

static void wwdg_start(uint32_t ms)
{
    uint32_t tb, t;
    (void)wwdg_pick(ms, SystemCoreClock, &tb, &t);    /* PCLK = HCLK: APB /1 */
    RCC->APBENR1 |= RCC_APBENR1_WWDGEN;
    WWDG->CR  = WWDG_CR_WDGA | t;                     /* on, and loaded      */
    WWDG->CFR = (tb << WWDG_CFR_WDGTB_Pos) | WWDG_CFR_W;   /* W = 0x7F       */
    WWDG->CR  = t;                                    /* refresh: WDGA stays */
    wwdg_reload = t;
    wwdg_ms_ = wwdg_timeout_us(tb, t, SystemCoreClock) / 1000u;
}

int dog_arm(uint32_t which, uint32_t ms)
{
    if (ms == 0u || ms > 32768u) { return -1; }
    if (which & DOG_SOFT) { soft.reload = ms; soft.left = ms; soft.armed = 1; }
    if (which & DOG_WWDG) { wwdg_start(ms); }
    if ((which & DOG_IWDG) && iwdg_start(ms) != 0) { return -1; }
    armed |= which;
    return 0;
}

int dog_set_ms(uint32_t ms) { return dog_arm(armed, ms); }

/* Feed every dog that is armed.  Called from ONE place in the main loop -
 * or, in the "isr" mode of Lab Part 4, from SysTick, which is the mistake. */
void dog_feed(void)
{
    soft_feed(&soft);
    if (armed & DOG_WWDG) { WWDG->CR = wwdg_reload; }
    if (armed & DOG_IWDG) { IWDG->KR = KEY_FEED; }
}

uint32_t dog_armed(void) { return armed; }

uint32_t dog_ms(uint32_t which)
{
    if (which == DOG_SOFT) { return soft.armed ? soft.reload : 0u; }
    if (which == DOG_WWDG) { return (armed & DOG_WWDG) ? wwdg_ms_ : 0u; }
    return (armed & DOG_IWDG) ? iwdg_ms_ : 0u;
}

int dog_soft_tick(void) { return soft_tick(&soft); }
