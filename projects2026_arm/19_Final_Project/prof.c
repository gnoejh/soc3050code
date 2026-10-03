/*
 * prof.c - TIM14 as a microsecond stopwatch.  See prof.h.
 *
 * TIM14 because the template leaves it free: TIM3 is the buzzer's (beep.c),
 * SysTick is the kernel's.  The Part 0 peripheral model once more - gate
 * (APBENR2.TIM14EN), control (PSC, ARR, CR1), data (CNT) - and lesson 06's
 * prescaler arithmetic: 48 MHz / (47 + 1) = 1 MHz.
 */
#include "stm32c031xx.h"
#include "prof.h"

void prof_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;
    TIM14->PSC = 48u - 1u;              /* 1 MHz, if SystemCoreClock = 48 MHz */
    TIM14->ARR = 0xFFFFu;               /* free-run the full 16 bits          */
    TIM14->EGR = TIM_EGR_UG;            /* load PSC now, not at the first wrap */
    TIM14->CR1 = TIM_CR1_CEN;
}

uint16_t prof_us(void) { return (uint16_t)TIM14->CNT; }

void prof_begin(prof_t *p, uint32_t release_ms, uint32_t now_ms)
{
    if (p->clear) {
        p->runs = 0;  p->resp_sum_us = 0;  p->resp_max_us = 0;
        p->late_max_ms = 0;  p->overruns = 0;
        p->clear = 0;
    }
    p->t0 = prof_us();
    uint32_t late = now_ms - release_ms;
    if (late > 0xFFFFu) { late = 0xFFFFu; }
    if (late > p->late_max_ms) { p->late_max_ms = (uint16_t)late; }
}

void prof_end(prof_t *p)
{
    uint16_t dt = (uint16_t)(prof_us() - p->t0);   /* modulo 2^16: one wrap ok */
    p->runs++;
    p->resp_sum_us += dt;
    if (dt > p->resp_max_us) { p->resp_max_us = dt; }
    if ((uint32_t)dt > p->period_ms * 1000u) { p->overruns++; }
}
