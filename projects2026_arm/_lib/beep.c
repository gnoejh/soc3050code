/*
 * beep.c - tones on PA6 / TIM3_CH1   LIBS=beep
 *
 *   timer clock 48 MHz / (PSC + 1 = 48)  = 1 MHz: one count per microsecond
 *   ARR + 1 = 1 000 000 / hz             period, in microseconds
 *   CCR1    = (ARR + 1) / 2              high for half of it: a square wave
 *
 * 440 Hz is ARR = 2272.  Anything from 16 Hz to 20 kHz fits in 16 bits.
 */
#include "stm32c031xx.h"
#include "gpio.h"
#include "beep.h"

#define BUZZ_PIN  6u            /* PA6 */
#define AF_TIM3   1u            /* PA6 = TIM3_CH1 on AF1 (lesson 06)  */

static uint32_t stop_at;
static uint8_t  on;

void beep_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    pin_af(GPIOA, BUZZ_PIN, AF_TIM3);

    TIM3->PSC   = 48u - 1u;                                 /* 1 MHz            */
    TIM3->ARR   = 1000u - 1u;
    TIM3->CCR1  = 0;                                        /* silent: 0% duty  */
    TIM3->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;   /* PWM mode 1 */
    TIM3->CCER  = TIM_CCER_CC1E;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->EGR   = TIM_EGR_UG;                               /* load PSC and ARR */
    TIM3->CR1  |= TIM_CR1_CEN;
}

void beep(uint32_t hz, uint32_t ms, uint32_t now_ms)
{
    if (hz < 16u || hz > 20000u) { TIM3->CCR1 = 0; on = 0; return; }
    uint32_t period = 1000000u / hz;
    TIM3->ARR  = period - 1u;        /* preloaded: takes effect at the next update, */
    TIM3->CCR1 = period / 2u;        /* so a pitch change never makes a runt pulse  */
    stop_at = now_ms + ms;
    on = 1;
}

void beep_poll(uint32_t now_ms)
{
    if (on && (int32_t)(now_ms - stop_at) >= 0) { TIM3->CCR1 = 0; on = 0; }
}

int beep_busy(void) { return on; }
