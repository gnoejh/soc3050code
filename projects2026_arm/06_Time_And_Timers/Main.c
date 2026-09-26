/*
 * Main.c - SOC3050 lesson 06: Time and Timers
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * A parking sensor, built from three clocks that all count the same 48 MHz:
 *
 *   SysTick   interrupts 1000 times a second.  ms_ticks is the program's
 *             sense of time, and it keeps counting while printf() blocks -
 *             which lesson 05's polled millisecond could not.
 *
 *   TIM14     counts microseconds and INPUT CAPTUREs the HC-SR04's echo
 *             pulse on PA7.  The hardware stamps each edge into CCR1 the
 *             instant it happens, so the measurement does not care how late
 *             the interrupt runs.
 *
 *   TIM3      counts microseconds and generates PWM on PA6: a 20 ms frame
 *             with a 1-2 ms pulse, which is what a hobby servo reads as an
 *             angle.  Its update interrupt, once per frame, eases the servo
 *             towards its target - the TIM3_IRQHandler lesson 04 promised.
 *
 * The servo is a gauge for distance, the LED bar a proximity bar: closer
 * means more LEDs.  Drag the HC-SR04's distance slider in Wokwi and watch
 * both follow.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, paste diagram.json into the Wokwi tab, upload
 *            Main.elf, then click the HC-SR04 to get its distance slider.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32c031xx.h"
#include "retarget.h"
#include "gpio.h"

#define BAUD            115200u

#define LD4_PIN         5u       /* PA5 - heartbeat                              */
#define SERVO_PIN       6u       /* PA6 - TIM3_CH1, alternate function 1          */
#define ECHO_PIN        7u       /* PA7 - TIM14_CH1, alternate function 4         */
#define TRIG_PIN        8u       /* PA8 - plain output, a 10 us pulse starts a ping */

#define AF_TIM3         1u       /* from the datasheet's AF table - per PIN       */
#define AF_TIM14        4u

#define TICK_HZ         1000000u /* both timers count microseconds                */

#define SERVO_FRAME_US  20000u   /* 50 Hz: the frame every hobby servo expects    */
#define SERVO_MIN_US    1000u    /* nominal 0 degrees   - Lab Part 4 measures     */
#define SERVO_MAX_US    2000u    /* nominal 180 degrees   where Wokwi's really are */
#define SERVO_SLEW_US   20u      /* most the pulse may change in one frame        */

#define PING_MS         100u     /* the HC-SR04 wants >= 60 ms between pings      */
#define REPORT_MS       1000u
#define HEARTBEAT_MS    500u
#define FAR_CM          200u     /* the gauge's full scale                        */

/* ============================================================================
 *  SysTick: time as an interrupt
 * ============================================================================
 * SysTick_Config() is CMSIS, not magic: it writes LOAD = ticks - 1, zeroes
 * VAL, sets SysTick's priority to the lowest, and sets CTRL to use the core
 * clock with the interrupt enabled.  Read it in core_cm0plus.h.
 */
static volatile uint32_t ms_ticks;

void SysTick_Handler(void)
{
    ms_ticks++;                    /* nothing to clear: SysTick's flag is not  */
}                                  /* an interrupt request, the wrap is        */

/* One aligned 32-bit word, one writer: safe to read anywhere (lesson 03,
 * slide 7).  It wraps after 49.7 days - and `now - then` in unsigned
 * arithmetic is still right across the wrap, which is why every timeout in
 * this file is written as a subtraction and never as `now > deadline`. */
static inline uint32_t millis(void) { return ms_ticks; }

/* ============================================================================
 *  TIM3: PWM for the servo, and an update interrupt once per frame
 * ============================================================================
 *   48 MHz --PSC+1 = 48--> 1 MHz --ARR+1 = 20000--> 50 Hz update
 *                                  CNT < CCR1 ? PA6 high : PA6 low
 */
static volatile uint32_t servo_target_us = 1500u;
static volatile uint32_t tim3_updates;

static void servo_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;             /* clock gate, first      */
    pin_af(GPIOA, SERVO_PIN, AF_TIM3);

    TIM3->PSC  = SystemCoreClock / TICK_HZ - 1u;    /* 47: one count per us   */
    TIM3->ARR  = SERVO_FRAME_US - 1u;               /* 19999: 20 ms frame     */
    TIM3->CCR1 = 1500u;                             /* mid-travel to start    */

    /* Channel 1 as output, PWM mode 1 (110: high while CNT < CCR1), with
     * preload - a new CCR1 waits for the next update, so a pulse is never
     * cut short half-way through a frame. */
    TIM3->CCMR1 = (TIM3->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M))
                | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM3->CCER |= TIM_CCER_CC1E;                    /* drive the pin          */
    TIM3->CR1  |= TIM_CR1_ARPE;                     /* ARR preloaded too      */

    /* PSC, ARR and CCR1 are all shadowed: the values above sit in preload
     * registers until an update event.  UG forces one now.  It also sets UIF,
     * so clear that before unmasking, or the first interrupt is a phantom. */
    TIM3->EGR  = TIM_EGR_UG;
    TIM3->SR   = ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;                       /* and go                 */
}

/* Once per 20 ms frame.  Moves the pulse at most SERVO_SLEW_US towards the
 * target, so the servo glides instead of snapping - a rate limiter, the
 * smallest possible piece of control code.  Lab Part 3 breaks the clear. */
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;     /* WRITE 0 TO CLEAR - the opposite of EXTI */
        tim3_updates++;

        uint32_t now  = TIM3->CCR1;
        uint32_t want = servo_target_us;
        if      (want > now + SERVO_SLEW_US) { now += SERVO_SLEW_US; }
        else if (want + SERVO_SLEW_US < now) { now -= SERVO_SLEW_US; }
        else                                 { now  = want; }
        TIM3->CCR1 = now;           /* preloaded: takes effect next frame      */
    }
}

/* ============================================================================
 *  TIM14: input capture - the hardware writes down the time for you
 * ============================================================================
 * TIM14 free-runs at 1 MHz over its full 16 bits (65.5 ms).  On every edge of
 * PA7 - rising AND falling, because CC1P and CC1NP are both set - it copies
 * CNT into CCR1 and raises CC1IF.  The copy happens in hardware at the edge;
 * the interrupt only collects it.
 */
static volatile uint16_t echo_rise;
static volatile uint32_t echo_us;        /* width of the last complete pulse  */
static volatile uint32_t echo_count;     /* complete pulses measured          */

static void ranger_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;

    pin_mode(GPIOA, TRIG_PIN, MODE_OUTPUT);
    pin_low(GPIOA, TRIG_PIN);
    pin_af(GPIOA, ECHO_PIN, AF_TIM14);

    TIM14->PSC   = SystemCoreClock / TICK_HZ - 1u;
    TIM14->ARR   = 0xFFFFu;
    TIM14->CCMR1 = (TIM14->CCMR1 & ~TIM_CCMR1_CC1S) | TIM_CCMR1_CC1S_0;  /* input, TI1 */
    TIM14->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E;       /* both edges */
    TIM14->EGR   = TIM_EGR_UG;
    TIM14->SR    = 0u;
    TIM14->DIER |= TIM_DIER_CC1IE;

    NVIC_SetPriority(TIM14_IRQn, 1);
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1  |= TIM_CR1_CEN;
}

void TIM14_IRQHandler(void)
{
    if (TIM14->SR & TIM_SR_CC1IF) {
        uint16_t t = (uint16_t)TIM14->CCR1;          /* reading CCR1 clears CC1IF */

        /* Which edge was it?  The pin says, as long as the pulse is longer
         * than this handler's latency - the shortest echo, 2 cm, is 116 us. */
        if (pin_read(GPIOA, ECHO_PIN)) {
            echo_rise = t;
        } else {
            echo_us = (uint16_t)(t - echo_rise);     /* 16-bit subtraction: right */
            echo_count++;                            /* across one counter wrap   */
        }
    }
}

/* A 10 us high pulse on TRIG starts a measurement.  TIM14 is already counting
 * microseconds, so it doubles as the stopwatch for the pulse itself. */
static void ranger_ping(void)
{
    uint16_t t0 = (uint16_t)TIM14->CNT;
    pin_high(GPIOA, TRIG_PIN);
    while ((uint16_t)((uint16_t)TIM14->CNT - t0) < 12u) { }
    pin_low(GPIOA, TRIG_PIN);
}

/* ============================================================================
 *  Outputs: the gauge and the bar
 * ============================================================================ */
static void bar_init(void)
{
    for (uint32_t pin = 0; pin < 8u; pin++) { pin_mode(GPIOB, pin, MODE_OUTPUT); }
}

static void bar_write(uint8_t leds)                  /* lesson 05: one BSRR store */
{
    GPIOB->BSRR = ((uint32_t)(uint8_t)~leds << 16) | leds;
}

static uint32_t clamp_cm(uint32_t cm) { return cm > FAR_CM ? FAR_CM : cm; }

/* Near = full deflection.  Integer arithmetic only - this core has no FPU. */
static uint32_t gauge_us(uint32_t cm)
{
    return SERVO_MAX_US - clamp_cm(cm) * (SERVO_MAX_US - SERVO_MIN_US) / FAR_CM;
}

static uint8_t proximity_bar(uint32_t cm)
{
    uint32_t lit = (FAR_CM - clamp_cm(cm)) * 8u / FAR_CM;   /* 0..8 LEDs */
    return (uint8_t)((1u << lit) - 1u);
}

/* ============================================================================
 *  The banner: every clock, read back from its registers
 * ============================================================================ */
static void report_config(void)
{
    uint32_t tim3_hz = SystemCoreClock / ((TIM3->PSC + 1u) * (TIM3->ARR + 1u));

    printf("\n=== SOC3050 lesson 06 - Time and Timers ===\n");
    printf("    STM32C031C6, Cortex-M0+, %lu Hz\n\n", (unsigned long)SystemCoreClock);

    printf("-- SysTick --\n");
    printf("  LOAD = %lu  ->  %lu interrupts/s    handler %s\n\n",
           (unsigned long)SysTick->LOAD,
           (unsigned long)(SystemCoreClock / (SysTick->LOAD + 1u)),
           handler_installed(SysTick_IRQn) ? "installed" : "NOT INSTALLED");

    printf("-- TIM3: PWM on PA6 (AF%lu) --\n",
           (unsigned long)((GPIOA->AFR[0] >> (SERVO_PIN * 4u)) & 0xFu));
    printf("  PSC = %lu  ARR = %lu  ->  %lu Hz update, 1 count = 1 us\n",
           (unsigned long)TIM3->PSC, (unsigned long)TIM3->ARR, (unsigned long)tim3_hz);
    printf("  CCR1 = %lu us pulse    TIM3_IRQHandler %s\n\n",
           (unsigned long)TIM3->CCR1,
           handler_installed(TIM3_IRQn) ? "installed" : "NOT INSTALLED");

    printf("-- TIM14: input capture on PA7 (AF%lu) --\n",
           (unsigned long)((GPIOA->AFR[0] >> (ECHO_PIN * 4u)) & 0xFu));
    printf("  PSC = %lu  ARR = %lu  ->  1 count = 1 us, wraps every 65.5 ms\n",
           (unsigned long)TIM14->PSC, (unsigned long)TIM14->ARR);
    printf("  CCER CC1P/CC1NP = %lu/%lu (both edges)    TIM14_IRQHandler %s\n\n",
           (unsigned long)((TIM14->CCER & TIM_CCER_CC1P)  ? 1u : 0u),
           (unsigned long)((TIM14->CCER & TIM_CCER_CC1NP) ? 1u : 0u),
           handler_installed(TIM14_IRQn) ? "installed" : "NOT INSTALLED");

    printf("Drag the HC-SR04 slider. One line per second:\n\n");
}

/* ============================================================================
 *                                  main
 * ============================================================================ */
int main(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    pin_mode(GPIOA, LD4_PIN, MODE_OUTPUT);
    bar_init();

    uart2_init(SystemCoreClock, BAUD);
    SysTick_Config(SystemCoreClock / 1000u);         /* 1 ms, interrupt driven */
    servo_init();
    ranger_init();

    report_config();

    uint32_t ping_at = 0, report_at = 0, beat_at = 0;
    uint32_t seen = 0, last_updates = 0, cm = 0, pulse = 0, print_ms = 0;
    uint8_t  beat_on = 0;

    for (;;) {
        uint32_t now = millis();

        /* ---- every 100 ms: start a measurement ---------------------------
         * `ping_at += PING_MS`, not `ping_at = now`: the schedule stays on
         * a 100 ms grid even when one pass of the loop runs late. */
        if (now - ping_at >= PING_MS) {
            ping_at += PING_MS;
            ranger_ping();
        }

        /* ---- a new echo? --------------------------------------------------
         * Two variables the ISR writes together.  Reading them one after the
         * other could pair a new count with an old width - lesson 03 slide
         * 7's "tearing".  So: a critical section, two loads long. */
        __disable_irq();
        uint32_t n  = echo_count;
        uint32_t us = echo_us;
        __enable_irq();

        if (n != seen) {
            seen  = n;
            pulse = us;
            cm    = us / 58u;                        /* HC-SR04: 58 us per cm */
            servo_target_us = gauge_us(cm);
            bar_write(proximity_bar(cm));
        }

        /* ---- every second: report, and time the report itself ------------ */
        if (now - report_at >= REPORT_MS) {
            report_at += REPORT_MS;
            uint32_t updates = tim3_updates;
            uint32_t t0 = millis();
            printf("t=%6lu ms  TIM3 %2lu/s  echo %5lu us = %3lu cm  servo %4lu us"
                   "  (last line took %lu ms)\n",
                   (unsigned long)now, (unsigned long)(updates - last_updates),
                   (unsigned long)pulse, (unsigned long)cm,
                   (unsigned long)TIM3->CCR1, (unsigned long)print_ms);
            print_ms = millis() - t0;
            last_updates = updates;
        }

        /* ---- heartbeat ---------------------------------------------------- */
        if (now - beat_at >= HEARTBEAT_MS) {
            beat_at += HEARTBEAT_MS;
            beat_on = (uint8_t)!beat_on;
            if (beat_on) { pin_high(GPIOA, LD4_PIN); } else { pin_low(GPIOA, LD4_PIN); }
        }
    }
}
