/*
 * Main.c - SOC3050 lesson 05: GPIO and Interrupts
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * Two buttons do the same job two different ways - the poll / interrupt pair
 * from lesson 03, made real.
 *
 *   Button A on PA0 is POLLED.  The main loop reads it once a millisecond and
 *   a debouncer decides when it has really been pressed.  Each press moves the
 *   lit LED one place LEFT on the bar.
 *
 *   Button B on PA1 is an INTERRUPT.  EXTI watches the pin, and every edge -
 *   rising or falling - runs EXTI0_1_IRQHandler(), which only counts.  The
 *   main loop decides what the edges meant.  Each press moves the LED RIGHT.
 *
 * Wokwi's pushbuttons BOUNCE, like real ones: one press is tens of edges in
 * about a millisecond.  The serial monitor prints, for every press and
 * release, how many transitions each method saw.  That difference is this
 * lesson's measurement.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, then upload Main.elf in the Wokwi browser tab.
 *            Paste diagram.json first - the buttons and the bar live there.
 *            Click a button, or focus the diagram and hold the A or B key.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32c031xx.h"
#include "retarget.h"

#define BAUD         115200u

#define LD4_PIN      5u        /* PA5 - the on-board green LED: heartbeat     */
#define BTN_A_PIN    0u        /* PA0 - button A, polled                      */
#define BTN_B_PIN    1u        /* PA1 - button B, EXTI line 1                 */

#define DEBOUNCE_MS  20u       /* a level must hold this long to count        */
#define HEARTBEAT_MS 500u

/* ============================================================================
 *  The Map: one GPIO pin is four 2-bit fields and three bits
 * ============================================================================
 * Every pin owns a 2-bit field in MODER and PUPDR, at bit (pin * 2), and one
 * bit in IDR, ODR and BSRR.  These helpers are the whole of it.  They are
 * plain functions on purpose - read them, then find the same registers in
 * the GPIO chapter of RM0490 and check the bit positions yourself.
 */
enum { MODE_INPUT = 0u, MODE_OUTPUT = 1u, MODE_AF = 2u, MODE_ANALOG = 3u };
enum { PULL_NONE  = 0u, PULL_UP     = 1u, PULL_DOWN = 2u };

static void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (mode << (pin * 2u));
}

static void pin_pull(GPIO_TypeDef *port, uint32_t pin, uint32_t pull)
{
    port->PUPDR = (port->PUPDR & ~(3u << (pin * 2u))) | (pull << (pin * 2u));
}

/* BSRR: the low half SETS pins, the high half RESETS them, and zeros do
 * nothing.  One store, no read - so nothing can slip in between a read and
 * a write, and no other pin on the port is touched.  Compare `ODR |= bit`,
 * which is a load, an OR and a store: lesson 03, slide 5's lost update. */
static inline void pin_high(GPIO_TypeDef *port, uint32_t pin) { port->BSRR = 1u << pin; }
static inline void pin_low (GPIO_TypeDef *port, uint32_t pin) { port->BSRR = 1u << (pin + 16u); }

static inline uint32_t pin_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 1u;
}

/* The buttons connect the pin to GND when pressed, and the internal pull-up
 * holds it high otherwise.  So pressed reads 0: "active low". */
static inline uint32_t button_down(uint32_t pin) { return pin_read(GPIOA, pin) == 0u; }

/* ---- The LED bar: PB0..PB7, one byte, written with ONE store -------------
 * Lesson 04 wrote GPIOB->ODR = leds, which also forced PB8..PB15 to zero.
 * Here the high half of BSRR clears the bar's pins that should be off and the
 * low half sets the ones that should be on.  PB8..PB15 are not mentioned, so
 * they are not touched.
 *
 * noinline keeps it a real function with its own symbol, so Lab Part 6 can
 * disassemble it by name.  Without it -Os folds it into main(). */
__attribute__((noinline))
static void bar_write(uint8_t leds)
{
    GPIOB->BSRR = ((uint32_t)(uint8_t)~leds << 16) | leds;
}

static uint32_t moder_at_reset;         /* GPIOA->MODER before we touched it */

static void gpio_init(void)
{
    /* Clock both ports FIRST.  A write to an unclocked port vanishes - lesson
     * 04, Lab Part 0, step 5. */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    moder_at_reset = GPIOA->MODER;      /* kept for the banner */

    pin_mode(GPIOA, LD4_PIN, MODE_OUTPUT);

    for (uint32_t pin = 0; pin < 8u; pin++) {
        pin_mode(GPIOB, pin, MODE_OUTPUT);
    }

    /* A pin comes out of reset in ANALOG mode (11), not input.  An analog pin
     * has its input buffer switched off and reads 0 in IDR - which, for an
     * active-low button, means "pressed", permanently.  So input mode is
     * written explicitly, never assumed.  Lab Part 1 removes this line. */
    pin_mode(GPIOA, BTN_A_PIN, MODE_INPUT);
    pin_mode(GPIOA, BTN_B_PIN, MODE_INPUT);
    pin_pull(GPIOA, BTN_A_PIN, PULL_UP);
    pin_pull(GPIOA, BTN_B_PIN, PULL_UP);
}

/* ============================================================================
 *  Button B: the interrupt path
 * ============================================================================
 * Three hops from the pin to your function, and each has its own switch:
 *
 *   pin PA1 --EXTICR--> EXTI line 1 --RTSR1/FTSR1, IMR1--> NVIC IRQ 5
 *            (which port)             (which edges, unmasked)
 *                                                      --ISER--> vector 21
 *
 * On the STM32C0 the port select is EXTI->EXTICR[].  On an F4 the same job is
 * SYSCFG->EXTICR[], and code copied from an F4 tutorial will not compile here.
 */
#define EXTI_PORT_A  0u        /* EXTICR codes: A=0, B=1, C=2, D=3, F=5       */

static void button_b_irq_init(void)
{
    const uint32_t line  = BTN_B_PIN;               /* line number = pin number */
    const uint32_t shift = (line % 4u) * 8u;        /* one byte per line        */

    EXTI->EXTICR[line / 4u] = (EXTI->EXTICR[line / 4u] & ~(0xFFu << shift))
                            | (EXTI_PORT_A << shift);

    EXTI->RTSR1 |= 1u << line;                      /* rising edge  (release)   */
    EXTI->FTSR1 |= 1u << line;                      /* falling edge (press)     */

    EXTI->RPR1 = 1u << line;                        /* discard anything stale - */
    EXTI->FPR1 = 1u << line;                        /* write 1 to clear         */

    EXTI->IMR1 |= 1u << line;                       /* unmask: let it reach NVIC */

    NVIC_SetPriority(EXTI0_1_IRQn, 2);              /* 0 (highest) .. 3 on M0+  */
    NVIC_EnableIRQ(EXTI0_1_IRQn);                   /* Lab Part 5 removes this  */
}

/* One writer (this handler), one reader (main).  A 32-bit aligned load or
 * store is a single instruction on this core, so main can read the count
 * without a lock - it sees the old value or the new one, never half of each.
 * That is lesson 03 slide 7's one safe case: one aligned word, one writer.
 * `volatile` is what stops main caching it in a register (slide 4). */
static volatile uint32_t b_edges;

/* The name is the whole connection.  startup.c puts a weak alias called
 * EXTI0_1_IRQHandler in vector 21; defining a function with exactly this name
 * replaces it at link time.  Spell it differently and you have written an
 * ordinary function nobody calls - Lab Part 4. */
void EXTI0_1_IRQHandler(void)
{
    uint32_t mask = 1u << BTN_B_PIN;

    if ((EXTI->RPR1 | EXTI->FPR1) & mask) {
        /* Clear BOTH pending bits, or the NVIC sees the line still pending
         * the moment this function returns, and calls it again, forever.
         * Lab Part 3 removes these two lines. */
        EXTI->RPR1 = mask;
        EXTI->FPR1 = mask;
        b_edges++;
    }
}

/* ============================================================================
 *  Time: a polled millisecond (lesson 06 makes this an interrupt)
 * ============================================================================ */
static uint32_t ms_now;

static void tick_init(void)
{
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

/* COUNTFLAG latches ONE wrap.  If the loop is away for 5 ms - printing, say -
 * four milliseconds are simply lost and ms_now runs slow.  Polled time is
 * only as good as the loop that polls it.  That is lesson 06's opening. */
static void tick_wait(void)
{
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) { }
    ms_now++;
}

/* ============================================================================
 *  Button A: the polled path, with a debouncer
 * ============================================================================
 * Called once per millisecond with the raw reading.  A new level counts only
 * after it has held for DEBOUNCE_MS samples in a row; anything shorter is
 * bounce.  Returns +1 on a press, -1 on a release, 0 otherwise.
 */
typedef struct {
    uint8_t  stable;           /* the level we have accepted: 1 = down        */
    uint8_t  raw;              /* the last raw sample                          */
    uint16_t held_ms;          /* how long raw has stayed the same             */
    uint32_t changes;          /* raw level changes seen - bounce included     */
} debounce_t;

static int debounce_step(debounce_t *d, uint8_t raw)
{
    if (raw != d->raw) {                 /* the level moved: restart the clock */
        d->raw     = raw;
        d->held_ms = 0;
        d->changes++;
        return 0;
    }
    if (d->held_ms < DEBOUNCE_MS) {
        d->held_ms++;
        if (d->held_ms == DEBOUNCE_MS && raw != d->stable) {
            d->stable = raw;
            return raw ? +1 : -1;
        }
    }
    return 0;
}

/* ============================================================================
 *  The banner: the configuration, read back from the registers
 * ============================================================================
 * Nothing below prints what the code MEANT to write.  Every field is read
 * back from the hardware, so a write that did not take shows up here.
 */
extern void (* const vectors[])(void);   /* startup.c */
void Default_Handler(void);              /* startup.c */

static uint32_t field2(uint32_t reg, uint32_t pin) { return (reg >> (pin * 2u)) & 3u; }

static void report_config(void)
{
    static const char *const mode[] = { "input ", "output", "AF    ", "ANALOG" };
    static const char *const pull[] = { "none   ", "pull-up", "pull-dn", "?      " };
    const uint32_t vec = 16u + (uint32_t)EXTI0_1_IRQn;         /* = 21 */
    const int mine = vectors[vec] != Default_Handler;

    printf("\n=== SOC3050 lesson 05 - GPIO and Interrupts ===\n");
    printf("    STM32C031C6, Cortex-M0+, %lu Hz\n\n", (unsigned long)SystemCoreClock);

    printf("-- how the port came out of reset --\n");
    printf("  GPIOA->MODER    : 0x%08lX   (RM0490 says 0xEBFFFFFF: every pin\n",
           (unsigned long)moder_at_reset);
    printf("                    ANALOG (11) except PA13/PA14, the debug pins)\n\n");

    printf("-- the pins, read back from MODER and PUPDR --\n");
    printf("  PA0 button A : %s  %s   polled every 1 ms\n",
           mode[field2(GPIOA->MODER, BTN_A_PIN)], pull[field2(GPIOA->PUPDR, BTN_A_PIN)]);
    printf("  PA1 button B : %s  %s   EXTI line 1\n",
           mode[field2(GPIOA->MODER, BTN_B_PIN)], pull[field2(GPIOA->PUPDR, BTN_B_PIN)]);
    printf("  PA5 LD4      : %s             heartbeat\n",
           mode[field2(GPIOA->MODER, LD4_PIN)]);
    printf("  idle levels  : A=%lu B=%lu   (1 = released; pull-up holds it high)\n\n",
           (unsigned long)pin_read(GPIOA, BTN_A_PIN), (unsigned long)pin_read(GPIOA, BTN_B_PIN));

    printf("-- the interrupt path, hop by hop --\n");
    printf("  EXTICR[0] line 1 port : %lu    (0 = port A)\n",
           (unsigned long)((EXTI->EXTICR[0] >> 8) & 0xFFu));
    printf("  RTSR1 / FTSR1 bit 1   : %lu / %lu (rising / falling edge)\n",
           (unsigned long)((EXTI->RTSR1 >> 1) & 1u), (unsigned long)((EXTI->FTSR1 >> 1) & 1u));
    printf("  IMR1 bit 1            : %lu    (1 = unmasked)\n",
           (unsigned long)((EXTI->IMR1 >> 1) & 1u));
    printf("  NVIC IRQ %d enabled    : %lu    priority %lu of 0..3\n",
           (int)EXTI0_1_IRQn, (unsigned long)NVIC_GetEnableIRQ(EXTI0_1_IRQn),
           (unsigned long)NVIC_GetPriority(EXTI0_1_IRQn));
    printf("  vector %lu              : 0x%08lX  %s\n\n", (unsigned long)vec,
           (unsigned long)(uintptr_t)vectors[vec],
           mine ? "EXTI0_1_IRQHandler - yours"
                : "Default_Handler - YOUR HANDLER IS NOT INSTALLED");

    printf("Press A (moves the LED left) or B (moves it right).\n");
    printf("Each line shows how many transitions that method saw.\n\n");
}

/* ============================================================================
 *                                  main
 * ============================================================================ */
int main(void)
{
    gpio_init();
    uart2_init(SystemCoreClock, BAUD);
    button_b_irq_init();
    tick_init();

    report_config();

    uint32_t cursor = 0;                 /* which LED is lit, 0 = PB0          */
    bar_write((uint8_t)(1u << cursor));

    debounce_t a = { 0 };
    uint32_t a_count = 0, a_reported = 0;

    uint32_t b_seen = 0, b_reported = 0, b_count = 0;
    uint32_t b_quiet_ms = 0;
    uint8_t  b_armed = 0, b_down = 0;

    uint32_t beat_at = 0;
    uint8_t  beat_on = 0;

    for (;;) {
        tick_wait();

        /* ---- A: sample, debounce, act ------------------------------------ */
        int ev = debounce_step(&a, (uint8_t)button_down(BTN_A_PIN));
        if (ev != 0) {
            if (ev > 0) {
                a_count++;
                cursor = (cursor + 1u) & 7u;            /* left, towards PB7  */
                bar_write((uint8_t)(1u << cursor));
            }
            printf("A polled     %-7s %3lu   level changes seen at 1 kHz: %lu\n",
                   ev > 0 ? "press" : "release", (unsigned long)a_count,
                   (unsigned long)(a.changes - a_reported));
            a_reported = a.changes;
        }

        /* ---- B: the ISR only counted edges; decide what they meant -------
         * Every new edge restarts a quiet-time clock.  When the line has been
         * quiet for DEBOUNCE_MS, the bouncing is over: read the pin once and
         * see whether it settled pressed or released. */
        uint32_t edges = b_edges;
        if (edges != b_seen) {
            b_seen     = edges;
            b_quiet_ms = 0;
            b_armed    = 1;
        } else if (b_armed && ++b_quiet_ms >= DEBOUNCE_MS) {
            b_armed = 0;
            uint8_t down = (uint8_t)button_down(BTN_B_PIN);
            if (down != b_down) {
                b_down = down;
                if (down) {
                    b_count++;
                    cursor = (cursor + 7u) & 7u;        /* right, towards PB0 */
                    bar_write((uint8_t)(1u << cursor));
                }
                printf("B interrupt  %-7s %3lu   edges caught by EXTI:        %lu\n",
                       down ? "press" : "release", (unsigned long)b_count,
                       (unsigned long)(edges - b_reported));
                b_reported = edges;
            }
        }

        /* ---- heartbeat: proof the main loop is still running ------------- */
        if (ms_now - beat_at >= HEARTBEAT_MS) {
            beat_at = ms_now;
            beat_on = (uint8_t)!beat_on;
            if (beat_on) { pin_high(GPIOA, LD4_PIN); } else { pin_low(GPIOA, LD4_PIN); }
        }
    }
}
