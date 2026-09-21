/*
 * Main.c - SOC3050 lesson 04: Startup and the Linker Script
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * The first program in this course that you build and run yourself.
 *
 * It has two halves.
 *
 *   1. A memory report.  Every number in it is MEASURED from a linker symbol,
 *      never typed in.  link.ld decides where your program's pieces live,
 *      startup.c moves them into place, and this program reads back what
 *      those two files actually did.  Lab.md breaks both files on purpose
 *      and this report is how you see the damage.  Leave this half alone.
 *
 *   2. An LED bar - eight LEDs on PB0..PB7 - driven by patterns you write.
 *      A pattern is a table of frames: which LEDs are lit, and for how long.
 *      This half is YOURS.  Look for "YOUR PART" below.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, then upload Main.elf in the Wokwi browser tab.
 *            The eight LEDs come from diagram.json - simulate.bat tells you
 *            how to load it.  Without it you still get the on-board LED and
 *            the serial monitor, just no bar.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32c031xx.h"

#define LED_PIN   5u          /* PA5 - the user LED, hardwired on the Nucleo */
#define BAUD      115200u

void uart2_init(uint32_t pclk, uint32_t baud);   /* retarget.c */

/* ---- Symbols placed by link.ld -------------------------------------------
 * These are not variables.  Each one is an ADDRESS the linker decided on,
 * and the only correct way to use it is to take its address.  Reading one as
 * if it held a value gives you whatever bytes happen to live there.
 */
extern uint32_t _sidata;      /* .data's home in FLASH                       */
extern uint32_t _sdata, _edata;
extern uint32_t _sbss,  _ebss;
extern uint32_t _estack;      /* top of RAM - the initial stack pointer      */
extern char     end;          /* first free byte after .bss - the heap start */
extern void (* const vectors[])(void);   /* startup.c, section .isr_vector   */

/* ---- Three globals that prove startup.c ran -------------------------------
 * data_witness is initialised, so it lives in .data: a copy in FLASH that
 * Reset_Handler must move into RAM.  If that copy does not happen, this
 * reads as something other than 0x00C0FFEE.
 *
 * bss_witness is initialised to zero, so the compiler puts it in .bss and
 * stores nothing at all in FLASH.  Its zero is a promise that Reset_Handler
 * must keep.
 *
 * rodata_witness is const, so it never leaves FLASH - nothing to copy, and
 * it costs zero bytes of RAM.  Compare the three in the size report.
 */
volatile uint32_t data_witness  = 0x00C0FFEEu;
volatile uint32_t bss_witness   = 0u;
const    uint32_t rodata_witness = 0x5AFEu;

static uint32_t frame_count;      /* also .bss - see Lab exercise 2 */

static void led_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;      /* clock the port FIRST, always */
    GPIOA->MODER &= ~(3u << (LED_PIN * 2));
    GPIOA->MODER |=  (1u << (LED_PIN * 2));  /* 01 = general purpose output  */
}

/* ---- A real millisecond ----------------------------------------------------
 * SysTick is a 24-bit down-counter built into every Cortex-M core.  Loaded
 * with (clock / 1000) - 1 it wraps once per millisecond and raises COUNTFLAG
 * each time; reading CTRL clears the flag again.  No interrupt is involved -
 * lesson 07 turns this same counter into one.  Until then, polling it is
 * honest, exact, and four lines long.
 */
static void systick_init(void)
{
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(uint32_t ms)
{
    SysTick->VAL = 0u;                       /* restart the current tick     */
    while (ms--) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) { }
    }
}

static void memory_report(void)
{
    uint32_t data_sz = (uint32_t)((uintptr_t)&_edata - (uintptr_t)&_sdata);
    uint32_t bss_sz  = (uint32_t)((uintptr_t)&_ebss  - (uintptr_t)&_sbss);

    printf("\n");
    printf("=== SOC3050 lesson 04 - Startup and the Linker Script ===\n");
    printf("    STM32C031C6, Cortex-M0+\n\n");

    printf("-- what the clock actually is --\n");
    printf("  SystemCoreClock : %lu Hz\n", (unsigned long)SystemCoreClock);
    printf("    (reset value is 12 MHz; SystemInit() in startup.c clears\n");
    printf("     RCC->CR HSIDIV to get 48. If this says 12000000, that\n");
    printf("     write did not take.)\n\n");

    printf("-- where link.ld put things --\n");
    printf("  vector table    : 0x%08lX  (must be the base of FLASH)\n",
           (unsigned long)(uintptr_t)vectors);
    printf("  .data in FLASH  : 0x%08lX  _sidata, the copy source\n",
           (unsigned long)(uintptr_t)&_sidata);
    printf("  .data in RAM    : 0x%08lX .. 0x%08lX  (%lu bytes)\n",
           (unsigned long)(uintptr_t)&_sdata,
           (unsigned long)(uintptr_t)&_edata, (unsigned long)data_sz);
    printf("  .bss  in RAM    : 0x%08lX .. 0x%08lX  (%lu bytes)\n",
           (unsigned long)(uintptr_t)&_sbss,
           (unsigned long)(uintptr_t)&_ebss,  (unsigned long)bss_sz);
    printf("  heap starts at  : 0x%08lX  (first byte after .bss)\n",
           (unsigned long)(uintptr_t)&end);
    printf("  stack top       : 0x%08lX  _estack, loaded into SP by\n",
           (unsigned long)(uintptr_t)&_estack);
    printf("                    the HARDWARE before any code runs\n\n");

    printf("-- did startup.c do its four jobs? --\n");
    printf("  .data copied    : 0x%08lX  %s  (expect 0x00C0FFEE)\n",
           (unsigned long)data_witness,
           data_witness == 0x00C0FFEEu ? "OK  " : "WRONG");
    printf("  .bss  zeroed    : 0x%08lX  %s  (expect 0x00000000)\n",
           (unsigned long)bss_witness,
           bss_witness == 0u ? "OK  " : "WRONG");
    printf("  .rodata in FLASH: 0x%08lX  at 0x%08lX - never copied,\n",
           (unsigned long)rodata_witness,
           (unsigned long)(uintptr_t)&rodata_witness);
    printf("                    so it costs no RAM at all\n\n");

    printf("  Every address above came from a linker symbol.\n");
    printf("  Nothing here is hardcoded. Edit link.ld and watch them move.\n\n");
    printf("LESSON 04 OK - LD4 on PA5 is the heartbeat; the bar is PB0..PB7\n\n");
}

/* ============================================================================
 *                              YOUR PART
 * ============================================================================
 *
 * The LED bar is eight LEDs on PB0..PB7, anode to the pin, cathode to GND.
 * A HIGH pin lights its LED, so ONE BYTE IS THE WHOLE BAR:
 *
 *        bit:   7 6 5 4 3 2 1 0
 *        pin:  PB7 . . . . . PB0
 *
 *   0b10000001  lights the two outer LEDs
 *   0b00001111  lights the right-hand half
 *   0xFF        lights all eight,  0x00 turns all eight off
 *
 * A PATTERN is a table of frames.  Each frame says which LEDs are on and
 * how many milliseconds to hold them.  The player walks the table, top to
 * bottom, and repeats it as many times as you ask.
 *
 * Things to try, easiest first:
 *
 *   1. Change a number in the ms column.  Rebuild, upload, watch.
 *   2. Change the 1s and 0s in an existing pattern.
 *   3. Add rows to a pattern, or delete some.
 *   4. Add a whole new pattern table and put it in the playlist in main().
 *   5. Write a pattern as a FUNCTION instead of a table, the way
 *      pattern_bounce() and pattern_counter() do below, using << and >>.
 *   6. Make a pattern whose timing changes as it runs - accelerate, or
 *      breathe - so the ms value is computed rather than stored.
 *   7. Look at the size report after each change: a const table costs
 *      FLASH only; anything you make writable costs RAM too (Lab Part 5).
 */

typedef struct {
    uint8_t  leds;      /* which LEDs are lit: bit 0 = PB0, bit 7 = PB7  */
    uint16_t ms;        /* how long to hold this frame, in milliseconds  */
} frame_t;

/* ---- Patterns as tables ---------------------------------------------------
 * `static const` puts these in FLASH (.rodata), where they cost no RAM.
 * Lab Part 5 shows what dropping the const would do.
 */

static const frame_t BLINK_ALL[] = {
    { 0b11111111, 500 },
    { 0b00000000, 500 },
};

static const frame_t ALTERNATE[] = {
    { 0b10101010, 250 },
    { 0b01010101, 250 },
};

static const frame_t KNIGHT_RIDER[] = {
    { 0b00000001,  80 },
    { 0b00000010,  80 },
    { 0b00000100,  80 },
    { 0b00001000,  80 },
    { 0b00010000,  80 },
    { 0b00100000,  80 },
    { 0b01000000,  80 },
    { 0b10000000,  80 },
    { 0b01000000,  80 },
    { 0b00100000,  80 },
    { 0b00010000,  80 },
    { 0b00001000,  80 },
    { 0b00000100,  80 },
    { 0b00000010,  80 },
};

static const frame_t INSIDE_OUT[] = {
    { 0b00011000, 120 },
    { 0b00100100, 120 },
    { 0b01000010, 120 },
    { 0b10000001, 120 },
    { 0b00000000, 240 },
};

static const frame_t SOS[] = {                 /* . . .  - - -  . . .        */
    { 0xFF, 150 }, { 0x00, 150 },
    { 0xFF, 150 }, { 0x00, 150 },
    { 0xFF, 150 }, { 0x00, 450 },
    { 0xFF, 450 }, { 0x00, 150 },
    { 0xFF, 450 }, { 0x00, 150 },
    { 0xFF, 450 }, { 0x00, 450 },
    { 0xFF, 150 }, { 0x00, 150 },
    { 0xFF, 150 }, { 0x00, 150 },
    { 0xFF, 150 }, { 0x00, 1200 },
};

/* ---- The bar itself -------------------------------------------------------
 * Same shape as led_init(): clock the port, then set eight MODER fields to
 * 01 (output).  Nothing else on GPIOB is used, so writing the whole ODR is
 * fine here; lesson 05 shows BSRR, the way to touch one pin atomically.
 */
static void ledbar_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;       /* clock the port FIRST, always */
    for (uint32_t pin = 0; pin < 8; pin++) {
        GPIOB->MODER &= ~(3u << (pin * 2));
        GPIOB->MODER |=  (1u << (pin * 2));
    }
}

/* Every frame goes through here, so this is also where the heartbeat lives:
 * LD4 toggles once per frame and a count is printed every 32nd frame.  The
 * count is a .bss global - Lab Part 2 asks where it starts. */
static void ledbar_show(uint8_t leds, uint16_t ms)
{
    GPIOB->ODR  = leds;
    GPIOA->ODR ^= (1u << LED_PIN);

    frame_count++;
    if ((frame_count & 31u) == 0u) {
        printf("frame %lu\n", (unsigned long)frame_count);
    }
    delay_ms(ms);
}

static void play(const char *name, const frame_t *p, uint32_t n, uint32_t repeats)
{
    printf("pattern %-14s %2lu frames x %lu\n",
           name, (unsigned long)n, (unsigned long)repeats);
    while (repeats--) {
        for (uint32_t i = 0; i < n; i++) {
            ledbar_show(p[i].leds, p[i].ms);
        }
    }
}

/* PLAY(TABLE, n) - play a table n times.  The macro works out how many
 * frames the table has, so adding or removing rows needs no other change. */
#define PLAY(table, repeats) \
    play(#table, table, sizeof(table) / sizeof((table)[0]), repeats)

/* ---- Patterns as code -----------------------------------------------------
 * Some patterns are shorter to compute than to write out.  These take the
 * frame time as an argument, so the playlist can run them fast or slow.
 */

/* One lit LED runs left, then right.  A shift moves it one place. */
static void pattern_bounce(uint16_t ms)
{
    printf("pattern bounce         computed, %u ms/frame\n", (unsigned)ms);
    for (uint8_t bit = 0x01; bit != 0x80; bit <<= 1) { ledbar_show(bit, ms); }
    for (uint8_t bit = 0x80; bit != 0x01; bit >>= 1) { ledbar_show(bit, ms); }
}

/* Fill from the right until all eight are lit, then empty from the left. */
static void pattern_fill(uint16_t ms)
{
    printf("pattern fill           computed, %u ms/frame\n", (unsigned)ms);
    uint8_t bar = 0;
    for (int i = 0; i < 8; i++) { bar = (uint8_t)((bar << 1) | 1u); ledbar_show(bar, ms); }
    for (int i = 0; i < 8; i++) { bar = (uint8_t)(bar << 1);        ledbar_show(bar, ms); }
}

/* Count 0..255 in binary.  Read the bar as a number: PB7 is 128, PB0 is 1. */
static void pattern_counter(uint16_t ms)
{
    printf("pattern counter        computed, %u ms/frame\n", (unsigned)ms);
    for (uint32_t n = 0; n < 256; n++) { ledbar_show((uint8_t)n, ms); }
}

/* Same bounce, but each pass is faster than the last: the ms value is
 * computed, not stored.  This is try-it number 6, done for you once. */
static void pattern_accelerate(void)
{
    printf("pattern accelerate     computed, 200 -> 25 ms/frame\n");
    for (uint16_t ms = 200; ms >= 25; ms /= 2) {
        for (uint8_t bit = 0x01; bit != 0; bit <<= 1) { ledbar_show(bit, ms); }
    }
}

int main(void)
{
    led_init();
    ledbar_init();
    uart2_init(SystemCoreClock, BAUD);
    systick_init();

    memory_report();

    /* ---- The playlist ------------------------------------------------------
     * Runs top to bottom, forever.  Reorder it, repeat things, drop things,
     * add your own.  Every entry is either PLAY(a table, times) or a call
     * to a pattern function.
     */
    for (;;) {
        PLAY(BLINK_ALL,    3);
        PLAY(ALTERNATE,    4);
        PLAY(KNIGHT_RIDER, 3);
        PLAY(INSIDE_OUT,   3);
        pattern_bounce(60);
        pattern_fill(100);
        pattern_accelerate();
        pattern_counter(100);
        PLAY(SOS,          1);
    }
}
