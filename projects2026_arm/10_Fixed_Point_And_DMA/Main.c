/*
 * Main.c - SOC3050 lesson 10: Fixed Point and DMA
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz, NO FPU
 *
 * How fast is maths on a chip with no floating-point unit?
 *
 * PART 1 - THE ARENA.  The same jobs - add, multiply, divide, sine, rotate a
 * point, a PI controller step, a 16-tap filter - written in int32, Q15,
 * Q16.16 and float, raced against each other.  SysTick counts CPU cycles
 * (one count per clock at 48 MHz); the leaderboard prints cycles per
 * iteration, how many times slower than the winner, and what each format
 * costs in flash.
 *
 * PART 2 - DMA.  The ADC scans the joystick (PA0, PA1) and the knob (PA4)
 * forever and DMA1 drops each result into RAM - the CPU never waits for a
 * conversion.  Then a memory-to-memory DMA copy races memcpy.  If the DMA
 * never moves (Wokwi's DMA model is unverified) the program says so and falls
 * back to polled adc_read().
 *
 * Then it runs live: the joystick's X goes through the 16-tap filter in Q15
 * and in float, side by side, at 100 Hz.  On the OLED: the FIR race and the
 * live inputs.
 *
 *   button A (PB4, key a)   run the arena again
 *   button B (PB5, key b)   switch the inputs between DMA and polled reads
 *
 * This lesson does NOT link the RTOS: it owns SysTick itself, as a free-running
 * 24-bit cycle counter with no interrupt.
 *
 * Build:     build.bat      (LIBS = retarget adc i2c oled)
 * Simulate:  simulate.bat, paste diagram.json, upload Main.elf.
 * Host:      host\run.bat   (accuracy of every format, and an M0+ cycle model)
 */

#include <stdio.h>
#include <string.h>
#include "stm32c031xx.h"
#include "gpio.h"
#include "retarget.h"
#include "adc.h"
#include "i2c.h"
#include "oled.h"
#include "fix.h"
#include "bench.h"
#include "dma.h"

#define BAUD        115200u
#define ARENA_SEED  2026u          /* fixed, so host/m0sim.py sees the same data */
#define LIVE_HZ     100u

#define BTN_A_PIN   4u             /* PB4 */
#define BTN_B_PIN   5u             /* PB5 */
#define LED_PIN     5u             /* PA5, LD4 */

/* Wokwi's documentation for this board lists DMA as NOT simulated.  The DMA
 * code still runs - it detects that nothing moved and falls back to polling -
 * but if your simulator instead FAULTS on the first DMA register access, the
 * HardFault handler below says so; set this to 0 to skip DMA entirely. */
#define USE_DMA     1

/* What the program was doing, for the HardFault message. */
static const char *volatile stage = "start-up";

/* A fault while touching a peripheral the simulator does not model is worth
 * one clear line rather than a silent hang.  Lesson 11 does faults properly. */
void HardFault_Handler(void)
{
    printf("\n!! HardFault during: %s\n", stage);
    GPIOA->BSRR = 1u << 5;                      /* LD4 on: parked here */
    for (;;) { }
}

/* ============================================================================
 *  SysTick as a cycle counter
 *
 *  SysTick counts DOWN from LOAD to 0 and reloads.  CLKSOURCE = 1 clocks it
 *  from the CPU clock, so one count is one cycle: 20.8 ns.  24 bits wrap
 *  every 16.7 M cycles = 349 ms, which is longer than anything timed here.
 *  TICKINT stays 0: no interrupt, nothing perturbing what we measure.
 * ============================================================================ */
static int systick_ok;

static void cycles_init(void)
{
    SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;              /* 0xFFFFFF: full range */
    SysTick->VAL  = 0;                                    /* any write clears it  */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Does it actually count?  In a simulator, find out rather than assume:
     * lesson 09 met one where SysTick was enabled and frozen. */
    uint32_t a = SysTick->VAL;
    for (volatile uint32_t i = 0; i < 200u; i++) { }
    systick_ok = (SysTick->VAL != a);
}

static inline uint32_t cycles_now(void) { return SysTick->VAL; }

static inline uint32_t cycles_since(uint32_t t0)          /* down-counter! */
{
    return (t0 - SysTick->VAL) & SysTick_LOAD_RELOAD_Msk;
}

/* Milliseconds since boot, built from SysTick.  Must be called more often
 * than every 349 ms or wraps are lost - the main loop calls it every pass. */
static uint32_t ms_now(void)
{
    static uint32_t last, acc, ms;
    if (!systick_ok) {                                    /* fallback: a rough 1 ms */
        for (volatile uint32_t i = 0; i < 4000u; i++) { }
        return ++ms;
    }
    uint32_t v = SysTick->VAL;
    acc += (last - v) & SysTick_LOAD_RELOAD_Msk;
    last = v;
    ms  += acc / 48000u;
    acc %= 48000u;
    return ms;
}

/* ============================================================================
 *  The arena
 * ============================================================================ */
static volatile uint32_t sink;          /* where checksums go, so nothing is dead code */

static uint32_t time_call(bench_fn fn, uint32_t n)
{
    uint32_t t0 = cycles_now();
    sink = fn(n);
    return cycles_since(t0);
}

/* Cycles per iteration, x10 (one decimal).  Timing fn(64) and subtracting
 * fn(0) cancels everything that is not the loop body: the call, the return,
 * reading SysTick itself.  Best of three runs, so a one-off stall does not
 * count. */
static uint32_t cost_x10(bench_fn fn, uint32_t *checksum)
{
    uint32_t best = 0xFFFFFFFFu;
    for (int r = 0; r < 3; r++) {
        uint32_t whole = time_call(fn, BENCH_N);
        *checksum = sink;
        uint32_t empty = time_call(fn, 0);
        uint32_t d = whole > empty ? whole - empty : 0;
        if (d < best) { best = d; }
    }
    return (best * 10u + BENCH_N / 2u) / BENCH_N;
}

static uint32_t result_x10[32];
static uint32_t fir_x10[3];             /* int32, Q15, float - for the OLED */

/* What each format drags into flash.  Measured, not computed: the sizes of
 * the library routines in THIS lesson's Main.elf, added up by
 *     python host/flashcost.py Main.elf      (arm-none-eabi-nm --size-sort -S)
 * They go stale the moment the code changes - re-run it and update them. */
static const struct { const char *what; uint16_t bytes; } flash_cost[] = {
    { "int32 + - *   built-in instructions",                  0 },
    { "Q15 sine      table + q15_sin()",                    198 },
    { "Q16.16 mul    __aeabi_lmul (32x32->64)",              78 },
    { "Q16.16 div    __aeabi_ldivmod, __divdi3 ...",        604 },
    { "int32 div     __aeabi_idiv (no divide instruction)", 470 },
    { "float         + - * / compare convert (soft float)", 4292 },
    { "libm sinf     + kernels and range reduction",       4742 },
};

static void print_x10(uint32_t v) { printf("%5lu.%lu", (unsigned long)(v / 10u), (unsigned long)(v % 10u)); }

static void run_arena(void)
{
    if (!systick_ok) {
        printf("\nArena skipped: SysTick does not count here, so nothing can be timed.\n");
        return;
    }
    printf("\n== THE ARENA ==  seed %lu, %lu iterations, best of 3, DMA scan %s\n",
           (unsigned long)ARENA_SEED, (unsigned long)BENCH_N, scan_running() ? "RUNNING" : "off");
    printf("  job    format      cyc/iter   net    x winner   checksum\n");

    /* Measure everything first, print second: a row's "x winner" needs the
     * fastest format for its job, which may come later in the table. */
    static uint32_t checksum[32];
    uint32_t rows = bench_count < 32u ? bench_count : 32u;
    bench_seed(ARENA_SEED);
    for (uint32_t i = 0; i < rows; i++) {
        result_x10[i] = cost_x10(bench_table[i].fn, &checksum[i]);
    }

    uint32_t base = result_x10[0];                     /* row 0 is the bare loop */
    for (uint32_t i = 0; i < rows; i++) {
        uint32_t win = result_x10[i];
        for (uint32_t j = 1; j < rows; j++) {
            if (strcmp(bench_table[j].work, bench_table[i].work) == 0 && result_x10[j] < win) { win = result_x10[j]; }
        }
        printf("  %-6s %-10s ", bench_table[i].work, bench_table[i].fmt);
        print_x10(result_x10[i]);
        if (i == 0) {
            printf("      -       -    ");
        } else {
            print_x10(result_x10[i] > base ? result_x10[i] - base : 0);
            uint32_t ratio_x10 = win ? (result_x10[i] * 10u + win / 2u) / win : 0;
            printf("  ");
            print_x10(ratio_x10);
        }
        printf("   %08lX\n", (unsigned long)checksum[i]);
        if (strcmp(bench_table[i].work, "fir16") == 0) {
            uint32_t slot = strcmp(bench_table[i].fmt, "int32") == 0 ? 0u
                          : strcmp(bench_table[i].fmt, "Q15")   == 0 ? 1u : 2u;
            fir_x10[slot] = result_x10[i];
        }
    }
    printf("  (net = minus the baseline loop; x winner = cycles / fastest format for that job)\n");

    /* The budget arithmetic: is float fast enough?  Usually, yes. */
    printf("\n-- is float fast enough? --\n");
    for (uint32_t i = 0; i < bench_count; i++) {
        uint32_t rate = 0;
        if (strcmp(bench_table[i].work, "pi") == 0)    { rate = 500u;  }
        if (strcmp(bench_table[i].work, "fir16") == 0) { rate = 1000u; }
        if (!rate) { continue; }
        uint32_t c = result_x10[i] / 10u;
        uint32_t pct_x100 = result_x10[i] * rate / 48000u;   /* % of 48 M cycles/s, x100; < 2^32 */
        printf("  %-5s %-7s %6lu cycles x %4lu Hz = %3lu.%02lu %% of the CPU\n",
               bench_table[i].work, bench_table[i].fmt, (unsigned long)c, (unsigned long)rate,
               (unsigned long)(pct_x100 / 100u), (unsigned long)(pct_x100 % 100u));
    }

    printf("\n-- what each format costs in flash (nm, this build) --\n");
    for (uint32_t i = 0; i < sizeof flash_cost / sizeof flash_cost[0]; i++) {
        printf("  %5u B  %s\n", (unsigned)flash_cost[i].bytes, flash_cost[i].what);
    }
}

/* ============================================================================
 *  DMA: the ADC scan, and a memory-to-memory copy
 * ============================================================================ */
static uint32_t copy_src[256], copy_dst[256];          /* 1 KB each */
static uint32_t read_cost_dma, read_cost_poll;         /* cycles per 3-channel read */
static uint32_t overruns;

static uint32_t time_scan_read(void)
{
    uint16_t v[SCAN_CH];
    uint32_t t0 = cycles_now();
    for (int i = 0; i < 16; i++) { (void)scan_read(v, &overruns); }
    return (cycles_since(t0) + 8u) / 16u;
}

static void print_dma_registers(void)
{
    printf("  DMA1_Channel1->CCR   = 0x%08lX  (expect 0x000025A1)\n", (unsigned long)DMA1_Channel1->CCR);
    printf("  DMA1_Channel1->CNDTR = %lu           (counts 3,2,1 - the NEXT slot to fill)\n", (unsigned long)DMA1_Channel1->CNDTR);
    printf("  DMA1_Channel1->CPAR  = 0x%08lX  (&ADC1->DR = 0x%08lX)\n", (unsigned long)DMA1_Channel1->CPAR, (unsigned long)(uint32_t)&ADC1->DR);
    printf("  DMA1_Channel1->CMAR  = 0x%08lX  (scan_buf  = 0x%08lX)\n", (unsigned long)DMA1_Channel1->CMAR, (unsigned long)(uint32_t)scan_buf);
    printf("  DMAMUX1_Channel0->CCR= 0x%08lX  (request 5 = ADC1)\n", (unsigned long)DMAMUX1_Channel0->CCR);
    printf("  ADC1->CFGR1          = 0x%08lX  (expect 0x00002003: CONT DMACFG DMAEN)\n", (unsigned long)ADC1->CFGR1);
    printf("  ADC1->CHSELR         = 0x%08lX  (expect 0x00000013: IN4 IN1 IN0)\n", (unsigned long)ADC1->CHSELR);
}

static __attribute__((unused)) void dma_demo(void)   /* unused when USE_DMA = 0 */
{
    printf("\n== DMA ==\n");
    int rc = scan_start();
    print_dma_registers();
    if (rc == 0) {
        printf("  scan: DMA RUNNING - scan_buf = %u %u %u\n",
               scan_buf[0], scan_buf[1], scan_buf[2]);
    } else {
        printf("  scan: DMA NEVER MOVED - buffer still 0xFFFF after the time limit.\n"
               "        EXPECTED IN WOKWI: its C031C6 model does not implement DMA (docs.wokwi.com).\n"
               "        On a real Nucleo this line says RUNNING.  Falling back to POLLED adc_read().\n");
    }

    if (systick_ok) {
        /* What does reading three inputs cost the CPU, each way? */
        if (scan_running()) {
            read_cost_dma = time_scan_read();
            scan_stop();
            read_cost_poll = time_scan_read();
            (void)scan_start();
        } else {
            read_cost_poll = time_scan_read();
        }
        printf("  one 3-channel read: DMA %lu cycles, polled %lu cycles\n",
               (unsigned long)read_cost_dma, (unsigned long)read_cost_poll);

        /* memcpy vs a DMA copy, 1 KB */
        for (uint32_t i = 0; i < 256u; i++) { copy_src[i] = i * 0x01010101u; }
        memset(copy_dst, 0, sizeof copy_dst);
        uint32_t t0 = cycles_now();
        memcpy(copy_dst, copy_src, sizeof copy_dst);
        uint32_t t_memcpy = cycles_since(t0);

        memset(copy_dst, 0, sizeof copy_dst);
        uint32_t spins = 0;
        t0 = cycles_now();
        int crc = dma_copy(copy_dst, copy_src, 256u, &spins);
        uint32_t t_dma = cycles_since(t0);
        int same = memcmp(copy_dst, copy_src, sizeof copy_dst) == 0;
        printf("  1 KB copy: memcpy %lu cycles; DMA %lu cycles (%s, data %s), CPU polled %lu times meanwhile\n",
               (unsigned long)t_memcpy, (unsigned long)t_dma,
               crc == 0 ? "completed" : "NEVER COMPLETED", same ? "correct" : "WRONG",
               (unsigned long)spins);
    }
}

/* ============================================================================
 *  The OLED: the FIR race and the live inputs
 * ============================================================================ */
static int oled_ok;

static void draw_race(void)
{
    static const char *name[3] = { "int", "Q15", "flt" };
    uint32_t max = 1;
    for (int i = 0; i < 3; i++) { if (fir_x10[i] > max) { max = fir_x10[i]; } }
    oled_fill_rect(0, 0, OLED_W, 40, OLED_OFF);
    oled_text(0, 0, "FIR16 race, cycles", OLED_ON);
    for (int i = 0; i < 3; i++) {
        int y = 11 + i * 10;
        int w = (int)(fir_x10[i] * 70u / max);
        oled_text(0, y, name[i], OLED_ON);
        oled_fill_rect(20, y, w > 0 ? w : 1, 7, OLED_ON);
        oled_printf(94, y, "%lu", (unsigned long)(fir_x10[i] / 10u));
    }
}

static void draw_live(const uint16_t v[SCAN_CH], int32_t fq, int32_t ff)
{
    oled_fill_rect(0, 42, OLED_W, 22, OLED_OFF);
    oled_printf(0, 42, "%s x%4u y%4u k%4u", scan_running() ? "DMA " : "POLL", v[0], v[1], v[2]);
    oled_printf(0, 54, "fir q15%6ld f%6ld", (long)fq, (long)ff);
}

/* ============================================================================ */
int main(void)
{
    uart2_init(SystemCoreClock, BAUD);
    cycles_init();

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    pin_mode(GPIOA, LED_PIN, MODE_OUTPUT);
    pin_mode(GPIOB, BTN_A_PIN, MODE_INPUT);  pin_pull(GPIOB, BTN_A_PIN, PULL_UP);
    pin_mode(GPIOB, BTN_B_PIN, MODE_INPUT);  pin_pull(GPIOB, BTN_B_PIN, PULL_UP);
    pin_mode(GPIOA, 0u, MODE_ANALOG);        /* joystick HORZ - adc_init() does PA4 */
    pin_mode(GPIOA, 1u, MODE_ANALOG);        /* joystick VERT                       */
    int adc_rc = adc_init();

    i2c_init();
    oled_ok = (oled_init() == 0);

    printf("\n\n== SOC3050 lesson 10: Fixed Point and DMA ==\n");
    printf("Cortex-M0+ at %lu Hz, no FPU.  SysTick cycle counter: %s\n",
           (unsigned long)SystemCoreClock, systick_ok ? "counting" : "FROZEN - no timing possible");
    printf("ADC: %s.  OLED: %s\n", adc_rc == 0 ? "ready" : "FAILED to start",
           oled_ok ? "found at 0x3C" : "not answering");

    stage = "the arena";
    run_arena();
#if USE_DMA
    stage = "the DMA demo (DMA1/DMAMUX registers)";
    dma_demo();
#else
    printf("\nDMA skipped (USE_DMA = 0): inputs are read by polled adc_read().\n");
#endif
    stage = "the live loop";
    printf("\nLive: joystick X through the 16-tap filter, Q15 and float.  A = arena, B = DMA/poll.\n");

    if (oled_ok) { oled_clear(); draw_race(); }

    /* ---- live: 100 Hz ------------------------------------------------------ */
    static q15_t ring_q[BENCH_N];
    static float ring_f[BENCH_N];
    uint32_t head = 0, next = ms_now(), last_print = 0, last_oled = 0, last_led = 0;
    uint8_t  a_prev = 1, b_prev = 1, a_last = 1, b_last = 1;
    uint32_t live_read_cost = 0;

    for (;;) {
        uint32_t now = ms_now();
        if ((int32_t)(now - next) < 0) { continue; }
        next += 1000u / LIVE_HZ;

        uint16_t v[SCAN_CH] = { 0, 0, 0 };
        uint32_t t0 = cycles_now();
        (void)scan_read(v, &overruns);
        live_read_cost = cycles_since(t0);

        /* X into both filters.  Q15: 12-bit, centred, shifted to the top of 16. */
        head = (head + 1u) & (BENCH_N - 1u);
        ring_q[(head + FIR_TAPS - 1u) & (BENCH_N - 1u)] = (q15_t)(((int32_t)v[0] - 2048) * 16);
        ring_f[(head + FIR_TAPS - 1u) & (BENCH_N - 1u)] = (float)((int32_t)v[0] - 2048) / 2048.0f;
        int32_t fq = fir_q15(ring_q, head);                       /* -32768..32767 */
        int32_t ff = (int32_t)(fir_float(ring_f, head) * 32768.0f);  /* same scale  */

        /* buttons: act on the press edge, two equal samples 10 ms apart */
        uint8_t a = (uint8_t)pin_read(GPIOB, BTN_A_PIN), b = (uint8_t)pin_read(GPIOB, BTN_B_PIN);
        if (a == a_prev && a != a_last) { a_last = a; if (!a) { stage = "the arena"; run_arena(); stage = "the live loop"; if (oled_ok) { draw_race(); } next = ms_now(); } }
        if (b == b_prev && b != b_last) {
            b_last = b;
            if (!b && USE_DMA) {
                if (scan_running()) { scan_stop(); printf("\ninputs: POLLED adc_read()\n"); }
                else { printf("\ninputs: %s\n", scan_start() == 0 ? "DMA scan" : "DMA never moved - still POLLED"); }
            }
        }
        a_prev = a; b_prev = b;

        if (now - last_print >= 2000u) {
            last_print = now;
            printf("x %4u y %4u knob %4u | fir16 q15 %6ld float %6ld | %s read %lu cyc | CNDTR %lu ovr %lu\n",
                   v[0], v[1], v[2], (long)fq, (long)ff, scan_running() ? "DMA " : "POLL",
                   (unsigned long)live_read_cost, (unsigned long)DMA1_Channel1->CNDTR, (unsigned long)overruns);
        }
        if (oled_ok && now - last_oled >= 200u) {
            last_oled = now;
            draw_live(v, fq, ff);
            (void)oled_flush();
            next = ms_now();          /* a flush takes milliseconds: do not try to catch up */
        }
        if (now - last_led >= 500u) { last_led = now; GPIOA->ODR ^= 1u << LED_PIN; }
    }
}
