/*
 * Main.c - SOC3050 lesson 11: Faults, Watchdog, Power - the Crash Lab
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * "Crash it on purpose - then make it survive."
 *
 * A menu of deliberate faults.  Each one is caught by a HardFault reporter
 * (crash.c), written into RAM that survives a reset (.noinit, link.ld), and
 * explained after the reboot by a decoder that reads the evidence the M0+
 * leaves behind (diag.c).  A watchdog (wdog.c), fed only when every job has
 * checked in, brings back a system that hangs - or one that merely LOOKS
 * alive.  The idle loop sleeps with WFI and measures how long it slept, which
 * is the CPU load.
 *
 * WHICH watchdog matters, because Wokwi models only some of them
 * (docs.wokwi.com/parts/board-st-nucleo-c031c6): IWDG not implemented, WWDG
 * "implemented, not tested yet", PWR and DBG not implemented, RCC partial.
 * So a SOFT watchdog - a counter in SysTick - is armed at boot and is the one
 * you will see bite in Wokwi.  `arm wwdg` and `arm iwdg` add the hardware
 * dogs; on silicon the IWDG is the one to ship.
 *
 *   button A          next experiment             (OLED shows the menu)
 *   button B          run it
 *   serial            help, list, crash N, mystery, guess N, reveal, stats,
 *                     wdt MS, arm wwdg|iwdg, wdmode hb|loop|isr,
 *                     sleep on|off, report, clear, reset
 *
 * No RTOS: a superloop of three jobs on a 1 kHz SysTick.  That keeps every
 * fault on the main stack and every cycle of idle time measurable.  (Lesson
 * 07's kernel has no HardFault handler of its own, so crash.c would work
 * under it unchanged - with the frame on PSP; diag.c decodes both.)
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, paste diagram.json, upload Main.elf.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32c031xx.h"
#include "retarget.h"
#include "gpio.h"
#include "pad.h"
#include "i2c.h"
#include "oled.h"
#include "diag.h"
#include "wdog.h"
#include "crash.h"

#define BAUD           115200u
#define LED_PIN        5u              /* PA5 = LD4                            */

/* diag.h keeps its own copy of the reset-flag bits so the host can compile
 * it.  Here the compiler checks that copy against ST's header. */
_Static_assert(RST_OBL  == RCC_CSR2_OBLRSTF,  "RCC_CSR2 bit");
_Static_assert(RST_PIN  == RCC_CSR2_PINRSTF,  "RCC_CSR2 bit");
_Static_assert(RST_PWR  == RCC_CSR2_PWRRSTF,  "RCC_CSR2 bit");
_Static_assert(RST_SFT  == RCC_CSR2_SFTRSTF,  "RCC_CSR2 bit");
_Static_assert(RST_IWDG == RCC_CSR2_IWDGRSTF, "RCC_CSR2 bit");
_Static_assert(RST_WWDG == RCC_CSR2_WWDGRSTF, "RCC_CSR2 bit");
_Static_assert(RST_LPWR == RCC_CSR2_LPWRRSTF, "RCC_CSR2 bit");
_Static_assert(xPSR_T_Pos == 24, "diag.c tests xPSR bit 24");

/* ============================================================================
 *  The menu
 * ============================================================================ */
static const struct { const char *name, *what; } menu[X_COUNT] = {
    [X_NULL_READ]   = { "null read",      "read *(uint32_t *)0 - legal here, so no fault" },
    [X_UNALIGNED]   = { "unaligned LDR",  "32-bit load from an odd address" },
    [X_WILD_READ]   = { "wild read",      "load from 0x60000000 - nothing mapped" },
    [X_FLASH_STORE] = { "flash store",    "store into flash - fault, or ignored?" },
    [X_NULL_CALL]   = { "NULL callback",  "call a function pointer that is 0" },
    [X_EXEC_PERIPH] = { "exec periph",    "jump into RCC's registers" },
    [X_UDF]         = { "udf",            "a permanently undefined instruction" },
    [X_BKPT]        = { "bkpt",           "breakpoint, no debugger attached" },
    [X_STACK]       = { "stack overflow", "recurse through the stack floor" },
    [X_HANG]        = { "hang",           "for (;;) with interrupts on" },
    [X_HANG_NOIRQ]  = { "hang, irq off",  "for (;;) with interrupts OFF" },
    [X_STARVE]      = { "starve",         "the screen job stops checking in" },
    [X_PACKET]      = { "packet bug",     "a real-looking bug: find the line" },
    [X_DIV0]        = { "div by zero",    "100 / 0 - is there a trap?" },
    [X_MYSTERY]     = { "MYSTERY",        "a random fault, cause hidden: guess" },
};

/* HardFault experiments the mystery picks from (all fault on silicon) */
static const uint8_t mystery_pool[] = { X_UNALIGNED, X_WILD_READ, X_NULL_CALL,
                                        X_EXEC_PERIPH, X_UDF, X_BKPT, X_PACKET };

static const char *const cause_short[CAUSE_COUNT] = {
    "unknown", "Thumb bit", "bad fetch", "unaligned", "unmapped",
    "read-only", "undefined", "bkpt", "svc"
};

/* ============================================================================
 *  Time, and the heartbeat
 * ============================================================================ */
volatile uint32_t ms_now;                  /* crash.c reads it too            */

enum { WD_HEARTBEAT = 0, WD_LOOP, WD_ISR };
static const char *const wd_names[] = { "heartbeat", "every loop", "SysTick ISR" };
static volatile uint8_t wd_mode = WD_HEARTBEAT;

enum { HB_BLINK = 1u << 0, HB_INPUT = 1u << 1, HB_SCREEN = 1u << 2 };
static const char *const hb_names[] = { "blink", "input", "screen" };
static volatile heartbeat_t hb = { HB_BLINK | HB_INPUT | HB_SCREEN, 0 };

static uint32_t last_feed, worst_gap, feeds;

/* A check-in also stamps the time into .noinit, so that after a watchdog
 * reset the next boot can say which job went quiet, and for how long. */
static void checkin(uint32_t bit, unsigned job)
{
    hb_checkin(&hb, bit);
    crash_rec.job_ms[job] = ms_now;
}

/* SysTick: the 1 ms clock, and the soft dog's clock.  In wd_mode WD_ISR it
 * also feeds the dogs - the classic mistake.  Lab Part 4 finds out what that
 * cannot catch. */
void SysTick_Handler(void)
{
    ms_now++;
    if (wd_mode == WD_ISR) { dog_feed(); }
    if (dog_soft_tick()) {
        /* The soft dog bit.  Interrupts are the only code still guaranteed to
         * run in a hung main loop, so the reset is issued from here. */
        crash_rec.uptime_ms = ms_now;
        crash_reset(RW_SOFTDOG);
    }
}

static void feed(void)
{
    uint32_t gap = ms_now - last_feed;
    if (gap > worst_gap && feeds > 0u) { worst_gap = gap; }
    last_feed = ms_now;
    feeds++;
    dog_feed();
}

static const char *dog_list(uint32_t d, char *buf, unsigned len)
{
    snprintf(buf, len, "%s%s%s", (d & DOG_SOFT) ? "soft " : "", (d & DOG_WWDG) ? "WWDG " : "",
             (d & DOG_IWDG) ? "IWDG " : "");
    return buf[0] ? buf : "none ";
}

/* ============================================================================
 *  Serial input: an interrupt into a small ring, so a fast typist (or a
 *  paste) is not lost while the loop sleeps or draws.
 * ============================================================================ */
static volatile uint8_t  rx_buf[64];
static volatile uint8_t  rx_head, rx_tail;

void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_ORE) { USART2->ICR = USART_ICR_ORECF; }
    while (USART2->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t c = (uint8_t)USART2->RDR;
        uint8_t next = (uint8_t)((rx_head + 1u) % sizeof rx_buf);
        if (next != rx_tail) { rx_buf[rx_head] = c; rx_head = next; }
    }
}

static int rx_get(void)
{
    if (rx_tail == rx_head) { return -1; }
    int c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) % sizeof rx_buf);
    return c;
}

/* ============================================================================
 *  Power: sleep until the next tick, and count how long we slept
 * ============================================================================
 * Interrupts are DISABLED around WFI on purpose.  WFI still wakes on a
 * pending interrupt with PRIMASK set (Armv6-M), but the handler cannot run
 * until __enable_irq() - so the second SysTick->VAL read happens FIRST, and
 * the measured time is pure sleep, with no handler inside it.
 *
 * It also closes a race: with interrupts on, a tick arriving between the
 * `ms_now == t` test and WFI would be handled first, and WFI would then sleep
 * through a whole extra millisecond.  With PRIMASK set, that tick stays
 * pending and WFI returns at once.
 *
 * sleep_on = 0 spins on the same condition instead.  Same measurement, same
 * load figure - but on silicon the core never stops its clock.  What the
 * difference costs in current is what a simulator cannot show you.
 *
 * SLEEP_WITH_PRIMASK 0 is the fallback if a simulator's WFI does NOT wake on
 * an interrupt that PRIMASK holds back (the board would freeze right after the
 * boot banner).  The idle figure then includes the few cycles of the handlers
 * that woke it - well under 1% - and the race above returns. */
#define SLEEP_WITH_PRIMASK 1

static uint8_t  sleep_on = 1;
static uint32_t idle_cycles, wakeups, wakeups_ps;
static uint32_t load_pm = 0;             /* CPU load, per mille, last second  */

static void idle_until_next_tick(void)
{
    uint32_t t = ms_now;
    while (ms_now == t) {
#if SLEEP_WITH_PRIMASK
        __disable_irq();
        if (ms_now == t) {
            uint32_t v0 = SysTick->VAL;
            if (sleep_on) {
                __WFI();                                    /* sleep: SLEEPDEEP = 0 */
            } else {
                while (!(SCB->ICSR & (SCB_ICSR_PENDSTSET_Msk)) &&
                       !(USART2->ISR & USART_ISR_RXNE_RXFNE)) { }
            }
            uint32_t v1 = SysTick->VAL;
            idle_cycles += systick_elapsed(v0, v1, SysTick->LOAD);
            wakeups++;
        }
        __enable_irq();                                     /* handlers run here */
#else
        uint32_t v0 = SysTick->VAL;
        if (sleep_on) { __WFI(); }                          /* handlers run inside */
        else { while (ms_now == t && rx_head == rx_tail) { } }
        uint32_t v1 = SysTick->VAL;
        idle_cycles += systick_elapsed(v0, v1, SysTick->LOAD);
        wakeups++;
#endif
    }
}

/* ============================================================================
 *  The three jobs
 * ============================================================================ */
static pad_t   pad;
static uint8_t sel = X_UNALIGNED;
static uint8_t starving;                 /* X_STARVE                          */
static int     oled_ok;
static uint32_t csr2_at_boot;
static char    line[40];
static uint8_t line_len;

static void command(char *cmd);
static void run(unsigned n);

/* 1. Blink LD4 at 2 Hz: the system's "I am alive" - which, Lab Part 4 shows,
 *    proves less than it seems. */
static void job_blink(void)
{
    static uint32_t next;
    if ((int32_t)(ms_now - next) < 0) { return; }
    next += 250u;
    GPIOA->ODR ^= 1u << LED_PIN;
    checkin(HB_BLINK, 0);
}

/* 2. Buttons at 100 Hz, and the serial line. */
static void job_input(void)
{
    static uint32_t next;
    if ((int32_t)(ms_now - next) < 0) { return; }
    next += 10u;

    pad_read(&pad);
    if (pad.pressed & PAD_A) { sel = (uint8_t)((sel + 1u) % X_COUNT); }
    if (pad.pressed & PAD_B) { printf("\n[button B] %u %s\n", sel, menu[sel].name); run(sel); }

    int c;
    while ((c = rx_get()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (line_len) { printf("\n"); line[line_len] = '\0'; command(line); line_len = 0; }
        } else if ((c == 8 || c == 127) && line_len) {
            line_len--; printf("\b \b");
        } else if (c >= ' ' && line_len < sizeof line - 1u) {
            line[line_len++] = (char)c; putchar(c);
        }
    }
    checkin(HB_INPUT, 1);
}

/* 3. The screen at 10 Hz.  When "starving" it waits for a frame-ready event
 *    that never comes - returning without drawing and WITHOUT checking in.
 *    LD4 still blinks; the serial port still answers.  Only the heartbeat
 *    bitmap notices that one job has died. */
static void job_screen(void)
{
    static uint32_t next;
    if ((int32_t)(ms_now - next) < 0) { return; }
    next += 100u;
    if (starving) { return; }

    if (oled_ok) {
        const crash_rec_t *r = &crash_rec;
        oled_clear();
        oled_printf(0, 0, "CRASH LAB   boot %lu", (unsigned long)r->boots);
        oled_hline(0, 9, 128, OLED_ON);
        oled_printf(0, 11, "%2u %s", sel, menu[sel].name);
        oled_text(0, 20, "A next   B run it", OLED_ON);
        if (r->kind == REC_HARDFAULT && !r->hidden) {
            insn_t ins;
            static char w[120];
            cause_t c = fault_classify(&r->regs, (int)r->code_ok, r->hw[0], r->hw[1], &ins, w, sizeof w);
            oled_printf(0, 29, "last: %s", cause_short[c]);
            oled_printf(0, 38, "pc %08lX", (unsigned long)r->regs.pc);
        } else if (r->kind == REC_HARDFAULT) {
            oled_text(0, 29, "last: ??? (guess N)", OLED_ON);
            oled_printf(0, 38, "pc %08lX", (unsigned long)r->regs.pc);
        } else if (r->kind == REC_STACK) {
            oled_text(0, 29, "last: stack overflow", OLED_ON);
        } else {
            oled_text(0, 29, "last: no crash", OLED_ON);
        }
        oled_printf(0, 47, "rst %s", reset_cause(csr2_at_boot));
        oled_printf(0, 56, "load %lu.%lu%% wd %lums  %lu/%lu",
                    (unsigned long)(load_pm / 10u), (unsigned long)(load_pm % 10u),
                    (unsigned long)dog_ms(DOG_SOFT), (unsigned long)r->right, (unsigned long)r->tries);
        oled_flush();
    }
    checkin(HB_SCREEN, 2);
}

/* ============================================================================
 *  The report - printed after the reboot, on a healthy system
 * ============================================================================ */
static void print_regs(const regs_t *g)
{
    static const char *const n[13] = { "r0", "r1", "r2", "r3", "r4", "r5", "r6",
                                       "r7", "r8", "r9", "r10", "r11", "r12" };
    for (unsigned i = 0; i < 13u; i++) {
        printf("  %-3s %08lX%s", n[i], (unsigned long)g->r[i], (i % 4u == 3u) ? "\n" : "");
    }
    printf("\n  sp  %08lX  lr  %08lX  pc  %08lX  xPSR %08lX\n",
           (unsigned long)g->sp, (unsigned long)g->lr, (unsigned long)g->pc, (unsigned long)g->xpsr);
    printf("  EXC_RETURN %08lX  = %s\n", (unsigned long)g->exc_return, exc_return_text(g->exc_return));
}

static void report(void)
{
    const crash_rec_t *r = &crash_rec;
    if (r->kind == REC_NONE) { printf("no crash recorded since the record was cleared\n"); return; }

    printf("---- last crash: #%lu, at %lu ms of uptime ----\n",
           (unsigned long)r->crashes, (unsigned long)r->uptime_ms);

    if (r->kind == REC_STACK) {
        printf("  STACK OVERFLOW: the canary words at _stack_floor (0x%08lX) were overwritten.\n"
               "  Nothing faulted - the stack simply grew into the heap.  Experiment: %s\n",
               (unsigned long)r->regs.sp, r->which ? menu[r->which - 1u].name : "none (a real one!)");
        return;
    }

    print_regs(&r->regs);
    if (r->code_ok) {
        printf("  code at pc: %04X %04X\n", r->hw[0], r->hw[1]);
    } else {
        printf("  code at pc: unreadable - pc is not in flash or RAM\n");
    }
    printf("  find it:    arm-none-eabi-addr2line -e Main.elf 0x%08lX 0x%08lX\n",
           (unsigned long)r->regs.pc, (unsigned long)r->regs.lr);

    if (r->hidden) {
        printf("  ** MYSTERY: the verdict is hidden.  Decode it, then:  guess N   (list shows N)\n");
        return;
    }
    static char why[120];
    insn_t ins;
    cause_t c = fault_classify(&r->regs, (int)r->code_ok, r->hw[0], r->hw[1], &ins, why, sizeof why);
    if (r->code_ok) { printf("  instruction: %s %s\n", ins.mnem, ins.ops); }
    printf("  VERDICT:     %s\n  because:     %s\n", cause_name(c), why);
    printf("  experiment:  %s\n", r->which ? menu[r->which - 1u].name : "none - a genuine crash!");
}

static void boot_report(void)
{
    char flags[64];
    reset_flags(csr2_at_boot, flags, sizeof flags);
    printf("\n=== 11 Crash Lab - crash it on purpose, then make it survive ===\n");
    printf("boot %lu since power-on   reset cause: %s   (RCC->CSR2 = 0x%08lX: %s)\n",
           (unsigned long)crash_rec.boots, reset_cause(csr2_at_boot),
           (unsigned long)csr2_at_boot, flags[0] ? flags : "none");

    /* RCC is only partly modelled in Wokwi.  On silicon every reset leaves at
     * least one flag (power-on sets PWRRSTF), so an all-zero CSR2 after a
     * reset means the flags are not modelled - and then the .noinit record,
     * written by this firmware before each reset it causes, is the witness. */
    static const char *const why[] = { "-", "HardFault reporter", "stack canary",
                                       "SOFT WATCHDOG", "reset command" };
    if (csr2_at_boot == 0u) {
        printf("  RCC->CSR2 has NO reset flag set: this simulator does not model them.\n");
    }
    if (crash_rec.reset_why) {
        printf("  .noinit says the firmware reset itself: %s\n", why[crash_rec.reset_why % 5u]);
    }

    int dog_bit = (csr2_at_boot & (RST_IWDG | RST_WWDG)) || crash_rec.reset_why == RW_SOFTDOG;
    if (crash_rec.fresh) {
        report();
        crash_rec.fresh = 0;
    } else if (dog_bit) {
        /* No handler runs on a watchdog reset - the evidence is whatever the
         * loop last wrote into .noinit.  A job far behind the loop's last
         * pass had stopped; all of them close to it means the LOOP stopped. */
        const crash_rec_t *r = &crash_rec;
        unsigned late = 0;
        char d[24];
        printf("---- a WATCHDOG reset us (armed: %stimeout %lu ms) ----\n  running: %s\n",
               dog_list(r->dogs, d, sizeof d), (unsigned long)r->wdt_ms,
               r->armed ? menu[r->armed - 1u].name : "no experiment - a genuine hang!");
        printf("  main loop's last pass at %lu ms; last check-ins:", (unsigned long)r->loop_ms);
        static const uint32_t period[3] = { 250u, 10u, 100u };   /* the jobs' periods */
        for (unsigned i = 0; i < 3u; i++) {
            uint32_t behind = r->loop_ms - r->job_ms[i];
            printf("  %s -%lu ms", hb_names[i], (unsigned long)behind);
            if (behind > period[i] + 50u) { late |= 1u << i; }   /* overdue */
        }
        printf("\n  diagnosis: %s\n", late ? "a JOB stopped while the loop kept running (starved)"
                                          : "every job was current - the main LOOP itself stopped (hung)");
    }
    crash_rec.armed = 0;
    crash_rec.armed_mystery = 0;
    crash_rec.reset_why = RW_NONE;
    crash_rec_seal();
    printf("score: %lu right of %lu mysteries.   Type  help\n",
           (unsigned long)crash_rec.right, (unsigned long)crash_rec.tries);
}

/* ============================================================================
 *  Commands
 * ============================================================================ */
static void stats(void)
{
    extern uint32_t end;                       /* link.ld: start of the heap   */
    extern void *_sbrk(int);
    printf("uptime %lu ms   CPU load %lu.%lu%%   sleep %s   wakeups/s %lu\n",
           (unsigned long)ms_now, (unsigned long)(load_pm / 10u), (unsigned long)(load_pm % 10u),
           sleep_on ? "WFI" : "busy-wait", (unsigned long)wakeups_ps);
    char d[24];
    printf("watchdogs armed: %s(soft %lu ms, WWDG %lu ms, IWDG %lu ms)\n",
           dog_list(dog_armed(), d, sizeof d), (unsigned long)dog_ms(DOG_SOFT),
           (unsigned long)dog_ms(DOG_WWDG), (unsigned long)dog_ms(DOG_IWDG));
    printf("fed by %s, %lu feeds, worst gap %lu ms, missing now:",
           wd_names[wd_mode], (unsigned long)feeds, (unsigned long)worst_gap);
    uint32_t m = hb_missing(&hb);
    for (unsigned i = 0; i < 3u; i++) { if (m & (1u << i)) { printf(" %s", hb_names[i]); } }
    printf("%s\n", m ? "" : " -");
    printf("stack %lu of %lu bytes used, canary %s   heap %ld bytes used\n",
           (unsigned long)stack_used(), (unsigned long)stack_size(),
           stack_guard_ok() ? "intact" : "DAMAGED",
           (long)((char *)_sbrk(0) - (char *)&end));
    printf("OLED %s, %lu bytes sent\n", oled_ok ? "ok" : "absent", (unsigned long)oled_bytes_sent());
}

static void help(void)
{
    printf("crash N      run experiment N (list)      mystery   a hidden random crash\n"
           "guess N      name the mystery's cause     reveal    give up, show the verdict\n"
           "report       the last crash again         stats     load, watchdog, stack\n"
           "wdt MS       retune every armed dog       wdmode hb|loop|isr  who feeds it\n"
           "arm wwdg     arm the window watchdog      arm iwdg  the independent one (not in Wokwi)\n"
           "sleep on|off WFI or busy idle             clear     forget record and score\n"
           "reset        NVIC_SystemReset()           list      the experiments\n");
}

static void list(void)
{
    for (unsigned i = 0; i < X_COUNT; i++) {
        printf("  %2u  %-15s %s\n", i, menu[i].name, menu[i].what);
    }
}

static void run(unsigned n)
{
    if (n == X_STARVE) {
        starving = 1;
        printf("the screen job is now stuck.  LD4 still blinks.  Dogs fed by %s - wait %lu ms.\n",
               wd_names[wd_mode], (unsigned long)dog_ms(DOG_SOFT));
        crash_rec.armed = n + 1u; crash_rec_seal();
        return;
    }
    if (n == X_MYSTERY) {
        unsigned k = mystery_pool[(SysTick->VAL ^ ms_now) % sizeof mystery_pool];
        printf("a mystery crash, coming up...\n");
        crash_run(k, 1);
        return;
    }
    crash_run(n, 0);
}

static void command(char *cmd)
{
    char *arg = strchr(cmd, ' ');
    if (arg) { *arg++ = '\0'; }
    long v = arg ? strtol(arg, 0, 10) : -1;

    if      (!strcmp(cmd, "help"))   { help(); }
    else if (!strcmp(cmd, "list"))   { list(); }
    else if (!strcmp(cmd, "crash"))  { if (v >= 0 && v < X_COUNT) { run((unsigned)v); } else { list(); } }
    else if (!strcmp(cmd, "mystery")){ run(X_MYSTERY); }
    else if (!strcmp(cmd, "report")) { report(); }
    else if (!strcmp(cmd, "stats"))  { stats(); }
    else if (!strcmp(cmd, "reset"))  { printf("NVIC_SystemReset()\n"); crash_reset(RW_COMMAND); }
    else if (!strcmp(cmd, "clear"))  { crash_rec_clear(); printf("record and score cleared\n"); }
    else if (!strcmp(cmd, "guess") || !strcmp(cmd, "reveal")) {
        crash_rec_t *r = &crash_rec;
        if (!r->hidden) { printf("no mystery pending - type  mystery\n"); return; }
        r->tries++;
        if (!strcmp(cmd, "guess") && v == (long)r->which - 1) {
            r->right++;
            printf("RIGHT - it was %ld, %s.\n", v, menu[v].name);
        } else {
            printf("%s - it was %lu, %s.\n", !strcmp(cmd, "guess") ? "WRONG" : "revealed",
                   (unsigned long)(r->which - 1u), menu[r->which - 1u].name);
        }
        r->hidden = 0;
        crash_rec_seal();
        report();
        printf("score: %lu / %lu\n", (unsigned long)r->right, (unsigned long)r->tries);
    }
    else if (!strcmp(cmd, "wdt")) {
        if (v > 0 && dog_set_ms((uint32_t)v) == 0) {
            worst_gap = 0;
            printf("asked %ld ms: soft %lu, WWDG %lu, IWDG %lu (0 = not armed)\n", v,
                   (unsigned long)dog_ms(DOG_SOFT), (unsigned long)dog_ms(DOG_WWDG),
                   (unsigned long)dog_ms(DOG_IWDG));
        } else {
            printf("wdt 1..32768  (ms)\n");
        }
    }
    else if (!strcmp(cmd, "arm")) {
        uint32_t which = (arg && !strcmp(arg, "wwdg")) ? DOG_WWDG :
                         (arg && !strcmp(arg, "iwdg")) ? DOG_IWDG : 0u;
        if (!which) { printf("arm wwdg | arm iwdg   (the soft dog is armed at boot)\n"); return; }
        printf("arming the %s - it cannot be disarmed until reset...\n", which == DOG_WWDG ? "WWDG" : "IWDG");
        dog_arm(which, dog_ms(DOG_SOFT));
        printf("  %s: %lu ms%s\n", which == DOG_WWDG ? "WWDG" : "IWDG", (unsigned long)dog_ms(which),
               which == DOG_IWDG ? "  (Wokwi does not model the IWDG: there it never bites)"
                                 : "  (Wokwi: \"implemented, not tested yet\")");
    }
    else if (!strcmp(cmd, "wdmode")) {
        if (arg && !strcmp(arg, "hb"))   { wd_mode = WD_HEARTBEAT; }
        else if (arg && !strcmp(arg, "loop")) { wd_mode = WD_LOOP; }
        else if (arg && !strcmp(arg, "isr"))  { wd_mode = WD_ISR; }
        printf("watchdog fed by: %s\n", wd_names[wd_mode]);
    }
    else if (!strcmp(cmd, "sleep")) {
        if (arg) { sleep_on = (uint8_t)!strcmp(arg, "on"); }
        printf("idle: %s\n", sleep_on ? "WFI (sleep)" : "busy-wait");
    }
    else { printf("? %s  - type help\n", cmd); }
}

/* ============================================================================
 *  main
 * ============================================================================ */
int main(void)
{
    stack_paint();                            /* first: before the stack grows */
    setvbuf(stdout, NULL, _IONBF, 0);         /* no heap buffer for printf     */

    /* The reset cause, read once and then cleared - or every later boot would
     * still report this one.  RMVF clears all the flags together. */
    csr2_at_boot = RCC->CSR2 & RST_ALL;
    RCC->CSR2 |= RCC_CSR2_RMVF;
    crash_rec_load();

    uart2_init(SystemCoreClock, BAUD);
    USART2->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    NVIC_EnableIRQ(USART2_IRQn);

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    pin_mode(GPIOA, LED_PIN, MODE_OUTPUT);

    boot_report();

    pad_init();
    i2c_init();
    oled_ok = (oled_init() == 0);

    SysTick_Config(SystemCoreClock / 1000u);  /* 1 ms; LOAD = 47999           */
    dog_arm(DOG_SOFT, WDT_MS_DEFAULT);            /* `arm wwdg`, `arm iwdg` add more */
    last_feed = ms_now;
    printf("soft watchdog armed: %lu ms, fed by %s.  OLED %s.\n",
           (unsigned long)dog_ms(DOG_SOFT), wd_names[wd_mode], oled_ok ? "ok" : "absent");

    uint32_t load_start = ms_now;
    for (;;) {
        job_blink();
        job_input();
        job_screen();

        if (!stack_guard_ok()) { stack_overflow_report(); }

        /* The loop's own timestamp goes into .noinit every pass, beside the
         * jobs' (checkin()), for the post-mortem after a watchdog reset. */
        crash_rec.loop_ms = ms_now;
        crash_rec.wdt_ms  = dog_ms(DOG_SOFT);
        crash_rec.dogs    = dog_armed();
        crash_rec_seal();

        if (wd_mode == WD_HEARTBEAT) { if (hb_ready(&hb)) { feed(); } }
        else if (wd_mode == WD_LOOP) { feed(); }

        if (ms_now - load_start >= 1000u) {          /* one-second window      */
            uint32_t window = ms_now - load_start;
            uint32_t idle_ms = idle_cycles / (SystemCoreClock / 1000u);
            load_pm = idle_ms >= window ? 0u : 1000u - (idle_ms * 1000u) / window;
            idle_cycles = 0;
            wakeups_ps = wakeups;
            wakeups = 0;
            load_start = ms_now;
        }

        idle_until_next_tick();
    }
}
