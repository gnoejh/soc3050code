/*
 * host/test.c - lesson 11's host verification, plain gcc, no hardware
 *
 *   1. the fault decoder against hand-checked cases, including the exact
 *      instructions and registers this lesson's own experiments fault on;
 *   2. the decoder against objdump: every 16-bit load, store, push, pop,
 *      ldm, stm, bx, blx, udf, bkpt and svc in Main.elf, text compared;
 *   3. reset-cause naming;
 *   4. IWDG and WWDG timeout arithmetic over their whole ranges;
 *   5. three ways to feed x two kinds of watchdog, simulated against two
 *      hangs and a starved job - and the tightest safe timeout;
 *   6. the SysTick elapsed-time arithmetic behind the CPU-load figure.
 *
 * Usage:  test [objdump.txt]      (run.sh / run.bat make the file)
 * Exit status 0 only if every check passes.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "../diag.h"
#include "../wdog.h"

static int fails, checks;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

/* ---------------------------------------------------------------------------
 *  1. decoder cases
 * --------------------------------------------------------------------------- */
typedef struct {
    const char *what;
    uint16_t hw1, hw2;
    uint32_t pc, xpsr, lr;
    int      rn; uint32_t rnval;          /* one register that matters   */
    int      rm; uint32_t rmval;
    cause_t  want;
    const char *text;                     /* expected "mnem ops", or NULL */
    uint32_t addr;                        /* expected address, or 0       */
} dcase_t;

#define T_ON (1u << 24)
static const dcase_t dcases[] = {
    /* the lesson's own experiments, as built (README quotes these addresses) */
    { "X_UNALIGNED  ldr r2,[r3,#0]  r3=words+1", 0x681a, 0, 0x080016ee, T_ON, 0x00000000, 3, 0x2000000d, -1, 0,
      CAUSE_UNALIGNED, "ldr r2, [r3, #0]", 0x2000000d },
    { "X_WILD_READ  ldr r2,[r3,#0]  r3=0x60000000", 0x681a, 0, 0x08001706, T_ON, 0x00000000, 3, 0x60000000, -1, 0,
      CAUSE_BAD_ADDRESS, "ldr r2, [r3, #0]", 0x60000000 },
    { "X_FLASH_STORE str r2,[r3,#0] r3=0x08007F00", 0x601a, 0, 0x0800171a, T_ON, 0x00000000, 3, 0x08007f00, -1, 0,
      CAUSE_READ_ONLY, "str r2, [r3, #0]", 0x08007f00 },
    { "X_NULL_CALL  pc=0, T=0", 0, 0, 0x00000000, 0, 0x0800173b, -1, 0, -1, 0,
      CAUSE_THUMB_BIT, NULL, 0 },
    { "X_EXEC_PERIPH pc=RCC", 0, 0, 0x40021000, T_ON, 0x0800173b, -1, 0, -1, 0,
      CAUSE_BAD_FETCH, NULL, 0 },
    { "X_UDF        udf #11", 0xde0b, 0, 0x08001740, T_ON, 0, -1, 0, -1, 0,
      CAUSE_UNDEFINED, "udf #11", 0 },
    { "X_BKPT       bkpt 0x0011", 0xbe11, 0, 0x08001744, T_ON, 0, -1, 0, -1, 0,
      CAUSE_BKPT, "bkpt 0x0011", 0 },
    { "X_PACKET     ldmia r2!,{r1} r2=packet+2", 0xca02, 0, 0x08001696, T_ON, 0x0800178d, 2, 0x20000016, -1, 0,
      CAUSE_UNALIGNED, "ldmia r2!, {r1}", 0x20000016 },
    /* forms the experiments do not reach */
    { "ldrh r1,[r0,#6] r0=0x20000001", 0x88c1, 0, 0x08000100, T_ON, 0, 0, 0x20000001, -1, 0,
      CAUSE_UNALIGNED, "ldrh r1, [r0, #6]", 0x20000007 },
    { "ldrb r1,[r0,#3] r0=0x20000001 - bytes never misalign", 0x78c1, 0, 0x08000100, T_ON, 0, 0, 0x20000001, -1, 0,
      CAUSE_UNKNOWN, "ldrb r1, [r0, #3]", 0x20000004 },
    { "str r0,[r1,r2]  r1+r2 = 0x70000000", 0x5088, 0, 0x08000100, T_ON, 0, 1, 0x6FFFFFF0, 2, 0x10,
      CAUSE_BAD_ADDRESS, "str r0, [r1, r2]", 0x70000000 },
    { "ldrsh r3,[r4,r5] r4+r5 odd", 0x5f63, 0, 0x08000100, T_ON, 0, 4, 0x20000100, 5, 0x3,
      CAUSE_UNALIGNED, "ldrsh r3, [r4, r5]", 0x20000103 },
    { "strh r0,[r1,#2] to flash", 0x8048, 0, 0x08000100, T_ON, 0, 1, 0x08001000, -1, 0,
      CAUSE_READ_ONLY, "strh r0, [r1, #2]", 0x08001002 },
    { "push {r4-r7,lr}, sp just above SRAM's floor", 0xb5f0, 0, 0x08000100, T_ON, 0, 13, 0x20000010, -1, 0,
      CAUSE_BAD_ADDRESS, "push {r4, r5, r6, r7, lr}", 0x1FFFFFFC },
    { "pop {r4, pc}", 0xbd10, 0, 0x08000100, T_ON, 0, 13, 0x20002FF0, -1, 0,
      CAUSE_UNKNOWN, "pop {r4, pc}", 0x20002FF0 },
    { "ldr r0,[pc,#4] literal", 0x4801, 0, 0x08000102, T_ON, 0, -1, 0, -1, 0,
      CAUSE_UNKNOWN, "ldr r0, [pc, #4]", 0x08000108 },
    { "ldr r1,[sp,#8]", 0x9902, 0, 0x08000100, T_ON, 0, 13, 0x20002F00, -1, 0,
      CAUSE_UNKNOWN, "ldr r1, [sp, #8]", 0x20002F08 },
    { "stmia r0!,{r1,r2} r0 unaligned", 0xc006, 0, 0x08000100, T_ON, 0, 0, 0x20000102, -1, 0,
      CAUSE_UNALIGNED, "stmia r0!, {r1, r2}", 0x20000102 },
    { "bl (32-bit) - no memory", 0xf7ff, 0xfe96, 0x08000100, T_ON, 0, -1, 0, -1, 0,
      CAUSE_UNKNOWN, NULL, 0 },
    { "svc 0", 0xdf00, 0, 0x08000100, T_ON, 0, -1, 0, -1, 0,
      CAUSE_SVC, "svc 0", 0 },
    { "udf.w (32-bit)", 0xf7f0, 0xa000, 0x08000100, T_ON, 0, -1, 0, -1, 0,
      CAUSE_UNDEFINED, NULL, 0 },
};

static void set_reg(regs_t *r, int n, uint32_t v)
{
    if (n < 0) return;
    if (n < 13) r->r[n] = v; else if (n == 13) r->sp = v;
}

static void test_decoder(void)
{
    printf("1. decoder, %u cases\n", (unsigned)(sizeof dcases / sizeof dcases[0]));
    printf("   %-48s %-22s %s\n", "case", "verdict", "address");
    for (size_t i = 0; i < sizeof dcases / sizeof dcases[0]; i++) {
        const dcase_t *c = &dcases[i];
        regs_t r; memset(&r, 0, sizeof r);
        r.pc = c->pc; r.xpsr = c->xpsr; r.lr = c->lr; r.sp = 0x20002F00; r.exc_return = 0xFFFFFFF9;
        set_reg(&r, c->rn, c->rnval); set_reg(&r, c->rm, c->rmval);
        insn_t ins; char why[160], text[64];
        cause_t got = fault_classify(&r, 1, c->hw1, c->hw2, &ins, why, sizeof why);
        snprintf(text, sizeof text, "%s %s", ins.mnem, ins.ops);
        printf("   %-48s %-22s %s\n", c->what, cause_name(got),
               ins.has_addr ? (snprintf(why, sizeof why, "0x%08lX", (unsigned long)ins.addr), why) : "-");
        CHECK(got == c->want, "%s: verdict %s, wanted %s", c->what, cause_name(got), cause_name(c->want));
        if (c->text) CHECK(!strcmp(text, c->text), "%s: text '%s', wanted '%s'", c->what, text, c->text);
        if (c->addr) CHECK(ins.has_addr && ins.addr == c->addr, "%s: addr 0x%08lX, wanted 0x%08lX",
                           c->what, (unsigned long)ins.addr, (unsigned long)c->addr);
    }
}

/* ---------------------------------------------------------------------------
 *  2. decoder vs objdump, over the whole image
 * --------------------------------------------------------------------------- */
static int interesting(const char *m)
{
    static const char *const set[] = { "ldr", "str", "ldrb", "strb", "ldrh", "strh", "ldrsb", "ldrsh",
        "push", "pop", "ldmia", "stmia", "ldm", "stm", "bx", "blx", "udf", "bkpt", "svc", NULL };
    for (int i = 0; set[i]; i++) if (!strcmp(m, set[i])) return 1;
    return 0;
}

static void test_objdump(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { printf("2. objdump cross-check SKIPPED - no %s (build first)\n", path); return; }
    char line[512];
    unsigned n16 = 0, compared = 0, bad = 0;
    while (fgets(line, sizeof line, f)) {
        unsigned long addr; unsigned hw; char rest[480];
        /* " 80014d6:\tca02      \tldmia\tr2!, {r1}" - one 4-digit group = 16-bit */
        if (sscanf(line, " %lx:\t%x %479[^\n]", &addr, &hw, rest) != 3) continue;
        const char *colon = strchr(line, ':');
        const char *p = colon + 2; int digits = 0;
        while (isxdigit((unsigned char)p[digits])) digits++;
        if (digits != 4) continue;                       /* .word, .byte      */
        if (p[4] == ' ' && isxdigit((unsigned char)p[5])) continue;   /* 32-bit */
        const char *t = strchr(p, '\t');
        if (!t) continue;
        char mnem[32] = "", ops[256] = "";
        sscanf(t + 1, "%31s", mnem);
        const char *o = strchr(t + 1, '\t');
        if (o) { strncpy(ops, o + 1, sizeof ops - 1); }
        char *at = strchr(ops, '@'); if (at) *at = '\0';
        for (int k = (int)strlen(ops) - 1; k >= 0 && isspace((unsigned char)ops[k]); k--) ops[k] = '\0';
        if (mnem[0] == '.') continue;
        n16++;
        insn_t ins;
        thumb_decode((uint16_t)hw, 0, (uint32_t)addr, NULL, &ins);
        int mine = (ins.kind != INS_OTHER);
        if (!mine && !interesting(mnem)) continue;
        compared++;
        if (strcmp(ins.mnem, mnem) || strcmp(ins.ops, ops)) {
            if (bad < 10) printf("   MISMATCH at %08lx %04x: objdump '%s %s'  decoder '%s %s'\n",
                                 addr, hw, mnem, ops, ins.mnem, ins.ops);
            bad++;
        }
    }
    fclose(f);
    printf("2. objdump cross-check: %u 16-bit instructions in Main.elf, %u memory/branch/trap "
           "instructions compared, %u mismatches\n", n16, compared, bad);
    CHECK(compared > 500, "too few instructions compared (%u)", compared);
    CHECK(bad == 0, "%u decoder/objdump mismatches", bad);
}

/* ---------------------------------------------------------------------------
 *  3. reset cause
 * --------------------------------------------------------------------------- */
static void test_reset(void)
{
    static const struct { uint32_t v; const char *want; } c[] = {
        { 0x0C000000, "POWER-ON/BROWN-OUT" },      /* PWR + PIN: power-up        */
        { 0x14000000, "SOFTWARE" },                /* SFT + PIN                  */
        { 0x24000000, "IWDG" },                    /* IWDG + PIN                 */
        { 0x04000000, "NRST PIN" },
        { 0x00000000, "NONE RECORDED" },
        { 0x40000000, "WWDG" },
    };
    char flags[80];
    printf("3. reset cause\n");
    for (size_t i = 0; i < sizeof c / sizeof c[0]; i++) {
        reset_flags(c[i].v, flags, sizeof flags);
        printf("   CSR2 0x%08lX -> %-20s flags: %s\n", (unsigned long)c[i].v, reset_cause(c[i].v), flags);
        CHECK(!strcmp(reset_cause(c[i].v), c[i].want), "reset 0x%08lX", (unsigned long)c[i].v);
    }
}

/* ---------------------------------------------------------------------------
 *  4. IWDG arithmetic
 * --------------------------------------------------------------------------- */
static void test_iwdg(void)
{
    static const uint32_t ms[] = { 1, 10, 50, 100, 250, 500, 1000, 4000, 10000, 32768 };
    printf("4. IWDG timeouts at LSI %u Hz: (RLR+1) * (4 << PR) / LSI\n", LSI_HZ);
    printf("   %8s %4s %6s %6s %12s\n", "asked ms", "PR", "div", "RLR", "actual us");
    CHECK(iwdg_timeout_us(0, 0, LSI_HZ) == 125, "min timeout should be 125 us");
    CHECK(iwdg_timeout_us(6, 4095, LSI_HZ) == 32768000u, "max timeout should be 32.768 s");
    for (size_t i = 0; i < sizeof ms / sizeof ms[0]; i++) {
        uint32_t pr = 0, rlr = 0;
        int rc = iwdg_pick(ms[i], LSI_HZ, &pr, &rlr);
        printf("   %8lu %4lu %6lu %6lu %12lu\n", (unsigned long)ms[i], (unsigned long)pr,
               (unsigned long)iwdg_div(pr), (unsigned long)rlr, (unsigned long)iwdg_timeout_us(pr, rlr, LSI_HZ));
        CHECK(rc == 0, "pick %lu", (unsigned long)ms[i]);
    }
    /* every millisecond in range: never early, at most one tick late, and the
     * smallest prescaler that fits */
    unsigned worst_late_us = 0, sweep_fail = 0;
    for (uint32_t m = 1; m <= 32768u; m++) {
        uint32_t pr = 0, rlr = 0;
        if (iwdg_pick(m, LSI_HZ, &pr, &rlr)) { sweep_fail++; continue; }
        uint32_t us = iwdg_timeout_us(pr, rlr, LSI_HZ), tick = iwdg_div(pr) * 1000000u / LSI_HZ;
        if (us < m * 1000u || us - m * 1000u >= tick) sweep_fail++;
        if (us - m * 1000u > worst_late_us) worst_late_us = us - m * 1000u;
        if (pr > 0 && (uint64_t)m * LSI_HZ <= 4096ull * 1000u * iwdg_div(pr - 1)) sweep_fail++;
    }
    printf("   sweep 1..32768 ms: %u failures, never early, worst rounding-up %u us\n", sweep_fail, worst_late_us);
    CHECK(sweep_fail == 0, "IWDG sweep");
    uint32_t pr = 0, rlr = 0;
    CHECK(iwdg_pick(32769, LSI_HZ, &pr, &rlr) == -1, "32769 ms must be out of range");
}

/* ---------------------------------------------------------------------------
 *  4b. WWDG arithmetic (PCLK 48 MHz)
 * --------------------------------------------------------------------------- */
static void test_wwdg(void)
{
    static const uint32_t ms[] = { 1, 10, 50, 100, 250, 500, 699, 1000 };
    const uint32_t pclk = 48000000u;
    printf("4b. WWDG timeouts at PCLK 48 MHz: (T[5:0]+1) * 4096 * 2^WDGTB / PCLK\n");
    printf("   %8s %6s %6s %12s\n", "asked ms", "WDGTB", "T", "actual us");
    for (size_t i = 0; i < sizeof ms / sizeof ms[0]; i++) {
        uint32_t tb = 0, t = 0;
        int clipped = wwdg_pick(ms[i], pclk, &tb, &t);
        printf("   %8lu %6lu   0x%02lX %12lu%s\n", (unsigned long)ms[i], (unsigned long)tb, (unsigned long)t,
               (unsigned long)wwdg_timeout_us(tb, t, pclk), clipped ? "  <- longest possible" : "");
    }
    unsigned bad = 0;
    for (uint32_t m = 1; m <= 699u; m++) {
        uint32_t tb = 0, t = 0;
        if (wwdg_pick(m, pclk, &tb, &t) || wwdg_timeout_us(tb, t, pclk) < m * 1000u || t < 0x40u || t > 0x7Fu) bad++;
    }
    uint32_t tb = 0, t = 0;
    CHECK(wwdg_pick(700, pclk, &tb, &t) == 1 && t == 0x7F && tb == 7, "700 ms must clip to the maximum");
    CHECK(wwdg_timeout_us(7, 0x7F, pclk) == 699050u, "WWDG max at 48 MHz is 699.05 ms");
    printf("   sweep 1..699 ms: %u failures (never early, T always 0x40..0x7F)\n", bad);
    CHECK(bad == 0, "WWDG sweep");
}

/* ---------------------------------------------------------------------------
 *  5. feeding policies x watchdog kinds, simulated
 * ---------------------------------------------------------------------------
 * The firmware's superloop, at 1 ms resolution: blink every 250 ms, input
 * every 10 ms, screen every 100 ms costing SCREEN_MS of blocking I2C, and
 * one long serial print (a `report`, ~2 KB at 87 us/char) at t = 5 s.
 * Failure at t = 10 s:
 *   HANG        the loop stops; interrupts still run       (crash 9)
 *   HANG_NOIRQ  the loop stops with interrupts disabled    (crash 10)
 *   STARVE      the screen job stops checking in           (crash 11)
 * Two kinds of dog: SOFT counts in the SysTick interrupt, so it stops
 * counting when interrupts are off; HW (WWDG or IWDG) has its own clock. */
enum { F_NONE, F_HANG, F_HANG_NOIRQ, F_STARVE, F_COUNT };
enum { M_HB, M_LOOP, M_ISR };
enum { D_SOFT, D_HW };
#define SIM_MS     60000u
#define FAIL_AT    10000u
#define SCREEN_MS  25u           /* a full-frame OLED flush at 400 kHz (lesson 12) */
#define PRINT_AT   5000u
#define PRINT_MS   180u          /* ~2 KB at 115200 baud                           */

typedef struct { uint32_t t; int64_t left_us; uint32_t timeout_us; int mode, dog, failure; uint32_t reset_at; } sim_t;

static int tick(sim_t *s)                 /* 1 ms passes; 1 = watchdog reset */
{
    s->t++;
    int irq_on = !(s->failure == F_HANG_NOIRQ && s->t >= FAIL_AT);
    if (s->mode == M_ISR && irq_on) { s->left_us = s->timeout_us; return 0; }   /* SysTick feeds */
    if (s->dog == D_SOFT && !irq_on) { return 0; }       /* its clock IS SysTick: stopped */
    s->left_us -= 1000;
    if (s->left_us <= 0) { s->reset_at = s->t; return 1; }
    return 0;
}

/* returns the time of the watchdog reset, or 0; *gap = worst feed gap */
static uint32_t simulate(int mode, int dog, int failure, uint32_t timeout_ms, int print, uint32_t *gap)
{
    sim_t s = { 0, 0, 0, mode, dog, failure, 0 };
    if (dog == D_HW) {
        uint32_t pr = 0, rlr = 0;
        iwdg_pick(timeout_ms, LSI_HZ, &pr, &rlr);
        s.timeout_us = iwdg_timeout_us(pr, rlr, LSI_HZ);
    } else {
        s.timeout_us = timeout_ms * 1000u;
    }
    s.left_us = s.timeout_us;
    heartbeat_t hb = { 7u, 0u };
    uint32_t nb = 0, ni = 0, ns = 0, last_feed = 0, worst = 0;
    int printed = 0;
    while (s.t < SIM_MS) {
        if ((failure == F_HANG || failure == F_HANG_NOIRQ) && s.t >= FAIL_AT) { if (tick(&s)) break; continue; }
        if (s.t >= nb) { nb += 250; hb_checkin(&hb, 1u); }
        if (s.t >= ni) {
            ni += 10;
            if (print && !printed && s.t >= PRINT_AT) {
                printed = 1;
                for (unsigned k = 0; k < PRINT_MS && !s.reset_at; k++) tick(&s);
                if (s.reset_at) break;
            }
            hb_checkin(&hb, 2u);
        }
        if (s.t >= ns) {
            ns += 100;
            if (!(failure == F_STARVE && s.t >= FAIL_AT)) {
                for (unsigned k = 0; k < SCREEN_MS && !s.reset_at; k++) tick(&s);
                if (s.reset_at) break;
                hb_checkin(&hb, 4u);
            }
        }
        int fed = (mode == M_LOOP) || (mode == M_HB && hb_ready(&hb));
        if (fed) {
            if (s.t - last_feed > worst && last_feed) worst = s.t - last_feed;
            last_feed = s.t;
            s.left_us = s.timeout_us;
        }
        if (tick(&s)) break;
    }
    if (gap) *gap = worst;
    return s.reset_at;
}

static void test_policies(void)
{
    static const char *const mn[] = { "heartbeat bitmap", "feed every loop", "feed in SysTick" };
    static const char *const dn[] = { "soft dog", "WWDG/IWDG" };
    static const char *const fn[] = { "healthy", "hang", "hang, irq off", "starve" };
    /* what each (feeder, dog) pair MUST do for hang, hang-irq-off, starve */
    static const int want[3][2][3] = {
        { { 1, 0, 1 }, { 1, 1, 1 } },      /* heartbeat: soft is blind only with irq off */
        { { 1, 0, 0 }, { 1, 1, 0 } },      /* every loop: never sees a starved job       */
        { { 0, 0, 0 }, { 0, 1, 0 } },      /* SysTick: hardware sees irq-off, nothing else */
    };
    printf("5. who feeds x which dog, timeout %u ms, 60 s simulated, failure at 10 s\n"
           "   (screen %u ms per frame, one %u ms print at 5 s)\n", WDT_MS_DEFAULT, SCREEN_MS, PRINT_MS);
    printf("   %-17s %-10s %-11s %-14s %-14s %-14s %s\n", "fed by", "dog", fn[0], fn[1], fn[2], fn[3], "worst gap");
    for (int m = 0; m < 3; m++) {
        for (int d = 0; d < 2; d++) {
            uint32_t r[F_COUNT], gap = 0;
            for (int f = 0; f < F_COUNT; f++) r[f] = simulate(m, d, f, WDT_MS_DEFAULT, 1, f == F_NONE ? &gap : NULL);
            char c[F_COUNT][32];
            for (int f = 0; f < F_COUNT; f++) {
                if (!r[f]) snprintf(c[f], 32, f == F_NONE ? "ok" : "NEVER");
                else if (f == F_NONE) snprintf(c[f], 32, "FALSE %lu", (unsigned long)r[f]);
                else snprintf(c[f], 32, "+%lu ms", (unsigned long)(r[f] - FAIL_AT));
            }
            printf("   %-17s %-10s %-11s %-14s %-14s %-14s %lu ms\n", d ? "" : mn[m], dn[d],
                   c[0], c[1], c[2], c[3], (unsigned long)gap);
            CHECK(r[F_NONE] == 0, "%s/%s: false reset when healthy", mn[m], dn[d]);
            for (int f = 1; f < F_COUNT; f++) {
                CHECK((r[f] != 0) == want[m][d][f - 1], "%s/%s/%s: expected %s", mn[m], dn[d], fn[f],
                      want[m][d][f - 1] ? "a reset" : "no reset");
            }
        }
    }
    /* the tightest timeout the heartbeat survives, with and without the print */
    for (int print = 0; print < 2; print++) {
        uint32_t t;
        for (t = 10; t <= 2000; t += 5) if (simulate(M_HB, D_SOFT, F_NONE, t, print, NULL) == 0) break;
        uint32_t gap = 0; simulate(M_HB, D_SOFT, F_NONE, t, print, &gap);
        printf("   tightest safe heartbeat timeout %s: %lu ms (worst gap %lu ms); hang then caught in %lu ms\n",
               print ? "with the 180 ms print" : "without long prints  ", (unsigned long)t, (unsigned long)gap,
               (unsigned long)(simulate(M_HB, D_SOFT, F_HANG, t, print, NULL) - FAIL_AT));
        CHECK(t > gap, "tightest timeout must exceed the worst gap");
    }
}

/* ---------------------------------------------------------------------------
 *  6. SysTick elapsed
 * --------------------------------------------------------------------------- */
static void test_systick(void)
{
    printf("6. SysTick elapsed (LOAD = 47999, counting down)\n");
    CHECK(systick_elapsed(40000, 10000, 47999) == 30000, "no wrap");
    CHECK(systick_elapsed(100, 47900, 47999) == 200, "wrap: 100 down to 0, reload, 47999 down to 47900");
    CHECK(systick_elapsed(5, 5, 47999) == 0, "same value");
    /* a whole sleep from just after a tick to the next: almost one period */
    CHECK(systick_elapsed(47990, 47995, 47999) == 47995, "nearly a full period");
    printf("   4 cases: 30000, 200, 0, 47995 cycles\n");
}

int main(int argc, char **argv)
{
    test_decoder();
    test_objdump(argc > 1 ? argv[1] : "objdump.txt");
    test_reset();
    test_iwdg();
    test_wwdg();
    test_policies();
    test_systick();
    printf("\n%d checks, %d failed - %s\n", checks, fails, fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
