/*
 * diag.c - the crash detective's tools, in pure C        (SOC3050 lesson 11)
 *
 * See diag.h.  No device header: this file is compiled into the firmware AND
 * into host/test.c, which is how it was checked (README, "Verified by
 * execution").
 *
 * Why this file exists at all.  On a Cortex-M3/M4/M7 (ARMv7-M) a fault sets
 * bits in the Configurable Fault Status Register, CFSR: UNALIGNED, INVSTATE,
 * UNDEFINSTR, PRECISERR with the bad address in BFAR...  The Cortex-M0+ is
 * ARMv6-M and has none of that - one exception, HardFault, and no status
 * register saying why.  What it DOES leave is the evidence: the stacked
 * registers, the stacked pc, and the instruction sitting at that pc.  This
 * file reads the evidence the way you would by hand with the slide-8 table.
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "diag.h"

/* ---------------------------------------------------------------------------
 *  The memory map (RM0490 / stm32c031xx.h base addresses, link.ld sizes)
 * ---------------------------------------------------------------------------
 * Coarse on purpose: a hole INSIDE the peripheral block (an address between
 * two peripherals) still counts as "peripheral space" here.  What matters for
 * a verdict is the big picture - is this flash, RAM, a peripheral, or nothing
 * at all - and which of read, write and execute that kind of memory allows.
 * Peripheral and private-peripheral space are Execute Never in the Armv6-M
 * default memory map: code fetched from there faults. */
static const region_t regions[] = {
    { 0x00000000u, 32u * 1024u, MEM_R | MEM_X,         "flash (boot alias at 0)" },
    { 0x08000000u, 32u * 1024u, MEM_R | MEM_X,         "flash" },
    { 0x1FFF0000u, 0x8000u,     MEM_R | MEM_X,         "system memory" },
    { 0x20000000u, 12u * 1024u, MEM_R | MEM_W | MEM_X, "SRAM" },
    { 0x40000000u, 0x00030000u, MEM_R | MEM_W,         "peripherals" },
    { 0x50000000u, 0x00002000u, MEM_R | MEM_W,         "GPIO ports (IOPORT)" },
    { 0xE0000000u, 0x00100000u, MEM_R | MEM_W,         "Cortex-M0+ private peripherals" },
};

const region_t *mem_region(uint32_t addr)
{
    for (size_t i = 0; i < sizeof regions / sizeof regions[0]; i++) {
        if (addr - regions[i].base < regions[i].size) { return &regions[i]; }
    }
    return NULL;
}

int mem_can(uint32_t addr, uint32_t len, int access)
{
    const region_t *a = mem_region(addr);
    const region_t *b = mem_region(addr + (len ? len - 1u : 0u));
    return a != NULL && a == b && (a->access & access) == access;
}

/* ---------------------------------------------------------------------------
 *  Thumb decoding - only what ARMv6-M has, and only what a fault needs
 * ---------------------------------------------------------------------------
 * objdump's register names, so the host test can compare text exactly. */
static const char *const rn_name[16] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "sl", "fp", "ip", "sp", "lr", "pc"
};

static uint32_t reg(const regs_t *r, unsigned n)
{
    if (n < 13u) { return r->r[n]; }
    if (n == 13u) { return r->sp; }
    if (n == 14u) { return r->lr; }
    return r->pc;
}

static void set(insn_t *o, ins_kind_t k, const char *m, unsigned size)
{
    o->kind = k;
    o->size = (uint8_t)size;
    snprintf(o->mnem, sizeof o->mnem, "%s", m);
}

/* "{r4, r5, lr}" from a register bitmap */
static unsigned reglist(char *buf, unsigned len, unsigned bits)
{
    unsigned n = 0, used = 0;
    used += (unsigned)snprintf(buf + used, len - used, "{");
    for (unsigned i = 0; i < 16u && used < len; i++) {
        if (bits & (1u << i)) {
            used += (unsigned)snprintf(buf + used, len - used, "%s%s", n ? ", " : "", rn_name[i]);
            n++;
        }
    }
    if (used < len) { snprintf(buf + used, len - used, "}"); }
    return n;
}

void thumb_decode(uint16_t hw1, uint16_t hw2, uint32_t pc, const regs_t *r, insn_t *o)
{
    memset(o, 0, sizeof *o);
    o->len = 2;
    set(o, INS_OTHER, "?", 0);

    /* 32-bit encodings start 0b11101, 0b11110 or 0b11111 */
    if ((hw1 >> 11) >= 0x1Du) {
        o->len = 4;
        if ((hw1 & 0xF800u) == 0xF000u && (hw2 & 0xD000u) == 0xD000u) {
            set(o, INS_BL, "bl", 0);
        } else if ((hw1 & 0xFFF0u) == 0xF7F0u && (hw2 & 0xF000u) == 0xA000u) {
            set(o, INS_UDF, "udf.w", 0);
        } else {
            set(o, INS_32BIT, "(32-bit)", 0);
        }
        return;
    }

    unsigned rt = hw1 & 7u, rn = (hw1 >> 3) & 7u, rm = (hw1 >> 6) & 7u;
    unsigned imm5 = (hw1 >> 6) & 31u;

    if ((hw1 & 0xF800u) == 0x4800u) {                     /* LDR Rt, [pc, #imm8*4] */
        unsigned t = (hw1 >> 8) & 7u, imm = (hw1 & 0xFFu) * 4u;
        set(o, INS_LOAD, "ldr", 4);
        snprintf(o->ops, sizeof o->ops, "%s, [pc, #%u]", rn_name[t], imm);
        o->addr = ((pc + 4u) & ~3u) + imm;                /* Align(PC,4) + imm  */
        o->has_addr = 1;
        return;
    }
    if ((hw1 & 0xF000u) == 0x5000u) {                     /* register offset    */
        static const char *const m[8] = { "str", "strh", "strb", "ldrsb", "ldr", "ldrh", "ldrb", "ldrsh" };
        static const uint8_t sz[8]    = { 4, 2, 1, 1, 4, 2, 1, 2 };
        unsigned op = (hw1 >> 9) & 7u;
        set(o, op < 3u ? INS_STORE : INS_LOAD, m[op], sz[op]);
        snprintf(o->ops, sizeof o->ops, "%s, [%s, %s]", rn_name[rt], rn_name[rn], rn_name[rm]);
        if (r) { o->addr = reg(r, rn) + reg(r, rm); o->has_addr = 1; }
        return;
    }
    if ((hw1 & 0xE000u) == 0x6000u || (hw1 & 0xF000u) == 0x8000u) {   /* immediate offset */
        unsigned load = (hw1 >> 11) & 1u, size, imm;
        if ((hw1 & 0xF000u) == 0x8000u)      { size = 2; set(o, load ? INS_LOAD : INS_STORE, load ? "ldrh" : "strh", 2); }
        else if (hw1 & 0x1000u)              { size = 1; set(o, load ? INS_LOAD : INS_STORE, load ? "ldrb" : "strb", 1); }
        else                                 { size = 4; set(o, load ? INS_LOAD : INS_STORE, load ? "ldr"  : "str",  4); }
        imm = imm5 * size;                                /* scaled by access size */
        snprintf(o->ops, sizeof o->ops, "%s, [%s, #%u]", rn_name[rt], rn_name[rn], imm);
        if (r) { o->addr = reg(r, rn) + imm; o->has_addr = 1; }
        return;
    }
    if ((hw1 & 0xF000u) == 0x9000u) {                     /* SP-relative        */
        unsigned load = (hw1 >> 11) & 1u, t = (hw1 >> 8) & 7u, imm = (hw1 & 0xFFu) * 4u;
        set(o, load ? INS_LOAD : INS_STORE, load ? "ldr" : "str", 4);
        snprintf(o->ops, sizeof o->ops, "%s, [sp, #%u]", rn_name[t], imm);
        if (r) { o->addr = r->sp + imm; o->has_addr = 1; }
        return;
    }
    if ((hw1 & 0xFE00u) == 0xB400u) {                     /* PUSH {list, lr}    */
        unsigned bits = (hw1 & 0xFFu) | ((hw1 & 0x100u) ? (1u << 14) : 0u);
        set(o, INS_STORE_MULTI, "push", 4);
        o->nregs = (uint8_t)reglist(o->ops, sizeof o->ops, bits);
        if (r) { o->addr = r->sp - 4u * o->nregs; o->has_addr = 1; }
        return;
    }
    if ((hw1 & 0xFE00u) == 0xBC00u) {                     /* POP {list, pc}     */
        unsigned bits = (hw1 & 0xFFu) | ((hw1 & 0x100u) ? (1u << 15) : 0u);
        set(o, INS_LOAD_MULTI, "pop", 4);
        o->nregs = (uint8_t)reglist(o->ops, sizeof o->ops, bits);
        if (r) { o->addr = r->sp; o->has_addr = 1; }
        return;
    }
    if ((hw1 & 0xF000u) == 0xC000u) {                     /* LDMIA / STMIA      */
        unsigned load = (hw1 >> 11) & 1u, n = (hw1 >> 8) & 7u, bits = hw1 & 0xFFu;
        /* STM always writes back; LDM writes back unless Rn is in the list */
        int wb = !load || !(bits & (1u << n));
        set(o, load ? INS_LOAD_MULTI : INS_STORE_MULTI, load ? "ldmia" : "stmia", 4);
        unsigned used = (unsigned)snprintf(o->ops, sizeof o->ops, "%s%s, ", rn_name[n], wb ? "!" : "");
        o->nregs = (uint8_t)reglist(o->ops + used, (unsigned)sizeof o->ops - used, bits);
        if (r) { o->addr = reg(r, n); o->has_addr = 1; }
        return;
    }
    if ((hw1 & 0xFF07u) == 0x4700u) {                     /* BX / BLX Rm        */
        unsigned m = (hw1 >> 3) & 15u;
        set(o, INS_BRANCH_REG, (hw1 & 0x80u) ? "blx" : "bx", 0);
        snprintf(o->ops, sizeof o->ops, "%s", rn_name[m]);
        return;
    }
    if ((hw1 & 0xFF00u) == 0xDE00u) {                     /* UDF #imm8          */
        set(o, INS_UDF, "udf", 0);
        snprintf(o->ops, sizeof o->ops, "#%u", hw1 & 0xFFu);
        return;
    }
    if ((hw1 & 0xFF00u) == 0xDF00u) {                     /* SVC #imm8          */
        set(o, INS_SVC, "svc", 0);
        snprintf(o->ops, sizeof o->ops, "%u", hw1 & 0xFFu);
        return;
    }
    if ((hw1 & 0xFF00u) == 0xBE00u) {                     /* BKPT #imm8         */
        set(o, INS_BKPT, "bkpt", 0);
        snprintf(o->ops, sizeof o->ops, "0x%04x", hw1 & 0xFFu);
        return;
    }
    /* everything else - ALU, compare, conditional branch - touches no memory */
}

/* ---------------------------------------------------------------------------
 *  The verdict
 * --------------------------------------------------------------------------- */
const char *cause_name(cause_t c)
{
    static const char *const n[CAUSE_COUNT] = {
        "UNKNOWN", "THUMB BIT CLEAR", "BAD INSTRUCTION FETCH", "UNALIGNED ACCESS",
        "NOTHING MAPPED THERE", "STORE TO READ-ONLY", "UNDEFINED INSTRUCTION",
        "BKPT, NO DEBUGGER", "SVC NOT TAKEN"
    };
    return (unsigned)c < CAUSE_COUNT ? n[c] : "?";
}

const char *exc_return_text(uint32_t e)
{
    switch (e) {
    case 0xFFFFFFF1u: return "handler mode, main stack (MSP) - the fault was in an ISR";
    case 0xFFFFFFF9u: return "thread mode, main stack (MSP)";
    case 0xFFFFFFFDu: return "thread mode, process stack (PSP) - an RTOS task";
    default:          return "not a valid EXC_RETURN";
    }
}

cause_t fault_classify(const regs_t *r, int code_ok, uint16_t hw1, uint16_t hw2,
                       insn_t *ins, char *why, unsigned len)
{
    memset(ins, 0, sizeof *ins);

    /* 1. The T bit.  Every Cortex-M instruction is Thumb; xPSR.T = 0 means a
     *    BX/BLX/POP {pc} loaded an address with bit 0 clear.  The pc is then
     *    the branch TARGET, and lr usually still points just past the call. */
    if (!(r->xpsr & (1UL << 24))) {
        snprintf(why, len, "xPSR.T=0: a branch to the EVEN address 0x%08lX. "
                 "The call is the instruction just before 0x%08lX (lr).",
                 (unsigned long)r->pc, (unsigned long)(r->lr & ~1u));
        return CAUSE_THUMB_BIT;
    }

    /* 2. The pc itself.  Fetching from peripheral space (Execute Never) or
     *    from nowhere faults before any instruction exists to decode. */
    if (!mem_can(r->pc, 2, MEM_X)) {
        const region_t *g = mem_region(r->pc);
        snprintf(why, len, "pc 0x%08lX is in %s, which cannot be executed.",
                 (unsigned long)r->pc, g ? g->name : "unmapped space");
        return CAUSE_BAD_FETCH;
    }
    if (!code_ok) {
        snprintf(why, len, "pc is executable but its code could not be read.");
        return CAUSE_UNKNOWN;
    }

    /* 3. The instruction at pc. */
    thumb_decode(hw1, hw2, r->pc, r, ins);
    switch (ins->kind) {
    case INS_UDF:  snprintf(why, len, "%s %s: permanently undefined - executing it always faults.", ins->mnem, ins->ops);
                   return CAUSE_UNDEFINED;
    case INS_BKPT: snprintf(why, len, "bkpt with no debugger to halt: Armv6-M escalates it to HardFault.");
                   return CAUSE_BKPT;
    case INS_SVC:  snprintf(why, len, "svc while SVCall could not run (masked or already higher priority).");
                   return CAUSE_SVC;
    case INS_LOAD: case INS_STORE: case INS_LOAD_MULTI: case INS_STORE_MULTI: {
        int store = (ins->kind == INS_STORE || ins->kind == INS_STORE_MULTI);
        uint32_t span = ins->nregs ? 4u * ins->nregs : ins->size;
        const region_t *g = mem_region(ins->addr);
        /* Armv6-M has NO unaligned support at all: every halfword or word
         * access must be aligned to its size (multiples: word aligned). */
        if (ins->addr & (uint32_t)(ins->size - 1u)) {
            snprintf(why, len, "%s %s: %u-byte access to 0x%08lX, which is not a multiple of %u.",
                     ins->mnem, ins->ops, (unsigned)ins->size, (unsigned long)ins->addr, (unsigned)ins->size);
            return CAUSE_UNALIGNED;
        }
        if (!g || !mem_can(ins->addr, span, MEM_R)) {
            snprintf(why, len, "%s %s: address 0x%08lX - nothing is mapped there (bus error).",
                     ins->mnem, ins->ops, (unsigned long)ins->addr);
            return CAUSE_BAD_ADDRESS;
        }
        if (store && !(g->access & MEM_W)) {
            snprintf(why, len, "%s %s: a store to 0x%08lX, in %s - read-only.",
                     ins->mnem, ins->ops, (unsigned long)ins->addr, g->name);
            return CAUSE_READ_ONLY;
        }
        snprintf(why, len, "%s %s: address 0x%08lX (%s) looks legal - a peripheral that "
                 "refused, or a fault during exception entry?",
                 ins->mnem, ins->ops, (unsigned long)ins->addr, g->name);
        return CAUSE_UNKNOWN;
    }
    case INS_32BIT:
        snprintf(why, len, "a 32-bit encoding Armv6-M may not implement: undefined.");
        return CAUSE_UNDEFINED;
    default:
        snprintf(why, len, "%s %s touches no memory - suspect the instruction BEFORE it, "
                 "or a corrupted stack frame.", ins->mnem, ins->ops);
        return CAUSE_UNKNOWN;
    }
}

/* ---------------------------------------------------------------------------
 *  Reset cause
 * --------------------------------------------------------------------------- */
static const struct { uint32_t bit; const char *name; } rst[] = {
    { RST_IWDG, "IWDG" }, { RST_WWDG, "WWDG" }, { RST_SFT, "SOFTWARE" },
    { RST_LPWR, "LOW-POWER" }, { RST_PWR, "POWER-ON/BROWN-OUT" },
    { RST_OBL, "OPTION-BYTE LOAD" }, { RST_PIN, "NRST PIN" },
};

const char *reset_cause(uint32_t csr2)
{
    for (size_t i = 0; i < sizeof rst / sizeof rst[0]; i++) {
        if (csr2 & rst[i].bit) { return rst[i].name; }
    }
    return "NONE RECORDED";
}

void reset_flags(uint32_t csr2, char *buf, unsigned len)
{
    unsigned used = 0;
    buf[0] = '\0';
    for (size_t i = 0; i < sizeof rst / sizeof rst[0] && used < len; i++) {
        if (csr2 & rst[i].bit) {
            used += (unsigned)snprintf(buf + used, len - used, "%s%s", used ? " " : "", rst[i].name);
        }
    }
}
