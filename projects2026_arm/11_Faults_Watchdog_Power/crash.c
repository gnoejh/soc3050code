/*
 * crash.c - HardFault reporter, crash record, deliberate faults
 *                                                       (SOC3050 lesson 11)
 *
 * What happens on a fault, in order:
 *
 *   1. The core pushes the same 8-word frame as for any exception -
 *      r0 r1 r2 r3 r12 lr pc xPSR - and enters HardFault_Handler with a magic
 *      EXC_RETURN value in lr that says which stack the frame is on.
 *   2. HardFault_Handler (naked, a few instructions of assembly) saves r4-r11,
 *      which the hardware did NOT push, before any C code can change them.
 *   3. crash_from_hardfault() copies all of it, plus the instruction at the
 *      stacked pc, into crash_rec - which lives in .noinit and so survives
 *      a reset - prints one line WITHOUT printf, and resets the chip.
 *   4. main() starts again, finds the record, decodes it with diag.c, and
 *      prints the full report at leisure, with printf, on a healthy system.
 *
 * Why not decode and printf inside the handler?  Because the system is
 * broken when it runs: the fault may have been INSIDE printf, or with the
 * heap half-updated, or with the stack nearly gone.  Do the minimum, record,
 * reset; analyse later.  It is the flight-recorder pattern.
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "stm32c031xx.h"
#include "crash.h"

#define REC_MAGIC        0xC0DEFA17u
#define STACK_FILL       0xDEADBEEFu  /* lesson 07's OS_STACK_FILL            */
#define GUARD_WORDS      16u          /* the bottom 64 bytes are the canary   */
#define FLASH_TEST_ADDR  0x08007F00u  /* last page of the 32 KB; unused here  */
#define WILD_ADDR        0x60000000u  /* nothing at all is mapped here        */

#define RESET_ON_FAULT   1            /* 0: park in the handler, for GDB      */

extern volatile uint32_t ms_now;      /* Main.c                               */
extern uint32_t _stack_floor, _estack;/* link.ld                              */

crash_rec_t crash_rec __attribute__((section(".noinit")));

/* ===========================================================================
 *  The record
 * =========================================================================== */
static uint32_t rec_sum(void)
{
    const uint32_t *w = (const uint32_t *)&crash_rec;
    uint32_t s = 0x5EED5EEDu;
    for (unsigned i = 0; i < offsetof(crash_rec_t, sum) / 4u; i++) {
        s = (s << 5 | s >> 27) ^ w[i];      /* rotate-xor: cheap, order-aware */
    }
    return s;
}

void crash_rec_seal(void) { crash_rec.sum = rec_sum(); }

void crash_rec_clear(void)
{
    memset(&crash_rec, 0, sizeof crash_rec);
    crash_rec.magic = REC_MAGIC;
    crash_rec_seal();
}

int crash_rec_load(void)
{
    int ok = (crash_rec.magic == REC_MAGIC && crash_rec.sum == rec_sum());
    if (!ok) { crash_rec_clear(); }     /* power-on garbage, or never written */
    crash_rec.boots++;
    crash_rec_seal();
    return ok;
}

/* ===========================================================================
 *  Printing with no printf - safe inside a fault handler
 * =========================================================================== */
static void raw_putc(char c)
{
    for (uint32_t i = 0; i < 100000u && !(USART2->ISR & USART_ISR_TXE_TXFNF); i++) { }
    USART2->TDR = (uint8_t)c;
}

static void raw_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') { raw_putc('\r'); }
        raw_putc(*s++);
    }
}

static void raw_hex(uint32_t v)
{
    raw_puts("0x");
    for (int i = 28; i >= 0; i -= 4) { raw_putc("0123456789ABCDEF"[(v >> i) & 0xFu]); }
}

/* Every reset this firmware asks for goes through here, so the next boot
 * knows why - whatever RCC->CSR2 does or does not say. */
void crash_reset(uint32_t why)
{
    crash_rec.reset_why = why;
    crash_rec_seal();
    NVIC_SystemReset();
}

/* ===========================================================================
 *  HardFault
 * =========================================================================== */
__attribute__((used)) uint32_t crash_hi[8];     /* r4..r11 at the fault       */

__attribute__((used)) void crash_from_hardfault(const uint32_t *frame, uint32_t exc_return)
{
    regs_t *r = &crash_rec.regs;

    crash_rec.kind      = REC_HARDFAULT;
    crash_rec.which     = crash_rec.armed;
    crash_rec.hidden    = crash_rec.armed_mystery;
    crash_rec.armed     = 0;
    crash_rec.armed_mystery = 0;
    crash_rec.fresh     = 1;
    crash_rec.crashes++;
    crash_rec.uptime_ms = ms_now;
    crash_rec.code_ok   = 0;
    memset(r, 0, sizeof *r);
    r->exc_return = exc_return;

    /* Read the frame only if it is in RAM.  A fault with a wild stack pointer
     * would fault again right here - and a fault inside HardFault is LOCKUP. */
    if (mem_can((uint32_t)frame, 32u, MEM_R | MEM_W)) {
        r->r[0] = frame[0]; r->r[1] = frame[1]; r->r[2] = frame[2]; r->r[3] = frame[3];
        r->r[12] = frame[4];
        r->lr    = frame[5];
        r->pc    = frame[6];
        r->xpsr  = frame[7];
        /* sp before the push: 32 bytes of frame, plus 4 of padding if the
         * hardware had to realign the stack to 8 (stacked xPSR bit 9). */
        r->sp    = (uint32_t)frame + 32u + ((r->xpsr >> 9) & 1u) * 4u;
    }
    for (unsigned i = 0; i < 8u; i++) { r->r[4 + i] = crash_hi[i]; }

    /* The instruction at pc - the evidence CFSR would have summarised. */
    if (!(r->pc & 1u) && mem_can(r->pc, 2u, MEM_R | MEM_X)) {
        crash_rec.hw[0]  = *(const volatile uint16_t *)r->pc;
        crash_rec.hw[1]  = mem_can(r->pc + 2u, 2u, MEM_R) ? *(const volatile uint16_t *)(r->pc + 2u) : 0u;
        crash_rec.code_ok = 1;
    }
    crash_rec_seal();

    raw_puts("\n*** HardFault  pc=");  raw_hex(r->pc);
    raw_puts("  lr=");                 raw_hex(r->lr);
    raw_puts("\n*** recorded in .noinit - resetting; the report follows the reboot\n");

#if RESET_ON_FAULT
    crash_reset(RW_FAULT);
#else
    for (;;) { }        /* parked: attach GDB.  The IWDG will reset us anyway. */
#endif
}

/* Naked: no prologue, so r4-r11 and the stack pointers are exactly as the
 * hardware left them.  Thumb-1 can only STM the low registers, so r8-r11 are
 * moved down through r4-r7 (already saved) first. */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "   .syntax unified          \n"
        "   ldr   r3, =crash_hi      \n"
        "   stmia r3!, {r4-r7}       \n"     /* r4..r7                         */
        "   mov   r4, r8             \n"
        "   mov   r5, r9             \n"
        "   mov   r6, r10            \n"
        "   mov   r7, r11            \n"
        "   stmia r3!, {r4-r7}       \n"     /* r8..r11                        */
        "   movs  r0, #4             \n"
        "   mov   r1, lr             \n"
        "   tst   r0, r1             \n"     /* EXC_RETURN bit 2: which stack? */
        "   beq   1f                 \n"
        "   mrs   r0, psp            \n"
        "   b     2f                 \n"
        "1: mrs   r0, msp            \n"
        "2: mov   r1, lr             \n"     /* arg 2: EXC_RETURN              */
        "   ldr   r2, =crash_from_hardfault \n"
        "   bx    r2                 \n"
        "   .align 2                 \n"
        "   .ltorg                   \n"
    );
}

/* ===========================================================================
 *  The stack canary
 * =========================================================================== */
void stack_paint(void)
{
    uint32_t *p   = &_stack_floor;
    uint32_t *top = (uint32_t *)(__get_MSP() - 64u);   /* leave our own frame */
    while (p < top) { *p++ = STACK_FILL; }
}

int stack_guard_ok(void)
{
    const uint32_t *p = &_stack_floor;
    for (unsigned i = 0; i < GUARD_WORDS; i++) {
        if (p[i] != STACK_FILL) { return 0; }
    }
    return 1;
}

uint32_t stack_size(void) { return (uint32_t)&_estack - (uint32_t)&_stack_floor; }

uint32_t stack_used(void)
{
    const uint32_t *p = &_stack_floor, *end = &_estack;
    while (p < end && *p == STACK_FILL) { p++; }
    return (uint32_t)end - (uint32_t)p;
}

void stack_overflow_report(void)
{
    crash_rec.kind      = REC_STACK;
    crash_rec.which     = crash_rec.armed;
    crash_rec.hidden    = 0;
    crash_rec.armed     = 0;
    crash_rec.fresh     = 1;
    crash_rec.crashes++;
    crash_rec.uptime_ms = ms_now;
    memset(&crash_rec.regs, 0, sizeof crash_rec.regs);
    crash_rec.regs.sp   = (uint32_t)&_stack_floor;
    crash_rec.code_ok   = 0;
    crash_rec_seal();
    raw_puts("\n*** STACK OVERFLOW: the canary at _stack_floor is damaged - resetting\n");
    crash_reset(RW_STACK);
}

/* Each level costs ~48 bytes (32 of pad, plus saved registers).  It digs
 * until its locals are BELOW `stop`, then returns normally.  Nothing faults:
 * the memory it overwrote was heap, which is legal to write.  That is what
 * makes overflow dangerous - and why only the canary notices. */
__attribute__((noinline)) static uint32_t dig(uint32_t stop)
{
    volatile uint32_t pad[8];
    for (unsigned i = 0; i < 8u; i++) { pad[i] = i; }
    if ((uint32_t)&pad[0] > stop) { return dig(stop) + pad[7]; }   /* not a tail call */
    return pad[0];
}

/* ===========================================================================
 *  The realistic bug (X_PACKET)
 * ===========================================================================
 * A packet arrives as bytes: [0xA5][len][payload: len bytes of uint32 LE].
 * Someone sums the payload as words, by casting.  It compiles cleanly with
 * -Wall -Wextra.  Find the line from the dump alone - Lab Part 6. */
static uint8_t packet[12] __attribute__((aligned(4))) =
    { 0xA5, 8, 1, 0, 0, 0, 2, 0, 0, 0, 0x55, 0 };

__attribute__((noinline)) uint32_t packet_sum(const uint8_t *pkt)
{
    uint8_t len = pkt[1];
    const uint32_t *payload = (const uint32_t *)(pkt + 2);
    uint32_t sum = 0;
    for (unsigned i = 0; i < len / 4u; i++) { sum += payload[i]; }
    return sum;
}

/* ===========================================================================
 *  The experiments
 * =========================================================================== */
static volatile uint32_t sink;                      /* results nobody reads   */
static uint32_t words[2] __attribute__((aligned(4))) = { 0x11223344u, 0x55667788u };


void crash_run(unsigned n, int mystery)
{
    /* Arm first: if this never returns, the record knows what was running.
     * (Volatile addresses stop the compiler proving the access is undefined
     * and replacing it with a trap of its own choosing.) */
    volatile uintptr_t a;
    crash_rec.armed         = n + 1u;
    crash_rec.armed_mystery = (uint32_t)mystery;
    crash_rec_seal();

    switch (n) {
    case X_NULL_READ:
        a = 0;
        sink = *(volatile uint32_t *)a;
        printf("read *(uint32_t *)0 = 0x%08lX - no fault.  Address 0 aliases flash here:\n"
               "  that is word 0 of the vector table, the initial stack pointer (&_estack = 0x%08lX).\n",
               (unsigned long)sink, (unsigned long)(uint32_t)&_estack);
        break;

    case X_UNALIGNED:
        a = (uintptr_t)words + 1u;                  /* 1 byte into a word     */
        sink = *(volatile uint32_t *)a;             /* LDR: Armv6-M faults    */
        printf("unaligned load returned 0x%08lX - this core did NOT fault (a simulator?)\n",
               (unsigned long)sink);
        break;

    case X_WILD_READ:
        a = WILD_ADDR;
        sink = *(volatile uint32_t *)a;
        printf("read 0x%08lX = 0x%08lX - no bus error from this memory system\n",
               (unsigned long)WILD_ADDR, (unsigned long)sink);
        break;

    case X_FLASH_STORE: {
        a = FLASH_TEST_ADDR;
        *(volatile uint32_t *)a = 0x12345678u;      /* flash is locked: what now? */
        uint32_t sr = FLASH->SR;
        printf("store to flash 0x%08lX did NOT fault.  It now reads 0x%08lX;\n"
               "  FLASH->SR = 0x%08lX (PGSERR bit 7, WRPERR bit 4, PROGERR bit 3).\n",
               (unsigned long)FLASH_TEST_ADDR, (unsigned long)*(volatile uint32_t *)a, (unsigned long)sr);
        FLASH->SR = sr & (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                          FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
        break;
    }

    case X_NULL_CALL:
        a = 0;                                      /* a callback never set   */
        ((void (*)(void))a)();                      /* BLX to 0: bit 0 clear  */
        break;

    case X_EXEC_PERIPH:
        a = (uintptr_t)RCC | 1u;                    /* Thumb bit SET: still bad */
        ((void (*)(void))a)();
        break;

    case X_UDF:
        __asm volatile("udf #11");
        break;

    case X_BKPT:
        __asm volatile("bkpt #0x11");
        printf("bkpt returned: a debugger caught it and you pressed continue.\n");
        break;

    case X_STACK:
        printf("digging %lu bytes below the stack floor...\n", 32ul);
        sink = dig((uint32_t)&_stack_floor - 32u);
        if (!stack_guard_ok()) { stack_overflow_report(); }
        printf("canary intact?! (the dig did not reach it)\n");
        break;

    case X_HANG:
        printf("hanging, interrupts ON...  only a watchdog can end this.\n");
        for (;;) { __NOP(); }

    case X_HANG_NOIRQ:
        /* SysTick cannot run, so the soft dog cannot count.  Only a hardware
         * dog - one with its own clock - ends this. */
        printf("hanging, interrupts OFF...  the soft dog is now blind.\n");
        __disable_irq();
        for (;;) { __NOP(); }

    case X_PACKET:
        sink = packet_sum(packet);
        printf("packet sum = %lu - no fault?!\n", (unsigned long)sink);
        break;

    case X_DIV0: {
        volatile int zero = 0, hundred = 100;
        int q = hundred / zero;                     /* __aeabi_idiv: a CALL   */
        printf("100 / 0 = %d - no trap.  The M0+ has no divide instruction; libgcc's\n"
               "  __aeabi_idiv handled it, and returned what you see.\n", q);
        break;
    }

    default:
        break;
    }

    crash_rec.armed = 0;                            /* survived: disarm       */
    crash_rec.armed_mystery = 0;
    crash_rec_seal();
}
