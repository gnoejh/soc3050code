/*
 * crash.h - the crash record, the HardFault reporter, and the experiments
 *                                                       (SOC3050 lesson 11)
 */
#ifndef CRASH_H
#define CRASH_H

#include <stdint.h>
#include "diag.h"

/* ---- the experiments: one number each, the same on serial and the OLED ---- */
enum {
    X_NULL_READ = 0,    /* read address 0: does NOT fault on this chip      */
    X_UNALIGNED,        /* LDR from an odd address                          */
    X_WILD_READ,        /* LDR from 0x60000000, where nothing is mapped     */
    X_FLASH_STORE,      /* STR into flash - fault, or silently ignored?     */
    X_NULL_CALL,        /* call a NULL function pointer: Thumb bit clear    */
    X_EXEC_PERIPH,      /* call into RCC's registers: Execute Never         */
    X_UDF,              /* a permanently undefined instruction              */
    X_BKPT,             /* a breakpoint with no debugger attached           */
    X_STACK,            /* recurse through the stack floor: the canary      */
    X_HANG,             /* an infinite loop: only the watchdog can help     */
    X_HANG_NOIRQ,       /* the same, interrupts OFF: the soft dog is blind  */
    X_STARVE,           /* one job stops checking in: does the dog notice?  */
    X_PACKET,           /* a realistic bug - find it from the dump          */
    X_DIV0,             /* divide by zero: no trap on the M0+               */
    X_MYSTERY,          /* a random HardFault, cause hidden: the game       */
    X_COUNT
};

/* ---- the record that survives a reset (.noinit, see link.ld) ------------- */
enum { REC_NONE = 0, REC_HARDFAULT, REC_STACK };

/* Why the firmware itself reset the chip - written just before it calls
 * NVIC_SystemReset().  RCC->CSR2 can only say "software reset"; this says
 * which software, and it still works where the simulator does not model the
 * reset flags at all.  A hardware watchdog reset leaves it at RW_NONE. */
enum { RW_NONE = 0, RW_FAULT, RW_STACK, RW_SOFTDOG, RW_COMMAND };

typedef struct {
    uint32_t magic;
    uint32_t boots;          /* resets since power-on                         */
    uint32_t crashes;        /* HardFaults + stack overflows recorded         */
    uint32_t right, tries;   /* the detective's score                         */
    uint32_t armed;          /* experiment running now, +1 (0 = none)         */
    uint32_t armed_mystery;
    uint32_t loop_ms;        /* ms_now at the main loop's latest pass        */
    uint32_t job_ms[3];      /* ms_now at each job's latest check-in         */
    uint32_t wdt_ms;         /* the watchdog timeout in force                */
    uint32_t dogs;           /* DOG_* armed                                   */
    uint32_t reset_why;      /* RW_*: set just before a firmware reset        */
    uint32_t kind;           /* REC_* of the last crash                       */
    uint32_t which;          /* experiment that caused it, +1 (0 = genuine)   */
    uint32_t hidden;         /* 1: a mystery, verdict not yet shown           */
    uint32_t fresh;          /* 1: not yet reported after the reset           */
    uint32_t uptime_ms;      /* when it happened                              */
    regs_t   regs;
    uint32_t code_ok;        /* 1: hw[] were read from the stacked pc         */
    uint16_t hw[2];          /* the instruction at the stacked pc             */
    uint32_t sum;            /* checksum of everything above                  */
} crash_rec_t;

extern crash_rec_t crash_rec;

int  crash_rec_load(void);   /* 1 if a valid record survived; boots++         */
void crash_rec_seal(void);   /* recompute the checksum after any change       */
void crash_rec_clear(void);  /* forget everything, score included             */

/* Run experiment n.  Most never return.  The ones that do print what happened. */
void crash_run(unsigned n, int mystery);

/* Stack watching (link.ld's _stack_floor .. _estack) */
void     stack_paint(void);
int      stack_guard_ok(void);
uint32_t stack_used(void);            /* bytes, high-water mark                */
uint32_t stack_size(void);
void     stack_overflow_report(void); /* record it and reset                   */

void     crash_reset(uint32_t why);   /* note RW_* in the record, then reset    */

uint32_t packet_sum(const uint8_t *pkt);   /* the bug of X_PACKET              */

#endif
