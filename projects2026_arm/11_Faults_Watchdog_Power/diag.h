/*
 * diag.h - the crash detective's tools, in pure C        (SOC3050 lesson 11)
 *
 * Nothing in this file or in diag.c includes a device header, so both compile
 * on a PC with plain gcc.  host/test.c does exactly that: it feeds the decoder
 * every 16-bit instruction objdump finds in Main.elf and checks it agrees.
 *
 * Four jobs:
 *
 *   1. thumb_decode()   read the halfword(s) at the stacked pc and say what
 *                       instruction it was - and, for a load or store, which
 *                       address it touched, computed from the saved registers.
 *   2. fault_classify() turn that, the stacked xPSR and the memory map into
 *                       one most-likely cause.  This is the job CFSR does in
 *                       hardware on an ARMv7-M part.  The Cortex-M0+ has no
 *                       CFSR, so here it is done by reading the evidence.
 *   3. reset_cause()    name the bits of RCC->CSR2.
 *   4. systick_elapsed() cycles between two SysTick->VAL readings, for the
 *                       idle-time (CPU load) measurement.
 */
#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>

/* ---- the register file at the moment of the fault ------------------------
 * r0-r3, r12, lr, pc and xPSR come from the 8-word frame the hardware pushed;
 * r4-r11 were saved by HardFault_Handler's first instructions, before any C
 * code could change them.  sp is the value the faulting code had. */
typedef struct {
    uint32_t r[13];          /* r0 .. r12                                    */
    uint32_t sp, lr, pc, xpsr;
    uint32_t exc_return;     /* lr on entry to the handler: 0xFFFFFFF1/9/D   */
} regs_t;

/* ---- instruction decoding --------------------------------------------- */
typedef enum {
    INS_OTHER = 0,           /* a 16-bit instruction that touches no memory  */
    INS_LOAD, INS_STORE,     /* LDR/STR family, one access                   */
    INS_LOAD_MULTI, INS_STORE_MULTI,   /* LDM, POP / STM, PUSH               */
    INS_BRANCH_REG,          /* BX / BLX Rm - where Thumb-bit faults start   */
    INS_UDF, INS_BKPT, INS_SVC,
    INS_BL,                  /* 32-bit BL                                    */
    INS_32BIT                /* another 32-bit encoding (MRS, MSR, DMB...)   */
} ins_kind_t;

typedef struct {
    ins_kind_t kind;
    uint8_t    len;          /* 2 or 4 bytes                                  */
    uint8_t    size;         /* bytes per access: 1, 2, 4; 0 if no access     */
    uint8_t    has_addr;     /* 1 if addr was computed from known registers   */
    uint8_t    nregs;        /* registers transferred, for LDM/STM/PUSH/POP   */
    uint32_t   addr;         /* lowest address accessed                       */
    char       mnem[12];     /* "ldr", "strh", "push" ... as objdump spells it */
    char       ops[40];      /* operands, as objdump prints them              */
} insn_t;

/* `r` may be NULL: the instruction is still decoded, without an address.
 * `pc` is the address of hw1 (needed for PC-relative loads). */
void thumb_decode(uint16_t hw1, uint16_t hw2, uint32_t pc, const regs_t *r, insn_t *out);

/* ---- the memory map, as far as a fault reporter needs it ---------------- */
enum { MEM_R = 1, MEM_W = 2, MEM_X = 4 };
typedef struct { uint32_t base, size; uint8_t access; const char *name; } region_t;

const region_t *mem_region(uint32_t addr);       /* NULL: nothing mapped there */
int  mem_can(uint32_t addr, uint32_t len, int access);

/* ---- the verdict ---------------------------------------------------------- */
typedef enum {
    CAUSE_UNKNOWN = 0,
    CAUSE_THUMB_BIT,         /* xPSR.T = 0: a branch to an even address        */
    CAUSE_BAD_FETCH,         /* pc is not in executable memory                 */
    CAUSE_UNALIGNED,         /* a halfword/word access to a misaligned address */
    CAUSE_BAD_ADDRESS,       /* a data access where nothing is mapped          */
    CAUSE_READ_ONLY,         /* a store to flash or another read-only region   */
    CAUSE_UNDEFINED,         /* UDF, or an encoding ARMv6-M does not have      */
    CAUSE_BKPT,              /* BKPT with no debugger attached                 */
    CAUSE_SVC,               /* SVC where SVCall could not be taken            */
    CAUSE_COUNT
} cause_t;

const char *cause_name(cause_t c);

/* `code_ok`: the halfwords at pc could be read (pc was in flash or RAM).
 * Fills *ins, writes a one-line explanation into why[why_len]. */
cause_t fault_classify(const regs_t *r, int code_ok, uint16_t hw1, uint16_t hw2,
                       insn_t *ins, char *why, unsigned why_len);

/* EXC_RETURN in words: "thread mode, main stack (MSP)" etc. */
const char *exc_return_text(uint32_t exc_return);

/* ---- reset cause: RCC->CSR2 bits 31..25 ------------------------------------
 * Copied from stm32c031xx.h so the host can compile this file; Main.c checks
 * every one against the real header with _Static_assert. */
#define RST_OBL   (1UL << 25)    /* RCC_CSR2_OBLRSTF  option-byte loader      */
#define RST_PIN   (1UL << 26)    /* RCC_CSR2_PINRSTF  NRST pin                */
#define RST_PWR   (1UL << 27)    /* RCC_CSR2_PWRRSTF  BOR or POR/PDR          */
#define RST_SFT   (1UL << 28)    /* RCC_CSR2_SFTRSTF  software (SYSRESETREQ)  */
#define RST_IWDG  (1UL << 29)    /* RCC_CSR2_IWDGRSTF independent watchdog    */
#define RST_WWDG  (1UL << 30)    /* RCC_CSR2_WWDGRSTF window watchdog         */
#define RST_LPWR  (1UL << 31)    /* RCC_CSR2_LPWRRSTF low-power entry         */
#define RST_ALL   (RST_OBL | RST_PIN | RST_PWR | RST_SFT | RST_IWDG | RST_WWDG | RST_LPWR)

/* The most specific cause set: IWDG beats SFT beats PIN, because on many
 * STM32s an internal reset also drives NRST and so sets PINRSTF too. */
const char *reset_cause(uint32_t csr2);
/* All set flags as "IWDG PIN" into buf. */
void reset_flags(uint32_t csr2, char *buf, unsigned len);

/* ---- SysTick arithmetic, for the idle-time measurement ---------------------
 * SysTick counts DOWN from LOAD to 0, then reloads.  Cycles from reading v0 to
 * reading v1, valid if less than one full period passed: */
static inline uint32_t systick_elapsed(uint32_t v0, uint32_t v1, uint32_t load)
{
    return (v0 >= v1) ? (v0 - v1) : (v0 + (load + 1u) - v1);
}

#endif
