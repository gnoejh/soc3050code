# INLINE ASSEMBLY FLOW DIAGRAMS
**ATmega128 Embedded Systems | SOC 3050**

---

## VISUAL LEARNING AIDS FOR INLINE ASSEMBLY

This document contains ASCII art diagrams to help students understand inline assembly concepts, execution flow, register usage, and optimization techniques.

---

## DIAGRAM 1: INLINE ASSEMBLY EXECUTION MODEL

```
┌────────────────────────────────────────────────────────────┐
│                   C PROGRAM COMPILATION                    │
└────────────────────────────────────────────────────────────┘
                            │
                            ▼
    ┌──────────────────────────────────────────┐
    │         C Source Code (Main.c)           │
    │  ┌────────────────────────────────────┐  │
    │  │  uint8_t result;                   │  │
    │  │  __asm__ volatile (                │  │
    │  │      "ldi r16, 0xFF \n\t"         │  │◄─── Inline Assembly
    │  │      "mov %0, r16   \n\t"         │  │     Inserted Here
    │  │      : "=r" (result)               │  │
    │  │      :                             │  │
    │  │      : "r16"                       │  │
    │  │  );                                │  │
    │  └────────────────────────────────────┘  │
    └──────────────────────────────────────────┘
                            │
                            ▼
    ┌──────────────────────────────────────────┐
    │          GCC COMPILER (avr-gcc)          │
    │                                          │
    │  Step 1: Parse C code                   │
    │  Step 2: Encounter __asm__ block        │
    │  Step 3: Allocate registers per         │
    │          constraints                     │
    │  Step 4: Substitute %0, %1 with         │
    │          actual registers                │
    │  Step 5: Insert assembly code directly  │
    │          into output                     │
    └──────────────────────────────────────────┘
                            │
                            ▼
    ┌──────────────────────────────────────────┐
    │         Generated Assembly Code          │
    │                                          │
    │  demo_function:                          │
    │    push r28                              │
    │    push r29                              │
    │    ldi r16, 0xFF     ◄──┐               │
    │    mov r24, r16      ◄──┤ Your inline   │
    │    pop r29              │ assembly       │
    │    pop r28              │ inserted here  │
    │    ret                  │                │
    └──────────────────────────────────────────┘
                            │
                            ▼
    ┌──────────────────────────────────────────┐
    │       Machine Code (Main.hex)            │
    │                                          │
    │  :10000000... (hex bytes)                │
    │  :10001000... (hex bytes)                │
    └──────────────────────────────────────────┘
                            │
                            ▼
    ┌──────────────────────────────────────────┐
    │         ATmega128 Execution              │
    │                                          │
    │  CPU fetches and executes instructions  │
    │  Register r24 contains result           │
    └──────────────────────────────────────────┘
```

---

## DIAGRAM 2: REGISTER ALLOCATION & CONSTRAINTS

```
┌────────────────────────────────────────────────────────────┐
│             AVR REGISTER FILE (32 registers)               │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  r0-r1   [××]  Reserved for multiply results              │
│  r2-r15  [  ]  Free for general use (any 'r')             │
│          [  ]                                              │
│          [  ]                                              │
│  r16-r23 [  ]  Upper registers (can use 'd' constraint)   │
│          [  ]  Support immediate operations (SUBI, ANDI)  │
│          [  ]                                              │
│  r24-r25 [  ]  Often used for function return ('w')       │
│  r26-r27 [XX]  X pointer register ('x' constraint)        │
│  r28-r29 [YY]  Y pointer register ('y' constraint)        │
│  r30-r31 [ZZ]  Z pointer register ('z' constraint)        │
│                 Used for LPM (flash memory access)        │
└────────────────────────────────────────────────────────────┘

CONSTRAINT EXAMPLES:

1. "r" constraint (any register):
   __asm__ ("mov %0, %1" : "=r" (a) : "r" (b));
   Compiler may choose: r0-r31 (any available)

2. "d" constraint (upper registers):
   __asm__ ("subi %0, 5" : "+d" (val));
   Compiler MUST choose: r16-r31
   Why? SUBI only works with r16-r31!

3. "x" constraint (X pointer):
   __asm__ ("ld %0, X+" : "=r" (data), "+x" (ptr));
   Compiler MUST use: r26:r27

4. "I" constraint (6-bit immediate):
   __asm__ ("sbi %0, %1" : : "I" (0x18), "I" (3));
   Compiler expects: compile-time constants 0-63
```

---

## DIAGRAM 3: INPUT/OUTPUT OPERAND FLOW

```
C VARIABLE → CONSTRAINT → REGISTER → ASSEMBLY → REGISTER → C VARIABLE
                                    INSTRUCTION

Example: result = a + b

    C Code:
    ┌──────────────────────────────────────────┐
    │ uint8_t a = 10, b = 20, result;          │
    │ __asm__ volatile (                       │
    │     "add %0, %1 \n\t"                    │
    │     : "=r" (result)      // %0           │
    │     : "r" (a), "0" (b)   // %1, reuse %0 │
    │ );                                       │
    └──────────────────────────────────────────┘
                │
                ▼
    ┌──────────────────────────────────────────┐
    │     COMPILER REGISTER ALLOCATION         │
    │                                          │
    │  a       → r24  (input)                  │
    │  b       → r22  (input, but reuse %0)    │
    │  result  → r22  (output, same as b)      │
    └──────────────────────────────────────────┘
                │
                ▼
    ┌──────────────────────────────────────────┐
    │      GENERATED ASSEMBLY CODE             │
    │                                          │
    │  mov r22, #20    ; b = 20                │
    │  mov r24, #10    ; a = 10                │
    │  add r22, r24    ; Your inline asm       │
    │  mov result, r22 ; result ← r22          │
    └──────────────────────────────────────────┘
                │
                ▼
    ┌──────────────────────────────────────────┐
    │           EXECUTION RESULT               │
    │                                          │
    │  result = 30                             │
    └──────────────────────────────────────────┘

KEY POINTS:
- %0 refers to first operand (result)
- %1 refers to second operand (a)
- "0" in input means "same register as %0"
- Compiler handles all register assignments
```

---

## DIAGRAM 4: MEMORY ADDRESSING MODES

```
┌────────────────────────────────────────────────────────────┐
│              ATMEGA128 MEMORY ARCHITECTURE                 │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────┐  0x0000                                 │
│  │   Registers  │  ◄─── IN/OUT instructions               │
│  │   (32 regs)  │        Fastest access (1 cycle)         │
│  ├──────────────┤  0x0020                                 │
│  │ I/O Registers│  ◄─── IN/OUT, SBI/CBI (low I/O)        │
│  │   (64 bytes) │        or LDS/STS (extended I/O)        │
│  ├──────────────┤  0x0060                                 │
│  │ Extended I/O │  ◄─── LDS/STS only                      │
│  ├──────────────┤  0x0100                                 │
│  │   SRAM       │  ◄─── LD/ST with X/Y/Z pointers        │
│  │   (4KB)      │        LDS/STS direct addressing        │
│  │              │                                          │
│  └──────────────┘  0x10FF                                 │
│                                                            │
│  ┌──────────────┐  0x0000                                 │
│  │  Flash       │  ◄─── LPM (Load Program Memory)        │
│  │  (128KB)     │        Using Z pointer (r30:r31)        │
│  └──────────────┘  0x1FFFF                                │
└────────────────────────────────────────────────────────────┘

ADDRESSING EXAMPLES:

1. Direct Register:
   __asm__ ("ldi r16, 0xFF");   // Load immediate to r16

2. I/O Register (IN/OUT):
   __asm__ ("in %0, %1" : "=r" (val) : "I" (_SFR_IO_ADDR(PORTB)));

3. I/O Register (SBI/CBI):
   __asm__ ("sbi %0, 3" : : "I" (_SFR_IO_ADDR(PORTB)));

4. Direct SRAM (LDS/STS):
   __asm__ ("lds r16, 0x0200");  // Load from SRAM address 0x0200

5. Indirect via X pointer:
   __asm__ ("ld %0, X+" : "=r" (data), "+x" (ptr));

6. Indirect via Z pointer (Flash):
   __asm__ ("lpm %0, Z" : "=r" (data) : "z" (flash_addr));
```

---

## DIAGRAM 5: CLOBBER LIST IMPORTANCE

```
WITHOUT CLOBBER LIST (WRONG):
┌────────────────────────────────────────────────────────────┐
│  uint8_t result;                                           │
│  __asm__ volatile (                                        │
│      "ldi r16, 0xFF \n\t"   ◄─── Modifies r16             │
│      "mov %0, r16   \n\t"                                  │
│      : "=r" (result)                                       │
│  );                         ◄─── NO CLOBBER LIST!          │
│  // Compiler doesn't know r16 was modified                │
└────────────────────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────────────────────┐
│              COMPILER OPTIMIZATION                         │
│                                                            │
│  mov r16, important_value  ; Compiler thinks r16 is safe  │
│  <your inline assembly>    ; OOPS! You clobber r16        │
│  use r16 here              ; Wrong value!                 │
└────────────────────────────────────────────────────────────┘

WITH CLOBBER LIST (CORRECT):
┌────────────────────────────────────────────────────────────┐
│  uint8_t result;                                           │
│  __asm__ volatile (                                        │
│      "ldi r16, 0xFF \n\t"                                  │
│      "mov %0, r16   \n\t"                                  │
│      : "=r" (result)                                       │
│      :                                                     │
│      : "r16"                ◄─── CLOBBER LIST              │
│  );                                                        │
│  // Compiler knows r16 is modified                        │
└────────────────────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────────────────────┐
│              COMPILER OPTIMIZATION                         │
│                                                            │
│  push r16                   ; Compiler saves r16 first    │
│  <your inline assembly>     ; Safe to modify r16          │
│  pop r16                    ; Compiler restores r16       │
└────────────────────────────────────────────────────────────┘

CLOBBER LIST CONTENTS:
- Individual registers: "r16", "r17", "r24"
- "memory": If assembly writes to memory
- "cc": If assembly modifies condition codes (SREG flags)
```

---

## DIAGRAM 6: 16-BIT ARITHMETIC WITH CARRY

```
┌────────────────────────────────────────────────────────────┐
│          16-BIT ADDITION: val16_a + val16_b                │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  val16_a = 0x12AF  (4783 decimal)                          │
│  val16_b = 0x0E51  (3665 decimal)                          │
│  result16 = ?      (should be 8448 decimal = 0x2100)       │
│                                                            │
│  ┌─────────────────────────────────────┐                  │
│  │  High Byte  │  Low Byte             │                  │
│  ├─────────────┼───────────────────────┤                  │
│  │    0x12     │   0xAF    val16_a     │                  │
│  │    0x0E     │   0x51    val16_b     │                  │
│  └─────────────┴───────────────────────┘                  │
│                                                            │
│  STEP 1: ADD LOW BYTES                                     │
│  ┌───────────────────────────────┐                        │
│  │  0xAF + 0x51 = 0x100          │                        │
│  │  Result: 0x00                 │                        │
│  │  Carry: 1  ◄─── IMPORTANT!    │                        │
│  └───────────────────────────────┘                        │
│                                                            │
│  STEP 2: ADD HIGH BYTES WITH CARRY                         │
│  ┌───────────────────────────────┐                        │
│  │  0x12 + 0x0E + 1 = 0x21       │                        │
│  │  Result: 0x21                 │                        │
│  │  Carry: 0                     │                        │
│  └───────────────────────────────┘                        │
│                                                            │
│  FINAL RESULT: 0x2100 = 8448 decimal ✓                    │
└────────────────────────────────────────────────────────────┘

ASSEMBLY CODE:
┌────────────────────────────────────────────────────────────┐
│  __asm__ volatile (                                        │
│      "add %A0, %A1 \n\t"    ; Add low bytes (sets carry)  │
│      "adc %B0, %B1 \n\t"    ; Add high bytes + carry      │
│      : "=r" (result16)                                     │
│      : "r" (val16_a), "0" (val16_b)                        │
│  );                                                        │
└────────────────────────────────────────────────────────────┘

REGISTER USAGE:
┌────────────────────────────────────────────────────────────┐
│  %A0 = Low byte of result16   (e.g., r22)                 │
│  %B0 = High byte of result16  (e.g., r23)                 │
│  %A1 = Low byte of val16_a    (e.g., r24)                 │
│  %B1 = High byte of val16_a   (e.g., r25)                 │
└────────────────────────────────────────────────────────────┘
```

---

## DIAGRAM 7: INLINE ASSEMBLY vs C PERFORMANCE

```
TASK: Toggle LED 1000 times
┌────────────────────────────────────────────────────────────┐
│                    C VERSION                               │
├────────────────────────────────────────────────────────────┤
│  for (i = 0; i < 1000; i++) {                              │
│      PORTB ^= (1 << 0);    // Toggle LED                   │
│  }                                                          │
│                                                            │
│  DISASSEMBLY:                                              │
│    in r24, 0x18       ; 1 cycle  - Read PORTB             │
│    ldi r25, 0x01      ; 1 cycle  - Load mask              │
│    eor r24, r25       ; 1 cycle  - XOR                     │
│    out 0x18, r24      ; 1 cycle  - Write PORTB            │
│    [loop overhead]    ; 3-4 cycles                         │
│  ────────────────────────────────────────                 │
│  Per iteration: ~8 cycles                                  │
│  Total: 8000 cycles = 500 µs @ 16MHz                       │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│              INLINE ASSEMBLY VERSION                       │
├────────────────────────────────────────────────────────────┤
│  for (i = 0; i < 1000; i++) {                              │
│      __asm__ volatile (                                    │
│          "sbi %0, 0 \n\t"   // Set bit                     │
│          "cbi %0, 0 \n\t"   // Clear bit                   │
│          : : "I" (_SFR_IO_ADDR(PORTB))                     │
│      );                                                    │
│  }                                                          │
│                                                            │
│  DISASSEMBLY:                                              │
│    sbi 0x18, 0        ; 1 cycle  - Set bit (atomic)       │
│    cbi 0x18, 0        ; 1 cycle  - Clear bit (atomic)     │
│    [loop overhead]    ; 3-4 cycles                         │
│  ────────────────────────────────────────                 │
│  Per iteration: ~6 cycles                                  │
│  Total: 6000 cycles = 375 µs @ 16MHz                       │
└────────────────────────────────────────────────────────────┘

PERFORMANCE IMPROVEMENT: 25% faster!

WHY ASSEMBLY IS FASTER:
1. SBI/CBI are atomic (no read-modify-write)
2. Direct bit manipulation (no mask calculation)
3. Fewer instructions per operation
4. Compiler cannot optimize C code as aggressively

WHEN TO USE ASSEMBLY:
✓ Critical timing paths
✓ Interrupt service routines
✓ Bit-level hardware control
✓ Performance-critical loops
✗ Complex logic (use C for readability)
✗ Portability matters (assembly is CPU-specific)
```

---

## DIAGRAM 8: POINTER REGISTER USAGE

```
┌────────────────────────────────────────────────────────────┐
│              X, Y, Z POINTER REGISTERS                     │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  X Pointer: r26 (XL) : r27 (XH)                            │
│  Y Pointer: r28 (YL) : r29 (YH)  ◄─── Also frame pointer  │
│  Z Pointer: r30 (ZL) : r31 (ZH)  ◄─── Used for LPM        │
│                                                            │
└────────────────────────────────────────────────────────────┘

EXAMPLE: Array Summation with X Pointer
┌────────────────────────────────────────────────────────────┐
│  uint8_t array[5] = {10, 20, 30, 40, 50};                  │
│  uint8_t *ptr = array;                                     │
│  uint8_t sum = 0;                                          │
│                                                            │
│  Memory Layout:                                            │
│  ┌─────┬─────┬─────┬─────┬─────┐                         │
│  │ 10  │ 20  │ 30  │ 40  │ 50  │                         │
│  └─────┴─────┴─────┴─────┴─────┘                         │
│    ▲                                                       │
│    │                                                       │
│   ptr (loaded into X: r26:r27)                            │
│                                                            │
│  __asm__ volatile (                                        │
│      "ld r16, X+  \n\t"   ; Load 10, X++                  │
│      "add %0, r16 \n\t"   ; sum += 10                     │
│      "ld r16, X+  \n\t"   ; Load 20, X++                  │
│      "add %0, r16 \n\t"   ; sum += 20                     │
│      "ld r16, X+  \n\t"   ; Load 30, X++                  │
│      "add %0, r16 \n\t"   ; sum += 30                     │
│      : "+r" (sum), "+x" (ptr)                             │
│      :                                                     │
│      : "r16"                                               │
│  );                                                        │
│                                                            │
│  Result: sum = 60, ptr points to array[3]                 │
└────────────────────────────────────────────────────────────┘

POINTER MODES:
┌────────────────────────────────────────────────────────────┐
│  ld Rd, X      Load from [X], X unchanged                  │
│  ld Rd, X+     Load from [X], then X = X + 1 (post-inc)   │
│  ld Rd, -X     X = X - 1, then load from [X] (pre-dec)    │
│  ld Rd, Y+q    Load from [Y + q], Y unchanged (offset)    │
│  ld Rd, Z+q    Load from [Z + q], Z unchanged             │
│  st X, Rr      Store Rr to [X]                            │
│  lpm Rd, Z     Load from Flash memory at [Z]              │
└────────────────────────────────────────────────────────────┘
```

---

## DIAGRAM 9: INTERRUPT SERVICE ROUTINE WITH ASSEMBLY

```
┌────────────────────────────────────────────────────────────┐
│           INTERRUPT HANDLING FLOW                          │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  Main Program                  Interrupt Occurs            │
│  ┌────────────┐               ┌──────────────┐            │
│  │ while(1) { │               │ Hardware      │            │
│  │   // code  │───────────────►│ detects INT0 │            │
│  │ }          │               └──────────────┘            │
│  └────────────┘                       │                    │
│                                       ▼                    │
│                            ┌─────────────────────┐         │
│                            │ Save SREG & regs    │         │
│                            │ (automatically)     │         │
│                            └─────────────────────┘         │
│                                       │                    │
│                                       ▼                    │
│                            ┌─────────────────────┐         │
│                            │ ISR(INT0_vect) {    │         │
│                            │   __asm__ volatile( │         │
│                            │     "inc %0 \n\t"   │         │
│                            │     : "+r" (count)  │         │
│                            │   );                │         │
│                            │ }                   │         │
│                            └─────────────────────┘         │
│                                       │                    │
│                                       ▼                    │
│                            ┌─────────────────────┐         │
│                            │ RETI instruction    │         │
│                            │ Restore SREG & regs │         │
│                            └─────────────────────┘         │
│                                       │                    │
│  ┌────────────┐                       │                    │
│  │ while(1) { │◄──────────────────────┘                    │
│  │   // code  │  Resume execution                          │
│  │ }          │                                            │
│  └────────────┘                                            │
└────────────────────────────────────────────────────────────┘

ASSEMBLY ISR EXAMPLE:
┌────────────────────────────────────────────────────────────┐
│  volatile uint8_t interrupt_count = 0;                     │
│                                                            │
│  ISR(INT0_vect)                                            │
│  {                                                         │
│      // Increment counter using inline assembly           │
│      __asm__ volatile (                                    │
│          "lds r16, interrupt_count \n\t"  ; Load from RAM │
│          "inc r16                  \n\t"  ; Increment     │
│          "sts interrupt_count, r16 \n\t"  ; Store to RAM  │
│          :                                                 │
│          :                                                 │
│          : "r16", "memory"                                 │
│      );                                                    │
│                                                            │
│      // Toggle LED with assembly                           │
│      __asm__ volatile (                                    │
│          "in r17, %0       \n\t"          ; Read PORTB    │
│          "ldi r18, 0x01    \n\t"          ; Mask          │
│          "eor r17, r18     \n\t"          ; Toggle bit 0  │
│          "out %0, r17      \n\t"          ; Write PORTB   │
│          :                                                 │
│          : "I" (_SFR_IO_ADDR(PORTB))                       │
│          : "r17", "r18"                                    │
│      );                                                    │
│  }                                                         │
└────────────────────────────────────────────────────────────┘

KEY POINTS:
- ISR automatically saves/restores SREG and used registers
- Keep ISR short and fast
- Use "memory" clobber if accessing volatile variables
- Assembly in ISR can reduce latency
```

---

## DIAGRAM 10: OPTIMIZATION DECISION TREE

```
                    Need to optimize code?
                            │
                            ▼
              ┌─────────────────────────┐
              │ Is it time-critical?    │
              │ (ISR, timing loop)      │
              └─────────────────────────┘
                     │             │
                     │ Yes         │ No
                     ▼             ▼
           ┌──────────────┐   ┌────────────────┐
           │ Profile code │   │ Use C code     │
           │ Find hotspot │   │ for readability│
           └──────────────┘   └────────────────┘
                     │
                     ▼
         ┌────────────────────────┐
         │ Can compiler optimize? │
         │ (try -O3 flag)         │
         └────────────────────────┘
                     │
            Yes ─────┴───── No
             │               │
             ▼               ▼
    ┌────────────────┐  ┌──────────────────┐
    │ Use C code     │  │ Consider assembly│
    │ Compiler wins! │  │ optimization     │
    └────────────────┘  └──────────────────┘
                                 │
                                 ▼
                    ┌──────────────────────┐
                    │ Type of operation?   │
                    └──────────────────────┘
                            │
         ┌──────────────────┼──────────────────┐
         │                  │                  │
         ▼                  ▼                  ▼
    Bit manip         Math ops          I/O access
         │                  │                  │
         ▼                  ▼                  ▼
  Use SBI/CBI        Use shifts         Use IN/OUT
  LSL/LSR/SWAP      for multiply       Direct SBI/CBI
    (faster!)        by powers of 2      (atomic!)

OPTIMIZATION CHECKLIST:
┌────────────────────────────────────────────────────────────┐
│ ✓ Profile first - measure before optimizing               │
│ ✓ Optimize hotspots only - don't waste time               │
│ ✓ Test thoroughly - assembly bugs are subtle              │
│ ✓ Comment heavily - explain WHY assembly used             │
│ ✓ Benchmark - prove assembly is actually faster           │
│ ✓ Consider maintainability - sometimes C is better        │
└────────────────────────────────────────────────────────────┘

EXAMPLE BENCHMARKS (ATmega128 @ 16MHz):
┌────────────────────────────────────────────────────────────┐
│ Operation          │ C Code  │ Assembly │ Improvement      │
├────────────────────┼─────────┼──────────┼──────────────────┤
│ Set single bit     │ 5 cyc   │ 1 cyc    │ 5x faster        │
│ Multiply by 8      │ 12 cyc  │ 3 cyc    │ 4x faster (LSL)  │
│ Swap nibbles       │ 8 cyc   │ 1 cyc    │ 8x faster (SWAP) │
│ Toggle LED         │ 8 cyc   │ 4 cyc    │ 2x faster        │
│ 16-bit addition    │ 6 cyc   │ 4 cyc    │ 1.5x faster      │
└────────────────────────────────────────────────────────────┘
```

---

## SUMMARY: KEY TAKEAWAYS

1. **Inline Assembly Syntax**: Master `__asm__ volatile()` with constraints
2. **Register Constraints**: Choose right constraint (r, d, x, y, z, I, M)
3. **Clobber Lists**: Always list modified registers and memory
4. **16-bit Operations**: Use %A0/%B0 for low/high bytes, ADC for carry
5. **Performance**: Assembly is faster for bit ops, shifts, atomic operations
6. **Pointers**: X/Y/Z for indirect addressing, LPM for flash access
7. **Interrupts**: Keep ISR fast, assembly reduces latency
8. **Optimization**: Profile first, optimize hotspots, benchmark results

---

**END OF FLOW DIAGRAMS**
*Study these diagrams before attempting lab exercises!*
