# ATmega128 Assembly Instructions & C Equivalents
**Comprehensive Reference Guide for Students**

---

## TABLE OF CONTENTS
1. [Data Transfer Instructions](#1-data-transfer-instructions)
2. [Arithmetic Instructions](#2-arithmetic-instructions)
3. [Logical Instructions](#3-logical-instructions)
4. [Bit Operations](#4-bit-operations)
5. [Branch Instructions](#5-branch-instructions)
6. [Control Instructions](#6-control-instructions)
7. [Assembly Directives](#7-assembly-directives)
8. [Register Architecture](#8-register-architecture)
9. [Memory Map](#9-memory-map)
10. [Practical Examples](#10-practical-examples)

---

## 1. DATA TRANSFER INSTRUCTIONS

### Load and Store Operations

| Assembly | C Equivalent | Description | Cycles |
|----------|-------------|-------------|--------|
| `LDI Rd, K` | `uint8_t x = K;` | Load immediate (r16-r31 only) | 1 |
| `MOV Rd, Rr` | `x = y;` | Copy register | 1 |
| `LDS Rd, k` | `x = *(uint8_t*)0x1234;` | Load from SRAM | 2 |
| `STS k, Rr` | `*(uint8_t*)0x1234 = x;` | Store to SRAM | 2 |
| `LD Rd, X` | `x = *ptr;` | Load indirect via X | 2 |
| `LD Rd, X+` | `x = *ptr++;` | Load, post-increment | 2 |
| `LD Rd, -X` | `x = *--ptr;` | Pre-decrement, load | 2 |
| `LD Rd, Y+q` | `x = *(ptr + q);` | Load with displacement | 2 |
| `ST X, Rr` | `*ptr = x;` | Store indirect via X | 2 |
| `ST X+, Rr` | `*ptr++ = x;` | Store, post-increment | 2 |
| `ST -X, Rr` | `*--ptr = x;` | Pre-decrement, store | 2 |
| `LPM Rd, Z` | `x = pgm_read_byte(ptr);` | Load from program memory | 3 |
| `ELPM Rd, Z` | `x = pgm_read_byte_far(ptr);` | Extended load (>64KB) | 3 |

**Examples:**
```assembly
; Load immediate
LDI r16, 0xFF          ; r16 = 255
```
```c
uint8_t val = 0xFF;    // Equivalent in C
```

```assembly
; Array access with pointer
LD r17, X+             ; Load *ptr, then ptr++
```
```c
uint8_t val = *ptr++;  // Equivalent in C
```

---

### I/O Register Access

| Assembly | C Equivalent | Description | Cycles |
|----------|-------------|-------------|--------|
| `IN Rd, P` | `x = PORTB;` | Read from I/O register | 1 |
| `OUT P, Rr` | `PORTB = x;` | Write to I/O register | 1 |
| `SBI P, b` | `PORTB \|= (1<<b);` | Set bit in I/O | 1-2 |
| `CBI P, b` | `PORTB &= ~(1<<b);` | Clear bit in I/O | 1-2 |
| `SBIS P, b` | `if (PORTB & (1<<b))` | Skip if bit set | 1-3 |
| `SBIC P, b` | `if (!(PORTB & (1<<b)))` | Skip if bit clear | 1-3 |

**Examples:**
```assembly
; Set LED on PORTB
SBI PORTB, 0           ; PORTB.0 = 1 (atomic!)
```
```c
PORTB |= (1 << 0);     // C version (3-4 instructions)
```

```assembly
; Read button on PIND
IN r16, PIND           ; r16 = PIND
SBIS PIND, 2           ; Skip next if PIND.2 = 1
RJMP button_pressed    ; Execute if button = 0
```
```c
if (!(PIND & (1 << 2))) {  // C equivalent
    // button_pressed
}
```

---

## 2. ARITHMETIC INSTRUCTIONS

### Addition and Subtraction

| Assembly | C Equivalent | Description | Flags | Cycles |
|----------|-------------|-------------|-------|--------|
| `ADD Rd, Rr` | `x += y;` | Add without carry | ZCNVSH | 1 |
| `ADC Rd, Rr` | `x += y + carry;` | Add with carry | ZCNVSH | 1 |
| `ADIW Rd, K` | `pair += K;` | Add immediate to word (r24-r30) | ZCNVS | 2 |
| `SUB Rd, Rr` | `x -= y;` | Subtract | ZCNVSH | 1 |
| `SUBI Rd, K` | `x -= K;` | Subtract immediate (r16-r31) | ZCNVSH | 1 |
| `SBC Rd, Rr` | `x -= y - carry;` | Subtract with carry | ZCNVSH | 1 |
| `SBCI Rd, K` | `x -= K - carry;` | Subtract immediate with carry | ZCNVSH | 1 |
| `SBIW Rd, K` | `pair -= K;` | Subtract immediate from word | ZCNVS | 2 |
| `INC Rd` | `x++;` | Increment | ZNVS | 1 |
| `DEC Rd` | `x--;` | Decrement | ZNVS | 1 |

**Examples:**
```assembly
; 8-bit addition
LDI r16, 10            ; r16 = 10
LDI r17, 20            ; r17 = 20
ADD r16, r17           ; r16 = r16 + r17 = 30
```
```c
uint8_t a = 10, b = 20;
a = a + b;             // a = 30
```

```assembly
; 16-bit addition with carry
ADD r24, r26           ; Low byte:  r24 += r26
ADC r25, r27           ; High byte: r25 += r27 + carry
```
```c
uint16_t sum = a + b;  // Compiler generates similar code
```

---

### Multiplication and Division

| Assembly | C Equivalent | Description | Result | Cycles |
|----------|-------------|-------------|--------|--------|
| `MUL Rd, Rr` | `result = x * y;` | Unsigned multiply | r1:r0 | 2 |
| `MULS Rd, Rr` | `result = (int8_t)x * (int8_t)y;` | Signed multiply | r1:r0 | 2 |
| `MULSU Rd, Rr` | `result = (int8_t)x * (uint8_t)y;` | Signed × unsigned | r1:r0 | 2 |
| `FMUL Rd, Rr` | `result = (x * y) << 1;` | Fractional multiply | r1:r0 | 2 |
| `FMULS Rd, Rr` | `result = ((int8_t)x * (int8_t)y) << 1;` | Signed fractional | r1:r0 | 2 |
| `FMULSU Rd, Rr` | `result = ((int8_t)x * (uint8_t)y) << 1;` | Signed × unsigned frac | r1:r0 | 2 |

**Note:** ATmega128 has no division instruction. Division is done via repeated subtraction or library functions.

**Examples:**
```assembly
; Unsigned 8×8 = 16-bit multiplication
LDI r16, 25            ; r16 = 25
LDI r17, 4             ; r17 = 4
MUL r16, r17           ; r1:r0 = 25 × 4 = 100
```
```c
uint16_t result = 25 * 4;  // result = 100
```

```assembly
; Division by repeated subtraction (x / 5)
LDI r16, 23            ; Dividend = 23
LDI r17, 5             ; Divisor = 5
CLR r18                ; Quotient = 0
div_loop:
    CP r16, r17        ; Compare dividend with divisor
    BRLO div_done      ; If dividend < divisor, done
    SUB r16, r17       ; Dividend -= divisor
    INC r18            ; Quotient++
    RJMP div_loop
div_done:              ; r18 = 4 (quotient), r16 = 3 (remainder)
```
```c
uint8_t quotient = 23 / 5;   // quotient = 4
uint8_t remainder = 23 % 5;  // remainder = 3
```

---

## 3. LOGICAL INSTRUCTIONS

### Bitwise Operations

| Assembly | C Equivalent | Description | Flags | Cycles |
|----------|-------------|-------------|-------|--------|
| `AND Rd, Rr` | `x &= y;` | Logical AND | ZNV | 1 |
| `ANDI Rd, K` | `x &= K;` | AND with immediate (r16-r31) | ZNV | 1 |
| `OR Rd, Rr` | `x \|= y;` | Logical OR | ZNV | 1 |
| `ORI Rd, K` | `x \|= K;` | OR with immediate | ZNV | 1 |
| `EOR Rd, Rr` | `x ^= y;` | Exclusive OR | ZNV | 1 |
| `COM Rd` | `x = ~x;` | One's complement | ZCNV | 1 |
| `NEG Rd` | `x = -x;` | Two's complement | ZCNVSH | 1 |
| `CLR Rd` | `x = 0;` | Clear register (EOR Rd, Rd) | ZNV | 1 |
| `SER Rd` | `x = 0xFF;` | Set register (LDI Rd, 0xFF) | - | 1 |
| `TST Rd` | `if (x == 0)` | Test for zero (AND Rd, Rd) | ZNV | 1 |

**Examples:**
```assembly
; Bit masking - keep lower 4 bits
LDI r16, 0b10101100    ; r16 = 0xAC
ANDI r16, 0x0F         ; r16 &= 0x0F → r16 = 0x0C
```
```c
uint8_t val = 0xAC;
val &= 0x0F;           // val = 0x0C
```

```assembly
; Toggle all bits
LDI r16, 0b10101100    ; r16 = 0xAC
COM r16                ; r16 = 0x53 (inverted)
```
```c
uint8_t val = 0xAC;
val = ~val;            // val = 0x53
```

---

## 4. BIT OPERATIONS

### Shift and Rotate

| Assembly | C Equivalent | Description | Flags | Cycles |
|----------|-------------|-------------|-------|--------|
| `LSL Rd` | `x <<= 1;` | Logical shift left | ZCNVH | 1 |
| `LSR Rd` | `x >>= 1;` | Logical shift right | ZCNV | 1 |
| `ROL Rd` | *complex* | Rotate left through carry | ZCNVH | 1 |
| `ROR Rd` | *complex* | Rotate right through carry | ZCNV | 1 |
| `ASR Rd` | `x = (int8_t)x >> 1;` | Arithmetic shift right (sign-extend) | ZCNV | 1 |
| `SWAP Rd` | `x = (x<<4) \| (x>>4);` | Swap nibbles | - | 1 |
| `BSET s` | `SREG \|= (1<<s);` | Set status flag | varies | 1 |
| `BCLR s` | `SREG &= ~(1<<s);` | Clear status flag | varies | 1 |
| `BST Rd, b` | `T = (Rd >> b) & 1;` | Bit store to T flag | T | 1 |
| `BLD Rd, b` | `Rd = (Rd & ~(1<<b)) \| (T<<b);` | Bit load from T flag | - | 1 |

**Examples:**
```assembly
; Multiply by 8 using shifts (faster than MUL)
LDI r16, 5             ; r16 = 5
LSL r16                ; r16 = 10 (×2)
LSL r16                ; r16 = 20 (×4)
LSL r16                ; r16 = 40 (×8)
```
```c
uint8_t val = 5;
val = val << 3;        // val = 40 (×8)
```

```assembly
; Divide by 4 using shifts
LDI r16, 100           ; r16 = 100
LSR r16                ; r16 = 50 (÷2)
LSR r16                ; r16 = 25 (÷4)
```
```c
uint8_t val = 100;
val = val >> 2;        // val = 25 (÷4)
```

```assembly
; Swap nibbles (high ↔ low)
LDI r16, 0xAB          ; r16 = 0xAB
SWAP r16               ; r16 = 0xBA (1 cycle!)
```
```c
uint8_t val = 0xAB;
val = (val << 4) | (val >> 4);  // val = 0xBA (4-6 cycles)
```

---

## 5. BRANCH INSTRUCTIONS

### Conditional Branches

| Assembly | C Equivalent | Condition | Range | Cycles |
|----------|-------------|-----------|-------|--------|
| `BREQ label` | `if (a == b)` | Z = 1 (Equal) | ±64 words | 1/2 |
| `BRNE label` | `if (a != b)` | Z = 0 (Not equal) | ±64 words | 1/2 |
| `BRLO label` | `if (a < b)` | C = 1 (Lower, unsigned) | ±64 words | 1/2 |
| `BRSH label` | `if (a >= b)` | C = 0 (Same/higher, unsigned) | ±64 words | 1/2 |
| `BRLT label` | `if ((int8_t)a < (int8_t)b)` | S = 1 (Less than, signed) | ±64 words | 1/2 |
| `BRGE label` | `if ((int8_t)a >= (int8_t)b)` | S = 0 (Greater/equal, signed) | ±64 words | 1/2 |
| `BRMI label` | `if ((int8_t)a < 0)` | N = 1 (Minus/negative) | ±64 words | 1/2 |
| `BRPL label` | `if ((int8_t)a >= 0)` | N = 0 (Plus/positive) | ±64 words | 1/2 |
| `BRCS label` | `if (carry)` | C = 1 (Carry set) | ±64 words | 1/2 |
| `BRCC label` | `if (!carry)` | C = 0 (Carry clear) | ±64 words | 1/2 |
| `BRVS label` | `if (overflow)` | V = 1 (Overflow set) | ±64 words | 1/2 |
| `BRVC label` | `if (!overflow)` | V = 0 (Overflow clear) | ±64 words | 1/2 |

**Examples:**
```assembly
; Compare and branch
LDI r16, 10            ; a = 10
LDI r17, 20            ; b = 20
CP r16, r17            ; Compare: a - b (flags set, no result)
BRLO a_is_less         ; Branch if a < b (unsigned)
; ... code if a >= b
a_is_less:
; ... code if a < b
```
```c
uint8_t a = 10, b = 20;
if (a < b) {           // Equivalent comparison
    // a_is_less
} else {
    // a >= b
}
```

```assembly
; Loop counter
LDI r16, 10            ; counter = 10
loop:
    ; ... loop body
    DEC r16            ; counter--
    BRNE loop          ; Branch if counter != 0
; Loop exits when r16 = 0
```
```c
for (uint8_t i = 10; i > 0; i--) {  // Equivalent loop
    // loop body
}
```

---

### Unconditional Branches

| Assembly | C Equivalent | Description | Range | Cycles |
|----------|-------------|-------------|-------|--------|
| `RJMP label` | `goto label;` | Relative jump | ±2K words | 2 |
| `JMP addr` | `goto label;` | Absolute jump (full 128KB) | 0-64K words | 3 |
| `RCALL label` | `function();` | Relative call | ±2K words | 3 |
| `CALL addr` | `function();` | Absolute call | 0-64K words | 4 |
| `RET` | `return;` | Return from subroutine | - | 4 |
| `RETI` | `return;` (from ISR) | Return from interrupt | - | 4 |
| `IJMP` | `(*func_ptr)();` | Indirect jump via Z | - | 2 |
| `ICALL` | `(*func_ptr)();` | Indirect call via Z | - | 3 |
| `EIJMP` | *extended* | Extended indirect jump | - | 2 |
| `EICALL` | *extended* | Extended indirect call | - | 4 |

**Examples:**
```assembly
; Infinite loop
main_loop:
    ; ... code
    RJMP main_loop     ; Jump back to main_loop
```
```c
while (1) {            // Equivalent infinite loop
    // code
}
```

```assembly
; Function call
RCALL my_function      ; Call function
; ... continues here after return

my_function:
    ; ... function code
    RET                ; Return to caller
```
```c
void my_function(void) {
    // function code
    return;            // RET instruction
}
my_function();         // RCALL instruction
```

---

## 6. CONTROL INSTRUCTIONS

### System Control

| Assembly | C Equivalent | Description | Cycles |
|----------|-------------|-------------|--------|
| `NOP` | `asm("nop");` | No operation (1 cycle delay) | 1 |
| `SLEEP` | `sleep_mode();` | Enter sleep mode | 1 |
| `WDR` | `wdt_reset();` | Watchdog reset | 1 |
| `BREAK` | *debugger* | Break for on-chip debug | 1 |
| `SEI` | `sei();` | Set global interrupt enable | 1 |
| `CLI` | `cli();` | Clear global interrupt enable | 1 |

**Examples:**
```assembly
; Precise timing delay (5 cycles @ 16MHz = 312.5ns)
NOP                    ; 1 cycle
NOP                    ; 1 cycle
NOP                    ; 1 cycle
NOP                    ; 1 cycle
NOP                    ; 1 cycle
```
```c
asm volatile("nop");   // Inline assembly for precise timing
```

```assembly
; Enable interrupts
SEI                    ; SREG.I = 1
```
```c
sei();                 // Enable global interrupts
// or
SREG |= (1 << 7);     // Set bit 7 (I flag)
```

---

## 7. ASSEMBLY DIRECTIVES

### Data Allocation Directives

| Directive | C Equivalent | Description | Example |
|-----------|-------------|-------------|---------|
| `.EQU name, value` | `#define name value` | Define constant | `.EQU LED_PIN = 0` |
| `.SET name, value` | `#define name value` | Define constant (changeable) | `.SET counter = 10` |
| `.DEF name, reg` | *N/A* | Define register alias | `.DEF temp = r16` |
| `.DB value` | `const uint8_t[] = {...};` | Define byte(s) in flash | `.DB 0x12, 0x34` |
| `.DW value` | `const uint16_t[] = {...};` | Define word(s) in flash | `.DW 0x1234` |
| `.DD value` | `const uint32_t[] = {...};` | Define double word | `.DD 0x12345678` |
| `.DQ value` | `const uint64_t[] = {...};` | Define quad word | `.DQ 0x123456789ABCDEF0` |
| `.BYTE n` | `uint8_t buffer[n];` | Reserve n bytes in SRAM | `.BYTE 10` |

**Examples:**
```assembly
; Define constants
.EQU F_CPU = 16000000  ; Clock frequency
.EQU BAUD = 9600       ; Baud rate
.EQU LED_ON = 0        ; Active low LED
```
```c
#define F_CPU 16000000UL
#define BAUD 9600
#define LED_ON 0
```

```assembly
; Define register aliases
.DEF temp = r16        ; temp is now alias for r16
.DEF counter = r17     ; counter is alias for r17
```
```c
// C doesn't have exact equivalent
// Use meaningful variable names instead
uint8_t temp, counter;
```

```assembly
; Store lookup table in flash
sine_table:
.DB 0, 25, 50, 71, 87, 98, 100, 98, 87, 71, 50, 25
```
```c
const uint8_t sine_table[] PROGMEM = {
    0, 25, 50, 71, 87, 98, 100, 98, 87, 71, 50, 25
};
```

---

### Segment Directives

| Directive | C Equivalent | Description |
|-----------|-------------|-------------|
| `.CSEG` | *code section* | Code segment (flash) |
| `.DSEG` | *data section* | Data segment (SRAM) |
| `.ESEG` | *eeprom section* | EEPROM segment |
| `.ORG address` | *linker script* | Set origin address |

**Examples:**
```assembly
.CSEG                  ; Code segment (flash memory)
.ORG 0x0000            ; Start at address 0x0000
    RJMP main          ; Reset vector

.ORG 0x0002            ; INT0 vector
    RJMP int0_handler

main:
    ; Main program code

.DSEG                  ; Data segment (SRAM)
.ORG 0x0100            ; Start SRAM variables at 0x0100
buffer: .BYTE 64       ; Reserve 64 bytes for buffer
counter: .BYTE 1       ; Reserve 1 byte for counter

.ESEG                  ; EEPROM segment
.ORG 0x0000
config_byte: .BYTE 1   ; Store config in EEPROM
```

---

### Macro Directives

| Directive | C Equivalent | Description |
|-----------|-------------|-------------|
| `.MACRO name` | `#define name(...) do { ... } while(0)` | Start macro definition |
| `.ENDMACRO` | *end of define* | End macro definition |
| `@0, @1, ...` | *function parameters* | Macro parameters |

**Examples:**
```assembly
; Define macro for LED control
.MACRO LED_ON
    CBI PORTB, @0      ; @0 is first parameter
.ENDMACRO

.MACRO LED_OFF
    SBI PORTB, @0
.ENDMACRO

; Use macros
LED_ON 0               ; Turn on LED0
LED_OFF 0              ; Turn off LED0
```
```c
#define LED_ON(pin) do { PORTB &= ~(1 << (pin)); } while(0)
#define LED_OFF(pin) do { PORTB |= (1 << (pin)); } while(0)

LED_ON(0);             // Turn on LED0
LED_OFF(0);            // Turn off LED0
```

---

## 8. REGISTER ARCHITECTURE

### General Purpose Registers

```
┌─────────────────────────────────────────────────────────────┐
│              ATmega128 Register File (32 registers)         │
├─────────────────────────────────────────────────────────────┤
│ r0-r1   [××] Reserved (multiply result, compiler use)      │
│ r2-r15  [  ] Free for general use                           │
│ r16-r23 [  ] Upper registers (support immediate ops)        │
│ r24-r25 [  ] Function return value / first parameter        │
│ r26-r27 [XX] X pointer (XH:XL)                              │
│ r28-r29 [YY] Y pointer (YH:YL) / Frame pointer             │
│ r30-r31 [ZZ] Z pointer (ZH:ZL) / LPM/ELPM                  │
└─────────────────────────────────────────────────────────────┘
```

### Register Usage Convention (AVR-GCC)

| Registers | Usage | Preserved Across Calls |
|-----------|-------|------------------------|
| r0, r1 | Temporary, multiply result | No - Compiler assumes r1 = 0 |
| r2-r17 | Free for general use | Yes (callee-saved) |
| r18-r27 | Free for general use | No (caller-saved) |
| r28-r29 | Y pointer / Frame pointer | Yes (callee-saved) |
| r30-r31 | Z pointer | No (caller-saved) |

**Examples:**
```assembly
; Calling convention example
my_function:
    PUSH r16           ; Save callee-saved register
    PUSH r17
    
    ; Function body uses r16, r17
    LDI r16, 10
    LDI r17, 20
    ADD r16, r17
    
    POP r17            ; Restore registers
    POP r16
    RET
```
```c
// C compiler handles register saving automatically
uint8_t my_function(void) {
    uint8_t a = 10, b = 20;
    return a + b;
}
```

---

## 9. MEMORY MAP

### ATmega128 Memory Layout

```
┌─────────────────────────────────────────────────────────────┐
│                    FLASH MEMORY (128KB)                     │
├─────────────────────────────────────────────────────────────┤
│ 0x0000-0x0024  Interrupt Vector Table (38 vectors)         │
│ 0x0025-0xFFFF  Program Code                                │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                      SRAM (4KB)                             │
├─────────────────────────────────────────────────────────────┤
│ 0x0000-0x001F  32 General Purpose Registers                │
│ 0x0020-0x005F  64 I/O Registers                            │
│ 0x0060-0x00FF  160 Extended I/O Registers                  │
│ 0x0100-0x10FF  Internal SRAM (4096 bytes)                  │
│                (Stack grows downward from top)              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                     EEPROM (4KB)                            │
├─────────────────────────────────────────────────────────────┤
│ 0x0000-0x0FFF  Non-volatile data storage                   │
└─────────────────────────────────────────────────────────────┘
```

### I/O Register Addressing

| Method | Address Range | Instructions | C Equivalent |
|--------|---------------|-------------|--------------|
| **I/O Direct** | 0x00-0x3F (I/O 0-63) | IN, OUT, SBI, CBI, SBIS, SBIC | Direct register access |
| **Memory Mapped** | 0x20-0x5F (I/O 0-63) | LDS, STS | `*(volatile uint8_t*)(0x20 + offset)` |
| **Extended I/O** | 0x60-0xFF | LDS, STS only | `*(volatile uint8_t*)(0x60 + offset)` |

**Examples:**
```assembly
; I/O Direct (fast - 1 cycle)
IN r16, PORTB          ; PORTB is in range 0x00-0x3F
OUT PORTB, r16

; Memory Mapped (slower - 2 cycles, but works for all I/O)
LDS r16, PORTB+0x20    ; Add 0x20 offset for memory-mapped access
STS PORTB+0x20, r16

; Extended I/O (must use LDS/STS)
LDS r16, TIMSK         ; TIMSK is at 0x6F (extended I/O)
STS TIMSK, r16
```
```c
// C compiler chooses best method automatically
uint8_t val = PORTB;   // Compiler uses IN if possible
PORTB = val;           // Compiler uses OUT if possible
```

---

## 10. PRACTICAL EXAMPLES

### Example 1: LED Blink (Comparison)

**Pure Assembly:**
```assembly
.INCLUDE "m128def.inc"

.CSEG
.ORG 0x0000
    RJMP main

main:
    ; Configure PORTB.0 as output
    SBI DDRB, 0        ; DDRB.0 = 1
    
loop:
    ; Turn LED ON
    CBI PORTB, 0       ; PORTB.0 = 0 (active low)
    RCALL delay_500ms
    
    ; Turn LED OFF
    SBI PORTB, 0       ; PORTB.0 = 1
    RCALL delay_500ms
    
    RJMP loop

delay_500ms:
    ; Delay implementation (simplified)
    LDI r16, 200
delay_loop:
    DEC r16
    BRNE delay_loop
    RET
```

**C Equivalent:**
```c
#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

int main(void) {
    // Configure PORTB.0 as output
    DDRB |= (1 << 0);
    
    while (1) {
        // Turn LED ON
        PORTB &= ~(1 << 0);  // Active low
        _delay_ms(500);
        
        // Turn LED OFF
        PORTB |= (1 << 0);
        _delay_ms(500);
    }
    
    return 0;
}
```

---

### Example 2: UART Transmit Character

**Pure Assembly:**
```assembly
; Transmit character in r16 via UART
uart_tx:
    ; Wait for empty transmit buffer
wait_empty:
    LDS r17, UCSR0A    ; Load UART status
    SBRS r17, UDRE0    ; Skip if UDRE0 bit set
    RJMP wait_empty    ; Loop until ready
    
    ; Transmit character
    STS UDR0, r16      ; Write character to data register
    RET
```

**C Equivalent:**
```c
void uart_tx(char c) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0))) {
        // Wait
    }
    
    // Transmit character
    UDR0 = c;
}
```

---

### Example 3: Array Sum

**Pure Assembly:**
```assembly
; Sum array of 10 bytes
; X pointer (r27:r26) points to array
; Result in r16

array_sum:
    CLR r16            ; sum = 0
    LDI r17, 10        ; counter = 10
    
sum_loop:
    LD r18, X+         ; Load *array++
    ADD r16, r18       ; sum += value
    DEC r17            ; counter--
    BRNE sum_loop      ; Loop if counter != 0
    
    RET                ; Return with sum in r16
```

**C Equivalent:**
```c
uint8_t array_sum(uint8_t *array) {
    uint8_t sum = 0;
    
    for (uint8_t i = 0; i < 10; i++) {
        sum += array[i];
    }
    
    return sum;
}
```

---

### Example 4: 16-bit Addition

**Pure Assembly:**
```assembly
; Add two 16-bit numbers
; r25:r24 = first number
; r23:r22 = second number
; Result in r25:r24

add16:
    ADD r24, r22       ; Add low bytes
    ADC r25, r23       ; Add high bytes with carry
    RET
```

**C Equivalent:**
```c
uint16_t add16(uint16_t a, uint16_t b) {
    return a + b;      // Compiler generates similar assembly
}
```

---

### Example 5: Bit Manipulation

**Pure Assembly:**
```assembly
; Count set bits (popcount)
; Input: r16
; Output: r17 (count)

popcount:
    CLR r17            ; count = 0
    LDI r18, 8         ; bit_counter = 8
    
count_loop:
    LSR r16            ; Shift right, LSB → carry
    BRCC skip_inc      ; Skip if carry = 0
    INC r17            ; count++
skip_inc:
    DEC r18            ; bit_counter--
    BRNE count_loop    ; Loop if not zero
    
    RET                ; Return count in r17
```

**C Equivalent:**
```c
uint8_t popcount(uint8_t x) {
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < 8; i++) {
        if (x & 1) {
            count++;
        }
        x >>= 1;
    }
    
    return count;
}
```

---

## QUICK REFERENCE TABLES

### Instruction Category Summary

| Category | # Instructions | Most Common |
|----------|----------------|-------------|
| Data Transfer | 15 | LDI, MOV, LD, ST, IN, OUT |
| Arithmetic | 16 | ADD, SUB, INC, DEC, MUL |
| Logical | 10 | AND, OR, EOR, COM |
| Bit Operations | 12 | LSL, LSR, SBI, CBI |
| Branch | 22 | BREQ, BRNE, BRLO, BRGE, RJMP |
| Control | 6 | NOP, SEI, CLI, RET, RETI |
| **Total** | **81** | **Most frequently used** |

### Flag Register (SREG) Bits

```
Bit:  7    6    5    4    3    2    1    0
     [I]  [T]  [H]  [S]  [V]  [N]  [Z]  [C]
```

| Bit | Name | Meaning | Set When |
|-----|------|---------|----------|
| **I** | Global Interrupt | Interrupts enabled | SEI instruction |
| **T** | Bit Copy Storage | Temporary bit | BST/BLD instructions |
| **H** | Half Carry | Carry from bit 3 | BCD arithmetic |
| **S** | Sign | N ⊕ V | Signed comparison |
| **V** | Overflow | Two's complement overflow | Signed overflow |
| **N** | Negative | Result is negative | MSB = 1 |
| **Z** | Zero | Result is zero | Result = 0 |
| **C** | Carry | Carry out of MSB | Unsigned overflow |

---

### Instruction Cycle Count Summary

| Cycles | Instructions |
|--------|--------------|
| **1** | Most ALU, logical, bit ops, NOP, SBI, CBI, SEI, CLI |
| **2** | Most branches (taken), ADIW, SBIW, MUL, RJMP, IJMP, LDS, STS, LD, ST |
| **3** | LPM, ELPM, RCALL, ICALL, CALL, JMP |
| **4** | RET, RETI, EICALL |

---

## TIPS FOR STUDENTS

### 🎯 When to Use Assembly vs C

**Use Assembly When:**
- ✅ Precise timing is critical (cycle-accurate delays)
- ✅ Bit-level hardware control (SBI/CBI is atomic and fast)
- ✅ Maximum performance needed (ISRs, tight loops)
- ✅ Learning low-level CPU architecture

**Use C When:**
- ✅ Complex algorithms (easier to read/maintain)
- ✅ Portability matters (code works on other MCUs)
- ✅ Development speed is important
- ✅ Compiler optimization is sufficient

### 📊 Performance Comparison

| Operation | C Code Cycles | Assembly Cycles | Speedup |
|-----------|---------------|-----------------|---------|
| Set single bit | 3-5 | 1 (SBI) | 3-5× |
| Clear single bit | 3-5 | 1 (CBI) | 3-5× |
| Multiply by 8 | 10-15 | 3 (LSL×3) | 3-5× |
| Swap nibbles | 6-8 | 1 (SWAP) | 6-8× |
| Simple loop | Variable | Predictable | Depends |

### 🐛 Common Mistakes

1. **Forgetting to set SREG flags**: Use CP before conditional branches
2. **Wrong register range**: LDI only works with r16-r31
3. **Incorrect pointer usage**: X/Y/Z are 16-bit (two registers)
4. **Stack overflow**: Always initialize stack pointer (SPH:SPL)
5. **Interrupt vectors**: Must be at specific addresses (see datasheet)

---

**END OF INSTRUCTION REFERENCE**

*Print this document for quick reference during lab exercises!*

---

## Additional Resources

- **ATmega128 Datasheet**: Official Microchip/Atmel documentation
- **AVR Instruction Set Manual**: Complete instruction reference
- **AVR-GCC Documentation**: C compiler specifics
- **AVR Assembler User Guide**: Assembler directives and syntax

**Version:** 1.0 | **Date:** October 2025 | **Target:** ATmega128 @ 16MHz
