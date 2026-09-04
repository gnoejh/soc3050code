# INLINE ASSEMBLY QUICK REFERENCE
**ATmega128 Embedded Systems | SOC 3050**

---

## PRINTABLE STUDENT REFERENCE CARD
*Keep this by your workstation during labs*

---

## 📋 INLINE ASSEMBLY SYNTAX

### Basic Template
```c
__asm__ volatile (
    "instruction operands \n\t"    // Assembly code
    "instruction operands \n\t"
    : "=r" (output_var)            // Output operands (optional)
    : "r" (input_var)              // Input operands (optional)
    : "r16", "memory"              // Clobber list (optional)
);
```

### Structure Breakdown
```
__asm__        Keyword to start inline assembly
volatile       Prevents compiler optimization (ALWAYS use for hardware)
("code")       Assembly instructions (separate with \n\t)
: outputs      Variables written by assembly
: inputs       Variables read by assembly  
: clobbers     Registers/memory modified by assembly
```

---

## 🔧 CONSTRAINT CHARACTERS

### Output/Input Modifiers
| Modifier | Meaning | Example |
|----------|---------|---------|
| `=` | Write-only output | `"=r" (result)` |
| `+` | Read-write output | `"+r" (counter)` |
| `&` | Early clobber (written before inputs read) | `"=&r" (temp)` |

### Register Constraints
| Constraint | Registers | Usage |
|------------|-----------|-------|
| `r` | r0-r31 | Any general register |
| `d` | r16-r31 | Upper registers (needed for immediate ops) |
| `a` | r16-r23 | Register pairs for 16-bit operations |
| `b` | Y, Z | Base pointer registers (r28:r29, r30:r31) |
| `w` | r24:r25 | Special register pair |
| `x` | r26:r27 | X pointer register |
| `y` | r28:r29 | Y pointer register |
| `z` | r30:r31 | Z pointer register (LPM, ELPM) |

### Immediate Constraints
| Constraint | Range | Usage |
|------------|-------|-------|
| `I` | 0-63 | 6-bit immediate (bit numbers, small constants) |
| `M` | 0-255 | 8-bit immediate |
| `L` | 0 | Constant zero |

### Special Constraints
| Constraint | Meaning |
|------------|---------|
| `m` | Memory operand |
| `"0"`, `"1"` | Same location as operand 0, 1 (matching) |

---

## 📖 AVR INSTRUCTION SET

### Data Transfer
| Instruction | Syntax | Operation | Cycles |
|-------------|--------|-----------|--------|
| `LDI` | `ldi Rd, K` | Rd ← K (immediate) | 1 |
| `MOV` | `mov Rd, Rr` | Rd ← Rr | 1 |
| `IN` | `in Rd, P` | Rd ← I/O[P] | 1 |
| `OUT` | `out P, Rr` | I/O[P] ← Rr | 1 |
| `LDS` | `lds Rd, k` | Rd ← SRAM[k] | 2 |
| `STS` | `sts k, Rr` | SRAM[k] ← Rr | 2 |
| `LD` | `ld Rd, X/Y/Z` | Rd ← [pointer] | 2 |
| `ST` | `st X/Y/Z, Rr` | [pointer] ← Rr | 2 |
| `LPM` | `lpm Rd, Z` | Rd ← Flash[Z] | 3 |

### Arithmetic
| Instruction | Syntax | Operation | Flags Affected |
|-------------|--------|-----------|----------------|
| `ADD` | `add Rd, Rr` | Rd ← Rd + Rr | Z, C, N, V, H |
| `ADC` | `adc Rd, Rr` | Rd ← Rd + Rr + C | Z, C, N, V, H |
| `SUBI` | `subi Rd, K` | Rd ← Rd - K | Z, C, N, V, H |
| `SUB` | `sub Rd, Rr` | Rd ← Rd - Rr | Z, C, N, V, H |
| `SBC` | `sbc Rd, Rr` | Rd ← Rd - Rr - C | Z, C, N, V, H |
| `INC` | `inc Rd` | Rd ← Rd + 1 | Z, N, V |
| `DEC` | `dec Rd` | Rd ← Rd - 1 | Z, N, V |
| `NEG` | `neg Rd` | Rd ← 0 - Rd | Z, C, N, V, H |

### Logic
| Instruction | Syntax | Operation | Flags |
|-------------|--------|-----------|-------|
| `AND` | `and Rd, Rr` | Rd ← Rd & Rr | Z, N, V |
| `ANDI` | `andi Rd, K` | Rd ← Rd & K | Z, N, V |
| `OR` | `or Rd, Rr` | Rd ← Rd \| Rr | Z, N, V |
| `ORI` | `ori Rd, K` | Rd ← Rd \| K | Z, N, V |
| `EOR` | `eor Rd, Rr` | Rd ← Rd ^ Rr | Z, N, V |
| `COM` | `com Rd` | Rd ← ~Rd | Z, C, N, V |

### Bit Operations
| Instruction | Syntax | Operation | Cycles |
|-------------|--------|-----------|--------|
| `SBI` | `sbi P, b` | I/O[P].b ← 1 | 1-2 |
| `CBI` | `cbi P, b` | I/O[P].b ← 0 | 1-2 |
| `LSL` | `lsl Rd` | Rd ← Rd << 1 | 1 |
| `LSR` | `lsr Rd` | Rd ← Rd >> 1 | 1 |
| `ROL` | `rol Rd` | Rotate left through C | 1 |
| `ROR` | `ror Rd` | Rotate right through C | 1 |
| `SWAP` | `swap Rd` | Swap nibbles | 1 |

### Branch & Control
| Instruction | Syntax | Condition | Cycles |
|-------------|--------|-----------|--------|
| `RJMP` | `rjmp label` | Unconditional jump | 2 |
| `BREQ` | `breq label` | Branch if Z=1 (equal) | 1/2 |
| `BRNE` | `brne label` | Branch if Z=0 (not equal) | 1/2 |
| `BRLT` | `brlt label` | Branch if S=1 (less than) | 1/2 |
| `BRGE` | `brge label` | Branch if S=0 (greater/equal) | 1/2 |
| `RCALL` | `rcall label` | Relative call | 3 |
| `RET` | `ret` | Return from subroutine | 4 |
| `RETI` | `reti` | Return from interrupt | 4 |
| `NOP` | `nop` | No operation | 1 |
| `SEI` | `sei` | Enable interrupts (I=1) | 1 |
| `CLI` | `cli` | Disable interrupts (I=0) | 1 |

---

## 🎯 COMMON PATTERNS

### Pattern 1: Simple Delay (NOPs)
```c
__asm__ volatile (
    "nop \n\t"    // 1 cycle = 62.5ns @ 16MHz
    "nop \n\t"    // 2 cycles
    "nop \n\t"    // 3 cycles
    "nop \n\t"    // 4 cycles
);
// Total: 4 cycles = 250ns
```

### Pattern 2: Set/Clear LED
```c
// Turn LED ON (clear bit)
__asm__ volatile (
    "cbi %0, %1 \n\t"    // Clear bit in I/O
    :
    : "I" (_SFR_IO_ADDR(PORTB)), "I" (0)
);

// Turn LED OFF (set bit)
__asm__ volatile (
    "sbi %0, %1 \n\t"    // Set bit in I/O
    :
    : "I" (_SFR_IO_ADDR(PORTB)), "I" (0)
);
```

### Pattern 3: Read Input
```c
uint8_t button;
__asm__ volatile (
    "in %0, %1 \n\t"    // Read port
    : "=r" (button)                    // Output
    : "I" (_SFR_IO_ADDR(PIND))        // Input
);
```

### Pattern 4: Variable Addition
```c
uint8_t a = 10, b = 20, result;
__asm__ volatile (
    "add %0, %1 \n\t"    // result = a + b
    : "=r" (result)      // Output (write)
    : "r" (a), "0" (b)   // Inputs (b reuses result register)
);
```

### Pattern 5: Increment Counter
```c
uint8_t count = 0;
__asm__ volatile (
    "inc %0 \n\t"        // count++
    : "+r" (count)       // Read-write
);
```

### Pattern 6: 16-bit Addition
```c
uint16_t x = 1000, y = 2000, sum;
__asm__ volatile (
    "add %A0, %A1 \n\t"   // Add low bytes
    "adc %B0, %B1 \n\t"   // Add high bytes with carry
    : "=r" (sum)
    : "r" (x), "0" (y)
);
```

### Pattern 7: Bit Masking
```c
uint8_t data = 0xAC;
__asm__ volatile (
    "andi %0, 0x0F \n\t"  // Mask lower 4 bits
    : "+d" (data)         // d constraint for immediate
);
```

### Pattern 8: Pointer Access
```c
uint8_t array[5] = {1, 2, 3, 4, 5};
uint8_t *ptr = array;
uint8_t value;

__asm__ volatile (
    "ld %0, X+ \n\t"      // Load and post-increment
    : "=r" (value), "+x" (ptr)
);
```

---

## ⚠️ IMPORTANT RULES

### 1. Always Use `volatile`
```c
// CORRECT:
__asm__ volatile ("nop");

// WRONG (compiler may remove):
__asm__ ("nop");
```

### 2. Constraint Matching
```c
// Immediate operations need 'd' constraint (r16-r31)
// CORRECT:
__asm__ volatile ("subi %0, 5" : "+d" (val));

// WRONG (compile error):
__asm__ volatile ("subi %0, 5" : "+r" (val));
```

### 3. List All Clobbers
```c
// If you modify r16, r17, and memory:
__asm__ volatile (
    "ldi r16, 0xFF \n\t"
    "mov r17, r16  \n\t"
    :
    :
    : "r16", "r17", "memory"  // Tell compiler!
);
```

### 4. SBI/CBI Addressing
```c
// Use _SFR_IO_ADDR() for I/O space:
// CORRECT:
"cbi %0, %1" : : "I" (_SFR_IO_ADDR(PORTB)), "I" (0)

// WRONG (wrong address):
"cbi %0, %1" : : "I" (PORTB), "I" (0)
```

### 5. Multi-byte Operations
```c
// For 16-bit variables, use %A0 (low) and %B0 (high):
uint16_t value;
__asm__ volatile (
    "ldi r16, 0x34   \n\t"
    "mov %A0, r16    \n\t"  // Low byte
    "ldi r16, 0x12   \n\t"
    "mov %B0, r16    \n\t"  // High byte
    : "=r" (value) : : "r16"
);
// value = 0x1234
```

---

## 🔍 REGISTER REFERENCE

### General Purpose Registers
```
r0-r1:   Result registers (multiply, etc.)
r2-r17:  Free for general use
r18-r27: Free for general use (preferred for temporaries)
r28-r29: Y pointer (frame pointer in C)
r30-r31: Z pointer (often used for LPM)
r26-r27: X pointer
```

### Special Registers (don't modify without saving!)
```
r0-r1:   Compiler uses for multiply result
r28-r29: Y register (stack frame pointer)
```

### I/O Register Addressing
```
Low I/O space (0x00-0x1F):  Use IN/OUT/SBI/CBI directly
    Example: PORTB = 0x18 → use 0x18
    
High I/O space (0x20-0x3F): Need _SFR_IO_ADDR()
    Example: PORTB → use _SFR_IO_ADDR(PORTB)
    
Extended I/O (0x60+):       Use LDS/STS (memory access)
```

---

## 📊 FLAG REGISTER (SREG)

```
Bit:  7    6    5    4    3    2    1    0
     [I]  [T]  [H]  [S]  [V]  [N]  [Z]  [C]

I: Global Interrupt Enable
T: Bit Copy Storage
H: Half Carry Flag
S: Sign Flag (N ⊕ V)
V: Two's Complement Overflow
N: Negative Flag
Z: Zero Flag
C: Carry Flag
```

### Flag Usage
- **Z**: Set if result is zero (BREQ, BRNE)
- **C**: Set on carry/borrow (multi-byte math)
- **N**: Set if result is negative (signed)
- **V**: Set on signed overflow

---

## 💡 OPTIMIZATION TIPS

### 1. Use Shifts for Multiply/Divide by Powers of 2
```c
// Multiply by 4 (faster than multiply):
__asm__ volatile (
    "lsl %0 \n\t"    // x2
    "lsl %0 \n\t"    // x4
    : "+r" (value)
);

// Divide by 8:
__asm__ volatile (
    "lsr %0 \n\t"    // /2
    "lsr %0 \n\t"    // /4
    "lsr %0 \n\t"    // /8
    : "+r" (value)
);
```

### 2. SWAP for Nibble Exchange
```c
// Faster than multiple shifts:
__asm__ volatile (
    "swap %0 \n\t"   // Swap high/low nibbles (1 cycle)
    : "+r" (value)
);
```

### 3. COM vs XOR for Inversion
```c
// COM is faster for full inversion:
__asm__ volatile ("com %0" : "+r" (val));

// vs:
val = ~val;  // May compile to multiple instructions
```

### 4. SBI/CBI for Single Bits
```c
// Atomic and fast (1 cycle):
__asm__ volatile ("sbi %0, %1" : : "I" (_SFR_IO_ADDR(PORTB)), "I" (3));

// vs C (3-5 instructions):
PORTB |= (1 << 3);
```

---

## 🐛 DEBUGGING TIPS

### View Disassembly
```bash
avr-objdump -d Main.elf > disassembly.txt
```

### Check Register Usage
Look for your inline assembly in disassembly:
```assembly
000001a4 <demo_function>:
 1a4:   8f e5           ldi r24, 0x5F    ; Your code here
 1a6:   0e 94           call 0x...
```

### Common Errors
1. **Compile error: "impossible constraint"**
   - Wrong constraint for instruction (use 'd' for immediate ops)

2. **Wrong results**
   - Forgot to list clobbered registers
   - Operand numbering mismatch

3. **Code not executing**
   - Missing `volatile` keyword
   - Compiler optimized away

---

## 📚 FURTHER READING

- AVR Instruction Set Manual
- GCC Inline Assembly Documentation
- ATmega128 Datasheet (Section 5: AVR CPU Core)
- "Embedded C Programming" by Mark Siegesmund

---

**END OF QUICK REFERENCE**
*Print this document and keep it handy during labs!*
