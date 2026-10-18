# SOC3050 Midterm — Overview and Study Guide

**Embedded System and Software Design · Fall 2026 · ARM edition**

This guide tells you what the midterm covers and what it looks like. It also
works through the basics of every lesson in scope, so you can study from it
directly. It does not replace the slide decks: each section names the slides
to re-read. If you understand everything here and can do the practice
problems at the end without looking, you are ready.

**Contents**

1. Exam format
2. Scope and how questions are spread
3. Study notes, lesson by lesson (00–08)
4. Formula and fact sheet
5. Glossary
6. Practice problems, with answers
7. How to prepare

---

## 1. Exam format

| | |
|---|---|
| Duration | **60 minutes** |
| Questions | **33 multiple-choice questions**, one mark each, total 33 |
| Answers | Four options (A–D) per question. **Choose exactly one.** |
| Answer sheet | Write each answer on the answer sheet on the cover page. **Only the answer sheet is marked.** |
| Pen | Blue or black, non-erasable. No pencil, highlighter or correction pen. |
| Materials | **None.** Closed book, no notes, no calculator, no electronic devices. |
| Identification | Name (capital letters), student ID and section number on every page. **A wrong section number is penalised.** |

There is no penalty for a wrong answer, so answer every question. About
1 minute 50 seconds per question: if one stalls you, mark your best guess and
come back to it.

The questions test **understanding and calculation**, not memorised trivia.
Expect to:

- **predict behaviour**: "this code does X; what does the board do?" Many
  questions are about *silent failures*, where the code compiles with no
  warnings and then does nothing, or hangs;
- **calculate by hand**: bit positions and register values for a pin, a
  32-bit value in hex, little-endian bytes, an IRQ's vector address, `PSC` and
  `ARR` for a timer, `BRR` for a baud rate. The numbers are chosen to be
  workable without a calculator;
- **choose the correct line of code** from four, for example which statement
  clears a flag correctly;
- **name the concept**: the general idea behind a mechanism, as each deck's
  "general idea / this chip's answer" sections present it.

You will **not** be asked to write code. You will not need to recall a
register address that the question does not give, or the exact numbers from a
measured run (switch counts, byte counts and so on).

---

## 2. Scope and how questions are spread

Everything in the slide decks of lessons **00 to 08** is in scope. Lesson 09
and later are **not** on the exam. Every lesson contributes 2–5 questions,
roughly in proportion to its size.

| Lesson | Deck | Part |
|---|---|---|
| 00 | The Programmer's Model | Part 0: models |
| 01 | Instruction Set | Part 0: models |
| 02 | Development | Part 0: models |
| 03 | Execution and Concurrency | Part 0: models |
| 04 | Startup and the Linker Script | Part 1: instances |
| 05 | GPIO and Interrupts | Part 1: instances |
| 06 | Time and Timers | Part 1: instances |
| 07 | RTOS | Part 1: instances |
| 08 | UART and a Python Host | Part 1: instances |

The decks are online at **https://gnoejh.github.io/soc3050code/**.

### Key contrasts: expect these

The course keeps asking one question: *what is the general idea, what are the
alternatives, and which one did this chip pick?* Several exam questions test
pairs of ideas. Know both sides of each pair, which one this chip uses, and
**what follows from it in practice** — the questions describe a situation
rather than ask for a definition.

| Pair | This chip | What follows | Where |
|---|---|---|---|
| **Load–store** vs register–memory | load–store | `x++` on memory is 3 instructions (`ldr`, `adds`, `str`), but the same 2 bus accesses as x86's single instruction; read-modify-write has a gap an interrupt can land in | 01, slide 4 |
| **RISC** vs CISC | RISC (Thumb) | fixed 2/4-byte instructions: simple decoder, uniform pipeline; costs more instructions, not more memory traffic | 01, slides 2, 4 |
| **Memory-mapped** vs port-mapped (isolated) I/O | memory-mapped | peripherals are reached by ordinary `ldr`/`str`; with no MMU, a stray pointer can write a peripheral | 00, slides 4, 8, 15 |
| **Von Neumann** vs Harvard | one address space | `const` data in flash is read by ordinary `ldr`; no `PROGMEM`/`LPM`; separate buses on some cores don't change the map | 00, slide 4 |
| **Polling** vs interrupt (vs DMA) | all three are used | polling is simple but depends on the loop coming back and samples (filters) fast events; an interrupt sees every edge, wanted or not | 03, slide 1; 05, slides 20–22, 32 |
| **Cooperative** (superloop) vs **preemptive** (RTOS) scheduling | both, lessons 06 and 07 | cooperative: a job stops only when it returns, so one CPU hog freezes everything; preemptive: the kernel's tick takes the CPU away | 06, slide 4; 07, slides 1, 9, 11 |
| Condition codes vs compare-and-branch | condition codes (N Z C V) | `cmp` then a branch; signed (`blt`) vs unsigned (`bcc`) branches | 01, slide 7 |
| Link register vs stack-based return | link register | `bl` saves the return in `LR`; non-leaf functions must `push {lr}` | 01, slide 8 |
| Vectored vs non-vectored interrupts | vectored, table of addresses | the handler's *name* puts its address in the table | 00, slide 9 |
| Compile-, load- or run-time address binding | build time | no loader, no MMU, so you write the linker script | 02, slide 2 |

---

## 3. Study notes, lesson by lesson

### Lesson 00 — The Programmer's Model

**The idea.** A programmer's model is the small, published contract a chip
offers: the state you can inspect and change, and what happens when you do. It
leaves out the pipeline, the bus matrix and the silicon. This one has **five
parts**:

1. **Registers**: the working state inside the CPU
2. **Memory**: one flat address space
3. **Peripheral registers**: memory-mapped controls of the hardware
4. **Modes and stacks**: Thread / Handler mode, MSP / PSP
5. **Exceptions**: how the hardware takes control away from you

**The registers.** The core has 16 × 32-bit registers.

| Register | Job |
|---|---|
| `R0`–`R3` | arguments in, result out, scratch |
| `R4`–`R12` | locals kept out of memory |
| `R13` = `SP` | stack pointer. One name, two physical registers: `MSP` and `PSP` |
| `R14` = `LR` | link register: the return address |
| `R15` = `PC` | program counter. Writing it is a branch |
| `xPSR` | one word with three views: `APSR` (flags N Z C V, bits 31:28), `EPSR` (Thumb bit T, bit 24), `IPSR` (exception number, bits 5:0) |
| `PRIMASK` | bit 0 = 1 masks every configurable interrupt |
| `CONTROL` | bit 1 `SPSEL` selects which stack Thread mode uses; bit 0 `nPRIV` selects privilege |

The special registers (`xPSR`, `PRIMASK`, `CONTROL`) have **no address**. Only
`mrs` and `msr` reach them. The Cortex-M0+ has **no** `BASEPRI` or
`FAULTMASK`; those registers are M3/M4 only.

**Two ways to reach a device. Know both names and which machine uses which.**

| | Port-mapped I/O | Memory-mapped I/O |
|---|---|---|
| How | separate I/O address space, special instructions | ordinary addresses, ordinary `ldr` / `str` |
| Examples | x86 `in`/`out`, **AVR** `IN`/`OUT` | **Cortex-M** (this chip) |

**One address space or two?**

| | Harvard | von Neumann |
|---|---|---|
| Code and data | separate address spaces | one address space |
| Consequence | reading flash constants needs a special instruction (AVR `LPM`, `PROGMEM`) | a `const` array in flash is read with ordinary `ldr` |
| Machine | **AVR** | **Cortex-M** address map |

Memory-map facts to remember: flash is at `0x08000000` and is **aliased to
`0x00000000` at reset**, so the CPU finds the vector table "at zero". SRAM
starts at `0x20000000`. ST's peripherals are at `0x40000000`. **GPIO on the
STM32C0 is at `0x50000000`.** The **NVIC and SysTick at `0xE000xxxx` are ARM's**,
identical on every Cortex-M, so ARM's manual describes them and ST's reference
manual describes everything else.

**A register is not a variable.**

| A variable | A register |
|---|---|
| lives in RAM | *is* a piece of hardware |
| reading twice gives the same value | reading twice can give different values |
| writing changes memory | writing makes something happen |
| reading has no side effect | reading can clear a flag or consume a byte |
| the compiler may cache, reorder or delete accesses | every access must happen exactly as written → `volatile` |

**Every peripheral has three kinds of register.** **Control** registers are
written by software to configure the peripheral. **Status** registers are
written by hardware and report what happened. **Data** registers carry values
in and out. Examples: USART `CR1` / `ISR` / `TDR`,`RDR`; Timer `CR1`,`PSC`,`ARR` / `SR` /
`CNT`,`CCR`; GPIO `MODER` / `IDR` / `ODR`,`BSRR`.

**Two gates you must open.** Both look the same from outside: correct code that
does nothing, and no error message.

- **Clock gate.** Every peripheral powers up unclocked. Writes vanish and reads
  return 0. This is the most common first-week failure.
- **Pin multiplexer.** Each pin is shared between peripherals. If you choose
  the wrong owner, the pin carries no signal.

**The clock is yours too.** A baud rate is a *division* of the peripheral's
bus clock. A delay is a *count* of ticks. A wrong clock gives garbage
characters from perfect code.

**Privilege without virtual memory.** Desktops use an **MMU** for private
virtual address spaces, so a bad pointer gives a segmentation fault. This chip
has two privilege levels, **no MMU**, one real address space, and optionally
an **MPU** of permission rules. A stray pointer therefore quietly reaches a
peripheral or someone else's data.

**Vectored interrupts.** In non-vectored dispatch, every interrupt enters at
one address and software asks who called. In vectored dispatch, the source's
number indexes a table. AVR's table holds jump instructions. **Cortex-M's
table holds plain addresses**, and it is built by your own code.

**Data model.**

- **Word size is 32 bits.** `uint8_t` is not cheaper than `uint32_t` here; it
  is often more expensive, because the compiler has to mask and shift.
- **Natural alignment.** An *n*-byte access wants an address divisible by *n*.
  **A Cortex-M0+ HardFaults on an unaligned word access.** An M3/M4 performs
  it, just more slowly. The classic trap is casting a `uint8_t *` buffer to
  `uint32_t *`.
- **Little-endian.** `0x12345678` stored at address *a* puts `0x78` at *a*,
  `0x56` at *a*+1, `0x34` at *a*+2 and `0x12` at *a*+3. Network byte order
  (internet headers) is big-endian.

**Where the implementation leaks through** (slide 13):

- **Pipelining**: the PC reads ahead, and a taken branch costs extra.
- **Two buses**: AHB and APB, each with its own clock-enable register.
- **Slow flash**: flash needs wait states (`FLASH->ACR`), set *before* the
  clock is raised.

Re-read: slides 3–13, and answer slide 17.

---

### Lesson 01 — Instruction Set

**Thumb only.** ARM designs two instruction sets: ARM (A32, always 32-bit)
and Thumb (T32, 16-bit, widened to 32-bit only when it must be). **Every
Cortex-M executes Thumb only**, so there is no ARM state to switch to. Thumb
exists for **code density**; it is about 30 % smaller. Compressed instruction
sets elsewhere include RISC-V's "C" extension and MIPS16.

**Thumb-1 vs Thumb-2.**

| | Cortex-M0/M0+ (ARMv6-M) | Cortex-M3/M4 (ARMv7-M) |
|---|---|---|
| Instruction set | Thumb-1 plus six 32-bit instructions | full Thumb-2 |
| Divide | **none**: calls `__aeabi_uidiv` | `sdiv`, `udiv` |
| Flags | most 16-bit forms always set them (`adds`, `movs`) | your choice (`add` or `adds`) |
| Conditional execution | branches only | `IT` blocks |
| `ldr` offset | 0–124 bytes, positive only | ±4095, pre/post-indexed |
| Registers | `r0`–`r7` in most forms | all 16 |
| FPU | none | M4F, M7 have one |

The six 32-bit instructions on the M0+: `bl`, `mrs`, `msr`, `dsb`, `dmb`, `isb`.

**Load–store vs register–memory.** This is the core RISC/CISC distinction.

| | Register–memory | Load–store |
|---|---|---|
| ALU operand | register **or** memory | register only |
| Memory touched by | most instructions | `ldr`/`str` only |
| Machines | x86, 68000, S/360 | **ARM**, RISC-V, MIPS, **AVR** |
| `counter++` | 1 instruction | 3 instructions (`ldr`, `adds`, `str`) |
| Bus accesses for `counter++` | 2 | 2 (the same) |

Consequence: **setting one bit in a peripheral register is a read, a modify
and a write**: three instructions, with a gap an interrupt can land in. AVR had
`sbi`; the M0+ has none. The fix comes back as hardware: GPIO `BSRR`
(lesson 05).

**The thirteen instructions that make up ~80 % of code.**

| Instruction | Meaning |
|---|---|
| `movs r0, #7` | constant → register (no memory access) |
| `ldr` / `str` | 32-bit load/store; `ldrb`/`strb` 8-bit, `ldrh`/`strh` 16-bit, `ldrsh` sign-extended |
| `adds`, `subs` | add, subtract |
| `cmp` | subtract, discard the result, set the flags |
| `ands`, `orrs`, `eors` | `&=`, `\|=`, `^=` |
| `bics` | **bit clear**: `x &= ~y` |
| `lsls` | shift left (`<<`, multiply by 2ⁿ). Common because of bit fields: `(2u << (pin*2))` |
| `lsrs` / `asrs` | shift right, unsigned / **signed** (keeps the sign) |
| `b`, `beq`, `bne` | branch, branch if equal / not equal |
| `bl` | branch with link: call, return address saved in `LR` |
| `bx lr` | return |
| `push {r4, lr}` / `pop {r4, pc}` | prologue / epilogue. **Popping into `PC` is the return.** |

**Flags.** Two families. A **condition-code machine** (ARM, x86, AVR) has `cmp`
set N Z C V and a later branch read them. A **compare-and-branch machine**
(RISC-V, MIPS) does both in one instruction (`blt a0, a1, L`) and has no flags
register. Signed and unsigned comparisons use different branches: **`blt`/`bge`
are signed**, **`bcc`/`bcs` are unsigned**. Mixing them up is a signedness bug
that shows only at the extremes of the range.

**Where the return address goes.** On **stack-based** machines (x86, AVR) the
call pushes it to memory. On **link-register** machines (ARM, RISC-V, MIPS)
`bl` writes it into `LR`. A *leaf* function never touches the stack for its
return. A function that calls another must save `LR` first, which is what
`push {..., lr}` is for.

**Addressing modes** (16-bit forms):

| C | Mode | Example |
|---|---|---|
| `*p` | register | `ldr r0, [r1]` |
| `s->field` | immediate offset | `ldr r0, [r4, #28]` |
| `buf[i]` | register offset | `ldrb r6, [r1, r3]` |
| local variable | SP-relative | `ldr r0, [sp, #8]` |
| big constant | PC-relative (literal pool) | `ldr r2, [pc, #44]` |

**Big constants live in a literal pool.** A 16-bit instruction cannot hold a
32-bit constant such as `0x40021000`. ARM keeps a pool of words after the
function and loads them PC-relative. RISC-V instead builds the constant in two
halves (`lui` + `addi`), and x86 just makes the instruction longer.

**AAPCS (the calling convention, part of the ABI).**

| Registers | Rule |
|---|---|
| `r0`–`r3` | arguments 1–4; **result in `r0`**; caller-saved (a function may destroy them) |
| `r4`–`r11` | **callee-saved**: a function that uses them must push and restore them |
| `r12` | scratch |

**The Thumb bit.** Every code address handed to the core has **bit 0 = 1**,
meaning "Thumb". `Reset_Handler` at `0x08000274` appears in the vector table as
`0x08000275`. After `bl`, `LR` is odd (return address | 1). `bx lr` branches
to `LR & ~1`, so nothing is skipped. **Branching to an address with bit 0
clear faults**, because the core is told to run ARM code, which it does not
have. Inside an exception handler, `LR` holds a special `EXC_RETURN` value such
as `0xFFFFFFF9` or `0xFFFFFFFD`, not a return address.

**What the compiler may do.** It preserves only the *observable behaviour*
defined by C. It may hoist constants out of loops, inline functions, reorder
blocks (so address order is not source order) and cache values in registers.
**`volatile`** forbids that for an object. CMSIS declares every peripheral
register `__IO` = `volatile`. The classic symptom of a missing `volatile` is
code that works at `-O0` and hangs at `-Os`.

Re-read: slides 2–21, and answer slide 22.

---

### Lesson 02 — Development

**No operating system.** On a PC the OS places your program, provides a stack,
zeroes globals and calls `main()`. Here, if you don't arrange those things,
nothing does.

**Five stages:** preprocess → compile → assemble → **link and locate**. Locate
is the embedded-specific stage: it gives every name a real address, driven by
a linker script you write.

**Address binding: when does a name get its address?**

| Bound at | Meaning | Where |
|---|---|---|
| compile (build) time | fixed in the image | **this chip** (no loader, no MMU), MS-DOS `.com` |
| load time | loader picks a base | relocatable executables |
| run time | translated by an MMU while running | desktop/phone OSes |

**One file at a time.** Each `.c` becomes an `.o` that knows nothing about the
others. Two link errors follow from that:

- `undefined reference to X`: someone uses `X` and nobody defines it.
- `multiple definition of X`: two files both define it.

**Sections.**

| Section | Contents | Lives in | Costs flash? | Costs RAM? |
|---|---|---|---|---|
| `.isr_vector` | vector table | flash, at `0x08000000` | yes | no |
| `.text` | code | flash | yes | no |
| `.rodata` | `const` data, string literals | flash | yes | no |
| `.data` | globals with a non-zero initial value | RAM, **with a copy of the initial values in flash** | **yes** | **yes** |
| `.bss` | globals that start at zero | RAM | **no** (only its size is recorded) | yes |

`.data` must be writable, so it lives in RAM. Its initial value must survive
power-off, so a copy is kept in flash. **It costs both.** Prefer `const`
(`.rodata`, flash only) where you can.

**Before `main()`.** `Reset_Handler` copies `.data` from flash to RAM and
zeroes `.bss`, using linker-made symbols `_sidata`, `_sdata`, `_edata`,
`_sbss`, `_ebss`. Without the `.bss` loop, globals start as whatever RAM held.
That works on a cold boot and fails after a reset, so the bug is
**intermittent**.

**The linker script is the memory map, written down.**

```
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (rw) : ORIGIN = 0x20000000, LENGTH = 12K
}
```

The numbers come from the datasheet. If `LENGTH` is too large, the build
succeeds and the stack later walks off the end of real memory.

**Output files.** `.elf` holds code, addresses, symbols and debug information
(used by the debugger and simulator). `.bin` is raw bytes. `.hex` is the same
bytes as text records.

**Reading an Intel HEX record:** `:LLAAAATT<data>CC`

| Field | Meaning |
|---|---|
| `:` | start |
| `LL` | number of data bytes |
| `AAAA` | 16-bit address offset |
| `TT` | type: `00` data, `01` end of file, **`04` extended linear address** (upper 16 bits of the address) |
| `CC` | checksum |

`:020000040800F2` is type `04` and sets the upper address to `0x0800`. An 8-bit
AVR never needed one, because its flash fits in 16 bits of address. The first
data words of the image are the **initial SP** and the **reset vector**, in
little-endian.

**Bootstrapping.** A PC's boot chain has four handoffs: firmware →
bootloader → kernel → program. This chip has **one**: the hardware reads two
words from address 0 (SP and reset vector) and starts your code.

**Tools.** **Wokwi** answers *does the circuit behave?* (LEDs, wires).
**Renode** answers *does the code behave?* (serial output, registers). Neither
compiles anything. *A thing is not working until you have watched it work.*

Re-read: all 10 slides, and answer slide 10.

---

### Lesson 03 — Execution and Concurrency

**Three ways to meet hardware.**

| Method | CPU | Use when |
|---|---|---|
| **Polling** | keeps asking "ready yet?", does nothing else | the wait is shorter than the cost of being interrupted |
| **Interrupt** | does useful work until the hardware calls a handler | events are rare or must not be missed |
| **DMA** | not involved; hardware moves the bytes | streams of data |

**An interrupt is a hardware-forced function call.** Between two instructions,
the hardware does three things:

- It pushes **eight registers**: `r0`–`r3`, `r12`, `lr`, `pc`, `xPSR`. That is
  exactly the set AAPCS lets a function destroy, **so an ISR is an ordinary C
  function**, with no keyword.
- It writes the exception number into `IPSR`.
- It loads `LR` with `EXC_RETURN` and jumps to the handler. Returning with
  `bx lr` unstacks everything.

A misspelt handler name compiles, links and does nothing.

**Hazard one: the compiler.** The **as-if rule** says the compiler only has to
produce the same *observable behaviour* as your source. In
`while (flag == 0) {}`, it reads `flag` once and loops on a register, because
hardware and ISRs are observers C knows nothing about.

**Fix: `volatile`.** Every access is then performed, in order. `volatile` is
about the *compiler*: it is **not a lock** and does **not make anything
atomic**.

**Hazard two: the interrupt lands mid-update.** `counter++` is `ldr`, `adds`,
`str`. If an ISR that also increments `counter` runs between the `ldr` and the
`str`, `main()` writes back a stale value and one increment is **lost**.
`volatile` does not help.

**Vocabulary** (the same on any system):

| Term | Meaning |
|---|---|
| **race condition** | the result depends on the relative timing of two contexts |
| **critical section** | code that must not be interleaved |
| **mutual exclusion** | at most one context inside the critical section at a time |
| **atomic** | cannot be observed half-done |
| **tearing** | a reader collects one logical value in two accesses and gets half old, half new |

**How you get mutual exclusion:**

| Contexts | Tool |
|---|---|
| ISR vs `main()`, one core | **mask interrupts**: `__disable_irq()` / `__enable_irq()` = `cpsid i` / `cpsie i` = `PRIMASK` bit 0 |
| two threads, one OS | a mutex (the scheduler doesn't switch to the waiter) |
| two cores | an atomic read-modify-write instruction (`ldrex`/`strex`, x86 `lock cmpxchg`). Masking interrupts does **not** stop the other core. |

Keep critical sections **short**: while interrupts are off, the chip can't
respond to anything. The longest critical section sets the worst-case
**interrupt latency**.

**Two fixes, two problems.** `volatile` makes accesses *real*. A critical
section makes a sequence *indivisible*. Most concurrency bugs come from using
the first where the second was needed.

**Why these bugs are rare.** The window is ~2 instructions (~40 ns). At
1000 interrupts per second the bug appears about once every 7 hours. **You
can't find it by running more**; you find it by reasoning about which
variables are shared.

**What needs protection:**

| Situation | Safe? |
|---|---|
| ISR writes a single aligned word (or byte); main only reads | **yes**: one access, one writer |
| ISR sets a flag; main reads and clears it | usually. The clear is a read-modify-write |
| both sides increment a counter | **no**: lost update |
| main reads a `uint64_t` the ISR writes | **no**: two bus accesses, tearing |
| a multi-field struct | **no**: seen half-updated |

**Priority.** The NVIC ranks interrupts. This chip implements the **top 2 bits**
of each 8-bit priority field, giving **4 levels: 0 is most urgent, 3 least**. A
more urgent interrupt **preempts** a running handler. Handlers must be short:
set a flag or buffer the data, return, and let `main()` do the work.
**Priority inversion** (Mars Pathfinder, 1997) is introduced here and treated
fully in lesson 07.

Re-read: all slides, and answer slide 10.

---

### Lesson 04 — Startup and the Linker Script

**Two files.** `link.ld` decides *where everything goes*. `startup.c` decides
*what runs first*: the vector table, and the code that makes RAM match C's
promises.

**The STM32C031C6 map:** flash 32 KB at `0x08000000`, SRAM 12 KB at
`0x20000000`, peripherals at `0x40000000` (RCC `0x40021000`, USART2
`0x40004400`, FLASH `0x40022000`), **GPIO at `0x50000000`** (GPIOA; GPIOB is
`0x50000400`, ports are `0x400` apart), and the core at `0xE0000000` (SysTick
`0xE000E010`, NVIC `0xE000E100`, SCB `0xE000ED00`). Reading an unmapped address
is a fault, not a zero.

**CMSIS** (Common Microcontroller Software Interface Standard) is **headers,
not a library**: `#define`s and structs that name addresses. `core_cm0plus.h`
is ARM's (NVIC, SysTick, SCB); `stm32c031xx.h` is ST's. `GPIOA->ODR` is just
base + offset, computed at compile time. CMSIS is **not** ST's HAL.

**RCC clock gates.** `RCC->IOPENR` has one bit per GPIO port (`GPIOAEN` = bit 0,
`GPIOBEN` = bit 1). `RCC->APBENR1` covers USART2, TIM3, I2C1 and others
(`USART2EN` = bit 17). `RCC->APBENR2` covers TIM1, SPI1, USART1, ADC and
others. **Enable the gate first, always.**

**Reset in hardware.** Unlike the AVR, the Cortex-M reads **word 0 → `SP`** and
**word 1 → `PC`** itself, before any instruction runs, so the reset handler can
be C.

**The vector table is a C array:**

```c
__attribute__((section(".isr_vector"), used))
void (* const vectors[])(void) = {
    (void (*)(void))&_estack,   /* 0: initial SP, not a function */
    Reset_Handler,              /* 1 */
    NMI_Handler, HardFault_Handler, ...
};
```

- `section(".isr_vector")` lets `link.ld` pin it at `0x08000000`.
- `used` stops the compiler deleting an array no C code reads.
- `const` puts it in flash, not RAM.

**Vector numbering.** Entries 0–15 belong to the core: 0 initial SP, 1 Reset,
2 NMI, 3 HardFault, 11 SVCall, 14 PendSV, 15 SysTick. **Peripheral IRQ *n* is
vector 16 + *n*** at address `0x08000000 + (16 + n) × 4`.

**Weak aliases.** Every handler in `startup.c` is a **weak** alias of
`Default_Handler` (an infinite loop). Your ordinary (strong) function of
**exactly** the same name replaces it at link time; **the name is the
registration**. Misspell it and you have defined an unused function: no
error, no warning, the weak default stays in the vector, and the first
interrupt hangs the board. The linker even deletes your misspelt function as
dead code. To check, `nm Main.elf` should show your handler as `T` (yours), not
`W` (weak).

**`Reset_Handler`, in order:** copy `.data` (flash → RAM), zero `.bss`,
`SystemInit()` (clock), `__libc_init_array()` (constructors), `main()`, then
`for(;;)`, because there is nothing to return to.

**Linker symbols are addresses, not variables.** `extern uint32_t _sdata;`
creates no storage. The symbol's *value* is the address. **Use it only with
`&`:** `&_sdata` is the address. `_sdata` alone reads whatever 4 bytes are
there: it compiles cleanly and gives garbage.

**`link.ld`:**

```
_estack = ORIGIN(RAM) + LENGTH(RAM);       /* 0x20003000, computed */
.text : { *(.text*) } >FLASH
.data : { _sdata = .; *(.data*) _edata = .; } >RAM AT> FLASH
_sidata = LOADADDR(.data);
.bss  : { _sbss = .; *(.bss*) *(COMMON); _ebss = .; } >RAM
```

`>RAM AT> FLASH` means the section *lives* in RAM and is *stored* in flash.
`LOADADDR()` gives the flash copy. `.` is the location counter. The stack
grows **down** from `_estack`.

**Reading `size -A`.** RAM used = `.data` + `.bss` + heap/stack reservation.
Flash used = vectors + `.text` + `.rodata` + **`.data` again**.

**The clock: 12 → 48 MHz.** The STM32C0 wakes on HSI48 ÷ 4 = 12 MHz.
`SystemInit()` sets one flash wait state (`FLASH->ACR` `LATENCY`), waits for
it, clears `RCC->CR` `HSIDIV` (÷1), waits for `HSIRDY`, then sets
`SystemCoreClock = 48000000`. **Rule: wait states go up *before* the frequency
and come down *after* it.** In the wrong order the CPU fetches before the flash
is ready and hangs, somewhere unrelated to the bug.

**`printf` needs `_write()`.** The C library calls `_write(fd, buf, len)`, and
`retarget.c` implements it by polling USART2: wait for `TXE`, then write `TDR`.
`BRR = 48 000 000 / 115 200 ≈ 417`. `_sbrk()` hands `malloc` (and `printf`'s
buffer) the gap between `.bss` and the stack. Get `_sbrk` wrong and `printf`
prints nothing.

**The board.** Nucleo-C031C6. User LED LD4 on **PA5**. USART2 TX on **PA2**, RX
on **PA3**, both **AF1**. The LED bar is on PB0–PB7. A simulator has a stated
range: Wokwi does not model DMA, RTC, IWDG, PWR, SYSCFG or DBG.

**Lab lesson to remember.** Moving `.isr_vector` after `.text` builds with no
error and no warning and produces a valid hex file, but a real chip can't
start, because the first two words are now instruction bytes.

Re-read: slides 3–21.

---

### Lesson 05 — GPIO and Interrupts

**GPIO in the model.** Clock gate `RCC->IOPENR`. Control: `MODER`, `OTYPER`,
`OSPEEDR`, `PUPDR`, `AFR`. Data in: `IDR`. Data out: `ODR`, `BSRR`, `BRR`.
Events are **not** in GPIO; a separate block, **EXTI**, handles them.

**Hex to binary.** One hex digit is exactly 4 bits. Bits are numbered 31 (left)
to 0 (right). Read `0xEBFFFFFF` as `1110 1011 1111 …`. Know the three bit
idioms:

| C | Effect |
|---|---|
| `1u << n` | only bit *n* set |
| `x & ~mask` | clear the bits that are 1 in `mask` |
| `x \| value` | set the bits that are 1 in `value` |

**Every GPIO port register** (same offsets for every port):

| Offset | Register | Bits per pin | Job | GPIOA reset value |
|---|---|---|---|---|
| `0x00` | `MODER` | 2 | 00 input, 01 output, 10 alternate function, 11 analog | `0xEBFFFFFF` |
| `0x04` | `OTYPER` | 1 | 0 push-pull, 1 open-drain | `0x00000000` |
| `0x08` | `OSPEEDR` | 2 | edge speed: 00 very low … 11 very high | `0x0C000000` |
| `0x0C` | `PUPDR` | 2 | 00 none, 01 pull-up, 10 pull-down, 11 reserved | `0x24000000` |
| `0x10` | `IDR` | 1 | read pin levels (read-only) | — |
| `0x14` | `ODR` | 1 | output levels | `0x00000000` |
| `0x18` | `BSRR` | 1+1 | write-only: bit *n* sets, bit *n*+16 resets | — |
| `0x1C` | `LCKR` | — | freeze configuration | — |
| `0x20`, `0x24` | `AFR[0]`, `AFR[1]` | 4 | alternate-function number, pins 0–7 / 8–15 | `0x00000000` |
| `0x28` | `BRR` | 1 | write-only: bit *n* resets | — |

**The one rule: pin *n* lives at *n*, 2*n* or 4*n*.**

| Register | Pin *n*'s bits |
|---|---|
| `OTYPER`, `IDR`, `ODR`, `BRR` | bit *n* |
| `MODER`, `OSPEEDR`, `PUPDR` | bits 2*n*+1 : 2*n* |
| `AFR[n / 8]` | bits 4(*n* mod 8)+3 : 4(*n* mod 8) |
| `BSRR` | set: bit *n*; reset: bit *n*+16 |

Examples: pin 5 → `MODER` bits 11:10. Pin 13 → `MODER` bits 27:26, `AFR[1]`
bits 23:20.

**Reset state.** Every pin wakes **analog (11)** except **PA13/PA14** (`10`,
alternate function), which are the SWD debug lines. **An analog pin reads 0 in
`IDR`.** An active-low button on an analog pin therefore reads "pressed"
forever. Input mode must be *written*, never assumed.

**Push-pull vs open-drain.** Push-pull drives both high and low. Open-drain
drives low only and lets go for high, so a pull-up resistor makes the high.
Two open-drain outputs can share a wire safely: it is a **wired-AND**, which is
how I²C works. Two push-pull outputs fighting on one wire make a short.

**Pull resistors.** A floating input reads randomly. A button to GND needs a
**pull-up**: released reads 1, pressed reads 0. That is **active low**.

**Clear-then-set** (how `pin_mode()` works). For example, PA5 as output:

```c
GPIOA->MODER = (GPIOA->MODER & ~(3u << 10)) | (1u << 10);
```

`3u << 10` = `0x00000C00` is the mask. `~` of it is `0xFFFFF3FF`. AND with
`0xEBFFFFFF` gives `0xEBFFF3FF`. OR with `0x400` gives **`0xEBFFF7FF`**. Only
bits 11:10 changed.

- `MODER = 1u << 10` alone would set all other pins to input, including the
  debug pins.
- `MODER |= 1u << 10` alone would leave `11` as `11`, because OR can't clear
  bits.

**`ODR` vs `BSRR`.** `ODR |= 1u << 5` is read-modify-write: three instructions.
If an ISR changes `ODR` in the gap, `main()` writes back a stale copy and the
ISR's change is lost. **`BSRR = 1u << 5` is one store, which is atomic.** In
`BSRR`, a 0 bit means "leave alone", so one write can set some pins, reset
others and ignore the rest:

```c
GPIOB->BSRR = ((uint32_t)(uint8_t)~leds << 16) | leds;   /* 8 LEDs at once */
```

**`AFR`.** `MODER = 10` says *a peripheral drives this pin*. `AFR` says *which*
peripheral. The number comes from the **datasheet's** alternate-function table
and differs per pin: USART2 on PA2/PA3 is AF1. A wrong AF gives a pin that
looks configured and carries nothing.

**Order of initialisation:** clock (`IOPENR`) → mode (`MODER`) →
pulls/type/speed → use. Every peripheral follows **clock, configure, use**.

**Bounce and debouncing.** A press is 10–100 transitions over about 1 ms.
Lesson 05 polls button A every 1 ms and accepts a new level only after it has
been **unchanged for `DEBOUNCE_MS` (20) samples in a row**. Every change
restarts the count, so the event is reported `DEBOUNCE_MS` after the *last*
bounce. Polling at 1 kHz is also a filter: it misses most bounces between
samples, and it depends on the loop coming back.

**The interrupt path: pin → EXTI → NVIC → vector → handler.**

| Hop | Register | Job |
|---|---|---|
| 1. EXTI port select | `EXTI->EXTICR[n/4]`, 8 bits per line | which port feeds line *n*: code 0 = A, 1 = B, 2 = C, 3 = D, 5 = F |
| 2. EXTI edge/pending/mask | `RTSR1`, `FTSR1` | trigger on rising / falling edge |
| | `RPR1`, `FPR1` | rising / falling edge **pending**; **write 1 to clear** |
| | `IMR1` | 1 = unmasked, pass on to the NVIC |
| 3. NVIC | `ISER` | enable IRQ *n* (bit *n*; writing 0 does nothing) |
| | `IPR[n/4]` | priority byte; only the top 2 bits exist: `prio << 6` |
| 4. vector | `0x08000000 + (16 + IRQ) × 4` | the handler's address, with the Thumb bit |
| global | `PRIMASK` | 0 = interrupts on (the one switch that starts on) |

Key facts:

- **Line *n* watches pin *n* of one port only.** PA1 and PB1 share line 1 and
  cannot both interrupt. Port A's code is 0, which is also the reset value, so
  forgetting `EXTICR` works by luck on port A only. On the STM32C0, `EXTICR` is
  in EXTI. On the F4 it is in `SYSCFG`.
- Clear pending flags with **`=`**, never `|=`. `|=` reads other lines'
  pending 1s and writes them back, clearing edges nobody has handled yet.
- **Unmask (`IMR1`) last**, after the edges are configured and old pending
  flags are cleared.
- **Shared vectors.** IRQ 5 = EXTI lines 0–1, IRQ 6 = lines 2–3, IRQ 7 =
  lines 4–15. The handler must check the pending bits to find which line fired.
- **The handler is an ordinary function** returning with `bx lr`. It uses only
  registers the hardware already saved.
- **Forget to clear the pending bit** and the handler runs forever. The board
  "freezes", but it hasn't crashed.
- **Handlers record; `main()` decides.** Don't wait 20 ms in a handler: it
  blocks everything of equal or lower priority. The handler counts edges, and
  `main()` debounces with a 20 ms "quiet" stopwatch.

**Seven ways to get silence** (slide 33): no clock gate, no input mode, no
pull-up, wrong `EXTICR`, `IMR1` not set, NVIC not enabled, handler name
misspelt. The opposite failure is not clearing the pending bit, which makes the
handler run forever.

**Debugger tricks** (slide 33b): latch a button with Ctrl-click; break where
the event is *decided*; inject an edge by writing `EXTI->SWIER1`, or pend an
IRQ by writing `NVIC->ISPR`.

Re-read: every slide. This is the largest lesson.

---

### Lesson 06 — Time and Timers

**SysTick** (ARM core, `0xE000E010`, the same on every Cortex-M):

| Register | Job |
|---|---|
| `CTRL` | `ENABLE` (bit 0), `TICKINT` (bit 1, interrupt), `CLKSOURCE` (bit 2), `COUNTFLAG` (bit 16, remembers only **one** wrap) |
| `LOAD` | reload value = period − 1, **24 bits** |
| `VAL` | current count, counts **down**; any write clears it |
| `CALIB` | read-only |

`SysTick_Config(SystemCoreClock / 1000)` gives a 1 ms tick: `LOAD = 47999`,
priority 3 (lowest), `CTRL = 0x7`. The longest period is 2²⁴ clocks ≈ 350 ms at
48 MHz. **`SysTick_Handler` has no flag to clear**; taking the exception
acknowledges it.

**Why interrupt instead of poll.** A polled `COUNTFLAG` loses ticks whenever
the loop is busy (a `printf` blocks for ms). An interrupt-driven `ms_ticks++`
keeps counting through anything.

**Wrap-safe time.** A `uint32_t` millisecond counter wraps after 2³² ms ≈
**49.7 days**.

```c
if (millis() - start >= period)    /* correct, always: modulo-2^32 subtraction */
if (millis() >= deadline)          /* WRONG once the counter wraps */
```

**Cooperative scheduling** (superloop):

```c
if (now - ping_at >= 100) { ping_at += 100; ... }
```

Use `+= period`, not `= now`, so the jobs stay on a fixed grid and lateness
doesn't accumulate. The weakness is that one slow job delays all the others
(lesson 07 fixes this).

**Timer register groups** (TIM3 at `0x40000400`, TIM14 at `0x40002000`; both
16-bit):

| Group | Registers | Job |
|---|---|---|
| time base | `PSC`, `ARR`, `CNT` | count every PSC+1 clocks, from 0 to ARR, then update |
| events | `EGR` (`UG`), `DIER` (`UIE`, `CC1IE`), `SR` (`UIF`, `CC1IF`) | force an event, enable interrupts, flags |
| channel | `CCMR1`, `CCER`, `CCR1` | compare (make a waveform) or capture (timestamp an edge) |
| control | `CR1` (`CEN`, `ARPE`) | run, preload |

**The time-base formulas:**

```
counter frequency = f_clk / (PSC + 1)
update frequency  = f_clk / ((PSC + 1) × (ARR + 1))
```

For 1 MHz counting and a 20 ms period at 48 MHz: `PSC = 47`, `ARR = 19999`.
**Both registers hold N − 1**; forgetting the −1 is about 2 % off, and only a
cross-check notices.

**Shadow registers.** `PSC`, `ARR` and `CCR1` are double-buffered: writes go to
a preload register and take effect at the next update event. Write `EGR = UG`
to force an update immediately, then clear the `UIF` that `UG` raised, or the
first interrupt is a phantom.

**Timer flags clear by writing 0.** `TIM3->SR = ~TIM_SR_UIF;` clears only
`UIF`, because writing 1 to the others leaves them alone. Writing
`TIM3->SR = TIM_SR_UIF` (the EXTI style) clears every *other* flag, leaves
`UIF` set, and the handler re-enters forever.

**PWM (output compare)**, mode 1 (`OC1M = 110`): the pin is high while
`CNT < CCR1`. Counting at 1 MHz, `CCR1` is the pulse width in µs. A servo
reads a pulse repeated every 20 ms: nominally 1.0 ms at one end, 1.5 ms at the
centre and 2.0 ms at the other end. To enable it: `CCMR1` (mode + `OC1PE`
preload), `CCER.CC1E` (connect to pin), `CR1.ARPE`, and the pin's AF. The CPU
is involved in **no** edge; the ISR only changes `CCR1`.

**AF numbers depend on the pin.** `TIM3_CH1` is AF1 on PA6 but AF12 on PB6.
`TIM14_CH1` is AF4 on PA7. With a wrong AF the timer works and the pin
doesn't.

**Rate limiter.** Each frame, the update ISR moves `CCR1` toward the target by
at most `SLEW` µs. This is the smallest control loop: measure, compare,
correct, once per period.

**Input capture.** On an edge, the hardware copies `CNT` into `CCR1`, so the
CPU is out of the measurement and interrupt latency doesn't matter. With both
edges captured (`CC1P = CC1NP = 1`), the width is
`(uint16_t)(t_fall − t_rise)`, which is correct across a 16-bit wrap. **Reading
`CCR1` clears `CC1IF`.** The HC-SR04 echo is ~58 µs per cm. (Wokwi doesn't
implement capture, so the lesson also timestamps with EXTI + `TIM14->CNT`.)

**Tearing again.** An ISR writes two related values (width and count). `main()`
reads both inside `__disable_irq()` … `__enable_irq()` so it gets a consistent
pair.

**Priorities follow deadlines** (deadline-monotonic). TIM14 capture = 1 (must
read before the next edge), TIM3 update = 2 (20 ms of slack), SysTick = 3 (a
late tick is still counted). Importance to the user is *not* the criterion.
Equal priorities don't preempt each other; they queue, lower IRQ number first.

**Self-test.** Counting TIM3 updates per SysTick second must give exactly 50.
Two independent clock paths that must agree.

**Three clearing conventions so far:** EXTI write 1; TIM `SR` write 0; `CC1IF`
read `CCR1`.

Re-read: slides 1–18.

---

### Lesson 07 — RTOS

**Cooperative vs preemptive.**

| Scheduler | Who stops a job | One **CPU hog** (a job that never returns) |
|---|---|---|
| cooperative (superloop) | the job itself, by returning | freezes the whole system |
| preemptive (RTOS) | the kernel, on a timer interrupt | is **preempted** |

**Terms.**

| Term | Meaning |
|---|---|
| task | a C function that runs as if it had the CPU to itself |
| context | everything it needs to continue: registers + stack pointer |
| context switch | save one task's context, restore another's |
| TCB (task control block) | the kernel's record of a task; `sp` is its **first** field |

Each task has its own stack. Its registers are saved *on that stack*, so the
TCB only needs to remember the saved stack pointer.

**Two stack pointers.** **MSP** is used by every exception handler (and by
`main()` after reset). **PSP** is used by tasks in Thread mode when
`CONTROL.SPSEL = 1`. Handlers never need room on a task's stack beyond the
8-word frame. **`EXC_RETURN = 0xFFFFFFFD`** means "return to Thread mode, use
PSP". `0xFFFFFFF9` means "Thread mode, MSP" and `0xFFFFFFF1` means "Handler
mode". Bit 2 of `EXC_RETURN` = 1 means PSP.

**Special registers** (reached only by `mrs`/`msr`): `xPSR`, `MSP`, `PSP`,
`PRIMASK`, `CONTROL`. No `BASEPRI`/`FAULTMASK` on the M0+.

**SCB** (`0xE000ED00`, ARM core): **`ICSR`** pends PendSV (`PENDSVSET`, bit 28)
or cancels it (`PENDSVCLR`). It is write-1-to-act, so write it with `=`.
**`SHPR3`** holds the PendSV and SysTick priorities (both set to 3).

**PendSV.** A context switch must never happen *inside* another handler.
PendSV is a system exception that is **pended by software** and given the
**lowest priority**, so it runs only when no other handler is active.

| | Deciding: `schedule()` | Switching: `PendSV_Handler` |
|---|---|---|
| does | picks the next task; pends PendSV if it differs | saves the old context, restores the new one |
| runs | anywhere: tick, `os_delay`, mutex unlock, queue put (tasks and ISRs) | one place only |
| can change its mind | yes (`PENDSVCLR`) | no |

**One switch, step by step:**

1. Hardware pushes **half** the context of task A (`r0`–`r3`, `r12`, `lr`,
   `pc`, `xPSR`) onto A's stack.
2. `SysTick_Handler` decides: it wakes sleepers and `pick()` chooses.
3. PendSV is pended.
4. PendSV runs when nothing else is active.
5. `PendSV_Handler` saves **`r4`–`r11`** (the other half) on A's stack, stores
   A's SP in A's TCB, loads B's SP and restores B's `r4`–`r11`.
6. It returns with `0xFFFFFFFD`, and the hardware pops B's eight words,
   including B's `pc`.

**Why `r8`–`r11` go the long way round.** In Thumb-1, `stm`/`ldm` reach only
`r0`–`r7`. So the kernel copies `r8`–`r11` into `r4`–`r7` (already saved)
with `mov` and stores them. On restore it does `r8`–`r11` *first* and `r4`–`r7`
last.

**Forging the first frame.** A new task has never run, so `os_task_create()`
builds the stack it *would* have if it had just been interrupted:

- `xPSR = 0x01000000` (Thumb bit set, or the task HardFaults)
- `pc` = the task function
- `lr` = `task_exit`
- `r0` = the argument (AAPCS)
- zeros everywhere else

**`pick()`.** **Fixed-priority preemptive:** the highest-priority `READY` task
runs. **Round-robin** among equal priorities, one tick each. **Idle**
(priority 0) is always ready.

**Two priority numberings on one chip:**

| | This kernel / FreeRTOS | NVIC |
|---|---|---|
| most urgent | **highest number** | **0** |

**Task states.**

| State | Means | Leaves when |
|---|---|---|
| `READY` | could run now (the running one is the `READY` task `pick()` chose) | picked |
| `SLEEPING` | waiting for a time (`os_delay`, `os_delay_until`) | the tick reaches `wake` |
| `BLOCKED` | waiting for a thing (mutex held, queue empty/full) | the owner unlocks / an item arrives |

**Busy-waiting vs sleeping.** A busy-waiting task spins at 100 % CPU and
nobody else runs. A sleeping task is off the CPU and every other task runs.
`os_delay_until(&last, period)` keeps a fixed grid, like lesson 06's
`+= period`. The per-task tick count is **statistical profiling** (CPU share).

**Mutex vs masking interrupts.**

| | Masking (`PRIMASK`) | Mutex |
|---|---|---|
| who is held up | everything, including every interrupt | only tasks that want this resource |
| how long | a few instructions | as long as the work takes |
| waiter | — | `BLOCKED`, off the CPU |
| usable from an ISR | yes | **no**: an ISR can't block |

The kernel's own bookkeeping, inside the mutex, is protected by masking for a
few instructions.

**Priority inversion.** High (H) needs a mutex that Low (L) holds. Medium (M),
which doesn't use the mutex, preempts L. L can't release the mutex, so **H
waits for M**, a task it outranks. **Blocking** (H waits for L) is expected and
bounded. **Unbounded inversion** lasts as long as M runs. Mars Pathfinder,
1997. **Fix: priority inheritance.** While L holds a mutex H wants, L runs at
H's priority, so M can't preempt it. On unlock, L returns to `base_prio`.

**Queues.** Copy data instead of sharing it. `put` blocks while full, `get`
blocks while empty. **`os_queue_put_from_isr()` never blocks**: it returns 0
when full, because a handler has no task to put to sleep. It can make a waiting
task `READY`, and the switch happens when the last handler returns.

**Stacks.** Each stack is pre-filled with `0xDEADBEEF`. After exercising every
path, the untouched words give the **high-water mark**. Size each stack to the
measured use plus a margin. `printf` costs ~115 words. **A stack overflow on an
MCU usually doesn't crash**: it quietly corrupts *another* task's data, far from
the cause.

**HardFault reporting.** The stacked frame word 6 is the faulting `pc`. Bit 2
of `LR` (`EXC_RETURN`) says which stack (PSP = a task). `addr2line` maps the
`pc` to a source line. The M0+ has no fault-status registers, so you learn
*where* but not *why*.

**Where this kernel sits.** FreeRTOS's `xTaskCreate`, `vTaskDelayUntil` and
`xQueueSendFromISR` correspond to this kernel's `os_task_create`,
`os_delay_until` and `os_queue_put_from_isr`. The RTOS switch is cheap (tens of
cycles) because it leaves out what a desktop OS must switch: address space
(MMU), FPU state and TLB.

Re-read: slides 1–19.

---

### Lesson 08 — UART and a Python Host

**USART2 in the model.**

| Slot | USART2 |
|---|---|
| clock gate | `RCC->APBENR1` `USART2EN` |
| pins | PA2 TX, PA3 RX, AF1 |
| control | `CR1`: `UE` (0), `RE` (2), `TE` (3), `RXNEIE` (5), `TXEIE` (7) |
| status | `ISR`: `ORE` (3, overrun), `RXNE` (5, a byte arrived), `TXE` (7, room to send) |
| clear | `ICR`: write 1 (`ORECF`) |
| data | `RDR` in (reading clears `RXNE`); `TDR` out (writing clears `TXE`) |
| rate | `BRR` = f_clk / baud |
| event | `USART2_IRQn` = 28: one vector for everything |

Defaults left alone: 8 data bits, no parity, 1 stop bit (**8N1**).

**Baud arithmetic.** `BRR = 48 000 000 / 115 200 = 416.67 → 417` (0.08 %
error; a UART tolerates a few percent because the receiver resynchronises on
every start bit). One byte is **10 bits** on the wire (start + 8 data + stop),
so at 115 200 baud: **11 520 bytes/s, 86.8 µs per byte**. A 90-character line
takes about 8 ms.

**Overrun (`ORE`).** The UART holds **one** received byte. If software hasn't
read it within one byte time (86.8 µs), the next byte overwrites it. Receive
must therefore be interrupt-driven. If `ORE` is not cleared, reception stops.

**Receive design: producer/consumer.** The ISR reads `RDR` and calls
`os_queue_put_from_isr()`, which meets the hard 86.8 µs deadline in a few
dozen instructions. The protocol task blocks in `os_queue_get()` and costs
nothing while idle. The queue turns a hard deadline into a soft one.

**Transmit design: a lock-free ring.** `tx_head` is written only by tasks
(producer). `tx_tail` is written only by the ISR (consumer). Each index has
**one writer**, so no lock is needed. This is lesson 03's safe case, twice.

- **Data before index.** Store the byte, *then* advance `head`.
- **One slot always empty.** `head == tail` means *empty*. If the ring could
  fill completely, full would look the same.

**`TXEIE` must be on only while there is work.** `TXE` is set whenever the
transmit register is empty, which is *always* when idle. If `TXEIE` stays on
with an empty ring, the interrupt fires continuously and starves everything
below it. (Measured: not even the banner printed.) The ISR turns `TXEIE` off
when the ring empties, and `_write()` turns it on again. That read-modify-write
of `CR1`, shared with the ISR, is a critical section.

**Weak `_write()` again.** `retarget.c`'s polled `_write` is `weak`. The
lesson's `uart.c` defines a strong one that wins at link time, the same
mechanism as the vector table. `printf` now returns as soon as the text is in
the ring.

**A mutex around printing.** Two tasks printing at once interleave characters,
and newlib-nano's `printf` is **not re-entrant**. `say()` wraps `vprintf` in
`os_mutex_lock`/`unlock`.

**Two audiences, one line format.** A line starting with `$` is a machine
**frame**. Any other line is a human **shell command** (`led 170`, `help`).
Lines over 63 characters are discarded whole.

**The frame: NMEA 0183, the GPS format.**

```
$LED,170*57      checksum = XOR of every byte between '$' and '*', 2 hex digits
```

XOR detects **any single flipped bit**. It does **not** detect two errors in
the **same bit position of different bytes**, because they cancel. A CRC
catches those; MAVLink (lessons 13–14) uses one. Replies are `$ACK,...` or
`$NAK,CHECKSUM` / `$NAK,NOSTAR`. **A frame that fails its checksum is thrown
away and counted, never half-used.**

**`host.py`.** Python with pyserial. `--selftest` checks the checksum against a
real GPS sentence. `--frame` builds a frame. `--file` checks a saved log, and
counts sequence gaps as lost frames. `--port rfc2217://localhost:4000` talks to
the Wokwi VS Code extension; `--port COM5` talks to a real board. A printable
protocol survives copy-and-paste; a binary one wouldn't.

**UART silent-failure checklist:**

| Forgot | Symptom |
|---|---|
| `USART2EN` | nothing sent or received |
| AF1 on PA2/PA3 | UART works internally, pins carry nothing |
| `BRR` for the wrong clock | **garbage characters** |
| clearing `ORE` | reception stops after the first overrun |
| turning `TXEIE` off when empty | interrupt storm |
| blocking in an ISR | impossible to do right |
| a lock around printing | interleaved lines, failed checksums |

**Five clearing conventions on one chip:**

| Flag | Cleared by |
|---|---|
| EXTI `RPR1`/`FPR1` | write **1** to it |
| TIM `UIF` | write **0** to it |
| TIM `CC1IF` | **read** `CCR1` |
| USART `RXNE` / `TXE` | read `RDR` / write `TDR` |
| USART `ORE` | write 1 to a **different** register, `ICR` |

*For every flag, look up how it clears.*

Re-read: slides 1–15.

---

## 4. Formula and fact sheet

| Quantity | Formula / fact |
|---|---|
| Bits per hex digit | 4 |
| GPIO bit position, pin *n* | 1-bit regs: *n* · 2-bit regs: 2*n*+1:2*n* · `AFR[n/8]`: 4(*n* mod 8)+3:4(*n* mod 8) · `BSRR`: *n* set, *n*+16 reset |
| `MODER` codes | 00 in · 01 out · 10 AF · 11 analog |
| `PUPDR` codes | 00 none · 01 up · 10 down |
| Vector number of IRQ *n* | 16 + *n* |
| Vector address | `0x08000000 + (16 + n) × 4` |
| NVIC priority byte | `IPR[n/4]`, byte *n* mod 4, value `prio << 6` (4 levels, 0 most urgent) |
| Timer count rate | f / (PSC + 1) |
| Timer update rate | f / ((PSC + 1)(ARR + 1)) |
| SysTick `LOAD` | ticks − 1 (24 bits max) |
| UART `BRR` | f / baud (rounded) |
| UART byte time | 10 / baud |
| `uint32_t` ms wrap | 2³² ms ≈ 49.7 days |
| Elapsed time | `now - start` (unsigned) |
| Memory | Flash 32 KB @ `0x08000000` · SRAM 12 KB @ `0x20000000` · top of RAM `0x20003000` |
| Bases | RCC `0x40021000` · USART2 `0x40004400` · TIM3 `0x40000400` · EXTI `0x40021800` · GPIOA `0x50000000` · GPIOB `0x50000400` · SysTick `0xE000E010` · NVIC `0xE000E100` · SCB `0xE000ED00` |
| Clock | reset 12 MHz (HSI48 ÷ 4) → 48 MHz with 1 flash wait state |
| `EXC_RETURN` | `0xFFFFFFF9` Thread/MSP · `0xFFFFFFFD` Thread/PSP · `0xFFFFFFF1` Handler |
| Hardware-stacked registers | `r0`–`r3`, `r12`, `lr`, `pc`, `xPSR` (8 words) |

---

## 5. Glossary

| Term | Meaning |
|---|---|
| AAPCS | ARM's calling convention: who owns which register across a call |
| Active low | the interesting state reads 0 (a pressed button with a pull-up) |
| Alternate function (AF) | a pin handed to a peripheral; the number selects which |
| As-if rule | the compiler need only preserve observable behaviour |
| Atomic | cannot be observed half-done |
| Bounce | a mechanical contact opening and closing many times per press |
| Callee-saved | a register a function must restore before returning (`r4`–`r11`) |
| Clock gate | the RCC bit that powers a peripheral; off at reset |
| CMSIS | ARM's vendor-neutral headers for Cortex-M; headers, not a library |
| Context switch | saving one task's registers and stack pointer, restoring another's |
| Cooperative scheduling | each job runs until it returns |
| Critical section | code that must not be interleaved with another context |
| Deadline-monotonic | the shorter the deadline, the more urgent the priority |
| EXTI | the block that turns pin edges into interrupt requests |
| Harvard | separate code and data address spaces |
| High-water mark | the deepest a stack has ever reached |
| Literal pool | constants stored beside the code, loaded PC-relative |
| Load–store | only `ldr`/`str` touch memory; arithmetic uses registers |
| Memory-mapped I/O | peripherals at ordinary addresses |
| Mutex | a lock that blocks only the tasks wanting one resource |
| NVIC | ARM's interrupt controller: enable, pending, priority |
| Open-drain | output that can only pull low; wired-AND |
| Overrun | a received byte overwritten before software read it |
| PendSV | a software-pended, lowest-priority exception used for context switches |
| Preemption | a more urgent context displacing a running one |
| Priority inheritance | a mutex holder temporarily runs at its highest waiter's priority |
| Priority inversion | a task waits, in effect, on a lower-priority task it outranks |
| Race condition | the result depends on the timing of two contexts |
| Read-modify-write | load, change, store: three steps an interrupt can split |
| Shadow (preload) register | a buffered copy that takes effect at the next update |
| Tearing | reading one logical value in two accesses and getting half old, half new |
| TCB | task control block: the kernel's record of one task |
| Thumb bit | bit 0 of every code address on a Cortex-M; always 1 |
| `volatile` | every access to this object must really happen, in order |
| Weak symbol | a default definition that any ordinary definition replaces at link time |

---

## 6. Practice problems, with answers

Try each one before reading the answer. These are similar in kind and
difficulty to the exam, but they are not the exam's questions.

**P1.** Which bits of `GPIOB->MODER` belong to PB3, and what value makes PB3
an output?

**P2.** Write the single `BSRR` value that sets PB1 high and PB6 low and leaves
every other pin of port B alone.

**P3.** PA9 needs alternate function 1. Which register and which bits hold
PA9's AF number?

**P4.** `GPIOA->PUPDR` reads `0x24000000`. What does it read after enabling a
pull-up on PA4 with the clear-then-set idiom?

**P5.** `EXTI4_15_IRQn` is IRQ 7. What is its vector number, and at which
address is its vector?

**P6.** At 48 MHz, choose `PSC` and `ARR` for a timer that counts at 1 MHz and
updates every 1 ms.

**P7.** What value of `BRR` gives 9600 baud at 48 MHz, and how long does one
byte take on the wire?

**P8.** `start = 0xFFFFFF00` and now `millis()` returns `0x00000100`. What does
`millis() - start` give, and is a 500 ms timeout expired?

**P9.** Compute the NMEA checksum of the body `LED,1`.

**P10.** The first data record of a hex file contains the bytes
`00 30 00 20 C5 06 00 08`. What are the initial SP and the reset handler's
actual address?

**P11.** Sort these by where they live and whether they cost flash:
`const char msg[] = "hi";`, `int total;`, `int limit = 50;`.

**P12.** A global `uint32_t` is written only by an ISR and only read by
`main()`. Does it need `volatile`? Does it need a critical section?

**P13.** A task of priority 2 in this course's kernel and an interrupt of NVIC
priority 2. Which numbering treats a larger number as more urgent?

**P14.** Why can't an interrupt handler call `os_mutex_lock()`?

### Answers

**A1.** Bits 7:6 (2*n*+1:2*n* with *n* = 3). Write `01`.

**A2.** `(1u << 1) | (1u << (6 + 16))` = `0x00000002 | 0x00400000` =
**`0x00400002`**.

**A3.** `AFR[1]` (9 / 8 = 1). 9 mod 8 = 1, so bits 7:4. Write `0001`.

**A4.** PA4's field is bits 9:8. Clearing them leaves `0x24000000`, and OR-ing
`1 << 8` gives **`0x24000100`**.

**A5.** Vector 16 + 7 = 23. Address `0x08000000 + 23 × 4` = `0x08000000 + 92`
= **`0x0800005C`**.

**A6.** `PSC = 47` (48 MHz / 48 = 1 MHz). `ARR = 999` (1000 counts = 1 ms).

**A7.** 48 000 000 / 9600 = **5000**. One byte = 10 bits / 9600 ≈ **1.04 ms**.

**A8.** `0x00000100 − 0xFFFFFF00` = `0x00000200` = **512 ms**, modulo 2³².
Yes, 512 ≥ 500, so it has expired. Compare the wrong form,
`millis() >= start + 500`. Here `start + 500` wraps to `0x000000F4`. Only
16 ms after `start`, when `millis()` is `0xFFFFFF10`, that test is already
true, so the timeout fires about 484 ms early. Only the subtraction form is
always right.

**A9.** `L`=`0x4C`, `E`=`0x45`, `D`=`0x44`, `,`=`0x2C`, `1`=`0x31`.
`4C ^ 45 = 09`, `^ 44 = 4D`, `^ 2C = 61`, `^ 31 = 50`. Frame:
**`$LED,1*50`**.

**A10.** Little-endian: SP = **`0x20003000`** (top of 12 KB RAM). Reset vector
= `0x080006C5`, so the handler is at **`0x080006C4`** (bit 0 is the Thumb bit).

**A11.** `msg`: `.rodata`, flash only. `total`: `.bss`, RAM only, no flash.
`limit`: `.data`, RAM **and** flash.

**A12.** It needs `volatile`, so `main()` really reads it each time. It does
**not** need a critical section: it is a single aligned word with one writer,
so it can't tear.

**A13.** The kernel (like FreeRTOS): a higher number is more urgent. In the
NVIC, 0 is the most urgent.

**A14.** Locking may have to **block**, putting the caller to sleep until the
owner unlocks. A handler is not a task; there is nothing to put to sleep, so an
ISR must use non-blocking APIs such as `os_queue_put_from_isr()`.

---

## 7. How to prepare

1. **Answer every "What You Should Be Able to Say" slide** at the end of
   lessons 00–03 out loud, without looking.
2. **Walk the silent-failure checklists**: lesson 05 slide 33, lesson 06
   slide 17, lesson 08 slide 14. For each row, explain *why* that symptom
   follows.
3. **Practise the arithmetic** until it is automatic: pin bit positions,
   clear-then-set on a reset value, `BSRR` values, vector addresses, `PSC`/`ARR`,
   `BRR`, little-endian decoding, XOR checksums. Section 6 is a starting point;
   make up more with other pins and IRQs.
4. **Know how each flag clears** (lesson 08 slide 4) and **which priority
   numbering is in use** (lesson 07 slide 9).
5. **For each mechanism, know the general idea and this chip's answer**: I/O
   model, address spaces, instruction-set model, flags, return address,
   address binding, dispatch, mutual exclusion, scheduling. Many questions ask
   which one this chip uses and what the alternative is.
6. **Bring:** a blue or black pen. Nothing else is allowed on the desk.

Good luck.
