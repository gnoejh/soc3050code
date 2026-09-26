# Instruction Set: What the Processor Actually Does
## SOC3050 ARM Edition — Part 0, Models

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

The second of four lessons with **no code to write**.

Last lesson: the machine is a CPU, memory and peripherals on one bus, and a
register is an address with a side effect. This lesson: what the CPU can
actually *do* with an address.

Every disassembly and every statistic on these slides is real output from this
repository's own build — `_spike/c031c6/Main.elf`, compiled with `-Os` by the
same `arm-none-eabi-gcc` you will use. Nothing here is illustrative.

---

## Slide 1: You Will Not Write Assembly. You Will Read It.

Nobody in this course writes an application in assembly. But four times a term,
something will go wrong in a way that C cannot explain:

- the debugger stops somewhere with no source line
- a variable changes when nothing wrote to it
- a loop that should take 3 µs takes 30
- the optimiser deletes code you needed

Each of those is answered in one place: **the instructions the compiler
actually emitted**. Reading them is a diagnostic skill, like reading a stack
trace.

---

## Slide 2: Thumb — the Only Instruction Set This Chip Has

Before reading a single mnemonic, know which language it is written in. ARM
designs **two** instruction sets, and this processor can execute exactly one of
them.

| | **ARM (A32)** | **Thumb (T32)** |
|---|---|---|
| Instruction width | always 32 bits | 16 bits, widening to 32 only when it must |
| Same program, code size | baseline | **roughly 30% smaller** |
| Conditional execution | on every instruction | on branches |
| Registers in one instruction | all 16, always | R0–R7 in most 16-bit forms |
| Arrived | 1985, the original | 1994, in the ARM7**T**DMI |
| Executed by | Cortex-A, Cortex-R | Cortex-A, Cortex-R, **and every Cortex-M** |

```svg
<svg viewBox="0 0 560 220" role="img" aria-label="ARM instructions are always 32 bits; Thumb is mostly 16 bits; Cortex-M has only Thumb">
  <text x="138" y="26" text-anchor="middle" class="lbl">ARM (A32) — four instructions</text>
  <rect class="reg" x="20" y="38" width="55" height="26" rx="3"/>
  <rect class="reg" x="80" y="38" width="55" height="26" rx="3"/>
  <rect class="reg" x="140" y="38" width="55" height="26" rx="3"/>
  <rect class="reg" x="200" y="38" width="55" height="26" rx="3"/>
  <text x="138" y="82" text-anchor="middle" class="lbl">16 bytes</text>
  <text x="422" y="26" text-anchor="middle" class="lbl">Thumb (T32) — the same four</text>
  <rect class="hifill" x="305" y="38" width="27" height="26" rx="3"/>
  <rect class="hifill" x="337" y="38" width="27" height="26" rx="3"/>
  <rect class="hifill" x="369" y="38" width="55" height="26" rx="3"/>
  <rect class="hifill" x="429" y="38" width="27" height="26" rx="3"/>
  <text x="396" y="82" text-anchor="middle" class="lbl">10 bytes</text>
  <text x="504" y="56" text-anchor="middle" class="lbl">one widened</text>
  <path class="dash" d="M280 30 V92"/>
  <rect class="box" x="60" y="112" width="200" height="46" rx="6"/>
  <text x="160" y="130" text-anchor="middle">Cortex-A, Cortex-R</text>
  <text x="160" y="148" text-anchor="middle" class="lbl">both, switched at run time</text>
  <rect class="hifill" x="300" y="112" width="200" height="46" rx="6"/>
  <text x="400" y="130" text-anchor="middle">Cortex-M — this chip</text>
  <text x="400" y="148" text-anchor="middle" class="lbl">Thumb only. No ARM state exists.</text>
  <text x="280" y="192" text-anchor="middle" class="lbl">A third less flash for the same program is worth more to a 32 KB part</text>
  <text x="280" y="210" text-anchor="middle" class="lbl">than anything the wider encoding was buying.</text>
</svg>
```

A phone's Cortex-A holds both and switches between them as it runs. **A
Cortex-M has no ARM state to switch to.** Thumb is not a mode you opt into
here; it is the only thing the decoder understands.

**The general idea behind all of this is code density** — how much program you
get per byte of memory — and it is a standing tension in instruction-set design.
Wide fixed-length instructions decode trivially and waste space; variable-length
ones (x86 runs from 1 to 15 bytes) pack tightly and make the decoder's first job
*working out where the next instruction starts*. The compromise most RISC
families reached is a **compressed instruction set**: a narrow encoding of the
common cases, sitting beside the full-width one. Thumb is ARM's; MIPS16 was
MIPS's; RISC-V calls its version the **"C" extension**. Every one of them exists
because memory, not arithmetic, is what a small system runs out of first.

You do not have to take that on faith — the build says so. This is the
runtime library the linker actually pulled in, straight out of `Main.map`:

```
.../lib/gcc/arm-none-eabi/15.2.1/thumb/v6-m/nofp/libgcc.a(_udivsi3.o)
```

Three directory names, three decisions: **`thumb`** the instruction set,
**`v6-m`** the architecture, **`nofp`** no floating-point unit. GCC ships a
separate copy of its runtime for each combination, and that is the copy your
chip got.

---

## Slide 3: The Good News — There Is Not Much to Learn

The whole spike program is **2100 instructions** built from just **51 distinct
mnemonics**. And they are not evenly used:

| Instruction | Share | Running total |
|---|---|---|
| `movs` | 15.9% | 15.9% |
| `ldr` | 15.3% | 31.2% |
| `cmp` | 8.3% | 39.5% |
| `str` | 8.0% | 47.5% |
| `adds` | 5.7% | 53.2% |
| `bl` | 4.3% | 57.5% |
| `lsls` | 4.0% | 61.5% |
| `b` | 3.9% | 65.4% |
| `bne` | 3.6% | 69.0% |
| `beq` | 3.5% | 72.5% |
| `subs` | 2.5% | 75.0% |
| `push` / `pop` | 4.6% | **79.6%** |

```svg
<svg viewBox="0 0 560 150" role="img" aria-label="Thirteen mnemonics account for eighty percent of all instructions">
  <rect class="reg" x="20" y="40" width="520" height="34" rx="5"/>
  <rect class="hifill" x="20" y="40" width="414" height="34" rx="5"/>
  <text x="227" y="58" text-anchor="middle">13 mnemonics = 80% of all code</text>
  <text x="487" y="58" text-anchor="middle" class="lbl">the other 38</text>
  <text x="280" y="106" text-anchor="middle" class="lbl">Move things. Add things. Compare things. Jump somewhere.</text>
  <text x="280" y="126" text-anchor="middle" class="lbl">That is most of what a processor does.</text>
</svg>
```

Learn those thirteen and you can read almost any disassembly on this chip. The
next five slides are those thirteen, by job.

---

## Slide 4: Load / Store — Two Machine Models, and Which One This Is

Every instruction set answers one question early and never revisits it: **may an
arithmetic instruction name a memory address?** There are two answers, and both
have names you will meet in any architecture text.

**Register–memory.** x86, the 68000, the IBM S/360. An operand may be an
address, so one instruction fetches, computes and writes back:

```
add  dword [counter], 1        ; x86 — one instruction, memory += 1
```

**Load–store.** ARM, RISC-V, MIPS — and the AVR you already know. The ALU sees
registers and nothing else. Memory is reached only by instructions whose entire
job is moving a word across the boundary:

```
ldr   r0, [r1]                 @ Cortex-M — the same job, in three
adds  r0, r0, #1
str   r0, [r1]
```

Both fragments increment the same counter. The second says out loud what the
first one hides.

```svg
<svg viewBox="0 0 560 258" role="img" aria-label="Register-memory lets one instruction span memory and the ALU; load-store puts a boundary between them that only LDR and STR cross">
  <defs><marker id="m4" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>

  <text x="14" y="16" class="lbl">REGISTER–MEMORY — x86, 68000</text>
  <rect class="dash" x="10" y="26" width="540" height="64" rx="7"/>
  <rect class="box" x="26" y="39" width="104" height="38" rx="5"/>
  <text x="78" y="58" text-anchor="middle">memory</text>
  <path class="wire" d="M134 49 H222" marker-end="url(#m4)"/>
  <path class="wire" d="M222 67 H134" marker-end="url(#m4)"/>
  <rect class="box" x="226" y="39" width="84" height="38" rx="5"/>
  <text x="268" y="58" text-anchor="middle">ALU</text>
  <text x="330" y="50" class="mono lbl">add dword [counter], 1</text>
  <text x="330" y="68" class="lbl">one instruction spans the whole dashed box</text>

  <text x="14" y="116" class="lbl">LOAD–STORE — ARM, RISC-V, AVR</text>
  <rect class="box" x="26" y="130" width="96" height="38" rx="5"/>
  <text x="74" y="149" text-anchor="middle">memory</text>
  <path class="dash" d="M152 114 V214"/>
  <text x="152" y="228" text-anchor="middle" class="lbl">only ldr / str cross here</text>
  <path class="hi" d="M124 140 H230" marker-end="url(#m4)"/>
  <text x="177" y="128" text-anchor="middle" class="mono lbl">ldr</text>
  <path class="hi" d="M230 158 H124" marker-end="url(#m4)"/>
  <text x="177" y="172" text-anchor="middle" class="mono lbl">str</text>
  <rect class="hifill" x="234" y="130" width="96" height="38" rx="5"/>
  <text x="282" y="149" text-anchor="middle">registers</text>
  <path class="wire" d="M334 140 H420" marker-end="url(#m4)"/>
  <path class="wire" d="M420 158 H334" marker-end="url(#m4)"/>
  <rect class="box" x="424" y="130" width="84" height="38" rx="5"/>
  <text x="466" y="149" text-anchor="middle">ALU</text>
  <text x="372" y="192" text-anchor="middle" class="lbl">arithmetic never sees an address</text>
  <text x="280" y="248" text-anchor="middle" class="lbl">Same work either way. Only one of them makes you write it down.</text>
</svg>
```

| | Register–memory | Load–store |
|---|---|---|
| Found in | x86, 68000, S/360 | **ARM**, RISC-V, MIPS, **AVR** |
| An ALU operand may be | a register **or** memory | a register only |
| Memory is touched by | most instructions | `ldr` / `str`, and nothing else |
| Instruction length | variable — x86 is 1 to 15 bytes | fixed — Thumb is 2 or 4 |
| `counter++` costs | 1 instruction | 3 instructions |
| …and bus accesses | 2 | 2 |

Read those last two rows together, because they are the whole trade:
**load–store costs instructions, not memory traffic.** The bus does identical
work either way; the only difference is whether the instruction stream admits
it. What the admission buys is a fixed-length encoding, a decoder that never has
to ask how long an instruction is before it can find the next one, and a
pipeline whose stages are the same depth for every opcode. A 48 MHz part with no
cache is built out of exactly those properties. This is most of what "RISC" and
"CISC" ever meant. (The VAX went further than x86 in the other direction —
*both* operands could be memory. Nobody builds those any more.)

**The AVR was load–store too, with one deliberate crack.** `sbi PORTB, 0` sets a
bit in a low I/O register in a single uninterruptible instruction, no `lds`/`sts`
around it — which is why `PORTB |= (1 << 0)` was something you never had to
think about. **Cortex-M0+ has no `sbi`.** Every peripheral register here is
ordinary memory at an ordinary address, reached the ordinary way. The
convenience comes back as *hardware* instead: STM32's GPIO has a set/reset
register, `BSRR`, that turns a bit-set back into one `str` — lesson 05.

### Crossing the boundary, on this chip

| Instruction | Moves | C equivalent |
|---|---|---|
| `ldr  r0, [r1]` | 32 bits, memory → register | `x = *p;` |
| `str  r0, [r1]` | 32 bits, register → memory | `*p = x;` |
| `ldrb` / `strb` | 8 bits | `char`, `uint8_t` |
| `ldrh` / `strh` | 16 bits | `uint16_t` |
| `ldrsh` | 16 bits, **sign-extended** | `int16_t` |
| `movs r0, #7` | a constant into a register | `x = 7;` |
| `movs r0, r3` | register to register | `x = y;` |

The last two rows are there for contrast: `movs` is the one that never touches
memory at all. Everything else in the deck operates on what `ldr` brought in.

So setting one bit in a peripheral register is never one instruction here. It is
a **read, a modify and a write** — three instructions, two bus events, and a gap
in between that belongs to whatever the processor does next. Lesson 03 shows
what goes wrong in that gap.

---

## Slide 5: Arithmetic and Logic

These operate on registers only, and almost all of them carry an `s`.

| Instruction | Does | Note |
|---|---|---|
| `adds r0, r1, r2` | add | |
| `subs r0, r1, #4` | subtract | |
| `negs r0, r1` | negate | `0 - r1` |
| `muls r0, r1` | multiply | 32×32, low 32 bits kept |
| `adcs` | add with carry | how 64-bit sums are built from 32-bit parts |
| `ands r0, r1` | bitwise AND | `x &= y` |
| `orrs r0, r1` | bitwise OR | `x \|= y` |
| `eors r0, r1` | bitwise XOR | `x ^= y` |
| `bics r0, r1` | **bit clear** — `r0 AND NOT r1` | `x &= ~y` — the mask-clearing idiom |

> **There is no divide instruction on Cortex-M0+.** `a / b` becomes a call to a
> library routine, `__aeabi_uidiv`, which is why it is dramatically slower than
> multiply. You can see it in the spike: `080000b4 <__udivsi3>` is linked in
> because one line of C divided. Cortex-M3 and above have `sdiv` and `udiv` in
> hardware.

**Why the `s` everywhere?** On the 16-bit Thumb encodings the M0+ uses, most
data-processing instructions *always* update the flags — there is no room in
the encoding for a choice. So the compiler writes `adds`, not `add`. On M3 and
above, the 32-bit encodings let you pick. Slide 16 has the rest of that
difference, generation by generation.

---

## Slide 6: Shifts — Why `lsls` Is 4% of All Code

| Instruction | Does |
|---|---|
| `lsls r0, r1, #3` | logical shift left — multiply by 8 |
| `lsrs r0, r1, #3` | logical shift right — unsigned divide by 8 |
| `asrs r0, r1, #3` | arithmetic shift right — **signed** divide, keeps the sign |
| `rors r0, r1` | rotate right |

Shifts are everywhere in embedded code for one reason: **bit fields**. Every
time you write

```c
GPIOA->MODER |= (2u << (pin * 2));
```

the `<<` is an `lsls`. Every peripheral configuration you write this term
compiles into shift-and-mask, which is why a shift instruction outranks
subtraction in real firmware.

Note `lsrs` versus `asrs`: shifting right is only a division if you pick the
one that matches your signedness. Getting it wrong turns a small negative
number into a very large positive one.

---

## Slide 7: Flags and Conditional Branches

**The general idea: how does a machine remember the result of a comparison?**
Two families, and you can tell them apart in one glance at any disassembly.

- **Condition-code machines** keep a few flag bits in a status register.
  A compare *sets* them, a later branch *reads* them, and the two instructions
  are coupled by invisible state. ARM, x86, AVR and the 68000 all work this way.
- **Compare-and-branch machines** have no flags register at all. The comparison
  and the branch are one instruction — RISC-V writes `blt a0, a1, label`
  directly, and MIPS does the same. Nothing is carried between instructions.

Flags buy density: one `cmp` can feed several branches. They cost a hidden
dependency that every instruction in between must preserve, which is awkward for
a deeply pipelined or out-of-order machine — the main reason the newer designs
dropped them.

**This chip is a condition-code machine.** Comparison does not produce a value.
It sets four bits in the status register, and the *next* instruction acts on
them.

```svg
<svg viewBox="0 0 560 190" role="img" aria-label="Compare sets flags, branch reads them">
  <defs><marker id="i3" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="30" y="34" width="150" height="34" rx="5"/>
  <text x="105" y="51" text-anchor="middle" class="mono">cmp r3, r2</text>
  <path class="wire" d="M180 51 H226" marker-end="url(#i3)"/>
  <rect class="hifill" x="230" y="26" width="140" height="50" rx="6"/>
  <text x="300" y="46" text-anchor="middle" class="mono">N Z C V</text>
  <text x="300" y="65" text-anchor="middle" class="lbl">flags, in APSR</text>
  <path class="wire" d="M370 51 H416" marker-end="url(#i3)"/>
  <rect class="box" x="420" y="34" width="120" height="34" rx="5"/>
  <text x="480" y="51" text-anchor="middle" class="mono">blt  label</text>
  <text x="300" y="110" text-anchor="middle" class="lbl">N negative · Z zero · C carry/borrow · V signed overflow</text>
  <text x="280" y="150" text-anchor="middle" class="lbl">cmp is a subtraction that throws the answer away and keeps the flags.</text>
  <text x="280" y="170" text-anchor="middle" class="lbl">tst is an AND that does the same.</text>
</svg>
```

| Branch | Taken when | From C |
|---|---|---|
| `beq` | equal (`Z`) | `if (a == b)` |
| `bne` | not equal | `if (a != b)` |
| `blt` / `bge` | signed less / greater-or-equal | `int` comparison |
| `bcc` / `bcs` | unsigned lower / higher-or-same | `unsigned` comparison |
| `bmi` / `bpl` | negative / positive | sign test |

> **`blt` and `bcc` are not interchangeable.** The compiler picks based on
> whether your variable is signed. A comparison that behaves oddly at the
> extremes of a range is very often a signedness bug, and the disassembly shows
> it immediately.

---

## Slide 8: Branches, Calls and Returns

| Instruction | Does | C |
|---|---|---|
| `b   label` | jump | `goto`, loop back, `else` |
| `bne label` | jump if condition | `if`, `while` |
| `bl  func` | **branch with link** — jump and save the return address in `LR` | `f();` |
| `bx  lr` | jump to the address in `LR` | `return;` |
| `push {r4, lr}` | save registers on the stack | function prologue |
| `pop  {r4, pc}` | restore — **and popping into `PC` returns** | function epilogue |

**The general idea: where does a call put the return address?** Again two
answers, and this one you will see in every function on the chip.

| | Where the return address goes | Machines |
|---|---|---|
| **Stack-based** | `call` pushes it to memory; `ret` pops it | x86, 68000, **AVR** (`rcall` / `ret`) |
| **Link register** | the call writes it to a register; return branches to that register | **ARM**, RISC-V (`ra`), MIPS (`$ra`), PowerPC |

A link register makes a **leaf** call — one that calls nothing else — touch
memory not at all: `bl` in, `bx lr` out, no stack traffic either way. The cost
is that `LR` holds exactly one return address, so a function that calls anything
must save it before it is overwritten. **That is precisely what `push {lr}` in
every non-leaf prologue is for**, and why leaf functions on this chip often have
no prologue at all.

That last table row is a neat trick worth recognising. Instead of `pop {r4, lr}`
followed by `bx lr`, the compiler pops the saved `LR` straight into `PC`, which
*is* a branch. One instruction saved on every function that touches the stack.

You can see both halves in the spike's `_write`:

```
 80002b8:  b5f0    push  {r4, r5, r6, r7, lr}
   ...
 80002c8:  bdf0    pop   {r4, r5, r6, r7, pc}
```

---

## Slide 9: How You Reach Memory — Addressing Modes

`ldr` and `str` need an address, and there are only a few ways to build one.
Each corresponds to a C construct you write constantly.

| Form | Example | In C | Encoding |
|---|---|---|---|
| register | `ldr r0, [r1]` | `*p` | 16-bit |
| **immediate offset** | `ldr r0, [r4, #28]` | `s->field` | 16-bit, small offsets |
| **register offset** | `ldrb r6, [r1, r3]` | `buf[i]` | 16-bit |
| SP-relative | `ldr r0, [sp, #8]` | a local variable | 16-bit |
| PC-relative | `ldr r2, [pc, #44]` | a big constant | 16-bit |
| scaled register | `ldr r0, [r1, r2, lsl #2]` | `arr[i]` for `int` | **32-bit, M3+ only** |
| pre-indexed | `ldr r0, [r1, #4]!` | `*++p` | **32-bit, M3+ only** |
| post-indexed | `ldr r0, [r1], #4` | `*p++` | **32-bit, M3+ only** |

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="Immediate offset reaches a struct field; register offset reaches an array element">
  <defs><marker id="i4" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="18" y="30" width="150" height="30" rx="4"/>
  <text x="93" y="45" text-anchor="middle" class="mono">[r4, #28]</text>
  <text x="93" y="80" text-anchor="middle" class="lbl">base + FIXED offset</text>
  <text x="93" y="100" text-anchor="middle" class="lbl">known at compile time</text>
  <text x="93" y="124" text-anchor="middle" class="mono lbl">USART2-&gt;ISR</text>
  <rect class="hifill" x="204" y="30" width="150" height="30" rx="4"/>
  <text x="279" y="45" text-anchor="middle" class="mono">[r1, r3]</text>
  <text x="279" y="80" text-anchor="middle" class="lbl">base + VARIABLE index</text>
  <text x="279" y="100" text-anchor="middle" class="lbl">computed at run time</text>
  <text x="279" y="124" text-anchor="middle" class="mono lbl">buf[i]</text>
  <rect class="box" x="390" y="30" width="152" height="30" rx="4"/>
  <text x="466" y="45" text-anchor="middle" class="mono">[sp, #8]</text>
  <text x="466" y="80" text-anchor="middle" class="lbl">base is the stack</text>
  <text x="466" y="100" text-anchor="middle" class="lbl">pointer</text>
  <text x="466" y="124" text-anchor="middle" class="mono lbl">a local</text>
  <text x="280" y="168" text-anchor="middle" class="lbl">A peripheral register access is literally a struct field access — base register</text>
  <text x="280" y="186" text-anchor="middle" class="lbl">plus constant offset. That is the whole trick the CMSIS headers use.</text>
</svg>
```

---

## Slide 10: The Offsets Are Small, and You Can See When They Run Out

A 16-bit instruction has very little room left for an offset once it has
encoded the opcode and two registers. The limits are strict:

| Instruction | Offset field | Reaches |
|---|---|---|
| `ldr  r0, [r1, #n]` | 5 bits, **scaled by 4** | 0 to 124 bytes |
| `ldrh r0, [r1, #n]` | 5 bits, scaled by 2 | 0 to 62 bytes |
| `ldrb r0, [r1, #n]` | 5 bits, unscaled | 0 to 31 bytes |
| `ldr  r0, [sp, #n]` | 8 bits, scaled by 4 | 0 to 1020 bytes |

Two things follow that you will actually observe:

**Offsets are always positive.** There is no `[r1, #-4]` in 16-bit Thumb. To
step backwards the compiler must adjust the base register first.

**A far field costs an extra instruction.** Reaching offset 28 is free:

```
 80002d0:  ldr   r6, [r4, #28]        ; USART2->ISR, one instruction
```

but a field beyond 124 bytes cannot be encoded, so the compiler emits an `add`
to move the base, then loads from the new base. **This is why a large
peripheral struct can generate visibly worse code at the far end of it**, and
why drivers that keep a pointer to the specific register they hammer in a loop
are sometimes measurably faster than ones that index from the block base.

> On Cortex-M3 and above the 32-bit encodings lift most of this: offsets up to
> 4095 bytes, negative offsets, scaled register offsets and auto-increment.
> Same source code, denser output — another instance of the "one model, many
> instances" idea from lesson 00.

---

## Slide 11: A Real Function, Disassembled

This is `SystemInit()` from the spike — the code that takes the chip from its
12 MHz reset state to 48 MHz. Here is what the processor got:

```
0800022c <SystemInit>:
 800022c:  2007        movs  r0, #7            ; r0 = the HSIDIV bit mask
 800022e:  2301        movs  r3, #1
 8000230:  4a0b        ldr   r2, [pc, #44]     ; r2 = 0x40022000  (FLASH base)
 8000232:  6811        ldr   r1, [r2, #0]      ; LOAD    FLASH->ACR
 8000234:  4381        bics  r1, r0            ; MODIFY  clear the field
 8000236:  430b        orrs  r3, r1            ; MODIFY  set the new value
 8000238:  6013        str   r3, [r2, #0]      ; STORE   back to FLASH->ACR
```

Four instructions — `ldr`, `bics`, `orrs`, `str` — are the load/modify/store of
slide 4, and they came from a single line of C:

```c
FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
```

One statement, four bus operations. `bics` is the `& ~`, `orrs` is the `|`.
That correspondence is the whole reason to read disassembly.

---

## Slide 12: Where Do Big Constants Live?

Look again at `ldr r2, [pc, #44]`. The CPU is loading from an address computed
from **the program counter itself**. Why?

```svg
<svg viewBox="0 0 560 210" role="img" aria-label="A 16-bit instruction cannot hold a 32-bit address, so it is stored nearby">
  <rect class="reg" x="20" y="30" width="200" height="38" rx="5"/>
  <text x="120" y="49" text-anchor="middle" class="mono">movs r0, #7</text>
  <text x="120" y="86" text-anchor="middle" class="lbl">small number: fits inside</text>
  <text x="120" y="104" text-anchor="middle" class="lbl">the instruction itself</text>
  <rect class="hifill" x="300" y="30" width="240" height="38" rx="5"/>
  <text x="420" y="49" text-anchor="middle" class="mono">0x40021000</text>
  <text x="420" y="86" text-anchor="middle" class="lbl">32 bits — cannot possibly fit in a</text>
  <text x="420" y="104" text-anchor="middle" class="lbl">16-bit instruction</text>
  <rect class="box" x="140" y="132" width="280" height="36" rx="5"/>
  <text x="280" y="150" text-anchor="middle" class="mono">ldr r2, [pc, #44]</text>
  <text x="280" y="192" text-anchor="middle" class="lbl">So the compiler parks the constant in a "literal pool" just past the
  code, and loads it relative to PC.</text>
</svg>
```

**This is a general consequence of fixed-width encoding, not an ARM quirk.** An
instruction that is itself only 16 or 32 bits wide cannot carry an arbitrary
32-bit constant inside it, so every fixed-width machine needs an answer:

| Machine | How a 32-bit constant reaches a register |
|---|---|
| x86 | the instruction simply grows — the constant is embedded, variable length |
| **ARM / Thumb** | **a literal pool near the code, loaded PC-relative** |
| RISC-V | built in two halves, `lui` then `addi` |
| AArch64 | built in halves too, `movz` then `movk` |

The pool sits at the end of the function, and every peripheral address the
function touches is listed in plain sight:

```
 8000260:  40022000   .word  0x40022000    ; FLASH
 8000264:  40021000   .word  0x40021000    ; RCC
 8000268:  ffffc7ff   .word  0xffffc7ff    ; ~(7 << 11)  — the HSIDIV mask
 800026c:  20000000   .word  0x20000000    ; &SystemCoreClock, in SRAM
 8000270:  02dc6c00   .word  0x02dc6c00    ; 48000000
```

`0x02dc6c00` is 48,000,000. The clock speed you assigned in C is sitting in
flash as a number, and the address it gets written to is `0x20000000` — the
very bottom of SRAM, exactly where lesson 02 will show the linker putting it.

---

## Slide 13: A Polling Loop, in Four Instructions

Lesson 00 described polling as "keep asking". Here it is literally, waiting for
the flash controller to acknowledge the new wait state:

```
 800023a:  6813        ldr   r3, [r2, #0]     ; read FLASH->ACR
 800023c:  4003        ands  r3, r0           ; isolate the LATENCY field
 800023e:  2b01        cmp   r3, #1           ; is it what we asked for?
 8000240:  d1fb        bne.n 800023a          ; no -> go back and read again
```

The branch target `800023a` is the load. That is the loop: **read, test,
branch back**. It costs the CPU every cycle until the hardware agrees.

Note there is no `volatile` keyword visible in the machine code — but without
it in the C source, the compiler would have read `FLASH->ACR` **once**, cached
it in `r3`, and looped forever on a value that never changes.

---

## Slide 14: A Whole C Function, Line by Line

This is `_write()` from the spike — the function that makes `printf` reach the
serial port. Eleven lines of C:

```c
static void uart2_putc(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) { }
    USART2->TDR = (uint8_t)c;
}

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') { uart2_putc('\r'); }
        uart2_putc(buf[i]);
    }
    return len;
}
```

And here is what the processor got. Nothing in it is exotic, and every
instruction is one you have now met:

```
080002b8 <_write>:
 80002b8:  b5f0    push  {r4, r5, r6, r7, lr}   ; prologue
 80002ba:  2300    movs  r3, #0                 ; i = 0
 80002bc:  2080    movs  r0, #128               ; 0x80 = the TXE flag
 80002be:  250d    movs  r5, #13                ; '\r'
 80002c0:  4c09    ldr   r4, [pc, #36]          ; r4 = 0x40004400 = USART2
 80002c2:  4293    cmp   r3, r2                 ; i < len ?
 80002c4:  db01    blt.n 80002ca                ;   yes -> body
 80002c6:  0010    movs  r0, r2                 ; return len
 80002c8:  bdf0    pop   {r4, r5, r6, r7, pc}   ; epilogue + return
 80002ca:  5cce    ldrb  r6, [r1, r3]           ; c = buf[i]
 80002cc:  2e0a    cmp   r6, #10                ; c == '\n' ?
 80002ce:  d103    bne.n 80002d8                ;   no -> skip
 80002d0:  69e6    ldr   r6, [r4, #28]          ; USART2->ISR
 80002d2:  4206    tst   r6, r0                 ; TXE set?
 80002d4:  d0fc    beq.n 80002d0                ;   no -> poll again
 80002d6:  62a5    str   r5, [r4, #40]          ; USART2->TDR = '\r'
 80002d8:  5cce    ldrb  r6, [r1, r3]           ; c = buf[i]
 80002da:  69e7    ldr   r7, [r4, #28]          ; poll again for the real character
 80002dc:  4207    tst   r7, r0
 80002de:  d0fc    beq.n 80002da
 80002e0:  62a6    str   r6, [r4, #40]          ; USART2->TDR = c
 80002e2:  3301    adds  r3, #1                 ; i++
 80002e4:  e7ed    b.n   80002c2                ; loop
```

Read the constants and the whole thing decodes itself:

| In the code | Is |
|---|---|
| `#10` | `'\n'` |
| `#13` | `'\r'` |
| `#128` | `USART_ISR_TXE`, bit 7 |
| `[r4, #28]` | `USART2->ISR` — offset `0x1C` |
| `[r4, #40]` | `USART2->TDR` — offset `0x28` |

---

## Slide 15: The Same Function, in One Picture

Twenty-two instructions, and only five things happening. Follow the arrows:

```svg
<svg viewBox="0 0 560 430" role="img" aria-label="Control flow of _write: setup once, the loop test, the character load, the carriage-return branch, the send, and the jump back to the test">
  <defs><marker id="i6" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>

  <rect class="box" x="56" y="16" width="240" height="46" rx="6"/>
  <text x="176" y="34" text-anchor="middle" class="mono">80002b8 - c0</text>
  <text x="176" y="52" text-anchor="middle" class="lbl">push, i = 0, masks, r4 = USART2</text>
  <text x="306" y="42" class="lbl">runs once</text>
  <path class="wire" d="M176 62 V76" marker-end="url(#i6)"/>

  <rect class="reg" x="56" y="80" width="240" height="34" rx="5"/>
  <text x="176" y="100" text-anchor="middle" class="mono">80002c2   i &lt; len ?</text>
  <path class="wire" d="M296 97 H330" marker-end="url(#i6)"/>
  <text x="313" y="88" text-anchor="middle" class="lbl">no</text>
  <rect class="box" x="336" y="80" width="184" height="34" rx="5"/>
  <text x="428" y="100" text-anchor="middle" class="mono">80002c6  return len</text>

  <path class="wire" d="M176 114 V132" marker-end="url(#i6)"/>
  <text x="168" y="128" text-anchor="end" class="lbl">yes</text>
  <rect class="box" x="56" y="138" width="240" height="34" rx="5"/>
  <text x="176" y="158" text-anchor="middle" class="mono">80002ca   c = buf[i]</text>
  <path class="wire" d="M176 172 V190" marker-end="url(#i6)"/>

  <rect class="reg" x="56" y="196" width="240" height="34" rx="5"/>
  <text x="176" y="216" text-anchor="middle" class="mono">80002cc   c == '\n' ?</text>
  <path class="wire" d="M296 213 H330" marker-end="url(#i6)"/>
  <text x="313" y="204" text-anchor="middle" class="lbl">yes</text>

  <rect class="hifill" x="336" y="190" width="184" height="52" rx="6"/>
  <text x="428" y="210" text-anchor="middle" class="mono">80002d0 - d6</text>
  <text x="428" y="228" text-anchor="middle" class="lbl">wait for TXE, send '\r'</text>
  <path class="hi" d="M520 202 C544 202 544 230 522 230" marker-end="url(#i6)"/>
  <text x="538" y="248" text-anchor="middle" class="lbl">poll</text>

  <path class="wire" d="M176 230 V276" marker-end="url(#i6)"/>
  <text x="168" y="256" text-anchor="end" class="lbl">no</text>
  <path class="wire" d="M428 242 V288 H304" marker-end="url(#i6)"/>

  <rect class="hifill" x="56" y="282" width="240" height="52" rx="6"/>
  <text x="176" y="302" text-anchor="middle" class="mono">80002d8 - e0</text>
  <text x="176" y="320" text-anchor="middle" class="lbl">reload c, wait for TXE, send c</text>
  <path class="hi" d="M296 296 C320 296 320 324 298 324" marker-end="url(#i6)"/>
  <text x="314" y="342" text-anchor="middle" class="lbl">poll</text>

  <path class="wire" d="M176 334 V352" marker-end="url(#i6)"/>
  <rect class="box" x="56" y="358" width="240" height="32" rx="5"/>
  <text x="176" y="377" text-anchor="middle" class="mono">80002e2   i++</text>

  <path class="wire" d="M56 374 H26 V97 H50" marker-end="url(#i6)"/>
  <text x="18" y="240" text-anchor="middle" class="lbl mono" transform="rotate(-90 18 240)">80002e4   b</text>

  <text x="280" y="414" text-anchor="middle" class="lbl">Two send blocks, one source line each: uart2_putc was inlined, so there is no bl here at all.</text>
</svg>
```

Every address in the listing belongs to one line of the C:

| C | Became |
|---|---|
| `int _write(...)` | `80002b8` — `push` |
| `(void)fd;` | **nothing** — `r0` arrived holding `fd` and is reused at `80002bc` |
| `int i = 0`, and the hoisted constants | `80002ba` – `80002c0` |
| `i < len` | `80002c2` – `80002c4` |
| `buf[i] == '\n'` | `80002ca` – `80002ce` |
| `uart2_putc('\r')` | `80002d0` – `80002d6` |
| `uart2_putc(buf[i])` | `80002d8` – `80002e0` |
| `i++`, and the jump back | `80002e2` – `80002e4` |
| `return len` | `80002c6` – `80002c8` — **in the middle** |

Three things the compiler did that the source does not show:

- **Hoisting.** `#128`, `#13` and the USART2 base are set up once, before the
  loop, although the C mentions them inside it. Anything that cannot change is
  lifted out.
- **Inlining, twice.** `uart2_putc` is a function in the source and is nowhere
  in the machine code — its three instructions appear once for `'\r'` and again
  for the character. `_write` contains no `bl` at all.
- **Rotation.** The `for` test sits at the top, so the exit is the *fall
  through*, and `return len` lands in the middle of the address range at
  `0x080002c6`. **Address order is not source order**, which is why a debugger
  stepping through optimised code appears to jump around.

> One more, worth chasing: `buf[i]` is loaded **twice**, at `80002ca` and again
> at `80002d8`. The compiler may not keep it in a register across the store to
> `USART2->TDR`, because a `char` object is allowed to alias anything, so as
> far as the standard is concerned that store might have changed `buf[i]`.
> Slide 21 is the same rule seen from the other side.

**Every `for` loop you ever write looks like this**: initialise, compare,
conditional branch to the body, increment, unconditional branch back.

---

## Slide 16: Thumb-1 and Thumb-2 — Two Generations

Look at the second column of the disassembly: `2007`, `4a0b`, `6811`. Four hex
digits. Two bytes. Almost every instruction on this chip is **16 bits wide**,
which is why 32 KB of flash holds a useful program at all.

Thumb was not always able to do anything else:

- **Thumb-1** (1994) — 16-bit encodings, and nothing else. Compact, and
  correspondingly restricted.
- **Thumb-2** (2003, ARMv6T2) — keeps every 16-bit encoding and adds 32-bit
  ones **beside** them, in the same instruction stream. There is no mode
  switch: the decoder reads a halfword, and if its top five bits are `11101`,
  `11110` or `11111`, a second halfword belongs to it.

So "Thumb" on its own does not tell you what a core can do. The generation
does:

| | **Cortex-M0 / M0+** | **Cortex-M3** | **Cortex-M4 / M7** |
|---|---|---|---|
| Architecture | ARMv6-M | ARMv7-M | ARMv7E-M |
| Instruction set | Thumb-1, plus six | full Thumb-2 | full Thumb-2 + DSP |
| Divide | **none** — calls `__aeabi_uidiv` | `sdiv` / `udiv` | `sdiv` / `udiv` |
| Flags | most 16-bit forms always set them | `add` **or** `adds`, your choice | same |
| Conditional execution | branches only | `IT` blocks, up to 4 instructions | same |
| `ldr` offset | 0–124 bytes, positive only | ±4095, pre- and post-indexed | same |
| Registers | R0–R7 in most 16-bit forms | 32-bit forms reach all 16 | same |
| Immediates | 8 bits, 0–255 | 12-bit modified constants, `movw` / `movt` | same |
| Floating point | none | none | M4F and M7: hardware FPU |

Every core in that table is Thumb-only — not one of them can execute an ARM
instruction. The only question is *which* Thumb.

---

## Slide 17: Which Thumb This Is — "Thumb-1, Plus Six"

The M0+ column says *Thumb-1, plus six*, and the six are worth memorising,
because they are the **only** 32-bit instructions this chip has:

| | |
|---|---|
| `bl` | call a function anywhere in the address space |
| `mrs` / `msr` | read and write the special registers (`PRIMASK`, `CONTROL`, `PSP`) |
| `dsb` / `dmb` / `isb` | memory and instruction barriers |

Everything else an M0+ executes is two bytes. Here is one of the six, decoded
out of the real image — the call in `Reset_Handler` that runs the clock setup
from slide 11:

```
 800028a:  f7ff ffcf   bl  800022c <SystemInit>
```

Four bytes, because a 16-bit branch reaches about ±2 KB and a call has to
reach anywhere. **That is the whole rule for when Thumb widens**: not when 32
bits would be convenient, but when 16 bits cannot do the job at all.

**`.n` and `.w`.** objdump has been writing `bne.n`, `blt.n` and `b.n`
throughout this deck. `.n` is *narrow*, 16 bits; `.w` is *wide*, 32. On this
chip there is nothing to choose and the suffix is only objdump being explicit.
On M3 and above the assembler picks the smallest encoding that reaches, and
`.w` forces the wide one — which is how you write a branch that crosses a
large flash.

> **Now re-read the last dozen slides.** The `s` on every `adds`, the missing
> divide, the 5-bit positive-only offsets, the eight reachable registers, the
> literal pool full of addresses — **none of those are ARM. They are
> Thumb-1.** Compile the same C for a Cortex-M4 and you get `add` that leaves
> the flags alone, `sdiv` instead of a library call, `ldr r0, [r1, #2000]` in
> one instruction, and far fewer literal pools, because `movw` / `movt` build a
> 32-bit constant inline. Same source, same instruction set family — a
> different instance of the model from lesson 00.

---

## Slide 18: Who Owns Which Register — the Calling Convention

**The general idea: an ABI.** Sixteen registers, one CPU, and every function
wanting all of them. Some agreement has to say where arguments go, where the
result comes back, and which registers a function may destroy. That agreement is
a **calling convention**, part of the **Application Binary Interface**, and
every platform has one: SysV AMD64 on Linux, the RISC-V and MIPS conventions,
ARM's **AAPCS** here.

The universal shape of it is the same everywhere — registers are split into

- **caller-saved** (*volatile*, *scratch*): a function may destroy them, so the
  caller saves anything it still wants;
- **callee-saved** (*non-volatile*): a function must give them back untouched,
  so it saves them on entry and restores them on exit.

It exists because of separate compilation: your C has to call a library compiled
years ago by a different compiler, possibly written in assembly, and neither
side can see the other's source. **This chip's answer is AAPCS**, and the
compiler follows it exactly.

```svg
<svg viewBox="0 0 560 210" role="img" aria-label="AAPCS register roles: arguments, callee-saved, special">
  <rect class="hifill" x="16" y="26" width="150" height="44" rx="5"/>
  <text x="91" y="42" text-anchor="middle" class="mono">R0 R1 R2 R3</text>
  <text x="91" y="60" text-anchor="middle" class="lbl">arguments in, result out</text>
  <rect class="box" x="182" y="26" width="150" height="44" rx="5"/>
  <text x="257" y="42" text-anchor="middle" class="mono">R4 ... R11</text>
  <text x="257" y="60" text-anchor="middle" class="lbl">callee must preserve</text>
  <rect class="box" x="348" y="26" width="196" height="44" rx="5"/>
  <text x="446" y="42" text-anchor="middle" class="mono">R12  SP  LR  PC</text>
  <text x="446" y="60" text-anchor="middle" class="lbl">scratch and special</text>
  <text x="280" y="106" text-anchor="middle" class="lbl">A function returning its answer in R0 is not a convention the compiler invented.</text>
  <text x="280" y="124" text-anchor="middle" class="lbl">It is why C you compile can call assembly someone else wrote.</text>
  <rect class="reg" x="140" y="148" width="280" height="34" rx="5"/>
  <text x="280" y="165" text-anchor="middle" class="mono">push {r4, r5, r6, r7, lr}</text>
</svg>
```

Now `_write` explains itself. Its arguments arrived in `r0`, `r1`, `r2` — the
file descriptor, the buffer and the length. It returns `len` by putting it in
`r0`. And it pushes `r4`–`r7` **because AAPCS says it must give them back
unchanged** — it wanted them for the USART base and the loop counter.

---

## Slide 19: The Thumb Bit — an Odd Address That Is Not a Bug

Here are the first sixteen bytes of `Main.hex`, the actual file that would be
flashed:

```
:1000000000300020750200082902000829020008BB
```

Decoded, ignoring the record framing:

| Bytes (little-endian) | Value | Meaning |
|---|---|---|
| `00300020` | `0x20003000` | initial stack pointer — top of 12 KB SRAM |
| `75020008` | `0x08000275` | reset vector — **where execution begins** |
| `29020008` | `0x08000229` | NMI handler |

`Reset_Handler` is at `0x08000274`, an even address. The vector says
`0x08000275`. **The low bit is set on purpose**: on this processor it means
"the code at this address is Thumb code". Every function pointer on the chip
is odd.

The bit exists because on a Cortex-A it is a genuine question — that core
has both instruction sets and this is how a branch says which one it is
landing in. On a Cortex-M, per slide 2, the answer is always Thumb, so the
bit is always 1 and the hardware still insists on being told.

> Clear that bit and the CPU faults immediately on reset. A vector table full
> of even addresses is a dead chip — and it is a common hand-written-assembly
> mistake.

---

## Slide 20: `LR` Is Odd Too — Reading a Return Address

Stop the debugger anywhere inside a function and `LR` ends in an odd digit.
It is the same bit as the vector table's, and it is the first thing in an ARM
debugging session that looks like an off-by-one.

Here is a real call, and the instruction it must come back to:

```
 800028a:  f7ff ffcf   bl  800022c <SystemInit>
 800028e:  f000 fb31   bl  80008f4
```

`bl` is four bytes wide, so the return address is `0x0800028e`. What the
processor puts in `LR` is not that:

| | |
|---|---|
| address of the instruction after the `bl` | `0x0800028e` |
| Thumb bit, set by `bl` | `\| 1` |
| **`LR` becomes** | **`0x0800028f`** |

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="bl stores the return address with bit 0 set; returning clears it again">
  <defs><marker id="i5" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="12" y="44" width="140" height="34" rx="5"/>
  <text x="82" y="61" text-anchor="middle" class="mono">bl 800022c</text>
  <path class="hi" d="M152 61 H206" marker-end="url(#i5)"/>
  <text x="179" y="36" text-anchor="middle" class="lbl">next | 1</text>
  <rect class="hifill" x="210" y="36" width="150" height="50" rx="6"/>
  <text x="285" y="56" text-anchor="middle" class="mono">LR = 0800028f</text>
  <text x="285" y="75" text-anchor="middle" class="lbl">odd — on purpose</text>
  <path class="hi" d="M360 61 H414" marker-end="url(#i5)"/>
  <text x="387" y="36" text-anchor="middle" class="lbl">&amp; ~1</text>
  <rect class="box" x="418" y="44" width="130" height="34" rx="5"/>
  <text x="483" y="61" text-anchor="middle" class="mono">PC = 0800028e</text>
  <text x="285" y="112" text-anchor="middle" class="lbl">bx lr · pop {pc} · both take bit 0 as "which instruction set", then drop it</text>
  <text x="280" y="152" text-anchor="middle" class="lbl">The odd value was never an address. It is an address plus a one-bit answer</text>
  <text x="280" y="172" text-anchor="middle" class="lbl">to a question this processor only has one answer to.</text>
</svg>
```

Returning undoes it. `bx lr` — and the `pop {r4, r5, r6, r7, pc}` from slide 8,
which is the same branch in disguise — read bit 0 as *the instruction set to
resume in*, then branch to the address with that bit **cleared**:

```
PC  <-  LR & ~1  =  0x0800028e
```

So execution continues at `0x0800028e`, the instruction immediately after the
call. **Nothing is skipped and nothing is off by one.** One bit is doing the
job of a processor mode flag, which is why it rides along inside the address
rather than living in a register of its own — and it is why `pop {pc}` can
return at all: the Thumb bit was saved with `LR` and comes back with it.

> **When it bites.** The bit is only ever 1 on a Cortex-M, so it is a trap
> rather than a feature. Branch to an address with bit 0 **clear** and the core
> faults — it has been told to execute ARM instructions, and it has none. A
> `HardFault` whose stacked `PC` or `LR` looks like a perfectly sensible *even*
> address usually means a function pointer that was computed rather than
> assigned, or hand-written assembly that returned with `mov pc, lr` instead of
> `bx lr`.

The one place `LR` is not a return address at all is inside an exception
handler: the core loads a magic `EXC_RETURN` value there instead
(`0xFFFFFFF9`, `0xFFFFFFFD`), and branching to it is what unstacks the frame
lesson 03 draws.

---

## Slide 21: What the Compiler Is Allowed to Do to You

The compiler's contract is to preserve the *observable behaviour* of your
program — as defined by the C standard, which has never heard of your
peripheral.

```svg
<svg viewBox="0 0 560 180" role="img" aria-label="The compiler may cache or delete accesses unless told otherwise">
  <rect class="box" x="20" y="26" width="180" height="106" rx="6"/>
  <text x="110" y="48" text-anchor="middle" class="lbl">you wrote</text>
  <text x="110" y="74" text-anchor="middle" class="mono">read</text>
  <text x="110" y="94" text-anchor="middle" class="mono">read</text>
  <text x="110" y="114" text-anchor="middle" class="mono">read</text>
  <rect class="hifill" x="360" y="26" width="180" height="106" rx="6"/>
  <text x="450" y="48" text-anchor="middle" class="lbl">you may get</text>
  <text x="450" y="84" text-anchor="middle" class="mono">read</text>
  <text x="450" y="108" text-anchor="middle" class="lbl">(cached in a register)</text>
  <path class="dash" d="M200 79 H356"/>
  <text x="278" y="66" text-anchor="middle" class="lbl">-Os</text>
</svg>
```

For ordinary variables this is exactly what you want. For a status register it
is catastrophic, and it produces the classic symptom: **code that works at
`-O0` and hangs at `-Os`**.

The keyword that forbids it is `volatile`, and it has already been applied for
you by **CMSIS** — the *Common Microcontroller Software Interface Standard*,
ARM's vendor-neutral set of headers for Cortex-M. Every register in
`stm32c031xx.h` is declared `__IO`, which is simply `volatile`. You inherit the
protection; you should still know why it is there.

---

## Slide 22: What You Should Be Able to Say

1. Roughly how many distinct instructions make up 80% of real firmware?
2. Why does setting one bit in a peripheral register take at least three
   instructions?
3. What does `bics` do, and which C operator produces it?
4. Why is `lsls` more common than `subs` in embedded code?
5. `cmp` computes a subtraction and discards the answer. What is the point?
6. `blt` and `bcc` both mean "less than". What is the difference, and what bug
   does confusing them cause?
7. What does `pop {r4, pc}` do that `pop {r4, lr}` does not?
8. Which addressing mode is `s->field`? Which is `buf[i]`?
9. Why is there no divide instruction, and how can you tell from a `.map` file
   that your code divided?
10. ARM designs two instruction sets. What is the other one called, and under
    what circumstances does a Cortex-M switch into it?
11. Name three things a Cortex-M4 does in one instruction that this M0+ cannot.
12. The reset vector holds `0x08000275` but the function is at `0x08000274`.
    Why, and what happens if you "fix" it?
13. You break inside a function and `LR` reads `0x0800028f`. Which instruction
    will run when it returns, and what would an **even** `LR` tell you?
14. Name the two instruction-set models. Which one is this, which one is x86,
    and which one was the AVR?
15. RISC-V has no flags register. How does it write `if (a < b) goto L`, and
    what does ARM gain by having one?
16. Two ways exist to remember a return address. Name both, say which this chip
    uses, and explain what `push {lr}` is really for.
17. Why can a 16-bit instruction not load `0x40021000`? Give ARM's answer and
    one other machine's.

**Next:** *Development* — how a `.c` file becomes the bytes in that hex record,
and what runs before `main()`.
