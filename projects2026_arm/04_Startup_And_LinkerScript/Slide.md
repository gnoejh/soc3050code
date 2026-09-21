# Startup and the Linker Script: Building the Ground You Stand On
## SOC3050 ARM Edition — Part 1, Instances

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](https://www.st.com/resource/en/user_manual/um2953-stm32c0-nucleo64-board-mb1717-stmicroelectronics.pdf)

**The first lesson with code you build and run.**

Part 0 gave you four models: the machine, its instructions, its toolchain, and
concurrency. Lesson 02 told you that something has to build the C environment
before `main()` runs, and that on a bare-metal part there is no operating
system to do it.

This week you write that something, and then you watch it work.

Every figure on these slides is real output from this lesson's own build —
`04_Startup_And_LinkerScript/Main.elf`, on the Nucleo-C031C6. Nothing here is
illustrative.

---

## Slide 1: `main()` Is Not Where Your Program Starts

Ask a first-year student where a C program begins and they say `main()`. They
are wrong everywhere, but on a desktop it never matters, because the operating
system quietly does four jobs on the way in.

```svg
<svg viewBox="0 0 580 210" role="img" aria-label="On a PC the operating system prepares the C environment; on bare metal nobody does">
  <text x="145" y="18" text-anchor="middle" class="lbl">ON A LAPTOP</text>
  <rect class="box" x="20" y="30" width="250" height="118" rx="7"/>
  <text x="145" y="52" text-anchor="middle">the operating system</text>
  <text x="145" y="74" text-anchor="middle" class="lbl">finds RAM for you</text>
  <text x="145" y="92" text-anchor="middle" class="lbl">gives you a stack</text>
  <text x="145" y="110" text-anchor="middle" class="lbl">zeroes your globals</text>
  <text x="145" y="128" text-anchor="middle" class="lbl">opens stdout, calls main()</text>
  <text x="145" y="172" text-anchor="middle" class="lbl">you never see any of it</text>

  <text x="435" y="18" text-anchor="middle" class="lbl">ON THIS CHIP</text>
  <rect class="hifill" x="310" y="30" width="250" height="118" rx="7"/>
  <text x="435" y="62" text-anchor="middle">nobody</text>
  <text x="435" y="92" text-anchor="middle" class="lbl">the four jobs still exist</text>
  <text x="435" y="112" text-anchor="middle" class="lbl">and nothing does them</text>
  <text x="435" y="172" text-anchor="middle" class="hi">unless you write it</text>
</svg>
```

> The question this lesson answers: **what are those four jobs, who does them
> here, and what exactly breaks when one of them is missing?**

You will find out by removing them, one at a time, and watching the board.

---

## Slide 2: The Two Files That Do It

Every program in the rest of this course rests on two files. They are small —
about 140 and 70 lines — and this week they are the whole subject.

| File | Answers |
|---|---|
| `link.ld` | **Where** does everything go? Which addresses are FLASH, which are RAM, and which piece of your program lands in each? |
| `startup.c` | **What runs first?** The vector table, and the code that makes RAM match what C promises before `main()` is called. |

The linker script decides the *layout*. The startup code makes the *hardware
match that layout*. Neither one is optional, and neither one is magic — you
will read every line.

But before either file makes sense, you need the addresses they talk about.

---

## Slide 3: The Memory Map, With This Chip's Real Numbers

Lesson 00 showed the Cortex-M regions in the abstract. Here they are with the
STM32C031C6's actual boundaries — **every number below is checked against ST's
own CMSIS header**, not copied from a tutorial.

```svg
<svg viewBox="0 0 580 396" role="img" aria-label="STM32C031C6 memory map with real addresses and sizes">
  <text x="300" y="16" text-anchor="middle" class="lbl">one flat 4 GB address space — 0x00000000 to 0xFFFFFFFF</text>

  <rect class="box" x="150" y="28" width="250" height="40" rx="5"/>
  <text x="275" y="45" text-anchor="middle" class="mono">SCS — NVIC, SysTick</text>
  <text x="275" y="61" text-anchor="middle" class="lbl">ARM's, identical on every Cortex-M</text>
  <text x="143" y="44" text-anchor="end" class="mono lbl">0xE000E000</text>
  <text x="410" y="52" class="lbl">ARM defines</text>

  <rect class="hifill" x="150" y="86" width="250" height="52" rx="5"/>
  <text x="275" y="104" text-anchor="middle" class="mono">IOPORT — GPIOA..GPIOD</text>
  <text x="275" y="120" text-anchor="middle" class="lbl">GPIOA = 0x50000000</text>
  <text x="275" y="134" text-anchor="middle" class="hi">not in the peripheral region!</text>
  <text x="143" y="106" text-anchor="end" class="mono lbl">0x50000000</text>

  <rect class="box" x="150" y="156" width="250" height="58" rx="5"/>
  <text x="275" y="174" text-anchor="middle" class="mono">PERIPHERALS</text>
  <text x="275" y="190" text-anchor="middle" class="lbl">RCC = 0x40021000 (AHB)</text>
  <text x="275" y="206" text-anchor="middle" class="lbl">USART2 = 0x40004400 (APB)</text>
  <text x="143" y="176" text-anchor="end" class="mono lbl">0x40000000</text>
  <text x="410" y="188" class="lbl">ST fills in</text>

  <rect class="hifill" x="150" y="232" width="250" height="52" rx="5"/>
  <text x="275" y="250" text-anchor="middle" class="mono">SRAM — 12 KB</text>
  <text x="275" y="266" text-anchor="middle" class="lbl">.data · .bss · heap · stack</text>
  <text x="275" y="280" text-anchor="middle" class="lbl">0x20000000 .. 0x20002FFF</text>
  <text x="143" y="252" text-anchor="end" class="mono lbl">0x20000000</text>

  <rect class="hifill" x="150" y="302" width="250" height="52" rx="5"/>
  <text x="275" y="320" text-anchor="middle" class="mono">FLASH — 32 KB</text>
  <text x="275" y="336" text-anchor="middle" class="lbl">.isr_vector · .text · .rodata</text>
  <text x="275" y="350" text-anchor="middle" class="lbl">0x08000000 .. 0x08007FFF</text>
  <text x="143" y="322" text-anchor="end" class="mono lbl">0x08000000</text>
  <text x="143" y="368" text-anchor="end" class="mono lbl">0x00000000</text>
  <text x="410" y="330" class="lbl">aliased at 0 on reset</text>

  <text x="300" y="388" text-anchor="middle" class="lbl">The two highlighted regions are exactly what link.ld's MEMORY block declares.</text>
</svg>
```

Three things to take from this:

- **You have 32 KB and 12 KB inside a 4 GB space.** Almost all of it is
  nothing at all. Addressability is not memory — reading an unmapped address
  is a fault, not a zero.
- **`link.ld`'s `MEMORY` block is two rows of this picture**, and nothing else.
  When you write `ORIGIN = 0x08000000, LENGTH = 32K`, you are copying the
  bottom box.
- **GPIO is at `0x50000000`, not `0x40000000`.** On the STM32C0 the I/O ports
  sit on their own port, outside the peripheral region — faster to reach, and
  a genuine difference from the F1 and F4 families. Code copied from an F4
  tutorial will not have the right address, and the chip will not tell you.

---

## Slide 4: The I/O Map — `GPIOA->MODER` Is an Address

Lesson 00 said *a register is an address with a side effect*. This is where you
see the address. There is no I/O instruction on this machine and no I/O address
space: **peripherals are reached by ordinary loads and stores**.

The names come from **CMSIS**:
**the Common Microcontroller Software Interface Standard**,
ARM's vendor-neutral header set for Cortex-M. This course uses two pieces of
it — the *core* headers from ARM (`core_cm0plus.h`: the NVIC and SysTick) and
the *device* header from ST (`stm32c031xx.h`: every peripheral on this part).
Both are vendored in `tools/cmsis/`.

**CMSIS is headers, not a library.** Nothing is linked and no function is
called. It is `#define`s and struct declarations that put names on addresses ST
published in the reference manual. It is *not* ST's HAL — that is a real
library, and lesson 34 compares the two.

Naming the address costs you nothing. Compile both spellings of the same
toggle at `-Os` and compare:

```
GPIOA->ODR ^= (1u<<5);          *(volatile uint32_t *)0x50000014UL ^= (1u<<5);

movs r2, #160                   movs r3, #32
movs r3, #32                    ldr  r2, [pc, #8]   <- literal pool
lsls r2, r2, #23   = 0x50000000 ldr  r1, [r2, #0]
ldr  r1, [r2, #20]              eors r3, r1
eors r3, r1                     str  r3, [r2, #0]
str  r3, [r2, #20]              bx   lr
bx   lr                         .word 0x50000014    <- 4 bytes of data

14 bytes, no literal pool       12 bytes + 4 = 16 bytes
```

The named version is **smaller**. It keeps the peripheral *base* in a register
and reaches each register by constant offset — exactly the addressing mode
lesson 01 showed — so one base serves `MODER`, `ODR`, `BSRR` and the rest,
while the raw-address version needs a fresh literal for every one.

```svg
<svg viewBox="0 0 580 210" role="img" aria-label="How GPIOA-MODER resolves to the address 0x50000000">
  <defs><marker id="m1" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>

  <text x="290" y="18" text-anchor="middle" class="lbl">what you write</text>
  <rect class="box" x="180" y="26" width="220" height="32" rx="5"/>
  <text x="290" y="47" text-anchor="middle" class="mono">GPIOA-&gt;ODR ^= (1 &lt;&lt; 5);</text>

  <path class="wire" d="M290 58 V80" marker-end="url(#m1)"/>

  <rect class="reg" x="60" y="86" width="200" height="32" rx="4"/>
  <text x="160" y="107" text-anchor="middle" class="mono">GPIOA = 0x50000000</text>
  <text x="160" y="132" text-anchor="middle" class="lbl">the base — a #define</text>

  <text x="290" y="107" text-anchor="middle" class="mono">+</text>

  <rect class="reg" x="320" y="86" width="200" height="32" rx="4"/>
  <text x="420" y="107" text-anchor="middle" class="mono">ODR at offset 0x14</text>
  <text x="420" y="132" text-anchor="middle" class="lbl">a struct member</text>

  <path class="wire" d="M160 140 V158 H290" marker-end="url(#m1)"/>
  <path class="wire" d="M420 140 V158 H290"/>

  <rect class="hifill" x="170" y="164" width="240" height="34" rx="5"/>
  <text x="290" y="186" text-anchor="middle" class="mono">read-modify-write 0x50000014</text>
</svg>
```

CMSIS declares `GPIOA` as a pointer to a struct pinned at a fixed address, so
`->` is nothing but **base + offset**, computed at compile time. Every register
this week's program touches, with the address the compiler actually emits:

| In `Main.c` / `retarget.c` | Base | Offset | Address |
|---|---|---|---|
| `RCC->CR` (clock, `HSIDIV`) | `0x40021000` | `0x00` | `0x40021000` |
| `RCC->IOPENR` (GPIOA clock gate) | `0x40021000` | `0x34` | `0x40021034` |
| `RCC->APBENR1` (USART2 clock gate) | `0x40021000` | `0x3C` | `0x4002103C` |
| `GPIOA->MODER` (pin direction) | `0x50000000` | `0x00` | `0x50000000` |
| `GPIOA->ODR` (LED on PA5) | `0x50000000` | `0x14` | `0x50000014` |
| `GPIOA->AFR[0]` (PA2/PA3 to USART2) | `0x50000000` | `0x20` | `0x50000020` |
| `USART2->BRR` (baud rate) | `0x40004400` | `0x0C` | `0x4000440C` |
| `USART2->ISR` (transmit ready?) | `0x40004400` | `0x1C` | `0x4000441C` |
| `USART2->TDR` (the byte out) | `0x40004400` | `0x28` | `0x40004428` |

> Every row was verified by compiling `_Static_assert((uintptr_t)&GPIOA->ODR
> == 0x50000014UL, ...)` and friends against ST's header. If a number here were
> wrong, this lesson would not compile.

**Two habits this should start.** First, when a peripheral does not respond,
ask *which address did that actually write to* — it is always answerable.
Second, notice that `RCC->IOPENR` appears in this table at all: a peripheral
with no clock reads back zero and ignores writes, silently. `led_init()` sets
that gate first, and every peripheral lesson after this one will too.

---

## Slide 5: Reset, in Hardware, Before Any Code

On AVR, reset jumps to address 0 and the bootstrap has to be assembly, because
the stack pointer is not set up yet and C cannot run without a stack.

**Cortex-M does it differently, and better.** The hardware reads the first two
words of memory *itself*, before fetching a single instruction.

```svg
<svg viewBox="0 0 580 235" role="img" aria-label="On reset the Cortex-M core loads SP from word 0 and PC from word 1">
  <defs><marker id="a4" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>

  <rect class="box" x="18" y="24" width="120" height="44" rx="6"/>
  <text x="78" y="44" text-anchor="middle">RESET</text>
  <text x="78" y="60" text-anchor="middle" class="lbl">power on / NRST</text>

  <text x="300" y="18" text-anchor="middle" class="lbl">0x08000000 — the base of FLASH</text>
  <rect class="hifill" x="200" y="26" width="200" height="34" rx="4"/>
  <text x="300" y="48" text-anchor="middle" class="mono">word 0 = 0x20003000</text>
  <rect class="hifill" x="200" y="66" width="200" height="34" rx="4"/>
  <text x="300" y="88" text-anchor="middle" class="mono">word 1 = 0x080006C5</text>

  <path class="wire" d="M138 46 H196" marker-end="url(#a4)"/>

  <rect class="reg" x="452" y="26" width="108" height="34" rx="4"/>
  <text x="506" y="48" text-anchor="middle" class="mono">SP</text>
  <rect class="reg" x="452" y="66" width="108" height="34" rx="4"/>
  <text x="506" y="88" text-anchor="middle" class="mono">PC</text>
  <path class="wire" d="M400 43 H448" marker-end="url(#a4)"/>
  <path class="wire" d="M400 83 H448" marker-end="url(#a4)"/>

  <text x="300" y="132" text-anchor="middle" class="lbl">the CORE does this — no instruction has executed yet</text>
  <line class="dash" x1="30" y1="150" x2="550" y2="150"/>
  <text x="300" y="176" text-anchor="middle">so the very first instruction runs <tspan class="hi">with a valid stack</tspan></text>
  <text x="300" y="200" text-anchor="middle" class="lbl">which is why Reset_Handler can be written in C, not assembly</text>
  <text x="300" y="222" text-anchor="middle" class="lbl">the AVR edition's startup could not be</text>
</svg>
```

Those two words are not illustrations. They are the **first eight bytes of the
`Main.hex` you will build this week**, and slide 7 decodes them.

---

## Slide 6: The Vector Table Is Just an Array at a Known Address

The table the hardware reads is, in C, an array of function pointers that the
linker has been told to place at the very start of FLASH.

```c
__attribute__((section(".isr_vector"), used))
void (* const vectors[])(void) = {
    (void (*)(void))&_estack,       /*  0  initial stack pointer  */
    Reset_Handler,                  /*  1  reset                  */
    NMI_Handler,                    /*  2                         */
    HardFault_Handler,              /*  3                         */
    0, 0, 0, 0, 0, 0, 0,            /*  4-10 reserved             */
    SVC_Handler,                    /* 11                         */
    0, 0,
    PendSV_Handler,                 /* 14                         */
    SysTick_Handler,                /* 15                         */
    /* then the peripheral interrupts, IRQ0 upward ... */
```

Three things are doing work here, and all three are load-bearing:

- **`section(".isr_vector")`** puts the array in a section of its own, so the
  linker script can pin it to `0x08000000` instead of letting it land wherever.
- **`used`** stops the compiler deleting an array that no C code ever reads.
  Nothing in your program mentions `vectors`. Only the hardware does.
- **`const`** puts it in FLASH rather than RAM. Drop it and you spend 180 bytes
  of your 12 KB on a table that never changes.

Entry 0 is **not a function pointer**. It is the initial stack pointer, wearing
a cast so the whole table can be one array.

---

## Slide 7: The First Eight Bytes of Your Own Firmware

This is record 2 of the `Main.hex` this lesson builds, byte for byte:

```
:10000000 00300020C50600087906000879060008 BF
           ^^^^^^^^ ^^^^^^^^ ^^^^^^^^ ^^^^^^^^
           word 0   word 1   word 2   word 3
```

ARM is **little-endian**, so each group of four bytes reads right to left:

| Bytes | Value | Meaning |
|---|---|---|
| `00 30 00 20` | `0x20003000` | initial SP — top of RAM |
| `C5 06 00 08` | `0x080006C5` | `Reset_Handler` |
| `79 06 00 08` | `0x08000679` | `NMI_Handler` |
| `79 06 00 08` | `0x08000679` | `HardFault_Handler` |

Three observations, each worth more than a paragraph of theory:

1. **`0x20003000` is not a number someone typed.** RAM starts at `0x20000000`
   and this part has 12 KB. `0x20000000 + 0x3000` is exactly the top. The
   linker computed it from `link.ld`, and the stack grows *downward* from it.
2. **`0x080006C5` is odd, and every handler address here is.** `nm` reports
   `Reset_Handler` at `080006c4`. The low bit is the **Thumb bit** from
   lesson 01 — the core reads it as "this target is Thumb code", not as part of
   the address. An even entry here faults on the first instruction.
3. **Words 2 and 3 are identical.** `NMI_Handler` and `HardFault_Handler` are
   the same address, because both are weak aliases of one `Default_Handler`.
   Slide 6 is about why that matters.

---

## Slide 8: Weak Aliases — and the Trap That Costs an Afternoon

Every handler in `startup.c` is declared like this:

```c
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

void TIM3_IRQHandler(void)     WEAK_ALIAS;
void USART2_IRQHandler(void)   WEAK_ALIAS;
```

A **weak** symbol is a default that any ordinary definition overrides. So a
lesson that wants the timer interrupt simply writes a plain C function:

```c
void TIM3_IRQHandler(void) { /* ... */ }      /* strong: wins at link time */
```

No `ISR()` macro, no vector number, nothing to register. The name **is** the
registration.

```svg
<svg viewBox="0 0 580 175" role="img" aria-label="A correctly spelled handler overrides the weak alias; a misspelled one does not">
  <text x="145" y="18" text-anchor="middle" class="lbl">SPELLED RIGHT</text>
  <rect class="box" x="20" y="28" width="250" height="30" rx="5"/>
  <text x="145" y="48" text-anchor="middle" class="mono">void TIM3_IRQHandler(void)</text>
  <path class="wire" d="M145 58 V84"/>
  <rect class="ok" x="20" y="86" width="250" height="30" rx="5"/>
  <text x="145" y="106" text-anchor="middle" class="lbl">overrides the weak alias</text>
  <text x="145" y="144" text-anchor="middle" class="lbl">the interrupt runs your code</text>

  <text x="435" y="18" text-anchor="middle" class="lbl">ONE LETTER WRONG</text>
  <rect class="box" x="310" y="28" width="250" height="30" rx="5"/>
  <text x="435" y="48" text-anchor="middle" class="mono">void TIM3_IRQhandler(void)</text>
  <path class="dash" d="M435 58 V84"/>
  <rect class="hifill" x="310" y="86" width="250" height="30" rx="5"/>
  <text x="435" y="106" text-anchor="middle" class="lbl">overrides nothing at all</text>
  <text x="435" y="138" text-anchor="middle" class="hi">compiles. links. runs.</text>
  <text x="435" y="158" text-anchor="middle" class="lbl">the interrupt does nothing. no warning.</text>
</svg>
```

**This is the single most expensive typo on this architecture.** You have
merely defined a function nobody calls, and the weak default still sits in the
vector table. There is no error, no warning, and no symptom except silence.

When an interrupt "does not fire" later this term, check the spelling *first*.

---

## Slide 9: Your Program Has Four Kinds of Bytes

Before the linker can place anything, the compiler sorts every byte of your
program into a **section**. Four of them account for almost everything.

```svg
<svg viewBox="0 0 580 250" role="img" aria-label="The four main sections and whether they need FLASH, RAM, or both">
  <rect class="box" x="16" y="30" width="128" height="86" rx="6"/>
  <text x="80" y="52" text-anchor="middle" class="mono">.text</text>
  <text x="80" y="74" text-anchor="middle" class="lbl">your code</text>
  <text x="80" y="94" text-anchor="middle" class="lbl">FLASH only</text>
  <text x="80" y="110" text-anchor="middle" class="lbl">runs in place</text>

  <rect class="box" x="156" y="30" width="128" height="86" rx="6"/>
  <text x="220" y="52" text-anchor="middle" class="mono">.rodata</text>
  <text x="220" y="74" text-anchor="middle" class="lbl">const data</text>
  <text x="220" y="94" text-anchor="middle" class="lbl">FLASH only</text>
  <text x="220" y="110" text-anchor="middle" class="lbl">read in place</text>

  <rect class="hifill" x="296" y="30" width="128" height="86" rx="6"/>
  <text x="360" y="52" text-anchor="middle" class="mono">.data</text>
  <text x="360" y="74" text-anchor="middle" class="lbl">initialised globals</text>
  <text x="360" y="94" text-anchor="middle" class="hi">FLASH and RAM</text>
  <text x="360" y="110" text-anchor="middle" class="lbl">must be copied</text>

  <rect class="hifill" x="436" y="30" width="128" height="86" rx="6"/>
  <text x="500" y="52" text-anchor="middle" class="mono">.bss</text>
  <text x="500" y="74" text-anchor="middle" class="lbl">zeroed globals</text>
  <text x="500" y="94" text-anchor="middle" class="hi">RAM only</text>
  <text x="500" y="110" text-anchor="middle" class="lbl">must be zeroed</text>

  <line class="dash" x1="16" y1="140" x2="564" y2="140"/>
  <text x="290" y="166" text-anchor="middle">The two on the left are finished when the chip is programmed.</text>
  <text x="290" y="190" text-anchor="middle" class="hi">The two on the right are promises nothing has kept yet.</text>
  <text x="290" y="222" text-anchor="middle" class="lbl">Reset_Handler exists to keep them — and that is nearly all it does.</text>
</svg>
```

The C standard says `int counter = 0;` is zero when `main()` begins and
`int magic = 0xC0FFEE;` holds that value. Those are promises made by the
language. **On this chip you are the one who keeps them.**

---

## Slide 10: Why `.data` Has Two Addresses

A global with an initial value presents a problem. The value must survive power
off, so it has to be in FLASH. But the variable must be writable, so it has to
live in RAM. It therefore needs **two addresses**, and the linker gives it two.

```svg
<svg viewBox="0 0 580 240" role="img" aria-label="data has a load address in FLASH and a virtual address in RAM; startup copies between them">
  <defs><marker id="a5" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>

  <text x="120" y="20" text-anchor="middle" class="lbl">FLASH — survives power off</text>
  <rect class="box" x="30" y="30" width="180" height="150" rx="6"/>
  <rect class="reg" x="44" y="44" width="152" height="30" rx="4"/>
  <text x="120" y="64" text-anchor="middle" class="mono">.text .rodata</text>
  <rect class="hifill" x="44" y="88" width="152" height="40" rx="4"/>
  <text x="120" y="106" text-anchor="middle" class="mono">.data master copy</text>
  <text x="120" y="122" text-anchor="middle" class="lbl">100 bytes</text>
  <text x="120" y="152" text-anchor="middle" class="mono">_sidata</text>
  <text x="120" y="168" text-anchor="middle" class="lbl">= 0x08001D64</text>

  <text x="460" y="20" text-anchor="middle" class="lbl">RAM — writable, empty at reset</text>
  <rect class="box" x="370" y="30" width="180" height="150" rx="6"/>
  <rect class="hifill" x="384" y="44" width="152" height="40" rx="4"/>
  <text x="460" y="62" text-anchor="middle" class="mono">.data working copy</text>
  <text x="460" y="78" text-anchor="middle" class="lbl">0x20000000 .. 0x20000064</text>
  <rect class="reg" x="384" y="96" width="152" height="40" rx="4"/>
  <text x="460" y="114" text-anchor="middle" class="mono">.bss</text>
  <text x="460" y="130" text-anchor="middle" class="lbl">372 bytes, must be zeroed</text>
  <text x="460" y="164" text-anchor="middle" class="lbl">stack grows down from the top</text>

  <path class="wire" d="M212 104 H366" marker-end="url(#a5)"/>
  <text x="289" y="96" text-anchor="middle" class="hi">Reset_Handler copies</text>
  <text x="289" y="210" text-anchor="middle" class="lbl">The load address (LMA) is where it is stored. The virtual address (VMA) is where it runs.</text>
  <text x="289" y="230" text-anchor="middle" class="lbl">For .text those are the same. For .data they are not — and that is the whole idea.</text>
</svg>
```

`.bss` has the opposite shape. Because every byte of it is zero, storing it in
FLASH would mean storing hundreds of identical zeros. It is given **no FLASH at
all** — only a size — and the startup code writes the zeros at run time.

---

## Slide 11: `Reset_Handler`, in Full

Those two promises are eleven lines of C:

```c
void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* 1. copy .data from FLASH into RAM */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; }

    /* 2. zero .bss */
    for (dst = &_sbss; dst < &_ebss; ) { *dst++ = 0; }

    SystemInit();          /* 3. clock: 12 MHz -> 48 MHz                 */
    __libc_init_array();   /* 4. C++ ctors, __attribute__((constructor)) */

    (void)main();
    for (;;) { }           /* main() must not return — but if it does    */
}
```

Read what those loops actually are: **two `memcpy`-shaped loops over addresses
the linker chose.** That is the entire mystery of "what happens before `main`".

Note the last line. `main()` returning on a desktop means the process exits.
Here there is nothing to exit *to*, so a bare return would run off into
whatever bytes follow in FLASH. The infinite loop is a wall.

---

## Slide 12: Linker Symbols Are Addresses, Not Variables

This is the one piece of syntax in the lesson that is genuinely strange, and
getting it wrong produces a bug that looks like corrupted memory.

```c
extern uint32_t _sdata, _edata;

for (dst = &_sdata; dst < &_edata; ) { ... }   /* correct  */
uint32_t n = _edata - _sdata;                  /* WRONG    */
```

`_sdata` is **not a variable holding an address**. The linker does not create
storage for it; it creates a *symbol* whose value **is** the address. So:

| You write | You get |
|---|---|
| `&_sdata` | the address the linker chose — what you want |
| `_sdata` | the four bytes that happen to live at that address |

The declaration `extern uint32_t _sdata;` is a polite lie told to the compiler
so that `&_sdata` has a type. Reading it directly compiles cleanly and returns
garbage.

> **Rule:** a linker symbol is only ever used with `&`. If you see one without
> an ampersand, it is a bug.

`Main.c` in this lesson uses eight of them, all with `&`, and prints every one.

---

## Slide 13: `link.ld`, Part 1 — Declaring the Map

The linker script has two halves. The first says what memory exists — and it is
**slide 3's picture, transcribed**:

```
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH = 12K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);     /* 0x20003000 */
```

Four facts, and all four come straight from the reference manual:

- FLASH is at `0x08000000` — **not** at 0. The Cortex-M memory map from lesson
  00 reserves the low addresses for the code region alias.
- RAM is at `0x20000000`, which is the same on every Cortex-M part you will
  ever meet. That address is architectural, not ST's choice.
- `32K` and `12K` are this exact part number. A C031**C6** has 32 KB of flash;
  change the part and this line changes with it.
- `_estack` is **computed**, not typed. That is why slide 7's `0x20003000` is
  trustworthy: nobody could have mistyped it.

---

## Slide 14: `link.ld`, Part 2 — Placing the Sections

The second half assigns each section to a region. Three lines carry the whole
idea:

```
.text   : { *(.text*) }   >FLASH

.data   : {
    _sdata = .;
    *(.data*)
    _edata = .;
}  >RAM AT> FLASH          /*  <-- lives in RAM, stored in FLASH  */

_sidata = LOADADDR(.data); /*  <-- where it was stored            */

.bss    : {
    _sbss = .;  *(.bss*) *(COMMON);  _ebss = .;
}  >RAM                    /*  no AT> — nothing to store          */
```

`>RAM AT> FLASH` is the syntax for slide 10's two addresses, and `LOADADDR()` is
how the startup code finds the FLASH copy. **These three lines are why `.data`
works.**

`.` is the location counter — "the address I have reached so far". Assigning it
to a symbol, as in `_sdata = .;`, is how every boundary marker in the file is
made. The dot is the linker's entire working memory.

---

## Slide 15: Where the Memory Actually Went

This is `arm-none-eabi-size -A` on this lesson's own build:

```
section               size          addr
.isr_vector            180    0x08000000
.text                 5684    0x080000b4
.rodata               1652    0x080016e8
.data                  100    0x20000000
.bss                   372    0x20000064
._user_heap_stack     1536    0x20000218
```

and the linker's own summary, printed at every build:

```
Memory region     Used Size   Region Size   %age Used
       FLASH:        7624 B        32 KB       23.27%
         RAM:        2008 B        12 KB       16.34%
```

Check the arithmetic yourself — both sums teach something:

- **RAM** = 100 + 372 + 1536 = **2008**. The heap and stack reservation is
  three quarters of your RAM use, and you have not written a real program yet.
- **FLASH** = 180 + 5684 + 1652 + 4 + 4 + **100** = **7624**. That 100 is
  `.data`, counted a second time — once at its RAM address, once at its FLASH
  load address. **`.data` costs you both.** A large initialised array is the
  most expensive thing you can declare on a part this size.

The lesson for later: prefer `const` wherever you can. `.rodata` costs FLASH
only, and `.bss` costs no FLASH at all.

---

## Slide 16: The Clock — 48 MHz in One Register Write

The last of the four startup jobs. The STM32C0 wakes on its 48 MHz internal
oscillator **divided by four**, so 12 MHz.

```c
void SystemInit(void)
{
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0) { }

    RCC->CR &= ~RCC_CR_HSIDIV;          /* divide by 1 -> 48 MHz */
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    SystemCoreClock = 48000000UL;
}
```

**The order is not negotiable.** Flash memory cannot be read as fast as the CPU
runs at 48 MHz, so it needs one wait state. Raise the wait state *first*, then
the clock. Do it the other way round and the CPU fetches an instruction that is
not ready yet — and the part hangs with no error, somewhere that has nothing to
do with your bug.

> **The rule for every clock change you will ever make:** wait states go up
> *before* the frequency, and come down *after* it.

This is also the gentlest clock setup in the STM32 family: no PLL, no lock bit
to poll, no source switch. The F446RE target later in the course needs all
three, and you will appreciate this one by comparison.

---

## Slide 17: `printf` Needs Somewhere to Print

`printf` lives in the C library, and the C library has never heard of a UART.
It calls `_write()` and expects the system to provide one. On a laptop the OS
does. Here, `retarget.c` does, in nine lines:

```c
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

That one function is the entire bridge between `<stdio.h>` and your hardware.
Implement it and `printf`, `puts`, `putchar` and `fprintf` all work at once.

Two notes worth keeping:

- **The AVR edition of this course hand-wrote fifteen formatters** to avoid
  avr-libc's `printf` on a 4 KB part. Here newlib-nano's costs about 1.4 KB of
  32 KB, which is affordable, so this edition simply uses the standard library.
- `_sbrk()` in the same file hands `malloc` the gap between `.bss` and the
  stack. `printf` allocates its buffer there. Get `_sbrk` wrong and `printf`
  prints *nothing* — a silent board running a perfectly good program.

---

## Slide 18: The Board You Are Simulating

Everything from here to the end of the course runs on one part: the
**ST Nucleo-C031C6**, in the browser at
[docs.wokwi.com/parts/board-st-nucleo-c031c6](https://docs.wokwi.com/parts/board-st-nucleo-c031c6).
That link is in the bar at the top of every slide, next to the manual for the
real board, **UM2953** — the header pinouts, solder bridges and schematics that
the simulated one is a model of.

```svg
<svg viewBox="0 0 580 224" role="img" aria-label="What this lesson uses on the Nucleo-C031C6 board">
  <rect class="box" x="100" y="24" width="380" height="150" rx="8"/>
  <text x="290" y="46" text-anchor="middle" class="mono">STM32C031C6</text>
  <text x="290" y="64" text-anchor="middle" class="lbl">Cortex-M0+ · 48 MHz · 32 KB flash · 12 KB RAM</text>

  <rect class="hifill" x="116" y="82" width="110" height="34" rx="4"/>
  <text x="171" y="103" text-anchor="middle" class="mono">PB0 .. PB7</text>
  <rect class="hifill" x="235" y="82" width="110" height="34" rx="4"/>
  <text x="290" y="103" text-anchor="middle" class="mono">PA5</text>
  <rect class="hifill" x="354" y="82" width="110" height="34" rx="4"/>
  <text x="409" y="103" text-anchor="middle" class="mono">PA2 / PA3</text>

  <text x="171" y="132" text-anchor="middle" class="lbl">eight LEDs</text>
  <text x="171" y="148" text-anchor="middle" class="lbl">from diagram.json</text>
  <text x="290" y="132" text-anchor="middle" class="lbl">user LED LD4</text>
  <text x="290" y="148" text-anchor="middle" class="lbl">the heartbeat</text>
  <text x="409" y="132" text-anchor="middle" class="lbl">USART2, AF1</text>
  <text x="409" y="148" text-anchor="middle" class="lbl">to the serial monitor</text>

  <rect class="reg" x="4" y="88" width="88" height="34" rx="4"/>
  <text x="48" y="109" text-anchor="middle" class="lbl">your patterns</text>
  <path class="wire" d="M92 105 H112"/>

  <rect class="reg" x="488" y="88" width="88" height="34" rx="4"/>
  <text x="532" y="109" text-anchor="middle" class="lbl">you read printf</text>
  <path class="wire" d="M468 105 H484"/>

  <text x="290" y="200" text-anchor="middle" class="lbl">Eleven pins. The eight on the left are the only ones you chose; the board fixed the rest.</text>
  <text x="290" y="218" text-anchor="middle" class="lbl">Wokwi part type: board-st-nucleo-c031c6</text>
</svg>
```

Eleven pins matter this week. Eight of them are the only hardware choice this
lesson makes; the other three are fixed by the board:

| Pin | Wired to | Used by |
|---|---|---|
| **PB0–PB7** | eight LEDs added by `diagram.json`, anode to pin, cathode to GND | `ledbar_init()`, your patterns |
| **PA5** | the on-board user LED (Wokwi labels it **LD4**) | `led_init()`, the per-frame heartbeat |
| **PA2** | USART2 TX, alternate function 1 | `printf` → the serial monitor |
| **PA3** | USART2 RX, alternate function 1 | (not used this week) |

The eight LEDs are wires in `diagram.json`, and a wire names its board pin by
**the string in Wokwi's board file, not the datasheet name**: `led0` is on
`nucleo:PB0.1`, because PB0 reaches two header positions and plain `PB0` does
not exist. Every name Wokwi accepts is on the **Board pins** page in the top
bar of this slide. Check it before you add a wire.

**Read the "not implemented" list on that page before you plan anything.**
Wokwi simulates the CPU and most peripherals, but not all of them. As of now
it lists **DMA, RTC, IWDG, PWR, SYSCFG and DBG as unavailable**, and I²C and
SPI as master-only.

That list is not a footnote — it decides what this course can demonstrate:

- It is why the external-interrupt lesson works at all. On an STM32F4 the pin
  that raises an interrupt is chosen in `SYSCFG->EXTICR[]`, and SYSCFG is on
  the unsupported list. **On the STM32C0 that register lives in `EXTI->EXTICR[]`
  instead**, so the lesson has a home here and would not have had one on an F4.
- It is why the low-power lesson moves to a different simulator later: `PWR`
  is not modelled, and neither simulator shows current draw anyway.

> A simulator is an *instrument*, and every instrument has a stated range.
> Knowing what it does not model is as much a part of using it as knowing what
> it does.

---

## Slide 19: What You Should See

Build, upload `Main.elf` to the Wokwi board in your browser, and the serial
monitor shows this — with **your** numbers, from **your** build:

```
=== SOC3050 lesson 04 - Startup and the Linker Script ===
    STM32C031C6, Cortex-M0+

-- what the clock actually is --
  SystemCoreClock : 48000000 Hz

-- where link.ld put things --
  vector table    : 0x08000000  (must be the base of FLASH)
  .data in FLASH  : 0x08001D64  _sidata, the copy source
  .data in RAM    : 0x20000000 .. 0x20000064  (100 bytes)
  .bss  in RAM    : 0x20000064 .. 0x200001D8  (372 bytes)
  heap starts at  : 0x200001D8  (first byte after .bss)
  stack top       : 0x20003000  _estack, loaded into SP by
                    the HARDWARE before any code runs

-- did startup.c do its four jobs? --
  .data copied    : 0x00C0FFEE  OK    (expect 0x00C0FFEE)
  .bss  zeroed    : 0x00000000  OK    (expect 0x00000000)
```

**Not one of those numbers is hardcoded.** Every address is read back through
`&` from a linker symbol at run time. Edit `link.ld` and they move — which is
exactly what the lab asks you to do.

Then the LED bar starts: eight LEDs on PB0–PB7 stepping through the pattern
tables at the bottom of `Main.c`, with LD4 on PA5 toggling once per frame as a
heartbeat. The serial monitor names each pattern as it starts. **That half of
`Main.c` is yours to change** — the tables, the timings, and the playlist in
`main()`. Lab Part 6 says how.

---

## Slide 20: The Lab — Break It on Purpose

Startup code is invisible when it works, so the lab makes it fail in four
controlled ways. Full instructions are in **`Lab.md`**.

| | You change | Predict, then observe |
|---|---|---|
| **1** | delete the `.data` copy loop | What does `data_witness` read instead of `0x00C0FFEE`? |
| **2** | delete the `.bss` zero loop | Where does the frame counter start? Is it the same on every run? |
| **3** | **move `.isr_vector` after `.text`** | Does it still build? Does anything warn you? |
| **4** | 12K → 8K → 2K → 1K of RAM | Which printed numbers move? Where does the linker stop you? |

**Write your prediction down before you build.** A prediction you got wrong
teaches you something; a result you merely watched teaches you nothing.

**Exercise 3 is the one to take seriously.** Moving one line in `link.ld`
produces firmware that compiles with no errors, links with no warnings, and
emits a structurally valid hex file with correct checksums — and that a real
chip cannot start, because the hardware reads the first two words of FLASH and
those are now *instruction bytes* rather than your stack pointer and reset
vector.

Every automated check in this repository would pass on that image. That is the
failure mode this course cares about more than any other, and in the lab you
will build it deliberately, so that you recognise it when you build it by
accident.

---

## Slide 21: What Carries Forward

You now own every line of code that runs on this chip. Nothing below `main()`
is a black box any more, and for the rest of the course that pays:

- **Every later lesson links this same `startup.c` shape.** When you add a
  timer interrupt in lesson 08, you will define `TIM3_IRQHandler` and know
  precisely why that name and no other name works.
- **When a board is silent, you now have a checklist.** Did `.data` get copied?
  Is `SystemCoreClock` 48 MHz or still 12? Is the handler name spelled exactly
  right? Is the peripheral clock even enabled?
- **`size -A` is now readable**, and on a 32 KB part you will read it often.

Next week: **GPIO** — the first peripheral, and the first instance of the
Part 0 peripheral model. Control, status, data, behind a clock gate and a pin
mux. You have already met its clock gate: `RCC->IOPENR`, the line in
`led_init()` that must come first or the entire port reads back zero.
