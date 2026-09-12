# Development: From Text to a Chip That Runs
## SOC3050 ARM Edition — Part 0, Models

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

The third of four lessons with **no code to write**.

You have a model of the machine and of its instructions. This lesson is about
the *other* machine — the toolchain — and it exists because on a desktop you
can ignore it for years, and here you cannot ignore it for a week.

Every figure is real output from this repository's build of
`_spike/c031c6/`.

---

## Slide 1: On a PC, an Operating System Hides All of This

When you run `gcc hello.c` on a laptop, something enormous happens that you
never see: the OS decides where your program lives in memory, provides a stack,
zeroes your globals, opens `stdout`, and calls `main()` for you.

**There is no operating system here.** Every one of those jobs still has to be
done, and if you do not arrange it, nothing does.

> The question this lesson answers: between power-on and the first line of
> `main()`, who does the work — and what happens if nobody does?

---

## Slide 2: Five Stages, Not One

`gcc` is not a compiler. It is a driver that runs four or five programs in
sequence, and on embedded the last one is the one that bites.

```svg
<svg viewBox="0 0 560 230" role="img" aria-label="Preprocess, compile, assemble, link and locate">
  <defs><marker id="d1" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="14" y="40" width="96" height="40" rx="5"/>
  <text x="62" y="60" text-anchor="middle" class="mono">main.c</text>
  <path class="wire" d="M110 60 H140" marker-end="url(#d1)"/>
  <rect class="box" x="144" y="40" width="96" height="40" rx="5"/>
  <text x="192" y="55" text-anchor="middle" class="lbl">preprocess</text>
  <text x="192" y="72" text-anchor="middle" class="lbl">compile</text>
  <path class="wire" d="M240 60 H270" marker-end="url(#d1)"/>
  <rect class="box" x="274" y="40" width="96" height="40" rx="5"/>
  <text x="322" y="60" text-anchor="middle" class="lbl">assemble</text>
  <path class="wire" d="M370 60 H400" marker-end="url(#d1)"/>
  <rect class="box" x="404" y="40" width="96" height="40" rx="5"/>
  <text x="452" y="60" text-anchor="middle" class="mono">main.o</text>
  <path class="wire" d="M452 80 V116" marker-end="url(#d1)"/>
  <rect class="hifill" x="180" y="120" width="280" height="44" rx="6"/>
  <text x="320" y="136" text-anchor="middle">LINK and LOCATE</text>
  <text x="320" y="154" text-anchor="middle" class="lbl">resolve names, then assign real addresses</text>
  <path class="hi" d="M180 142 H120 V60 H110" />
  <rect class="box" x="14" y="120" width="96" height="44" rx="5"/>
  <text x="62" y="136" text-anchor="middle" class="mono">startup.c</text>
  <text x="62" y="154" text-anchor="middle" class="mono">link.ld</text>
  <text x="280" y="200" text-anchor="middle" class="lbl">On a PC the last stage is invisible. Here you write its input by hand.</text>
</svg>
```

Stages 1–3 are ordinary and familiar. **Stage 4, locate, is the embedded one**:
deciding that `SystemInit` lives at `0x0800022c` and `SystemCoreClock` at
`0x20000000`. On a PC the OS does that at load time. Here it is decided at
build time and burned into the image.

---

## Slide 3: The Compiler Sees One File at a Time

Each `.c` file is compiled alone, into an `.o`, knowing nothing about the
others. Where it needs a name it does not have, it leaves a hole and a note.

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="Object files with defined and undefined symbols, resolved by the linker">
  <rect class="box" x="20" y="24" width="150" height="92" rx="6"/>
  <text x="95" y="44" text-anchor="middle" class="mono">main.o</text>
  <text x="95" y="68" text-anchor="middle" class="lbl">defines: main, _write</text>
  <text x="95" y="90" text-anchor="middle" class="lbl">needs: printf, SystemCoreClock</text>
  <rect class="box" x="200" y="24" width="150" height="92" rx="6"/>
  <text x="275" y="44" text-anchor="middle" class="mono">startup.o</text>
  <text x="275" y="68" text-anchor="middle" class="lbl">defines: Reset_Handler,</text>
  <text x="275" y="86" text-anchor="middle" class="lbl">SystemCoreClock</text>
  <text x="275" y="106" text-anchor="middle" class="lbl">needs: main</text>
  <rect class="box" x="380" y="24" width="160" height="92" rx="6"/>
  <text x="460" y="44" text-anchor="middle" class="mono">libc_nano.a</text>
  <text x="460" y="68" text-anchor="middle" class="lbl">defines: printf</text>
  <text x="460" y="90" text-anchor="middle" class="lbl">needs: _write, _sbrk</text>
  <path class="dash" d="M95 116 V140 H460 V116"/>
  <path class="dash" d="M275 116 V140"/>
  <text x="280" y="168" text-anchor="middle" class="lbl">The linker matches every "needs" to exactly one "defines".</text>
  <text x="280" y="186" text-anchor="middle" class="lbl">None left over, none duplicated — or it refuses.</text>
</svg>
```

This explains the two errors you will meet most often:

- **`undefined reference to X`** — somebody needs `X` and nobody defines it.
  Real example from building the spike: `undefined reference to SystemCoreClock`,
  because ST's header *declares* it but their `system_stm32c0xx.c` — which we
  deliberately do not use — *defines* it.
- **`multiple definition of X`** — two files both define it, and the linker
  will not choose for you.

---

## Slide 4: Your Program Is Not One Blob — It Is Sections

The compiler sorts everything it emits into named **sections** by what kind of
thing it is. This is the real output for the spike:

```
section          size    addr
.isr_vector       180    0x08000000     the vector table
.text            4792    0x080000b4     code
.rodata           184    0x08002cac     const data, string literals
.data              96    0x20000000     initialised variables
.bss              364    0x20000060     variables that start at zero
```

Read the addresses. Two distinct places:

```svg
<svg viewBox="0 0 560 220" role="img" aria-label="Sections placed into flash and SRAM">
  <text x="140" y="22" text-anchor="middle" class="lbl">FLASH — 0x08000000</text>
  <rect class="box" x="40" y="34" width="200" height="30" rx="4"/>
  <text x="140" y="49" text-anchor="middle" class="mono">.isr_vector</text>
  <rect class="box" x="40" y="68" width="200" height="30" rx="4"/>
  <text x="140" y="83" text-anchor="middle" class="mono">.text</text>
  <rect class="box" x="40" y="102" width="200" height="30" rx="4"/>
  <text x="140" y="117" text-anchor="middle" class="mono">.rodata</text>
  <rect class="hifill" x="40" y="136" width="200" height="30" rx="4"/>
  <text x="140" y="151" text-anchor="middle" class="mono">.data (a copy)</text>
  <text x="420" y="22" text-anchor="middle" class="lbl">SRAM — 0x20000000</text>
  <rect class="hifill" x="320" y="34" width="200" height="30" rx="4"/>
  <text x="420" y="49" text-anchor="middle" class="mono">.data (the real one)</text>
  <rect class="box" x="320" y="68" width="200" height="30" rx="4"/>
  <text x="420" y="83" text-anchor="middle" class="mono">.bss</text>
  <rect class="reg" x="320" y="102" width="200" height="30" rx="4"/>
  <text x="420" y="117" text-anchor="middle" class="lbl">heap</text>
  <rect class="reg" x="320" y="136" width="200" height="30" rx="4"/>
  <text x="420" y="151" text-anchor="middle" class="lbl">stack (grows down)</text>
  <path class="hi" d="M240 151 H290 V49 H320"/>
  <text x="280" y="196" text-anchor="middle" class="lbl">.data exists twice: the values live in flash, the variables live in RAM.</text>
</svg>
```

---

## Slide 5: The Awkward One — `.data`

A variable like `uint32_t SystemCoreClock = 12000000;` must be **writable**, so
it has to live in RAM. Its initial value must **survive power-off**, so it has
to live in flash. Both are true, so it exists in both places.

Somebody has to copy 96 bytes from flash to RAM before `main()` runs. On a PC
the loader does it. Here, `Reset_Handler` does — it is four lines of the
startup file:

```c
src = &_sidata;                                  /* where the values sit in flash */
for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; }
for (dst = &_sbss;  dst < &_ebss;  ) { *dst++ = 0; }
```

`_sidata`, `_sdata`, `_edata`, `_sbss`, `_ebss` are not variables. **The linker
invents them**, and their values are addresses it chose. This is the one place
where your C code reads decisions the linker made.

> Skip the `.bss` loop and globals start as whatever was in RAM at power-on.
> The program works perfectly on a cold boot and misbehaves after a reset.

---

## Slide 6: The Linker Script Is the Memory Map, Written Down

The linker knows nothing about your chip. Everything on the last two slides —
where flash starts, how big RAM is, which section goes where — you tell it:

```
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (rw) : ORIGIN = 0x20000000, LENGTH = 12K
}
```

Those four numbers come from the datasheet, and they are the difference between
firmware that runs and firmware that does not. Get `LENGTH` wrong and the build
succeeds — the failure arrives later, as a stack that quietly walks off the end
of memory.

The build also reports what it used:

```
Memory region   Used Size   Region Size   %age Used
       FLASH:     5260 B        32 KB       16.05%
         RAM:     2000 B        12 KB       16.28%
```

**On a 32 KB part that number belongs in every build log.** It is the only
early warning you get.

---

## Slide 7: What Actually Gets Programmed

The linker's output is an `.elf` — rich, with symbols and debug info, for the
debugger and the simulator. What a programmer writes to flash is stripped down:

| File | What it is | Used by |
|---|---|---|
| `Main.elf` | code + addresses + symbols + debug | debugger, simulator |
| `Main.bin` | raw bytes, exactly as in flash | some programmers |
| `Main.hex` | the same bytes as printable text records | most programmers |

The first two records of the real `Main.hex`:

```
:020000040800F2
:1000000000300020750200082902000829020008BB
```

| Field | Meaning |
|---|---|
| `:` | every record starts with a colon |
| `10` | 16 data bytes follow |
| `0000` | at offset 0x0000 |
| `00` | record type: data |
| `00300020` | `0x20003000` — the initial stack pointer |
| `750200 08` | `0x08000275` — the reset vector |
| `BB` | checksum |

The first record, `:02000004 0800`, is an **extended linear address** record —
it sets the upper half of the address to `0x0800`. An 8-bit AVR never needed
one, because its whole flash fits in 16 bits of address. Yours does not.

---

## Slide 8: What Runs Before `main()`

Putting it together — the complete path from power to your code:

```svg
<svg viewBox="0 0 560 250" role="img" aria-label="Reset sequence from power-on to main">
  <defs><marker id="d2" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="160" y="16" width="240" height="30" rx="4"/>
  <text x="280" y="31" text-anchor="middle">power on / reset</text>
  <path class="wire" d="M280 46 V62" marker-end="url(#d2)"/>
  <rect class="hifill" x="160" y="62" width="240" height="30" rx="4"/>
  <text x="280" y="77" text-anchor="middle">hardware loads SP and PC from vector[0], [1]</text>
  <path class="wire" d="M280 92 V108" marker-end="url(#d2)"/>
  <rect class="box" x="160" y="108" width="240" height="30" rx="4"/>
  <text x="280" y="123" text-anchor="middle" class="mono">Reset_Handler</text>
  <path class="wire" d="M280 138 V154" marker-end="url(#d2)"/>
  <rect class="box" x="120" y="154" width="320" height="30" rx="4"/>
  <text x="280" y="169" text-anchor="middle">copy .data · zero .bss · SystemInit · __libc_init_array</text>
  <path class="wire" d="M280 184 V200" marker-end="url(#d2)"/>
  <rect class="hifill" x="200" y="200" width="160" height="30" rx="4"/>
  <text x="280" y="215" text-anchor="middle" class="mono">main()</text>
</svg>
```

The second box is the only step the hardware does for you, and it is worth
being precise: the CPU reads **two 32-bit words from address zero of flash** —
the stack pointer and the reset vector — and starts. That is the entire boot
process. Everything else is code you can read.

---

## Slide 9: Your Instruments

You will be given two tools, and they answer different questions.

| | Wokwi | Renode |
|---|---|---|
| Where | a browser tab | a terminal |
| Shows you | the board: LEDs, displays, wires | serial output, registers, instruction counts |
| Good for | *does the circuit behave?* | *does the code behave?* |
| Cost | free, nothing to install | free, nothing to install |

Both load the same `Main.elf` you just built. Neither compiles anything — the
compiler runs on your machine, and the simulator only executes.

> A habit worth forming now: **a thing is not working until you have watched it
> work.** A clean build, a valid hex file and a correct-looking register dump
> have all, in this repository's history, been produced by firmware that drove
> nothing at all.

---

## Slide 10: What You Should Be Able to Say

1. Name the five stages of a build. Which one is the embedded-specific one?
2. Where does `.data` live — flash or RAM? Defend your answer.
3. What are `_sdata` and `_ebss`, and who creates them?
4. What happens if the `.bss` loop is missing? Why is that bug intermittent?
5. Two numbers in the linker script come straight from the datasheet. Which,
   and what goes wrong if they are too large?
6. The hex file begins `:02000004 0800`. What is that record for, and why did
   the 8-bit chips in the old course never need one?

**Next:** *Execution and Concurrency* — what happens when the hardware
interrupts `main()`, and why two pieces of code sharing one variable is the
hardest problem in the course.
