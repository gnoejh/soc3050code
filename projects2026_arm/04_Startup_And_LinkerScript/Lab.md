# Lab 04 — Startup and the Linker Script

**SOC3050 ARM Edition · Nucleo-C031C6 · allow 2 hours**

**Reference**: [RM0490, STM32C0x1 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board**: [ST Nucleo-C031C6 on Wokwi](https://docs.wokwi.com/parts/board-st-nucleo-c031c6)

---

## What this lab is

**A guided walkthrough, not a test.** You will build the first program of this
course, then break its startup code on purpose — five times — and each time the
lab tells you what happened and why.

Startup code is invisible when it works. The only way to see what it does is to
take it away.

Work through it in order. Each part says **Do this**, asks you to **guess**
before you build, then explains **what you see and why**. The guess is not
marked and nobody collects it — it is there because you remember a surprise far
better than a paragraph.

**Bring nothing.** The compiler is in the repository; the simulator is a web
page.

---

## Part 0 — Build it and run it (20 min)

### Step 1. Build

Open a terminal in this folder and run:

```
build.bat
```

You should see, with no warnings at all:

```
Building 04_Startup_And_LinkerScript  [target c031c6] ...
Memory region         Used Size  Region Size  %age Used
           FLASH:        6736 B        32 KB     20.56%
             RAM:        2008 B        12 KB     16.34%
...
Build OK: Main.elf, Main.hex and Main.bin created
```

Three files appear. **`Main.elf`** is what you will load — it carries the symbol
table a debugger needs. `Main.hex` and `Main.bin` are the same program in
plainer formats.

> **If it fails:** you are probably not in this folder. `build.bat` compiles
> every `.c` in the *current* directory, so it must be run from inside the
> lesson.

### Step 2. Simulate

```
simulate.bat
```

This **always rebuilds first** — deliberately, so you can never simulate a
stale binary — then opens a Wokwi tab on the Nucleo-C031C6.

In that tab:

1. Click into the code editor area and press **`F1`**
2. Choose **`Upload Firmware and Start Simulation…`**
3. Select **`Main.elf`** from this folder
4. Watch the **serial monitor** panel under the board

> Wokwi documents "upload your own firmware" only in its ESP32 guide, but it
> works for STM32 the same way. Don't go hunting for it in the STM32 docs.

### Step 3. Read the output

You get the memory report from slide 19, and the user LED (PA5, which Wokwi
labels **LD4**) blinks.

Now look at what it printed, because every number in it was **measured at run
time from a linker symbol** — none of it is hardcoded.

**Try this — where does FLASH end?** It starts at `0x08000000` and the part has
32 KB.

> `0x08000000 + 0x8000 = 0x08008000`, so the last valid byte is `0x08007FFF`.
> Your printed `.data in FLASH` value is `0x080019EC`, comfortably inside it.
> That address is where the *master copy* of your initialised globals is kept.

**Sort the printed addresses onto slide 3's map.** Two groups:

> - **`0x08…`** — the vector table (`0x08000000`) and `_sidata` (`0x080019EC`)
>   are in FLASH.
> - **`0x20…`** — `.data`, `.bss`, the heap start and the stack top are all in
>   SRAM.
>
> Nothing printed is outside those two regions. The peripheral addresses from
> slide 4 exist, but this program never prints them.

### Step 4. Check a peripheral address yourself

Slide 4 claims `GPIOA->ODR` is the address `0x50000014`. Don't take its word
for it. Add this line to `main()` after `memory_report();`, rebuild, re-upload:

```c
printf("GPIOA->ODR is at 0x%08lX\n", (unsigned long)(uintptr_t)&GPIOA->ODR);
```

> It prints `0x50000014`.
>
> **That is not in the `0x40000000` peripheral region** — on the STM32C0 the
> I/O ports live in their own `IOPORT` region at `0x50000000`. On an F4 or an
> F1 the same line would print an address starting `0x40…`.
>
> This is why register-level code does not port between STM32 families by
> copy-paste, and why the reference manual for *your* part is the only address
> you should trust.

### Step 5. Switch off a clock and watch something vanish

`led_init()` in `Main.c` starts with:

```c
RCC->IOPENR |= RCC_IOPENR_GPIOAEN;      /* clock the port FIRST, always */
```

Comment out **only that line** — the one in `Main.c`, not the identical one in
`retarget.c`. Rebuild and re-upload.

**Guess first:** does the LED stop? Does the serial output stop? Both?

> **The serial output is fine. The LED is dead.**
>
> Both use pins on GPIOA, so why do they behave differently? Look at the order
> in `main()`:
>
> ```c
> led_init();                          /* GPIOA has no clock yet */
> uart2_init(SystemCoreClock, BAUD);   /* this one still enables it */
> ```
>
> `led_init()` writes `GPIOA->MODER` while the port is unclocked. **A write to
> an unclocked peripheral is silently discarded** — no fault, no warning, the
> value simply never arrives. So PA5 is never configured as an output.
>
> Then `uart2_init()` runs, enables the same clock gate, and configures PA2 and
> PA3 successfully. The UART works. PA5 was configured during the dead window
> and nobody goes back to fix it.
>
> **Remember this shape.** One peripheral works, another on the same port does
> not, and both pieces of code look correct. The fault is not in either one —
> it is in the *order*. You will meet it again.

**Put the line back.**

---

## Part 1 — Remove the `.data` copy (15 min)

`Main.c` declares a global with an initial value:

```c
volatile uint32_t data_witness = 0x00C0FFEEu;
```

It is initialised and not `const`, so it lives in `.data`: a master copy in
FLASH, a working copy in RAM, and a loop in `Reset_Handler` that moves one to
the other.

### Do this

In `startup.c`, find `---- LAB EXERCISE 1 ----` and comment out the copy loop:

```c
    src = &_sidata;
    /* for (dst = &_sdata; dst < &_edata; ) { *dst++ = *src++; } */
```

**Guess first:** does `data_witness` read `0x00C0FFEE` anyway, read zero, read
garbage, or fail to build?

### What you see

> It builds and runs, and the report line stops saying `OK`:
>
> ```
>   .data copied    : 0x00000000  WRONG  (expect 0x00C0FFEE)
> ```
>
> **Almost certainly zero, because the simulator hands you RAM that starts
> zeroed.** Nothing copied the real value in, so you are reading whatever the
> RAM happened to contain. If your simulator shows something other than
> `0x00000000` here, that is not a mistake — read on, because it is the more
> honest answer.
>
> **On a real chip you are not promised zero.** Powered-on SRAM comes up in an
> arbitrary state, and after a *warm* reset it still holds the previous
> program's data. The honest answer to "what does it read?" is **indeterminate**
> — the simulator is showing you one of the possible answers, not the rule.
>
> That gap matters: code that works in simulation and fails on hardware very
> often has exactly this shape.

**Also notice what did *not* happen.** The LED still blinks and the serial port
still works.

> Removing a whole initialisation step crashed nothing, because most of the
> program never depended on it. Missing startup work does not announce itself —
> it produces one wrong value somewhere and lets everything else carry on.

**Put the line back.**

---

## Part 2 — Remove the `.bss` zeroing (15 min)

`Main.c` also has:

```c
volatile uint32_t bss_witness = 0u;
static   uint32_t blink_count;      /* no initialiser at all */
```

Both are `.bss`. Nothing about them is stored in FLASH — only their *size* is
recorded — and `Reset_Handler` writes the zeros at run time.

### Do this

Comment out the second loop, at `---- LAB EXERCISE 2 ----`:

```c
    /* for (dst = &_sbss; dst < &_ebss; ) { *dst++ = 0; } */
```

**Guess first:** what does the first `blink N` line say?

### What you see

> **Most likely: nothing changes at all.** `bss_witness` still reports `OK` and
> the counter still starts at 8.
>
> This is the *right* result and the interesting one. Wokwi starts you with
> zeroed RAM, so deleting the code that writes zeros changes nothing you can
> see. **The bug is completely invisible in this simulator.**
>
> Now reason about the real part. `.bss` on hardware is whatever was in SRAM
> beforehand. After a power-on it is arbitrary; after a warm reset it is the
> *previous run's* leftovers. So on a bench this same firmware might count from
> `3,141,592`, or work perfectly for a week and then not.

**Why this bug is famously hard to find.** Three reasons, and you can see all
of them from here:

> 1. **It is invisible wherever RAM starts clean** — your simulator, your
>    debugger session that loads fresh, your colleague's board.
> 2. **It moves when the program changes.** Add one unrelated variable and
>    every `.bss` object shifts to a different address, landing on different
>    leftover bytes. The symptom vanishes or changes, which reads as "the fix
>    worked".
> 3. **It depends on how you reset.** Power-cycle and warm-reset give different
>    garbage, so "it only fails when I press the button" is a real report.

> If you want to *see* it fail, you need RAM with something in it: run the
> normal firmware first so it fills `.bss` with a running counter, then
> reset without removing power. That is exercise territory rather than a step
> here, because Wokwi's restart does not reliably preserve RAM.

**Put the line back.**

---

## Part 3 — Put the vector table in the wrong place (25 min)

**This is the most important part of the lab.** Everything so far broke
something *inside* your program. This breaks the program's ability to start at
all — and every tool you have will tell you it is fine.

In `link.ld` the vector table is the **first** output section, so it lands at
`0x08000000`, where slide 5 showed the hardware looking.

### Do this

Cut this line out of `SECTIONS`:

```
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH
```

and paste it back **after** `.text`, just above `.rodata`. Change nothing else
— same `KEEP()`, same `>FLASH`.

**Guess first:** error, warning, or clean build?

### What you see

> ```
>            FLASH:        6736 B        32 KB     20.56%
>              RAM:        2008 B        12 KB     16.34%
> Build OK: Main.elf, Main.hex and Main.bin created
> ```
>
> **Zero errors. Zero warnings. Identical sizes.** The linker did exactly what
> you asked, and it had no reason to object.

Now look at where things actually landed:

```
..\..\tools\arm-toolchain\bin\arm-none-eabi-size.exe -A Main.elf
```

> ```
> .text                 5244   134217728     <- 0x08000000
> .isr_vector            180   134222972     <- 0x0800147C
> ```
>
> `.text` is now at the base of FLASH, and the vector table has been pushed
> 5244 bytes up.

Read the first bytes of the image, the way slide 7 did — open `Main.hex` and
look at the second line:

> ```
> :10000000002243088B4274D303098B425FD3030A57
> ```
>
> Little-endian, four bytes at a time:
>
> | Bytes | Value | The hardware treats it as |
> |---|---|---|
> | `00 22 43 08` | `0x08432200` | the initial stack pointer |
> | `8B 42 74 D3` | `0xD374428B` | the reset vector |
>
> **Those are not addresses. They are machine code.** They are the first
> instructions of `.text`, which now occupies the two words the core reads on
> reset.

### Why the chip cannot start

> - **`0x08432200` is not a valid stack pointer.** Your RAM is
>   `0x20000000`–`0x20002FFF`. That value points into FLASH, which is not
>   writable — the first thing that pushes to the stack fails.
> - **`0xD374428B` is not a valid reset vector.** It is nowhere near the
>   `0x08…` FLASH region, so the core fetches from unmapped memory.
> - On Cortex-M there is no handler installed to catch this yet, so a fault
>   this early becomes a **lockup**: the core halts and nothing runs at all.

### The point of the whole exercise

> This firmware **compiled with no errors, linked with no warnings, produced a
> structurally valid Intel HEX file with correct checksums, and reports sensible
> section sizes**. Every automated check in this repository passes on it. It
> cannot run.
>
> What *would* have caught it? Two checks, neither of which the compiler or
> linker makes:
>
> 1. **Assert the vector table is at the start of FLASH** — compare
>    `.isr_vector`'s address against `ORIGIN(FLASH)` after linking. The linker
>    will not do this because placing sections wherever you say is its *job*,
>    not a mistake.
> 2. **Assert word 1 of the image is a plausible reset vector** — inside FLASH,
>    and odd (the Thumb bit). Cheap to check, and it would have failed loudly.
>
> The compiler checks your *language*. The linker checks your *arithmetic*.
> **Neither checks your intent**, and nothing in the toolchain knows what a
> vector table is for.
>
> This course has a rule about exactly this — *never trust a build's exit code
> alone.* You have just manufactured the reason.

**Restore `link.ld`** (`.isr_vector` first, with `KEEP()`), rebuild, and confirm
the first data record is back to `003000200D050008...`.

### Optional: two one-line variations

> **Remove `const`** from `void (* const vectors[])` in `startup.c`. It builds,
> but now warns:
> `Main.elf has a LOAD segment with RWX permissions`. Making the table writable
> marks the whole FLASH segment readable-writable-executable. The table still
> works; you have just given up a protection for nothing.
>
> **Remove only the `KEEP()`**, leaving the section first. It still works — on
> *this* toolchain, today. `KEEP()` protects the section from `--gc-sections`,
> which deletes anything nothing references, and **nothing in C references the
> vector table** — only the hardware does. That it survives here is luck about
> how this linker roots its scan, not a guarantee. Keep the `KEEP()`.

---

## Part 4 — Change the memory map (25 min)

The C031**C6** has 12 KB of RAM. What if it had less?

### Step 1 — 8 KB

In `link.ld`, change the RAM line only:

```
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH = 8K
```

**Guess first:** which of the six printed values move?

> **Exactly one: the stack top.**
>
> | Value | 12 KB | 8 KB | |
> |---|---|---|---|
> | vector table | `0x08000000` | `0x08000000` | unchanged — it is in FLASH |
> | `.data in FLASH` (`_sidata`) | `0x080019EC` | `0x080019EC` | unchanged — FLASH |
> | `.data in RAM` | `0x20000000` | `0x20000000` | unchanged |
> | `.bss` | `0x20000064` | `0x20000064` | unchanged |
> | heap start (`end`) | `0x200001D8` | `0x200001D8` | unchanged |
> | **stack top (`_estack`)** | **`0x20003000`** | **`0x20002000`** | **moved** |
>
> Everything is placed from the **bottom** of RAM upward, so shrinking the
> region cannot disturb it. The stack is the only thing measured from the
> **top**, because it grows downward — so it is the only thing that moves.

**Trace how that reaches the firmware**, because nothing in your C mentions
`_estack`:

> 1. `link.ld` computes `_estack = ORIGIN(RAM) + LENGTH(RAM)` = `0x20002000`.
> 2. `startup.c` puts `&_estack` in `vectors[0]`.
> 3. `.isr_vector` is placed at `0x08000000`, so `vectors[0]` is the first word
>    of the image.
> 4. `objcopy` writes that word into `Main.hex` — its first data record now
>    starts `00200020` instead of `00300020`.
> 5. On reset the **hardware** loads it into SP.
>
> One number in a text file becomes the stack pointer, with no code involved
> anywhere along the way.

### Step 2 — 2 KB

```
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH = 2K
```

> **It builds.**
>
> ```
>              RAM:        2008 B         2 KB     98.05%
> ```
>
> 2008 of 2048 bytes, and the linker is content. **Is this program safe to
> run?** No — and the linker cannot tell you so.
>
> Look at `link.ld`:
>
> ```
> _Min_Heap_Size  = 0x200;    /*  512 bytes */
> _Min_Stack_Size = 0x400;    /* 1024 bytes */
> ```
>
> Those are a **reservation**, not a limit. They make the link *fail* if 1536
> bytes of headroom will not fit. They do **not** stop the stack growing past
> 1024 bytes at run time. Nothing does. The stack simply keeps growing
> downward, through the heap, into `.bss`, quietly corrupting your variables.
>
> A deep call chain, a big local array, or one unlucky interrupt arriving at
> the wrong moment, and you are overwriting `.data` with return addresses.

### Step 3 — 1 KB

> **Now it fails**, and the message is precise:
>
> ```
> Main.elf section `._user_heap_stack' will not fit in region `RAM'
> region `RAM' overflowed by 984 bytes
>              RAM:        2008 B         1 KB    196.09%
> ```
>
> Check the arithmetic against slide 15: you need 2008 bytes and have 1024, so
> you are over by `2008 − 1024 = 984`. Exactly what it says.
>
> It names `._user_heap_stack` because that is the section the location counter
> was inside when it ran off the end — `.data` and `.bss` were placed first and
> fitted.

### The rule worth taking away

> **A linker checks what it can compute; it cannot check what happens later.**
>
> Sizes of sections are known at link time, so overflow is caught exactly. How
> deep your call stack gets, how much you `malloc`, whether an interrupt nests —
> all run-time behaviour, all invisible to the linker. Step 3 was caught. Step 2
> is arguably more dangerous *because* it was not.

**Restore `LENGTH = 12K`**, rebuild, and confirm 2008 B / 16.34%.

---

## Part 5 — What your data costs (20 min)

Add a 64-entry array to `Main.c` and watch which section it lands in. Do all
three; each is one line, and the differences are the lesson.

Put the declaration near the top, and print one element in `main()` so the
compiler cannot discard it:

```c
/* (a) */ static const volatile uint32_t tbl[64] = {1,2,3,[63]=99};
/* (b) */ static       volatile uint32_t tbl[64] = {1,2,3,[63]=99};
/* (c) */ static       volatile uint32_t tbl[64];
```

Measure each with:

```
..\..\tools\arm-toolchain\bin\arm-none-eabi-size.exe -A Main.elf
```

### What you get

> Baseline is `.rodata 1204`, `.data 100`, `.bss 372` — FLASH 6736, RAM 472.
> A 64-entry `uint32_t` array is 256 bytes.
>
> | | `.rodata` | `.data` | `.bss` | FLASH | RAM |
> |---|---|---|---|---|---|
> | baseline | 1204 | 100 | 372 | 6736 | 472 |
> | **(a) `const`** | **1468** | 100 | 372 | **+280** | **+0** |
> | **(b) initialised** | 1212 | **356** | 372 | **+280** | **+256** |
> | **(c) uninitialised** | 1212 | 100 | **628** | **+24** | **+256** |
>
> - **(a) `const` → `.rodata`.** Lives in FLASH and is read in place. Costs
>   flash only. **Zero RAM.**
> - **(b) initialised, writable → `.data`.** Costs **both**: 256 bytes of RAM
>   to live in, *and* 256 bytes of FLASH holding the master copy that
>   `Reset_Handler` copies out. This is slide 15's "counted twice", now yours.
> - **(c) uninitialised → `.bss`.** Costs RAM only. FLASH barely moves, because
>   only the *size* is recorded — there are no zeros to store.

### One more thing worth seeing

> Try (a) or (c) **without** `volatile`. The array disappears from the size
> report entirely.
>
> At `-Os` the compiler can see that you only ever read one element whose value
> it already knows, so it folds the read into a constant and deletes the array.
> That is why the declarations above carry `volatile` — it forces the storage
> to exist so you can measure it.
>
> It is also a preview of lesson 01's warning: the optimiser removes anything
> it can prove you do not need, and `volatile` is how you tell it that it
> cannot prove that.

### The rule of thumb

> On a 32 KB / 12 KB part, **RAM is the scarce one** — you have less than half
> as much, and the stack is eating 1.5 KB of it before you start.
>
> So: **mark it `const` unless you will write to it.** Lookup tables, strings,
> menu text, font data, calibration constants — all of it can sit in FLASH and
> cost you no RAM at all. On this part that is not a style preference; it is the
> difference between fitting and not fitting.

---

## Where to go next

Nothing here is handed in. If you want to push further:

- Disassemble the copy loop you deleted in Part 1 and compare it with slide 11:
  `python ..\_build\disasm.py Main.elf -f Reset_Handler`
- Dump the vector table as bytes and decode more of it than slide 7 did:
  `..\..\tools\arm-toolchain\bin\arm-none-eabi-objdump.exe -s -j .isr_vector Main.elf`
- Find `Reset_Handler` in `Main.map` and confirm its address matches word 1 of
  the hex, minus the Thumb bit.

---

## Troubleshooting

| Symptom | Almost certainly |
|---|---|
| `build.bat` says "Main.c not found" | You are not in the lesson folder. |
| `no input files` | You edited the engine or a target file. Check `_targets/c031c6.bat` sets `MCUFLAGS` and `DEVINC`. |
| Serial monitor is blank | You uploaded `Main.bin` instead of `Main.elf`/`Main.hex`, or the simulation is not running. |
| `SystemCoreClock : 12000000` | The `RCC->CR &= ~RCC_CR_HSIDIV` write did not take. Everything still works, at a quarter speed. |
| LED dead but serial fine | You are still in Part 0 Step 5 — put the `RCC->IOPENR` line back. |
| A change had no effect | You are looking at an old binary. Re-upload after every build; `simulate.bat` always rebuilds. |
