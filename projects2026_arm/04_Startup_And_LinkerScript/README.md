# 04 — Startup and the Linker Script

**Part 1, lesson 1 — the first lesson with code.**
Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz).

Part 0 (lessons 00–03) taught four models with no code. This lesson builds the
thing lesson 02 described: the vector table, the reset handler, `.data`/`.bss`
initialisation and the clock setup — and then takes them apart.

## Files

| | |
|---|---|
| `Slide.md` | the lecture, 20 slides — rendered to `_slides/04_Startup_And_LinkerScript.html` |
| `Lab.md` | the lab: seven parts, ~2 hours, **hands-on with the explanation inline** — not assessed, nothing handed in |
| `Main.c` | the lab program — prints a memory report measured from linker symbols, then drives an eight-LED bar on PB0–PB7 from pattern tables **students edit** |
| `startup.c` | **the subject of the lesson.** Vector table, `Reset_Handler`, `SystemInit()` |
| `link.ld` | **the subject of the lesson.** Memory map and section placement |
| `retarget.c` | infrastructure: `_write()` and friends, so `printf` reaches USART2 |
| `build.bat` | one command, no arguments |
| `simulate.bat` | always rebuilds, then opens the Wokwi board |
| `diagram.json` | the board plus the eight LEDs on PB0–PB7. The browser route needs it pasted into the Wokwi tab's `diagram.json`; the VS Code extension reads it directly. Pin names are Wokwi's, not the datasheet's — `led0` sits on `nucleo:PB0.1` because no plain `PB0` exists; see `_slides/board-pins.html`, linked from every slide's top bar |
| `wokwi.toml` | for the Wokwi VS Code extension only |

**This lesson is self-sufficient.** It links nothing from `_lib/`, and it
carries its own `startup.c` and `link.ld` rather than sharing the target's
defaults — because in this lesson those two files are what the student edits.

## Build and run

```
build.bat        # compiles every .c in this folder, links with link.ld
simulate.bat     # rebuilds, then opens wokwi.com on the Nucleo-C031C6
```

In the Wokwi tab: paste this folder's `diagram.json` into the `diagram.json`
tab (that adds the eight LEDs), then `F1` → **Upload Firmware and Start
Simulation…** → pick `Main.elf`. Free, no account, no licence.

Expected build output:

```
FLASH:  7624 B  /  32 KB   23.27%
RAM:    2008 B  /  12 KB   16.34%
```

Zero warnings. The slides quote those figures and the hex's first data record
(`:1000000000300020C50600087906000879060008BF`) literally, so **if `Main.c`,
`startup.c` or `link.ld` changes, re-derive slides 7, 15 and 19** rather than
letting them drift.

## The memory and I/O maps

Slides 3 and 4 give the address space **with this part's real numbers** before
any code is read — lesson 00 covered the Cortex-M regions in the abstract, and
this is where they get concrete. Students meeting ARM for the first time
otherwise see `0x08000000`, `0x20000000` and `GPIOA->MODER` with no picture to
hang them on.

Slide 3 is the memory map (FLASH 32 KB at `0x08000000`, SRAM 12 KB at
`0x20000000`, peripherals, SCS) and makes the point that `link.ld`'s `MEMORY`
block is two rows of it. Slide 4 is the I/O map: `GPIOA->ODR` resolving to
`0x50000014` as base + offset, with a table of every register this program
touches.

**Every address on both slides is compiler-verified.** They were checked with
`_Static_assert((uintptr_t)&GPIOA->ODR == 0x50000014UL, ...)` and 19 similar
assertions against ST's CMSIS header; the file compiles, so the numbers hold.
Re-run that check if the target ever changes — the script is trivial to
recreate and the alternative is slides that quietly drift from the header.

One fact worth flagging for anyone porting these slides: **on the STM32C0 the
GPIO ports live at `IOPORT_BASE = 0x50000000`, not in the `0x40000000`
peripheral region** where the F1 and F4 families put them. Slide 3 calls this
out explicitly, because example code copied from an F4 tutorial gets it wrong
silently.

## Registers and peripherals touched

| | |
|---|---|
| `RCC->CR` | `HSIDIV` cleared — 12 MHz → 48 MHz, in `SystemInit()` |
| `FLASH->ACR` | one wait state, set *before* the clock speeds up |
| `RCC->IOPENR` | GPIOA and GPIOB clock gates — without one the port reads back zero |
| `RCC->APBENR1` | USART2 clock gate |
| `GPIOA->MODER`, `ODR`, `AFR[0]` | PA5 output (LD4 heartbeat), PA2/PA3 alternate function 1 (USART2) |
| `GPIOB->MODER`, `ODR` | PB0–PB7 outputs — the LED bar; one byte written per frame |
| `SysTick->LOAD`, `VAL`, `CTRL` | polled 1 ms tick for `delay_ms()`; no interrupt |
| `USART2->BRR`, `CR1`, `ISR`, `TDR` | 115200 baud, polled transmit |

`EXTI` and `NVIC` are **not** used here, and `SysTick` is only polled —
lesson 04 has no interrupts at all. The vector table is full of handlers that never fire, which
is the point: it exists for the hardware to read on reset, not for this program
to use.

## What the lab does

It is a **guided walkthrough, not a quiz.** Each step is: do this → guess →
*here is what happens and why*. The explanation sits inline, in blockquotes,
immediately after the step. Nothing is collected and nothing is marked; the
guess exists only because a surprise sticks better than a paragraph.

| Part | Change | What it demonstrates |
|---|---|---|
| 0 | build, run, then delete one `RCC->IOPENR` write | a write to an unclocked peripheral vanishes silently — **UART works, LED dies** |
| 1 | delete the `.data` copy loop | initialised globals are a promise somebody has to keep |
| 2 | delete the `.bss` zero loop | and so is zero-init — invisibly, in a simulator |
| 3 | **move `.isr_vector` after `.text`** | firmware that passes every check and cannot start |
| 4 | shrink RAM: 12K → 8K → 2K → 1K | what a linker can and cannot check for you |
| 5 | add an array, `const` or not | `.rodata` vs `.data` vs `.bss`, measured |
| 6 | **edit the LED patterns** — tables, timings, playlist, or a computed pattern | the half of `Main.c` that is theirs; open-ended, brought to the next class |

**Every figure quoted in the answers was measured, not reasoned about:**

- Part 0 step 5 works because `led_init()` runs *before* `uart2_init()`, which
  re-enables the same gate — verified by reading the call order.
- Part 3: moving the section builds clean with **zero warnings**; `.text` lands
  at `0x08000000` and `.isr_vector` at `0x08001634`, so the reset words become
  `SP = 0x08432200`, `PC = 0xD374428B` — instruction bytes. Removing `KEEP()`,
  the *obvious* exercise, **does not** delete the table on this toolchain, so
  it demonstrates nothing and is demoted to an optional aside.
- Part 4: 8 KB moves **only `_estack`** (`0x20003000` → `0x20002000`) — nothing
  else, because everything but the stack is placed bottom-up. 2 KB **builds**
  at 98.05%. 1 KB fails: `._user_heap_stack will not fit`, `overflowed by 984
  bytes`, 196.09%.
- Part 5: `const` +288 FLASH / **+0 RAM**; initialised +288 FLASH / +256 RAM
  (the double cost); uninitialised +32 FLASH / +256 RAM. (32 of each figure
  is the `printf` that keeps the array alive.) The declarations carry
  `volatile` because without it `-Os` deletes the array outright — itself worth
  showing, and the lab does.

**Two runtime outcomes are predicted rather than observed**, because nobody has
run this in Wokwi yet: what `data_witness` reads with the copy loop gone
(Part 1) and whether Part 2 shows any symptom at all. Both are written to be
correct either way — Part 1 says "almost certainly zero, and if yours differs
that is the more honest answer", and Part 2's whole point is that a simulator
with clean RAM hides the bug completely.

## Status

- Builds clean, zero warnings, with the vendored `tools/arm-toolchain/`.
- `Main.hex` validated: 484 records, all checksums correct, extended
  linear address record `:020000040800F2` present, entry point `0x080006c5`
  (odd — the Thumb bit).
- Every exercise in `Lab.md` has been built and its stated outcome checked —
  re-done on 2026-09-22 after the LED bar was added, since `Main.c` growing
  moved every FLASH figure. RAM figures did not move: the pattern tables are
  `const`, and `frame_count` replaced `blink_count` byte for byte.
- **Not yet watched running in Wokwi.** Per `CLAUDE.md` §9e that means the
  lesson is not finished. The startup, UART and PA5 half is byte-comparable
  to the Phase 0 spike that *was* seen running (`_spike/FINDINGS.md`, A1b),
  which is evidence and not proof. The LED bar half — GPIOB, the polled
  SysTick delay, and the eight `wokwi-led` parts in `diagram.json` — has no
  such precedent and has been checked only by building. First thing to do
  when a browser is at hand: paste the diagram, upload, and watch.
