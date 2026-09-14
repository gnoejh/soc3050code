# SOC3050 — 2026 ARM edition (STM32)

The live edition. `projects2026_avr/` is the finished 2026 AVR edition and is
**frozen**; the two trees are independent and neither imports from the other.

## What is here now

```
_build/build-slides.py    the renderer (this tree's own copy)
_slides/                  generated decks - do not edit, re-render instead
_notebooks/               the Colab workbench, one notebook for the course
00_Architecture/          Part 0 - the programmer's model
01_Instruction_Set/       Part 0 - Thumb vs ARM, what the CPU does
02_Development/           Part 0 - toolchain, sections, linking, boot
03_Execution_Concurrency/ Part 0 - interrupts, volatile, atomicity
_spike/                   Phase 0 proof of concept - throwaway, see below
```

## The slides

Read them at `_slides/index.html`, or online once pushed:
**https://gnoejh.github.io/soc3050code/**

That is the site root. This edition took it over from the AVR edition on
2026-09-14; `.github/workflows/pages.yml` now renders and publishes this tree
alone, and the AVR decks are no longer online anywhere.

Arrow keys move, `o` gives an overview grid, `p` prints or saves as PDF.

Re-render after editing any `Slide.md`:

```
python projects2026_arm/_build/build-slides.py
```

`_slides/` is generated. Edit the `Slide.md`, never the HTML.

## The Colab notebook

Every deck's top bar carries an **Open in Colab** link, next to the reference
manual. It opens one shared notebook, `_notebooks/SOC3050_ARM.ipynb`, straight
from GitHub - Colab renders any notebook GitHub can serve, so the URL is just
this repository's path with `colab.research.google.com/github/` in front.

The notebook gives a student with no toolchain the **build half** of the course:
it `apt-get`s a real `arm-none-eabi-gcc`, sparse-clones this tree and the CMSIS
headers, builds the C031C6 target, and then walks the Part 0 material against
real output - sections and the memory map, the vector table read out of
`Main.bin`, Thumb-2 disassembly, and the `volatile` and read-modify-write
experiments from lesson 03.

It cannot flash a board or run the firmware; Colab has no USB, and Renode and
Wokwi stay local steps. That boundary is stated on the notebook's first screen
so nobody goes looking for a blinking LED.

The link is one constant, `COLAB`, at the top of `build-slides.py` - **change it
there and re-render**, rather than editing the decks. The notebook itself is
maintained by hand; it is not generated from anything.

### Writing diagrams

This renderer passes a ` ```svg ` fenced block through verbatim into a
`<figure class="diagram">`. Ordinary code fences still escape as usual.

Diagrams must work in both the light and dark themes, so **use `currentColor`
and the CSS variables, never hardcoded colours**. Helper classes:

| class | use |
|---|---|
| `.box` | filled panel with border |
| `.reg` | outlined cell — a register, an address |
| `.wire` / `.dash` | connector / muted dashed line |
| `.hi` `.hifill` | accent outline / tinted fill — *the thing being taught* |
| `.ok` | correct-path highlight |
| `.mono` `.lbl` | monospace / small muted caption |

Give every `<svg>` a `viewBox` (it scales; fixed width/height does not) and a
`role="img"` with an `aria-label`.

## Curriculum shape: models before code

**Part 0 (lessons 00–03) is theory. No `Main.c`, no build, no simulator.**
It teaches one model — the programmer's model, the instruction set, the
toolchain, and concurrency — so that the peripheral lessons that follow are
*instances* of it rather than twenty-five separate things to memorise.

Part 0 slides quote **real output from this repository's own build** (real
disassembly, real section sizes, a real Intel HEX record decoded field by
field). That is evidence, not student coding.

**No toolchain is vendored yet, so that promise has to be checkable without
one.** Everything a Part 0 deck quotes must be reproducible from the committed
spike artefacts - `_spike/c031c6/Main.bin`, `Main.hex` and `Main.map` - by
reading bytes and symbols out of them. Lesson 01 had one row that was not:
a 32-bit encoding `f000 f812` that appears nowhere in the image. It is now
`f7ff ffcf`, the `bl SystemInit` at `0x0800028a`, decoded from `Main.bin` and
checked against the symbol addresses in `Main.map`. **Check a quoted encoding
against the image before adding it**, the same way the AVR edition checks a
hex file rather than trusting that a build ran.

From Part 1 onward every lesson has code and follows three beats:

> **Model → Map → Measure**
> — the abstract idea, drawn; mapped onto this chip's registers by reading the
> reference manual; then written, run and *observed*.

## The `_spike/` directory

Phase 0 proof of concept. **Not lessons** — delete or promote when the build
engine lands. It exists because the plan's riskiest assumptions needed testing
before any lesson was written. All six passed; see `_spike/FINDINGS.md`.

```
_spike/c031c6/   Nucleo C031C6 (Cortex-M0+)  - builds, runs in Wokwi
_spike/l031k6/   Nucleo L031K6 (Cortex-M0+)  - builds, runs in Renode
_spike/f103c8/   Blue Pill F103C8 (Cortex-M3)- builds, runs in Renode
_spike/f446re/   Nucleo F446RE (Cortex-M4F)  - builds, runs in Renode
```

Each carries the three files that replace what avr-libc did for free:
`startup.c` (vector table, reset handler, `.data`/`.bss` init, clock),
`link.ld` (memory map), and a `main.c` that blinks and prints over USART2.

## Building

**Nothing is vendored yet.** The toolchain, CMSIS headers and Renode currently
live in a scratch directory; moving them into `tools/` is Phase 1 work. Until
then `_spike/README.md` has the exact commands and the versions used.

> **This tree cannot repeat the AVR edition's "a clone needs no installs"
> promise.** The student simulator is hosted. Say so plainly rather than
> discovering it in week one.

## Simulation, and what it costs

| Route | Licence | Cost | Renewal |
|---|---|---|---|
| wokwi.com in a browser, loading a locally built ELF | **none** | **free** | never |
| Wokwi VS Code extension | personal key | free | every 30 days |
| Renode, offline | none | free | never |

**Wokwi is a simulator only — it never compiles anything for this course.**
Students build locally with `arm-none-eabi-gcc`, then load `Main.elf` in the
browser via `F1 → Upload Firmware and Start Simulation…`. That works for STM32
and is documented nowhere except `_spike/FINDINGS.md`; Wokwi's own docs
describe the feature only under ESP32.

The maintainer licence buys one thing: the simulator appearing inside VS Code
instead of a browser tab. **Students pay nothing and need no account.**

## Open decisions

- **Target A is not chosen.** C031C6 (more pins, more RAM, modern `MODER`
  GPIO, Wokwi-only) versus F103C8 (native Renode support and therefore a free
  offline loop, but STM32F1's `CRL`/`CRH` GPIO is unlike every later family).
  Part 0 is deliberately target-neutral, so the choice only bites at the first
  GPIO lesson.
- **Target B** for the application track is the F446RE under Renode.

## Conventions

- `_slides/` is generated. Re-run the renderer; never edit the HTML.
- **A `---` line is what starts a slide.** A heading alone does not; two
  sections merge silently and the deck still renders, so check the slide count
  the renderer prints after editing. Lesson 01 shipped with slides 9 and 10
  fused this way.
- **Never touch `projects2026_avr/`.** `git status projects2026_avr` should
  always be empty.
- **Never rely on a backslash escape surviving a heredoc** when generating
  files — use `chr(10)` rather than a newline escape. This has bitten this
  repository three times; see `CLAUDE.md` §9a, §9e and `_spike/FINDINGS.md`.
- Per `CLAUDE.md` §11: never trust a build's exit code alone. A lesson is not
  done until its output has been *watched*.
