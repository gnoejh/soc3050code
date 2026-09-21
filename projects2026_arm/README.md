# SOC3050 — 2026 ARM edition (STM32)

The live edition. `projects2026_avr/` is the finished 2026 AVR edition and is
**frozen**; the two trees are independent and neither imports from the other.

## What is here now

```
_build/build-lesson.bat   the build engine - lessons call it, nobody runs it
_build/build-slides.py    the renderer (this tree's own copy)
_build/disasm.py          disassemble an ELF without the toolchain
_build/wokwi-pins.py      renders the board pin map (below) from the facts file
_targets/c031c6.bat       per-chip flags, include paths and memory sizes
_targets/c031c6-pins.json every pin name Wokwi accepts for the board -> MCU pin
_slides/                  generated decks + board-pins.html - do not edit, re-render instead
_docs/                    the board manual (ST UM2953), copied beside the decks on render
_notebooks/               the Colab workbench, one notebook for the course
00_Architecture/          Part 0 - the programmer's model
01_Instruction_Set/       Part 0 - Thumb vs ARM, what the CPU does
02_Development/           Part 0 - toolchain, sections, linking, boot
03_Execution_Concurrency/ Part 0 - interrupts, volatile, atomicity
04_Startup_And_LinkerScript/  Part 1 - the first lesson with code
_spike/                   Phase 0 proof of concept - throwaway, see below
```

**Every lesson folder carries both a `Slide.md` (the lecture) and, from
lesson 04 onward, a `Lab.md` (the lab handout), so one week's teaching is one
directory.** Part 0's four lessons are theory and have no `Lab.md`.

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

## The board pin map

Every deck's top bar carries a **Board pins** link to `_slides/board-pins.html`:
one table of every string Wokwi accepts on the board end of a `diagram.json`
wire, and the MCU pin or rail each one reaches.

It exists because those strings are visible nowhere a student would look. The
board image does not print them, the editor does not list them, and they are
not the datasheet names: plain `PB0` is rejected, `PB0.1`, `PB0.2` and `D10`
all work and all reach PB0, and `GND` is really `GND.1` to `GND.9`. Lesson 04's
`diagram.json` hit exactly this, and a student adding one LED would have had
no way to find out why.

Two files, one derived from the other:

- `_targets/c031c6-pins.json` — the facts, committed. Derived from Wokwi's
  board definition (`wokwi/wokwi-boards`, `boards/st-nucleo-c031c6/board.json`)
  by `python _build/wokwi-pins.py --refresh`. That repository declares no
  licence, so the board file itself is **not** vendored; only the name-to-pin
  facts are, in our own format.
- `_slides/board-pins.html` — the page. `build-slides.py` renders it on every
  run, so it cannot be stale relative to the facts, and the Pages workflow
  verifies it is present. The notes column (pins the course reserves, board-file
  quirks such as `PD3` mapping to the 3.3 V rail) is the `NOTES` table in the
  script, maintained by hand.

If Wokwi renames a pin, run `--refresh` and re-render; do not edit the page.

The top bar also carries the real board's manual, **ST UM2953, STM32 Nucleo-64
boards (MB1717)**: header pinouts, solder bridges, schematics. The pin-name
page says what Wokwi calls a pin; the manual says what it is.

The PDF lives in the tree, at `_docs/UM2953_Nucleo64_MB1717.pdf`, and
`build-slides.py` copies it beside the decks on every render, so the link is
relative and works from a clone and from the site without st.com having to
answer (it did not, from the machine that set this up). The renderer refuses
to run if the file is missing. The copy in `_slides/` is git-ignored — one
committed copy, in `_docs/`, is enough — and ST's own URL is offered on the
index page as a fallback. The constants are at the top of `build-slides.py`.

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

**Everything a Part 0 deck quotes must be reproducible from the image**, by
reading bytes and symbols out of `_spike/c031c6/Main.elf`, `Main.bin`,
`Main.hex` and `Main.map`. Lesson 01 had one row that was not: a 32-bit
encoding `f000 f812` that appears nowhere in the image. It is now `f7ff ffcf`,
the `bl SystemInit` at `0x0800028a`, checked against the symbol addresses in
`Main.map`. **Check a quoted encoding against the image before adding it**, the
same way the AVR edition checks a hex file rather than trusting that a build
ran.

> **Those artefacts are committed** (2026-09-19). They were described as
> committed long before they were: `.gitignore` excluded
> `projects2026_arm/**/Main.elf` and friends, so a fresh clone had none of
> them, every encoding above was uncheckable there, and the documented
> `disasm.py _spike/c031c6/Main.elf` example could not run. The ignore rule now
> carries an explicit exception for `_spike/`, which is a record rather than a
> lesson. ~2 MB.

With the toolchain vendored you can also just regenerate them — `_spike/README.md`
has the exact command, and it reproduces the committed figures (FLASH 5260 B,
RAM 2000 B) section for section.

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

**The toolchain is vendored** (2026-09-19). A clone compiles with no installs:

```
tools/arm-toolchain/   xPack arm-none-eabi-gcc 15.2.1-1.1, pruned to 227 MB
tools/cmsis/           CMSIS 6 Core headers + ST's STM32C0xx device headers
                       (CMSIS = Common Microcontroller Software Interface
                       Standard, ARM's vendor-neutral Cortex-M header set)
```

The prune keeps three multilibs — `thumb/v6-m/nofp` (M0+, the C031C6 and
L031K6), `thumb/v7-m/nofp` (M3, the F103C8) and `thumb/v7e-m+fp/hard`
(M4F, the F446RE) — so all four candidate targets still link. It drops the
other 36, C++, Fortran, LTO and the bundled Python. `arm-none-eabi-gdb.exe` is
kept deliberately: both Wokwi and Renode expose a GDB server, and the AVR tree
never had a source-level debugger at all.

The archive's sha256 was verified against xPack's published `.sha` before
unpacking, and the vendored compiler was confirmed to rebuild
`_spike/c031c6/` to **byte-identical** figures (FLASH 5260 B, RAM 2000 B) —
the same numbers `_spike/FINDINGS.md` recorded under A2.

To build a lesson, run `build.bat` from inside its folder. The engine compiles
**every `.c` in that folder**, so a lesson is self-sufficient by construction
with no source list to maintain:

```bat
set TARGET=c031c6
set LIBS=
call "%~dp0..\_build\build-lesson.bat"
```

A lesson that carries its own `startup.c` and `link.ld` uses them; otherwise
the engine falls back to the target's defaults.

> **The simulator is still hosted.** The compiler ships in the repository, but
> Wokwi is a web page. That is the one part of the AVR edition's "a clone needs
> nothing" promise this tree cannot repeat — and students still pay nothing.

### The `.cfg` trap, recorded so it is not repeated

`_targets/c031c6.bat` was first written as `c031c6.cfg`. **CMD's `call`
silently does nothing on an unknown extension** — no error, no exit code. Every
variable it should have set stayed empty, and an empty `DEVINC` made the
include flag end `-I"...\cmsis\"`, where the trailing backslash escaped the
closing quote and swallowed the entire source list. gcc reported *"no input
files"*, naming nothing that pointed at the real cause.

Target files are therefore `.bat`, and the engine now refuses to build if
`MCUFLAGS` or `DEVINC` came back empty.

## Seeing the assembly

Every Part 0 deck quotes disassembly, and until the toolchain is vendored
nobody with a fresh clone can reproduce it. `_build/disasm.py` closes that gap
by reading the ELF itself:

```
pip install capstone
python _build/disasm.py _spike/c031c6/Main.elf -f _write
python _build/disasm.py _spike/c031c6/Main.elf            # the whole image
python _build/disasm.py _spike/c031c6/Main.hex            # no symbols - see below
```

capstone is a 3 MB wheel rather than a 335 MB toolchain download. The script
follows the ELF's `$t` / `$d` **mapping symbols**, so literal pools print as
`.word` instead of being rendered as invented instructions, and it labels
branch targets with the function they land in. Its output for `_write` was
compared line by line with lesson 01 slide 14 - 23 instructions, every encoding
and every mnemonic identical.

A `.hex` or `.bin` carries neither symbols nor mapping symbols, so it is
disassembled as one long Thumb run and the vector table at the front comes out
as nonsense. That is worth showing a class exactly once: it is what "the ELF is
not just the bytes" means.

**`arm-none-eabi-objdump -d` remains the real answer** - richer output, source
interleaving with `-S`, and no second implementation to trust. The Colab
notebook already runs it. This is the offline stopgap, not a replacement.

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

## Decisions

- **Target A is the STM32C031C6** (decided 2026-09-19, at lesson 04 — the
  first lesson with code, which is where the choice finally bites). Three
  things settled it: Wokwi hosts it and the browser-upload route is proven
  free (`_spike/FINDINGS.md`, A1b); its `MODER` GPIO and one-write 48 MHz clock
  match every modern STM32 family, where the F103C8's `CRL`/`CRH` matches none;
  and all four Part 0 decks already quote RM0490, the STM32C0x1 reference
  manual, so a different part would contradict four weeks of taught slides.

  What was given up: the F103C8 would have run offline in Renode with no
  dependence on a hosted service staying free. That risk is accepted, and
  `_startup/` is structured per-target so a port is a new `_targets/*.bat`
  plus a startup file, not a rewrite.
- **Target B** for the application track is the F446RE under Renode. Its
  multilib (`thumb/v7e-m+fp/hard`) is kept in the vendored prune, so nothing
  blocks it.

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
