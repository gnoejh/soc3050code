# Phase 0 spike — findings

Working log for the assumptions listed in the plan. Nothing here is a lesson;
this whole directory is deleted or promoted when Phase 0 closes.

**PHASE 0 IS CLOSED (2026-09-12). All six assumptions resolved, none fatal.**

| | Assumption | Status |
|---|---|---|
| A1 | Students can run a locally built ELF in Wokwi without paying | **RESOLVED - YES, free** |
| A2 | Hand-written startup.c + link.ld + 48 MHz init prints over USART2 | **RESOLVED - both targets** |
| A3 | Renode portable runs here and boots an ELF on a derived .repl | **RESOLVED - YES** |
| A4 | A pruned arm-none-eabi toolchain fits in <=350 MB | **RESOLVED - 247 MB** |
| A5 | resolve-libs.py does not trip over weak *_IRQHandler aliases | **RESOLVED - YES** |
| A6 | STM32C0 EXTI port select is in EXTI->EXTICR[], not SYSCFG | **RESOLVED - YES** |
| A1b | wokwi.com **web** can load a locally built hex for STM32 | **RESOLVED - YES** |

## A6 — RESOLVED, favourable (2026-09-12)

Lesson 06 `EXTI_Interrupts` has a home. Wokwi lists SYSCFG as not implemented,
which would have been fatal if SYSCFG owned the external-interrupt port
selection the way it does on STM32F4. On STM32C0 it does not.

From ST's own CMSIS header, `STMicroelectronics/cmsis-device-c0`,
`Include/stm32c031xx.h`:

    typedef struct {
      __IO uint32_t RTSR1;    __IO uint32_t FTSR1;
      __IO uint32_t SWIER1;   __IO uint32_t RPR1;
      __IO uint32_t FPR1;     /* ... reserved ... */
      __IO uint32_t EXTICR[4];   /* <-- port selection lives HERE */
      /* ... */
      __IO uint32_t IMR1;     __IO uint32_t EMR1;
    } EXTI_TypeDef;

    typedef struct {
      __IO uint32_t CFGR1;  __IO uint32_t CFGR2;  __IO uint32_t CFGR3;
      __IO uint32_t IT_LINE_SR[32];
    } SYSCFG_TypeDef;          /* no EXTICR anywhere */

So `EXTI->EXTICR[]` selects the port and `EXTI->IMR1`/`RTSR1`/`FTSR1` do the
rest. None of it is SYSCFG.

**Worth a slide.** This is a real architectural difference between STM32
families, not a detail: on F4 the same job is `SYSCFG->EXTICR[]`, and code
copied from an F4 tutorial will not compile here. Lesson 06 should say so.

## A1 — PARTIAL (2026-09-12)

`wokwi.com/license` states, verbatim:

> Wokwi for Visual Studio code is free for open source projects.
> For commercial use, you will need to purchase a license.

This repository is public, so it qualifies on its face. Two gaps remain, and
they are the ones that actually matter:

1. **Education/classroom terms are undocumented.** The licence page, the
   pricing page and the VS Code getting-started page all say nothing about
   academic use. A separate "Wokwi Classroom" licence is advertised with a
   quote-on-request form, which implies classroom use is expected to be a paid
   product.
2. **The student path needs the extension, not just the browser.** This is the
   crux and it was not obvious. The browser editor at wokwi.com needs no
   licence at all, but it *compiles the code itself* and has no route to load
   an arbitrary locally built ELF. Our lessons are bare-metal CMSIS with a
   hand-written linker script, which the browser build service will not build.
   Loading a local `Main.elf` requires `wokwi.toml`, which means the VS Code
   extension (licence key) or `wokwi-cli` (API token).

**Still to do, and it must be a human asking Wokwi directly:** whether ~60
students per term, each running the VS Code extension against this public
repository, is covered by "free for open source projects" or requires a
Classroom licence, and what that costs. Nothing else in Phase 0 is blocked
while that answer is pending, but Phase 1 should not start without it.


## A4 — RESOLVED (2026-09-12). 1.5 GB -> 247 MB.

xPack `arm-none-eabi-gcc` v15.2.1-1.1, win32-x64, 335,571,206 bytes compressed,
sha256 `bae6a3d1...fd93ea` **verified against the published .sha**.

Unpacked it is **1.5 GB**, not the ~1 GB the plan estimated. Pruned it is
**247 MB**, comfortably inside the 350 MB budget, and both `gcc` and `gdb`
still run afterwards.

The two multilib directories the plan predicted are exactly right:

    -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft            -> thumb/v6-m/nofp
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -> thumb/v7e-m+fp/hard

What the prune removes:

| Removed | Why | Saved |
|---|---|---|
| 37 of 39 multilib variants | arm/v5te, v7-a, v8-a, v8.1-m, pacbti, mve, dp, softfp ... | ~950 MB |
| `cc1plus.exe`, `f951.exe` | no C++, no Fortran in this course | 65 MB |
| `lto1.exe`, `lto-dump.exe` | we do not build with `-flto` | 58 MB |
| bundled Python + `gdb-py3` | plain `gdb.exe` needs neither | 34 MB |
| `ld.gold`, `dwp`, `gfortran`, `g++`, `c++` | unused drivers | ~12 MB |
| `share/doc`, `share/info`, `share/man` | | 17 MB |

**Kept `arm-none-eabi-gdb.exe`** (8.2 MB) deliberately. Both Wokwi and Renode
expose a GDB server, and the AVR tree never had a source-level debugger at all.

## A5 — RESOLVED, favourable (2026-09-12)

`resolve-libs.py:56` registers a symbol as defined only for
`elif kind in "TDBRCG"`. `W` and `V` are absent from that set, so a weak symbol
is never counted as a definition and cannot collide. Its docstring already
states the intent:

> 'A' (absolute) and 'W'/'V' (weak) are skipped because they legitimately
> repeat in every object.

So a lesson defining `TIM3_IRQHandler` **overrides** the startup file's weak
alias instead of colliding with it. The ISR-collision table in `CLAUDE.md` §5,
which forced a hand-maintained per-lesson `LIBS` list on AVR, largely does not
recur on ARM. `resolve-libs.py` still earns its place for genuine duplicate
definitions and for flash budgeting on a 32 KB part.

Belt and braces: `startup.c` lives in `_startup/`, not `_lib/`, so it is never
part of the catalogue `resolve-libs.py` scans in the first place.

## A2 — BUILDS CLEAN (2026-09-12); execution still unproven

`_spike/c031c6/` — `startup.c` (132 lines), `link.ld` (68), `main.c` (85).
Builds with **zero warnings** and produces valid artefacts:

    FLASH:  5260 B  /  32 KB   16.05%
    RAM:    2000 B  /  12 KB   16.28%
    .text 4792   .rodata 184   .data 96   .bss 364   heap+stack 1540
    Entry point 0x8000275      <- odd, the Thumb bit, as the verifier expects
    Main.hex record 1: :020000040800F2

Three plan predictions confirmed in passing:

1. **The extended-linear-address record is real.** `:020000040800F2` is the
   `0x08000000` base that the ATmega128's 128 KB flash never needed.
   `Test-IntelHex` must accept record type 04, and lesson 00 gets a concrete
   detail to decode.
2. **`size -A` behaves.** No Berkeley `.data`-folded-into-`text` trap
   (`CLAUDE.md` §3) on this linker, and `-Wl,--print-memory-usage` gives
   percent-of-flash at link time for free. On a 32 KB part that belongs in
   every build.
3. **The Thumb-bit check is meaningful** — the entry point really is odd.

### Three gotchas found by building, all of which belong in the shared files

- **`SystemCoreClock` must be defined by us.** ST's `system_stm32c0xx.h`
  declares it `extern`; their `system_stm32c0xx.c` defines it, and we
  deliberately do not take that file. Without a definition the link fails with
  `undefined reference to SystemCoreClock`. Defined in `startup.c` at the reset
  value (12 MHz) and raised to 48 MHz by `SystemInit()`.
- **`.init_array`/`.fini_array` must be marked `(READONLY)`.** They carry the
  writable flag by default, which marks the *entire* FLASH LOAD segment `RWE`
  and emits `warning: Main.elf has a LOAD segment with RWX permissions`.
  Adding `(READONLY)` to the output sections gives a correct `R E` segment and
  silences it honestly rather than with `--no-warn-rwx-segments`.
- **Implement the newlib syscalls; do not lean on `nosys.specs`.** Each stub it
  links emits `warning: _close is not implemented and will always fail` — seven
  of them, in every lesson. Writing `_write`/`_read`/`_close`/`_isatty`/
  `_lseek`/`_fstat`/`_sbrk` ourselves costs ~20 lines, removes all seven, and
  is what `_lib/retarget.c` is for in the plan. **This validates that design
  decision** rather than merely assuming it.

### Still unproven, and it is the important half

Nothing has **executed**. The firmware is correct by every static measure, which
is precisely the state `CLAUDE.md` §9e warns about: this repository has shipped
firmware that passed every automated check and drove nothing. A2 is not closed
until the banner is seen on a serial monitor, which needs A1 answered first.


## A3 — RESOLVED (2026-09-12). It runs, and it prints.

Renode 1.17.0 windows-portable: 106,735,414 bytes compressed, **298 MB
unpacked** — the plan quoted the 102 MB compressed figure, so budget 298 MB.
Runs out of the box, no .NET or Mono install needed.

`_spike/f446re/` builds with **zero warnings** (FLASH 5172 B / 512 KB = 0.99%,
RAM 3536 B / 128 KB = 2.70%) and under Renode produces:

    cpu: Setting initial values: PC = 0x80001ED, SP = 0x20020000
    usart2: === SOC3050 ARM spike: STM32F446RE ===
    usart2: SystemCoreClock : 16000000 Hz
    usart2: .data in RAM    : 96 bytes
    usart2: .bss  in RAM    : 364 bytes
    usart2: FPU 3.5 * 2.0   : 7000 (x1000)
    usart2: SPIKE OK

Everything that line-up proves, and none of it was provable before:

- **The hand-written `startup.c` executes.** The CPU took MSP and PC straight
  out of our vector table. `SP = 0x20020000` is `0x20000000 + 0x20000`, i.e.
  exactly the 128 KB top — so the derived `.repl` resize took effect too.
- **`.data` copy and `.bss` zero work.** Those byte counts are read from linker
  symbols at runtime, not hardcoded. Measured, not asserted.
- **`printf` works** through newlib-nano plus our own `_write`. The retarget
  design in the plan is confirmed, not assumed.
- **The FPU works.** `3.5 * 2.0` printed as `7000`, so `-mfloat-abi=hard`,
  `-mfpu=fpv4-sp-d16` and the `SCB->CPACR` enable in `SystemInit()` are all
  correct. This is the capability the ATmega128 never had.
- **The `.repl` derivation is ~20 lines and works**, exactly as planned.
  Renode ships no F446 board file; `nucleo_f446re.repl` is `using
  "platforms/cpus/stm32f4.repl"` plus a flash resize, an SRAM resize and an LED.

### Renode gives deterministic headless runs for free

`emulation RunFor "0.5"` runs half a second of **virtual** time and returns, so
a CI run is reproducible rather than wall-clock dependent. Combined with
`showAnalyzer sysbus.usart2 Antmicro.Renode.Analyzers.LoggingUartAnalyzer`,
UART lands on stdout with no GUI and no Robot Framework. That is a simpler CI
path than the `renode-test` route the plan assumed, and it needs nothing
installed.

`renode-test.bat` **does** need Robot Framework (`No module named 'robot'`),
which is not bundled. `test.robot` is written and left in place for when the
richer assertions are wanted; the headless `.resc` covers the basic case
without the dependency.

### Two things to fix before this is vendored

- **`stm32f4.repl` fetches an SVD from the network at load time**
  (`dl.antmicro.com/projects/renode/svd/STM32F40x.svd.gz`). It failed here and
  retried five times, costing ~5 seconds, then carried on — the SVD only
  affects register-name prettiness in logs, so the run was unaffected. But a
  vendored offline toolchain must not depend on a download: pre-fetch that SVD
  into `tools/renode/` and point the `.repl` at the local copy.
- **The F446RE spike runs on the raw 16 MHz HSI, not the planned 84 MHz PLL.**
  Renode does not model PLL lock faithfully, so a `while (!(RCC->CR &
  RCC_CR_PLLRDY))` loop is as likely to prove nothing as to prove something.
  The 84 MHz configuration stays Phase 4 work, against something that can
  actually be measured. Nothing here contradicts it.

### What is still NOT proven

**The C031C6 half has never executed.** Renode has no STM32C0 platform, so the
C0 spike is proven only to build. Its execution needs Wokwi, which needs A1
answered. Per `CLAUDE.md` §9e this repository has shipped firmware that passed
every static check and drove nothing, so "builds clean" is explicitly not
enough — and the C0 target is the one carrying 25 of the 36 lessons.


## A1b — the split model, and the one thing it rests on (2026-09-12)

Proposed after Phase 0: **maintainer develops in VS Code with a licence;
students use wokwi.com in the browser, free.** This is a much better cost shape
than 60 student licences, and it maps cleanly onto how the AVR course already
works — students clone the repo, run `build.bat`, then simulate.

Under that model `simulate.bat` stops launching SimulIDE and instead opens the
lesson's published Wokwi project, where the student loads the `Main.hex` they
just built. Nothing about the bare-metal CMSIS decision changes, and no student
needs a licence.

**It rests entirely on one unverified capability: can the wokwi.com web editor
load a locally built firmware file for an STM32 board?**

What is actually known:

| Claim | Source | Trust |
|---|---|---|
| `F1` -> `Upload Firmware and Start Simulation…` accepts `.bin`/`.elf`/`.uf2` | Wokwi's own ESP32 guide | **High — but documented for ESP32 only** |
| The same works for STM32, compile locally and upload | breadboardhub.com article | **Low** — the same article claims the NUCLEO-F446RE is supported, which Wokwi's own supported-hardware page contradicts |

Wokwi's supported-hardware page lists exactly **three** STM32 boards:
ST Nucleo C031C6, ST Nucleo L031K6, and the STM32F103C8 Bluepill. (Three, not
two as recorded earlier. The Bluepill is a Cortex-M3 with 64-128 KB flash and
20 KB RAM — no FPU, so it changes nothing about the F4/Renode split, but it is
a third option for the peripheral half if the C0 ever disappoints.)

### The alternative if A1b fails

If the browser cannot load external firmware for STM32, students would have to
write code **in** the browser and let Wokwi's build service compile it. That
service appears to target STM32 HAL/CMSIS templates, and it is unlikely to
accept a hand-written linker script and `startup.c`. That would put
"students use wokwi web" in direct conflict with "bare-metal CMSIS first" —
the two decisions cannot both survive. Resolving it would mean choosing:

1. students build locally and simulate some other way (Renode, terminal only);
2. the C0 half moves to ST HAL so the browser can build it, abandoning the
   register-level identity; or
3. students get licences after all.

None of those is cheap, which is exactly why A1b must be tested before Phase 1
rather than discovered in week one.

### The test — about two minutes, and it closes A2 as well

1. Open `https://wokwi.com/projects/new/st-nucleo-c031c6`
2. Click into the code editor and press `F1`
3. Look for **`Load HEX File and Start Simulation…`** (or `Upload Firmware…`)
4. If it is there, choose
   `projects2026_arm/_spike/c031c6/Main.elf` (fall back to `Main.hex`)
5. Expect on the serial monitor:

        === SOC3050 ARM spike: STM32C031C6 ===
        SystemCoreClock : 48000000 Hz
        .data in RAM    : 96 bytes
        .bss  in RAM    : 364 bytes
        SPIKE OK

A pass resolves **A1b and the execution half of A2 at once** — it would confirm
the hand-written C0 startup, the 48 MHz HSIDIV clock, the USART2 AF1 pin
mapping and the printf retarget, none of which any static check can prove.
If the banner appears but `SystemCoreClock` reads 12000000, the HSIDIV write
did not take and only the clock code is wrong.


## A1b and A2 — RESOLVED (2026-09-12). The C0 runs in the browser.

`Upload Firmware and Start Simulation…` **exists and works for STM32** in the
wokwi.com web editor. Loading `_spike/c031c6/Main.elf` into a
`board-st-nucleo-c031c6` project produced, on the serial monitor:

    === SOC3050 ARM spike: STM32C031C6 ===
    SystemCoreClock : 48000000 Hz
    .data in RAM    : 96 bytes
    .bss  in RAM    : 364 bytes
    SPIKE OK

with LD3 lit on the rendered board, so the blink loop runs too.

**Wokwi's own documentation describes this feature only in its ESP32 guide.**
It works for STM32 anyway. Recorded here because the docs will not tell the
next person, and because the one third-party source that did claim it also
claimed the F446RE was supported, which is false. Trust this entry, not the
web.

### What that single run proves

Everything the C0 half depends on, none of which any static check could reach:

- the hand-written `startup.c` executes — vector table, MSP, reset handler;
- `.data` copy and `.bss` zero work (96 and 364 bytes, read from linker
  symbols at runtime, not hardcoded);
- **the 48 MHz clock is real.** `SystemCoreClock : 48000000` means the
  `RCC->CR &= ~RCC_CR_HSIDIV` write took effect and the part left its 12 MHz
  reset state. Had this printed 12000000, everything else would still have
  passed;
- the flash wait state at 48 MHz did not hang the part;
- USART2 on PA2/PA3 with **AF1** is the correct alternate function;
- `printf` reaches the serial monitor through newlib-nano and our own
  `_write` — so `_lib/retarget.c` is confirmed, and the AVR tree's fifteen
  hand-rolled formatters are genuinely unnecessary here;
- `board-st-nucleo-c031c6` + `$serialMonitor` in `diagram.json` is the right
  wiring.

### The teaching model this settles

**Maintainer:** VS Code + one licence (expires 2026-10-12) for authoring,
debugging and GDB. **Students:** wokwi.com in the browser, **no licence, no
account cost** — they run `build.bat` as they already do on AVR, then load
`Main.hex`/`Main.elf` into the lesson's published Wokwi project.

The course workflow is therefore unchanged from the AVR edition — build, then
simulate — with the browser replacing SimulIDE. That was the single largest
open risk in the plan and it is now closed in the cheapest possible direction.

### Still open, but not blocking

- **Wokwi CI minutes.** `wokwi-cli` needs an API token (separate from the VS
  Code licence) and the free allowance is ~50 min/month. That constrains the
  Tier 2 automated checks, not the students. Phase 1 should confirm the token
  and the actual allowance before the CI matrix is designed around it.
- **The maintainer licence expires 2026-10-12** — one month out. Renewal terms
  for "free for open source projects" are undocumented.


## What the wokwi.com browser editor can and cannot build (2026-09-12)

Prompted by the obvious question: if code can be written directly in the web
editor, why build locally at all? Because the browser's build service only
compiles within frameworks it controls.

| Path | Where it compiles | Abstraction | Works for this course |
|---|---|---|---|
| `sketch.ino` (STM32duino) | **browser** | Arduino - `setup()`/`loop()`/`Serial` | No |
| STM32 HAL `main.c` templates | browser, apparently | `HAL_GPIO_WritePin()` | Only lessons 34-35 |
| **Bare-metal CMSIS** + own `link.ld` + `startup.c` | **locally, always** | `GPIOA->MODER` | **Yes - this is us** |

The decisive evidence is Wokwi's own examples. `wokwi/stm32-hello-wokwi` and
the community CMSIS example both ship `wokwi.toml`, `STM32C031C6TX_FLASH.ld`,
`startup_stm32c031xx.s` and a `Makefile`, and their READMEs say to build with
the Arm GNU Toolchain or STM32CubeCLT and then simulate. **They are not
compiled in the browser.** No browser project can supply its own linker script
and startup file, because the framework owns those.

So the two things are not interchangeable:

- **Browser compiles** only Arduino (and, it appears, HAL templates).
- **Browser runs** anything, via `Upload Firmware and Start Simulation…`.

Our lessons use the second. That is not a limitation we are working around - it
is the same build-then-simulate workflow the AVR edition already has, and
students install nothing because the toolchain is vendored.

**The fork this leaves open, should it ever be wanted:** teaching STM32duino in
the browser would remove the local build entirely. It would also abandon
register-level teaching, which is the course's identity and the reason 42% of
the AVR slides are register bit-field diagrams. Recorded as considered and
rejected, not overlooked.


## Target A bake-off: C031C6 vs L031K6 vs F103C8 (2026-09-12)

Driven by a real concern: **students cannot pay for Wokwi.** They do not have
to today — the browser upload path is free and proven — but the C031C6 is the
one candidate with no offline fallback, so the course would rest entirely on a
commercial service staying free. All three were built and run.

| | C031C6 | L031K6 | F103C8 |
|---|---|---|---|
| Core | M0+ 48 MHz | M0+ 16-32 MHz | **M3 8-72 MHz** |
| Flash / RAM | 32K / 12K | 32K / **8K** | **64K / 20K** |
| Usable GPIO | ~50 (Nucleo-64) | **~15 (Nucleo-32)** | ~32 |
| Wokwi | **proven in browser** | listed | listed |
| Renode | **none, none possible** | proven, 4-line derivation | **proven, ships natively** |
| GPIO model | MODER | MODER | **CRL/CRH** |
| USART | ISR/TDR | ISR/TDR | **SR/DR** |
| RCC | APBENR1 | APB1ENR | APB1ENR + APB2ENR |
| Spike RAM used | 2000 B = 16.3% | 2000 B = **24.4%** | 2000 B = 9.8% |
| Toolchain | v6-m/nofp | v6-m/nofp | **v7-m/nofp, +28 MB** |

All three printed their banner. L031K6 ran on a four-line `.repl` derived from
`stm32l071.repl` (`SP = 0x20002000` confirms the 8 KB resize took); F103C8 ran
on Renode's own `stm32f103.repl` with no derivation at all.

### Two corrections to earlier reasoning in this log

1. **"C0 and F446RE are consistent because both use MODER" was only half
   right.** GPIO matches; **USART does not**. C0 and L0 use `ISR`/`TDR`, while
   F4 — and F1 — use `SR`/`DR`. So no candidate is fully consistent with the
   F446RE:

   - C031C6 / L031K6 + F446RE: GPIO matches, USART differs
   - F103C8 + F446RE: USART matches, GPIO differs

   GPIO appears in nearly every lesson and USART mostly behind a library, so
   GPIO consistency is worth more — but the gap is narrower than claimed.

2. **Even C0 -> L0 is not a free port.** The C0 spike would not compile for the
   L0 until two things changed: `RCC->APBENR1` -> `RCC->APB1ENR`, and
   `USART_ISR_TXE_TXFNF` -> `USART_ISR_TXE` (the C0's name is FIFO-aware).
   Peripheral *layout* transfers between modern families; **RCC naming does
   not.** Budget per-lesson edits for any family change, not a recompile.

### A prune consequence worth remembering

The F103 link failed at first with `Unknown destination type (ARM/Thumb)` and
`dangerous relocation`, because the A4 prune keeps only `thumb/v6-m/nofp` and
`thumb/v7e-m+fp/hard`. **Cortex-M3 needs `thumb/v7-m/nofp`**, and without it
the linker silently falls back to the default ARM (non-Thumb) libraries.
Restoring it costs **+28 MB (247 -> 275 MB)**, still inside budget.

The general rule: **the pruned toolchain supports exactly the cores it was
pruned for.** Adding a core is a toolchain change, not just a compiler flag,
and the failure it produces names relocations rather than multilibs, so it
reads as a code bug. `verify-all.ps1` should check the multilib exists for each
declared target.

### L031K6 is eliminated

Not on the Renode question, which it passed, but on capacity: **8 KB of RAM and
~15 usable GPIO on a Nucleo-32**. The spike alone uses 24.4% of its RAM, and a
1 KB framebuffer for the graphics and game lessons would take another 12.5%.
Fifteen pins will not carry a 4x4 keypad (8), a character LCD (6+), an OLED,
and SPI. The remaining choice is between C031C6 and F103C8.


## The fee question, settled (2026-09-12)

Three distinct things get conflated. They have different costs.

| Route | Licence | Cost | Renewal |
|---|---|---|---|
| **wokwi.com browser** + locally built ELF | **none** | **free** | never |
| **Wokwi VS Code extension** | personal key | free | **every 30 days** |
| `wokwi-cli` (CI) | API token | ~50 min/month free | n/a |

**The VS Code licence is not a trial.** Wokwi issues a *personal 30-day*
licence, and when it expires you generate another one free, indefinitely. The
maintainer key reading `x=20261012` is simply 30 days from issue. So the
earlier note in this log that the licence "expires 2026-10-12, renewal terms
undocumented" was half wrong: it does expire, and renewal is free and
self-service.

### Why students still belong in the browser, not VS Code

Students *could* each generate a free personal key. Against that:

- **Every student renews every 30 days.** A 15-week term is four renewals each.
  Someone will hit an expired key the night before a deadline.
- Each needs a Wokwi account before they can build anything.
- **Wokwi sells "Wokwi Classroom" for exactly this use**, minimum 5 students,
  aimed at universities. A whole class running free *personal* licences is at
  best a grey area in the spirit of those terms.

The browser route has none of that: no key, no account requirement, no
renewal, no grey area, and it is the route already proven working with a
locally built ELF.

**Settled model:** maintainer in VS Code on a self-renewed personal licence;
students build locally with the vendored toolchain and load `Main.elf` in the
browser. Zero cost to students, no terms risk.

### What this does to the Target A decision

It removes **money** from the C031C6-vs-F103C8 question entirely. What remains
for F103C8 is narrower than it looked: resilience against Wokwi changing its
free tier, and the ability to run with no internet at all. Those are real, but
they are availability arguments, not cost arguments — and they should be
weighed against the C031C6 being the better teaching part (12 KB vs 20 KB RAM
is the one point against it; ~50 pins, 48 MHz, and a one-write clock setup are
all points for it).


## DECIDED: the simulation model (2026-09-12)

**Wokwi is a simulator only. It never compiles anything for this course.**

    edit Main.c   ->  VS Code            free, no licence
    build         ->  arm-none-eabi-gcc  free, vendored in the repo
    simulate      ->  wokwi.com browser, upload Main.elf   free, no licence
    (maintainer)  ->  Wokwi VS Code extension              free 30-day key,
                                                           self-renewed

The licence buys exactly one thing: the simulator appearing inside VS Code
rather than in a browser tab. It does not gate editing, compiling, debugging,
or the ability to change and re-run code. **Students pay nothing, ever, and
need no account.**

This closes the fee question completely. It was the last open item from
Phase 0.


## The backslash hazard, third sighting (2026-09-12)

CLAUDE.md 9a and 9e both document text tooling silently eating backslashes -
`` and `` swallowed inside `.bat` paths, invisible in a terminal and in
rendered Markdown. It happened again while patching `build-slides.py`, in a
new place: a heredoc-delivered Python patch.

The patch contained a generated line

    % "BACKSLASH-n".join(body)

and what reached the file was a string literal containing a **real newline**,
producing `SyntaxError: unterminated string literal`. Loud this time, because
Python refused to parse it - but the same class of damage as the silent cases,
and the third distinct tool to do it.

**Rule: never rely on a backslash escape surviving a heredoc.** Where a control
character is needed in generated code, write it without a backslash:

    chr(10)   instead of   "BACKSLASH-n"
    chr(9)    instead of   "BACKSLASH-t"

`build-slides.py` now uses `chr(10).join(body)` for exactly this reason. It
reads slightly oddly and that is the point: the odd spelling is load-bearing.

After any generated-file edit, parse or lint the result before trusting it
(`python -c "import ast; ast.parse(open(f).read())"` took two seconds and
caught this).
