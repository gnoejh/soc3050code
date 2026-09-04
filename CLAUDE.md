# CLAUDE.md — SOC3050 Embedded Systems Course

Working notes and development tracker for this repository. Claude Code reads this
at session start; keep it current as work proceeds.

---

## 1. Repository purpose

ATmega128 embedded-systems teaching framework. Each lesson ships **code + slides +
a simulator circuit** so students can build, run and study a topic without hardware.

Self-contained: the AVR toolchain and the SimulIDE simulator are vendored under
`tools/`. No external installs required.

## 2. Layout

```
projects/             53 legacy lesson folders (previous edition, SimulIDE 1.1.0-SR1)
projects2026_avr/     22 curated lessons for SimulIDE 1.1.0-SR2   <-- current edition
shared_libs/          _port, _adc, _uart, _timer, _glcd, _eeprom, _pwm, _interrupt ...
tools/avr-toolchain/  avr-gcc 15.1.0, avr-objcopy, avr-size, avrdude
tools/simulide/       SimulIDE 0.4.15-SR10 and 1.1.0-SR1 + the original master board
tools/simulide110sr2/ SimulIDE 1.1.0-SR2  <-- the simulator the 2026 edition targets
tools/simulide200/    SimulIDE 2.x (R260501) — not targeted this cycle
python_projects/      separate Python track (out of scope here)
docs/                 framework + SimulIDE documentation
```

`tools/simulide110sr2/SimulIDE_1.1.0-SR2_Win64/` is committed, so a fresh clone
can simulate. The `.zip` files beside it are ignored - one duplicates the
extracted folder, the other four are SimulIDE 2.x companion apps nothing here
uses. `tools/simulide200/` (159 MB) remains untracked and unused.

## 3. Toolchain facts (verified)

| Item | Value |
|---|---|
| MCU | ATmega128 at 16 MHz |
| Compiler | `tools/avr-toolchain/bin/avr-gcc.exe` — avr-gcc 15.1.0 |
| Flags | `-mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -Os -Wall -Wextra -ffunction-sections -fdata-sections -Wl,--gc-sections` |
| Output | `Main.elf` → `avr-objcopy -O ihex -R .eeprom` → `Main.hex` |
| SR2 binary | `tools/simulide110sr2/SimulIDE_1.1.0-SR2_Win64/simulide.exe` |
| SR2 native circuit | `.sim1`, `<circuit version="" rev="2208">` |
| Legacy circuit | `.simu`, `rev="2005"` — SR2 opens these fine (verified by launching) |
| ATmega128 model | `data/AVR/mega128.mcu` present in SR2; `itemtype="MCU"` unchanged from SR1 |
| Shared board | `projects2026_avr/Simulator.simu` — every component in it is present in the SR2 binary |

**`avr-size` in this toolchain does not support `--format=avr`.** Use the plain
Berkeley output.

## 4. Decisions (confirmed 2026-09-05)

| Decision | Choice |
|---|---|
| Scope | Curated core set — 22 lessons, one per topic |
| Circuits | **One shared master board**; each lesson loads it against its own `Main.hex` |
| Clock | **16 MHz everywhere**, matching the board's `Frequency="16 MHz"` |

## 5. Build system

The legacy `build.bat` files each hand-maintained their own list of
`shared_libs` sources. That list cannot be replaced with "link everything",
because several lessons **deliberately define their own ISR or UART routine**
that a shared library also defines. Verified collisions:

| Lesson | Collision |
|---|---|
| `Timer0_Overflow_Blink`, `RTOS_UPGRADED` | `__vector_16` (TIMER0_OVF) vs `_timer.c` |
| `Timer1_CTC_Precision` | `__vector_12` vs `_timer.c` |
| `Timer1_Input_Capture` | `__vector_11` vs `_timer.c` |
| `Power_Sleep_Modes` | `__vector_10` vs `_timer.c` |
| `Watchdog_System_Reset` | `led_all_on` / `led_all_off` vs `_port.c` |
| `UART` | `putch_USART1` / `puts_USART1` / `getch_USART1` vs `_uart.c` |

**Design:** one shared engine plus a per-lesson library list.

```
projects2026_avr/
├── _build/build-lesson.bat     compile + link + hex + size
├── _build/simulate-lesson.bat  place the board next to the hex, launch SR2
├── Simulator.simu              the single maintained board
└── NN_Topic/
    ├── build.bat               "set LIBS=..." then calls the engine
    ├── simulate.bat            calls the launcher
    └── Main.c  config.h  Slide.md  README.md
```

`_build/resolve-libs.py` derives each lesson's minimal conflict-free library set
from `avr-nm` symbol tables. Re-run with `python _build/resolve-libs.py --all`.

**Batch gotcha, cost an hour:** `echo Build OK -> Main.hex` inside a `.bat` is a
*redirect* — it silently overwrote every `Main.hex` with the text `Build OK  -`.
Existence checks passed; the firmware was empty. Validate hex content
(starts with `:`, more than a few hundred bytes), never just `Test-Path`.

## 6. The shared board — traced from the circuit file

| Port | Connected to |
|---|---|
| PORTA0–7 | KS0108 GLCD data bus |
| PORTB0–7 | 8 LEDs **active low**; PB0–3 also stepper + SPI; PB0 scope; PB6–7 logic analyser |
| **PORTC** | **not connected at all** |
| PORTD0, D1, D4–D7 | push buttons, pulled up |
| PORTD2, D3 | RXD1 / TXD1 — the serial port |
| PORTE3 | joystick; PORTE4–E7 GLCD control (also INT4–INT7) |
| PORTF0 | potentiometer (ADC0); PORTF1, F2 joystick axes |
| PORTG1 | GLCD control; PORTG4 slide switch |

LED wiring is rail → resistor → anode, cathode → pin, so **driving a pin LOW
lights its LED**. This matches `shared_libs/_port.c`, which documents PORT B as
the LED array.

**Placed but unwired:** DS1307, DS1621, DS18B20, DHT22, ESP01, KY040, TouchPadR.
Lesson 16 needs its DS1307 wired to PD1 (SDA) / PD0 (SCL) before the RTC demos
respond; those pins already carry the pull-ups an I2C bus wants. Left as a GUI
step on purpose - see section 10 for why, and `16_I2C_RTC_DS1307/README.md` for
the click-by-click instructions.

## 7. What was changed during the upgrade

- **Clock unified at 16 MHz.** Five lessons disagreed with themselves between
  `build.bat` and `config.h`; in `ADC_Basic` and `Timer_PWM_Motor_DC` the
  `#define` was unguarded and silently overrode the compiler flag, so delays and
  baud rates were computed for the wrong clock. `config.h` now guards it.
- **Status LEDs moved off PORT C.** Six lessons drove the unconnected PORT C.
  They now go through `LED_WRITE()` / `LED_TOGGLE()` / `LED_DDR` in `config.h`,
  targeting PORT B and handling the active-low inversion in one place.
  `15_SPI_Master_Basic` was deliberately left alone — PB0–PB3 carry its SPI bus.
- **Serial pin conflict.** `02_Port_Button_Debounce` used PD2 as a button; PD2 is
  RXD1 and carried the lesson's own output. Moved to PD4.
- **Real defects fixed:** `sprintf` buffer overflows in the keypad (30–34 bytes
  into `buf[20]`) and LCD lessons; `%u` given an `unsigned long` in the servo
  lesson; a `PROGMEM` array declared without `static` so the attribute was
  ignored and `lpm` read a RAM address as flash; a status print in the
  ring-buffer burst demo that sat outside the data branch and repeated forever.
- **Encoding damage repaired.** 310 sequences mangled by an old UTF-8/CP949
  round-trip, including banner lines where the `\` of `\r` had been eaten.
- **Slides completed.** `02`, `05`, `14`, `19` had none; all 22 now do.
- **`#include "config.h"` made universal** — `09`, `18`, `21` did not include it.

## 8. Status

**All 22 lessons build clean and produce valid Intel HEX**, checked by
`projects2026_avr/_build/verify-all.ps1`, which validates hex records rather
than merely testing that a file exists. SimulIDE 1.1.0-SR2 was launched against
the board and confirmed to load both circuit and firmware.

Remaining compiler warnings are only `defined but not used` on demo functions
students are meant to uncomment (`04_Inline_Assembly`, `18_GLCD_Graphics`), plus
unused-parameter warnings inside `shared_libs` stubs. Those stubs are honest
`// TODO` placeholders and **no lesson calls any of them** — checked.

| Date | Status |
|---|---|
| 2026-09-05 | Surveyed repo; verified SR2 format, `mega128.mcu`, toolchain. |
| 2026-09-05 | Scope/circuit/clock decisions confirmed; lib-collision problem found and solved. |
| 2026-09-05 | 22 lessons imported, normalised, defects fixed, slides completed, all building and verified in SR2. |
| 2026-09-05 | `projects2026_avr/` + this tracker committed and pushed. |
| 2026-09-05 | System pass: VS Code tasks repaired, verifier and slide renderer added, both READMEs rewritten, `projects/` archived, SimulIDE SR2 committed. |
| 2026-09-05 | History squashed to a single root commit on `main` and force-pushed; all prior history deleted. See section 12. |

## 9. The system pass (2026-09-05)

### VS Code tasks were broken for the 2026 tree
`tools/cli/cli-build-project.ps1` gates on `$ProjectDir -like "*\projects\*"`,
so **"Build Current Project" — the entry point the README documents — failed
outright on every 2026 lesson** with "Not a project folder". That script also
guesses the library set by branching on filenames, and one branch links
`_interrupt_manager.c`, which does not exist in `shared_libs/`.

Fixed with two dispatchers, `tools/cli/build-current.ps1` and
`simulate-current.ps1`. They identify the 2026 tree by the shared engine sitting
in a `_build` directory beside the lesson — **not** by the presence of a
`build.bat`, since legacy projects have one of those too — and fall through to
the legacy scripts for anything under `projects/`. `tasks.json` was rewritten
around them, legacy tasks relabelled `Legacy:`, and `$gcc` problem matchers
added so compiler errors become clickable.

### Added
- **`_build/verify-all.ps1`** — builds all 22 and validates each `Main.hex` as
  real Intel HEX. This exists because of the `echo Build OK -> Main.hex`
  redirect bug: every hex was 13 bytes of the word "Build OK" and every
  `Test-Path` check still passed.
- **`_build/build-slides.py`** — renders all 22 `Slide.md` into self-contained
  HTML decks (452 slides) with an index, keyboard navigation, an overview grid
  and print-to-PDF. No third-party packages, since `markdown` is not installed
  and the repo promises a clone needs no downloads. Two renderer bugs found and
  fixed while checking output: bold spanning a source line break, and `\|`
  escapes inside table cells.

### Documentation
- Root `README.md` rewritten. The old one **duplicated its entire first 75
  lines**, carried mojibake headings, described "35 Progressive Projects", and
  linked `FRAMEWORK_GUIDE.md` and `PROJECT_CATALOG.md`, neither of which exists
  at the repo root.
- `projects/ARCHIVED.md` added, saying plainly that the tree is not maintained,
  why, and listing the topics not yet carried across.

### SimulIDE SR2 is now committed
`tools/simulide110sr2/SimulIDE_1.1.0-SR2_Win64/` (2851 files) is in git, so a
fresh clone can simulate. The five `.zip` files beside it are ignored: one
duplicates the extracted folder, the other four are SimulIDE 2.x companion apps
nothing in the course uses. `tools/simulide200/` (159 MB) remains untracked.

## 10. Next steps

- [ ] **Wire the DS1307 on the shared board** — the one item deliberately not
      automated. Both junction nodes on PD0 and PD1 are already at their
      three-connection limit, so adding the RTC means splicing new nodes into
      existing wires. That is safe by hand in the GUI and risky to do by editing
      the circuit file every lesson shares, with no way to confirm the
      electrical result from the CLI. Step-by-step instructions are in
      `16_I2C_RTC_DS1307/README.md`. The same applies to DHT22, DS18B20,
      DS1621, ESP01 and KY040 if those lessons are ever added.
- [ ] Decide whether to save the board in SR2's native `.sim1` format
      (rev 2208). It is currently the rev-2005 `.simu`, which SR2 reads but
      does not write.
- [ ] `shared_libs/` still has checked-in `.o` files (`_init.o`, `_port.o`) and
      unused variants (`_glcd_legacy`, `_init_safe`, `_init_simple`,
      `uart_enhanced`) that `resolve-libs.py` has to exclude by name.
- [ ] Carry across any wanted topics listed in `projects/ARCHIVED.md`.

## 11. Conventions for future work

- `projects2026_avr/` is the **source of truth**. The one-time importers in
  `_build/` (`migrate.py`, `remap-leds.py`, `fix-mojibake.py`) are a record of
  what changed, not a pipeline — re-running `migrate.py --force` would discard
  every fix made since.
- `verify-all.ps1`, `gen-readme.py`, `build-slides.py` and `resolve-libs.py`
  **are** meant to be re-run; all derive their output from the lesson sources.
- Run `verify-all.ps1` after touching anything shared — the engine,
  `shared_libs`, or a `config.h`.
- Re-run `build-slides.py` after editing a deck; `_slides/` is generated.
- Board-specific facts belong in a lesson's `config.h`, not scattered in `Main.c`.
- **Never trust a build's exit code alone** on this project. Batch redirects and
  missing artefacts have both produced green builds with no firmware.
- Work directly on `main`. The repository has one branch and no history to
  preserve alongside it - see section 12.

## 12. Git history

On 2026-09-05 the repository was deliberately flattened: every prior commit was
replaced by a single root commit and force-pushed over `origin/main`.

```
0cc10fa  SOC3050 ATmega128 embedded systems course   (no parent)
```

What went with it, permanently:

- the ten commits from the 2025-12-04 root onward,
- the `projects2026-avr-simulide-sr2` and `copilot/upgrade-copilot-plan`
  branches on the remote,
- the contribution history those commits represented - the green squares for
  that period are gone from the GitHub profile, since contributions are keyed
  to commits that no longer exist.

Local backup refs and the reflog were expired afterwards at the maintainer's
instruction, so **nothing survives to restore from**. The squashed tree was
verified byte-identical to the pre-squash tip before the push, and all 22
lessons were rebuilt from the new root, so no file content was lost - only
history.

`.git` went from 112 MB to 92 MB. It will not shrink much further: the 89 MB
pack is almost entirely the vendored AVR toolchain and SimulIDE install, which
are meant to be there.

Anyone holding a clone from before this needs a fresh one; `git pull` will
refuse to fast-forward.

**Before any future history rewrite**, weigh the same three costs: contribution
history disappears, existing clones break, and open pull requests lose their
base commits.
