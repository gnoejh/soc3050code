# SOC3050 — ATmega128 Embedded Systems

A self-contained teaching framework for the ATmega128. Every lesson ships with
source, a lecture deck and a simulator circuit, and the whole toolchain is
vendored — clone the repository and it works, with nothing to install.

---

## Quick start

```bash
git clone <repository-url>
cd soc3050code
code .
```

Then open any file under `projects2026_avr/01_Port_Basic/` and press
**Ctrl+Shift+B** to build, or run **Terminal → Run Task → Simulate Current
Project** to open it in the simulator.

From a terminal instead:

```
cd projects2026_avr\01_Port_Basic
build.bat        compiles Main.c to Main.hex
simulate.bat     opens the shared board in SimulIDE 1.1.0-SR2
```

In SimulIDE press **Play**. Where a lesson prints to the serial port, open the
**Serial Monitor** at **9600 baud, 8N1**.

To check the whole course still builds:

```
pwsh projects2026_avr\_build\verify-all.ps1
```

---

## Which lesson tree to use

There are two, and **`projects2026_avr/` is the current one**.

| | `projects2026_avr/` | `projects/` |
|---|---|---|
| Status | **current, maintained** | archived, kept for reference |
| Lessons | an introduction plus 22, one per topic | 53, with overlaps and variants |
| Simulator | SimulIDE **1.1.0-SR2** | SimulIDE 1.1.0-SR1 |
| Clock | 16 MHz throughout | mixed 16 MHz / 7.3728 MHz |
| Build | one shared engine | per-project scripts of several shapes |
| Slides | all 23 lesson folders | 34 of 53 |

Start in `projects2026_avr/`. See [its README](projects2026_avr/README.md) for
the lesson list and the board map. `projects/` is left in place because a lot of
supporting material still points at it, but it is not being maintained — see
[projects/ARCHIVED.md](projects/ARCHIVED.md).

---

## The lessons

**Lesson 00** is the first class: what an embedded processor is, how the
toolchain turns C into firmware, and how the ATmega128 is built. It is the one
lesson to read before touching anything else.

Then twenty-two, in teaching order:

| | | | |
|---|---|---|---|
| 01 Port Basic | 07 Timer1 CTC | 13 UART Basic | 19 EEPROM |
| 02 Button Debounce | 08 Input Capture | 14 UART Ring Buffer | 20 Sleep Modes |
| 03 Keypad Matrix | 09 PWM DC Motor | 15 SPI Master | 21 Watchdog |
| 04 Inline Assembly | 10 PWM Servo | 16 I2C RTC | 22 RTOS Scheduler |
| 05 External Interrupts | 11 ADC Basic | 17 Character LCD | |
| 06 Timer0 Overflow | 12 ADC Light Sensor | 18 GLCD Graphics | |

Each lesson folder holds `Main.c`, `config.h`, `Slide.md`, `README.md`,
`build.bat` and `simulate.bat`.

---

## Lecture slides

Every lesson has a `Slide.md`. To turn all 23 into presentable HTML decks:

```
python projects2026_avr\_build\build-slides.py
```

Open `projects2026_avr/_slides/index.html`. Arrow keys or space to move, `o`
for an overview grid, `p` to print or save as PDF. There is also a
**Build Slides (2026)** task in VS Code.

### On the web

The same decks are published to GitHub Pages at
**<https://gnoejh.github.io/soc3050code/>**, for teaching from a machine that
does not have this repository on it.

`.github/workflows/pages.yml` re-renders the decks from their `Slide.md`
sources and deploys on every push that touches a deck or the renderer, so the
published site never lags the lesson text. Nothing else in the repository is
published — the workflow checks out only `projects2026_avr/` and uploads only
the rendered `_slides/` directory.

The site carries the slides only. Building and simulating a lesson still needs
a clone, because the compiler and SimulIDE are vendored under `tools/`.

---

## What is included

```
projects2026_avr/     the 22 current lessons + the shared board + _build tooling
projects/             the archived 53-project edition
shared_libs/          _port, _adc, _uart, _timer, _glcd, _eeprom, _pwm, _interrupt
tools/avr-toolchain/  avr-gcc 15.1.0, avr-objcopy, avr-size, avrdude
tools/simulide110sr2/ SimulIDE 1.1.0-SR2 — the simulator the current edition targets
tools/simulide/       SimulIDE 1.1.0-SR1 and 0.4.15, for the archived tree
tools/cli/            build, program and analysis scripts
docs/                 framework, hardware and SimulIDE documentation
python_projects/      a separate Python track — see PYTHON_PROJECTS_GUIDE.md
```

No external downloads and no internet connection are needed to build, simulate
or program.

---

## VS Code tasks

| Task | What it does |
|------|--------------|
| **Build Current Project** (Ctrl+Shift+B) | Builds the lesson containing the open file |
| **Simulate Current Project** | Builds if needed, then opens it in SimulIDE |
| **Verify All Lessons (2026)** | Builds all 23 and validates every `Main.hex` |
| **Build Slides (2026)** | Renders all `Slide.md` files into HTML decks |
| **Show Memory Usage** | `avr-size` for the lesson's `Main.elf` |
| **Clean Project** | Removes build output from the lesson folder |
| **Program Hardware** | Flashes the built hex via avrdude |

Build and Simulate work in **both** trees — the task looks at which folder the
open file is in and picks the matching build system.

---

## Hardware

The lessons are written against the shared simulator board, and the same pin
map applies to a physical ATmega128 lab board.

| Port | Used for |
|------|----------|
| **PORTA0–7** | KS0108 graphic LCD data bus |
| **PORTB0–7** | 8 LEDs, **active low**; also stepper coils and the SPI bus |
| **PORTD0, D1, D4–D7** | push buttons, pulled up |
| **PORTD2, D3** | RXD1 / TXD1 — serial, 9600 8N1 |
| **PORTE4–E7** | GLCD control lines, also INT4–INT7 |
| **PORTF0–F2** | ADC inputs — potentiometer and joystick axes |
| **PORTG1, G4** | GLCD control, slide switch |

**LEDs are active low**: the anode is on the 5 V rail and the cathode on the
pin, so driving a pin LOW lights its LED. **PORTC is not connected** on this
board.

For real hardware you also need an ATmega128 board, a programmer (Arduino as
ISP, USBasp or similar) and a USB-serial adapter for the console.

---

## Troubleshooting

**PowerShell refuses to run a script**

```
setup.bat                                              # easiest
powershell -ExecutionPolicy Bypass -File fix-powershell.ps1
```

**Check the environment**

```
powershell -ExecutionPolicy Bypass -File verify-environment.ps1 -Verbose
```

**A lesson will not build** — run `pwsh projects2026_avr\_build\verify-all.ps1
-ShowWarnings` to see whether it is just that lesson or something shared.

**Nothing appears in the serial monitor** — check it is on 9600 baud, and that
the board's serial component is the one wired to PD2/PD3.

**The simulator will not start** — check that
`tools/simulide110sr2/SimulIDE_1.1.0-SR2_Win64/simulide.exe` is present. It is
committed to the repository, so a complete clone has it.

---

## Documentation

- [projects2026_avr/README.md](projects2026_avr/README.md) — the current edition, board map, build system
- [CLAUDE.md](CLAUDE.md) — development tracker: decisions, known issues, next steps
- [docs/LAB_GUIDE.md](docs/LAB_GUIDE.md) — lab manual
- [docs/LIBRARY_REFERENCE.md](docs/LIBRARY_REFERENCE.md) — shared library functions
- [docs/FRAMEWORK_GUIDE.md](docs/FRAMEWORK_GUIDE.md) — architecture and design
- [docs/HARDWARE_REFERENCE.md](docs/HARDWARE_REFERENCE.md) — board and pin details
- [PYTHON_PROJECTS_GUIDE.md](PYTHON_PROJECTS_GUIDE.md) — the Python track

---

## License

Educational use. See individual source files for specific terms.
