# SOC3050 — ATmega128 Lessons, 2026 AVR Edition

Twenty-two lessons for the ATmega128, each with source, a lecture deck and a
one-command path into the simulator. Built and verified against
**SimulIDE 1.1.0-SR2**.

This is the successor to `../projects`. It is a curated set — one strong lesson
per topic rather than every variation — normalised so that every lesson builds
the same way, runs at the same clock, and works on the same board.

---

## Quick start

```
cd 01_Port_Basic
build.bat        compiles Main.c -> Main.hex
simulate.bat     opens the shared board in SimulIDE 1.1.0-SR2
```

In SimulIDE press **Play**. Where a lesson prints to the serial port, open the
**Serial Monitor** at **9600 baud, 8N1**.

Nothing needs installing. The AVR toolchain and SimulIDE both live under
`../tools`.

---

## The lessons

| # | Lesson | Focus |
|---|--------|-------|
| 01 | `01_Port_Basic` | DDR, PORT and PIN; driving LEDs, reading switches |
| 02 | `02_Port_Button_Debounce` | Pull-ups, contact bounce, software debouncing |
| 03 | `03_Port_Keypad_Matrix` | Scanning a 4x4 keypad with 8 pins |
| 04 | `04_Inline_Assembly` | `asm volatile`, operand constraints, AVR I/O instructions |
| 05 | `05_INT_External_Pins` | INT0–INT7, edge selection, writing an ISR |
| 06 | `06_Timer0_Overflow_Blink` | Prescalers, the overflow flag, interrupt timing |
| 07 | `07_Timer1_CTC_Precision` | Clear-Timer-on-Compare for exact frequencies |
| 08 | `08_Timer1_Input_Capture` | Measuring pulse width and frequency |
| 09 | `09_PWM_Motor_DC` | Fast PWM, duty cycle, H-bridge motor drive |
| 10 | `10_PWM_Servo` | 50 Hz pulse-width servo positioning |
| 11 | `11_ADC_Basic` | ADMUX, ADCSRA, references, 10-bit conversion |
| 12 | `12_ADC_Light_Sensor` | Voltage dividers and scaling counts to real units |
| 13 | `13_UART_Basic` | UBRR and baud rate, framing, polled transmit/receive |
| 14 | `14_UART_Ring_Buffer` | RX interrupts and a lock-free ring buffer |
| 15 | `15_SPI_Master_Basic` | SPCR, clock modes, full-duplex transfer |
| 16 | `16_I2C_RTC_DS1307` | TWI start/stop/ack, talking to a DS1307 |
| 17 | `17_LCD_Character` | Text output, cursor positioning, formatting |
| 18 | `18_GLCD_Graphics` | KS0108 pages and columns, pixels, lines, bitmaps |
| 19 | `19_EEPROM_ReadWrite` | Non-volatile storage, write timing, endurance |
| 20 | `20_Power_Sleep_Modes` | Sleep modes, wake-up sources, current budgeting |
| 21 | `21_Watchdog_Reset` | WDT time-outs, safe reset, recovering from a hang |
| 22 | `22_RTOS_Scheduler` | Task control blocks, tick scheduler, context switching |

---

## The shared board

Every lesson opens the same circuit, `Simulator.simu`. `simulate.bat` copies it
next to the lesson's own `Main.hex`; SimulIDE resolves the MCU's
`Program="Main.hex"` relative to the circuit file, so each lesson loads its own
firmware from one maintained board.

**MCU: ATmega128 at 16 MHz.** Every lesson is compiled with
`-DF_CPU=16000000UL` to match.

### What is wired where

| Port | Connected to |
|------|--------------|
| **PORTA0–7** | KS0108 graphic LCD data bus |
| **PORTB0–7** | 8 LEDs (**active low**); PB0–PB3 also stepper coils and the SPI bus; PB0 to the oscilloscope; PB6–PB7 to the logic analyser |
| **PORTC** | **not connected** |
| **PORTD0, D1, D4–D7** | push buttons, pulled up |
| **PORTD2, D3** | RXD1 / TXD1 — the serial port |
| **PORTE3** | joystick |
| **PORTE4–E7** | GLCD control lines (also INT4–INT7) |
| **PORTF0** | potentiometer (ADC0) |
| **PORTF1, F2** | joystick axes (ADC1, ADC2) |
| **PORTG1** | GLCD control |
| **PORTG4** | slide switch |

### LEDs are active low
The anode sits on the 5 V rail through a resistor and the cathode on the pin, so
a pin must be driven **LOW** to light its LED. This matches
`../shared_libs/_port.c`, which documents PORT B as the LED array. Lessons that
write LED patterns go through `LED_WRITE()` in their `config.h`, which does the
inversion in one place — `LED_WRITE(0x0F)` lights the low four LEDs.

### Components placed but not wired
The board carries a **DS1307**, **DS1621**, **DS18B20**, **DHT22**, **ESP01**,
**KY040** and a **touch pad** that are present but have no connections. Any
lesson using one needs its wiring drawn in SimulIDE first.

This affects **lesson 16**, whose RTC demos stay silent until the DS1307's SCL
and SDA reach PD0 and PD1 — [its README](16_I2C_RTC_DS1307/README.md) has
click-by-click instructions. It is left as a manual step on purpose: the
junction points on PD0 and PD1 are already at their three-connection limit, so
adding the RTC means splicing new nodes into existing wires. That is
straightforward in the GUI and risky to do by editing the circuit file, which
all 22 lessons share.

---

## How a lesson is put together

```
NN_Topic/
├── Main.c          the lesson source
├── config.h        F_CPU, BAUD, and any board map this lesson needs
├── Slide.md        the lecture deck
├── README.md       objectives, what it touches, how to run it
├── build.bat       names its libraries, calls the shared engine
└── simulate.bat    calls the shared simulator launcher
```

`build.bat` is three lines. The work is in `_build/build-lesson.bat`, so a change
to the compile flags happens once for all 22 lessons:

```bat
set LIBS=_adc _glcd _init _port
call "%~dp0..\_build\build-lesson.bat"
```

### Why `LIBS` is per-lesson
Linking every shared library into every lesson does not work: several lessons
deliberately define their own ISR or UART routine that a library also defines,
and the link fails with a duplicate symbol. `_build/resolve-libs.py` works out
each lesson's minimal conflict-free set from the object symbol tables:

```
python _build/resolve-libs.py --all
```

### Compile flags
```
-mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600
-Os -Wall -Wextra -ffunction-sections -fdata-sections -Wl,--gc-sections
```

`config.h` defines `F_CPU` behind an `#ifndef`, so the header and the compiler
flag can never disagree — a real problem in the previous edition, where five
lessons built at 7.3728 MHz while their header claimed 16 MHz.

---

## Tooling in `_build/`

| Script | What it does |
|--------|--------------|
| `build-lesson.bat` | The shared compile/link/hex engine |
| `simulate-lesson.bat` | Places the board next to the lesson's hex and launches SimulIDE |
| `resolve-libs.py` | Derives each lesson's minimal shared-library set |
| `verify-all.ps1` | Builds all 22 lessons and validates every `Main.hex` |
| `build-slides.py` | Renders every `Slide.md` into a presentable HTML deck |
| `gen-readme.py` | Regenerates every lesson README from its source |
| `migrate.py` | The one-time importer from `../projects` |
| `remap-leds.py` | One-time: moved status LEDs from unwired PORT C to the board's PORT B |
| `fix-mojibake.py` | One-time: repaired text damaged by an old encoding round-trip |

`migrate.py`, `remap-leds.py` and `fix-mojibake.py` have already run. This tree
is the source of truth now; they are kept as a record of what was changed and
why, not as a pipeline to re-run.

---

## What changed from `../projects`

- **Clock unified at 16 MHz.** Five lessons previously disagreed with themselves
  between `build.bat` and `config.h`; two of those had an unguarded `#define`
  that silently overrode the compiler flag, so delays and baud rates were
  computed for the wrong clock.
- **One build engine** instead of six differently-shaped batch files, plus
  `Makefile`s in a couple of projects and none at all in two others.
- **Status LEDs moved to the board's LED bank.** Six lessons drove PORT C, which
  this board does not connect.
- **Serial-port pin conflict fixed.** The debounce lesson used PD2 as a button;
  PD2 is RXD1 and carried its own output. It now uses PD4.
- **Real defects fixed** — `sprintf` overflows in the keypad and LCD lessons, a
  `%u` given a `long` in the servo lesson, a `PROGMEM` array declared without
  `static` so the attribute was ignored and `lpm` read the wrong address space,
  and a status print in the ring-buffer lesson that repeated forever.
- **Garbled text repaired.** 310 sequences damaged by an earlier UTF-8/CP949
  round-trip, including banner lines whose `\r` had lost its backslash.
- **Slides completed.** Four lessons had none; all 22 now do.
- **Stray sources dropped** — `Main copy.c`, `Main_old.c`, `Main_Simple.c`.

Every lesson builds clean. The only remaining compiler warnings are
`defined but not used` on the demo functions students are meant to uncomment.

---

## Verifying the whole course

```
pwsh _builderify-all.ps1                  # build all 22, validate each hex
pwsh _builderify-all.ps1 -ShowWarnings    # and list lesson-code warnings
pwsh _builderify-all.ps1 -Clean           # leave the tree as committed
```

It checks more than the exit code. Each `Main.hex` is validated as real Intel
HEX — records begin with `:`, an end-of-file record is present, and there is a
plausible amount of code. That matters: a batch file's `echo ... -> Main.hex`
is a *redirect*, and that bug once left every lesson with a 13-byte `Main.hex`
containing the words "Build OK" while every existence check still passed.

## Lecture slides

```
python _builduild-slides.py
```

Renders all 22 `Slide.md` files into self-contained HTML decks under
`_slides/`, with an index. Arrow keys or space to move, `o` for an overview
grid, `p` to print or save as PDF. No third-party packages — the Markdown
subset these decks use is rendered by the script itself.

Re-run it after editing a deck.

## Requirements

Nothing to install. Everything is vendored:

- `../tools/avr-toolchain` — avr-gcc 15.1.0, avr-objcopy, avr-size
- `../tools/simulide110sr2/SimulIDE_1.1.0-SR2_Win64` — the simulator
- `../shared_libs` — the framework's peripheral libraries

Python 3 is needed only for `build-slides.py`, `gen-readme.py` and
`resolve-libs.py` — not to build or run a lesson.
