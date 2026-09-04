# This tree is archived

These 53 projects are the previous edition of the course. They are kept because
a fair amount of documentation still refers to them and a few contain material
that has not been carried across yet.

**For teaching and coursework, use [`../projects2026_avr/`](../projects2026_avr/).**

## Why it was replaced

The 2026 edition is a curated set of 22 lessons — one per topic instead of
several overlapping variants — normalised so that every lesson builds the same
way, runs at the same clock, and works on the same simulator board.

Problems in this tree that the new edition fixes:

- **The clock is inconsistent.** Across the `build.bat` files, 28 use 16 MHz,
  22 use 7.3728 MHz and one is empty. Five projects disagree *with themselves*
  between `build.bat` and `config.h`; in `ADC_Basic` and `Timer_PWM_Motor_DC`
  the header's `#define` is unguarded and silently overrides the compiler flag,
  so `_delay_ms()` and UART baud rates are computed for the wrong clock.
- **Status LEDs are on PORT C**, which the simulator board does not connect, so
  several projects show nothing when simulated.
- **`Port_Button_Debounce_Simple` uses PD2 as a button**, but PD2 is RXD1 and
  carries that project's own serial output.
- **Real defects**: `sprintf` buffer overflows in the keypad and LCD projects,
  a `%u` given an `unsigned long`, a `PROGMEM` array declared without `static`
  so the attribute is ignored and `lpm` reads a RAM address as flash.
- **Text damaged by an old encoding round-trip** — garbled box-drawing in the
  serial banners, including lines where the backslash of `\r` was eaten.
- **Only 3 of 53 projects ship a circuit**, and **19 have no slides**.
- **Six different build script shapes**, plus two projects with none at all.

None of these are fixed here. They are fixed in `../projects2026_avr/`.

## Building something in this tree anyway

The VS Code **Build Current Project** task still works: it detects which tree
the open file is in and falls back to the legacy `cli-build-project.ps1` and the
SimulIDE 1.1.0-SR1 launcher for anything under `projects/`.

Be aware that the legacy builder guesses which shared libraries to link from the
filenames present in the folder, and one of its branches references
`_interrupt_manager.c`, which does not exist in `shared_libs/`.

## Material not yet carried across

Topics that exist here but not in the 2026 edition, if you want to bring one
over: pin-change interrupts, rotary encoder, I2C multi-sensor, SPI multi-device
and external SPI EEPROM, the software RTC and stopwatch, USART binary protocol
and command parser, the keypad calculator application, the ESP01 WiFi project,
and the low-power sensor and wake-up optimisation variants.

Adding one means creating the folder under `projects2026_avr/`, then running:

```
python _build/resolve-libs.py <lesson>     # work out its shared_libs set
python _build/gen-readme.py                # regenerate the lesson README
pwsh _build/verify-all.ps1                 # confirm the tree still builds
```
