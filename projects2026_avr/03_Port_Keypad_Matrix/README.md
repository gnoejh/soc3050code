# 3. Matrix Keypad Scanning

Row and column scanning of a 4x4 keypad without dedicated hardware.

Part of the SOC3050 ATmega128 course, 2026 AVR edition. Runs on the shared
SimulIDE 1.1.0-SR2 board at 16 MHz.

## Files

| File | What it is |
|------|------------|
| `Main.c` | The lesson source |
| `config.h` | `F_CPU`, `BAUD` and this lesson's board map |
| `Slide.md` | The lecture deck for this topic |
| `build.bat` | Builds `Main.hex` |
| `simulate.bat` | Builds if needed, then opens the shared board in SimulIDE |

## Build and run

```
build.bat        produces Main.elf and Main.hex
simulate.bat     opens the shared board with this lesson's firmware loaded
```

In SimulIDE press **Play** to start the simulation.

## What this lesson touches

**Ports**

- **PORTA** - KS0108 graphic LCD data bus
- **PORTG** - PG1 GLCD control, PG4 slide switch
- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser, via `LED_WRITE` in config.h

**Peripherals**

- None beyond the core CPU.

**Shared libraries linked**

- `_uart.c`

## Notes
- The original lesson drove status LEDs on PORT C, which is not connected on this board. They now go through `LED_WRITE` in `config.h`, which targets the board's PORT B bank and handles its active-low wiring.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
