# 1. Digital I/O Fundamentals

DDR, PORT and PIN registers; driving LEDs and reading switches.

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
| `GPIO_FLOW_DIAGRAMS.md` | Supporting handout |
| `GPIO_REGISTER_QUICK_REFERENCE.md` | Supporting handout |

## Build and run

```
build.bat        produces Main.elf and Main.hex
simulate.bat     opens the shared board with this lesson's firmware loaded
```

In SimulIDE press **Play** to start the simulation.

## What this lesson touches

**Ports**

- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser
- **PORTD** - PD0/PD1/PD4-PD7 push buttons; PD2/PD3 are RXD1/TXD1

**Peripherals**

- None beyond the core CPU.

**Shared libraries linked**

- None - this lesson is self-contained.

## Notes
- Nothing lesson-specific; the shared board covers everything this lesson needs.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
