# 12. Light Sensing with a CDS Cell

Voltage dividers and scaling raw counts into engineering units.

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

- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser

**Peripherals**

- None beyond the core CPU.

**Shared libraries linked**

- `_adc.c`
- `_init.c`
- `_port.c`
- `_uart.c`
- `_glcd.c`

## Notes
- Nothing lesson-specific; the shared board covers everything this lesson needs.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
