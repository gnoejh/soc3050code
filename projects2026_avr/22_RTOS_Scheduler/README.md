# 22. A Small Cooperative RTOS

Task control blocks, a tick-driven scheduler, and context switching.

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
| `QUICK_REFERENCE.md` | Supporting handout |

## Build and run

```
build.bat        produces Main.elf and Main.hex
simulate.bat     opens the shared board with this lesson's firmware loaded
```

In SimulIDE press **Play** to start the simulation.

This lesson prints to the serial port, so open the **Serial Monitor** at 9600 baud, 8N1. The board's serial component is wired to PD2 (RXD1) and PD3 (TXD1).

## What this lesson touches

**Ports**

- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser
- **PORTD** - PD0/PD1/PD4-PD7 push buttons; PD2/PD3 are RXD1/TXD1
- **PORTF** - ADC inputs - PF0 potentiometer, PF1/PF2 joystick axes
- **PORTG** - PG1 GLCD control, PG4 slide switch

**Peripherals**

- USART1 (serial, 9600 8N1)
- ADC
- Timer0
- Timer1
- KS0108 graphic LCD

**Shared libraries linked**

- `_glcd.c`

## Notes
- Nothing lesson-specific; the shared board covers everything this lesson needs.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
