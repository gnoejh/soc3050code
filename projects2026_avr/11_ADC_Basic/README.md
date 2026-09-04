# 11. Analog-to-Digital Conversion

ADMUX, ADCSRA, reference selection, and 10-bit conversion.

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
- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser
- **PORTD** - PD0/PD1/PD4-PD7 push buttons; PD2/PD3 are RXD1/TXD1
- **PORTE** - PE4-PE7 GLCD control and INT4-INT7; PE3 joystick
- **PORTF** - ADC inputs - PF0 potentiometer, PF1/PF2 joystick axes

**Peripherals**

- ADC
- KS0108 graphic LCD

**Shared libraries linked**

- `_adc.c`
- `_glcd.c`
- `_init.c`
- `_port.c`

## Notes
- Nothing lesson-specific; the shared board covers everything this lesson needs.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
