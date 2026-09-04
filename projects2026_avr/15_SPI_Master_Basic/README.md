# 15. SPI Master

SPCR, clock modes, and full-duplex shift-register transfer.

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

This lesson prints to the serial port, so open the **Serial Monitor** at 9600 baud, 8N1. The board's serial component is wired to PD2 (RXD1) and PD3 (TXD1).

## What this lesson touches

**Ports**

- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser
- **PORTC** - not connected on this board

**Peripherals**

- USART1 (serial, 9600 8N1)
- SPI
- Timer1

**Shared libraries linked**

- `_uart.c`

## Notes
- This lesson keeps its status LEDs on PORT C, which the board does not connect. PB0-PB3 carry its SPI bus, so the LED bank cannot be driven without corrupting the transfer. Watch the serial output instead.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
