# 13. Serial Communication

UBRR and baud rate, framing, and polled transmit and receive.

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
| `LAB_GUIDE.md` | Supporting handout |
| `UART_INTERRUPT_FLOW_DIAGRAMS.md` | Supporting handout |
| `UART_REGISTER_QUICK_REFERENCE.md` | Supporting handout |

## Build and run

```
build.bat        produces Main.elf and Main.hex
simulate.bat     opens the shared board with this lesson's firmware loaded
```

In SimulIDE press **Play** to start the simulation.

This lesson prints to the serial port, so open the **Serial Monitor** at 9600 baud, 8N1. The board's serial component is wired to PD2 (RXD1) and PD3 (TXD1).

## What this lesson touches

**Ports**

- **PORTA** - KS0108 graphic LCD data bus
- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser

**Peripherals**

- USART1 (serial, 9600 8N1)

**Shared libraries linked**

- None - this lesson is self-contained.

## Notes
- Nothing lesson-specific; the shared board covers everything this lesson needs.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
