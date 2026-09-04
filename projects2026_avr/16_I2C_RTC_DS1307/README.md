# 16. I2C and a Real-Time Clock

TWI start, stop and acknowledge, and talking to a DS1307.

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

- **PORTB** - 8 LEDs (active low), stepper coils, SPI bus, logic analyser, via `LED_WRITE` in config.h

**Peripherals**

- USART1 (serial, 9600 8N1)
- TWI / I2C

**Shared libraries linked**

- `_uart.c`

## Notes
- The original lesson drove status LEDs on PORT C, which is not connected on this board. They now go through `LED_WRITE` in `config.h`, which targets the board's PORT B bank and handles its active-low wiring.
- **The DS1307 on the board is not wired yet — do this first.**
  The component is placed but has no connections, so the RTC demos will report
  no response until you draw two wires in SimulIDE. It is a five-minute job and
  a reasonable lab exercise in its own right:

  1. Run `simulate.bat` and find the **DS1307** near the top left of the board.
  2. The ATmega128's TWI pins are **PD0 = SCL** and **PD1 = SDA**. Both already
     carry a push button and a pull-up resistor to 5 V — that pull-up is exactly
     what an I2C bus needs, so nothing further is required there.
  3. Drag from the DS1307's **SCL** pin to the wire already running from
     **PD0**, and from its **SDA** pin to the wire from **PD1**. SimulIDE
     inserts a junction node where you drop the wire onto an existing one.
  4. **File > Save Circuit** to keep the change.

  This is deliberately left as a manual step: the two junction points on PD0 and
  PD1 are already at their three-connection limit, so adding the RTC means
  splicing new nodes into existing wires. That is safe to do by hand in the GUI
  and risky to do by editing the circuit file, which every lesson shares.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
