# 0. Embedded Processors and the ATmega128

What an embedded processor is, how the toolchain builds firmware, and how the ATmega128 is put together.

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
- **PORTD** - PD0/PD1/PD4-PD7 push buttons; PD2/PD3 are RXD1/TXD1, via the `BTN_` macros in config.h

**Peripherals**

- USART1 (serial, 9600 8N1)

**Shared libraries linked**

- None - this lesson is self-contained.

## Notes
- **Start here.** This lesson exists to prove the toolchain, the board and the
  serial monitor all work before any of them is load-bearing.
- It links **nothing** from `shared_libs`, on purpose: every line in `Main.c` is
  one you can read on the first day.
- The LEDs are on PORT B and are **active low** - `LED_WRITE` in `config.h`
  inverts, so a set bit means a lit LED.
- **If the LEDs walk but no banner appears**, the Serial Monitor is on the wrong
  settings or the wrong component. It must be 9600 baud, 8N1, on the serial
  component wired to PD2/PD3.
- **If the banner is garbage**, `F_CPU` and the board disagree. Both must be
  16 MHz; `build.bat` passes `-DF_CPU=16000000UL` and the MCU in the circuit is
  set to `Frequency="16 MHz"`.
- **If nothing happens at all**, check `Main.hex` exists and starts with `:`.
  A batch redirect can leave a file that passes an existence check and contains
  no firmware - see `_build/verify-all.ps1`.

## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
