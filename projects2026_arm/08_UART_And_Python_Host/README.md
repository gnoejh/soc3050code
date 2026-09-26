# 08 — UART and a Python Host

**Part 1, lesson 5.**
Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz).

USART2 driven by interrupts in both directions — receive through the ISR into
a kernel queue that a task blocks on, transmit through a lock-free ring drained
by the TXE interrupt — and a line protocol with two audiences: a shell for
people (`led 170`) and NMEA-style checksummed frames for programs
(`$LED,170*57`). `host.py`, the course's first Python, checks, charts, builds
and sends those frames.

## Files

| | |
|---|---|
| `Slide.md` | the lecture, a title and 15 slides |
| `Lab.md` | the lab: seven parts, ~2 hours, **nothing handed in** |
| `uart.c`, `uart.h` | **the subject of the lesson.** ISR with ORE/RXNE/TXE, RX kernel queue, TX SPSC ring, strong `_write()` |
| `proto.c`, `proto.h` | the frame format: XOR checksum and frame check. Pure C — no hardware |
| `Main.c` | three tasks: protocol (shell + frames), telemetry, buttons; a mutex so whole lines never interleave |
| `host.py` | the PC side. `--selftest`, `--frame`, `--file`, `--port`, `--send`, `--demo`, `--plot` |
| `build.bat` | `LIBS=retarget os` |
| `simulate.bat` | always rebuilds, then opens the Wokwi board and explains both Python routes |
| `diagram.json` | lesson 05's board: LED bar and buttons A/B |
| `wokwi.toml` | also sets `rfc2217ServerPort = 4000` — the live route for `host.py` via the VS Code extension |

**Shared-code changes that came with this lesson:**

- `_lib/os.c`, `_lib/os.h` — lesson 07's kernel, copied once it had a second
  user, exactly as `_startup/` was copied from lesson 04. `07_RTOS/` keeps its
  own copy because there it is the subject.
- `_lib/retarget.c` — `uart2_init()` and `_write()` are now **weak**, so a
  lesson can replace them by defining its own. This one does. Lessons 05–07
  rebuild to identical sizes.

## Build and run

```
build.bat        # FLASH 10840 B / 32 KB, RAM 6504 B / 12 KB, zero warnings
simulate.bat
python host.py --selftest
```

## Python and Wokwi — the three routes

The browser's serial port cannot be opened from the PC. So:

| Route | How |
|---|---|
| **browser** (every student) | copy the serial monitor's text to `capture.txt`; `host.py --file capture.txt`. Frames to send: `host.py --frame "LED,170"` and paste |
| **Wokwi VS Code extension** | `wokwi.toml` forwards USART2 as RFC2217 on port 4000; `host.py --port rfc2217://localhost:4000` (documented by Wokwi; not yet exercised here) |
| **real Nucleo** | `host.py --port COM5` |

The protocol is printable ASCII **because** of the first route: a checksummed
text line survives a copy and paste; a binary protocol would not.

## Verified by execution — in Renode, not yet Wokwi

Run with the lesson 07 harness (Renode 1.17.0, STM32F072 Cortex-M0; `Main.c`,
`uart.c`, `proto.c` and `_lib/os.c` unchanged; test-only `SystemInit()`). Input
was typed into USART2 with Renode's `WriteChar`; output was captured and given
to `host.py --file`.

| Test | Result |
|---|---|
| session: `help`, `led 170`, `$LED,85*6C`, `$LED,85*00`, `$LED,85`, `bogus`, `$RATE,250*19`, `$STAT*12`, `stats` | help text; `bar = 0xAA`; `$ACK,LED,85*09`; `$NAK,CHECKSUM*65`; `$NAK,NOSTAR*7D`; `unknown command`; `$ACK,RATE,250*7C` and telemetry moves to 250 ms; `$STAT,71,3,2,0,0,0*25`; 0 overruns, 0 drops |
| `host.py --file` on that session | **frames good 9, bad 0, lost 0** — every C-built frame passed the Python check |
| `TXEIE` left on (Lab 5a) | **nothing printed, not even the banner** |
| print lock removed (Lab 5b) | one warning (`print_lock` unused); no visible interleaving — Renode's UART is instantaneous, so the lab makes this a reasoning question |
| `RX_DEPTH` 4, a pasted line (Lab 4) | `rx dropped 29`, `overruns 0`, and the shell executed the remains as `bar = 0x00` |
| seventh `$TEL` field (Lab 6) | `$TEL,1000,0,500,0,0,1*68`; unchanged `host.py` reads it: good 2, bad 0 |

`host.py --selftest` passes: it accepts the reference GPS sentence
`$GPGGA,...*47`, produces `$LED,170*57`, and rejects the one-bit change
`$LED,171*57`. (Writing this README caught an error: the example frame had
first been written as `*62`; the self-test said `57`, and it was right.)

**Still to do:** watch it in Wokwi — whether its serial monitor passes typed
and pasted input to USART2 as expected, and how fast a paste arrives (Lab
Part 4); and exercise the RFC2217 live route.

## Registers and peripherals touched

| | |
|---|---|
| `RCC->APBENR1` | USART2 clock gate |
| `GPIOA` | PA2/PA3 AF1 (USART2), PA0/PA1 buttons |
| `USART2` | `BRR` 417, `CR1` UE/TE/RE/RXNEIE (+TXEIE on demand), `ISR`, `ICR.ORECF`, `RDR`, `TDR` |
| NVIC | `USART2_IRQn` 28, priority 1 — above the kernel's 3 |
| `GPIOB` | the LED bar, set by `led` / `$LED` |
