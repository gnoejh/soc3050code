# 07 — RTOS

**Part 1, lesson 4.**
Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz).

A preemptive kernel written from scratch — `os.c`, about 300 lines of C and a
34-instruction PendSV context switch — and two programs that use it.
Scenario 1 is a small system in which a task that never yields shares the CPU
with an LED bar, buttons and a report that prints each task's CPU share and
stack high-water mark. Scenario 2 reproduces priority inversion and measures
what priority inheritance does about it.

## Files

| | |
|---|---|
| `Slide.md` | the lecture, a title and 20 slides |
| `Lab.md` | the lab: seven parts, ~2 hours, **nothing handed in** |
| `os.c`, `os.h` | **the subject of the lesson.** Tasks, the scheduler, SysTick, PendSV, sleep, mutex with optional priority inheritance, queue (with an ISR-safe put), stack high-water marks |
| `Main.c` | `SCENARIO 1` or `2`, the per-task report, and a HardFault handler that prints the faulting `pc` and the `addr2line` command for it |
| `build.bat` | `LIBS=retarget`; `os.c` is compiled because it is in the folder |
| `simulate.bat` | always rebuilds, then opens the Wokwi board |
| `diagram.json` | lesson 05's board: LED bar PB0–PB7, button A on PA0 (speed), B on PA1 (hog on/off) |
| `wokwi.toml` | for the Wokwi VS Code extension only |

`os.c` lives in the lesson folder, not `_lib/`, for the same reason lesson
04's `startup.c` did: here it is what the student reads and changes. Lesson 08
uses it from a shared copy.

## Build and run

```
build.bat        # FLASH 7472 B / 32 KB, RAM 6128 B / 12 KB, zero warnings
simulate.bat
```

RAM is ~6 KB because every task owns a stack: 128 + 128 + 128 + 512 words,
plus idle's 64.

## Design, briefly

- Priorities: **higher number = more urgent** (FreeRTOS's convention, the
  opposite of the NVIC's). Idle is 0 and always ready. Equal priorities
  round-robin one tick at a time.
- SysTick and PendSV both at NVIC priority 3, the lowest, so a switch never
  happens inside another handler; every other interrupt should be 0–2.
- Tasks run on PSP; handlers on MSP. Every switch returns with `0xFFFFFFFD`.
- `schedule()` **cancels** a pending PendSV when the current task turns out to
  be the right one after all — otherwise a switch requested earlier would go to
  a task chosen on stale information.
- Kernel critical sections save and restore `PRIMASK`, so they nest and are
  safe from handlers.
- Inheritance is correct for one mutex held at a time, and the slide says so.

## Verified by execution — in Renode, not yet Wokwi

Unlike lessons 05 and 06, this lesson's behaviour was **run**, not just built.
Wokwi cannot be driven from the command line here, so the kernel ran in
**Renode 1.17.0 on its STM32F072 platform** — a Cortex-M0, the same ARMv6-M
architecture, with SysTick, PendSV and the NVIC modelled — using:

- `os.c` and `Main.c` **unchanged**;
- a test-only `startup.c` whose `SystemInit()` skips the C0's `FLASH->ACR` and
  `HSIRDY` polls (Renode tags the F0 flash interface as a zero-reading stub,
  and the F0 RCC model's `HSIRDY` is not at the C0's bit);
- the C0's GPIO block at `0x50000000` mapped as plain memory, with `IDR`
  written to simulate button presses;
- the F0's USART2, which is at the C0's address with the C0's register layout,
  so `printf` needed nothing.

What was observed (all quoted in `Slide.md` and `Lab.md`):

| Test | Result |
|---|---|
| scenario 1, 5 s, B pressed at 3.2 s | ~420 switches/s; hog 100% while bar and buttons keep working; after B, hog → sleeping and idle 80% → 100% |
| hog raised to priority 4 | banner, then nothing — total starvation |
| forged `xPSR` with T = 0 | HardFault in `button` at `0x08000228`, its first instruction |
| scenario 2, inheritance on / off | high's worst wait **10 ms / 110 ms**; waits over 40 ms **0 / 6** in 49 |
| report stack 96 words (needs 115) | no crash; `OVERFLOW` flagged; `primes` (at `0x20000084`, just below `stk_report`) corrupted to `0x20000062` |
| `__builtin_trap()` in the hog | HardFault report naming `hog`, `pc` resolving via `addr2line` to the trap line |
| unaligned word store | **no fault in Renode** — silicon would fault; hence the lab uses the trap, and asks what Wokwi does |
| Lab Part 6 worked answer (queue logger) | logger blocked at 0% until presses; logged `button A at 1300 ms`, `button B at 1750 ms`; stack 89 words |

**Still to do:** watch it in Wokwi — in particular whether Wokwi's STM32C0
model handles PSP/PendSV exactly as Renode's Cortex-M0 does, and whether it
faults on the unaligned store (Lab Part 5's open question).

## Registers and peripherals touched

| | |
|---|---|
| `SysTick` | 1 kHz tick, priority 3 (via `SysTick_Config()`) |
| `SCB->ICSR` | `PENDSVSET` / `PENDSVCLR` — requesting and cancelling a switch |
| NVIC | PendSV priority 3 |
| core registers | `PSP`, `PRIMASK`; `EXC_RETURN` `0xFFFFFFFD` |
| `GPIOA`, `GPIOB` | buttons (PA0, PA1) and the bar (PB0–PB7) |
| `USART2` | via `_lib/retarget.c`, polled — only the report task prints |
