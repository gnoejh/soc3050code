# 05 — GPIO and Interrupts

**Part 1, lesson 2.**
Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz).

The first peripheral, taken all the way to an interrupt. Two buttons do the
same job two ways — PA0 polled at 1 kHz with a debouncer, PA1 through EXTI and
the NVIC — and the serial monitor prints how many transitions each method saw
for every press and release. Wokwi's pushbuttons bounce, so the numbers differ,
and that difference is the lesson's measurement.

## Files

| | |
|---|---|
| `Slide.md` | the lecture, a title and 34 slides in five parts (the renderer counts 35), every register explained with worked values — rendered to `_slides/05_GPIO_And_Interrupts.html` |
| `Lab.md` | the lab: eight parts, ~2 hours, guided, **nothing handed in** |
| `Main.c` | the lab program — GPIO helpers, the debouncer, the EXTI setup, the handler, and a banner that reads the whole configuration back from the registers |
| `build.bat` | one command; `LIBS=retarget` |
| `simulate.bat` | always rebuilds, then opens the Wokwi board |
| `diagram.json` | the board, the eight-LED bar on PB0–PB7 (as in lesson 04), button A on PA0 (key **A**) and button B on PA1 (key **B**). **Required** — the browser route needs it pasted into the Wokwi tab |
| `wokwi.toml` | for the Wokwi VS Code extension only |

**No `startup.c`, no `link.ld`, no `retarget.c`.** Lesson 04 carried all three
because they were its subject. From this lesson on the engine links the shared
defaults: `_startup/c031c6/startup.c` and `link.ld` (lesson 04's, with the lab
exercises removed), and `_lib/retarget.c` via `LIBS=retarget`.

## Build and run

```
build.bat        # FLASH 7156 B / 32 KB, RAM 2008 B / 12 KB, zero warnings
simulate.bat     # rebuilds, then opens wokwi.com on the Nucleo-C031C6
```

Slides 27, 28 and 30 quote this build's addresses and sizes literally — vector
21 = `0x0800023D`, the handler at `0x0800023c`, FLASH 7156 B against 7120 B
misspelt — so **if `Main.c` changes, re-derive them** (`objdump -s -j
.isr_vector`, `nm`, `objdump --disassemble=EXTI0_1_IRQHandler`).

## Registers and peripherals touched

| | |
|---|---|
| `RCC->IOPENR` | GPIOA and GPIOB clock gates |
| `GPIOA->MODER`, `PUPDR`, `IDR`, `BSRR` | PA0/PA1 input + pull-up (buttons), PA5 output (LD4 heartbeat) |
| `GPIOB->MODER`, `BSRR` | PB0–PB7 output — the bar, written with one `BSRR` store |
| `EXTI->EXTICR[0]`, `RTSR1`, `FTSR1`, `IMR1`, `RPR1`, `FPR1` | line 1 on port A, both edges, unmasked; pending cleared by writing 1 |
| NVIC | IRQ 5 (`EXTI0_1`), priority 2, enabled; handler `EXTI0_1_IRQHandler` |
| `SysTick` | polled 1 ms tick, `COUNTFLAG` — still no SysTick interrupt; lesson 06 |
| `USART2` | via `_lib/retarget.c`, polled transmit |

## What the lab does

| Part | Change | What it demonstrates |
|---|---|---|
| 0 | build, run, read the banner, press both buttons | polled vs interrupt counts of the same bounce |
| 1 | delete PA0's `MODER` input write | pins reset **analog**; analog reads 0 = "pressed" forever |
| 2 | `"bounce": "0"` on button B | a clean switch is exactly 1 edge each way; everything above that was bounce |
| 3 | delete the pending-bit clear | the handler tail-chains forever; board freezes on the first press |
| 4 | misspell the handler | **zero warnings**, 36 B smaller, `nm` shows `W`, banner says NOT INSTALLED |
| 5 | delete `NVIC_EnableIRQ` | EXTI latches (`FPR1` = 1 forever), NVIC never delivers |
| 6 | `bar_write()` via `ODR` instead of `BSRR` | smaller, one store — and clobbers PB8–PB15 |
| 7 | **a third button on PC13**, open-ended | every hop again, plus GPIOC's clock and `EXTICR` code 2 — which button B never needed because port A is `EXTICR`'s reset value |

**Build-verified, 2026-09-26.** Every change in Parts 1 and 3–7 was built from
a copy of this folder and checked: all build with **zero warnings**; Part 4
gives FLASH 7120 B with `EXTI0_1_IRQHandler` listed `W` at `Default_Handler`'s
address and the misspelt function gone; Part 6's `ODR` version is 8 B smaller
and disassembles to one `str r0, [r3, #20]`; a worked Part 7 builds to 7316 B
FLASH / 2016 B RAM with `T EXTI4_15_IRQHandler`.

**Facts checked against ST's sources, not memory:** `GPIOA_MODER` reset value
`0xEBFFFFFF` (ST's STM32C031 SVD); `EXTICR` port codes A=0, B=1, C=2, D=3, F=5
(the SVD's enumerated values and `LL_EXTI_CONFIG_PORTx` in ST's C0 LL driver);
`__NVIC_PRIO_BITS = 2` and IRQs 0–28 (`stm32c031xx.h`). Wokwi's bounce model —
10 to 100 transitions over about 1 ms, disabled by `"bounce": "0"` — is from
its `wokwi-pushbutton` documentation. **Wokwi's board file defines no
pushbutton** — it models LD3 and LD4 and nothing else — so the Nucleo's user
button B1 is not available; the lesson uses external `wokwi-pushbutton`
parts.

## Status

- Builds clean, zero warnings, with the vendored toolchain.
- **Not yet watched running in Wokwi.** Every runtime outcome in `Slide.md`
  slide 32 and `Lab.md` — the edge counts, the Part 1 analog read, the Part 3
  freeze, the Part 5 `FPR1` readout, whether Wokwi reports `MODER` at reset as
  `0xEBFFFFFF` — is predicted from the silicon's documented behaviour and
  Wokwi's documentation, and the text says so where it matters. Per `CLAUDE.md`
  §9e the lesson is not finished until someone has watched it: paste
  `diagram.json`, upload `Main.elf`, press both buttons, and confirm the banner
  and the counts.
