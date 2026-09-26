# GPIO and Interrupts: One Pin, Two Ways to Listen
## SOC3050 ARM Edition — Part 1, Instances

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](https://www.st.com/resource/en/user_manual/um2953-stm32c0-nucleo64-board-mb1717-stmicroelectronics.pdf)

**The first peripheral, taken all the way to an interrupt.**

Lesson 00 said every peripheral is the same shape: a clock gate, control
registers, status and data registers. Lesson 03 said there are three ways to
meet the hardware — poll, interrupt, DMA. This week both models get their
first real instance: a pin, and a button on it, heard two different ways.

Every figure on these slides is real output from this lesson's own build —
`05_GPIO_And_Interrupts/Main.elf`, on the Nucleo-C031C6.

---

## Slide 1: The Model, Filled In Once

Lesson 00 drew the peripheral model with empty slots. GPIO is the first time
you fill them:

| Model slot | GPIO on this chip |
|---|---|
| **clock gate** | `RCC->IOPENR`, one bit per port |
| **control** | `MODER` (what the pin is), `PUPDR` (pull-up/down), `OTYPER`, `OSPEEDR`, `AFR[2]` |
| **status / data in** | `IDR` — what the pin *is* |
| **data out** | `ODR` — what the pin *should be*; `BSRR`, `BRR` — change it without reading |
| **event** | not in GPIO at all — a separate block, **EXTI**, watches the pins |

> Every later peripheral fills the same table. When you meet the USART in
> lesson 08, the question is not "what are all these registers" but "which
> slot is each one".

The last row is the surprise, and the second half of this lesson.

---

## Slide 2: One Pin, Drawn

A pin is not a wire to the CPU. It is a small circuit, and `MODER` chooses
which part of it is connected.

```svg
<svg viewBox="0 0 580 260" role="img" aria-label="Inside one GPIO pin: output driver, input buffer, pull resistors, and the MODER selection">
  <defs><marker id="p2" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>

  <rect class="box" x="20" y="30" width="150" height="40" rx="5"/>
  <text x="95" y="48" text-anchor="middle" class="mono">ODR / BSRR</text>
  <text x="95" y="63" text-anchor="middle" class="lbl">data out</text>

  <rect class="box" x="20" y="170" width="150" height="40" rx="5"/>
  <text x="95" y="188" text-anchor="middle" class="mono">IDR</text>
  <text x="95" y="203" text-anchor="middle" class="lbl">data in</text>

  <rect class="hifill" x="230" y="30" width="130" height="40" rx="5"/>
  <text x="295" y="48" text-anchor="middle">output driver</text>
  <text x="295" y="63" text-anchor="middle" class="lbl">MODER = 01</text>

  <rect class="hifill" x="230" y="170" width="130" height="40" rx="5"/>
  <text x="295" y="188" text-anchor="middle">input buffer</text>
  <text x="295" y="203" text-anchor="middle" class="lbl">off when MODER = 11</text>

  <path class="wire" d="M170 50 H226" marker-end="url(#p2)"/>
  <path class="wire" d="M230 190 H174" marker-end="url(#p2)"/>

  <path class="wire" d="M360 50 H430 V120"/>
  <path class="wire" d="M430 120 V190 H364" marker-end="url(#p2)"/>

  <rect class="reg" x="400" y="98" width="60" height="44" rx="4"/>
  <text x="430" y="118" text-anchor="middle" class="lbl">pull-up</text>
  <text x="430" y="134" text-anchor="middle" class="lbl">PUPDR</text>

  <path class="wire" d="M430 120 H500"/>
  <circle cx="512" cy="120" r="12" class="hi" fill="none"/>
  <text x="512" y="150" text-anchor="middle" class="mono">PA1</text>
  <text x="512" y="166" text-anchor="middle" class="lbl">the pin</text>

  <text x="290" y="246" text-anchor="middle" class="lbl">MODER: 00 input · 01 output · 10 alternate function (a peripheral drives it) · 11 analog</text>
</svg>
```

Three things decide what a pin does, and all three are yours to set: which
path is connected (`MODER`), whether something holds it when nothing drives
it (`PUPDR`), and — only for outputs — how hard it drives (`OTYPER`,
`OSPEEDR`).

---

## Slide 3: The Registers, and Where They Are

`GPIOA` is at `0x50000000` (lesson 04, slide 3). Each register is a fixed
offset from the base, and every pin owns the same bits in each:

| Offset | Register | Bits per pin | Pin *n* lives at |
|---|---|---|---|
| `0x00` | `MODER` | 2 | bits `2n+1 : 2n` |
| `0x04` | `OTYPER` | 1 | bit `n` |
| `0x08` | `OSPEEDR` | 2 | bits `2n+1 : 2n` |
| `0x0C` | `PUPDR` | 2 | bits `2n+1 : 2n` |
| `0x10` | `IDR` | 1 | bit `n` — read only |
| `0x14` | `ODR` | 1 | bit `n` |
| `0x18` | `BSRR` | 1 + 1 | bit `n` sets, bit `n+16` resets — write only |
| `0x28` | `BRR` | 1 | bit `n` resets — write only |

The whole of this lesson's pin code is two helpers that follow the table:

```c
static void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (mode << (pin * 2u));
}
```

Clear the two bits, OR in the new value. `pin_pull()` is the same line with
`PUPDR`. **Check the bit positions against RM0490 yourself** — that habit is
the skill, not the helper.

---

## Slide 4: A Pin Wakes Up Analog

What does a pin do before you configure it? ST's own register description says
`GPIOA->MODER` resets to **`0xEBFFFFFF`**:

```
0xEBFFFFFF = 1110 1011 1111 1111 1111 1111 1111 1111
             PA15 PA14 PA13 PA12 ...                PA0
              11   10   10   11  ...                 11
```

**Every pin is `11` — analog — except PA13 and PA14**, which are `10`:
alternate function, because they are the SWD debug lines and the debugger
must reach the chip before your code has run.

Analog is the safe default: the input buffer is switched off, so a floating
pin draws no current. But an analog pin **reads 0 in `IDR`**, whatever voltage
is on it. For a button that reads 0 when pressed, that means *pressed,
forever*.

> So input mode is **written, never assumed**. `gpio_init()` sets `00`
> explicitly, and the banner prints `MODER` as it was at reset so you can
> check this slide against the simulator. Lab Part 1 deletes the line.

---

## Slide 5: Writing a Pin — `ODR` Reads First, `BSRR` Does Not

Lesson 01 promised this. Two ways to set PA5 high, compiled with this
lesson's flags:

```
GPIOA->ODR |= 1u << 5;           GPIOA->BSRR = 1u << 5;

movs r2, #160                    movs r3, #160
movs r3, #32                     movs r2, #32
lsls r2, r2, #23   = 0x50000000  lsls r3, r3, #23   = 0x50000000
ldr  r1, [r2, #20] <- read ODR   str  r2, [r3, #24] <- write BSRR
orrs r3, r1                      bx   lr
str  r3, [r2, #20] <- write ODR
bx   lr
```

The left is lesson 03 slide 5's lost update waiting to happen: if an
interrupt changes another PA pin between the `ldr` and the `str`, the `str`
puts the old value back. The right has **no read to be stale**. `BSRR`'s low
half sets pins, its high half resets them, and zeros mean *leave alone* — so
one store changes exactly the pins you name and no others.

This lesson writes the whole LED bar that way, eight pins in one store:

```c
GPIOB->BSRR = ((uint32_t)(uint8_t)~leds << 16) | leds;   /* one str */
```

Lesson 04 wrote `GPIOB->ODR = leds`, which also forced PB8–PB15 low. Nothing
is on them today. Something will be.

---

## Slide 6: Reading a Pin — Pull-Ups and Active Low

A button is a switch to ground. Pressed, it connects the pin to 0 V. Released,
it connects the pin to **nothing** — and a pin connected to nothing floats,
reading whatever noise it picks up.

```svg
<svg viewBox="0 0 580 200" role="img" aria-label="Button to ground with the internal pull-up: released reads 1, pressed reads 0">
  <text x="140" y="20" text-anchor="middle" class="lbl">RELEASED</text>
  <text x="140" y="40" text-anchor="middle" class="mono">3.3 V</text>
  <rect class="reg" x="125" y="50" width="30" height="40" rx="3"/>
  <text x="170" y="75" class="lbl">pull-up (inside the chip)</text>
  <path class="wire" d="M140 90 V120 H60"/>
  <text x="60" y="112" class="mono">PA1 = 1</text>
  <path class="wire" d="M140 120 V140"/>
  <path class="dash" d="M140 140 L165 160"/>
  <path class="wire" d="M140 165 V180"/>
  <text x="140" y="196" text-anchor="middle" class="lbl">GND — switch open</text>

  <text x="430" y="20" text-anchor="middle" class="lbl">PRESSED</text>
  <text x="430" y="40" text-anchor="middle" class="mono">3.3 V</text>
  <rect class="reg" x="415" y="50" width="30" height="40" rx="3"/>
  <path class="wire" d="M430 90 V120 H350"/>
  <text x="350" y="112" class="mono hi">PA1 = 0</text>
  <path class="hi" d="M430 120 V180"/>
  <text x="430" y="196" text-anchor="middle" class="lbl">GND — switch closed</text>
</svg>
```

`PUPDR = 01` switches on a resistor inside the chip that pulls the pin up when
nothing else drives it. Pressing the button overpowers it. So:

| Button | Pin | `button_down()` |
|---|---|---|
| released | 1 | false |
| pressed | **0** | true |

This is **active low**, and it is the normal way to wire a button — no
external resistor, and ground is available everywhere.

---

## Slide 7: A Press Is Not One Edge

Metal contacts do not close cleanly. They hit, bounce apart, and hit again,
for about a millisecond:

```svg
<svg viewBox="0 0 580 170" role="img" aria-label="A bouncing button: many transitions before the level settles low">
  <text x="20" y="40" class="lbl">1</text>
  <text x="20" y="120" class="lbl">0</text>
  <path class="wire" d="M40 36 H150 V116 H162 V36 H170 V116 H184 V36 H190 V116 H206 V36 H210 V116 H540"/>
  <path class="dash" d="M150 140 V150 M230 140 V150"/>
  <path class="dash" d="M150 145 H230"/>
  <text x="190" y="164" text-anchor="middle" class="lbl">bounce, ~1 ms</text>
  <text x="380" y="100" text-anchor="middle" class="hi">settled: pressed</text>
  <text x="95" y="28" text-anchor="middle" class="lbl">released</text>
</svg>
```

**Wokwi simulates this.** Its pushbutton, by default, produces 10 to 100
transitions over about a millisecond on every press — following Horowitz and
Hill's *The Art of Electronics*. Setting the part's `"bounce": "0"` attribute
turns it off, and Lab Part 2 does exactly that to compare.

A program that counts edges counts **bounces**. A program that says "the pin
went low, so that was a press" gets many presses from one. Debouncing is not
optional; it is part of reading a button.

---

## Slide 8: Button A — Poll, and Believe Only What Holds

Button A is read by the main loop once per millisecond. A new level is
believed only after it has held for 20 samples in a row:

```c
static int debounce_step(debounce_t *d, uint8_t raw)
{
    if (raw != d->raw) {                 /* the level moved: restart the clock */
        d->raw     = raw;
        d->held_ms = 0;
        d->changes++;
        return 0;
    }
    if (d->held_ms < DEBOUNCE_MS) {
        d->held_ms++;
        if (d->held_ms == DEBOUNCE_MS && raw != d->stable) {
            d->stable = raw;
            return raw ? +1 : -1;        /* press or release */
        }
    }
    return 0;
}
```

Bounce restarts the clock; only silence lets it finish. The cost is a fixed
20 ms delay between the press and the response — far below what a person
notices, and you can measure the trade-off by changing `DEBOUNCE_MS`.

---

## Slide 9: What Polling Costs, and What It Hides

Polling at 1 kHz has two properties worth saying out loud.

**It is a filter.** Bounce transitions are microseconds apart; samples are a
millisecond apart. Most bounces fall *between* two samples and are never seen.
So button A reports only one or two level changes per press — not because the
button is clean, but because the sampling is too slow to see the mess.

**It depends on the loop coming back.** Every millisecond the loop must
return to look. Anything slow in the loop — a `printf` at 115200 baud costs
about 87 µs *per character* — delays the next sample. For a button that is
harmless. For a pulse shorter than the loop period, it is invisible.

> Polling is simple, cheap and deterministic, and for a human-speed button it
> is often the right answer. It is the wrong answer when you must not miss an
> event, or when the CPU has better things to do than ask. That is where the
> interrupt comes in.

---

## Slide 10: Button B — The Interrupt Path, Three Hops

GPIO does not interrupt. A separate block, **EXTI** (extended interrupt and
event controller), watches pins and raises interrupts:

```svg
<svg viewBox="0 0 580 250" role="img" aria-label="Path from pin PA1 through EXTICR, EXTI edge detection and mask, NVIC, to vector 21">
  <defs><marker id="p10" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>

  <rect class="box" x="10" y="40" width="70" height="44" rx="5"/>
  <text x="45" y="67" text-anchor="middle" class="mono">PA1</text>

  <path class="wire" d="M80 62 H104" marker-end="url(#p10)"/>
  <rect class="reg" x="108" y="34" width="100" height="56" rx="5"/>
  <text x="158" y="56" text-anchor="middle" class="mono">EXTICR</text>
  <text x="158" y="74" text-anchor="middle" class="lbl">which port?</text>

  <path class="wire" d="M208 62 H232" marker-end="url(#p10)"/>
  <rect class="hifill" x="236" y="20" width="130" height="84" rx="5"/>
  <text x="301" y="40" text-anchor="middle">EXTI line 1</text>
  <text x="301" y="58" text-anchor="middle" class="mono lbl">RTSR1 FTSR1</text>
  <text x="301" y="74" text-anchor="middle" class="mono lbl">RPR1 FPR1 (latch)</text>
  <text x="301" y="92" text-anchor="middle" class="mono lbl">IMR1 (mask)</text>

  <path class="wire" d="M366 62 H390" marker-end="url(#p10)"/>
  <rect class="reg" x="394" y="34" width="80" height="56" rx="5"/>
  <text x="434" y="56" text-anchor="middle" class="mono">NVIC</text>
  <text x="434" y="74" text-anchor="middle" class="lbl">IRQ 5</text>

  <path class="wire" d="M474 62 H498" marker-end="url(#p10)"/>
  <rect class="box" x="502" y="34" width="70" height="56" rx="5"/>
  <text x="537" y="56" text-anchor="middle" class="mono">vec 21</text>
  <text x="537" y="74" text-anchor="middle" class="lbl">handler</text>

  <text x="158" y="130" text-anchor="middle" class="lbl">EXTICR[0]</text>
  <text x="158" y="146" text-anchor="middle" class="lbl">bits 15:8 = 0</text>
  <text x="301" y="130" text-anchor="middle" class="lbl">edge → pending bit</text>
  <text x="301" y="146" text-anchor="middle" class="lbl">you clear it</text>
  <text x="434" y="130" text-anchor="middle" class="lbl">ISER, IPR</text>
  <text x="434" y="146" text-anchor="middle" class="lbl">enable, priority</text>
  <text x="537" y="130" text-anchor="middle" class="lbl">0x08000054</text>
  <text x="537" y="146" text-anchor="middle" class="lbl">in FLASH</text>

  <text x="290" y="200" text-anchor="middle" class="hi">Every arrow has its own switch, and every switch defaults to OFF.</text>
  <text x="290" y="222" text-anchor="middle" class="lbl">Miss one and nothing happens — no error, no fault, no warning.</text>
</svg>
```

```c
EXTI->EXTICR[0] = (EXTI->EXTICR[0] & ~(0xFFu << 8)) | (EXTI_PORT_A << 8);
EXTI->RTSR1 |= 1u << 1;             /* rising edge  - release           */
EXTI->FTSR1 |= 1u << 1;             /* falling edge - press             */
EXTI->IMR1  |= 1u << 1;             /* unmask: let it reach the NVIC    */
NVIC_SetPriority(EXTI0_1_IRQn, 2);
NVIC_EnableIRQ(EXTI0_1_IRQn);
```

The banner reads each of these back from the hardware, one line per hop.

---

## Slide 11: EXTI on This Chip — Lines, Not Pins

EXTI has **16 lines for GPIO**, numbered 0–15, and line *n* can watch pin *n*
of **exactly one port**. `EXTICR` picks which:

| Code in `EXTICR` | 0 | 1 | 2 | 3 | 5 |
|---|---|---|---|---|---|
| Port | A | B | C | D | F |

Two consequences:

- **PA1 and PB1 cannot both interrupt.** They share line 1. Pin planning for
  interrupts is done by *number*, not by port.
- **Port A is code 0 — the reset value.** Forget `EXTICR` entirely and a
  port-A button works anyway. Move it to port C and it silently stops. The
  code that forgot was wrong all along; port A hid it.

**`EXTICR` is in EXTI on the STM32C0.** On an STM32F4 the same job is
`SYSCFG->EXTICR[]`, and code copied from an F4 tutorial will not compile here.
Worse would be code that compiles against a register that exists but does
something else — which is why the reference manual for *your* part is the only
one to trust.

Also C0-specific: rising and falling edges latch into **separate** pending
registers, `RPR1` and `FPR1`. Older families have one `PR`.

---

## Slide 12: The NVIC on a Cortex-M0+

The NVIC is ARM's, not ST's — it is identical on every M0+, and CMSIS's
`NVIC_EnableIRQ()` is the same function everywhere.

| On this core | |
|---|---|
| External interrupts | up to 32; the C031 uses IRQ 0–28 |
| Priority levels | **4** — `__NVIC_PRIO_BITS = 2` in ST's header |
| Enable / disable | `ISER` / `ICER` — write 1 to the bit, zeros do nothing |
| Pending | `ISPR` / `ICPR` — same convention |
| No | `BASEPRI`, sub-priority groups — those are M3 and up |

**Vectors are shared.** IRQ 5 serves EXTI lines 0 *and* 1, IRQ 6 lines 2–3,
IRQ 7 lines 4–15. One handler, many possible causes — so a handler must look
at the pending bits to find out which line fired, and must never assume.

The vector's address is arithmetic, not magic: 16 core exceptions come first,
so IRQ 5 is vector 21, and vector 21 is at `21 × 4 = 0x54` from the start of
FLASH. Read it out of this lesson's image:

```
$ arm-none-eabi-objdump -s -j .isr_vector --start-address=0x08000054 ...
 8000054 3d020008          ->  0x0800023D
$ arm-none-eabi-nm Main.elf | grep EXTI0_1
0800023c T EXTI0_1_IRQHandler
```

`0x0800023D` is `EXTI0_1_IRQHandler` plus the Thumb bit (lesson 01, slide 19).

---

## Slide 13: The Handler Is an Ordinary Function

This is the entire handler, and its entire disassembly:

```c
void EXTI0_1_IRQHandler(void)
{
    uint32_t mask = 1u << BTN_B_PIN;
    if ((EXTI->RPR1 | EXTI->FPR1) & mask) {
        EXTI->RPR1 = mask;
        EXTI->FPR1 = mask;
        b_edges++;
    }
}
```

```
0800023c <EXTI0_1_IRQHandler>:
 ldr  r3, [pc, #24]     ; r3 = EXTI = 0x40021800
 ldr  r1, [r3, #12]     ; RPR1
 ldr  r2, [r3, #16]     ; FPR1
 orrs r1, r2
 movs r2, #2            ; mask = 1 << 1
 tst  r1, r2
 beq  done
 str  r2, [r3, #12]     ; RPR1 = mask   (write 1 to clear)
 str  r2, [r3, #16]     ; FPR1 = mask
 ldr  r2, [pc, #12]     ; &b_edges
 ldr  r3, [r2, #0]
 adds r3, #1
 str  r3, [r2, #0]      ; b_edges++
done:
 bx   lr
```

No special prologue, no special return instruction. It uses only `r1`–`r3`,
which the hardware already stacked (lesson 03, slide 2), and it returns with an
ordinary `bx lr` — the `lr` the hardware loaded with a magic `EXC_RETURN`
value that tells the core to unstack instead of branching. **Fourteen
instructions** — with the hardware's own stacking on the way in and out, the
whole visit is on the order of a microsecond at 48 MHz.

---

## Slide 14: Write 1 to Clear — or Run Forever

The pending bit is the EXTI's memory that an edge happened. The NVIC sees the
line as long as the bit is set. So:

```svg
<svg viewBox="0 0 580 150" role="img" aria-label="If the pending bit is not cleared, the handler is re-entered immediately and main never runs again">
  <defs><marker id="p14" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="30" y="40" width="120" height="40" rx="5"/>
  <text x="90" y="65" text-anchor="middle" class="mono">edge → FPR1=1</text>
  <path class="wire" d="M150 60 H196" marker-end="url(#p14)"/>
  <rect class="hifill" x="200" y="40" width="150" height="40" rx="5"/>
  <text x="275" y="65" text-anchor="middle" class="mono">handler, no clear</text>
  <path class="wire" d="M350 60 H396" marker-end="url(#p14)"/>
  <rect class="box" x="400" y="40" width="150" height="40" rx="5"/>
  <text x="475" y="58" text-anchor="middle">return: still</text>
  <text x="475" y="74" text-anchor="middle">pending</text>
  <path class="hi" d="M475 80 V110 H275 V84" marker-end="url(#p14)"/>
  <text x="375" y="128" text-anchor="middle" class="hi">tail-chained straight back in — main() never runs again</text>
</svg>
```

**"Write 1 to clear"** is the convention: writing a 1 clears that bit, writing
0 leaves it alone. That is why the handler writes `mask`, not
`FPR1 &= ~mask` — the read-modify-write version would write 1s to *every*
pending bit it read, clearing edges on other lines that nobody has handled.

The symptom of forgetting is a board that works until the first press and then
freezes completely — heartbeat stops, serial goes quiet. The ISR is running
flat out. Lab Part 3.

---

## Slide 15: The Name Is the Only Connection

`startup.c` fills vector 21 with a **weak** alias of `Default_Handler`, an
infinite loop. Defining `EXTI0_1_IRQHandler` makes a **strong** symbol that
replaces it at link time. That is all that connects your function to the
hardware.

Misspell it — `EXTI0_1_IRQhandler`, one lower-case letter — and this lesson's
build reports:

| | Correct name | Misspelt |
|---|---|---|
| warnings | 0 | **0** |
| FLASH | 7156 B | **7120 B** |
| `nm` for `EXTI0_1_IRQHandler` | `T` (yours) | **`W`** (the weak alias) |

The 36 missing bytes are your handler. Nothing calls a function with the
wrong name, so `--gc-sections` **deleted it as dead code**. The first press
jumps to `Default_Handler` and the board hangs.

Two defences, both in this lesson:

- **Look.** `arm-none-eabi-nm Main.elf | grep IRQHandler` — every handler you
  wrote must show `T`.
- **Check at run time.** The banner compares vector 21 with `Default_Handler`
  and prints `YOUR HANDLER IS NOT INSTALLED` when they match. Lab Part 4.

---

## Slide 16: Sharing With `main()` — Count in the ISR, Decide in `main()`

The handler does the least it can: clear, count, return. Deciding what the
edges *mean* is `main()`'s job:

```c
uint32_t edges = b_edges;                 /* one aligned load               */
if (edges != b_seen) {                    /* new edges: restart the clock   */
    b_seen = edges;  b_quiet_ms = 0;  b_armed = 1;
} else if (b_armed && ++b_quiet_ms >= DEBOUNCE_MS) {
    b_armed = 0;                          /* quiet for 20 ms: bounce is over */
    uint8_t down = button_down(BTN_B_PIN);
    ...                                   /* settled pressed, or released?  */
}
```

`b_edges` is lesson 03 slide 7's **one safe case**: a single aligned word,
written by exactly one context. `main()` sees the old value or the new one,
never half of each, so no critical section is needed. It is `volatile` because
otherwise the compiler may read it once and keep the copy in a register forever
(lesson 03, slide 4).

> Why not debounce in the handler? Because a handler that waits 20 ms blocks
> everything of equal or lower priority for 20 ms. **Handlers record; `main()`
> decides.** That rule survives into the RTOS in lesson 07, unchanged.

---

## Slide 17: What You Will Measure

Run it, press each button a few times, and the serial monitor prints one line
per press and per release:

```
A polled     press     1   level changes seen at 1 kHz: 1
A polled     release   1   level changes seen at 1 kHz: 1
B interrupt  press     1   edges caught by EXTI:        ...
B interrupt  release   1   edges caught by EXTI:        ...
```

**Predict before you look**, from slides 7 and 9:

| | Button A, polled | Button B, interrupt |
|---|---|---|
| bounce on (Wokwi default) | 1, occasionally 2–3 | many — tens per press |
| bounce off (`"bounce": "0"`) | 1 | **exactly 1** |

The two methods see the same button. The interrupt sees what actually
happened; the poll sees a sampled summary of it. Neither is wrong — they answer
different questions, and choosing between them is the engineering.

> Those counts are predictions from Wokwi's documented behaviour, not yet
> watched. If yours differ, yours are the real answer — write them down.

---

## Slide 18: Seven Ways to Get Silence

Every one of these builds with **zero warnings** and produces a board that
does nothing, or nearly nothing:

| # | Forgot | Symptom |
|---|---|---|
| 1 | `RCC->IOPENR` for the port | writes vanish; the pin never becomes anything |
| 2 | `MODER` input mode | pin stays analog, reads 0 — "pressed" forever |
| 3 | `PUPDR` pull-up | released button floats; phantom presses |
| 4 | `EXTICR` port select | works on port A by luck, dead elsewhere |
| 5 | `IMR1` unmask | edges latch in `FPR1`, never reach the NVIC |
| 6 | `NVIC_EnableIRQ` | NVIC holds it pending; handler never runs |
| 7 | handler name exact | vector 21 → `Default_Handler`; board hangs on first press |
| — | clearing the pending bit | the opposite failure: handler runs forever |

This is the debugging checklist for every interrupt in the rest of the course:
walk the path of slide 10 **hop by hop**, reading each register back. The
banner does exactly that, and that is why it exists.

---

## Slide 19: What Carries Forward

- **Poll or interrupt is a choice you now make on evidence.** You have measured
  both on the same button.
- **The interrupt path is the same shape everywhere.** A timer, a UART, an ADC
  — each ends in a pending bit, an NVIC line and a vector with an exact name.
  Lessons 06, 08 and 09 reuse this slide 10 with different boxes.
- **The polled millisecond is lying a little.** `tick_wait()` polls SysTick's
  `COUNTFLAG`, which latches *one* wrap. While a `printf` blocks for 5 ms, four
  ticks are lost and `ms_now` runs slow. **Lesson 06 turns SysTick into an
  interrupt**, and time stops depending on the loop coming back.
