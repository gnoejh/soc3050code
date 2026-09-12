# The Programmer's Model: What You Can See and Change
## SOC3050 ARM Edition — Part 0, Models

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

This is the first of four lessons with **no code at all**.

Before you write a line of C for this chip, you need a model of what the chip
*is*. Not a list of peripherals to memorise — a model, so that the twenty-five
peripherals you meet later are recognisably the same thing wearing different
names.

Three lessons follow this one: the **instruction set**, the **development
environment**, and **execution and concurrency**. Then you start building.

---

## Slide 1: What a Programmer's Model Is

A microcontroller contains millions of transistors, and you can see almost none
of them. What you *can* see is a deliberately small, published abstraction — a
contract that says: **here is the state you may inspect and change, and here is
what happens when you do.**

```svg
<svg viewBox="0 0 560 240" role="img" aria-label="The programmer's model as a contract over a hidden implementation">
  <rect class="hifill" x="60" y="24" width="440" height="76" rx="8"/>
  <text x="280" y="50" text-anchor="middle">THE PROGRAMMER'S MODEL</text>
  <text x="280" y="74" text-anchor="middle" class="lbl">registers · memory · peripheral registers · modes · exceptions</text>
  <text x="280" y="90" text-anchor="middle" class="lbl">everything software can name</text>
  <rect class="reg" x="60" y="124" width="440" height="24" rx="4"/>
  <text x="280" y="137" text-anchor="middle" class="mono lbl">the contract — documented, stable, the same on every chip of this family</text>
  <rect class="box" x="60" y="168" width="440" height="52" rx="8"/>
  <text x="280" y="190" text-anchor="middle" class="lbl">THE IMPLEMENTATION</text>
  <text x="280" y="208" text-anchor="middle" class="lbl">pipeline · bus matrix · silicon · none of it named by your code</text>
</svg>
```

That contract is what this lesson teaches. It has exactly five parts:

1. **Registers** — the working state inside the CPU
2. **Memory** — one flat address space, and what lives where
3. **Peripheral registers** — the memory-mapped controls of the hardware
4. **Modes and stacks** — which of two contexts you are running in
5. **Exceptions** — how the hardware takes control away from you

Everything in the next thirty lessons is one of those five.

---

## Slide 2: One Model, Many Instances

A course that teaches GPIO, then timers, then ADC, then UART, then SPI, then
I2C has taught six things. A course that teaches **one peripheral model** has
taught one thing six times — and the seventh peripheral, the one not on the
syllabus, the one in the datasheet of whatever you build after you graduate,
comes free.

> Whenever you meet a new peripheral in this course, the first question is not
> *what does it do*. It is **which part of the model is this?**

---

## Slide 3: Visible State, Part One — the Registers

The CPU cannot compute on memory. It loads into **registers**, operates there,
and stores back. Sixteen of them, and four are special.

```svg
<svg viewBox="0 0 560 190" role="img" aria-label="The ARM register file, R0 to R15 plus xPSR">
  <text x="10" y="22" class="lbl">general purpose — arguments, locals, scratch</text>
  <rect class="reg" x="10" y="32" width="58" height="30" rx="4"/><text x="39" y="47" text-anchor="middle" class="mono">R0</text>
  <rect class="reg" x="74" y="32" width="58" height="30" rx="4"/><text x="103" y="47" text-anchor="middle" class="mono">R1</text>
  <rect class="reg" x="138" y="32" width="58" height="30" rx="4"/><text x="167" y="47" text-anchor="middle" class="mono">R2</text>
  <rect class="reg" x="202" y="32" width="58" height="30" rx="4"/><text x="231" y="47" text-anchor="middle" class="mono">R3</text>
  <rect class="reg" x="266" y="32" width="122" height="30" rx="4"/><text x="327" y="47" text-anchor="middle" class="mono lbl">R4 ... R12</text>
  <rect class="hifill" x="10" y="86" width="106" height="34" rx="5"/>
  <text x="63" y="103" text-anchor="middle" class="mono">R13 / SP</text>
  <rect class="hifill" x="124" y="86" width="106" height="34" rx="5"/>
  <text x="177" y="103" text-anchor="middle" class="mono">R14 / LR</text>
  <rect class="hifill" x="238" y="86" width="106" height="34" rx="5"/>
  <text x="291" y="103" text-anchor="middle" class="mono">R15 / PC</text>
  <text x="63" y="138" text-anchor="middle" class="lbl">stack pointer</text>
  <text x="177" y="138" text-anchor="middle" class="lbl">return address</text>
  <text x="291" y="138" text-anchor="middle" class="lbl">what runs next</text>
  <rect class="reg" x="362" y="86" width="118" height="34" rx="5"/>
  <text x="421" y="103" text-anchor="middle" class="mono">xPSR</text>
  <text x="421" y="138" text-anchor="middle" class="lbl">N Z C V flags, mode</text>
  <text x="280" y="176" text-anchor="middle" class="lbl">Write to PC and you have branched. It is an ordinary register.</text>
</svg>
```

**This is the entire working state of the processor.** Everything else your
program owns is in memory. That is a remarkably small amount of state, and it
is why an interrupt can save and restore it in a handful of cycles.

You will rarely name these in C — the compiler does — but the debugger shows
them, a crash report is mostly their contents, and lesson 01 lives here.

---

## Slide 4: Visible State, Part Two — One Flat Address Space

Everything the CPU can reach, it reaches by **address**. There is no other
channel: no separate I/O instructions, no special port space.

```svg
<svg viewBox="0 0 560 300" role="img" aria-label="Cortex-M address regions from code to private peripheral bus">
  <rect class="box" x="150" y="16" width="230" height="38" rx="5"/>
  <text x="265" y="30" text-anchor="middle" class="mono">PRIVATE PERIPHERAL BUS</text>
  <text x="265" y="46" text-anchor="middle" class="lbl">NVIC, SysTick — ARM's own</text>
  <text x="140" y="36" text-anchor="end" class="mono lbl">0xE0000000</text>
  <rect class="hifill" x="150" y="76" width="230" height="38" rx="5"/>
  <text x="265" y="90" text-anchor="middle" class="mono">PERIPHERAL</text>
  <text x="265" y="106" text-anchor="middle" class="lbl">GPIO, USART, timers — ST's</text>
  <text x="140" y="96" text-anchor="end" class="mono lbl">0x40000000</text>
  <rect class="box" x="150" y="136" width="230" height="38" rx="5"/>
  <text x="265" y="150" text-anchor="middle" class="mono">SRAM</text>
  <text x="265" y="166" text-anchor="middle" class="lbl">variables, stack, heap</text>
  <text x="140" y="156" text-anchor="end" class="mono lbl">0x20000000</text>
  <rect class="box" x="150" y="196" width="230" height="38" rx="5"/>
  <text x="265" y="210" text-anchor="middle" class="mono">CODE</text>
  <text x="265" y="226" text-anchor="middle" class="lbl">flash — and aliased at 0</text>
  <text x="140" y="216" text-anchor="end" class="mono lbl">0x08000000</text>
  <text x="140" y="246" text-anchor="end" class="mono lbl">0x00000000</text>
  <path class="dash" d="M265 54 V76"/>
  <path class="dash" d="M265 114 V136"/>
  <path class="dash" d="M265 174 V196"/>
  <text x="398" y="36" class="lbl">fixed by ARM</text>
  <text x="398" y="96" class="lbl">filled in by ST</text>
  <text x="398" y="156" class="lbl">size chosen by ST</text>
  <text x="398" y="216" class="lbl">size chosen by ST</text>
  <text x="280" y="282" text-anchor="middle" class="lbl">The regions are the same on every Cortex-M. Only the contents change.</text>
</svg>
```

Two details to carry forward:

- **Flash appears twice.** It is really at `0x08000000`, but at reset the chip
  *aliases* it to `0x00000000`, which is how the CPU finds the vector table
  "at address zero".
- **`0xE0000000` is not ST's.** The NVIC and SysTick live there and are
  identical on a chip from any vendor. That tells you which manual to open.

---

## Slide 5: Visible State, Part Three — a Register Is Not a Variable

The peripheral region is where the model stops resembling ordinary programming.
This is the single idea that separates embedded C from the C you have written
before, and almost every beginner bug traces back to getting it wrong.

```svg
<svg viewBox="0 0 560 170" role="img" aria-label="A store instruction to a peripheral address changes a pin">
  <defs><marker id="ar2" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="14" y="46" width="96" height="52" rx="6"/>
  <text x="62" y="72" text-anchor="middle">CPU</text>
  <path class="wire" d="M110 72 H186" marker-end="url(#ar2)"/>
  <text x="148" y="56" text-anchor="middle" class="lbl mono">store</text>
  <rect class="reg" x="192" y="46" width="140" height="52" rx="6"/>
  <text x="262" y="66" text-anchor="middle" class="mono">0x50000014</text>
  <text x="262" y="85" text-anchor="middle" class="lbl">just an address</text>
  <path class="hi" d="M332 72 H416" marker-end="url(#ar2)"/>
  <rect class="hifill" x="422" y="46" width="120" height="52" rx="6"/>
  <text x="482" y="66" text-anchor="middle">a pin goes high</text>
  <text x="482" y="85" text-anchor="middle" class="lbl">a motor turns</text>
  <text x="280" y="140" text-anchor="middle" class="lbl">The store instruction has a side effect in the physical world.</text>
  <text x="280" y="158" text-anchor="middle" class="lbl">Nothing in the C language says so.</text>
</svg>
```

| A variable | A register |
|---|---|
| Lives in RAM | **Is** a piece of hardware |
| Reading it twice gives the same value | Reading it twice can give different values |
| Writing it changes memory | Writing it **makes something happen** |
| Reading is free of consequence | Reading can **clear a flag** or consume a byte |
| The compiler may cache, reorder or delete accesses | Every access must happen, exactly as written |

That last row is why `volatile` exists. Lesson 03 returns to it.

---

## Slide 6: Every Peripheral Has the Same Three Registers

Strip away the names and every peripheral on this chip — and on most chips you
will ever meet — is the same shape.

```svg
<svg viewBox="0 0 560 250" role="img" aria-label="Control, status and data registers around a state machine">
  <defs><marker id="ar3" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="16" y="26" width="130" height="42" rx="6"/>
  <text x="81" y="41" text-anchor="middle" class="mono">CONTROL</text>
  <text x="81" y="58" text-anchor="middle" class="lbl">you write: settings</text>
  <rect class="box" x="16" y="96" width="130" height="42" rx="6"/>
  <text x="81" y="111" text-anchor="middle" class="mono">STATUS</text>
  <text x="81" y="128" text-anchor="middle" class="lbl">you read: what happened</text>
  <rect class="box" x="16" y="166" width="130" height="42" rx="6"/>
  <text x="81" y="181" text-anchor="middle" class="mono">DATA</text>
  <text x="81" y="198" text-anchor="middle" class="lbl">the payload, both ways</text>
  <path class="wire" d="M146 47 H236" marker-end="url(#ar3)"/>
  <path class="wire" d="M236 117 H146" marker-end="url(#ar3)"/>
  <path class="wire" d="M146 187 H236" marker-end="url(#ar3)"/>
  <rect class="hifill" x="240" y="26" width="150" height="182" rx="8"/>
  <text x="315" y="105" text-anchor="middle">the peripheral</text>
  <text x="315" y="126" text-anchor="middle" class="lbl">a state machine</text>
  <path class="hi" d="M390 117 H470" marker-end="url(#ar3)"/>
  <rect class="reg" x="474" y="92" width="70" height="50" rx="6"/>
  <text x="509" y="117" text-anchor="middle" class="lbl">pins</text>
  <text x="280" y="236" text-anchor="middle" class="lbl">Configure with CONTROL. Watch with STATUS. Move bytes through DATA.</text>
</svg>
```

Fill that template in and you have learned a peripheral:

| Peripheral | Control | Status | Data |
|---|---|---|---|
| USART | `CR1` — baud, enable, interrupts | `ISR` — is the buffer empty? | `TDR` / `RDR` |
| SPI | `CR1` — clock speed, polarity | `SR` — transfer complete? | `DR` |
| ADC | `CFGR` — which channel, resolution | `ISR` — conversion done? | `DR` |
| Timer | `CR1`, `PSC`, `ARR` | `SR` — did it overflow? | `CNT`, `CCR` |
| GPIO | `MODER`, `PUPDR` | `IDR` — what are the pins? | `ODR`, `BSRR` |

The names move between chips. **The three roles never do.**

---

## Slide 7: Two Kinds of Peripheral, Two Manuals

Your chip contains peripherals from **two different companies**, and knowing
which is which tells you where to look things up.

```svg
<svg viewBox="0 0 560 220" role="img" aria-label="ARM core peripherals versus ST vendor peripherals and their headers">
  <rect class="hifill" x="24" y="24" width="230" height="120" rx="7"/>
  <text x="139" y="46" text-anchor="middle">ARM supplies</text>
  <text x="139" y="70" text-anchor="middle" class="mono">NVIC · SysTick · SCB · MPU</text>
  <text x="139" y="94" text-anchor="middle" class="lbl">same on every Cortex-M,</text>
  <text x="139" y="110" text-anchor="middle" class="lbl">any vendor</text>
  <text x="139" y="132" text-anchor="middle" class="mono lbl">core_cm0plus.h</text>
  <rect class="box" x="306" y="24" width="230" height="120" rx="7"/>
  <text x="421" y="46" text-anchor="middle">ST supplies</text>
  <text x="421" y="70" text-anchor="middle" class="mono">GPIO · USART · TIM · ADC · RCC</text>
  <text x="421" y="94" text-anchor="middle" class="lbl">different on every vendor,</text>
  <text x="421" y="110" text-anchor="middle" class="lbl">and between ST families</text>
  <text x="421" y="132" text-anchor="middle" class="mono lbl">stm32c031xx.h</text>
  <text x="280" y="180" text-anchor="middle" class="lbl">Knowledge of the left box transfers to every ARM microcontroller you ever use.</text>
  <text x="280" y="200" text-anchor="middle" class="lbl">Knowledge of the right box transfers to the next STM32 — and no further.</text>
</svg>
```

A question about interrupt priorities is answered in ARM's *Cortex-M Generic
User Guide*. A question about which pin USART2 comes out on is answered in ST's
reference manual. Opening the wrong document is the commonest way to lose an
hour.

---

## Slide 8: Visible State, Part Four — Modes and Stacks

The processor is always in one of two modes, and it switches without being
asked.

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="Thread mode and handler mode with main and process stack pointers">
  <defs><marker id="ar9" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="40" y="30" width="180" height="70" rx="7"/>
  <text x="130" y="54" text-anchor="middle">THREAD mode</text>
  <text x="130" y="74" text-anchor="middle" class="lbl">your main() runs here</text>
  <text x="130" y="91" text-anchor="middle" class="lbl mono">MSP, or PSP</text>
  <rect class="hifill" x="340" y="30" width="180" height="70" rx="7"/>
  <text x="430" y="54" text-anchor="middle">HANDLER mode</text>
  <text x="430" y="74" text-anchor="middle" class="lbl">every interrupt runs here</text>
  <text x="430" y="91" text-anchor="middle" class="lbl mono">always MSP</text>
  <path class="hi" d="M220 54 H336" marker-end="url(#ar9)"/>
  <text x="278" y="44" text-anchor="middle" class="lbl">interrupt</text>
  <path class="wire" d="M336 84 H224" marker-end="url(#ar9)"/>
  <text x="278" y="106" text-anchor="middle" class="lbl">return</text>
  <text x="280" y="154" text-anchor="middle" class="lbl">Handler mode is always privileged. Thread mode need not be —</text>
  <text x="280" y="172" text-anchor="middle" class="lbl">which is how an RTOS keeps one task from corrupting another.</text>
</svg>
```

Until the RTOS lesson, one stack (`MSP`) serves everything and you can ignore
the distinction. It matters later for two reasons: an RTOS gives each task its
own stack on `PSP`, and a fault handler runs in Handler mode, which is how it
still works when the faulting code has wrecked everything else.

---

## Slide 9: Visible State, Part Five — the Exception Model

An interrupt is a **hardware-forced function call**. The NVIC does not search
for your handler; it indexes a table of addresses at the start of flash.

```svg
<svg viewBox="0 0 560 230" role="img" aria-label="Vector table indexed by exception number to find the handler address">
  <defs><marker id="ara" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="hifill" x="20" y="80" width="110" height="46" rx="6"/>
  <text x="75" y="98" text-anchor="middle" class="lbl">USART2 fires</text>
  <text x="75" y="116" text-anchor="middle" class="mono">IRQ 28</text>
  <path class="wire" d="M130 103 H176" marker-end="url(#ara)"/>
  <text x="290" y="26" text-anchor="middle" class="lbl">the vector table, at 0x08000000</text>
  <rect class="reg" x="180" y="36" width="220" height="26" rx="4"/>
  <text x="290" y="49" text-anchor="middle" class="mono lbl">[0]  initial stack pointer</text>
  <rect class="reg" x="180" y="66" width="220" height="26" rx="4"/>
  <text x="290" y="79" text-anchor="middle" class="mono lbl">[1]  Reset_Handler</text>
  <rect class="reg" x="180" y="96" width="220" height="26" rx="4"/>
  <text x="290" y="109" text-anchor="middle" class="mono lbl">...</text>
  <rect class="hifill" x="180" y="126" width="220" height="26" rx="4"/>
  <text x="290" y="139" text-anchor="middle" class="mono">[16+28] USART2_IRQHandler</text>
  <path class="hi" d="M400 139 H452" marker-end="url(#ara)"/>
  <rect class="box" x="456" y="116" width="90" height="46" rx="6"/>
  <text x="501" y="134" text-anchor="middle" class="lbl">your</text>
  <text x="501" y="151" text-anchor="middle" class="lbl">function</text>
  <text x="280" y="196" text-anchor="middle" class="lbl">The first 16 slots are ARM's — reset and faults. Vendor interrupts start at 16.</text>
  <text x="280" y="216" text-anchor="middle" class="lbl">Put the wrong address in a slot and the chip jumps there anyway.</text>
</svg>
```

**The table is built by your own code** — you will write one in the first
coding lesson. There is no registration call; a handler is connected because
its address sits in the right slot.

---

## Slide 10: The Data Model — Width, Alignment, Byte Order

Three rules that produce puzzling bugs when violated.

**Everything is 32 bits wide.** The registers, the ALU, the bus. A `uint8_t` is
not cheaper than a `uint32_t` here — it is often more expensive, because the
compiler must mask and shift. *Do not* reach for `uint8_t` out of AVR habit.

**Accesses must be aligned.** A 32-bit load wants an address divisible by four.

| | Unaligned access |
|---|---|
| Cortex-M0+ | **faults immediately** — HardFault |
| Cortex-M3 / M4 | works for ordinary loads and stores, slower |

This bites when you cast a `uint8_t*` buffer to a `uint32_t*` — a favourite
trick when parsing packets, and a reliable crash on an M0+.

**Little-endian.** `0x12345678` stored at address 0 puts `0x78` at byte 0.

---

## Slide 11: Your Responsibilities — Two Gates You Must Open

The model says a peripheral has registers. It does not promise they answer.
Two gates are shut at power-on, and **opening them is your job**.

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="Clock gate and pin multiplexer guarding a peripheral">
  <defs><marker id="ar4" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="14" y="66" width="92" height="46" rx="6"/>
  <text x="60" y="89" text-anchor="middle">clock</text>
  <path class="wire" d="M106 89 H156" marker-end="url(#ar4)"/>
  <rect class="hifill" x="160" y="60" width="96" height="58" rx="6"/>
  <text x="208" y="82" text-anchor="middle" class="lbl">GATE</text>
  <text x="208" y="102" text-anchor="middle" class="mono">RCC</text>
  <path class="wire" d="M256 89 H306" marker-end="url(#ar4)"/>
  <rect class="box" x="310" y="60" width="96" height="58" rx="6"/>
  <text x="358" y="89" text-anchor="middle">peripheral</text>
  <path class="wire" d="M406 89 H452" marker-end="url(#ar4)"/>
  <rect class="hifill" x="456" y="60" width="90" height="58" rx="6"/>
  <text x="501" y="82" text-anchor="middle" class="lbl">MUX</text>
  <text x="501" y="102" text-anchor="middle" class="mono">pin</text>
  <text x="208" y="150" text-anchor="middle" class="lbl">off by default</text>
  <text x="501" y="150" text-anchor="middle" class="lbl">one pin, many owners</text>
  <text x="280" y="184" text-anchor="middle" class="lbl">A peripheral with no clock reads back as zero and ignores every write.</text>
</svg>
```

**The clock gate.** Every peripheral powers up *switched off*, to save energy.
Write to its control register first and the write goes nowhere — no error, the
register simply reads back zero. The single most common first-week failure.

**The pin multiplexer.** More peripherals than pins, so each pin is shared and
you must say which owner it belongs to. Choose wrong and the pin is
electrically healthy and carries no signal.

> Both look identical from outside: correct code that does nothing. Neither
> produces an error message. Suspect them first.

---

## Slide 12: Your Responsibilities — the Clock Is Yours Too

Every timed thing on the chip derives from one oscillator by division. Nothing
picks the divisions for you.

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="Oscillator through divider to CPU, fast bus and slow bus">
  <defs><marker id="ar5" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="10" y="76" width="96" height="44" rx="6"/>
  <text x="58" y="92" text-anchor="middle">oscillator</text>
  <text x="58" y="110" text-anchor="middle" class="lbl mono">48 MHz</text>
  <path class="wire" d="M106 98 H150" marker-end="url(#ar5)"/>
  <rect class="hifill" x="154" y="76" width="90" height="44" rx="6"/>
  <text x="199" y="98" text-anchor="middle" class="mono">divider</text>
  <path class="wire" d="M244 98 H278"/>
  <path class="wire" d="M278 42 V154"/>
  <path class="wire" d="M278 42 H300" marker-end="url(#ar5)"/>
  <path class="wire" d="M278 98 H300" marker-end="url(#ar5)"/>
  <path class="wire" d="M278 154 H300" marker-end="url(#ar5)"/>
  <rect class="reg" x="304" y="24" width="128" height="36" rx="5"/>
  <text x="368" y="42" text-anchor="middle" class="lbl">CPU core</text>
  <rect class="reg" x="304" y="80" width="128" height="36" rx="5"/>
  <text x="368" y="98" text-anchor="middle" class="lbl">fast bus — GPIO, DMA</text>
  <rect class="reg" x="304" y="136" width="128" height="36" rx="5"/>
  <text x="368" y="154" text-anchor="middle" class="lbl">slow bus — USART, timers</text>
  <text x="494" y="90" text-anchor="middle" class="lbl">each with its</text>
  <text x="494" y="108" text-anchor="middle" class="lbl">own prescaler</text>
</svg>
```

- **A baud rate is a division.** The USART does not know what 115200 means; you
  give it a divisor computed from *its own bus clock*, not the CPU clock. Get
  that wrong and the output is garbage while the code is perfect.
- **A delay is a count.** There is no `sleep()`. One millisecond means counting
  a known number of ticks.

---

## Slide 13: Where the Implementation Leaks Through

The model is an abstraction, and abstractions leak. Three places where the
hardware underneath becomes visible in code you write — these are the *only*
implementation details this course asks you to care about.

```svg
<svg viewBox="0 0 560 240" role="img" aria-label="Three places the implementation becomes programmer-visible">
  <rect class="box" x="16" y="24" width="168" height="104" rx="7"/>
  <text x="100" y="46" text-anchor="middle" class="lbl">PIPELINE</text>
  <text x="100" y="70" text-anchor="middle" class="lbl">instructions overlap</text>
  <text x="100" y="94" text-anchor="middle" class="mono">PC reads ahead</text>
  <text x="100" y="114" text-anchor="middle" class="lbl">branches cost extra</text>
  <rect class="box" x="196" y="24" width="168" height="104" rx="7"/>
  <text x="280" y="46" text-anchor="middle" class="lbl">TWO BUSES</text>
  <text x="280" y="70" text-anchor="middle" class="lbl">fast AHB, slow APB</text>
  <text x="280" y="94" text-anchor="middle" class="mono">RCC-&gt;AHBENR</text>
  <text x="280" y="112" text-anchor="middle" class="mono">RCC-&gt;APBENR1</text>
  <rect class="hifill" x="376" y="24" width="168" height="104" rx="7"/>
  <text x="460" y="46" text-anchor="middle" class="lbl">FLASH IS SLOW</text>
  <text x="460" y="70" text-anchor="middle" class="lbl">the core outruns it</text>
  <text x="460" y="94" text-anchor="middle" class="mono">FLASH-&gt;ACR</text>
  <text x="460" y="112" text-anchor="middle" class="lbl">wait states</text>
  <text x="280" y="170" text-anchor="middle" class="lbl">You cannot name the pipeline or the bus matrix in C. But you must name</text>
  <text x="280" y="188" text-anchor="middle" class="lbl">the register that enables a clock on the right bus, and the one that</text>
  <text x="280" y="206" text-anchor="middle" class="lbl">sets the flash wait state — before you raise the clock, never after.</text>
</svg>
```

That last clause is the first of many **ordering requirements the compiler
knows nothing about**. The code compiles identically either way round; only one
order works.

---

## Slide 14: The Implementation, Once

For completeness — here is what is actually under the model. You will not
program any of this directly, but it explains the leaks on the previous slide.

```svg
<svg viewBox="0 0 620 300" role="img" aria-label="Block diagram of core, bus matrix, memory and peripherals">
  <defs><marker id="ov" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="hifill" x="12" y="20" width="150" height="112" rx="8"/>
  <text x="87" y="40" text-anchor="middle">Cortex-M core</text>
  <rect class="reg" x="24" y="52" width="126" height="22" rx="4"/>
  <text x="87" y="63" text-anchor="middle" class="mono lbl">R0-R15</text>
  <rect class="reg" x="24" y="78" width="126" height="22" rx="4"/>
  <text x="87" y="89" text-anchor="middle" class="mono lbl">NVIC</text>
  <rect class="reg" x="24" y="104" width="126" height="22" rx="4"/>
  <text x="87" y="115" text-anchor="middle" class="mono lbl">SysTick</text>
  <rect class="box" x="12" y="150" width="150" height="36" rx="6"/>
  <text x="87" y="172" text-anchor="middle" class="lbl">oscillators</text>
  <rect class="hifill" x="186" y="20" width="52" height="166" rx="6"/>
  <text x="212" y="94" text-anchor="middle" class="lbl">BUS</text>
  <text x="212" y="112" text-anchor="middle" class="lbl">MATRIX</text>
  <path class="wire" d="M162 76 H182" marker-end="url(#ov)"/>
  <rect class="box" x="266" y="20" width="150" height="38" rx="6"/>
  <text x="341" y="34" text-anchor="middle" class="mono">FLASH</text>
  <text x="341" y="50" text-anchor="middle" class="lbl">+ wait states</text>
  <rect class="box" x="266" y="66" width="150" height="34" rx="6"/>
  <text x="341" y="87" text-anchor="middle" class="mono">SRAM</text>
  <path class="wire" d="M238 40 H262" marker-end="url(#ov)"/>
  <path class="wire" d="M238 83 H262" marker-end="url(#ov)"/>
  <rect class="box" x="266" y="108" width="150" height="42" rx="6"/>
  <text x="341" y="124" text-anchor="middle" class="mono">AHB — fast</text>
  <text x="341" y="141" text-anchor="middle" class="lbl">GPIO, DMA, RCC</text>
  <path class="wire" d="M238 129 H262" marker-end="url(#ov)"/>
  <rect class="reg" x="266" y="160" width="150" height="26" rx="5"/>
  <text x="341" y="176" text-anchor="middle" class="lbl">APB bridge</text>
  <rect class="box" x="266" y="196" width="150" height="42" rx="6"/>
  <text x="341" y="212" text-anchor="middle" class="mono">APB — slow</text>
  <text x="341" y="229" text-anchor="middle" class="lbl">USART, SPI, I2C, TIM, ADC</text>
  <path class="wire" d="M212 186 V210 H262" marker-end="url(#ov)"/>
  <rect class="hifill" x="446" y="108" width="66" height="130" rx="6"/>
  <text x="479" y="166" text-anchor="middle" class="lbl">PIN</text>
  <text x="479" y="184" text-anchor="middle" class="lbl">MUX</text>
  <path class="wire" d="M416 129 H442" marker-end="url(#ov)"/>
  <path class="wire" d="M416 217 H442" marker-end="url(#ov)"/>
  <rect class="reg" x="540" y="108" width="66" height="130" rx="6"/>
  <text x="573" y="173" text-anchor="middle" class="lbl">pins</text>
  <path class="hi" d="M512 173 H536" marker-end="url(#ov)"/>
  <text x="310" y="272" text-anchor="middle" class="lbl">Left of the matrix is ARM's, identical on every Cortex-M. Right of it is ST's.</text>
</svg>
```

---

## Slide 15: What Changed, Coming From 8 Bits

Many of you have met an 8-bit AVR. Two differences change how you write code.

| | AVR (8-bit) | Cortex-M (32-bit) |
|---|---|---|
| Registers | 32 × 8-bit | 16 × 32-bit |
| Reaching a peripheral | a **separate I/O space**, own instructions (`IN`, `OUT`) | ordinary addresses, ordinary `LDR`/`STR` |
| Code and data | Harvard — separate spaces, `LPM` to read flash | **one space**; a pointer reaches anything |
| Interrupt vectors | fixed table of jump instructions | table of **addresses**, built by your code |

On AVR, `PORTB` was a special kind of thing needing special instructions. Here
there is no such category — and that uniformity is both the gift and the trap.
Nothing stops a stray pointer landing on a peripheral.

---

## Slide 16: The Programmer's Model, Complete

```svg
<svg viewBox="0 0 560 250" role="img" aria-label="Summary: five parts of the programmer's model">
  <rect class="hifill" x="20" y="20" width="250" height="80" rx="7"/>
  <text x="145" y="42" text-anchor="middle">STATE YOU OWN</text>
  <text x="145" y="64" text-anchor="middle" class="lbl">16 registers + xPSR</text>
  <text x="145" y="84" text-anchor="middle" class="lbl">one flat address space</text>
  <rect class="box" x="290" y="20" width="250" height="80" rx="7"/>
  <text x="415" y="42" text-anchor="middle">STATE THE CHIP OWNS</text>
  <text x="415" y="64" text-anchor="middle" class="lbl">peripheral registers:</text>
  <text x="415" y="84" text-anchor="middle" class="lbl">control · status · data</text>
  <rect class="box" x="20" y="116" width="250" height="72" rx="7"/>
  <text x="145" y="138" text-anchor="middle">WHO IS RUNNING</text>
  <text x="145" y="160" text-anchor="middle" class="lbl">thread or handler mode</text>
  <text x="145" y="178" text-anchor="middle" class="lbl">MSP or PSP</text>
  <rect class="box" x="290" y="116" width="250" height="72" rx="7"/>
  <text x="415" y="138" text-anchor="middle">HOW CONTROL IS TAKEN</text>
  <text x="415" y="160" text-anchor="middle" class="lbl">vector table, NVIC</text>
  <text x="415" y="178" text-anchor="middle" class="lbl">an address in a slot</text>
  <text x="280" y="222" text-anchor="middle" class="lbl">Five parts. Everything in the next thirty lessons is an instance of one of them.</text>
</svg>
```

---

## Slide 17: What You Should Be Able to Say

1. What does a *programmer's model* include, and what does it deliberately
   leave out?
2. Name the five parts of this one.
3. The CPU has one mechanism for reaching anything outside itself. What is it?
4. Give three ways a register behaves differently from a variable.
5. A peripheral's registers fall into three roles. Name them, and say who
   writes each.
6. Which peripherals are ARM's and which are ST's? Why does it decide which
   manual you open?
7. Your code configures a peripheral and it reads back as zero. First suspect?
8. Your serial output is garbage but the code is right. Name two causes from
   this lesson.
9. You cast a `uint8_t*` to a `uint32_t*` and the chip HardFaults. Why — and
   would a Cortex-M3 fault too?
10. Name the three places the implementation leaks into code you must write.
11. How does the hardware find the function you wrote for USART2?

**Next:** *Instruction Set* — what this state looks like to the processor, and
how your C becomes something it can execute.
