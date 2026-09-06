# Embedded Processors and the ATmega128
## ATmega128 Embedded Systems Course — First Class

**Reference**: [ATmega128 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf)

Three questions, in order:

1. **What is an embedded processor**, and how is it unlike the computer you write essays on?
2. **What does the development environment do** to turn your `.c` file into something a chip executes?
3. **How is the ATmega128 built**, and where does that shape everything you write for the rest of the course?

---

## Slide 1: You Already Own Fifty of These

Count the processors within three metres of you. Not the phone and the laptop —
the other ones.

The washing machine. The microwave. The car key. The USB charger that
negotiates voltage. The wireless earbuds, three of them: one per ear and one in
the case. The keyboard you are typing on has a microcontroller scanning its
key matrix right now. So does the mouse, the monitor, the router, the smoke
alarm, the electric toothbrush, and every one of the fifty-odd modules in a
modern car.

### The scale of it
| | Roughly per year | Typical price |
|---|---|---|
| Desktop and laptop CPUs | 260 million | $100 – $500 |
| Smartphone application processors | 1.2 billion | $20 – $150 |
| **Microcontrollers** | **25 – 30 billion** | **$0.20 – $5** |

Well over 90 % of the processors manufactured each year are never seen by the
person using them. They have no screen, no keyboard, no operating system to
speak of, and they run one program from the moment they are powered until they
are thrown away.

**That is the field you are entering.** It is not a smaller version of desktop
programming. It is a different discipline with different constraints, and the
constraints are the interesting part.

---

## Slide 2: What Makes a System "Embedded"

An **embedded system** is a computer built into a device whose purpose is not
computing.

A laptop's purpose is to run whatever program you install. A dishwasher's
purpose is to wash dishes; the processor inside it is a component, like the
pump and the heating element, and it exists to do exactly one job for the life
of the appliance.

### Four properties that follow from that
- **Dedicated function.** The program is fixed at manufacture. Nobody installs
  an app on a microwave.
- **Real-time behaviour.** A late answer is a wrong answer. An airbag
  controller that decides correctly 20 ms after the crash has failed.
- **Resource constraints.** Kilobytes, not gigabytes. Milliwatts, not watts.
  Cents, not dollars.
- **Direct contact with the physical world.** Sensors in, actuators out. The
  program's output is a motor turning, not a window opening.

### And one that surprises people
**Reliability without supervision.** There is no user to reboot it, no
administrator to patch it, no log file anyone will read. A heating controller
must run for ten years and never wedge. That is why lesson 21 covers the
watchdog timer, and why "it works on my desk" is not a standard this field
accepts.

---

## Slide 3: Microprocessor vs Microcontroller

Both contain a CPU. The difference is what else is on the die.

```
   MICROPROCESSOR (MPU)                MICROCONTROLLER (MCU)
   e.g. an x86 or Cortex-A             e.g. the ATmega128

   +---------------+                   +-----------------------------+
   |   CPU core    |                   |  CPU core                   |
   |   cache       |                   |  Flash 128 KB   SRAM 4 KB   |
   +-------+-------+                   |  EEPROM 4 KB                |
           |  external buses           |  Timers  ADC  UART  SPI TWI |
   +-------+-------+-------+           |  53 I/O pins                |
   | RAM  | Flash | I/O    |           +-----------------------------+
   +------+-------+--------+                    one chip
     several chips, a circuit
     board, a power sequence
```

| | Microprocessor | Microcontroller |
|---|---|---|
| Memory | External chips | **On the die** |
| Peripherals | External controllers | **On the die** |
| Boots into | An operating system | **Your program** |
| Typical clock | 1 – 5 GHz | 1 – 200 MHz |
| Typical RAM | 4 – 64 GB | 1 – 512 KB |
| Power | 15 – 150 W | 1 mW – 500 mW |
| Cost | $100 upwards | $0.20 upwards |
| Boot time | Tens of seconds | **Microseconds** |

**The ATmega128 is a microcontroller.** Everything it needs is inside it: give
it 5 V and a clock, and it runs. That single-chip completeness is why an
appliance maker will choose an MCU over a faster part that needs a board around
it.

---

## Slide 4: The Constraints, in Numbers

Put the machine you know beside the one you are about to program.

| | A 2026 laptop | ATmega128 | Ratio |
|---|---|---|---|
| Clock | 4 GHz, 8 cores | 16 MHz, 1 core | 250x per core |
| RAM | 16 GB | **4 KB** | 4,000,000x |
| Program store | 1 TB SSD | **128 KB Flash** | 8,000,000x |
| Word size | 64-bit | **8-bit** | 8x |
| Floating point | In hardware | **In software, slowly** | — |
| Power | ~45 W | ~0.02 W | 2000x |
| Price | $1200 | ~$5 | 240x |

### What those numbers actually mean when you write code
- **4 KB of RAM** is one 64x64 array of bytes. A single careless
  `char buffer[1024]` is a quarter of everything you have.
- **8-bit registers** mean a 16-bit addition takes two instructions and a
  32-bit multiply takes a subroutine. `int` arithmetic is not free here.
- **No floating-point unit.** `float x = a * b;` compiles into a call into a
  software library that takes hundreds of cycles. Lesson 12 shows how to scale
  integers instead.
- **No memory protection.** A pointer bug does not produce a segmentation
  fault; it silently overwrites something else and the fault appears an hour
  later somewhere unrelated.
- **No `printf` to a console.** The banner you will see in a moment exists
  because we configured a serial port and pushed bytes out of it, one at a time.

**None of this makes the work harder than desktop work. It makes it different.**
You will know where every byte went, which is a luxury desktop programmers gave
up long ago.

---

## Slide 5: The Microcontroller Landscape

The ATmega128 is one of hundreds of families. A rough map:

| Family | Bits | Vendor | Where you meet it |
|---|---|---|---|
| **AVR** (ATmega, ATtiny) | 8 | Microchip | Arduino, hobby, appliances, **this course** |
| PIC | 8/16/32 | Microchip | Industrial, automotive, huge installed base |
| MSP430 | 16 | TI | Ultra-low-power metering and sensing |
| **ARM Cortex-M** | 32 | many vendors | The industry default: STM32, nRF, RP2040 |
| ESP32 | 32 | Espressif | Wi-Fi and Bluetooth for a few dollars |
| RISC-V | 32/64 | many | Open ISA, growing quickly |

### Why start on an 8-bit AVR when industry runs Cortex-M?
Because the AVR is small enough to hold in your head, all of it.

- **The datasheet is 386 pages**, and you can genuinely read it. An STM32
  reference manual is over 1700 pages plus a separate core manual.
- **Every register is documented and reachable.** There is no vendor HAL layer
  between your code and the silicon, so nothing is hidden by "the library
  handles that".
- **Instructions take a countable number of cycles.** No cache, no branch
  prediction, no pipeline stall you cannot predict. When you need a 4 µs pulse
  you can count the cycles and get exactly 4 µs.
- **The concepts transfer completely.** A timer is a timer, a UART is a UART, an
  ADC is an ADC. Moving to a Cortex-M after this course means learning new
  register names, not new ideas.

Learn the mechanism on a chip that lets you see it. Then scale up.

---

## Slide 6: Bare Metal — Programming With No Floor Under You

On a laptop, your program sits on a stack of things other people wrote.

```
   DESKTOP                          BARE METAL (this course)

   your program                     your program
   -----------------                -----------------------------
   libraries / runtime              avr-libc (thin: _delay_ms, sprintf)
   operating system                        (nothing)
   drivers                                 (nothing)
   -----------------                -----------------------------
   hardware                         hardware
```

Remove the operating system and several everyday assumptions go with it:

| You are used to | On bare metal |
|---|---|
| `main` returns and the process exits | **Nothing to return to** — `main` must never end |
| The OS schedules your threads | **You** decide what runs, in one loop |
| `malloc` from a large heap | 4 KB total; prefer static allocation |
| The OS drives the hardware | **You** write to the peripheral registers |
| A crash kills one process | A crash wedges the whole device |
| `printf` reaches a terminal | You configure a UART and send bytes |

### The shape of nearly every bare-metal program
```c
int main(void)
{
    hardware_setup();      /* run once: direction registers, clocks, peripherals */

    while (1)              /* run forever: there is nowhere else to go */
    {
        read_inputs();
        decide();
        drive_outputs();
    }
}
```

This is the **super loop**, and it is the right structure for a surprising number
of real products. Lesson 22 shows what to do when it stops being enough.

---

## Slide 7: The Peripheral Register Is the API

There is one idea underneath the entire course, and it is worth stating plainly
before you meet it thirty times.

**Hardware is controlled by writing numbers to addresses.**

A timer is not an object with methods. It is a block of silicon that watches a
handful of bytes at fixed addresses and behaves according to the bits in them.
Setting a bit in `TCCR1B` does not *ask* the timer to start — the bit *is* the
switch, and the counter advances on the next clock edge.

```c
DDRB = 0xFF;              /* address 0x37: all 8 PORT B pins become outputs */
PORTB = 0xFE;             /* address 0x38: drive PB0 low, the rest high     */
```

`DDRB` looks like a variable. It is not. `avr/io.h` defines it as a
dereferenced pointer to a fixed address:

```c
#define DDRB  (*(volatile uint8_t *) 0x37)
```

### Why `volatile` is not optional
It tells the compiler: **this location can change without the program changing
it, and writing it has effects you cannot see.** Without `volatile`, the
optimiser would look at

```c
while (!(UCSR1A & (1 << UDRE1)))   /* wait for the transmitter */
    ;
```

conclude that `UCSR1A` cannot change inside a loop that does not assign to it,
and turn the whole thing into an infinite loop. Every register definition in
`avr/io.h` is `volatile` for this reason.

**So: to learn a peripheral, you learn its registers.** That is why every deck
in this course has a register table, and why the datasheet is the real textbook.

---

## Slide 8: Cross-Compilation

Your PC is an x86-64 machine running Windows. The target is an 8-bit AVR with
no operating system. The compiler that produced your PC's programs cannot
produce this one's.

A **cross-compiler** runs on one architecture and emits code for another. That
is `avr-gcc`: an x86 Windows program that reads C and writes AVR machine code.

```
   HOST (your PC, x86-64 Windows)          TARGET (ATmega128, 8-bit AVR)

   Main.c
     |  avr-gcc  -mmcu=atmega128
     v
   Main.elf  (code + symbols + debug info)
     |  avr-objcopy -O ihex
     v
   Main.hex  (just the bytes, as text) ------>  Flash memory
                                                   |
                                                   v
                                              the program runs
```

Every tool in the chain is prefixed `avr-` for exactly this reason: `avr-gcc`,
`avr-objcopy`, `avr-size`, `avr-objdump`, `avr-nm`. They all live in
`tools/avr-toolchain/bin/` in this repository, so there is nothing to install.

### The consequence people trip over
**The host cannot run what the compiler produced.** You cannot double-click
`Main.hex`. To see it work you either program a real chip or run a **simulator** —
which is what SimulIDE is, and why this course needs no hardware.

---

## Slide 9: From Source to Firmware, Step by Step

```
   Main.c   config.h                    what you write
      |
      |  [1] PREPROCESSOR      #include pasted in, #define substituted,
      |                        -DF_CPU=16000000UL injected from the command line
      v
   translation unit
      |
      |  [2] COMPILER          C  ->  AVR assembly  ->  machine code
      |                        -Os chooses the smallest code
      v
   Main.o  (+ any shared_libs objects)
      |
      |  [3] LINKER            joins the objects, pulls what it needs from
      |                        avr-libc, lays out the 35-entry vector table,
      |                        assigns every address, discards unused sections
      v
   Main.elf     2474 bytes of code + 812 bytes of constants, plus symbols
      |
      |  [4] avr-objcopy -O ihex -R .eeprom
      v
   Main.hex     the same bytes, as printable Intel HEX text
      |
      |  [5] avrdude to a real chip  /  SimulIDE to a simulated one
      v
   running firmware
```

### The one command that does all of it
From `_build/build-lesson.bat`, unchanged:

```
avr-gcc -mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -Os -Wall -Wextra
        -ffunction-sections -fdata-sections -I. -I..\..\shared_libs
        Main.c -Wl,--gc-sections -o Main.elf

avr-objcopy -O ihex -R .eeprom Main.elf Main.hex
```

`avr-gcc` runs steps 1 to 3 in one invocation. That is a convenience, not a
simplification — the four stages are still there, and the error messages tell
you which one failed.

---

## Slide 10: What Each Flag Is For

Not decoration. Each one changes the firmware.

| Flag | Effect |
|---|---|
| `-mmcu=atmega128` | Selects the instruction set, the memory sizes and the **35-entry** vector table. Get this wrong and the code is for a different chip. |
| `-DF_CPU=16000000UL` | Defines the clock frequency for the preprocessor. **`_delay_ms` and every baud-rate calculation derive from it.** |
| `-DBAUD=9600` | The serial bit rate, used to compute `UBRR1`. |
| `-Os` | Optimise for **size**, not speed. Flash is the scarce resource. |
| `-Wall -Wextra` | Turn on the warnings. On a chip with no memory protection, a warning you ignored is a bug you will spend an evening on. |
| `-ffunction-sections -fdata-sections` | Put every function and object in its own section... |
| `-Wl,--gc-sections` | ...so the linker can discard the ones nothing calls. |

### F_CPU is the one that bites
It is a *promise you make to the compiler*, not something it can measure.
`_delay_ms(120)` becomes a counted busy loop whose length is computed from
`F_CPU`. Tell it 8 MHz while the board runs at 16 MHz and every delay is half
as long, every baud rate is doubled, and the serial monitor prints garbage.

This is why the 2026 edition fixes 16 MHz everywhere, and why `config.h` guards
the definition:

```c
#ifndef F_CPU
#define F_CPU 16000000UL     /* only used if the command line did not set it */
#endif
```

An **unguarded** `#define F_CPU` in a header silently overrides `-DF_CPU` and
reintroduces exactly this bug. Two lessons in the previous edition had one.

---

## Slide 11: Intel HEX — What Actually Reaches the Chip

`Main.hex` is a text file. Open it and the first line of the program you just
built reads:

```
:100000000C9446000C945D000C945D000C945D0013
```

Take it apart:

| Field | Bytes | Meaning |
|---|---|---|
| `:` | — | Every record starts with a colon |
| `10` | 1 | 16 data bytes follow |
| `0000` | 2 | Load them at address 0x0000 |
| `00` | 1 | Record type 00 = data (01 = end of file) |
| `0C9446000C945D00...` | 16 | The data |
| `13` | 1 | Checksum: two's complement of the sum of all the other bytes |

And the last line of every valid image:

```
:00000001FF
```

Zero data bytes, type `01` — end of file.

### Read the data
`0C 94 46 00` is little-endian for the AVR instruction `JMP 0x8c`. `0C 94 5D 00`
is `JMP 0xba`. Those sixteen bytes are the **first four entries of the interrupt
vector table**: reset jumps to the startup code, and the next three jump to
`__bad_interrupt` because this program installs no interrupt handlers.

### Why we check this and not just the exit code
This project once had every lesson pass its build with a 13-byte `Main.hex`
containing the words "Build OK", because `echo Build OK -> Main.hex` inside a
batch file is a **redirect**. Every `Test-Path` check passed. No firmware
existed. `_build/verify-all.ps1` now validates the records themselves.

**Never trust an exit code alone on an embedded project.** Check the artefact.

---

## Slide 12: The Course Environment

```
soc3050code/
├── projects2026_avr/          <-- everything you will run this term
│   ├── 00_Introduction/       <-- you are here
│   ├── 01_Port_Basic/  ...  22_RTOS_Scheduler/
│   ├── Simulator.simu         the one shared board, maintained in one place
│   ├── _build/                the build engine and the verifier
│   └── _slides/               these decks, rendered to HTML
├── shared_libs/               _port, _adc, _uart, _timer, _glcd, _eeprom ...
└── tools/
    ├── avr-toolchain/         avr-gcc 15.1.0 and friends
    └── simulide110sr2/        SimulIDE 1.1.0-SR2
```

**Nothing needs installing.** The compiler and the simulator are both in the
repository. A fresh clone builds and simulates.

### One lesson folder
| File | What it is |
|---|---|
| `Main.c` | The lesson's program |
| `config.h` | `F_CPU`, `BAUD`, and this lesson's board map |
| `Slide.md` | The deck you are reading |
| `README.md` | Ports used, libraries linked, lesson-specific notes |
| `build.bat` | Sets `LIBS`, then calls the shared engine |
| `simulate.bat` | Builds if needed, then opens the board in SimulIDE |

`build.bat` is three lines because the real work is shared:

```bat
set LIBS=
call "%~dp0..\_build\build-lesson.bat"
```

`LIBS` is per-lesson rather than "link everything" for a concrete reason: several
lessons **deliberately write their own ISR** that a shared library also defines.
Linking both is a duplicate-symbol error. Lesson 00 links nothing at all.

---

## Slide 13: The Shared Board

Every lesson opens the same circuit, `Simulator.simu`. `simulate.bat` copies it
next to that lesson's own `Main.hex`, and SimulIDE resolves the MCU's
`Program="Main.hex"` relative to the circuit file — so one maintained board
loads twenty-three different firmwares.

**MCU: ATmega128 at 16 MHz**, matching `-DF_CPU=16000000UL` exactly.

| Port | Wired to |
|---|---|
| **PORTA0–7** | KS0108 graphic LCD data bus |
| **PORTB0–7** | **8 LEDs, active low**; PB0–3 also stepper coils and the SPI bus; PB0 to the scope; PB6–7 to the logic analyser |
| **PORTC** | **nothing at all** — a pin you drive here does nothing visible |
| **PORTD0, D1, D4–D7** | push buttons, pulled up |
| **PORTD2, D3** | **RXD1 / TXD1** — the serial port, not buttons |
| **PORTE3** | joystick; **PORTE4–E7** GLCD control (also INT4–INT7) |
| **PORTF0** | potentiometer (ADC0); **F1, F2** joystick axes |
| **PORTG1** | GLCD control; **PORTG4** slide switch |

### Active low, and why it matters immediately
The LEDs are wired **rail → resistor → anode, cathode → pin**. Current flows
when the pin is pulled to ground, so:

**writing a 0 lights the LED; writing a 1 turns it off.**

`config.h` puts that inversion in exactly one place, so lesson code can read the
way it looks:

```c
#define LED_WRITE(v)  (LED_PORT = (uint8_t)~(uint8_t)(v))

LED_WRITE(0x01);   /* bit 0 set -> PB0 lit.  The macro writes 0xFE. */
```

---

## Slide 14: Your First Program

`00_Introduction/Main.c`, in full outline. Nothing from `shared_libs` — every
line is one you can read today.

```c
#include "config.h"

static void uart_init(void)
{
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * BAUD)) - 1UL);  /* = 103 */
    UBRR1H = (uint8_t)(ubrr >> 8);
    UBRR1L = (uint8_t)ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);          /* transmitter + receiver */
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);        /* 8 data bits, 1 stop    */
}

static void uart_putc(char c)
{
    while (!(UCSR1A & (1 << UDRE1)))   /* wait until the register is free */
        ;
    UDR1 = (uint8_t)c;                 /* writing UDR1 sends the byte */
}

int main(void)
{
    uint8_t position = 0;

    LED_DDR = 0xFF;                    /* all 8 LED pins are outputs */
    LED_WRITE(0x00);                   /* all off */

    BTN_DDR  &= ~(1 << BTN_PAUSE);     /* PD0 is an input... */
    BTN_PORT |=  (1 << BTN_PAUSE);     /* ...with its pull-up enabled */

    uart_init();
    report_architecture();             /* prints the banner */

    while (1)
    {
        LED_WRITE(1 << position);      /* one lit LED, walking */

        if (BTN_PRESSED(BTN_PAUSE))    /* pressed reads 0: active low */
        {
            uart_puts("PD0 held - paused\r\n");
            while (BTN_PRESSED(BTN_PAUSE))
                ;
        }

        _delay_ms(120);
        position = (position + 1) & 7;
    }
}
```

**Setup once, then loop forever** — the shape from Slide 6, with the two halves
of embedded work already present: an output you drive and an input you read.

---

## Slide 15: Running It

```
cd projects2026_avr\00_Introduction
build.bat        compiles Main.c -> Main.elf -> Main.hex
simulate.bat     opens the shared board in SimulIDE 1.1.0-SR2
```

In VS Code, **Ctrl+Shift+B** runs the same build.

`build.bat` should end with:

```
Building 00_Introduction ...

   text    data     bss     dec     hex  filename
   3286       0       0    3286     cd6  Main.elf

Build OK: Main.hex created
```

**3286 bytes** go into Flash — 2.5 % of the 128 KB available. You will watch that
number grow all term.

### Do not trust the `data` and `bss` columns here
They read 0, and this program certainly does use SRAM. This toolchain marks the
`.data` section read-only, so the Berkeley format folds it into `text`. Ask for
the real breakdown instead:

```
> avr-size -A Main.elf
.data     812      <- string literals, COPIED INTO SRAM at reset
.text    2474      <- the instructions
```

812 of your 4096 bytes of RAM are spent holding the banner you are about to
read. Slide 18 shows how to get them back.

### In SimulIDE
1. Press **Play** to start the simulation.
2. Open the **Serial Monitor** on the board's serial port — **9600 baud, 8N1**.
3. The LEDs on PORT B walk one position every 120 ms.
4. Click and hold the **PD0** push button: the walk stops and the monitor says so.

---

## Slide 16: What the Banner Tells You

```
==============================================
 Hello, ATmega128 - SOC3050, lesson 00
==============================================
 Clock          16000000 Hz, 62 ns per cycle
 Serial         USART1, 9600 baud 8N1, UBRR1 = 103
 Flash          131072 bytes = 65536 words of 16 bits
 EEPROM         4096 bytes, in an address space of its own
 SRAM           4096 bytes at 0x0100 - 0x10FF
 Statics        0x0100 - 0x042B, 812 bytes
 Stack          from 0x10FF, growing downwards
 Free RAM       3284 bytes between the two

 Where some of the registers live:
  SREG   data 0x005F   I/O 0x3F
  DDRB   data 0x0037   I/O 0x17
  PORTB  data 0x0038   I/O 0x18
  PINB   data 0x0036   I/O 0x16
  UDR1   data 0x009C   I/O  -  (needs LDS/STS)
```

None of those numbers is typed into the program. The addresses come from
`&PORTB` and friends, and the memory figures from linker symbols, so the banner
reports what was actually built rather than what the slides claim.

**Four things to notice, each of which the rest of this deck explains:**
- The stack starts at **0x10FF**, the very last byte of SRAM, and grows
  *downwards* towards your variables.
- **812 bytes are already gone** before `main` does anything — that is this
  banner's own text, sitting in RAM. Slide 18 shows why, and how to avoid it.
- `PORTB` is at data address **0x38** and I/O address **0x18** — one register,
  two numbering schemes, exactly 0x20 apart.
- `UDR1` has **no** I/O address. It lives above 0x5F, which is why the compiler
  must reach it with a different instruction.

---

## Slide 17: The ATmega128 at a Glance

**8-bit AVR RISC microcontroller**, introduced 2002, still in production.

| Property | Value |
|---|---|
| Architecture | 8-bit **RISC**, **Harvard**, 2-stage pipeline |
| Instructions | **133**, most executing in **one clock cycle** |
| Working registers | **32 x 8-bit**, all directly connected to the ALU |
| Throughput | **up to 16 MIPS at 16 MHz** |
| Flash | **128 KB**, in-system programmable, 10,000 write cycles |
| SRAM | **4 KB** internal (plus up to 64 KB external) |
| EEPROM | **4 KB**, 100,000 write cycles |
| I/O pins | **53**, in ports A–F (8 bits each) and G (5 bits) |
| Package | 64-pin TQFP |
| Supply | 4.5 – 5.5 V (2.7 – 5.5 V for the ATmega128L) |
| Speed grade | 0 – 16 MHz |

### RISC, and what "one cycle" buys you
A **Reduced Instruction Set** means many simple instructions rather than few
complex ones. `ADD r16, r17` does one thing and finishes in one clock. At
16 MHz that is **62.5 ns**.

The pay-off is *determinism*. Count the instructions in a loop and you know its
duration exactly — not on average, not usually. On a desktop CPU, with caches
and out-of-order execution, the same loop takes a different time on every run.
Half the techniques in this course depend on the AVR's predictability.

---

## Slide 18: Harvard Architecture — Two Memories, Two Buses

```
             +---------------------------+
   FLASH     |                           |     SRAM
   128 KB    |        AVR CPU CORE       |     4 KB
   program   |                           |     data
      |      |   +-------------------+   |       |
      +----->|   | 32 x 8 registers  |   |<----->+
   16-bit    |   |  R0 ... R31       |   |    8-bit
   instr.    |   +---------+---------+   |    data bus
   bus       |             |             |
             |          +--v--+          |
             |          | ALU |          |
             |          +--+--+          |
             |             |             |
             |          +--v---+         |
             |          | SREG |         |
             |          +------+         |
             +---------------------------+
                  |                 |
              PERIPHERALS        EEPROM 4 KB
              timers, ADC,       (a third space,
              UART, SPI, TWI      reached through
              ports              its own registers)
```

**Von Neumann** machines (your laptop) keep code and data in one memory on one
bus. **Harvard** machines keep them separate, with separate buses.

### Why it is faster
The CPU can **fetch the next instruction while executing the current one** —
they use different buses, so they never contend. That two-stage overlap is what
makes single-cycle execution possible.

### The price you pay
Code and data are in **different address spaces**, so `0x0100` is ambiguous: it
means one thing in Flash and another in SRAM. An ordinary C pointer reads SRAM.
To read a constant table stored in Flash you need `PROGMEM` and `pgm_read_byte`:

```c
static const uint8_t table[] PROGMEM = { 1, 2, 4, 8 };

uint8_t v = pgm_read_byte(&table[i]);   /* LPM: a flash-specific instruction */
```

Forget the `static`, or use plain `table[i]`, and the compiler reads the *SRAM*
address that happens to have the same number. It compiles, it runs, it returns
garbage. That exact bug was found and fixed in this repository.

### This is where lesson 00's 812 bytes went
A string literal is data. By default the linker puts it in `.data`, which means
a copy is made **in SRAM** at reset — even though the text never changes.

```c
uart_puts("Hello, ATmega128
");        /* the text lives in RAM */

uart_puts_P(PSTR("Hello, ATmega128
")); /* the text stays in Flash */
```

`PSTR` keeps the literal in Flash and hands back a Flash address; the `_P`
version of the print routine reads it with `pgm_read_byte`. Two extra
characters per string, and a banner this size costs nothing in RAM. On a chip
where RAM is 32 times scarcer than Flash, that trade is almost always right.

---

## Slide 19: The Register File

Thirty-two 8-bit registers, R0 to R31, all wired straight to the ALU. Any two
can be an instruction's operands, and the result lands in one clock.

```
   R0  ..  R15    general purpose
                  (LDI cannot reach these)
   ----------------------------------------
   R16 ..  R25    general purpose
                  (LDI, ANDI, ORI, SUBI, CPI work here)
   ----------------------------------------
   R26 R27  = X   \
   R28 R29  = Y    >  16-bit pointer pairs for indirect addressing
   R30 R31  = Z   /   (Z also addresses Flash, for LPM)
```

### Two facts that explain a lot of compiler output
- **`LDI` only reaches R16–R31.** Loading an immediate constant into R0–R15 is
  impossible in one instruction, which is why avr-gcc keeps constants high in the
  file and why hand-written assembly in lesson 04 always uses R16 and above.
- **R1 is kept at zero** by the C calling convention, and R0 is a scratch
  register. You will see `eor r1, r1` as the very first instruction after reset:
  that is the startup code zeroing R1.

### SREG — the status register, at I/O 0x3F
| Bit | Name | Set when |
|---|---|---|
| 7 | **I** | **Global interrupt enable** — `sei()` sets it, `cli()` clears it |
| 6 | T | Bit copy storage |
| 5 | H | Half carry |
| 4 | S | Sign (N xor V) |
| 3 | V | Two's-complement overflow |
| 2 | N | Result negative |
| 1 | **Z** | **Result zero** |
| 0 | **C** | **Carry** |

Every conditional branch you write is a test of one of these bits.

---

## Slide 20: The Memory Map

The data space is **one flat 16-bit address range** with the registers, the I/O
and the SRAM all inside it.

| Address range | Size | What lives there |
|---|---|---|
| `0x0000 – 0x001F` | 32 | The **register file**, R0–R31 |
| `0x0020 – 0x005F` | 64 | **I/O registers** — reachable by `IN` / `OUT` |
| `0x0060 – 0x00FF` | 160 | **Extended I/O** — `LDS` / `STS` only |
| `0x0100 – 0x10FF` | 4096 | **Internal SRAM** — your variables, heap and stack |
| `0x1100 – 0xFFFF` | up to 61 KB | External SRAM, if any is fitted |

```
  0x0000 +------------------+
         |  R0 .. R31       |   32 registers, also addressable as memory
  0x0020 +------------------+
         |  I/O registers   |   PORTB is here, at 0x38
  0x0060 +------------------+
         |  Extended I/O    |   UDR1 is here, at 0x9C
  0x0100 +------------------+
         |  .data           |   initialised globals
         |  .bss            |   zeroed globals
         |  heap  |         |   grows up
         |        v         |
         |                  |   <-- free RAM: what the banner measured
         |        ^         |
         |  stack |         |   grows down
  0x10FF +------------------+   SP starts here
```

### The 4 KB problem
In lesson 00, statics occupy 0x0100–0x042B and the stack starts at 0x10FF,
leaving 3284 bytes in the middle. The heap grows up into it, the stack grows
down into it, and **nothing checks whether they meet**. When they do, a local variable and a heap object occupy the same bytes
and the program misbehaves in a way that looks like anything but the real cause.

This is the strongest argument for the embedded habit of **static allocation**:
fixed-size buffers, sized at compile time, so `avr-size` can tell you before you
flash the chip whether it fits.

---

## Slide 21: I/O and Extended I/O — Seen in the Compiler's Output

Slide 16 said `PORTB` has two addresses and `UDR1` only one. Here is why it
matters, taken from the disassembly of the program you just built:

```asm
; LED_DDR = 0xFF;  and  LED_WRITE(0x00);   both values are 0xFF
 174:   eor  r13, r13
 176:   dec  r13            ; r13 = 0xFF, built in two cycles from nothing
 178:   out  0x17, r13      ; DDRB  - one instruction, one cycle
 17a:   out  0x18, r13      ; PORTB - one instruction, one cycle

; BTN_DDR &= ~(1 << PD0);  and  BTN_PORT |= (1 << PD0);
 17c:   cbi  0x11, 0        ; DDRD  bit 0 -> input,   one instruction
 17e:   sbi  0x12, 0        ; PORTD bit 0 -> pull-up, one instruction

; uart_init(): UBRR1H, UBRR1L, UCSR1B, UCSR1C
 180:   sts  0x0098, r1     ; UBRR1H = 0     - two words, two cycles
 188:   sts  0x0099, r15    ; UBRR1L = 103
 18e:   sts  0x009A, r24    ; UCSR1B = 0x18
 196:   sts  0x009D, r24    ; UCSR1C = 0x06
```

| | I/O space (0x00–0x3F) | Extended I/O (0x60+) |
|---|---|---|
| Instructions | `IN`, `OUT`, `SBI`, `CBI`, `SBIC`, `SBIS` | `LDS`, `STS` only |
| Size | 1 word | 2 words |
| Speed | 1 cycle | 2 cycles |
| Single-bit ops | **Yes, atomically** | No — read, modify, write |

**`SBI` and `CBI` set or clear one bit in one uninterruptible instruction.** In
extended I/O the same change is three separate steps, and an interrupt landing
between them can lose a write. That is a real class of bug, and lesson 05 shows
what to do about it.

Notice also that the compiler recognised `PORT |= (1 << n)` on a low-address
register and emitted `SBI` by itself. **Write the clear thing; the optimiser is
better at this than you are.**

---

## Slide 22: What Is On the Chip

Everything below is inside the ATmega128, and each has a lesson.

| Peripheral | What it does | Lesson |
|---|---|---|
| **Digital I/O**, 53 pins | Read and drive individual pins | 01, 02, 03 |
| **External interrupts** INT0–INT7 | Run code on a pin edge | 05 |
| **Timer/Counter0**, 8-bit | Periodic ticks, PWM | 06 |
| **Timer/Counter1**, 16-bit | Precise frequencies, input capture, 3 compare channels | 07, 08, 09, 10 |
| **Timer/Counter2**, 8-bit | A second 8-bit timer, asynchronous operation | 20 |
| **Timer/Counter3**, 16-bit | A second 16-bit timer, identical to Timer1 | — |
| **PWM**, 6 channels | Motor speed, servo position, brightness | 09, 10 |
| **ADC**, 8 channels, 10-bit | Turn a voltage into a number 0–1023 | 11, 12 |
| **USART0 / USART1** | Asynchronous serial; the board uses **USART1** | 13, 14 |
| **SPI** | Fast synchronous serial to nearby chips | 15 |
| **TWI (I2C)** | Two wires, many devices, addressed | 16 |
| **EEPROM**, 4 KB | Data that survives power loss | 19 |
| **Analog comparator** | Compare two voltages in hardware | — |
| **Watchdog timer** | Reset the chip if the program stops feeding it | 21 |
| **Sleep modes**, 6 of them | Trade responsiveness for microamps | 20 |

**All of it for a few dollars, on one chip, with no external components beyond a
crystal and a decoupling capacitor.** That density is the whole argument for
microcontrollers.

---

## Slide 23: Interrupts and the Vector Table

Polling means asking "has it happened yet?" in a loop. **Interrupts** invert
that: the hardware stops the CPU where it stands, runs your handler, and resumes.

The ATmega128 has **35 vectors**, four bytes each, occupying `0x0000`–`0x0088`
in Flash. Your program's very first bytes *are* that table:

| # | Address | Source | avr-libc name |
|---|---|---|---|
| 1 | 0x0000 | **RESET** | — |
| 2 | 0x0002 | INT0 | `INT0_vect` |
| ... | ... | INT1 – INT7 | `INT1_vect` … |
| 11 | 0x0014 | Timer2 overflow | `TIMER2_OVF_vect` |
| 17 | 0x0020 | Timer0 overflow | `TIMER0_OVF_vect` |
| 31 | 0x003C | USART1 receive complete | `USART1_RX_vect` |
| 35 | 0x0044 | Store program memory ready | `SPM_READY_vect` |

Lower address means **higher priority** when two fire at once. RESET is at
0x0000, so a reset always wins.

### In this program's own disassembly
```asm
00000000 <__vectors>:
   0:  0c 94 46 00   jmp 0x8c    ; RESET -> the startup code
   4:  0c 94 5d 00   jmp 0xba    ; INT0  -> __bad_interrupt
   8:  0c 94 5d 00   jmp 0xba    ; INT1  -> __bad_interrupt
   ...                            ; 35 entries, ending at 0x88
```

Lesson 00 installs no handlers, so every vector but RESET points at
`__bad_interrupt`, which jumps to 0x0000 — an unhandled interrupt **resets the
chip**. Writing `ISR(TIMER0_OVF_vect) { ... }` replaces one of those entries.

Two rules from the start: an ISR must be **short**, and any variable it shares
with `main` must be **`volatile`**.

---

## Slide 24: Clock and Timing

One clock drives everything. At 16 MHz:

| | |
|---|---|
| One clock cycle | **62.5 ns** |
| Most instructions | 1 cycle = 62.5 ns |
| Multiply, load, store | 2 cycles = 125 ns |
| `CALL` / `RET` | 4 cycles = 250 ns |
| Timer0 overflow, no prescaler | 256 x 62.5 ns = **16 µs** |
| Timer0 overflow, prescaler 1024 | **16.384 ms** |

### Prescalers
A peripheral rarely wants the full 16 MHz. A **prescaler** divides the clock
before it reaches the counter — /8, /64, /256, /1024 — which is how an 8-bit
timer covers microseconds *and* tens of milliseconds. Every timer lesson begins
by choosing one.

### `_delay_ms` is not a timer
```c
_delay_ms(120);   /* a busy loop, counted out from F_CPU at compile time */
```
The CPU executes nothing else for those 120 ms. It cannot respond to a button,
service a UART, or update a display. That is acceptable in lesson 00 and
unacceptable by lesson 06 — which is exactly why timers and interrupts are the
next thing you learn.

The argument must also be a **compile-time constant**: `_delay_ms(n)` with a
variable `n` drags in floating-point code and stops being accurate.

---

## Slide 25: Exercises

1. **Read the size line.** Build the lesson and record the `text` figure. Now
   change `sprintf` in `report_architecture` to plain `uart_puts` calls with
   fixed strings, rebuild, and compare. How many bytes does `sprintf` cost, and
   why is that worth knowing on a 128 KB chip?
2. **Break `F_CPU` deliberately.** Add `#define F_CPU 8000000UL` *above* the
   guard in `config.h`, rebuild, and run it. Time the LED walk and read the
   serial output. Explain both symptoms from one cause, then put it back.
3. **Decode a record.** Take the third line of `Main.hex`, split it into the six
   Intel HEX fields, and verify the checksum by hand.
4. **Find a register.** Using the datasheet, give the data address and the I/O
   address of `TCCR0`, `ADMUX` and `SPCR`. Which of the three can `OUT` reach?
5. **Predict the instruction.** Before compiling, say whether
   `PORTB |= (1 << PB3);` and `PORTB |= (1 << PB3) | (1 << PB4);` produce `SBI`
   or a read-modify-write. Check with
   `avr-objdump -d Main.elf` and explain the difference.
6. **Count the vector table.** Confirm from the disassembly that it is 35 entries
   and ends at 0x88. What would `-mmcu=atmega328p` have produced instead?
7. **Measure the free RAM.** Add a `char big[2048];` global, rebuild, and compare
   `avr-size -A Main.elf` with the banner's free-RAM line. Then raise it to 4096
   and describe exactly what the linker says. At what size does the build stop
   being *safe*, as opposed to stop *linking*?
8. **Move the strings to Flash.** Convert `report_architecture` to `PSTR` and a
   `uart_puts_P` helper, as on Slide 18. Report the change in `.data` and in the
   banner's own free-RAM figure.
9. **Change the walk.** Make the LEDs bounce back and forth instead of wrapping,
   and make PD1 reverse the direction rather than pausing.

---

## Summary

### Key Points
✓ **Embedded systems are the majority** — 25+ billion MCUs a year, none of them visible
✓ **A microcontroller is a whole computer on one die** — CPU, Flash, SRAM, peripherals
✓ **Constraints define the discipline** — 4 KB of RAM, 8-bit words, no OS, no safety net
✓ **Bare metal means `main` never returns** — set up once, then loop forever
✓ **Peripheral registers are the API** — writing a number to an address *is* the control
✓ **Cross-compilation** — `avr-gcc` runs on your PC and emits code for the AVR
✓ **`.c` → `.elf` → `.hex`** — and only the `.hex` reaches the chip
✓ **`F_CPU` is a promise, not a measurement** — wrong value, wrong delays and baud rate
✓ **The ATmega128 is Harvard 8-bit RISC** — separate code and data spaces, 1 cycle per instruction
✓ **Two address schemes for one register** — I/O address = data address − 0x20
✓ **35 interrupt vectors** occupy the first 0x88 bytes of Flash

### Numbers Worth Memorising
| | |
|---|---|
| Clock | 16 MHz, 62.5 ns per cycle |
| Flash / SRAM / EEPROM | 128 KB / 4 KB / 4 KB |
| Registers | 32, of which R16–R31 take immediates |
| I/O space | data 0x20–0x5F, I/O 0x00–0x3F |
| SRAM | 0x0100 – 0x10FF, stack starts at the top |
| Vectors | 35, from 0x0000 to 0x0088 |
| Serial | USART1, 9600 8N1, UBRR1 = 103 |

### Best Practices from Day One
1. **Read the size output** after every build — `text` is Flash, `bss` is RAM
2. **Validate the artefact, not the exit code** — a green build can produce no firmware
3. **Keep `F_CPU` in one place**, guarded, matching the board
4. **Prefer static allocation** — the heap and the stack share 4 KB and nobody is watching
5. **Treat warnings as errors** — there is no memory protection to catch what you missed
6. **The datasheet is the textbook**; these slides are the reading guide

### Next Steps
- **Lesson 01** — `DDR`, `PORT` and `PIN` properly: driving LEDs, reading switches
- **Lesson 02** — inputs, pull-ups and why a mechanical button lies to you
- **Lesson 05, 06** — interrupts and timers, and the end of `_delay_ms`

---

## References and Resources

### Documentation
- ATmega128 Datasheet — the primary reference for the whole course
  - Section 5, "AVR CPU Core" — the register file, SREG, the ALU
  - Section 6, "Memories" — Flash, SRAM, EEPROM and the memory map
  - Section 9, "Interrupts" — Table 23, the 35 vectors
  - Section 13, "I/O Ports" — `DDRx`, `PORTx`, `PINx`
- [AVR Instruction Set Manual](https://ww1.microchip.com/downloads/en/devicedoc/atmel-0856-avr-instruction-set-manual.pdf) — every instruction with its cycle count
- [avr-libc manual](https://www.nongnu.org/avr-libc/user-manual/) — `avr/io.h`, `util/delay.h`, `avr/pgmspace.h`

### In This Repository
- `projects2026_avr/README.md` — the board map and the full lesson list
- `_build/build-lesson.bat` — the exact compiler invocation from Slide 9
- `_build/verify-all.ps1` — the Intel HEX validation from Slide 11
- `00_Introduction/README.md` — running this lesson, and what to do when it does not run

### Further Reading
- Elliot Williams, *Make: AVR Programming* — the AVR peripherals, worked through
- Jack Ganssle, *The Embedded Muse* — thirty years of field notes on this discipline
- Michael Barr, *Embedded C Coding Standard* — short, opinionated, and worth the hour
