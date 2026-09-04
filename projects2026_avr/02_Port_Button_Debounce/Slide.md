# Buttons and Debouncing
## ATmega128 Embedded Systems Course

**Reference**: [ATmega128 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf) — Section 13, I/O Ports

---

## Slide 1: Why Buttons Are Harder Than LEDs

An LED is an output: you write a bit and the world obeys. A button is an input,
and the world does not obey — it argues.

A mechanical switch is two pieces of springy metal. When they meet they do not
settle instantly; they collide, separate, collide again, and ring down over a
few milliseconds. The pin sees a burst of edges, not one clean transition.

```
Ideal (what you assume):        Real (what the pin sees):

 5V ────┐                        5V ────┐ ┌┐ ┌──┐
        │                               │ ││ │  │
 0V     └────────                0V     └─┘└─┘  └────────
        ^                               ^~~~~~~~^
        press                           press    settled
                                        |<- 1 to 20 ms ->|
```

### The consequence
A loop that counts every falling edge will count one press as five. A menu will
skip three entries. A counter will jump. Nothing is broken in your logic — the
input is simply lying to you for a few milliseconds.

### What this lesson covers
- Reading a pin, and why `PIN` is not `PORT`
- Pull-up resistors and why a floating input is meaningless
- Contact bounce, measured and then removed
- A debounce routine you can reuse, and its cost

---

## Slide 2: Reading an Input Pin

Three registers control one pin. For inputs you use all three.

| Register | Role for an input pin |
|----------|----------------------|
| **DDRx**  | Write **0** to make the pin an input |
| **PORTx** | Write **1** to switch the internal pull-up **on** |
| **PINx**  | **Read** this to get the pin's actual level |

The classic beginner bug is reading `PORTD` instead of `PIND`. `PORTD` tells you
what you asked for; `PIND` tells you what is actually there.

```c
DDRD  &= ~(1 << PD0);   // PD0 is an input
PORTD |=  (1 << PD0);   // internal pull-up on

if (!(PIND & (1 << PD0)))
{
    // pin is LOW -> button is pressed
}
```

### Why the test is inverted
The board wires each button between the pin and **ground**. The pull-up holds
the pin HIGH while nothing is happening; pressing the button shorts it to
ground. So *pressed* reads as **0**, not 1. This is called *active low*, and it
is the normal arrangement — it needs no external parts and it is immune to a
disconnected wire reading as a phantom press.

---

## Slide 3: The Pull-Up Resistor

Leave an input pin unconnected and it is *floating*: its voltage is whatever
nearby wiring and your fingertip induce. It will read 0 and 1 at random.

```
        VCC (5 V)
          │
         ┌┴┐
         │ │  internal pull-up, roughly 20 k to 50 k ohm
         └┬┘
          │
  PD0 ────┼──────────  reads HIGH while the button is open
          │
         ─┴─  button
          │
         GND            pressing pulls the pin to 0 V
```

The ATmega128 has one of these on every I/O pin, and enabling it costs a single
bit. Use it. An external resistor is only needed when you want a different
value, or when `PUD` in `SFIOR` has globally disabled the internal ones.

### On this board
The shared SimulIDE board has push buttons on **PD0, PD1, PD4, PD5, PD6 and
PD7**, each wired to ground with a supply rail on the other side.
**PD2 and PD3 are not buttons** — they are RXD1 and TXD1, the serial link this
lesson prints to. That is why the code uses PD4 for its third button.

---

## Slide 4: Measuring the Bounce

Before removing bounce, look at it. Run this and watch the count climb on a
single press:

```c
uint8_t previous = 1;
uint32_t edges = 0;

while (1)
{
    uint8_t now = (PIND & (1 << PD0)) ? 1 : 0;
    if (previous == 1 && now == 0)   // falling edge, no debounce
        edges++;
    previous = now;
}
```

### Typical results
| Switch type | Bounce duration | Edges per press |
|-------------|-----------------|-----------------|
| Tactile push button | 1 – 5 ms | 2 – 10 |
| Cheap slide switch | 5 – 20 ms | 5 – 50 |
| Relay contact | 1 – 10 ms | 2 – 20 |
| Reed switch | up to 1 ms | 1 – 3 |

The numbers vary with the part, its age and how hard it is pressed. That is the
point: bounce is not a fixed quantity you can compensate for, it is noise you
have to reject.

---

## Slide 5: Debouncing by Waiting

The simplest cure: after seeing a change, wait past the bouncing window, then
look again. If the pin still says the same thing, believe it.

```c
#define DEBOUNCE_MS 50

uint8_t button_read(uint8_t pin)
{
    if (!(PIND & (1 << pin)))          // looks pressed
    {
        _delay_ms(DEBOUNCE_MS);        // let the contacts settle
        if (!(PIND & (1 << pin)))      // still pressed -> it is real
        {
            while (!(PIND & (1 << pin)))
                ;                      // wait for release
            _delay_ms(DEBOUNCE_MS);    // and let the release settle too
            return 1;
        }
    }
    return 0;
}
```

### Reading it carefully
1. **First test** — a candidate press.
2. **Delay** — 50 ms is far longer than any bounce; the contacts are settled.
3. **Second test** — rejects a spike that was never a press.
4. **Wait for release** — makes one press produce exactly one event, instead of
   repeating for as long as the finger is down.
5. **Delay again** — the release bounces too, and would otherwise look like a
   fresh press.

### What it costs
`_delay_ms` is a busy loop. During those 50 ms the CPU does nothing else, and
`while (!(PIND & ...))` blocks for as long as the user holds the button — which
could be forever. For this lesson that is fine. For anything with a display to
refresh or a motor to control, it is not.

---

## Slide 6: Debouncing Without Blocking

The same idea, restructured so the main loop keeps running. Sample the pin on a
timer tick and require several agreeing samples in a row.

```c
/* Called every 1 ms from a timer interrupt. */
uint8_t button_poll(uint8_t pin)
{
    static uint8_t history = 0xFF;   // last 8 samples, 1 = released
    static uint8_t state   = 1;      // debounced level

    history = (history << 1) | ((PIND & (1 << pin)) ? 1 : 0);

    if (history == 0x00 && state == 1) { state = 0; return 1; }  // pressed
    if (history == 0xFF)                 state = 1;              // released
    return 0;
}
```

Eight consecutive zeros means the pin has read LOW for 8 ms without a single
disagreement — no bounce burst survives that. Nothing blocks; the function
returns immediately every time.

| | Blocking version | Sampling version |
|---|---|---|
| CPU while waiting | fully occupied | free |
| Holding the button | stalls the program | no effect |
| Multiple buttons | handled one at a time | all handled together |
| Needs a timer | no | yes |
| Lines of code | fewer | slightly more |

---

## Slide 7: Debouncing in Hardware

Sometimes the right answer is not code.

```
            VCC
             │
            ┌┴┐ 10k
            └┬┘
             ├──────────── to pin
    button  ─┴─          │
             │          ═╪═ 100 nF
            GND          │
                        GND
```

The capacitor cannot change voltage instantly, so the contact chatter is
smoothed into one slow edge. Feed that into a **Schmitt-trigger** input (the
ATmega128's digital inputs are Schmitt-triggered) and you get one clean
transition, with no CPU cost at all.

**RC time constant** = R x C = 10 kΩ x 100 nF = 1 ms, which is the right order
for a tactile switch.

Use hardware when the pin drives an interrupt, when timing must be exact, or
when you have more buttons than patience. Use software when the parts budget is
zero, which is most of the time.

---

## Slide 8: The Complete Lesson Program

```c
#include "config.h"

#define DEBOUNCE_MS 50

int main(void)
{
    init_devices();
    uart_init_b();

    /* PD2 and PD3 are RXD1/TXD1, so the third button is PD4. */
    DDRD  &= ~((1 << PD0) | (1 << PD1) | (1 << PD4));
    PORTD |=  (1 << PD0) | (1 << PD1) | (1 << PD4);
    DDRB  |=  (1 << PB7) | (1 << PB6) | (1 << PB5);

    uint32_t count0 = 0;

    while (1)
    {
        if (button_read(0))
        {
            count0++;
            PORTB ^= (1 << PB7);       // toggle works whatever the polarity
            char msg[40];
            sprintf(msg, "BTN0: %lu\r\n", count0);
            uart_puts(msg);
        }
        /* ... same for PD1 and PD4 ... */
    }
}
```

### Why `PORTB ^=` and not `PORTB =`
The board's LEDs are **active low** — anode on the 5 V rail, cathode on the pin
— so writing a 1 turns an LED *off*. A toggle sidesteps the question entirely:
XOR flips whatever was there, and the LED changes state either way.

---

## Slide 9: Running It

```
cd projects2026_avr\02_Port_Button_Debounce
build.bat        # produces Main.hex
simulate.bat     # opens the shared board in SimulIDE 1.1.0-SR2
```

In SimulIDE:
1. Press **Play** to start the simulation.
2. Open the **Serial Monitor** on the board's serial port, 9600 baud.
3. Click a push button on PD0, PD1 or PD4.
4. One click gives exactly one line and one LED change.

### If the count jumps by more than one
The debounce is being skipped. Check that `DEBOUNCE_MS` really is being compiled
in, and that `F_CPU` is 16 MHz — `_delay_ms` derives its loop count from `F_CPU`,
so a wrong clock makes a 50 ms delay much shorter or longer than it claims.

---

## Slide 10: Exercises

1. **Measure it.** Remove the debounce and count raw edges for a single press.
   Report the number for three different buttons on the board.
2. **Find the threshold.** Reduce `DEBOUNCE_MS` until miscounts return. What is
   the smallest reliable value on this board?
3. **Long press.** Extend `button_read` to distinguish a short press from one
   held longer than a second, and print which occurred.
4. **Non-blocking.** Replace the blocking routine with the 8-sample version from
   Slide 6, driven by a Timer0 overflow interrupt. Confirm the LEDs still
   respond while a button is held down.
5. **Two at once.** Make the program detect PD0 and PD1 pressed together as a
   distinct third event. Note what the blocking version cannot do here.
6. **Repeat rate.** Add auto-repeat: holding a button emits an event every
   200 ms after an initial 500 ms delay, like a keyboard.

---

## Summary

### Key Points
✓ **Inputs need three registers** — `DDRx` = 0, `PORTx` = 1 for pull-up, read `PINx`
✓ **Read PINx, never PORTx** — `PORTx` is what you wrote, `PINx` is what is there
✓ **Buttons are active low** — pressed reads 0, because the switch goes to ground
✓ **Bounce is real** — one press produces several edges over 1 to 20 ms
✓ **Debounce = confirm, then wait** — test, delay past the bounce, test again
✓ **Blocking is a choice** — `_delay_ms` is simple but stops everything

### Register Summary
| Register | Purpose |
|----------|---------|
| **DDRD** | 0 = input, 1 = output |
| **PORTD** | on an input pin, 1 enables the internal pull-up |
| **PIND** | actual pin levels, read-only |
| **SFIOR** | `PUD` bit disables every pull-up at once |

### Best Practices
1. **Always enable a pull-up** on a switch input, or read noise
2. **Debounce every mechanical contact**, including relays and reed switches
3. **Wait for release** if one press must mean one event
4. **Prefer sampling over blocking** as soon as anything else needs the CPU
5. **Add hardware RC filtering** when the button drives an interrupt

### Next Steps
- **Matrix keypads** — the same problem across 16 keys and 8 pins
- **External interrupts** — reacting to an edge instead of polling for it
- **Timer-driven sampling** — the foundation of a non-blocking input layer

---

## References and Resources

### Documentation
- ATmega128 Datasheet, Section 13 "I/O Ports" — pull-up configuration
- ATmega128 Datasheet, Table 13-1 — `DDxn` / `PORTxn` combinations
- `shared_libs/_port.c` — the framework's board map and LED conventions

### Related Lessons
- `01_Port_Basic` — the three port registers from first principles
- `03_Port_Keypad_Matrix` — scanning many buttons with few pins
- `05_INT_External_Pins` — edge-triggered input without polling

### Further Reading
- Jack Ganssle, *A Guide to Debouncing* — measurements of dozens of real switches
- AVR Application Note AVR243 — matrix keyboard scanning
