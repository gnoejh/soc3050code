# External Interrupts
## ATmega128 Embedded Systems Course

**Reference**: [ATmega128 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf) — Section 11, External Interrupts

---

## Slide 1: Polling Versus Interrupting

In the previous lesson the CPU asked "is the button down yet?" thousands of
times a second. That works, but it has two costs: the CPU can do nothing else
while it asks, and anything shorter than the gap between two questions is
missed entirely.

An interrupt inverts the arrangement. The hardware watches the pin. When the
event happens it stops whatever the CPU was doing, runs a short handler, and
puts the CPU back exactly where it was.

```
Polling:                        Interrupt:

  while (1) {                     while (1) {
      check_button();                 do_real_work();
      do_real_work();             }
  }                               ISR(INT0_vect) { ... }
      ^                                    ^
  work delayed by every check      hardware calls you, only when needed
```

### What you gain
- **Latency** — response in microseconds, not "next time round the loop"
- **Efficiency** — no cycles spent asking about things that did not happen
- **Short events** — a 10 µs pulse is caught, not missed between polls

### What you pay
- Code now runs in two places at once, and they share variables
- The order of events becomes harder to reason about
- A bug in an ISR is much harder to see

---

## Slide 2: The ATmega128's Eight External Interrupts

Eight pins can generate an external interrupt, split across two ports.

| Interrupt | Pin | Also known as | Vector |
|-----------|-----|---------------|--------|
| **INT0** | PD0 | SCL | `INT0_vect` |
| **INT1** | PD1 | SDA | `INT1_vect` |
| **INT2** | PD2 | RXD1 | `INT2_vect` |
| **INT3** | PD3 | TXD1 | `INT3_vect` |
| **INT4** | PE4 | OC3B | `INT4_vect` |
| **INT5** | PE5 | OC3C | `INT5_vect` |
| **INT6** | PE6 | T3 | `INT6_vect` |
| **INT7** | PE7 | IC3 | `INT7_vect` |

Lower vector number means higher priority when two fire at once.

### An important asymmetry
**INT0 to INT3 can only be triggered by a *level*, or by an edge detected from
the I/O clock.** **INT4 to INT7 have full asynchronous edge detection** and can
wake the chip from deep sleep on an edge. This matters in the power lesson: if
you need an edge to wake a sleeping CPU, use INT4–INT7 or a low level on
INT0–INT3.

### On this board
PD0 and PD1 carry push buttons, so **INT0 and INT1 are the two this lesson
uses**. PD2 and PD3 are the serial port and must not be repurposed. PE4–PE7 are
wired to the graphic LCD's control lines.

---

## Slide 3: Choosing the Trigger — EICRA and EICRB

Each interrupt has two bits saying *what* counts as an event.

**EICRA** covers INT0–INT3, **EICRB** covers INT4–INT7.

| ISCn1 | ISCn0 | Triggers on |
|:-----:|:-----:|-------------|
| 0 | 0 | **Low level** — fires continuously while the pin is low |
| 0 | 1 | **Any edge** — both rising and falling |
| 1 | 0 | **Falling edge** |
| 1 | 1 | **Rising edge** |

```c
EICRA = (1 << ISC01);                  // INT0 on a falling edge
EICRA = (1 << ISC11) | (1 << ISC10);   // INT1 on a rising edge
```

### Read the bit positions carefully
`EICRA` packs four interrupts into eight bits: INT0 in bits 1:0, INT1 in bits
3:2, INT2 in 5:4, INT3 in 7:6. Writing `EICRA = (1 << ISC01)` sets INT0 and
**clears the other three** — which is exactly what the lesson wants when it
switches demos, but is a trap if you meant to configure them independently. Use
`|=` and a mask when several interrupts are live at once.

### Why "low level" is dangerous
A low-level interrupt re-fires as long as the pin stays low. Hold the button
down and the ISR runs back to back, forever, and the main program never
advances. Use it only when you will remove the cause inside the ISR.

---

## Slide 4: Enabling — EIMSK and EIFR

```c
EIMSK |= (1 << INT0);    // let INT0 through
sei();                   // and allow interrupts globally
```

| Register | Bit | Meaning |
|----------|-----|---------|
| **EIMSK** | INT7..INT0 | 1 = this interrupt is enabled |
| **EIFR**  | INTF7..INTF0 | 1 = this interrupt is pending |
| **SREG**  | I (bit 7) | the global enable, set by `sei()` |

Both the individual mask **and** the global flag must be set. Forgetting `sei()`
is the single most common reason an ISR "never runs".

### Clearing a stale flag
Changing the trigger condition can itself set the flag, so a spurious interrupt
fires the moment you enable it. Clear it first — and note the odd convention:

```c
EIFR |= (1 << INTF0);    // writing ONE clears the flag
```

Writing a one to clear looks backwards, but it lets you clear one flag without
a read-modify-write that might lose another arriving in between.

---

## Slide 5: Writing the Handler

```c
volatile uint32_t int0_count = 0;

ISR(INT0_vect)
{
    int0_count++;
    PORTB ^= (1 << PB7);
}
```

### `volatile` is not optional
`int0_count` is written by the ISR and read by `main`. Without `volatile` the
compiler is entitled to notice that `main` never changes it, cache it in a
register, and print the same value forever. `volatile` forces every access back
to memory. **Every variable shared between an ISR and main code must be
`volatile`.**

### Keep it short
An ISR runs with interrupts disabled. Everything else waits. The handler above
is two instructions of real work — that is the right size.

**Do not** call `_delay_ms`, `printf`, or anything that blocks, from an ISR.
Set a flag; do the work in `main`:

```c
volatile uint8_t pressed = 0;

ISR(INT0_vect) { pressed = 1; }          // fast

int main(void) {
    while (1) {
        if (pressed) { pressed = 0; handle_it(); }   // slow, but safe
    }
}
```

### Atomic access to multi-byte variables
`int0_count` is 32 bits, so the AVR reads it in four instructions. An interrupt
landing in the middle gives you two halves of two different values. When it
matters, read it with interrupts off:

```c
uint32_t snapshot;
uint8_t sreg = SREG;
cli();
snapshot = int0_count;
SREG = sreg;            // restores the previous interrupt state
```

---

## Slide 6: Interrupts and Bounce

An interrupt is faster than a human, and much faster than a bouncing contact.
Wiring an undebounced button to INT0 gives you five to ten interrupts per press.

Three ways out:

1. **Hardware filter** — an RC network plus the pin's Schmitt trigger, as in the
   debounce lesson. The ISR then fires once, and costs nothing extra.
2. **Ignore repeats in time** — record a timer tick in the ISR and reject any
   edge closer than 20 ms to the last accepted one.
3. **Disable, then re-enable** — clear the mask bit inside the ISR, and set it
   again from a timer some milliseconds later.

```c
ISR(INT0_vect)
{
    if ((uint8_t)(tick_ms - last_ms) < 20) return;   // still bouncing
    last_ms = tick_ms;
    int0_count++;
}
```

The lesson program counts raw edges deliberately, so you can *see* the bounce in
the count and appreciate why the previous lesson existed.

---

## Slide 7: The Lesson Program

Two demos, selected over the serial link.

```c
void demo1_falling_edge(void)
{
    EICRA = (1 << ISC01);        // INT0, falling edge
    EIMSK = (1 << INT0);
    sei();

    for (uint8_t i = 0; i < 100; i++)
    {
        char msg[40];
        sprintf(msg, "\rCount: %lu  ", int0_count);
        uart_puts(msg);
        _delay_ms(100);
    }
    EIMSK = 0;                   // stop listening before returning
}
```

`demo2_rising_edge` is the same shape with `EICRA = (1 << ISC11) | (1 << ISC10)`
and `EIMSK = (1 << INT1)`.

### Two details worth noticing
- **`EIMSK = 0` on the way out.** Leaving an interrupt enabled after its demo
  has finished means the next demo's counter is corrupted by the wrong pin.
- **`\r` and no `\n`.** The carriage return rewrites the same terminal line, so
  the count appears to tick in place instead of scrolling.

### Falling versus rising on this board
The buttons are active low: pressing pulls the pin down, releasing lets the
pull-up take it back up. So **demo 1 counts presses** and **demo 2 counts
releases**. Running both on the same button is the clearest way to see that the
edge really is what you selected.

---

## Slide 8: Running It

```
cd projects2026_avr\05_INT_External_Pins
build.bat
simulate.bat
```

In SimulIDE:
1. Press **Play**.
2. Open the **Serial Monitor** at 9600 baud.
3. Send `1` for the falling-edge demo, or `2` for rising edge.
4. Click the button on PD0 (demo 1) or PD1 (demo 2) and watch the count.
5. LEDs on PB7 and PB6 toggle on each accepted edge.

Each demo runs for about 10 seconds (100 iterations x 100 ms) then returns to
the menu.

### If the count never moves
- `sei()` missing, or `EIMSK` never set
- Wrong `EICRA` bits — check you set `ISCn1:ISCn0` for the interrupt you meant
- Pull-up not enabled, so the pin never made a clean edge

### If the count jumps several per click
That is contact bounce, and it is the expected result. Exercise 3 asks you to
fix it.

---

## Slide 9: Exercises

1. **Count both edges.** Set INT0 to "any edge" (`ISC00` only) and confirm the
   count rises on press *and* release.
2. **Use INT4.** Move the demo to PE4. Note what else on the board uses PORTE
   and what breaks — this is a real constraint, not a trick question.
3. **Debounce the interrupt.** Add the 20 ms rejection window from Slide 6,
   driven by a Timer0 millisecond tick. Compare counts before and after.
4. **Priority.** Enable INT0 and INT1 together, hold both buttons, and work out
   from the counts which handler ran first. Explain it from the vector table.
5. **Flag-and-defer.** Rewrite the ISR to set a flag only, doing the counting and
   printing in `main`. Measure how the response changes.
6. **Latency.** Toggle a spare pin at the top of the ISR and look at the delay
   from the button edge on the board's oscilloscope. How many clock cycles is it?

---

## Summary

### Key Points
✓ **Eight external interrupts** — INT0–INT3 on PORTD, INT4–INT7 on PORTE
✓ **EICRA/EICRB choose the trigger** — low level, any edge, falling, or rising
✓ **EIMSK enables, `sei()` permits** — you need both
✓ **EIFR clears by writing a one**, which is counter-intuitive but deliberate
✓ **Shared variables must be `volatile`**, or the compiler will optimise them away
✓ **ISRs must be short** — set a flag, do the work in `main`
✓ **Interrupts do not debounce** — a bouncing switch produces many interrupts

### Register Summary
| Register | Purpose |
|----------|---------|
| **EICRA** | Trigger condition for INT0–INT3 |
| **EICRB** | Trigger condition for INT4–INT7 |
| **EIMSK** | Per-interrupt enable |
| **EIFR** | Pending flags; write 1 to clear |
| **SREG** | Bit 7 is the global interrupt enable |

### Best Practices
1. **Clear the flag** before enabling, to swallow a spurious first interrupt
2. **Disable on the way out** of a routine that owned an interrupt
3. **Guard multi-byte shared variables** with `cli()`/restore, not bare `cli()`/`sei()`
4. **Avoid low-level triggers** unless the ISR removes the cause
5. **Use INT4–INT7** when an edge must wake the chip from deep sleep

### Next Steps
- **Timer interrupts** — periodic events instead of external ones
- **Input capture** — an interrupt that also records *when* it happened
- **Sleep modes** — using an interrupt as the only way back to life

---

## References and Resources

### Documentation
- ATmega128 Datasheet, Section 11 "External Interrupts" — EICRA, EICRB, EIMSK, EIFR
- ATmega128 Datasheet, Section 9 "Interrupts" — the vector table and priorities
- avr-libc manual, `<avr/interrupt.h>` — `ISR`, `sei`, `cli`, `ATOMIC_BLOCK`

### Related Lessons
- `02_Port_Button_Debounce` — why the raw edge count is not the press count
- `08_Timer1_Input_Capture` — interrupts that timestamp themselves
- `20_Power_Sleep_Modes` — which interrupts survive which sleep mode

### Further Reading
- AVR Application Note AVR130 — setting up and using the AVR timers and interrupts
- avr-libc FAQ, "Why does the compiler optimise my variable away?"
