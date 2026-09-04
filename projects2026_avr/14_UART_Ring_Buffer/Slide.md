# Interrupt-Driven UART and the Ring Buffer
## ATmega128 Embedded Systems Course

**Reference**: [ATmega128 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf) — Section 19, USART

---

## Slide 1: The Problem With Polled Receive

The previous lesson received a byte like this:

```c
while (!(UCSR1A & (1 << RXC1)))
    ;                        // stand here until a byte arrives
char c = UDR1;
```

That is correct and it is useless for a real program, because while it stands
there nothing else happens. Worse, the USART holds only **two** received bytes:
one in the shift register and one in the receive buffer. Miss the window and the
third byte overwrites the second, and it is gone with no way to recover it.

### How long is the window?
At 9600 baud a character takes ten bit-times:

```
10 bits / 9600 bits per second = 1.04 ms per character
```

At 16 MHz that is **16,600 clock cycles** — enormous. But a `_delay_ms(10)`
somewhere in your main loop is ten characters long, and a slow GLCD redraw is
worse. The moment your program does anything substantial, polled receive starts
dropping data.

### The fix, in two parts
1. An **interrupt** takes the byte out of the hardware the instant it lands.
2. A **ring buffer** holds it until the main program is ready.

---

## Slide 2: What a Ring Buffer Is

An array plus two indices, where "the end" wraps round to "the beginning".

```
        write here                     read here
            |                              |
            v                              v
   +----+----+----+----+----+----+----+----+
   |    |    | E  | L  | L  | O  |    |    |
   +----+----+----+----+----+----+----+----+
     0    1    2    3    4    5    6    7
            ^                        ^
          head                      tail        (indices wrap at 8)
```

- **head** — where the *producer* (the ISR) will write next
- **tail** — where the *consumer* (main) will read next
- **head == tail** — the buffer is empty
- **head + 1 == tail** — the buffer is full

Nothing is ever moved. Adding and removing a byte are both constant-time, which
is what makes this safe to do inside an interrupt handler.

### Why one slot is sacrificed
With `head == tail` meaning empty, a completely full buffer would also have
`head == tail` and the two states would be indistinguishable. Declaring "full"
one slot early keeps them apart. A 128-byte array therefore holds 127 bytes.
Spending one byte to avoid an ambiguity is a good trade.

---

## Slide 3: The Receive Interrupt

```c
#define BUFFER_SIZE 128
volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

ISR(USART1_RX_vect)
{
    uint8_t data = UDR1;                          // always read UDR1
    uint8_t next = (rx_head + 1) % BUFFER_SIZE;

    if (next != rx_tail)                          // room for one more?
    {
        rx_buffer[rx_head] = data;
        rx_head = next;
    }
    /* else: buffer full, byte discarded */
}
```

### Read `UDR1` unconditionally
Reading `UDR1` is what clears `RXC1` and dismisses the interrupt. If you skip
the read when the buffer is full, the flag stays set, the ISR is re-entered
immediately, and the program locks up in an interrupt storm. Take the byte out
of the hardware first, *then* decide whether to keep it.

### Publish the data before the index
`rx_buffer[rx_head] = data;` comes before `rx_head = next;`. Main code decides
there is data by comparing the indices, so the byte must already be in place
when `head` moves. Doing it the other way round exposes a slot that has not been
written yet.

---

## Slide 4: Enabling It

```c
void usart_init_rb(void)
{
    uint16_t ubrr = (F_CPU / (16UL * BAUD)) - 1;
    UBRR1H = ubrr >> 8;
    UBRR1L = ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);   /* 8 data bits, no parity, 1 stop */
    sei();
}
```

| Bit | Register | Meaning |
|-----|----------|---------|
| **RXEN1** | UCSR1B | Enable the receiver |
| **TXEN1** | UCSR1B | Enable the transmitter |
| **RXCIE1** | UCSR1B | **Interrupt when a byte has been received** |
| **UDRIE1** | UCSR1B | Interrupt when the transmit register is free |
| **UCSZ11:10** | UCSR1C | 8-bit character size |

`RXCIE1` is the one line that turns polling into interrupt-driven receive.

### The baud rate figure
At 16 MHz with `U2X` off, `UBRR = 16000000 / (16 x 9600) - 1 = 103.17`, which
truncates to 103. That gives an actual 9615 baud — a **0.16 % error**, well
inside the roughly 2 % a UART frame tolerates. If you ever need exactness,
7.3728 MHz divides perfectly; 16 MHz is chosen here because it matches the MCU
frequency on the shared SimulIDE board.

---

## Slide 5: Reading From the Buffer

```c
uint8_t rb_available(void)
{
    return (rx_head - rx_tail + BUFFER_SIZE) % BUFFER_SIZE;
}

uint8_t rb_read(void)
{
    uint8_t data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    return data;
}
```

### The `+ BUFFER_SIZE` in the count
When `head` has wrapped and `tail` has not, `head - tail` is negative. Adding
`BUFFER_SIZE` before the modulo makes it positive without a branch.

### Why no `cli()` is needed here
There is exactly one producer (the ISR) and one consumer (main). The ISR only
ever writes `head`; main only ever writes `tail`. Each index has a single writer,
and on an 8-bit AVR a byte-sized load or store is a single instruction and
cannot be interrupted halfway. That makes this particular arrangement safe
without disabling interrupts.

**This guarantee is narrow.** Add a second consumer, make the indices 16-bit
(`BUFFER_SIZE` above 256), or have main modify `head`, and it evaporates — then
you need an `ATOMIC_BLOCK`.

### Always check before reading
`rb_read` does not check for an empty buffer. Calling it when `rb_available()`
is zero returns a stale byte and corrupts `tail`. The caller must ask first.

---

## Slide 6: Transmit Stays Polled — And Why That Is Fine Here

```c
void rb_write(char c)
{
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = c;
}
```

This blocks, but the blocking is bounded: at most one character time, about
1 ms, because the transmitter is never asked for more than it can take. Receive
is the side with a hard deadline — a byte you fail to collect is lost forever —
whereas a byte you are slow to *send* merely leaves later.

### When transmit needs a buffer too
As soon as your program sends long strings while something time-critical is
running, add a second ring buffer driven by `UDRIE1`:

```c
ISR(USART1_UDRE_vect)
{
    if (tx_head == tx_tail)
        UCSR1B &= ~(1 << UDRIE1);    // nothing left: stop asking
    else
        UDR1 = tx_buffer[tx_tail++ % BUFFER_SIZE];
}
```

Note the handler must **disable its own interrupt** when the buffer empties.
`UDRE1` is set whenever the register is free, which is almost always, so an
enabled and unfed UDRE interrupt is another interrupt storm.

---

## Slide 7: Sizing the Buffer

Pick the size from the worst-case burst and how long the main loop can look away.

```
bytes needed = baud / 10 x longest_blind_interval_seconds
```

At 9600 baud the line delivers 960 bytes per second, so:

| Main loop is blind for | Bytes that can arrive | Sensible size |
|---|---|---|
| 1 ms | 1 | 16 |
| 10 ms | 10 | 32 |
| 50 ms | 48 | 64 |
| 200 ms | 192 | 256 |

This lesson uses **128**, which covers a blind interval of about 130 ms — ample
for a program whose slowest step is a 10 ms delay.

### Make it a power of two
`% 128` on a power-of-two size compiles to a single `AND` with 127. On a
non-power-of-two it becomes a division routine — dozens of cycles, inside an
interrupt handler. Always size a ring buffer 16, 32, 64, 128 or 256.

### Deciding it is too small
Count the discards. Add `volatile uint16_t rx_dropped;` and increment it in the
`else` branch of the ISR. A non-zero value after a realistic test means the
buffer, the main loop, or the baud rate has to change.

---

## Slide 8: The Lesson Program

**Demo 1 — see the buffer work.** Type characters; each is echoed together with
the current occupancy. Press `q` to leave.

```c
if (rb_available() > 0)
{
    char c = rb_read();
    if (c == 'q') break;
    sprintf(msg, "Got: '%c' | Buffer: %u/%u\r\n", c, rb_available(), BUFFER_SIZE);
    rb_puts(msg);
}
_delay_ms(10);
```

That `_delay_ms(10)` is the point of the exercise. It makes the main loop blind
for ten character times — exactly the situation polled receive cannot survive —
and the ring buffer absorbs it without losing a byte.

**Demo 2 — burst handling.** Paste a block of text as fast as the terminal will
send it. Every tenth character the program reports its progress, and the count
reaching 100 proves nothing was dropped.

### Try the comparison
Comment out `RXCIE1` in `usart_init_rb`, replace `rb_available()` with a direct
`RXC1` test, and repeat demo 2. The dropped characters are the whole argument
for this lesson.

---

## Slide 9: Running It

```
cd projects2026_avr\14_UART_Ring_Buffer
build.bat
simulate.bat
```

In SimulIDE:
1. Press **Play**.
2. Open the **Serial Monitor** at 9600 baud, 8N1.
3. Send `1` for the buffered echo, or `2` for the burst test.
4. In demo 2, paste a long line rather than typing it.

### If nothing echoes
- `sei()` missing — `usart_init_rb` calls it, but check nothing later runs `cli()`
- `RXCIE1` not set, so `USART1_RX_vect` never fires
- The serial monitor is on the wrong port; the board's serial component is wired
  to **PD2 (RXD1) and PD3 (TXD1)**

### If characters are lost in demo 2
Confirm `BUFFER_SIZE` is 128 and that the ISR's full-buffer branch still reads
`UDR1`. Losing bytes *and* locking up points at the missing read.

---

## Slide 10: Exercises

1. **Count the drops.** Add `rx_dropped` and report it. Find a burst size that
   makes it non-zero with a 16-byte buffer.
2. **Shrink it.** Set `BUFFER_SIZE` to 8 and re-run demo 2. Explain the result in
   terms of the sizing formula.
3. **Not a power of two.** Set it to 100 and compare the generated code size for
   the ISR (`avr-objdump -d Main.elf`). Explain the difference.
4. **Buffered transmit.** Add a TX ring buffer driven by `UDRIE1`. Verify that
   `rb_puts` returns immediately even for a long string.
5. **Line assembly.** Buffer incoming bytes until `\n`, then hand the whole line
   to the main program. This is the basis of a command parser.
6. **Break it deliberately.** Remove `volatile` from `rx_head` and `rx_tail`,
   build with `-O2`, and describe what happens and why.
7. **Overrun detection.** Check `DOR1` in `UCSR1A` and report hardware overruns
   separately from buffer overflows. What is the difference?

---

## Summary

### Key Points
✓ **The USART holds only two bytes** — collect them or lose them
✓ **`RXCIE1` turns receive into an interrupt**, decoupling it from the main loop
✓ **A ring buffer is an array plus head and tail**, both wrapping
✓ **One slot is sacrificed** so that empty and full stay distinguishable
✓ **Always read `UDR1` in the ISR**, even when discarding, or the interrupt re-fires
✓ **Single producer, single consumer, 8-bit indices** is what makes it lock-free
✓ **Size it as a power of two**, from the worst-case blind interval

### Register Summary
| Register | Purpose |
|----------|---------|
| **UBRR1H/L** | Baud rate divisor |
| **UCSR1A** | Status: `RXC1`, `UDRE1`, `DOR1` (overrun), `FE1` (framing error) |
| **UCSR1B** | `RXEN1`, `TXEN1`, `RXCIE1`, `UDRIE1` |
| **UCSR1C** | Frame format: character size, parity, stop bits |
| **UDR1** | The data register; reading it clears `RXC1` |

### Best Practices
1. **Declare every shared index `volatile`**
2. **Keep the ISR to a read, a bounds test and two stores**
3. **Check `rb_available()` before every `rb_read()`**
4. **Count discards** rather than assuming the buffer is big enough
5. **Disable `UDRIE1` from inside its own handler** when the transmit buffer empties

### Next Steps
- **Command parsing** — turning a byte stream into structured commands
- **SPI and I2C** — the same buffering ideas on synchronous buses
- **RTOS** — where the ring buffer becomes a queue between tasks

---

## References and Resources

### Documentation
- ATmega128 Datasheet, Section 19 "USART" — registers, baud tables, error flags
- ATmega128 Datasheet, Table 19-1 — baud rate error at each `UBRR` value
- avr-libc manual, `<avr/interrupt.h>` and `<util/atomic.h>`

### Related Lessons
- `13_UART_Basic` — the polled version this lesson replaces
- `05_INT_External_Pins` — `volatile` and the discipline of short handlers
- `22_RTOS_Scheduler` — queues between tasks, built on the same idea

### Further Reading
- AVR Application Note AVR306 — using the USART on tinyAVR and megaAVR
- "Lock-free single-producer single-consumer queues" — the general form of this pattern
