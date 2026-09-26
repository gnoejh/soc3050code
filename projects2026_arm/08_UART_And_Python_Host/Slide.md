# UART and a Python Host: Talking to the Board, Properly
## SOC3050 ARM Edition — Part 1, Instances

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](https://www.st.com/resource/en/user_manual/um2953-stm32c0-nucleo64-board-mb1717-stmicroelectronics.pdf)

**The serial port you have printed to since lesson 04 — now in both
directions, driven by interrupts, and spoken to by a program.**

Until now `printf` stopped the CPU for every character, and nothing ever went
*into* the board. This week the UART gets an interrupt each way, a protocol a
program can trust, and a partner on the PC: `host.py`, the first Python in the
course. From here on, the board is half of a system.

Figures marked **measured** come from running this lesson's firmware in
Renode (the harness of lesson 07, slide 20) and feeding its output to
`host.py`. Not yet watched in Wokwi.

---

## Slide 1: The Model, Fourth Instance

| Model slot | USART2 |
|---|---|
| **clock gate** | `RCC->APBENR1` `USART2EN` |
| **pins** | PA2 (TX), PA3 (RX), alternate function 1 |
| **control** | `CR1`: `UE`, `TE`, `RE`, and the two interrupt enables `RXNEIE`, `TXEIE` |
| **status** | `ISR`: `RXNE` (a byte arrived), `TXE` (room to send), `ORE` (overrun) |
| **data** | `RDR` in, `TDR` out |
| **event** | `USART2_IRQn` = 28, one vector for everything |
| **rate** | `BRR` = 48 000 000 / 115 200 = 416.67 → **417** |

`BRR = 417` gives 115 108 baud, 0.08% slow — far inside the few percent a
UART tolerates, because the receiver resynchronises on every start bit. The
banner prints `BRR = 417` read back from the register (**measured**).

Every row is a slot you already know. That is the whole point of Part 0's
model: the fourth peripheral is not a fourth new thing.

---

## Slide 2: One Byte on the Wire

```svg
<svg viewBox="0 0 580 150" role="img" aria-label="UART frame: idle high, start bit low, eight data bits LSB first, stop bit high">
  <path class="wire" d="M20 40 H70 V100 H120 V40 H170 V100 H220 V40 H270 V100 H320 V40 H370 V40 H420 V100 H470 V100 H520 V40 H570"/>
  <text x="45" y="30" text-anchor="middle" class="lbl">idle</text>
  <text x="95" y="125" text-anchor="middle" class="hi">start</text>
  <text x="145" y="125" text-anchor="middle" class="mono lbl">b0</text>
  <text x="195" y="125" text-anchor="middle" class="mono lbl">b1</text>
  <text x="245" y="125" text-anchor="middle" class="mono lbl">b2</text>
  <text x="295" y="125" text-anchor="middle" class="mono lbl">b3</text>
  <text x="345" y="125" text-anchor="middle" class="mono lbl">b4</text>
  <text x="395" y="125" text-anchor="middle" class="mono lbl">b5</text>
  <text x="445" y="125" text-anchor="middle" class="mono lbl">b6</text>
  <text x="495" y="125" text-anchor="middle" class="mono lbl">b7</text>
  <text x="545" y="125" text-anchor="middle" class="hi">stop</text>
  <text x="290" y="146" text-anchor="middle" class="lbl">10 bit-times per byte; least significant bit first</text>
</svg>
```

Ten bits per byte at 115 200 bits per second is **11 520 bytes a second —
86.8 µs per byte**. That one number drives both halves of this lesson:

- **Sending:** a 90-character line takes ~8 ms. A polled `printf` holds the
  CPU for all of it.
- **Receiving:** the UART holds **one** received byte. The next arrives 86.8 µs
  later and overwrites it unless software has read it — the **overrun**,
  `ORE`.

---

## Slide 3: Why Receive Must Be an Interrupt

Polling would work only if every path through the program came back to the
UART within 86.8 µs. In lesson 07's system one `printf` holds a task for 8 ms,
and the hog never comes back at all.

So receive becomes a producer and a consumer:

```svg
<svg viewBox="0 0 580 150" role="img" aria-label="RX path: UART RXNE interrupt puts each byte in a kernel queue; the protocol task blocks on the queue">
  <defs><marker id="u3" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="10" y="45" width="90" height="44" rx="5"/>
  <text x="55" y="65" text-anchor="middle" class="mono">RDR</text>
  <text x="55" y="81" text-anchor="middle" class="lbl">1 byte</text>
  <path class="wire" d="M100 67 H130" marker-end="url(#u3)"/>
  <rect class="hifill" x="134" y="40" width="150" height="54" rx="5"/>
  <text x="209" y="60" text-anchor="middle" class="mono">USART2_IRQHandler</text>
  <text x="209" y="78" text-anchor="middle" class="lbl">priority 1, every 86.8 us</text>
  <path class="wire" d="M284 67 H314" marker-end="url(#u3)"/>
  <rect class="reg" x="318" y="45" width="110" height="44" rx="5"/>
  <text x="373" y="65" text-anchor="middle" class="mono">os_queue</text>
  <text x="373" y="81" text-anchor="middle" class="lbl">64 bytes</text>
  <path class="wire" d="M428 67 H458" marker-end="url(#u3)"/>
  <rect class="box" x="462" y="45" width="110" height="44" rx="5"/>
  <text x="517" y="65" text-anchor="middle" class="mono">task_proto</text>
  <text x="517" y="81" text-anchor="middle" class="lbl">blocked: 0% CPU</text>
  <text x="290" y="130" text-anchor="middle" class="lbl">The deadline moves from 87 us to "before 64 bytes pile up" — 5.5 ms.</text>
</svg>
```

The ISR meets the hard 86.8 µs deadline with a few dozen instructions. The
queue turns it into a soft one: the task only has to catch up before 64 bytes
accumulate. And while nothing arrives, the protocol task costs **nothing** —
it is blocked in `os_queue_get()` (lesson 07, slide 15).

**Measured:** 77 bytes typed back to back — `rx overruns 0, rx dropped 0`.

---

## Slide 4: One Interrupt, Three Jobs

```c
void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;
    if (isr & USART_ISR_ORE) {                /* overrun             */
        USART2->ICR = USART_ICR_ORECF;        /* clear: write 1, ICR */
        stats.rx_overruns++;
    }
    if (isr & USART_ISR_RXNE_RXFNE) {         /* a byte arrived      */
        uint8_t c = (uint8_t)USART2->RDR;     /* reading clears RXNE */
        if (!os_queue_put_from_isr(&rx_queue, &c)) stats.rx_dropped++;
    }
    if ((USART2->CR1 & TXEIE) && (isr & USART_ISR_TXE_TXFNF)) { ... }
}
```

Count the ways a flag is cleared in this course so far:

| Flag | How | Lesson |
|---|---|---|
| EXTI `FPR1` | write 1 to it | 05 |
| TIM `UIF` | write **0** to it | 06 |
| TIM `CC1IF` | read `CCR1` | 06 |
| USART `RXNE` / `TXE` | read `RDR` / write `TDR` | 08 |
| USART `ORE` | write 1 to a **different** register, `ICR` | 08 |

Five conventions on one chip. There is no rule to memorise — only the habit:
**for every flag, look up how it clears.** The penalty for guessing is an
interrupt that never stops.

---

## Slide 5: Transmit — a Lock-Free Ring

Sending goes the other way: tasks produce, the interrupt consumes.

```c
static uint8_t           tx_ring[256];
static volatile uint16_t tx_head;   /* written ONLY by tasks (producer) */
static volatile uint16_t tx_tail;   /* written ONLY by the ISR (consumer) */

static void tx_put(uint8_t c)
{
    uint16_t next = (tx_head + 1) % 256;
    while (next == tx_tail) { os_delay(1); }    /* full: let the ISR drain */
    tx_ring[tx_head] = c;                        /* 1. the data            */
    tx_head = next;                              /* 2. THEN publish it     */
}
```

No mutex, no masked interrupts — because **each index has exactly one
writer**. That is lesson 03 slide 7's safe case, applied twice: tasks read
`tail` and write `head`; the ISR reads `head` and writes `tail`. Neither can
catch the other half-way.

Two details carry the correctness:

- **Data before index.** If `head` moved first, the ISR could send a slot
  that has not been filled yet.
- **One slot always empty.** `head == tail` means *empty*; if the ring could
  fill completely, *full* would look identical.

The RX side uses a kernel queue *with* locking, and the TX side a ring
*without* — on purpose, so you have seen both.

---

## Slide 6: `TXEIE` — On Only While There Is Work

`TXE` means "the transmit register is empty". When the UART is idle it is
empty **all the time**. So a permanently enabled `TXEIE` would interrupt
continuously:

```c
if (tx_tail != tx_head) {
    USART2->TDR = tx_ring[tx_tail];           /* send one, clears TXE   */
    tx_tail = (tx_tail + 1) % 256;
} else {
    USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;  /* nothing left: stop     */
}
```

and `_write()` switches it back on after filling the ring.

**Measured**, with the `else` line removed: **nothing is printed at all — not
even the banner.** The first `_write()` enabled `TXEIE`; the ISR sent the
first character, found the ring empty, and was called again, and again, at
priority 1, above everything that could ever put more data in. Lesson 05's
endless interrupt, in a new costume.

`_write()` enables `TXEIE` with a read-modify-write of `CR1` — which the ISR
also writes. So that one line is a **critical section**, two instructions long.

---

## Slide 7: Replacing `printf`'s Back End — Weak Again

`printf` has called `_write()` since lesson 04, and `_lib/retarget.c`'s
version polls. This lesson's `uart.c` defines its own. How do both link?

```c
/* _lib/retarget.c */
__attribute__((weak)) int _write(int fd, const char *buf, int len) { ...polled... }

/* 08_UART_And_Python_Host/uart.c */
int _write(int fd, const char *buf, int len) { ...ring buffer... }
```

**The same mechanism as the vector table** (lesson 05, slide 15): a strong
definition beats a weak one at link time. The lesson keeps `retarget.c`'s
syscall stubs and replaces exactly one function. `printf` does not know
anything changed — it now returns as soon as the text is in the ring, not
when the last bit has left the pin.

---

## Slide 8: Two Tasks, One Serial Port

The telemetry task and the protocol task both print. Without coordination,
two lines can interleave character by character — and a line that is half
one frame and half another fails its checksum on the PC.

Lesson 07's mutex makes each line atomic:

```c
static void say(const char *fmt, ...)
{
    va_list ap;  va_start(ap, fmt);
    os_mutex_lock(&print_lock);
    vprintf(fmt, ap);
    os_mutex_unlock(&print_lock);
    va_end(ap);
}
```

There is a second reason: newlib-nano's `printf` keeps state in one shared
structure and is **not re-entrant**. Two tasks inside it at once can corrupt
each other's output even when the timing looks harmless.

The lesson makes no measured claim here: in Renode the UART sends
instantly, so the window never opened. Lab Part 5 asks you to reason out when
it would.

---

## Slide 9: Lines — for People and for Programs

The protocol task collects bytes until `\r` or `\n`, then looks at the first
character:

| Line starts with | It is | Example | Reply |
|---|---|---|---|
| anything else | a **shell command** from a person | `led 170` | `bar = 0xAA` |
| `$` | a **frame** from a program | `$LED,85*6C` | `$ACK,LED,85*09` |

One UART, two audiences, and the difference is deliberate. A person wants
words and forgiveness. A program wants a format it can parse and **proof the
line arrived intact** — because a serial line picks up noise, drops bytes when
a buffer overflows, and gets cut in half when a cable is pulled.

A line longer than 63 characters is thrown away whole, rather than acted on in
part.

---

## Slide 10: The Frame — Borrowed From GPS

```
$LED,170*57
 \_____/ \/
  body   checksum: XOR of every byte of the body, two hex digits
```

This is **NMEA 0183**, the sentence format GPS receivers have printed since
the 1980s. Worked by hand:

```
L  E  D  ,  1  7  0
4C 45 44 2C 31 37 30    XOR all seven  ->  0x57
```

`host.py --selftest` checks its own arithmetic against a real GPS sentence
(`$GPGGA,...*47`) — so the checksum is *the* NMEA checksum, not merely one
that agrees with itself.

What XOR catches, and what it does not:

- **Any single flipped bit** — `$LED,171*57` (`0` → `1`, one bit) is rejected:
  the checksum comes out `56`.
- **Not** two errors in the same bit position of different bytes — they
  cancel. A CRC catches those; MAVLink, the drone protocol of lessons 13–14,
  uses one.

---

## Slide 11: Measured — a Session

Typed into the running firmware (Renode), one line at a time, and what came
back:

```
help            -> commands: led <0-255>   rate <20-5000 ms>   quiet   stats   help
led 170         -> bar = 0xAA
$LED,85*6C      -> $ACK,LED,85*09
$LED,85*00      -> $NAK,CHECKSUM*65          (damaged on purpose)
$LED,85         -> $NAK,NOSTAR*7D            (no checksum at all)
bogus           -> unknown command 'bogus' - try help
$RATE,250*19    -> $ACK,RATE,250*7C
$STAT*12        -> $STAT,71,3,2,0,0,0*25     (71 bytes in, 3 frames ok, 2 bad)
```

and, interleaved with all of it, telemetry — first once a second, then every
250 ms after the `RATE` frame:

```
$TEL,1000,0,500,0,0*75
$TEL,2000,1,1000,0,0*43
$TEL,2250,2,875,0,0*7C
```

---

## Slide 12: `host.py` — the Other End

A 230-line Python program with no dependencies beyond pyserial:

| Command | Does |
|---|---|
| `host.py --selftest` | checks the checksum against the GPS sentence |
| `host.py --frame "LED,170"` | prints `$LED,170*57` — to paste into Wokwi |
| `host.py --file capture.txt` | checks and charts a saved log |
| `host.py --port rfc2217://localhost:4000` | live, through the Wokwi VS Code extension |
| `host.py --port COM5 --send "RATE,100"` | live, on a real Nucleo |
| `host.py --demo` | no board at all: fake telemetry, with a damaged and a lost line |

**Why three routes?** Wokwi in a browser keeps its serial port inside the
browser; nothing on your PC can open it. So the browser route is **copy the
serial monitor's text into a file**. The VS Code extension can forward the port
as a TCP server (`rfc2217ServerPort = 4000` in this lesson's `wokwi.toml`), and
pyserial speaks that protocol natively — the live route. A real board is just
`COM5`.

The protocol being printable is what makes the copy-and-paste route possible
at all. A binary protocol could not survive a text box.

---

## Slide 13: Measured — `host.py` Reads the Board

The session on slide 11, saved to a file and given to `host.py --file`:

```
t=   1.00s seq=0     tri= 500  A=0 B=0  |####################
FRAME ACK,LED,85
FRAME NAK,CHECKSUM
FRAME NAK,NOSTAR
FRAME ACK,RATE,250
t=   2.00s seq=1     tri=1000  A=0 B=0  |########################################
t=   2.25s seq=2     tri= 875  A=0 B=0  |###################################
FRAME STAT,71,3,2,0,0,0
t=   2.50s seq=3     tri= 750  A=0 B=0  |##############################

frames good 9, bad 0, lost (sequence gaps) 0, other lines 11
```

**Every frame the C code built passed the Python check**, and every frame the
Python code built (`LED`, `RATE`, `STAT`) was accepted by the C check. Two
implementations, two languages, one arithmetic — each testing the other.

`--demo` shows the failures: a line altered in transit is counted **bad**, and
the sequence number it carried never arrives intact, so it is **also** a gap.

---

## Slide 14: The Silent-Failure Checklist, UART Edition

| Forgot | Symptom |
|---|---|
| `USART2EN` in `APBENR1` | nothing sent, nothing received |
| AF1 on PA2/PA3 | UART works internally; the pins carry nothing |
| `BRR` for the wrong clock | garbage characters — the most recognisable UART fault |
| clearing `ORE` | reception stops after the first overrun |
| turning `TXEIE` off when empty | interrupt storm; **measured: not even the banner** |
| blocking in the ISR | impossible to do right: the ISR version of `put` never blocks |
| a lock around printing | lines interleave; checksums fail on the PC |

The last two are design, not configuration: which context may wait, and which
data is shared. Lessons 03 and 07 are why you can reason about them.

---

## Slide 15: What Carries Forward

- **Every later peripheral that moves a stream of data looks like this:** an
  interrupt that feeds or drains a buffer, and a task that blocks. Lesson 10
  replaces the per-byte interrupt with **DMA** — one interrupt per block.
- **The board now has a partner.** Lesson 09 sends real sensor readings in
  `$TEL` frames and `host.py` plots them. Part 3 swaps this protocol for
  **MAVLink**, and `host.py` for mission software that flies a drone.
- **Checked data, not trusted data.** A frame that fails its checksum is
  thrown away and counted — never half-used. That rule scales from a text line
  to a flight controller.
