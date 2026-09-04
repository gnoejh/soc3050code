# Internal EEPROM
## ATmega128 Embedded Systems Course

**Reference**: [ATmega128 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf) — Section 4, EEPROM Data Memory

---

## Slide 1: Three Kinds of Memory

The ATmega128 has three memory spaces, and they are good at different things.

| | Flash | SRAM | **EEPROM** |
|---|---|---|---|
| Size | 128 KB | 4 KB | **4 KB** |
| Holds your | program | variables | **settings** |
| Survives power off | yes | **no** | **yes** |
| Written at run time | rarely | constantly | **occasionally** |
| Write speed | slow, in pages | 1 cycle | **3.4 ms per byte** |
| Endurance | 10,000 erase cycles | unlimited | **100,000 writes per byte** |

EEPROM fills the gap between the other two: it remembers across a power cut like
Flash, but you can change one byte at a time like SRAM. That makes it the place
for calibration constants, user preferences, a serial number, an hours-run
meter — anything the device should still know tomorrow.

### What it is not
It is not general storage. At 3.4 ms per byte it is roughly **50,000 times
slower** than SRAM, and each cell wears out. Writing a variable to EEPROM inside
your main loop will destroy the chip in an afternoon — Slide 6 does that
arithmetic.

---

## Slide 2: The Hardware Registers

Four registers, and a strict order of operations.

| Register | Purpose |
|----------|---------|
| **EEARH:EEARL** | Address, 12 bits, so 0 to 4095 |
| **EEDR** | Data byte, in or out |
| **EECR** | Control: `EERE`, `EEWE`, `EEMWE`, `EERIE` |

### Reading is simple
```c
uint8_t eeprom_read(uint16_t addr)
{
    while (EECR & (1 << EEWE))     // a write in progress blocks reads
        ;
    EEAR = addr;
    EECR |= (1 << EERE);           // strobe read enable
    return EEDR;                   // data is ready on the very next cycle
}
```

### Writing has a timed sequence
```c
void eeprom_write(uint16_t addr, uint8_t data)
{
    while (EECR & (1 << EEWE))
        ;
    EEAR = addr;
    EEDR = data;

    uint8_t sreg = SREG;
    cli();                         // the next two lines must not be split
    EECR |= (1 << EEMWE);          // master write enable: opens a 4-cycle window
    EECR |= (1 << EEWE);           // and start the write
    SREG = sreg;
}
```

**`EEMWE` must be set within four clock cycles of `EEWE`.** That is the hardware
guarding against an accidental write from a runaway program — a single stray
store cannot corrupt your settings, it takes a deliberate two-step. An interrupt
landing between those two lines misses the window and the write silently does
nothing, which is why interrupts are disabled across them.

---

## Slide 3: Using avr-libc Instead

You will almost never write those sequences by hand. `<avr/eeprom.h>` provides
tested versions:

```c
#include <avr/eeprom.h>

/* single values */
uint8_t  b = eeprom_read_byte  ((uint8_t  *)0x10);
uint16_t w = eeprom_read_word  ((uint16_t *)0x12);
uint32_t d = eeprom_read_dword ((uint32_t *)0x14);

eeprom_write_byte  ((uint8_t  *)0x10, 0xAA);
eeprom_write_word  ((uint16_t *)0x12, 1234);
eeprom_write_dword ((uint32_t *)0x14, 100000UL);

/* blocks */
eeprom_read_block  (dest_in_ram, (const void *)0x20, length);
eeprom_write_block (src_in_ram,  (void *)0x20,       length);

/* write only if the value differs - saves an erase cycle */
eeprom_update_byte ((uint8_t *)0x10, 0xAA);
```

### That cast is not a real pointer
`(uint8_t *)0x10` is an *EEPROM* address, not an SRAM address. It is written as a
pointer only because that is the interface avr-libc chose. Dereferencing it
directly would read SRAM location 0x10 and give you nonsense.

### Prefer the `_update_` family
`eeprom_update_byte` reads first and skips the write when the byte already holds
that value. Settings usually do not change, so this can eliminate almost every
write in a save routine — and every write avoided is an erase cycle not spent.

---

## Slide 4: Writing Takes Time

A byte write takes **3.4 ms** and the EEPROM is unavailable throughout. The
library's write functions already poll `EEWE` on entry, so back-to-back calls are
safe. The lesson code adds explicit delays as well:

```c
eeprom_write_byte((uint8_t *)0x10, 0xAA);
_delay_ms(10);                      // belt and braces
```

Those delays are not strictly required — they are there so that the timing is
visible to you while learning. In production, let the library's `EEWE` poll do
the waiting, and spend the time on something useful.

### Timing to expect
| Operation | Time |
|-----------|------|
| Read a byte | under 1 µs |
| Write a byte | 3.4 ms |
| Write 16 bytes | about 55 ms |
| Write the whole 4 KB | about 14 seconds |

### The brown-out trap
A write interrupted by a falling supply can leave the byte half-programmed —
neither the old value nor the new one. This is the most common cause of
mysteriously corrupted settings in the field. **Enable brown-out detection** via
the `BODEN`/`BODLEVEL` fuses so the chip is held in reset before the supply drops
below the level where a write can go wrong.

---

## Slide 5: Laying Out the Address Space

4096 bytes, addresses 0x000 to 0xFFF. Nothing organises them for you, so decide
a map and write it down.

The map this lesson uses:

| Address | Size | Contents |
|---------|------|----------|
| 0x10 – 0x11 | 2 | byte-access demo |
| 0x20 – 0x3F | 32 | block-access demo, a string |
| 0x40 – 0x43 | 4 | boot counter, a 32-bit value |
| 0x50 – 0x5F | 16 | erase-test scratch area |

### Two conventions worth adopting
**Leave address 0 alone.** A pointer that gets corrupted to zero is common, and
address 0 is where it will write. Losing a byte you never used is better than
losing your calibration constant.

**Version your layout.** Put a magic number and a version at a fixed address:

```c
#define CONFIG_MAGIC   0xA5
#define CONFIG_VERSION 2

if (eeprom_read_byte((uint8_t *)0) != CONFIG_MAGIC ||
    eeprom_read_byte((uint8_t *)1) != CONFIG_VERSION)
{
    load_defaults();
    save_config();          /* first boot, or a firmware upgrade */
}
```

Without this, new firmware reads an old layout as though it were the new one and
starts with garbage.

---

## Slide 6: Endurance, and the Arithmetic That Matters

Each EEPROM cell is rated for **100,000 write cycles**. Reads are unlimited and
do not wear anything.

```
writes once per second   -> 100000 s        -> about 28 hours
writes once per minute   -> 100000 min      -> about 69 days
writes once per hour     -> 100000 hours    -> about 11 years
writes once per power-up -> effectively forever
```

A logger that saves a reading every second destroys that byte in **just over one
day**. This is the single most important number in the lesson.

### Wear levelling
When you genuinely must write often, spread the writes over many cells. Store a
record plus a sequence number, and rotate through a block:

```
slot 0: [seq=5][value]      <- oldest
slot 1: [seq=6][value]
slot 2: [seq=7][value]      <- newest, highest sequence number
slot 3: [seq=4][value]
```

On boot, scan for the highest sequence number to find the current value; write
to the next slot each time. Sixteen slots multiply the life of the data by
sixteen, at the cost of a scan at start-up.

---

## Slide 7: The Lesson Program

Four demos on a serial menu.

**1. Byte access** — write `0xAA` to 0x10 and `0x55` to 0x11, read them back,
print both. The simplest possible round trip.

**2. Block access** — write the string `"Hello EEPROM!"` to 0x20 with
`eeprom_write_block`, read it back into RAM, print it.

```c
char write_data[] = "Hello EEPROM!";
char read_data[32] = {0};

eeprom_write_block(write_data, (void *)0x20, sizeof(write_data));
eeprom_read_block(read_data,  (void *)0x20, sizeof(write_data));
```

`sizeof(write_data)` is 14, not 13 — it counts the terminating `'\0'`. That is
what makes the string safe to print after reading it back.

**3. Power-on counter** — the demo that shows why EEPROM exists:

```c
uint32_t counter = eeprom_read_dword((uint32_t *)0x40);
counter++;
eeprom_write_dword((uint32_t *)0x40, counter);
```

Reset the simulated board and the count carries on from where it was. Nothing
else in the system can do that.

**4. Erase test** — write `0xFF` to sixteen bytes and verify. "Erased" for
EEPROM means all bits set, which is also what a factory-fresh chip reads.

---

## Slide 8: Running It

```
cd projects2026_avr\19_EEPROM_ReadWrite
build.bat
simulate.bat
```

In SimulIDE:
1. Press **Play**.
2. Open the **Serial Monitor** at 9600 baud.
3. Send `1`, `2`, `3` or `4` to pick a demo.
4. The LED on PB7 toggles after each one.

### Making the counter demo convincing
Run demo 3, note the count, then **reset the MCU** (right-click the MCU, or stop
and start the simulation) and run it again. The count continues.

The board's MCU has `saveEepr="true"`, so SimulIDE keeps EEPROM contents in the
circuit file between runs, exactly as the real chip keeps them between power
cycles.

### If every read returns 0xFF
That is a blank EEPROM — the write did not happen. Check that the write call
comes before the read, and that nothing cleared the EEPROM between them.

---

## Slide 9: Exercises

1. **Write your own driver.** Implement `eeprom_read`/`eeprom_write` with the raw
   registers from Slide 2 and confirm they match the library's results.
2. **Break the four-cycle window.** Remove the `cli()` from the write sequence and
   run a busy Timer0 interrupt at the same time. Report how often writes fail.
3. **Time it.** Toggle a pin around a byte write and measure the pulse on the
   board's oscilloscope. How close is it to 3.4 ms?
4. **Versioned settings.** Add the magic-number-and-version check from Slide 5, and
   demonstrate that changing `CONFIG_VERSION` triggers a reload of defaults.
5. **`update` versus `write`.** Save the same settings block 100 times with each,
   and count the actual writes performed. Explain the difference.
6. **Wear levelling.** Implement the 16-slot rotating counter from Slide 6 and
   verify that the correct value survives a reset at any point in the cycle.
7. **Checksum.** Add a one-byte XOR checksum to the settings block and reject the
   block if it fails. Corrupt a byte deliberately to show the check working.

---

## Summary

### Key Points
✓ **EEPROM is non-volatile, byte-writable and slow** — 4 KB on the ATmega128
✓ **Writing takes 3.4 ms per byte** and blocks further EEPROM access
✓ **`EEMWE` must precede `EEWE` within four cycles**, with interrupts disabled
✓ **Use `<avr/eeprom.h>`** rather than hand-rolling the sequence
✓ **`eeprom_update_*` skips unchanged bytes** and saves erase cycles
✓ **100,000 writes per cell** — write once a second and it lasts a day
✓ **Enable brown-out detection**, or a dying supply will corrupt a write

### Register Summary
| Register | Purpose |
|----------|---------|
| **EEARH:EEARL** | 12-bit EEPROM address |
| **EEDR** | Data register |
| **EECR** `EERE` | Read enable |
| **EECR** `EEWE` | Write enable / busy flag |
| **EECR** `EEMWE` | Master write enable, self-clearing after four cycles |
| **EECR** `EERIE` | Interrupt when the EEPROM becomes ready |

### Best Practices
1. **Map the address space** and keep the map in a header
2. **Never use address 0** for anything you care about
3. **Version your layout** with a magic number, and reload defaults on mismatch
4. **Prefer `update` over `write`** everywhere
5. **Checksum anything whose corruption would be dangerous**
6. **Level the wear** if writes happen more often than once a minute

### Next Steps
- **External I2C or SPI EEPROM** — the same ideas at megabyte scale
- **Data logging** — ring buffers in non-volatile memory
- **Watchdog and brown-out** — protecting a write from a failing supply

---

## References and Resources

### Documentation
- ATmega128 Datasheet, Section 4 "EEPROM Data Memory" — registers and timing
- ATmega128 Datasheet, "Preventing EEPROM Corruption" — the brown-out discussion
- avr-libc manual, `<avr/eeprom.h>` — the full function list

### Related Lessons
- `21_Watchdog_Reset` — surviving a failure without losing stored state
- `20_Power_Sleep_Modes` — brown-out detection and supply behaviour
- `15_SPI_Master_Basic` — reaching a much larger external memory

### Further Reading
- AVR Application Note AVR101 — high-endurance EEPROM storage
- AVR Application Note AVR100 — accessing the EEPROM from C and assembly
