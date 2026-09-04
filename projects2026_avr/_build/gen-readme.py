#!/usr/bin/env python3
"""Generate a README.md for each lesson.

The hardware section is derived from the lesson's own source rather than
written by hand, so it cannot drift: the port and peripheral lists come from
what Main.c actually touches, and the library list comes from the LIBS line the
resolver put in build.bat.

Run from anywhere:  python _build/gen-readme.py
"""
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# What each port reaches on the shared SimulIDE board, traced from the circuit.
BOARD = {
    "A": "KS0108 graphic LCD data bus",
    "B": "8 LEDs (active low), stepper coils, SPI bus, logic analyser",
    "C": "not connected on this board",
    "D": "PD0/PD1/PD4-PD7 push buttons; PD2/PD3 are RXD1/TXD1",
    "E": "PE4-PE7 GLCD control and INT4-INT7; PE3 joystick",
    "F": "ADC inputs - PF0 potentiometer, PF1/PF2 joystick axes",
    "G": "PG1 GLCD control, PG4 slide switch",
}

PERIPHERALS = [
    (r"\bUCSR1|UBRR1|UDR1", "USART1 (serial, 9600 8N1)"),
    (r"\bUCSR0|UBRR0|UDR0", "USART0"),
    (r"\bADMUX|ADCSRA", "ADC"),
    (r"\bTWCR|TWBR|TWDR|TWSR", "TWI / I2C"),
    (r"\bSPCR|SPDR|SPSR", "SPI"),
    (r"\bTCCR0|TCNT0", "Timer0"),
    (r"\bTCCR1|TCNT1|ICR1|OCR1", "Timer1"),
    (r"\bTCCR2|TCNT2", "Timer2"),
    (r"\bTCCR3|TCNT3", "Timer3"),
    (r"\bEIMSK|EICRA|EICRB", "External interrupts"),
    (r"\bWDTCR|wdt_enable|wdt_reset", "Watchdog timer"),
    (r"\bEECR\b|eeprom_read|eeprom_write", "Internal EEPROM"),
    (r"\bsleep_mode|set_sleep_mode|sleep_cpu", "Sleep modes"),
    (r"ks0108_|glcd_", "KS0108 graphic LCD"),
]

RTC_WIRING = """- **The DS1307 on the board is not wired yet — do this first.**
  The component is placed but has no connections, so the RTC demos will report
  no response until you draw two wires in SimulIDE. It is a five-minute job and
  a reasonable lab exercise in its own right:

  1. Run `simulate.bat` and find the **DS1307** near the top left of the board.
  2. The ATmega128's TWI pins are **PD0 = SCL** and **PD1 = SDA**. Both already
     carry a push button and a pull-up resistor to 5 V — that pull-up is exactly
     what an I2C bus needs, so nothing further is required there.
  3. Drag from the DS1307's **SCL** pin to the wire already running from
     **PD0**, and from its **SDA** pin to the wire from **PD1**. SimulIDE
     inserts a junction node where you drop the wire onto an existing one.
  4. **File > Save Circuit** to keep the change.

  This is deliberately left as a manual step: the two junction points on PD0 and
  PD1 are already at their three-connection limit, so adding the RTC means
  splicing new nodes into existing wires. That is safe to do by hand in the GUI
  and risky to do by editing the circuit file, which every lesson shares."""

TEMPLATE = """# {number}. {title}

{focus_sentence}

Part of the SOC3050 ATmega128 course, 2026 AVR edition. Runs on the shared
SimulIDE 1.1.0-SR2 board at 16 MHz.

## Files

| File | What it is |
|------|------------|
| `Main.c` | The lesson source |
| `config.h` | `F_CPU`, `BAUD` and this lesson's board map |
| `Slide.md` | The lecture deck for this topic |
| `build.bat` | Builds `Main.hex` |
| `simulate.bat` | Builds if needed, then opens the shared board in SimulIDE |
{extra_files}
## Build and run

```
build.bat        produces Main.elf and Main.hex
simulate.bat     opens the shared board with this lesson's firmware loaded
```

In SimulIDE press **Play** to start the simulation.{serial_note}

## What this lesson touches

**Ports**

{ports}

**Peripherals**

{peripherals}

**Shared libraries linked**

{libs}

## Notes
{notes}
## See also

- `../README.md` for the board map and the full lesson list
- `Slide.md` for the theory, register tables and exercises
"""

SERIAL_NOTE = ("\n\nThis lesson prints to the serial port, so open the "
               "**Serial Monitor** at 9600 baud, 8N1. The board's serial "
               "component is wired to PD2 (RXD1) and PD3 (TXD1).")


def lesson_meta():
    """Read the lesson list back out of migrate.py so there is one source."""
    sys.path.insert(0, os.path.join(BASE, "_build"))
    import migrate
    return {folder: (title, focus) for folder, _, title, focus in migrate.LESSONS}


def bullets(items, empty):
    if not items:
        return "- %s" % empty
    return "\n".join("- %s" % i for i in items)


def main():
    meta = lesson_meta()
    made = 0
    for folder in sorted(os.listdir(BASE)):
        d = os.path.join(BASE, folder)
        if not (os.path.isdir(d) and re.match(r"\d\d_", folder)):
            continue
        title, focus = meta.get(folder, (folder, ""))
        src = open(os.path.join(d, "Main.c"), encoding="utf-8",
                   errors="replace").read()

        ports = []
        for p in sorted(set(re.findall(r"\b(?:DDR|PORT|PIN)([A-G])\b", src))):
            ports.append("**PORT%s** - %s" % (p, BOARD[p]))
        # the LED board-map macro hides PORTB behind a name
        if "LED_WRITE" in src and not any("PORTB" in x for x in ports):
            ports.append("**PORTB** - %s, via `LED_WRITE` in config.h"
                         % BOARD["B"])

        periph = [name for pat, name in PERIPHERALS if re.search(pat, src)]

        build = open(os.path.join(d, "build.bat"), encoding="utf-8").read()
        m = re.search(r"^set LIBS=(.*)$", build, re.M)
        libs = (m.group(1).strip().split() if m else [])

        extra = ""
        for fn in sorted(os.listdir(d)):
            if fn.endswith(".md") and fn != "Slide.md" and fn != "README.md":
                extra += "| `%s` | Supporting handout |\n" % fn

        notes = []
        if "LED_WRITE" in src:
            notes.append(
                "- The original lesson drove status LEDs on PORT C, which is "
                "not connected on this board. They now go through `LED_WRITE` "
                "in `config.h`, which targets the board's PORT B bank and "
                "handles its active-low wiring.")
        if folder == "15_SPI_Master_Basic":
            notes.append(
                "- This lesson keeps its status LEDs on PORT C, which the "
                "board does not connect. PB0-PB3 carry its SPI bus, so the "
                "LED bank cannot be driven without corrupting the transfer. "
                "Watch the serial output instead.")
        if folder == "16_I2C_RTC_DS1307":
            notes.append(RTC_WIRING)
        if not notes:
            notes.append("- Nothing lesson-specific; the shared board covers "
                         "everything this lesson needs.")

        number = folder.split("_")[0].lstrip("0")
        focus_sentence = focus[0].upper() + focus[1:] + "." if focus else ""

        text = TEMPLATE.format(
            number=number,
            title=title,
            focus_sentence=focus_sentence,
            extra_files=extra,
            serial_note=SERIAL_NOTE if any("USART" in p for p in periph) else "",
            ports=bullets(ports, "No direct port access; the lesson works "
                                 "through the shared libraries."),
            peripherals=bullets(periph, "None beyond the core CPU."),
            libs=bullets(["`%s.c`" % l for l in libs],
                         "None - this lesson is self-contained."),
            notes="\n".join(notes) + "\n",
        )
        with open(os.path.join(d, "README.md"), "w", encoding="utf-8",
                  newline="\n") as fh:
            fh.write(text)
        made += 1
    print("wrote %d lesson READMEs" % made)
    return 0


if __name__ == "__main__":
    sys.exit(main())
