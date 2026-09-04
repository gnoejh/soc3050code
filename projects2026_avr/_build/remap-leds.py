#!/usr/bin/env python3
"""One-shot: point the lessons' status LEDs at the LEDs the board actually has.

Several lessons were written against a board whose 8 status LEDs sit on PORT C
and light on a HIGH level.  The shared SimulIDE board - and the framework's own
shared_libs/_port.c, which documents "PORTB: LED array (8 LEDs, active LOW)" -
wires the LEDs to PORT B with the anode on the 5 V rail, so a pin must be driven
LOW to light its LED.  PORT C is not connected to anything on the board at all,
so those lessons showed nothing when simulated.

This rewrites the affected lessons to go through a small board-map in config.h:

    LED_DDR         the LED bank's direction register
    LED_WRITE(v)    show bit pattern v, 1 = lit  (inverts for the active-low wiring)
    LED_TOGGLE(m)   flip the bits in mask m      (polarity-independent)

15_SPI_Master_Basic is deliberately left alone: on the ATmega128 the SPI bus
(SS, SCK, MOSI, MISO) is PB0-PB3, which is the same port as the LED bank, so
that lesson cannot drive the LEDs without corrupting its own bus.

Run once, from projects2026_avr.  Safe to re-run - it detects work already done.
"""
import os
import re
import sys

LESSONS = [
    "03_Port_Keypad_Matrix",
    "10_PWM_Servo",
    "16_I2C_RTC_DS1307",
    "17_LCD_Character",
    "20_Power_Sleep_Modes",
    "21_Watchdog_Reset",
]

LED_BLOCK = """
/* ---------------------------------------------------------------------------
 * Board map: status LEDs
 *
 * The shared SimulIDE board wires 8 LEDs to PORT B, anode to the 5 V rail
 * through a resistor and cathode to the pin, so a pin must be driven LOW to
 * light its LED.  This matches shared_libs/_port.c, which documents PORT B as
 * the LED array.  Write patterns through LED_WRITE so the inversion lives in
 * one place: bit set means lit.
 * ------------------------------------------------------------------------ */
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_WRITE(v) (LED_PORT = (uint8_t)~(uint8_t)(v))
#define LED_TOGGLE(m) (LED_PORT ^= (uint8_t)(m))
"""

# PORTC = <expr>;  and  PORTC ^= <expr>;  where <expr> may span lines.
ASSIGN = re.compile(r"\bPORTC\s*=\s*(.*?);", re.S)
XOR = re.compile(r"\bPORTC\s*\^=\s*(.*?);", re.S)
DDR = re.compile(r"\bDDRC\b")


def convert(text):
    n = [0, 0, 0]

    def do_xor(m):
        n[1] += 1
        return "LED_TOGGLE(%s);" % m.group(1).strip()

    def do_assign(m):
        n[0] += 1
        return "LED_WRITE(%s);" % m.group(1).strip()

    text = XOR.sub(do_xor, text)
    text = ASSIGN.sub(do_assign, text)
    text, n[2] = DDR.subn("LED_DDR", text)
    return text, n


def add_led_block(path):
    s = open(path, encoding="utf-8", errors="surrogateescape", newline="").read()
    if "LED_WRITE" in s:
        return False
    crlf = "\r\n" in s
    body = s.replace("\r\n", "\n")
    marker = "#endif /* CONFIG_H_ */"
    assert marker in body, path
    body = body.replace(marker, LED_BLOCK + "\n" + marker)
    if crlf:
        body = body.replace("\n", "\r\n")
    open(path, "w", encoding="utf-8", errors="surrogateescape", newline="").write(body)
    return True


def main():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for name in LESSONS:
        d = os.path.join(base, name)
        mainc = os.path.join(d, "Main.c")
        s = open(mainc, encoding="utf-8", errors="surrogateescape", newline="").read()
        if "LED_WRITE" in s:
            print("%-26s already remapped" % name)
            continue
        crlf = "\r\n" in s
        body = s.replace("\r\n", "\n")
        body, n = convert(body)
        if crlf:
            body = body.replace("\n", "\r\n")
        open(mainc, "w", encoding="utf-8", errors="surrogateescape",
             newline="").write(body)
        add_led_block(os.path.join(d, "config.h"))
        print("%-26s %d writes, %d toggles, %d DDR" % (name, n[0], n[1], n[2]))
    print("\n15_SPI_Master_Basic left on PORT C by design "
          "(PB0-PB3 carry its SPI bus).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
