#!/usr/bin/env python3
"""Repair text that was corrupted by an earlier encoding round-trip.

Some lesson sources went through a UTF-8 <-> CP949 conversion at some point in
their history, which turned box-drawing characters, the degree sign and the
micro sign into Hangul syllables preceded by a literal '?'.  The damage is
lossy, so the original bytes cannot simply be decoded back; the sequences are
matched literally instead.

The replacements are plain ASCII rather than the original Unicode.  Nearly all
of these strings are banners written to the serial port and read in SimulIDE's
serial monitor, where ASCII is guaranteed to render correctly whatever the
terminal's encoding is.

Run from projects2026_avr.  Re-runnable: sequences already fixed no longer match.
"""
import glob
import os
import re
import sys

# Corrupted sequence -> replacement.  Longest first so that the three-character
# forms are consumed before their trailing pair could match on its own.
REPLACEMENTS = [
    ("?붴븧", "+"),    # was U+2554 box drawing double down and right
    ("?싢븧", "+"),    # was U+255A box drawing double up and right
    ("?먥븮", "+"),    # was U+2557 box drawing double down and left
    ("?먥븴", "+"),    # was U+255D box drawing double up and left
    ("?먥븧", "="),    # was U+2550 box drawing double horizontal
    ("?묺", "<->"),        # was U+2194 left right arrow
    ("째", " deg"),        # was U+00B0 degree sign
    ("쨉", "u"),           # was U+00B5 micro sign
]

# A second, worse class of damage: characters that were replaced by literal '?'
# marks, so nothing about the original survives except the surrounding words.
# These are restored from context.  The banner cases matter beyond looks - the
# corruption ate the backslash of \r, so those lines printed a stray "r" and no
# carriage return.
LOST = [
    # banner side rails:  "??  TITLE ... ??r\n"  ->  "|  TITLE ... |\r\n"
    (re.compile(r'"\?\?(.*?)\?\?r\\n"'), r'"|\1|\\r\\n"'),
    # status markers
    (re.compile(r'\?\?(Keypad test PASSED|Correct)'), r'[OK] \1'),
    (re.compile(r'\?\?(Wrong key)'), r'[X] \1'),
    (re.compile(r'\?\?(Wake-up)'), r'[*] \1'),
    (re.compile(r'\?\?(Sent:)'), r'TX \1'),
    (re.compile(r'\?\?(Received:)'), r'RX \1'),
    (re.compile(r'\?\?(START HERE)'), r'>>> \1'),
    # arrows between two operands, e.g. "r16 ??0xFF" or "MOSI ??MISO"
    (re.compile(r'\(MOSI \?\?MISO\)'), '(MOSI <-> MISO)'),
    (re.compile(r'(0)\?\?( deg, )(5)\?\?(00 deg, )(9)\?\?(80 deg)'),
     r'\1->0\2\3->1\4\5->1\6'),
    (re.compile(r' \?\?'), ' -> '),      # "180 deg ??0 deg"
    (re.compile(r'\?\?'), '-> '),        # "Servo A ??%d deg"
]

SAFE = set("─│┌┐└┘├┤┬┴┼"
           "═║╔╗╚╝╠╣╦╩╬"
           "→←↑↓✓✅°µ×±"
           "•—–…“”‘’")


def main():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    total = 0
    for path in sorted(glob.glob(os.path.join(base, "*", "*.c"))
                       + glob.glob(os.path.join(base, "*", "*.h"))):
        raw = open(path, "rb").read()
        bom = raw.startswith(b"\xef\xbb\xbf")
        text = raw.decode("utf-8-sig")
        n = 0
        for bad, good in REPLACEMENTS:
            c = text.count(bad)
            if c:
                text = text.replace(bad, good)
                n += c
        for pat, good in LOST:
            text, c = pat.subn(good, text)
            n += c
        if n:
            out = text.encode("utf-8")
            if bom:
                out = b"\xef\xbb\xbf" + out
            open(path, "wb").write(out)
            print("%-34s %d replacements" % (os.path.relpath(path, base), n))
            total += n
    print("\ntotal: %d" % total)

    # Report anything non-ASCII left that is not recognised, for manual review.
    leftovers = {}
    for path in sorted(glob.glob(os.path.join(base, "*", "*.c"))):
        for ch in open(path, encoding="utf-8-sig").read():
            if ord(ch) > 0x7F and ch not in SAFE:
                leftovers.setdefault(ch, []).append(os.path.basename(
                    os.path.dirname(path)))
    if leftovers:
        print("\nunrecognised non-ASCII still present:")
        for ch, where in sorted(leftovers.items()):
            print("  U+%04X  x%-4d %s" % (ord(ch), len(where),
                                          ", ".join(sorted(set(where))[:4])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
