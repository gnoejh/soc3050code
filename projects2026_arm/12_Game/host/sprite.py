#!/usr/bin/env python3
"""
sprite.py - draw a sprite in ASCII, get the bytes oled_sprite() wants.  lesson 12

The SSD1306 - and so oled_sprite() - stores a picture as COLUMNS: one byte
per column per 8 rows, bit 0 at the top.  Nobody draws like that, so draw
rows of '#' and '.' in a text file and let this script turn it round:

    python sprite.py ship.txt

ship.txt:
    ..##....
    .####...
    ########
    .####...
    ..##....

prints the C array (width 8, height 5) ready to paste.  Blank lines separate
animation frames.  Standard library only.
"""
import sys


def convert(rows):
    w = max(len(r) for r in rows)
    rows = [r.ljust(w, ".") for r in rows]
    h = len(rows)
    out = []
    for page in range((h + 7) // 8):            # sprites taller than 8 rows
        for c in range(w):                      # are stored page by page
            b = 0
            for bit in range(8):
                r = page * 8 + bit
                if r < h and rows[r][c] not in ". ":
                    b |= 1 << bit
            out.append(b)
    return w, h, out


def main(path):
    with open(path) as f:
        text = f.read()
    frames = [[ln.rstrip() for ln in blk.splitlines() if ln.strip()]
              for blk in text.split("\n\n") if blk.strip()]
    for i, rows in enumerate(frames):
        w, h, data = convert(rows)
        print("/* frame %d: %d x %d */" % (i, w, h))
        for r in rows:
            print("/*   %s */" % r)
        print("{ " + ", ".join("0x%02X" % b for b in data) + " },")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    sys.exit(main(sys.argv[1]))
