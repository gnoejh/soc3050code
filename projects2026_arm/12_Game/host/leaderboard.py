#!/usr/bin/env python3
"""
leaderboard.py - the class high-score table, from $SCORE frames.   lesson 12

Every finished game prints one line on the serial monitor:

    $SCORE,SNAKE,42*1C

Copy your best lines out of the Wokwi serial monitor into a text file, one
file per player (the file name is the player's name), e.g.  scores/kim.txt.
Then:

    python leaderboard.py scores/*.txt          # Windows: scores\\*.txt works too
    python leaderboard.py --check '$SCORE,SNAKE,42*1C'

Only lines whose checksum is right are counted: the same XOR as lesson 08's
proto.c, so a typo made while copying is caught.  A determined cheat can of
course compute the XOR too - two hex digits detect accidents, not intent.
That needs a secret key (a MAC), and that is a security course.

Standard library only.
"""
import glob
import os
import re
import sys

FRAME = re.compile(r"\$([^*$]*)\*([0-9A-Fa-f]{2})")


def checksum(body):
    cs = 0
    for ch in body.encode("ascii", "replace"):
        cs ^= ch
    return cs


def frames(text):
    """Yield (body, ok) for every $...*CS frame in the text."""
    for m in FRAME.finditer(text):
        body, cs = m.group(1), int(m.group(2), 16)
        yield body, checksum(body) == cs


def main(argv):
    if len(argv) >= 2 and argv[0] == "--check":
        for line in argv[1:]:
            for body, ok in frames(line):
                print(("ok   " if ok else "BAD  ") + body + "   checksum %02X" % checksum(body))
        return 0

    paths = []
    for a in argv:
        paths.extend(glob.glob(a) or [a])
    if not paths:
        print(__doc__)
        return 1

    best = {}                     # game -> list of (score, player)
    rejected = 0
    for path in paths:
        player = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read()
        mine = {}
        for body, ok in frames(text):
            parts = body.split(",")
            if len(parts) != 3 or parts[0] != "SCORE":
                continue
            if not ok:
                rejected += 1
                print("rejected (checksum): %s in %s" % (body, path))
                continue
            game, score = parts[1], int(parts[2])
            mine[game] = max(mine.get(game, score), score)
        for game, score in mine.items():
            best.setdefault(game, []).append((score, player))

    for game in sorted(best):
        print("\n== %s ==" % game)
        for rank, (score, player) in enumerate(sorted(best[game], reverse=True)[:10], 1):
            print("  %2d. %-16s %6d" % (rank, player, score))
    print("\n%d frame(s) rejected" % rejected)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
