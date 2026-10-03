#!/usr/bin/env python3
"""
leaderboard.py - the class leaderboard for lesson 15, from serial captures.

Each student copies the Wokwi serial monitor into a text file named after
themselves (alice.txt, bob.txt ...).  Every completed AUTO lap printed a frame

    $LAP,<track>,<ms>*CS

and this script checks each frame's checksum (lesson 08's NMEA-style XOR),
keeps each person's best lap per track, and ranks them.  A frame whose
checksum fails is counted and ignored - hand-edited lap times do not count.

    python host/leaderboard.py captures/*.txt
    python host/leaderboard.py --track 3 captures/*.txt

Standard library only.
"""
import argparse
import glob
import os
import re
import sys

TRACKS = {1: "OVAL", 2: "S-BENDS", 3: "HAIRPINS", 4: "FIGURE-8"}
FRAME = re.compile(r"\$([^*\r\n]*)\*([0-9A-Fa-f]{2})")


def checksum(body):
    cs = 0
    for ch in body.encode("ascii", "replace"):
        cs ^= ch
    return cs


def laps_in(path):
    good, bad = [], 0
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            for m in FRAME.finditer(line):
                body, cs = m.group(1), int(m.group(2), 16)
                if not body.startswith("LAP,"):
                    continue
                if checksum(body) != cs:
                    bad += 1
                    continue
                parts = body.split(",")
                try:
                    good.append((int(parts[1]), int(parts[2])))
                except (IndexError, ValueError):
                    bad += 1
    return good, bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("files", nargs="+", help="capture files; the file name is the player's name")
    ap.add_argument("--track", type=int, help="only this track (1-4)")
    a = ap.parse_args()

    files = []
    for pattern in a.files:
        files.extend(glob.glob(pattern) or [pattern])

    best = {}            # (track, name) -> (ms, laps)
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        try:
            laps, bad = laps_in(path)
        except OSError as e:
            print(f"{path}: {e}", file=sys.stderr)
            continue
        if bad:
            print(f"{name}: {bad} frame(s) with a bad checksum ignored", file=sys.stderr)
        for track, ms in laps:
            key = (track, name)
            old = best.get(key)
            best[key] = (min(ms, old[0]) if old else ms, (old[1] if old else 0) + 1)

    tracks = sorted({t for t, _ in best} if a.track is None else {a.track})
    if not tracks:
        print("no $LAP frames found")
        return 1
    for t in tracks:
        rows = sorted((v[0], n, v[1]) for (tt, n), v in best.items() if tt == t)
        print(f"\nTrack {t} {TRACKS.get(t, '?')}")
        print("  #  name                best lap   laps")
        for i, (ms, n, count) in enumerate(rows, 1):
            print(f"  {i:<2} {n:<18} {ms / 1000:8.2f} s  {count:5d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
