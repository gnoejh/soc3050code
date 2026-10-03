#!/usr/bin/env bash
# host/run.sh - build and run lesson 15's Software-In-The-Loop test with plain gcc.
#   bash host/run.sh                  all the tests; exit 1 on any failure
#   bash host/run.sh race 3 600       one race: track 3 (HAIRPINS), knob 600
#   bash host/run.sh race 2 700 20 0  ... with kp 20, kd 0 (then ki, dfilt, noise)
#   bash host/run.sh ascii 4          the OLED frame for track 4, as text
set -e
H="$(cd "$(dirname "$0")" && pwd)"
L="$H/.."
gcc -O2 -std=c11 -Wall -Wextra -I"$L" -I"$L/../_lib" -o "$H/sitl.exe" \
    "$H/sitl.c" "$L/world.c" "$L/track.c" "$L/control.c" "$L/view.c" "$L/../_lib/oled.c" -lm
"$H/sitl.exe" "$@"
