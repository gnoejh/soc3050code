#!/usr/bin/env bash
# run.sh - lesson 17's host verification (Git Bash, Linux, macOS)
#   1. lqr.py designs K from ../params.h, checks it stabilises the linear
#      model, and checks control.c carries the same K (--check)
#   2. sitl.c runs the firmware's own world.c + control.c + view.c on the PC
# Exit status non-zero if either fails.
set -e
cd "$(dirname "$0")"
PY=python; command -v python >/dev/null 2>&1 || PY=python3
echo "=== lqr.py ==="
$PY lqr.py --check ../control.c
echo
echo "=== sitl ==="
gcc -O2 -std=c11 -Wall -Wextra -I.. -I../../_lib \
    sitl.c ../world.c ../control.c ../view.c ../../_lib/oled.c -lm -o sitl
./sitl "$@"
