#!/usr/bin/env bash
# run.sh - lesson 10 host tests.  bash host/run.sh [path/to/Main.elf]
#
#   1. test_fix   fix.c + bench.c compiled with the PC's gcc, checked vs double
#   2. m0sim.py   the arena's workloads executed from Main.elf on a Cortex-M0+
#                 instruction model, cycles counted - needs Main.elf (build.bat)
#   3. flashcost  what each number format costs, from arm-none-eabi-nm
# Exit status non-zero if any step fails.
set -e
set -o pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
L="$HERE/.."
ELF=${1:-$L/Main.elf}
OUT="${TMPDIR:-/tmp}"

gcc -O2 -std=c11 -Wall -Wextra -ffp-contract=off -I"$L" -I"$L/../_lib" \
    "$L/fix.c" "$L/bench.c" "$HERE/test_fix.c" -lm -o "$OUT/test_fix.exe"
"$OUT/test_fix.exe" | tee "$OUT/test_fix.txt"

if [ -f "$ELF" ]; then
    python "$HERE/m0sim.py" "$ELF" --expect "$OUT/test_fix.txt"
    python "$HERE/flashcost.py" "$ELF"
else
    echo
    echo "No $ELF yet - run build.bat first for the cycle model and the flash costs."
fi
