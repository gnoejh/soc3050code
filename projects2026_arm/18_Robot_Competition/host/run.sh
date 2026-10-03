#!/usr/bin/env bash
# run.sh [strategy.c ...] - build and run the sumo league on the PC (Git Bash / Linux)
#
#   ./run.sh                                     built-ins + student.c
#   ./run.sh strategies/alice.c strategies/bob.c   ... plus these
#   ./run.sh -- -n 20 --tour alice               pass options to league
#
# Each extra file is compiled ON ITS OWN with -DSTRATEGY_PREFIX=<file name>,
# so its strategy_step becomes <name>_step, and register.h is force-included
# to hand it to the league.  Then its object file is checked for globals:
# anything in .data or .bss means state outside the 64-byte memory, and the
# file is rejected.  Same steps as run.bat.
set -u
H=$(cd "$(dirname "$0")" && pwd)
L=$(cd "$H/.." && pwd)
LIB=$(cd "$L/../_lib" && pwd)
OUT=$H/build
mkdir -p "$OUT"
CFLAGS="-std=c11 -Wall -Wextra -ffp-contract=off -I$L -I$LIB"

files=(); opts=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--" ]; then shift; opts=("$@"); break; fi
  files+=("$1"); shift
done

objs=()
for f in "${files[@]}"; do
  name=$(basename "$f" .c)
  case "$name" in [A-Za-z_]*) ;; *) echo "REJECT $f: the file name must be a C identifier"; exit 1;; esac
  gcc $CFLAGS -O2 -c -DSTRATEGY_PREFIX="$name" -include "$H/register.h" "$f" -o "$OUT/$name.o" || { echo "REJECT $f: does not compile"; exit 1; }
  if nm "$OUT/$name.o" | grep -E ' [BbDdCc] [^.]' ; then
    echo "REJECT $f: the symbols above are variables outside strategy_mem_t"; exit 1
  fi
  objs+=("$OUT/$name.o")
done

SRC="$H/league.c $L/world.c $L/referee.c $L/bots.c $L/student.c $L/scene.c $LIB/oled.c"
gcc $CFLAGS -O2 $SRC "${objs[@]}" -o "$OUT/league.exe" || exit 1
# The same program at -O0 and -Os: if optimisation changed a single result, the
# physics depends on the compiler, and the chip's -Os build could disagree.
gcc $CFLAGS -O0 $SRC "${objs[@]}" -o "$OUT/league_O0.exe" || exit 1
gcc $CFLAGS -Os $SRC "${objs[@]}" -o "$OUT/league_Os.exe" || exit 1

"$OUT/league.exe" "${opts[@]}" || exit 1

echo
echo "6. Compiler independence: -O2 against -O0 and -Os (the chip's level)"
a=$("$OUT/league.exe" "${opts[@]}" --hash)
b=$("$OUT/league_O0.exe" "${opts[@]}" --hash)
c=$("$OUT/league_Os.exe" "${opts[@]}" --hash)
echo "     -O2: $a"
echo "     -O0: $b"
echo "     -Os: $c"
if [ "$a" = "$b" ] && [ "$a" = "$c" ]; then echo "  [PASS] identical league and tournament hashes"; else echo "  [FAIL] the optimiser changed the results"; exit 1; fi
