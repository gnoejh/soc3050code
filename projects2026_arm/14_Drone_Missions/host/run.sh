#!/usr/bin/env sh
# run.sh - build and run lesson 14's host tests (Git Bash, Linux, macOS)
#
#   1. sitl: every flight test - default mission in still air and in wind,
#      position hold, line vs aim, determinism, all three failsafes
#   2. mission.py scores the wind flight's telemetry log and must agree with
#      the C test's own numbers
#   3. the round trip: mission.py -> frames -> the C parser -> $MIS frames
#      -> mission.py, which must get back exactly the file it started from
#
# Exits non-zero if anything fails.
set -e
cd "$(dirname "$0")"
mkdir -p out
# "python3" can be the Windows Store stub, which runs nothing: test it.
PY=python3
"$PY" -c "import sys" 2>/dev/null || PY=python

echo "== build =="
gcc -O2 -std=c11 -Wall -Wextra -I.. -I../../_lib -o out/sitl \
    sitl.c ../world.c ../flight.c ../mavlite.c ../ui.c ../fm.c \
    ../../_lib/proto.c ../../_lib/oled.c -lm
echo

./out/sitl
echo

echo "== mission.py --score on the wind flight, checked against the C numbers =="
"$PY" mission.py --score out/wind.log --race --expect out/wind.expect
echo

echo "== round trip: mission.py -> C parser -> mission.py =="
"$PY" mission.py default.mission > out/frames.txt
./out/sitl --parse out/frames.txt out/loaded.txt
"$PY" mission.py --verify-mis default.mission out/loaded.txt
echo
echo "ALL HOST TESTS PASSED"
