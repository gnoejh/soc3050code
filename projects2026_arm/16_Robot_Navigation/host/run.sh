#!/usr/bin/env bash
# host/run.sh - build and run lesson 16's software-in-the-loop test on a PC,
# then check planner.py's A* against the C one on every level.
#   bash host/run.sh        (from anywhere)
#   bash host/run.sh -v     one line per robot run
# Exit status 0 only if both pass.
set -e
cd "$(dirname "$0")"
L=..
gcc -O2 -std=c11 -Wall -Wextra -I$L -I$L/../_lib -o sitl.exe sitl.c \
    $L/world.c $L/nav.c $L/astar.c $L/levels.c $L/view.c $L/mathx.c $L/../_lib/oled.c -lm
./sitl.exe "$@"
python planner.py --compare paths.txt
