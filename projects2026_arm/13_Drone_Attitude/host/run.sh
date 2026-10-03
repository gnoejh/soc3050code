#!/usr/bin/env sh
# host/run.sh - build and run lesson 13's SITL test with the PC's gcc.
# Exit status is the test's: 0 = every check passed.
cd "$(dirname "$0")" || exit 1
gcc -std=c11 -O2 -Wall -Wextra -I.. -I../../_lib -o sitl sitl.c ../world.c ../control.c -lm || exit 1
./sitl
