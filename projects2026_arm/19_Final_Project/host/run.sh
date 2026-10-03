#!/usr/bin/env sh
# 19_Final_Project/host - compile the pure-C half with the PC's gcc and run
# the tests.  Exit status 0 = every check passed.  See run.bat.
set -e
cd "$(dirname "$0")"
gcc -std=c11 -O2 -Wall -Wextra -I.. -I../../_lib -o test_app test_app.c ../app.c ../health.c ../../_lib/oled.c
./test_app
