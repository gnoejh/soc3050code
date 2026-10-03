#!/usr/bin/env bash
# host/run.sh - lesson 11 host test (Git Bash, Linux, macOS).  Build the
# firmware first (build.bat) so the objdump cross-check has a Main.elf.
set -e
cd "$(dirname "$0")"
OBJDUMP=../../../tools/arm-toolchain/bin/arm-none-eabi-objdump
[ -x "$OBJDUMP" ] || [ -x "$OBJDUMP.exe" ] || OBJDUMP=arm-none-eabi-objdump
rm -f objdump.txt
if [ -f ../Main.elf ]; then "$OBJDUMP" -d ../Main.elf > objdump.txt; fi
gcc -std=c11 -O2 -Wall -Wextra -o test test.c ../diag.c
./test objdump.txt
