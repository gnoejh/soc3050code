#!/usr/bin/env bash
# fpu_compare.sh - compile host/kernel.c for the Cortex-M0+ and the Cortex-M4F
# and disassemble both.  SOC3050 lesson 10.  Run from anywhere:
#     bash host/fpu_compare.sh
# Uses only the vendored toolchain.  Compile only: nothing is linked or run.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
BIN="$HERE/../../../tools/arm-toolchain/bin"
OUT="${TMPDIR:-/tmp}/soc3050_fpu"
mkdir -p "$OUT"

"$BIN/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -Os -c \
    "$HERE/kernel.c" -o "$OUT/kernel_m0plus.o"
"$BIN/arm-none-eabi-gcc" -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -c \
    "$HERE/kernel.c" -o "$OUT/kernel_m4f.o"

echo "================ Cortex-M0+ (no FPU): soft float ================"
"$BIN/arm-none-eabi-objdump" -d -r "$OUT/kernel_m0plus.o"
echo
echo "================ Cortex-M4F (FPv4-SP): hard float ==============="
"$BIN/arm-none-eabi-objdump" -d "$OUT/kernel_m4f.o"
echo
echo "---- code size (bytes of .text) ----"
"$BIN/arm-none-eabi-size" "$OUT/kernel_m0plus.o" "$OUT/kernel_m4f.o"
