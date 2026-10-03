#!/usr/bin/env python3
"""
flashcost.py - what each number format costs in flash, measured from Main.elf
               (SOC3050 lesson 10; Python standard library only)

    python host/flashcost.py Main.elf

Runs the vendored arm-none-eabi-nm --size-sort -S over the ELF and adds up the
library routines each format pulls in.  A routine that two names share (gcc
gives __aeabi_lmul and __muldi3 the same address) is counted once.

The groups are this lesson's judgement of "who needed it": __clzsi2, for
example, is a soft-float helper here, and __udivsi3 is counted with neither
because printf needs it anyway.  The numbers are measured; the grouping is
an opinion, and the slide says so.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NM = os.path.join(HERE, "..", "..", "..", "tools", "arm-toolchain", "bin", "arm-none-eabi-nm")

GROUPS = [
    ("float + - * /, compare, convert (libgcc soft float)",
     r"__aeabi_f(add|sub|mul|div|2iz|cmp\w*)$|__aeabi_(i2f|ui2f|cf\w*)$|__\w+sf[23]$|__clzsi2$"),
    ("libm sinf (newlib: sinf + kernels + range reduction)",
     r"(sinf|__kernel_sinf|__kernel_cosf|__ieee754_rem_pio2f|__kernel_rem_pio2f|floorf|scalbnf|fabsf"
     r"|two_over_pi|npio2_hw|PIo2|init_jk)$"),
    ("Q16.16 mul: 32x32->64 multiply (__aeabi_lmul)", r"(__aeabi_lmul|__muldi3)$"),
    ("Q16.16 div: 64-bit signed divide (__aeabi_ldivmod ...)",
     r"(__aeabi_ldivmod|__gnu_ldivmod_helper|__divdi3|__udivmoddi4|__clzdi2|__aeabi_ldiv0|__aeabi_uldivmod)$"),
    ("int32 div: 32-bit signed divide (__aeabi_idiv)", r"(__divsi3|__aeabi_idiv|__aeabi_idivmod|__aeabi_idiv0)$"),
    ("Q15 sine: table + q15_sin()", r"(quarter|q15_sin)$"),
    ("int32 + - *: built-in instructions", r"$^"),
]


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    nm = NM + (".exe" if os.name == "nt" else "")
    if not os.path.exists(nm):
        nm = "arm-none-eabi-nm"
    out = subprocess.run([nm, "--size-sort", "-S", sys.argv[1]], capture_output=True, text=True, check=True).stdout
    seen = set()
    totals = [[name, 0, []] for name, _ in GROUPS]
    bench = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) != 4:
            continue
        addr, size, _, sym = parts[0], int(parts[1], 16), parts[2], parts[3]
        if sym.startswith("b_"):
            bench.append((sym, size))
        for g, (_, pat) in enumerate(GROUPS):
            if re.match(pat, sym):
                if addr not in seen:
                    seen.add(addr)
                    totals[g][1] += size
                    totals[g][2].append(sym)
                break
    print("\n== Flash cost per number format: %s ==" % sys.argv[1])
    for name, size, syms in totals:
        print("  %5d B  %s" % (size, name))
        if syms:
            print("           %s" % " ".join(sorted(syms)))
    print("\n  each workload's own loop (static functions in bench.c):")
    for sym, size in sorted(bench):
        print("  %5d B  %s" % (size, sym))
    return 0


if __name__ == "__main__":
    sys.exit(main())
