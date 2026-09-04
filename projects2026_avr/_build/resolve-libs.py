#!/usr/bin/env python3
"""Work out the minimal, conflict-free shared_libs set a lesson needs.

Several lessons deliberately define their own ISR or UART routine that a shared
library also defines, so "link everything" fails with multiple-definition errors.
This reads the symbol tables of Main.o and of every shared library object and
picks the smallest closure that resolves Main's undefined symbols without ever
introducing a duplicate definition.

Usage:  python resolve-libs.py <lesson-dir> [<lesson-dir> ...]
        python resolve-libs.py --all          # every lesson under projects2026_avr
Prints  "<lesson-name>: _init _port _uart"
"""
import os, re, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BIN = os.path.join(ROOT, "tools", "avr-toolchain", "bin")
GCC = os.path.join(BIN, "avr-gcc.exe")
NM = os.path.join(BIN, "avr-nm.exe")
LIBDIR = os.path.join(ROOT, "shared_libs")

# Variant implementations of the same module; never link these.
EXCLUDED = {"_glcd_legacy", "_init_safe", "_init_simple", "uart_enhanced"}

CFLAGS = ["-mmcu=atmega128", "-DF_CPU=16000000UL", "-DBAUD=9600", "-Os",
          "-ffunction-sections", "-fdata-sections", "-w"]


def compile_obj(src, out, extra_inc=()):
    cmd = [GCC, *CFLAGS, "-I" + LIBDIR]
    cmd += ["-I" + i for i in extra_inc]
    cmd += ["-c", src, "-o", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def symbols(obj):
    """Return (globally-defined, undefined) symbol names for an object file.

    Only uppercase nm types are global and therefore linkable; lowercase types
    are file-local and cannot collide.  'A' (absolute) and 'W'/'V' (weak) are
    skipped because they legitimately repeat in every object.
    """
    r = subprocess.run([NM, obj], capture_output=True, text=True)
    defined, undefined = set(), set()
    for line in r.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3:
            kind, name = parts[1], parts[2]
        elif len(parts) == 2:
            kind, name = parts[0], parts[1]
        else:
            continue
        if kind == "U":
            undefined.add(name)
        elif kind in "TDBRCG":
            defined.add(name)
    return defined, undefined


def lib_catalog(tmp):
    """{libname: (defined, undefined)} for each linkable shared library."""
    cat = {}
    for fn in sorted(os.listdir(LIBDIR)):
        if not fn.endswith(".c"):
            continue
        name = fn[:-2]
        if name in EXCLUDED:
            continue
        obj = os.path.join(tmp, name + ".o")
        ok, err = compile_obj(os.path.join(LIBDIR, fn), obj)
        if not ok:
            print(f"  ! {name} does not compile, skipping", file=sys.stderr)
            continue
        cat[name] = symbols(obj)
    return cat


def resolve(lesson_dir, cat, tmp):
    main = os.path.join(lesson_dir, "Main.c")
    if not os.path.isfile(main):
        main = os.path.join(lesson_dir, "main.c")
    obj = os.path.join(tmp, "main_lesson.o")
    ok, err = compile_obj(main, obj, extra_inc=[lesson_dir])
    if not ok:
        return None, err

    defined, pending = symbols(obj)
    chosen = []
    changed = True
    while changed:
        changed = False
        for name, (ldef, lund) in cat.items():
            if name in chosen:
                continue
            if not (lund | ldef) and not ldef:
                continue
            # does this library supply anything still missing?
            if not (ldef & pending):
                continue
            # would it duplicate something already defined?
            if ldef & defined:
                continue
            chosen.append(name)
            defined |= ldef
            pending = (pending | lund) - defined
            changed = True
    return chosen, None


def main(argv):
    targets = argv
    base = os.path.join(ROOT, "projects2026_avr")
    if not targets or targets == ["--all"]:
        targets = [os.path.join(base, d) for d in sorted(os.listdir(base))
                   if os.path.isdir(os.path.join(base, d)) and not d.startswith("_")]
    with tempfile.TemporaryDirectory() as tmp:
        cat = lib_catalog(tmp)
        rc = 0
        for d in targets:
            libs, err = resolve(d, cat, tmp)
            if libs is None:
                print(f"{os.path.basename(d)}: COMPILE ERROR")
                print("\n".join(err.splitlines()[:6]))
                rc = 1
            else:
                print(f"{os.path.basename(d)}: {' '.join(libs)}")
        return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
