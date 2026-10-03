#!/usr/bin/env python3
"""
m0sim.py - run the arena's workloads from Main.elf on a Cortex-M0+ model
           (SOC3050 lesson 10; Python standard library only)

    python host/m0sim.py Main.elf [--expect test_fix.txt] [--ws 1] [--mul 32]

Nothing in this course can be run in Wokwi from the machine that wrote it, so
the lesson's cycle counts needed a second witness.  This is it: a small
instruction-set simulator for ARMv6-M Thumb, the Cortex-M0+'s instruction set.
It loads the very Main.elf that build.bat produced - the same machine code,
the same libgcc soft-float, the same newlib sinf - calls bench_seed(2026) and
then every workload in bench_table[] exactly as Main.c does (fn(64) minus
fn(0)), and counts cycles with the Cortex-M0+ timing table:

    ALU, MOV, ADD, shifts, MULS*        1        * the M0+ multiplier is a
    LDR / STR (any width)               2          build option: 1 cycle
    LDM / STM / PUSH / POP           1 + N         (fast) or 32 (small).
    POP {.., pc}                     3 + N         --mul 32 tries the other.
    B<cond> taken / not taken        2 / 1
    B, BX, BLX, MOV/ADD pc           2
    BL                               3
    (ARM Cortex-M0+ Technical Reference Manual, "Instruction set summary")

That is ZERO-WAIT-STATE memory.  At 48 MHz the C031's flash needs one wait
state (startup.c sets FLASH_ACR LATENCY = 1).  --ws 1 adds a crude model of
it: one extra cycle whenever the CPU fetches a new 32-bit word of code from
flash, and per data load from flash.  The real figure lies between the two
models - and what Wokwi reports is a third opinion (Lab Part 2).

The simulator is also checked against itself: every workload returns a
checksum of all its results, and --expect compares those with host/test_fix's
checksums, computed by the PC from the same C.  Bit-identical answers from
soft float and x86 SSE are what IEEE 754 promises for + - * / and conversions.
libm sinf is the exception: newlib's and MinGW's sinf are different code and
may round differently - the script reports that rather than failing on it.

Exit status: 0 = ran and every checksum agreed; 1 = a mismatch or an error.
"""
import argparse
import struct
import sys

MAGIC_RET = 0x0F000000          # LR for the outermost call; PC here = done
FLASH_LO, FLASH_HI = 0x08000000, 0x08008000
RAM_LO, RAM_HI = 0x20000000, 0x20003000


class Fault(Exception):
    pass


# ---------------------------------------------------------------- ELF loading
def load_elf(path):
    data = open(path, "rb").read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise SystemExit("%s: not a 32-bit little-endian ELF" % path)
    (e_type, e_machine, _, e_entry, e_phoff, e_shoff, _, _, e_phentsize,
     e_phnum, e_shentsize, e_shnum, e_shstrndx) = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    if e_machine != 40:
        raise SystemExit("%s: not an ARM ELF" % path)
    flash = bytearray(FLASH_HI - FLASH_LO)
    ram = bytearray(RAM_HI - RAM_LO)
    for i in range(e_phnum):
        p_type, p_off, p_vaddr, p_paddr, p_filesz, p_memsz, _, _ = struct.unpack_from(
            "<IIIIIIII", data, e_phoff + i * e_phentsize)
        if p_type != 1 or p_memsz == 0:
            continue
        seg = data[p_off:p_off + p_filesz] + bytes(p_memsz - p_filesz)
        # place it where the CPU will SEE it: .data at its RAM address with its
        # initial values - what Reset_Handler's copy loop would have done
        for base, buf in ((FLASH_LO, flash), (RAM_LO, ram)):
            if base <= p_vaddr < base + len(buf):
                buf[p_vaddr - base:p_vaddr - base + len(seg)] = seg
    # symbols
    secs = [struct.unpack_from("<IIIIIIIIII", data, e_shoff + i * e_shentsize) for i in range(e_shnum)]
    syms = {}
    for s in secs:
        if s[1] != 2:                     # SHT_SYMTAB
            continue
        strtab = secs[s[6]]
        for off in range(s[4], s[4] + s[5], 16):
            st_name, st_value, st_size, st_info, _, _ = struct.unpack_from("<IIIBBH", data, off)
            if st_name == 0:
                continue
            end = data.index(b"\0", strtab[4] + st_name)
            name = data[strtab[4] + st_name:end].decode()
            if not name.startswith("$"):
                syms.setdefault(name, (st_value, st_size))
    return flash, ram, syms


# ---------------------------------------------------------------- the CPU
class M0:
    def __init__(self, flash, ram, ws=0, mul_cycles=1):
        self.flash, self.ram = flash, ram
        self.r = [0] * 16
        self.n = self.z = self.c = self.v = 0
        self.cycles = 0
        self.instrs = 0
        self.ws = ws
        self.mul_cycles = mul_cycles
        self.last_fetch_word = -1

    # memory ---------------------------------------------------------------
    def _buf(self, addr, size):
        if FLASH_LO <= addr and addr + size <= FLASH_HI:
            if self.ws:
                self.cycles += self.ws          # data read from flash: wait state
            return self.flash, addr - FLASH_LO
        if RAM_LO <= addr and addr + size <= RAM_HI:
            return self.ram, addr - RAM_LO
        raise Fault("access outside flash/RAM at 0x%08X (a peripheral? workloads are pure)" % addr)

    def rd(self, addr, size, signed=False):
        if addr % size:
            raise Fault("unaligned %d-byte access at 0x%08X - the M0+ would HardFault" % (size, addr))
        buf, o = self._buf(addr, size)
        return int.from_bytes(buf[o:o + size], "little", signed=signed) & 0xFFFFFFFF

    def wr(self, addr, size, val):
        if addr % size:
            raise Fault("unaligned %d-byte store at 0x%08X" % (size, addr))
        if not (RAM_LO <= addr and addr + size <= RAM_HI):
            raise Fault("store outside RAM at 0x%08X" % addr)
        o = addr - RAM_LO
        self.ram[o:o + size] = (val & ((1 << (8 * size)) - 1)).to_bytes(size, "little")

    def fetch16(self, addr):
        if not (FLASH_LO <= addr < FLASH_HI):
            raise Fault("executing outside flash at 0x%08X" % addr)
        w = addr >> 2
        if self.ws and w != self.last_fetch_word:
            self.cycles += self.ws
        self.last_fetch_word = w
        o = addr - FLASH_LO
        return self.flash[o] | (self.flash[o + 1] << 8)

    # flags ----------------------------------------------------------------
    def nz(self, x):
        self.n = x >> 31
        self.z = int(x == 0)

    def addc(self, a, b, carry):
        u = a + b + carry
        res = u & 0xFFFFFFFF
        sa, sb, sr = a >> 31, b >> 31, res >> 31
        self.c = int(u > 0xFFFFFFFF)
        self.v = int(sa == sb and sr != sa)
        self.nz(res)
        return res

    def cond(self, cc):
        n, z, c, v = self.n, self.z, self.c, self.v
        return [z, not z, c, not c, n, not n, v, not v,
                c and not z, (not c) or z, n == v, n != v,
                (not z) and n == v, z or n != v, True, False][cc]

    # one instruction ---------------------------------------------------------
    def step(self):
        r = self.r
        pc = r[15]
        op = self.fetch16(pc)
        self.instrs += 1
        npc = pc + 2
        cyc = 1
        top5 = op >> 11

        if top5 < 3:                                   # LSLS/LSRS/ASRS #imm
            rd, rm, imm = op & 7, (op >> 3) & 7, (op >> 6) & 31
            x = r[rm]
            if top5 == 0:
                if imm:
                    self.c = (x >> (32 - imm)) & 1
                    x = (x << imm) & 0xFFFFFFFF
            elif top5 == 1:
                imm = imm or 32
                self.c = (x >> (imm - 1)) & 1
                x = x >> imm if imm < 32 else 0
            else:
                imm = imm or 32
                sx = x - (1 << 32) if x >> 31 else x
                self.c = (sx >> (imm - 1)) & 1
                x = (sx >> min(imm, 31)) & 0xFFFFFFFF
            r[rd] = x
            self.nz(x)
        elif top5 == 3:                                # ADDS/SUBS reg or imm3
            rd, rn, rm = op & 7, (op >> 3) & 7, (op >> 6) & 7
            b = rm if op & 0x400 else r[rm]
            if op & 0x200:
                r[rd] = self.addc(r[rn], (~b) & 0xFFFFFFFF, 1)
            else:
                r[rd] = self.addc(r[rn], b, 0)
        elif top5 < 8:                                 # MOVS/CMP/ADDS/SUBS imm8
            rd, imm = (op >> 8) & 7, op & 0xFF
            k = top5 - 4
            if k == 0:
                r[rd] = imm
                self.nz(imm)
            elif k == 1:
                self.addc(r[rd], (~imm) & 0xFFFFFFFF, 1)
            elif k == 2:
                r[rd] = self.addc(r[rd], imm, 0)
            else:
                r[rd] = self.addc(r[rd], (~imm) & 0xFFFFFFFF, 1)
        elif (op >> 10) == 0x10:                       # data processing
            k, rm, rd = (op >> 6) & 15, (op >> 3) & 7, op & 7
            a, b = r[rd], r[rm]
            if k == 0:   x = a & b; r[rd] = x; self.nz(x)
            elif k == 1: x = a ^ b; r[rd] = x; self.nz(x)
            elif k in (2, 3, 4, 7):                    # LSL LSR ASR ROR by register
                s = b & 0xFF
                if k == 2:
                    if s == 0: x = a
                    elif s < 32: self.c = (a >> (32 - s)) & 1; x = (a << s) & 0xFFFFFFFF
                    elif s == 32: self.c = a & 1; x = 0
                    else: self.c = 0; x = 0
                elif k == 3:
                    if s == 0: x = a
                    elif s < 32: self.c = (a >> (s - 1)) & 1; x = a >> s
                    elif s == 32: self.c = a >> 31; x = 0
                    else: self.c = 0; x = 0
                elif k == 4:
                    sa = a - (1 << 32) if a >> 31 else a
                    if s == 0: x = a
                    elif s < 32: self.c = (sa >> (s - 1)) & 1; x = (sa >> s) & 0xFFFFFFFF
                    else: self.c = a >> 31; x = 0xFFFFFFFF if a >> 31 else 0
                else:
                    if s == 0: x = a
                    else:
                        s5 = s & 31
                        x = ((a >> s5) | (a << (32 - s5))) & 0xFFFFFFFF if s5 else a
                        self.c = x >> 31
                r[rd] = x; self.nz(x)
            elif k == 5: r[rd] = self.addc(a, b, self.c)
            elif k == 6: r[rd] = self.addc(a, (~b) & 0xFFFFFFFF, self.c)
            elif k == 8: self.nz(a & b)
            elif k == 9: r[rd] = self.addc(0, (~b) & 0xFFFFFFFF, 1)     # RSBS #0 (NEG)
            elif k == 10: self.addc(a, (~b) & 0xFFFFFFFF, 1)
            elif k == 11: self.addc(a, b, 0)
            elif k == 12: x = a | b; r[rd] = x; self.nz(x)
            elif k == 13:
                x = (a * b) & 0xFFFFFFFF; r[rd] = x; self.nz(x); cyc = self.mul_cycles
            elif k == 14: x = a & ~b & 0xFFFFFFFF; r[rd] = x; self.nz(x)
            else: x = (~b) & 0xFFFFFFFF; r[rd] = x; self.nz(x)
        elif (op >> 10) == 0x11:                       # hi-register ops, BX/BLX
            k = (op >> 8) & 3
            rm = (op >> 3) & 15
            rd = (op & 7) | ((op >> 4) & 8)
            val = (pc + 4) if rm == 15 else r[rm]
            if k == 0:
                cur = (pc + 4) if rd == 15 else r[rd]
                x = (cur + val) & 0xFFFFFFFF
                if rd == 15: npc = x & ~1; cyc = 2
                else: r[rd] = x
            elif k == 1:
                cur = (pc + 4) if rd == 15 else r[rd]
                self.addc(cur, (~val) & 0xFFFFFFFF, 1)
            elif k == 2:
                if rd == 15: npc = val & ~1; cyc = 2
                else: r[rd] = val
            else:
                if not (val & 1):
                    raise Fault("BX to an ARM-state address 0x%08X at 0x%08X" % (val, pc))
                if op & 0x80:
                    r[14] = (pc + 2) | 1
                npc = val & ~1
                cyc = 2
        elif top5 == 9:                                # LDR literal
            rd = (op >> 8) & 7
            r[rd] = self.rd(((pc + 4) & ~3) + (op & 0xFF) * 4, 4)
            cyc = 2
        elif (op >> 12) == 5:                          # load/store register offset
            k, rm, rn, rd = (op >> 9) & 7, (op >> 6) & 7, (op >> 3) & 7, op & 7
            addr = (r[rn] + r[rm]) & 0xFFFFFFFF
            if k == 0: self.wr(addr, 4, r[rd])
            elif k == 1: self.wr(addr, 2, r[rd])
            elif k == 2: self.wr(addr, 1, r[rd])
            elif k == 3: r[rd] = self.rd(addr, 1, True)
            elif k == 4: r[rd] = self.rd(addr, 4)
            elif k == 5: r[rd] = self.rd(addr, 2)
            elif k == 6: r[rd] = self.rd(addr, 1)
            else: r[rd] = self.rd(addr, 2, True)
            cyc = 2
        elif (op >> 13) == 3:                          # LDR/STR(B) imm5
            b, load = (op >> 12) & 1, (op >> 11) & 1
            imm, rn, rd = (op >> 6) & 31, (op >> 3) & 7, op & 7
            size = 1 if b else 4
            addr = (r[rn] + imm * size) & 0xFFFFFFFF
            if load: r[rd] = self.rd(addr, size)
            else: self.wr(addr, size, r[rd])
            cyc = 2
        elif top5 in (0x10, 0x11):                     # LDRH/STRH imm5
            imm, rn, rd = (op >> 6) & 31, (op >> 3) & 7, op & 7
            addr = (r[rn] + imm * 2) & 0xFFFFFFFF
            if top5 & 1: r[rd] = self.rd(addr, 2)
            else: self.wr(addr, 2, r[rd])
            cyc = 2
        elif top5 in (0x12, 0x13):                     # LDR/STR SP-relative
            rd = (op >> 8) & 7
            addr = (r[13] + (op & 0xFF) * 4) & 0xFFFFFFFF
            if top5 & 1: r[rd] = self.rd(addr, 4)
            else: self.wr(addr, 4, r[rd])
            cyc = 2
        elif top5 == 0x14:                             # ADR
            r[(op >> 8) & 7] = ((pc + 4) & ~3) + (op & 0xFF) * 4
        elif top5 == 0x15:                             # ADD Rd, SP, #imm
            r[(op >> 8) & 7] = (r[13] + (op & 0xFF) * 4) & 0xFFFFFFFF
        elif (op >> 12) == 0xB:                        # miscellaneous
            if (op >> 8) == 0xB0:
                imm = (op & 0x7F) * 4
                r[13] = (r[13] - imm if op & 0x80 else r[13] + imm) & 0xFFFFFFFF
            elif (op >> 8) == 0xB2:
                k, rm, rd = (op >> 6) & 3, (op >> 3) & 7, op & 7
                x = r[rm]
                if k == 0: x = (x & 0xFFFF) - ((x & 0x8000) << 1)
                elif k == 1: x = (x & 0xFF) - ((x & 0x80) << 1)
                elif k == 2: x = x & 0xFFFF
                else: x = x & 0xFF
                r[rd] = x & 0xFFFFFFFF
            elif (op >> 9) == 0x5A:                    # PUSH
                regs = [i for i in range(8) if op & (1 << i)] + ([14] if op & 0x100 else [])
                sp = r[13] - 4 * len(regs)
                for i, reg in enumerate(regs):
                    self.wr(sp + 4 * i, 4, r[reg])
                r[13] = sp
                cyc = 1 + len(regs)
            elif (op >> 9) == 0x5E:                    # POP
                regs = [i for i in range(8) if op & (1 << i)]
                sp = r[13]
                for reg in regs:
                    r[reg] = self.rd(sp, 4); sp += 4
                cyc = 1 + len(regs)
                if op & 0x100:
                    val = self.rd(sp, 4); sp += 4
                    npc = val & ~1
                    cyc = 3 + len(regs) + 1
                r[13] = sp
            elif (op >> 8) == 0xBA:
                k, rm, rd = (op >> 6) & 3, (op >> 3) & 7, op & 7
                x = r[rm]
                if k == 0: x = int.from_bytes(x.to_bytes(4, "little"), "big")
                elif k == 1: x = ((x & 0x00FF00FF) << 8 | (x >> 8) & 0x00FF00FF) & 0xFFFFFFFF
                elif k == 3:
                    h = ((x & 0xFF) << 8) | ((x >> 8) & 0xFF)
                    x = (h - 0x10000 if h & 0x8000 else h) & 0xFFFFFFFF
                else: raise Fault("undefined 0x%04X at 0x%08X" % (op, pc))
                r[rd] = x
            elif (op >> 8) == 0xBF or (op & 0xFFE8) == 0xB660:
                pass                                   # NOP / hints / CPS
            else:
                raise Fault("unsupported 0x%04X at 0x%08X" % (op, pc))
        elif (op >> 12) == 0xC:                        # STM / LDM
            rn = (op >> 8) & 7
            regs = [i for i in range(8) if op & (1 << i)]
            addr = r[rn]
            if op & 0x800:
                for reg in regs:
                    r[reg] = self.rd(addr, 4); addr += 4
                if rn not in regs:
                    r[rn] = addr
            else:
                for reg in regs:
                    self.wr(addr, 4, r[reg]); addr += 4
                r[rn] = addr
            cyc = 1 + len(regs)
        elif (op >> 12) == 0xD:                        # B<cond>, SVC, UDF
            cc = (op >> 8) & 15
            if cc >= 14:
                raise Fault("SVC/UDF 0x%04X at 0x%08X" % (op, pc))
            if self.cond(cc):
                off = op & 0xFF
                off = off - 256 if off & 0x80 else off
                npc = pc + 4 + off * 2
                cyc = 2
        elif top5 == 0x1C:                             # B
            off = op & 0x7FF
            off = off - 2048 if off & 0x400 else off
            npc = pc + 4 + off * 2
            cyc = 2
        elif top5 == 0x1E:                             # 32-bit: BL (or DMB/DSB/ISB)
            op2 = self.fetch16(pc + 2)
            npc = pc + 4
            if (op2 & 0xD000) == 0xD000:
                s = (op >> 10) & 1
                j1, j2 = (op2 >> 13) & 1, (op2 >> 11) & 1
                i1, i2 = 1 - (j1 ^ s), 1 - (j2 ^ s)
                imm = (s << 24) | (i1 << 23) | (i2 << 22) | ((op & 0x3FF) << 12) | ((op2 & 0x7FF) << 1)
                if s:
                    imm -= 1 << 25
                r[14] = (pc + 4) | 1
                npc = pc + 4 + imm
                cyc = 3
            elif op == 0xF3BF:
                cyc = 3                                # barriers
            else:
                raise Fault("unsupported 32-bit 0x%04X %04X at 0x%08X" % (op, op2, pc))
        else:
            raise Fault("unsupported 0x%04X at 0x%08X" % (op, pc))

        r[15] = npc & 0xFFFFFFFF
        self.cycles += cyc

    def call(self, addr, *args, limit=50_000_000):
        """Call a Thumb function; return (r0, cycles, instructions)."""
        r = self.r
        for i, a in enumerate(args):
            r[i] = a & 0xFFFFFFFF
        r[13] = RAM_HI                                 # a fresh stack each call
        r[14] = MAGIC_RET | 1
        r[15] = addr & ~1
        c0, i0 = self.cycles, self.instrs
        self.last_fetch_word = -1
        while r[15] != MAGIC_RET:
            self.step()
            if self.instrs - i0 > limit:
                raise Fault("no return after %d instructions" % limit)
        return r[0], self.cycles - c0, self.instrs - i0


def cstr(cpu, addr):
    out = bytearray()
    while True:
        b = cpu.rd(addr, 1)
        if b == 0:
            return out.decode()
        out.append(b)
        addr += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("elf")
    ap.add_argument("--expect", help="host/test_fix output with CHK lines to compare against")
    ap.add_argument("--ws", type=int, default=None, help="flash wait states for a second column (default: show 0 and 1)")
    ap.add_argument("--mul", type=int, default=1, help="MULS cycles: 1 (fast multiplier) or 32 (small)")
    ap.add_argument("--n", type=int, default=64, help="iterations per workload (Main.c: 64)")
    ap.add_argument("--seed", type=int, default=2026)
    a = ap.parse_args()

    flash, ram, syms = load_elf(a.elf)
    for need in ("bench_seed", "bench_table", "bench_count"):
        if need not in syms:
            raise SystemExit("%s has no symbol %s - is it lesson 10's Main.elf?" % (a.elf, need))

    models = [0, 1] if a.ws is None else [a.ws]
    results = {}
    names = []
    for ws in models:
        cpu = M0(flash[:], ram[:], ws=ws, mul_cycles=a.mul)
        cpu.call(syms["bench_seed"][0], a.seed)
        count = cpu.rd(syms["bench_count"][0], 4)
        table = syms["bench_table"][0]
        for i in range(count):
            p_work, p_fmt, p_fn = (cpu.rd(table + 12 * i + 4 * k, 4) for k in range(3))
            key = (cstr(cpu, p_work), cstr(cpu, p_fmt))
            chk, c_full, n_full = cpu.call(p_fn, a.n)
            _, c_empty, n_empty = cpu.call(p_fn, 0)
            per_x10 = ((c_full - c_empty) * 10 + a.n // 2) // a.n
            results.setdefault(key, {})[ws] = (per_x10, chk, (n_full - n_empty) / a.n)
            if ws == models[0]:
                names.append(key)

    expect = {}
    if a.expect:
        for line in open(a.expect, encoding="utf-8", errors="replace"):
            if line.startswith("CHK "):
                w, f, h = line[4:].strip().split("|")
                expect[(w, f)] = int(h, 16)

    print("\n== Cortex-M0+ cycle model: %s, seed %d, n = %d, MULS = %d cycle(s) ==" % (a.elf, a.seed, a.n, a.mul))
    hdr = "  job    format      instr/iter " + "".join("  cyc/iter %dWS" % ws for ws in models) + "   x winner   checksum  vs host"
    print(hdr)
    win = {}
    for key in names:
        v = results[key][models[0]][0]
        win[key[0]] = min(win.get(key[0], v), v)
    bad = 0
    for key in names:
        row = results[key]
        per0, chk, ipi = row[models[0]]
        cols = "".join("  %10.1f   " % (row[ws][0] / 10) for ws in models)
        ratio = per0 / win[key[0]] if win[key[0]] else 0
        verdict = ""
        if key in expect:
            if expect[key] == chk:
                verdict = "same"
            elif key[1] == "libm sinf":
                verdict = "differs (newlib vs MinGW sinf - expected)"
            else:
                verdict = "MISMATCH (host %08X)" % expect[key]
                bad += 1
        print("  %-6s %-10s %9.1f %s %8.1f    %08X  %s" % (key[0], key[1], ipi, cols, ratio if key[0] != "loop" else 0, chk, verdict))
    print("  (cycles from the Cortex-M0+ TRM timing table; 0WS = ideal memory, 1WS = +1 per flash word fetched or flash data read)")

    # memcpy of 1 KB, as Main.c's DMA demo does it - the number the DMA copy races
    if all(k in syms for k in ("memcpy", "copy_src", "copy_dst")):
        for ws in models:
            cpu = M0(flash[:], ram[:], ws=ws, mul_cycles=a.mul)
            _, cyc, ins = cpu.call(syms["memcpy"][0], syms["copy_dst"][0], syms["copy_src"][0], 1024)
            print("  memcpy 1 KB RAM->RAM (newlib-nano): %d instructions, %d cycles at %dWS = %.2f bytes/cycle"
                  % (ins, cyc, ws, 1024 / cyc))
    if a.expect:
        print("\n%s: %d checksum mismatch(es) between the M0+ model and the PC" % ("FAILED" if bad else "PASSED", bad))
    return 1 if bad else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fault as e:
        print("m0sim: fault: %s" % e)
        sys.exit(1)
