#!/usr/bin/env python3
"""
cycles.py - how many Cortex-M0+ cycles do world_step() and control_step() cost?
            (SOC3050 lesson 13; Python standard library only)

    python host/cycles.py [Main.elf] [--ws 0|1] [--seconds 1.2]

The firmware measures this itself (`stats` in the shell, SysTick cycle
counting), but nothing can be run in Wokwi from the machine that wrote this
lesson.  So the cycle budget has a second witness: lesson 10's instruction-
level Cortex-M0+ model, host/m0sim.py in 10_Fixed_Point_And_DMA, which runs
the very machine code in this lesson's Main.elf - libgcc's soft float and all -
and counts cycles from the Cortex-M0+ TRM timing table.

It flies the same sequence as host/sitl.c's take-off, inside the model:
world_init, control_init, 0.6 s on the ground with the arm switch on
(disarmed until the estimator has settled, then armed at zero throttle),
then throttle 0.35 and a 20 degree roll step.  Every call is timed.

Exit status 1 if the total exceeds the CPU budget (world + control > 50 %).
"""
import argparse
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True        # leave no __pycache__ in lesson 10's folder
sys.path.insert(0, os.path.join(HERE, "..", "..", "10_Fixed_Point_And_DMA", "host"))
try:
    import m0sim
except ImportError:
    raise SystemExit("needs lesson 10's host/m0sim.py (10_Fixed_Point_And_DMA/host)")

# sensors_t layout (control.h): 3 gyro, 3 accel floats, seq, imu_ok(+pad),
# rc_roll, rc_pitch, rc_yaw_rate, rc_throttle, rc_arm(+pad)
OFF_RC_ROLL, OFF_RC_THR, OFF_RC_ARM = 32, 44, 48
OFF_ARMED = 16                                  # actuators_t.armed


def f2u(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]


def u2f(u):
    return struct.unpack("<f", struct.pack("<I", u))[0]


class Stat:
    def __init__(self):
        self.n = self.total = 0
        self.lo, self.hi = 10 ** 9, 0

    def add(self, c):
        self.n += 1
        self.total += c
        self.lo, self.hi = min(self.lo, c), max(self.hi, c)

    def row(self, name, period_us):
        mean = self.total / max(self.n, 1)
        print("  %-26s %6d %8d %8.0f %8d %7.0f us %6.1f %%" % (
            name, self.n, self.lo, mean, self.hi, mean / 48.0, 100.0 * mean / 48.0 / period_us))
        return mean


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf", nargs="?", default=os.path.join(HERE, "..", "Main.elf"))
    ap.add_argument("--ws", type=int, default=1, help="flash wait states (startup.c sets 1 at 48 MHz)")
    ap.add_argument("--seconds", type=float, default=1.2)
    a = ap.parse_args()

    flash, ram, syms = m0sim.load_elf(a.elf)
    need = ("world_init", "world_step", "world_sense", "control_init", "control_step",
            "world", "g_sens", "g_act")
    for s in need:
        if s not in syms:
            raise SystemExit("%s has no symbol %s - build lesson 13 first" % (a.elf, s))
    S = {k: syms[k][0] for k in need}
    cpu = m0sim.M0(flash, ram, ws=a.ws)

    cpu.call(S["world_init"], S["world"], 0x1234567)
    cpu.call(S["control_init"])
    cpu.call(S["world_sense"], S["world"], S["g_sens"])
    cpu.wr(S["g_sens"] + OFF_RC_ARM, 1, 1)                 # arm switch on
    cpu.wr(S["g_sens"] + OFF_RC_THR, 4, f2u(0.0))

    st_w, st_s = Stat(), Stat()
    st_c_safe, st_c_fly = Stat(), Stat()
    steps = int(a.seconds * 500)
    motors = S["g_act"]                                     # motor[4] is first
    armed_at = None
    for i in range(1, steps + 1):
        st_w.add(cpu.call(S["world_step"], S["world"], motors)[1])
        st_s.add(cpu.call(S["world_sense"], S["world"], S["g_sens"])[1])
        if i % 2 == 0:
            flying = u2f(cpu.rd(S["g_sens"] + OFF_RC_THR, 4)) > 0.05
            c = cpu.call(S["control_step"], S["g_sens"], S["g_act"])[1]
            (st_c_fly if flying else st_c_safe).add(c)
            if armed_at is None and cpu.rd(S["g_act"] + OFF_ARMED, 1):
                armed_at = i * 0.002
                cpu.wr(S["g_sens"] + OFF_RC_THR, 4, f2u(0.35))
                cpu.wr(S["g_sens"] + OFF_RC_ROLL, 4, f2u(20.0 * 3.14159265 / 180.0))

    print("== Cortex-M0+ model (lesson 10's m0sim.py), %s, %d flash wait state(s) ==" % (
        os.path.basename(a.elf), a.ws))
    print("  %.1f s simulated; armed at %s s, then throttle 0.35 and a 20 deg roll step\n" % (
        a.seconds, "%.3f" % armed_at if armed_at else "NEVER"))
    print("  %-26s %6s %8s %8s %8s %10s %8s" % ("function", "calls", "min cyc", "mean", "max",
                                                "time", "of period"))
    w = st_w.row("world_step   (every 2 ms)", 2000)
    s = st_s.row("world_sense  (every 2 ms)", 2000)
    st_c_safe.row("control_step disarmed/ground", 4000)
    c = st_c_fly.row("control_step flying (4 ms)", 4000)
    load = 100.0 * ((w + s) / 48.0 / 2000.0 + c / 48.0 / 4000.0)
    print("\n  CPU for world + control at 500 / 250 Hz: %.1f %% of 48 MHz" % load)
    ok = armed_at is not None and st_c_fly.n > 0 and load < 50.0
    print("%s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except m0sim.Fault as e:
        print("m0sim: fault: %s" % e)
        sys.exit(1)
