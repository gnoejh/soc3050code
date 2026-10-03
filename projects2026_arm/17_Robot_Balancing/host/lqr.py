#!/usr/bin/env python3
"""lqr.py - design the balancing robot's LQR gain.  SOC3050 lesson 17.

Standard library only.  It:

  1. reads ../params.h - the SAME numbers the firmware's world model uses -
     by evaluating every "#define NAME expression" line in order;
  2. linearises the robot about upright: x' = A x + B u, with the state
     x = [position, speed, tilt, tilt rate] and u the motor voltage;
  3. discretises it for the controller's period CTRL_DT (zero-order hold,
     a matrix exponential by Taylor series with scaling and squaring);
  4. solves the discrete-time algebraic Riccati equation by iterating it
     until it stops changing, and forms K = (R + B'PB)^-1 B'PA;
  5. CHECKS the result: closed-loop eigenvalues of A - BK (characteristic
     polynomial by Faddeev-LeVerrier, roots by Durand-Kerner), spectral
     radius < 1, and a simulated recovery of the LINEAR model from 6 degrees;
  6. prints K and one line of C to paste into control.c.

    python lqr.py                        design with the weights in params.h
    python lqr.py --Q 30,2,50,0.5 --R 1  try other weights (Lab Part 4)
    python lqr.py --payload 0.3          design for a robot carrying 0.3 kg
    python lqr.py --check ../control.c   exit 1 unless control.c's LQR_K matches

Exit status: 0 = a stabilising K (and, with --check, the firmware has it).
"""
import argparse
import cmath
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


# --------------------------------------------------------------------------
# params.h -> a dict of floats
# --------------------------------------------------------------------------
def read_params(path):
    names = {}
    text = re.sub(r"/\*.*?\*/", " ", open(path).read(), flags=re.S)   # strip comments
    for line in text.splitlines():
        m = re.match(r"\s*#define\s+([A-Z_][A-Z0-9_]*)\s+(.+?)\s*$", line)
        if not m:
            continue                           # the include guard has no value
        expr = re.sub(r"(\d)[fF]\b", r"\1", m.group(2))   # 0.80f -> 0.80
        if not re.fullmatch(r"[\w\s.+\-*/()]+", expr):
            sys.exit(f"params.h: cannot read  #define {m.group(1)} {m.group(2)}")
        names[m.group(1)] = float(eval(expr, {"__builtins__": {}}, dict(names)))
    return names


# --------------------------------------------------------------------------
# Tiny matrix helpers: a matrix is a list of rows
# --------------------------------------------------------------------------
def zeros(n, m):
    return [[0.0] * m for _ in range(n)]


def eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
            for i in range(len(a))]


def add(a, b, s=1.0):
    return [[a[i][j] + s * b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def scale(a, s):
    return [[s * v for v in row] for row in a]


def tr(a):
    return [list(r) for r in zip(*a)]


def norm1(a):
    return max(sum(abs(a[i][j]) for i in range(len(a))) for j in range(len(a[0])))


def expm(a):
    """e^A: scale A down until it is small, Taylor to 20 terms, square back up."""
    s = 0
    while norm1(a) / (2 ** s) > 0.5:
        s += 1
    a = scale(a, 1.0 / 2 ** s)
    n = len(a)
    term, out = eye(n), eye(n)
    for k in range(1, 21):
        term = scale(mul(term, a), 1.0 / k)
        out = add(out, term)
    for _ in range(s):
        out = mul(out, out)
    return out


# --------------------------------------------------------------------------
# The model, linearised about upright
# --------------------------------------------------------------------------
def linear_model(p, payload=0.0):
    """Continuous-time A (4x4) and B (4x1) - the same equations as world.c with
    sin(th) = th, cos(th) = 1 and th'^2 = 0."""
    m = p["BOT_MB"] + payload
    l = (p["BOT_MB"] * p["BOT_L"] + payload * p["PAY_H"]) / m
    i = p["BOT_IB"] + p["BOT_MB"] * (p["BOT_L"] - l) ** 2 + payload * (p["PAY_H"] - l) ** 2
    r, g = p["BOT_R"], p["GRAV"]
    a11 = m + p["BOT_MW"] + p["BOT_IW"] / r ** 2
    a12 = m * l
    a22 = i + m * l * l
    det = a11 * a22 - a12 * a12
    cu = p["MOT_N"] * p["MOT_K"] / p["MOT_RES"]                       # N m per volt
    cw = p["MOT_N"] * (p["MOT_K"] ** 2 / p["MOT_RES"] + p["MOT_B"])  # N m per rad/s
    #         over  [x,  x',                     th,         th',     u]
    b1 = [0.0, -cw / r ** 2 - p["BOT_BROLL"], 0.0, cw / r, cu / r]     # wheel equation
    b2 = [0.0, cw / r, m * g * l, -cw, -cu]                            # body equation
    xdd = [(a22 * u - a12 * v) / det for u, v in zip(b1, b2)]
    thdd = [(-a12 * u + a11 * v) / det for u, v in zip(b1, b2)]
    A = [[0, 1, 0, 0], xdd[:4], [0, 0, 0, 1], thdd[:4]]
    B = [[0.0], [xdd[4]], [0.0], [thdd[4]]]
    return A, B


def discretise(A, B, dt):
    """Zero-order hold: exp([[A, B], [0, 0]] dt) = [[Ad, Bd], [0, 1]]."""
    n = len(A)
    big = zeros(n + 1, n + 1)
    for i in range(n):
        for j in range(n):
            big[i][j] = A[i][j] * dt
        big[i][n] = B[i][0] * dt
    e = expm(big)
    return [row[:n] for row in e[:n]], [[row[n]] for row in e[:n]]


def dare(Ad, Bd, Q, R, iters=200000, tol=1e-11):
    """Iterate P <- Q + A'PA - A'PB (R + B'PB)^-1 B'PA until it settles."""
    P = [row[:] for row in Q]
    At, Bt = tr(Ad), tr(Bd)
    for k in range(iters):
        PA, PB = mul(P, Ad), mul(P, Bd)
        s = R + mul(Bt, PB)[0][0]                  # one input: the inverse is 1/s
        BtPA = mul(Bt, PA)                          # 1 x n
        Pn = add(add(Q, mul(At, PA)), scale(mul(mul(At, PB), BtPA), -1.0 / s))
        diff = max(abs(Pn[i][j] - P[i][j]) for i in range(4) for j in range(4))
        P = Pn
        if diff < tol * max(1.0, max(abs(v) for row in P for v in row)):
            break
    K = scale(mul(Bt, mul(P, Ad)), 1.0 / (R + mul(Bt, mul(P, Bd))[0][0]))[0]
    return K, P, k + 1


def eigenvalues(M):
    """Characteristic polynomial (Faddeev-LeVerrier), then its roots
    (Durand-Kerner).  Fine for a 4x4; not a general eigen-solver."""
    n = len(M)
    c = [1.0]                                       # z^n + c1 z^(n-1) + ... + cn
    Mk = zeros(n, n)
    for k in range(1, n + 1):
        Mk = add(mul(M, Mk), eye(n), c[-1]) if k > 1 else eye(n)
        AM = mul(M, Mk)
        c.append(-sum(AM[i][i] for i in range(n)) / k)
    roots = [complex(0.4, 0.9) ** i for i in range(n)]
    for _ in range(2000):
        new = []
        for i, z in enumerate(roots):
            num = sum(ci * z ** (n - j) for j, ci in enumerate(c))
            den = 1.0
            for j, w in enumerate(roots):
                if j != i:
                    den *= (z - w)
            new.append(z - num / den)
        if max(abs(a - b) for a, b in zip(new, roots)) < 1e-14:
            roots = new
            break
        roots = new
    return roots


def analyse_pid(p, gains):
    """Closed-loop eigenvalues of u = KP th + KI int(th) + KD th' - tilt only.
    The state grows a fifth member, the integral, z[k+1] = z[k] + dt th[k]."""
    kp, ki, kd = gains
    dt = p["CTRL_DT"]
    A, B = linear_model(p)
    Ad, Bd = discretise(A, B, dt)
    M = [row[:] + [0.0] for row in Ad] + [[0.0, 0.0, dt, 0.0, 1.0]]
    Bb = [[b[0]] for b in Bd] + [[0.0]]
    Acl = add(M, mul(Bb, [[0.0, 0.0, kp, kd, ki]]))          # u = +K x here
    ev = sorted(eigenvalues(Acl), key=abs, reverse=True)
    print(f"PID on tilt only: KP {kp:g}, KI {ki:g}, KD {kd:g}, every {dt * 1000:.0f} ms")
    print("closed-loop eigenvalues (state x, x', th, th', integral):")
    for z in ev:
        note = ""
        if abs(z) > 1.0 + 1e-4:
            note = f"   UNSTABLE: grows e-fold every {dt / math.log(abs(z)):.2f} s"
        elif abs(abs(z) - 1.0) < 1e-4 and abs(z.imag) < 1e-4:
            note = "   = 1: position - nothing pulls it back (drift)"
        elif abs(z.imag) > 1e-9:
            note = f"   slow swing, period {2 * math.pi * dt / abs(cmath.phase(z)):.1f} s"
        print(f"  {z.real:+.5f}{z.imag:+.5f}j   |z| = {abs(z):.5f}{note}")
    worst = max(abs(z) for z in ev)
    print("RESULT: " + ("unstable - it runs away" if worst > 1.0 + 1e-4
                        else "marginal at best: position is not controlled"))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default=os.path.join(HERE, "..", "params.h"))
    ap.add_argument("--Q", help="four weights: x, x_dot, theta, theta_dot")
    ap.add_argument("--R", type=float)
    ap.add_argument("--payload", type=float, default=0.0, help="design for this payload, kg")
    ap.add_argument("--check", help="control.c: exit 1 unless its LQR_K matches this design")
    ap.add_argument("--pid", help="KP,KI,KD: analyse PID on TILT ONLY instead (Slide 13)")
    a = ap.parse_args()

    p = read_params(a.params)
    if a.pid:
        return analyse_pid(p, [float(v) for v in a.pid.split(",")])
    Qd = [float(v) for v in a.Q.split(",")] if a.Q else \
         [p["LQR_QX"], p["LQR_QV"], p["LQR_QTH"], p["LQR_QW"]]
    R = a.R if a.R is not None else p["LQR_R"]
    dt = p["CTRL_DT"]
    Q = [[Qd[i] if i == j else 0.0 for j in range(4)] for i in range(4)]

    A, B = linear_model(p, a.payload)
    print(f"params.h: {len(p)} values read from {os.path.relpath(a.params)}")
    print(f"robot: body {p['BOT_MB'] + a.payload:.3f} kg, payload {a.payload:.3f} kg, "
          f"supply {p['VBAT']:.1f} V, controller every {dt * 1000:.1f} ms")
    print("\ncontinuous model  x' = A x + B u,   x = [x, x_dot, theta, theta_dot],  u = volts")
    for row, b in zip(A, B):
        print("  [" + " ".join(f"{v:10.4f}" for v in row) + f" ]   [{b[0]:9.4f} ]")
    ol = eigenvalues(A)
    unstable = max(z.real for z in ol)
    print("open-loop poles (1/s): " + ", ".join(f"{z.real:+.2f}{z.imag:+.2f}j" for z in ol))
    print(f"  the unstable one, {unstable:+.2f}/s: an uncontrolled tilt grows "
          f"e-fold every {1000 / unstable:.0f} ms")

    Ad, Bd = discretise(A, B, dt)
    K, P, n = dare(Ad, Bd, Q, R)
    print(f"\nQ = diag({', '.join(f'{v:g}' for v in Qd)}),  R = {R:g}")
    print(f"Riccati iteration converged in {n} steps")
    print("K = [ " + "  ".join(f"{k:+.4f}" for k in K) + " ]    u = -K x")
    print(f"  per unit:  {K[0]:+.1f} V per m,  {K[1]:+.2f} V per m/s,  "
          f"{K[2] * math.pi / 180:+.3f} V per degree,  {K[3] * math.pi / 180:+.4f} V per deg/s")

    # ---- does it stabilise the linear model? ----
    Acl = add(Ad, mul(Bd, [K]), -1.0)
    cl = eigenvalues(Acl)
    rho = max(abs(z) for z in cl)
    print("\nclosed-loop eigenvalues of Ad - Bd K (|z| < 1 is stable):")
    for z in sorted(cl, key=abs, reverse=True):
        tc = -dt / math.log(abs(z)) if 0 < abs(z) < 1 else float("inf")
        print(f"  {z.real:+.5f}{z.imag:+.5f}j   |z| = {abs(z):.5f}   time constant {tc * 1000:6.1f} ms")
    # an independent check on the eigenvalue code: ||Acl^k||^(1/k) -> rho
    M = eye(4)
    for _ in range(2000):
        M = mul(M, Acl)
    rho_pow = max(norm1(M), 1e-300) ** (1 / 2000)
    print(f"spectral radius {rho:.5f}  (power check ||Acl^2000||^(1/2000) = {rho_pow:.5f})")

    # ---- the linear model recovering from 6 degrees ----
    x = [[0.0], [0.0], [math.radians(6)], [0.0]]
    peak_u, settle = 0.0, None
    for k in range(int(5 / dt)):
        u = -sum(K[i] * x[i][0] for i in range(4))
        peak_u = max(peak_u, abs(u))
        x = add(mul(Ad, x), scale(Bd, u))
        if abs(x[2][0]) > math.radians(0.5) or abs(x[0][0]) > 0.01:
            settle = None
        elif settle is None:
            settle = (k + 1) * dt
    print(f"linear model from 6 degrees: peak |u| {peak_u:.2f} V "
          f"({'within' if peak_u <= p['VBAT'] else 'BEYOND'} the {p['VBAT']:.1f} V supply), "
          f"settled (|tilt| < 0.5 deg, |x| < 1 cm) after {settle if settle else float('nan'):.2f} s")

    line = "const float LQR_K[4] = { " + ", ".join(f"{k:.5g}f" for k in K) + " };"
    print("\npaste into control.c:\n  " + line)

    ok = rho < 1.0 and settle is not None
    print("\nRESULT: " + ("stabilising" if ok else "NOT STABILISING"))

    if a.check:
        src = open(a.check).read()
        m = re.search(r"LQR_K\[4\]\s*=\s*\{([^}]*)\}", src)
        if not m:
            print(f"--check: no LQR_K[4] = {{...}} in {a.check}")
            return 1
        have = [float(v.strip().rstrip("fF")) for v in m.group(1).split(",")]
        bad = [i for i in range(4) if abs(have[i] - K[i]) > 1e-3 * max(1.0, abs(K[i]))]
        print(f"--check {os.path.basename(a.check)}: firmware K = {have} -> "
              + ("MATCHES this design" if not bad else f"DIFFERS at index {bad}: re-paste"))
        ok = ok and not bad
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
