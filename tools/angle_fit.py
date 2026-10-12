"""Fit inter-chip phase differences against known TX angles (single boot, no reset between captures).

    uv run --python 3.12 --with numpy,matplotlib tools/angle_fit.py 0=runs/s/deg+00 30=runs/s/deg+30 -30=runs/s/deg-30 ...

The first argument is the calibration reference (broadside, 0 deg). For a collinear array with
spacing d = 61 mm (lambda/2 at 2.4 GHz), after subtracting the reference offset
    dphi_ij(theta) - dphi_ij(0) = k_ij * pi * sin(theta),   k_ij = (x_i - x_j) / d  (+-1 adjacent, +-2 outer)
k is fitted per pair by grid search on the circular residual; |k| ~ 2 marks the outer pair.
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
F_CH1 = 2.412e9
D = 0.061
K_NOMINAL = D / (3e8 / F_CH1 / 2)          # d / (lambda/2) on channel 1 (~0.98)


def wrap(x):
    return np.angle(np.exp(1j * x))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("points", nargs="+", help="ANGLE_DEG=RUN_DIR; first one is the 0-deg reference")
    args = ap.parse_args()

    pts = []
    for p in args.points:
        ang, run = p.split("=", 1)
        if not os.path.exists(os.path.join(run, "phase.json")):
            subprocess.run([sys.executable, os.path.join(HERE, "phase_analysis.py"), run],
                           capture_output=True, check=True)
        with open(os.path.join(run, "phase.json")) as f:
            pts.append((float(ang), json.load(f)))

    ref_ang, ref = pts[0]
    pairs = list(ref)
    th = np.radians([a for a, _ in pts])
    print(f"reference {ref_ang:+.0f} deg; expected slope per element step: {K_NOMINAL:.2f} * pi * sin(theta)\n")
    print(f"{'angle':>6} " + " ".join(f"{p:>22}" for p in pairs))
    rel = {}
    for p in pairs:
        rel[p] = wrap(np.radians([ph[p]["mean_deg"] - ref[p]["mean_deg"] for _, ph in pts]))
    for k, (a, ph) in enumerate(pts):
        print(f"{a:6.0f} " + " ".join(f"{np.degrees(rel[p][k]):9.1f} (σ{ph[p]['std_deg']:4.1f})      " for p in pairs))

    print("\nfit dphi_rel = k * pi * sin(theta):")
    for p in pairs:
        ks = np.linspace(-2.6, 2.6, 2601)
        cost = [np.sum(np.abs(wrap(rel[p] - k * np.pi * np.sin(th))) ** 2) for k in ks]
        kb = ks[int(np.argmin(cost))]
        res = np.degrees(wrap(rel[p] - kb * np.pi * np.sin(th)))
        print(f"  {p:>13}: k = {kb:+.2f}  (nominal +-{K_NOMINAL:.2f} adjacent / +-{2 * K_NOMINAL:.2f} outer)  "
              f"residual rms {np.sqrt(np.mean(res ** 2)):.1f} deg, max {np.max(np.abs(res)):.1f} deg")
        rel[p + "_k"] = kb


if __name__ == "__main__":
    main()
