"""Gain-dependent phase: per-frame pair intercept vs each chip's AGC code, lattice/segment removed.

    uv run --python 3.12 --with numpy research/scripts/agc_phase.py RUN [RUN ...]

Within each relock segment (constant LO state) the intercept b_ij is referenced to the
frames where both chips sit at their modal AGC code; the table gives the mean shift (deg)
per (agc_i - modal_i) step for chip i, holding chip j at its modal code, and vice versa.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from csi_frame import RELOCK_SHIFT, load  # noqa: E402
from phase_analysis import LABELS, VALID, fit_line  # noqa: E402


def load_dev(path):
    hdrs, H = load(path)
    seq, idx = np.unique([h.tx_seq for h in hdrs], return_index=True)
    g = lambda f: np.array([f(h) for h in hdrs])[idx]  # noqa: E731
    return dict(seq=seq, H=H[idx][:, VALID], rl=g(lambda h: (h.flags >> RELOCK_SHIFT) & 255),
                agc=g(lambda h: h.agc_gain).astype(int), lo=g(lambda h: h.lo_mhz))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    args = ap.parse_args()
    acc = {}
    for run in args.runs:
        devs = {LABELS.get(os.path.basename(f)[:-4]): load_dev(f) for f in sorted(glob.glob(run + "/*.bin"))}
        names = sorted(devs)
        for i in range(3):
            for j in range(i + 1, 3):
                di, dj = devs[names[i]], devs[names[j]]
                c, ii, jj = np.intersect1d(di["seq"], dj["seq"], return_indices=True)
                X = di["H"][ii] * np.conj(dj["H"][jj])
                _, b = fit_line(np.unwrap(np.angle(X), axis=1))
                key = di["rl"][ii] * 256 + dj["rl"][jj]
                gi, gj = di["agc"][ii], dj["agc"][jj]
                mi, mj = np.bincount(gi).argmax(), np.bincount(gj).argmax()
                for k in np.unique(key):
                    s = (key == k) & (di["lo"][ii] > 0)
                    ref = s & (gi == mi) & (gj == mj)
                    if ref.sum() < 30:
                        continue
                    z0 = np.mean(np.exp(1j * b[ref]))
                    for who, g, other, m_o, m_s in (("i", gi, gj, mj, mi), ("j", gj, gi, mi, mj)):
                        for d in range(-4, 5):
                            sel = s & (g == m_s + d) & (other == m_o)
                            if sel.sum() >= 10:
                                chip = names[i] if who == "i" else names[j]
                                sign = 1 if who == "i" else -1   # report as phase of the chip itself
                                acc.setdefault((chip, f"{names[i]}-{names[j]}"), {}).setdefault(d, []).append(
                                    (sel.sum(), sign * np.angle(np.mean(np.exp(1j * b[sel])) / z0)))
    print("phase shift of a chip's CSI vs its AGC code offset from modal (deg, frame-weighted), per pair:")
    for (chip, pair), dd in sorted(acc.items()):
        cells = []
        for d in sorted(dd):
            n = sum(x[0] for x in dd[d])
            v = np.angle(sum(x[0] * np.exp(1j * x[1]) for x in dd[d]))
            cells.append(f"{d:+d}: {np.degrees(v):6.1f} (n={n})")
        print(f"  {chip} in {pair}:  " + "  ".join(cells))


if __name__ == "__main__":
    main()
