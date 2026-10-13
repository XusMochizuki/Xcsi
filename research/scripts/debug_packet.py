"""Per-symbol diagnostics for the strongest packets of a capture file: EVM / common phase of
L-SIG, RL-SIG, HE-SIG-A1/2 (all BPSK), with the L-LTF channel.

    uv run --python 3.12 --with numpy,scipy research/scripts/debug_packet.py runs/<capture>.npz [--n 5]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dsp"))
import wifi_ofdm as w  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("capture")
ap.add_argument("--n", type=int, default=5)
ap.add_argument("--min-snr", type=float, default=18)
a = ap.parse_args()
d = np.load(a.capture)
shown = 0
for i, c in enumerate(d["iq"]):
    x = w.to_20msps(np.conj(c - c.mean()), float(d["fs"]))     # C6 dump: inverted Q (see iq_to_csi.py)
    for st, cfo in w.detect_stf(x):
        xc = w.rotate(x, cfo, st)
        n = w.fine_timing(xc, st)
        if n < 40 or n + 720 > len(x):
            continue
        _, _, fine = w.ltf_csi(xc, n)
        xf = w.rotate(xc, fine, n)
        H, noise, _ = w.ltf_csi(xf, n)
        snr = 10 * np.log10(np.mean(np.abs(H) ** 2) / np.mean(noise))
        if snr < a.min_snr:
            continue
        row = []
        for name, off in (("L-SIG", 128), ("RL-SIG", 208), ("SIGA1", 288), ("SIGA2", 368)):
            s = w.legacy_symbol(xf, n + off, H)
            cpe = np.angle(np.sum(s ** 2)) / 2                     # BPSK common phase (mod pi)
            s2 = s * np.exp(-1j * cpe)
            evm = np.sqrt(np.mean((np.abs(s2.real) - 1) ** 2 + s2.imag ** 2))
            qb = np.mean(np.abs(s.imag) > np.abs(s.real))        # fraction on the Q axis
            row.append(f"{name} cpe {np.degrees(cpe):6.1f} evm {evm * 100:4.1f}% Qfrac {qb:.2f}")
        print(f"cap {i} n_ltf {n} snr {snr:.1f} cfo {(cfo + fine) / 1e3:.1f}k | " + " | ".join(row))
        shown += 1
    if shown >= a.n:
        break
