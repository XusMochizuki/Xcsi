"""Print the 1-us power envelope and the 16-sample (L-STF) autocorrelation metric of one capture.

    uv run --python 3.12 --with numpy,scipy research/scripts/inspect_capture.py runs/<capture>.npz [--cap N]
Without --cap the capture with the strongest 2-us burst is shown.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dsp"))
import wifi_ofdm as w  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("capture")
ap.add_argument("--cap", type=int)
a = ap.parse_args()
d = np.load(a.capture)
iq = d["iq"].astype(complex)
x = iq - iq.mean(1, keepdims=True)
p = np.abs(x) ** 2
k = np.ones(160) / 160
if a.cap is None:
    r = [np.convolve(q, k, "valid").max() / np.median(np.convolve(q, k, "valid")) for q in p]
    a.cap = int(np.argmax(r))
q = p[a.cap][: len(p[a.cap]) // 80 * 80].reshape(-1, 80).mean(1)
print(f"capture {a.cap}: 1-us power envelope, dB re median")
print(" ".join(f"{v:.0f}" for v in 10 * np.log10(q / np.median(q))))
x20 = w.to_20msps(x[a.cap], float(d["fs"]))
lag = 16
P = np.convolve(x20[:-lag] * np.conj(x20[lag:]), np.ones(64), "valid")
R = np.convolve(np.abs(x20[lag:]) ** 2, np.ones(64), "valid")
M = np.abs(P) ** 2 / R ** 2
M = M[: len(M) // 20 * 20].reshape(-1, 20).max(1)
print("max L-STF metric M per 1 us")
print(" ".join(f"{v:.1f}" for v in M))
print("raw code range I", int(iq[a.cap].real.min()), int(iq[a.cap].real.max()), "Q", int(iq[a.cap].imag.min()), int(iq[a.cap].imag.max()))
