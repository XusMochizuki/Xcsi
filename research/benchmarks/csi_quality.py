"""Static-channel CSI quality: IQ-reconstructed (ESP-SDR) vs native ESP-IDF CSI, same chip/position.

    uv run --python 3.12 --with numpy research/benchmarks/csi_quality.py \
        --iq runs/<capture>.csi.npz --native runs/<loss_test dir> [--mac 9888E0765D68] [--json out.json]

The two sets cannot come from the same packets (one firmware at a time), so they are
recorded back to back in a static scene; differences in the mean channel shape are
reported to show whether the scene stayed put.

Per packet: amplitude normalised to unit mean |H|, the best-fit linear phase (timing) and
common phase removed. Then, against the across-packet mean template:
  resid_db   median over packets of |H - mean|^2 / |mean|^2   (lower = cleaner; ~ -SNR_csi)
  phase_std  median over tones of the circular std of phase across packets [deg]
  amp_cv     median over tones of std/mean of |H| across packets
Native CSI is compared on the tones actually measured by the 2x HE-LTF (even tones
+-2..+-122, bins 128+tone); the odd native bins are interpolated by the chip.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "research", "dsp"))
from csi_frame import load  # noqa: E402

HE2X_4X = np.array([2 * q for q in range(-61, 62) if q != 0])
LEG_4X = np.array([4 * m for m in range(-26, 27) if m != 0])


def normalise(H, tones):
    H = H / np.mean(np.abs(H), axis=1, keepdims=True)
    ph = np.unwrap(np.angle(H), axis=1)
    A = np.vstack([tones, np.ones_like(tones)]).T.astype(float)
    coef, *_ = np.linalg.lstsq(A, ph.T, rcond=None)
    return H * np.exp(-1j * (A @ coef).T)


def metrics(H, tones):
    H = H[np.all(np.abs(H) > 0, axis=1)]            # drop frames with empty (zero) tones
    Hn = normalise(H, tones)
    # re-reference the common phase to the template, then recompute the template
    tpl = np.mean(Hn, axis=0)
    Hn = Hn * np.exp(-1j * np.angle(np.sum(Hn * np.conj(tpl), axis=1)))[:, None]
    tpl = np.mean(Hn, axis=0)
    resid = np.sum(np.abs(Hn - tpl) ** 2, axis=1) / np.sum(np.abs(tpl) ** 2)
    z = np.mean(Hn / np.abs(Hn), axis=0)
    pstd = np.degrees(np.sqrt(-2 * np.log(np.clip(np.abs(z), 1e-9, 1))))
    a = np.abs(Hn)
    return dict(n=int(len(H)), resid_db=float(10 * np.log10(np.median(resid))),
                phase_std_deg=float(np.median(pstd)), amp_cv=float(np.median(a.std(0) / a.mean(0)))), tpl


def shape_corr(t1, t2):
    return float(np.abs(np.vdot(t1, t2)) / np.linalg.norm(t1) / np.linalg.norm(t2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iq", required=True)
    ap.add_argument("--native", required=True)
    ap.add_argument("--mac", default="9888E0765D68")
    ap.add_argument("--json")
    args = ap.parse_args()

    q = np.load(args.iq)
    hdrs, Hn_all = load(glob.glob(os.path.join(args.native, args.mac + "*.bin"))[0])
    he = np.array([h.bb_format == 4 for h in hdrs])
    Hnat = Hn_all[he].astype(complex)
    rows = {}
    rows["IQ L-LTF (52 tones, 312.5 kHz)"], t_leg = metrics(q["H_lltf"], LEG_4X)
    rows["IQ 2x HE-LTF (122 tones)"], t_he = metrics(q["H_he"], HE2X_4X)
    rows["native HE CSI, measured even tones (122)"], t_nat = metrics(Hnat[:, 128 + HE2X_4X], HE2X_4X)
    allk = np.array([k for k in range(-122, 123) if abs(k) >= 2])
    rows["native HE CSI, all bins +-2..122 (242)"], _ = metrics(Hnat[:, 128 + allk], allk)
    print(f"{'set':45} {'n':>5} {'resid dB':>9} {'phase std':>10} {'amp CV':>7}")
    for k, v in rows.items():
        print(f"{k:45} {v['n']:5d} {v['resid_db']:9.1f} {v['phase_std_deg']:9.2f}° {v['amp_cv']:7.3f}")
    sc = shape_corr(t_he, t_nat)
    # L-LTF tones vs native at the same frequencies (4x tone 4m)
    _, t_nat_leg = metrics(Hnat[:, 128 + LEG_4X], LEG_4X)
    sc_leg = shape_corr(t_leg, t_nat_leg)
    print(f"\nmean-channel shape correlation |<a,b>|/|a||b| (1 = identical shape, slope/phase removed):"
          f"\n  IQ HE-LTF vs native (even tones) {sc:.3f};  IQ L-LTF vs native at the same frequencies {sc_leg:.3f}")
    if args.json:
        json.dump(dict(rows=rows, shape_corr_he=sc, shape_corr_leg=sc_leg), open(args.json, "w"), indent=1)


if __name__ == "__main__":
    main()
