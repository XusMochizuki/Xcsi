"""Per-segment inter-chip phase offsets for a relock_test.py run.

    uv run --python 3.12 --with numpy research/scripts/relock_analysis.py runs/<relock run> [...]

A segment = frames whose (relock count of chip i, relock count of chip j) are constant
(flags bits 8..15). Per segment and pair: circular mean/std of the DC intercept b of
unwrap(angle(H_i conj H_j)) (see tools/phase_analysis.py), first --skip frames dropped.
Prints the per-segment means, the segment-to-segment spread, and the resultant length
R of the segment means (R ~ 1: repeatable offset, R ~ 1/sqrt(n): random).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from csi_frame import RELOCK_SHIFT, load  # noqa: E402
from phase_analysis import LABELS, VALID, circ_mean, circ_std, fit_line  # noqa: E402


def load_dev(path):
    hdrs, H = load(path)
    seq = np.array([h.tx_seq for h in hdrs])
    seq, idx = np.unique(seq, return_index=True)
    rl = np.array([(h.flags >> RELOCK_SHIFT) & 0xFF for h in hdrs])[idx]
    lo = np.array([h.lo_mhz for h in hdrs])[idx]
    rssi = np.array([h.rssi for h in hdrs])[idx]
    return dict(seq=seq, H=H[idx][:, VALID], rl=rl, lo=lo, rssi=rssi)


def analyse(run, skip, min_frames):
    devs = {}
    for f in sorted(glob.glob(os.path.join(run, "*.bin"))):
        key = os.path.basename(f)[:-4]
        devs[LABELS.get(key, key)] = load_dev(f)
    names = sorted(devs)
    ev = json.load(open(os.path.join(run, "events.json")))
    print(f"\n### {run}  args {ev['args']}")
    out = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            di, dj = devs[names[i]], devs[names[j]]
            common, ii, jj = np.intersect1d(di["seq"], dj["seq"], return_indices=True)
            X = di["H"][ii] * np.conj(dj["H"][jj])
            ok = (np.abs(X).mean(axis=1) > 0)
            _, b = fit_line(np.unwrap(np.angle(X), axis=1))
            key = di["rl"][ii].astype(int) * 256 + dj["rl"][jj]
            segs = []
            for k in np.unique(key):
                m = np.flatnonzero((key == k) & ok)[skip:]
                if len(m) < min_frames:
                    continue
                segs.append((int(k >> 8), int(k & 255), len(m), circ_mean(b[m]), circ_std(b[m]),
                             int(di["lo"][ii][m[0]]), int(dj["lo"][jj][m[0]])))
            means = np.array([s[3] for s in segs])
            R = np.abs(np.mean(np.exp(1j * means))) if len(means) else np.nan
            pair = f"{names[i]}-{names[j]}"
            print(f"{pair}: {len(segs)} segments, segment means R = {R:.3f} "
                  f"(random expectation ~{1 / np.sqrt(max(len(segs), 1)):.2f}), "
                  f"circ std of means {np.degrees(circ_std(means)):.1f} deg, "
                  f"median in-segment std {np.degrees(np.median([s[4] for s in segs])):.1f} deg")
            # Quantized-state test: if offsets take values theta0 + m*90deg, 4*theta is constant.
            z4 = np.mean(np.exp(4j * means)) if len(means) else np.nan
            res4 = np.angle(np.exp(1j * (means - np.angle(z4) / 4)) ** 4) / 4
            print(f"   mod 90: R4 = {np.abs(z4):.3f}, theta0 = {np.degrees(np.angle(z4) / 4):.1f} deg, "
                  f"residual rms {np.degrees(np.sqrt(np.mean(res4 ** 2))):.1f} deg, "
                  f"states {np.round(np.degrees(means - res4)).astype(int).tolist()}")
            print("   relock(i,j)  lo(i,j)    n   mean   std")
            for s in segs:
                print(f"   {s[0]:3d},{s[1]:3d}   {s[5]:4d},{s[6]:4d} {s[2]:5d} {np.degrees(s[3]):6.1f} {np.degrees(s[4]):5.1f}")
            out[pair] = dict(R=float(R), R4=float(np.abs(z4)), theta0_deg=float(np.degrees(np.angle(z4) / 4)),means_deg=[float(np.degrees(s[3])) for s in segs],
                             stds_deg=[float(np.degrees(s[4])) for s in segs], n=[s[2] for s in segs],
                             lo_ok=[bool(s[5] and s[6]) for s in segs])
    with open(os.path.join(run, "relock.json"), "w") as f:
        json.dump(out, f, indent=1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--skip", type=int, default=30)
    ap.add_argument("--min-frames", type=int, default=100)
    args = ap.parse_args()
    for r in args.runs:
        analyse(r, args.skip, args.min_frames)


if __name__ == "__main__":
    main()
