"""Inter-chip CSI phase offset analysis for a loss_test.py run directory.

    uv run --python 3.12 --with numpy,matplotlib tools/phase_analysis.py runs/<name> [--png out.png]

For each chip pair (i, j) and each tx_seq received by both:
    X_k = H_i[k] * conj(H_j[k])          # common TX phase / CFO cancels (shared crystal)
    unwrap(angle(X_k)) ~= a * k + b       # least squares over valid subcarriers
  b : phase offset at DC [deg]  (PLL offset + geometry + gain-dependent phase)
  a : relative timing slope -> delta_t = a / (2*pi*delta_f) [ns]  (packet-detection offset)
Also reports the single-chip intercept, which should be random (no shared phase with TX).
"""
import argparse
import json
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_frame import load  # noqa: E402

LABELS = {"9888E0764D04": "COM15", "9888E0765D68": "COM17", "9888E07607C4": "COM16"}
DF = 78.125e3                                   # HE20 subcarrier spacing
SC = np.arange(256) - 128                       # bin k <-> subcarrier k-128
VALID = (np.abs(SC) >= 2) & (np.abs(SC) <= 122) # HE20 242 tones; drops DC spike at -1
K = SC[VALID].astype(np.float64)


def circ_mean(x):
    return np.angle(np.mean(np.exp(1j * x)))


def circ_std(x):
    r = np.abs(np.mean(np.exp(1j * x)))
    return np.sqrt(-2 * np.log(max(r, 1e-12)))


def fit_line(phase):
    """phase: [n, k] unwrapped along k. Returns slope a [rad/subcarrier], intercept b [rad]."""
    A = np.vstack([K, np.ones_like(K)]).T
    coef, *_ = np.linalg.lstsq(A, phase.T, rcond=None)
    return coef[0], coef[1]


def load_dev(path):
    hdrs, H = load(path)
    seq = np.array([h.tx_seq for h in hdrs])
    agc = np.array([h.agc_gain for h in hdrs])
    fft = np.array([h.fft_gain for h in hdrs])
    ts = np.array([h.timestamp_us for h in hdrs], dtype=np.float64)
    temp = np.array([h.temp_c for h in hdrs], dtype=np.float64)
    seq, idx = np.unique(seq, return_index=True)
    return dict(seq=seq, H=H[idx][:, VALID], agc=agc[idx], fft=fft[idx], ts=ts[idx], temp=temp[idx])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--png", help="write figure here (default: <run>/phase.png)")
    args = ap.parse_args()

    devs = {}
    for f in sorted(glob.glob(os.path.join(args.run, "*.bin"))):
        key = os.path.basename(f)[:-4]
        d = load_dev(f)
        if len(d["seq"]):
            devs[LABELS.get(key, key)] = d
    names = sorted(devs)
    print(f"run {args.run}: " + ", ".join(f"{n} {len(devs[n]['seq'])} frames" for n in names))

    print("\n== single chip vs TX (expect random intercept)")
    for n in names:
        d = devs[n]
        a, b = fit_line(np.unwrap(np.angle(d["H"]), axis=1))
        print(f"{n}: intercept circ_std {np.degrees(circ_std(b)):6.1f} deg   "
              f"agc {sorted(set(d['agc'].tolist()))}  fft {sorted(set(d['fft'].tolist()))}")

    results = {}
    summary = {}
    print("\n== chip pairs")
    print(f"{'pair':>13} {'n':>6} {'b mean':>8} {'b std':>7} {'b p2p':>7} {'drift/10s':>9} "
          f"{'dt mean':>8} {'dt std':>7}   (deg, deg, deg, deg, ns, ns)")
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            di, dj = devs[names[i]], devs[names[j]]
            common, ii, jj = np.intersect1d(di["seq"], dj["seq"], return_indices=True)
            X = di["H"][ii] * np.conj(dj["H"][jj])
            a, b = fit_line(np.unwrap(np.angle(X), axis=1))
            b = np.angle(np.exp(1j * b))
            bm = circ_mean(b)
            bc = np.angle(np.exp(1j * (b - bm)))            # centred, for linear stats
            dt = a / (2 * np.pi * DF) * 1e9
            t = (common - common[0]) / 100.0                # tx at 100 Hz -> seconds
            # drift: circular mean per 10 s block
            blocks = [circ_mean(b[(t >= s) & (t < s + 10)]) for s in np.arange(0, t[-1] - 10, 10)]
            drift = np.ptp(np.unwrap(blocks)) if len(blocks) > 1 else 0.0
            pair = f"{names[i]}-{names[j]}"
            print(f"{pair:>13} {len(common):6d} {np.degrees(bm):8.1f} {np.degrees(circ_std(b)):7.2f} "
                  f"{np.degrees(np.ptp(bc)):7.1f} {np.degrees(drift):9.2f} {dt.mean():8.2f} {dt.std():7.2f}")
            summary[pair] = dict(n=int(len(common)), mean_deg=float(np.degrees(bm)),
                                 std_deg=float(np.degrees(circ_std(b))), drift10s_deg=float(np.degrees(drift)),
                                 dt_mean_ns=float(dt.mean()))
            results[pair] = dict(t=t, b=b, bc=bc, dt=dt, X=X,
                                 temp=(di["temp"][ii], dj["temp"][jj]),
                                 dagc=di["agc"][ii].astype(int) - dj["agc"][jj].astype(int),
                                 agc=(di["agc"][ii], dj["agc"][jj]))

    # Long runs: 60 s block means vs chip temperature (temp_c = 0 means not yet read).
    first = next(iter(results.values()), None)
    if first is not None and first["t"][-1] > 300:
        print("\n== drift: 60 s block means (deg) and chip temperatures (degC)")
        print("   t[s]  " + "  ".join(f"{p:>13}" for p in results) + "   temps")
        edges = np.arange(0, first["t"][-1], 60)
        tbl = {p: [] for p in results}
        for s in edges:
            row, temps = [], {}
            for p, r in results.items():
                m = (r["t"] >= s) & (r["t"] < s + 60)
                if m.sum() < 100:
                    row.append(f"{'.':>13}")
                    continue
                v = circ_mean(r["b"][m])
                tbl[p].append((s, v, r["temp"][0][m].mean(), r["temp"][1][m].mean()))
                row.append(f"{np.degrees(v):13.1f}")
                ni, nj = p.split("-")
                temps[ni], temps[nj] = r["temp"][0][m].mean(), r["temp"][1][m].mean()
            print(f"  {s:5.0f}  " + "  ".join(row) + "   " + " ".join(f"{n} {v:.1f}" for n, v in sorted(temps.items())))
        for p, rows in tbl.items():
            if len(rows) < 3:
                continue
            a = np.array(rows)
            v = np.degrees(np.unwrap(a[:, 1]))
            dtemp = a[:, 2] - a[:, 3]
            mtemp = (a[:, 2] + a[:, 3]) / 2
            print(f"{p}: block range {np.ptp(v):.1f} deg; corr with mean temp {np.corrcoef(v, mtemp)[0, 1]:+.2f}, "
                  f"with temp diff {np.corrcoef(v, dtemp)[0, 1]:+.2f}  (temp ranges: mean {np.ptp(mtemp):.1f}, diff {np.ptp(dtemp):.1f} degC)")

    print("\n== intercept vs AGC difference (agc_i - agc_j)")
    for pair, r in results.items():
        line = []
        for g in np.unique(r["dagc"]):
            m = r["dagc"] == g
            if m.sum() >= 20:
                line.append(f"{g:+d}: {np.degrees(circ_mean(r['b'][m])):7.1f} (n={m.sum()})")
        print(f"{pair:>13}  " + "   ".join(line))

    print("\n== intercept vs each chip's own AGC (rows: agc_i, cols: agc_j, mean deg, n>=20)")
    for pair, r in results.items():
        gi, gj = r["agc"]
        ui, uj = np.unique(gi), np.unique(gj)
        print(f"{pair}: cols {uj.tolist()}")
        for a_ in ui:
            cells = []
            for b_ in uj:
                m = (gi == a_) & (gj == b_)
                cells.append(f"{np.degrees(circ_mean(r['b'][m])):6.1f}" if m.sum() >= 20 else "     .")
            print(f"  {a_:3d} " + " ".join(cells))

    # Figure
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(len(results), 3, figsize=(15, 3.2 * len(results)), squeeze=False)
    for row, (pair, r) in enumerate(results.items()):
        ax[row, 0].plot(r["t"], np.degrees(r["b"]), ".", ms=1)
        ax[row, 0].set_title(f"{pair}: intercept b over time")
        ax[row, 0].set_xlabel("s"); ax[row, 0].set_ylabel("deg")
        ax[row, 1].plot(r["t"], r["dt"], ".", ms=1)
        ax[row, 1].set_title(f"{pair}: relative timing (slope)")
        ax[row, 1].set_xlabel("s"); ax[row, 1].set_ylabel("ns")
        mean_x = np.angle(np.mean(r["X"] / np.abs(r["X"] + 1e-9), axis=0))
        ax[row, 2].plot(K, np.degrees(np.unwrap(mean_x)), lw=1)
        ax[row, 2].set_title(f"{pair}: mean phase diff per subcarrier")
        ax[row, 2].set_xlabel("subcarrier"); ax[row, 2].set_ylabel("deg")
    fig.tight_layout()
    out = args.png or os.path.join(args.run, "phase.png")
    fig.savefig(out, dpi=90)
    print(f"\nfigure: {out}")
    with open(os.path.join(args.run, "phase.json"), "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
