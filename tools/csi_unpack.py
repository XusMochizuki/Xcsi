"""Convert a recorded run (<MAC>.bin per receiver) into aligned numpy arrays.

    uv run --python 3.12 --with numpy tools/csi_unpack.py runs/NAME [--interp linear|none] [--span common|union]

Frames from all receivers are put on one grid indexed by the sender's tx_seq, so index t
is the same transmitted packet on every device. Frames missing on a device (lost over
the air or dropped locally) are filled according to --interp:

  linear (default)  amplitude: linear in time; phase: along the shorter arc between the
                    two neighbouring received frames; header fields: linear.
                    Before the first / after the last received frame: nearest value.
  none              NaN

valid.npy marks which entries were actually received. Note that the absolute phase of
each packet carries a random common term (CFO / timing), so interpolated absolute phase
is only a placeholder; inter-device phase differences and amplitude are what interpolate
meaningfully. Analyses that care should weight or drop frames with valid == False.

Writes into runs/NAME/npy/ (or --out):
  csi.npy      complex64 [dev, t, 256]   bin k = subcarrier k-128, raw int8 scale
  valid.npy    bool      [dev, t]
  tx_seq.npy   int64     [t]
  ts_us.npy    float64   [dev, t]        RX timestamp, 32-bit wrap removed, per-chip clock
  rssi.npy agc.npy fft_gain.npy temp_c.npy noise_floor.npy   float32 [dev, t]
  subcarrier.npy int16 [256], tone_mask.npy bool [256] (|k| in 2..122: HE20 tones, no DC)
  info.json    device order, labels, per-device counts, options
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_frame import load  # noqa: E402

FIELDS = ["rssi", "agc_gain", "fft_gain", "temp_c", "noise_floor"]
OUT_NAMES = {"agc_gain": "agc"}
N_SC = 256
# Fixed device order (board positions A, B, C from docs/hardware.md); others follow by MAC.
ORDER = ["98:88:E0:76:5D:68", "98:88:E0:76:07:C4", "98:88:E0:76:4D:04"]
LABELS = dict(zip(ORDER, ["A·COM17", "B·COM16", "C·COM15"]))


def read_device(path):
    """Return dict with tx_seq-sorted, de-duplicated frames of one receiver."""
    hdrs, csi = load(path)
    if not hdrs:
        return None
    seq = np.array([h.tx_seq for h in hdrs], np.int64)
    if np.any(np.diff(seq) < 0):
        print(f"warning: {os.path.basename(path)}: tx_seq goes backwards (TX reset / reorder); sorting", file=sys.stderr)
    seq, idx = np.unique(seq, return_index=True)            # keeps the first of duplicates
    if csi.shape[1] != N_SC:
        sys.exit(f"{path}: {csi.shape[1]} bins, expected {N_SC} (HE20 SU)")
    ts = np.array([h.timestamp_us for h in hdrs], np.int64)
    ts = ts + (np.cumsum(np.r_[0, np.diff(ts) < 0]) << 32)  # undo 32-bit wrap (in arrival order)
    return dict(seq=seq, csi=csi[idx], ts=ts[idx].astype(np.float64),
                **{f: np.array([getattr(h, f) for h in hdrs], np.float32)[idx] for f in FIELDS})


def neighbours(grid, have):
    """For each grid point: index of the received frame at/left of it, at/right of it, and weight."""
    r = np.searchsorted(have, grid)                         # first received >= grid
    l = np.clip(r - 1, 0, len(have) - 1)
    r = np.clip(r, 0, len(have) - 1)
    exact = have[r] == grid
    l = np.where(exact, r, l)
    span = (have[r] - have[l]).astype(np.float64)
    w = np.where(span > 0, (grid - have[l]) / np.where(span > 0, span, 1), 0.0)
    return l, r, np.clip(w, 0, 1)                           # edges clamp to nearest


def fill(grid, have, values, mode):
    """values: [n_have, ...] real. Linear fill onto grid (NaN when mode == 'none')."""
    if mode == "none":
        out = np.full((len(grid),) + values.shape[1:], np.nan, np.float64)
        out[np.isin(grid, have)] = values
        return out
    l, r, w = neighbours(grid, have)
    w = w.reshape((-1,) + (1,) * (values.ndim - 1))
    return (1 - w) * values[l] + w * values[r]


def fill_csi(grid, have, csi, mode):
    if mode == "none":
        out = np.full((len(grid), csi.shape[1]), np.nan + 1j * np.nan, np.complex64)
        out[np.isin(grid, have)] = csi
        return out
    l, r, w = neighbours(grid, have)
    w = w[:, None].astype(np.float32)
    amp = np.abs(csi)
    with np.errstate(invalid="ignore", divide="ignore"):
        u = np.where(amp > 0, csi / amp, 1).astype(np.complex64)
    a = (1 - w) * amp[l] + w * amp[r]
    p = (1 - w) * u[l] + w * u[r]                           # chord between unit phasors -> shorter arc
    pa = np.abs(p)
    p = np.where(pa > 1e-6, p / np.where(pa > 1e-6, pa, 1), u[l])
    return (a * p).astype(np.complex64)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="run directory containing <MAC>.bin files")
    ap.add_argument("--interp", choices=["linear", "none"], default="linear")
    ap.add_argument("--span", choices=["common", "union"], default="common",
                    help="common: tx_seq range covered by every device (default); union: all of it")
    ap.add_argument("--out", help="output directory (default RUN/npy)")
    args = ap.parse_args()

    meta = {}
    if os.path.exists(os.path.join(args.run, "meta.json")):
        with open(os.path.join(args.run, "meta.json")) as f:
            meta = json.load(f)
    devs = {}
    for path in sorted(glob.glob(os.path.join(args.run, "*.bin"))):
        name = os.path.basename(path)[:-4]
        if len(name) != 12:
            continue
        mac = ":".join(name[i:i + 2] for i in range(0, 12, 2))
        d = read_device(path)
        if d is None:
            print(f"{mac}: no frames, skipped", file=sys.stderr)
            continue
        devs[mac] = d
    if not devs:
        sys.exit(f"no <MAC>.bin with frames in {args.run}")
    macs = sorted(devs, key=lambda m: (ORDER.index(m) if m in ORDER else 99, m))

    if args.span == "common":
        lo, hi = max(devs[m]["seq"][0] for m in macs), min(devs[m]["seq"][-1] for m in macs)
    else:
        lo, hi = min(devs[m]["seq"][0] for m in macs), max(devs[m]["seq"][-1] for m in macs)
    if hi < lo:
        sys.exit(f"devices don't overlap in tx_seq ({lo} > {hi}); try --span union")
    grid = np.arange(lo, hi + 1, dtype=np.int64)
    T, D = len(grid), len(macs)
    print(f"tx_seq {lo}..{hi}: {T} packets x {D} devices, interp={args.interp}")

    csi = np.empty((D, T, N_SC), np.complex64)
    valid = np.zeros((D, T), bool)
    ts = np.empty((D, T))
    hdr = {f: np.empty((D, T), np.float32) for f in FIELDS}
    info = dict(run=os.path.abspath(args.run), interp=args.interp, span=args.span, tx_seq=[int(lo), int(hi)],
                devices=[], meta=meta)
    for i, m in enumerate(macs):
        d = devs[m]
        keep = (d["seq"] >= lo) & (d["seq"] <= hi)
        have = d["seq"][keep]
        valid[i] = np.isin(grid, have)
        csi[i] = fill_csi(grid, have, d["csi"][keep], args.interp)
        ts[i] = fill(grid, have, d["ts"][keep], args.interp)
        for f in FIELDS:
            hdr[f][i] = fill(grid, have, d[f][keep], args.interp)
        gaps = np.diff(np.r_[lo - 1, have, hi + 1]) - 1
        n = int(valid[i].sum())
        label = meta.get("devices", {}).get(m, {}).get("label") or LABELS.get(m, "")
        info["devices"].append(dict(mac=m, label=label, received=n, missing=T - n,
                                    loss_pct=round(100 * (T - n) / T, 3), longest_gap=int(gaps.max())))
        print(f"  {label or m:>10}  received {n:7d}  missing {T - n:6d} ({100 * (T - n) / T:5.2f}%)  "
              f"longest gap {gaps.max()}")
    info["all_received"] = int(valid.all(0).sum())
    print(f"  received by all: {info['all_received']} ({100 * info['all_received'] / T:.2f}%)")

    out = args.out or os.path.join(args.run, "npy")
    os.makedirs(out, exist_ok=True)
    sc = np.arange(N_SC, dtype=np.int16) - 128
    np.save(os.path.join(out, "csi.npy"), csi)
    np.save(os.path.join(out, "valid.npy"), valid)
    np.save(os.path.join(out, "tx_seq.npy"), grid)
    np.save(os.path.join(out, "ts_us.npy"), ts)
    for f in FIELDS:
        np.save(os.path.join(out, OUT_NAMES.get(f, f) + ".npy"), hdr[f])
    np.save(os.path.join(out, "subcarrier.npy"), sc)
    np.save(os.path.join(out, "tone_mask.npy"), (np.abs(sc) >= 2) & (np.abs(sc) <= 122))
    with open(os.path.join(out, "info.json"), "w") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    print(f"-> {out}")


if __name__ == "__main__":
    main()
