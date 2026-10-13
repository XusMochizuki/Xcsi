# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial", "numpy", "scipy"]
# ///
"""Test 4: raw IQ from ESP-SDR -> L-LTF / 2x HE-LTF CSI, and comparison with native CSI.

    uv run research/repro/t4_iq_csi.py --sdr <usb serial of the board running esp-sdr> \
        [--count 600] [--gain auto|N] [--tx-rate 1000] [--native runs/<dir>/A.bin]
    uv run research/repro/t4_iq_csi.py --analyze runs/<dir>/iq.npz [--native ...]

Needs one receiver flashed with ESP-SDR (github.com/ESPARGOS/esp-sdr, built with its
pinned ESP-IDF, see research/README_RESEARCH.md) and the TX running firmware/csi_send.
The TX is reset first (2412 MHz; retuned if --freq differs), its rate raised to --tx-rate during the capture (each IQ snapshot covers only
205 us) and set back to 100 Hz afterwards. Manual gain: hardware AGC clips the packet
start; --gain auto sweeps a few indices and keeps the best one that does not clip.

--native: a raw stream of NATIVE CSI from the same chip at the same position (e.g. the
A.bin of a test 1 run made before flashing ESP-SDR), for the quality comparison. The two
cannot be the same packets; record them back to back in a static scene.

Packets of our TX: L-SIG hard bits within 3 of the most common pattern (constant length).
Pass criterion for the reconstruction: >= 10 such packets and >= 90 % of them agree with
the majority HE-LTF sign pattern on >= 98 % of the 122 tones (single wrong decisions on
faded tones are expected at finite SNR; a wrong estimator gives ~50 % agreement).
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

for sub in ("capture", "scripts", "benchmarks", "dsp"):
    sys.path.insert(0, os.path.join(c.ROOT, "research", sub))
from csi_frame import load  # noqa: E402
from csi_quality import HE2X_4X, LEG_4X, metrics, shape_corr  # noqa: E402
from iq_to_csi import reconstruct, summary_lines  # noqa: E402
from sdr_capture import FS, Sdr  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def pick_gain(sdr, n, gains=(10, 20, 30, 40, 50, 60), shots=40):
    """Manual gain with the largest packet-to-noise ratio among gains that never clip.
    Per gain: 99th percentile / median of the 1-us power envelope, median over the
    snapshots that contain a burst (> 10 dB)."""
    best, best_r = gains[0], -1.0
    for g in gains:
        sdr.cmd(f"GAIN MANUAL {g}")
        ratios, clip = [], False
        for _ in range(shots):
            x, _ = sdr.capture(n)
            clip |= max(np.abs(x.real).max(), np.abs(x.imag).max()) >= 500
            p = np.abs(x - x.mean()) ** 2
            e = p[: len(p) // 80 * 80].reshape(-1, 80).mean(1)
            r = np.percentile(e, 99) / np.median(e)
            if r > 10:
                ratios.append(r)
        score = float(np.median(ratios)) if ratios else 0.0
        print(f"gain {g}: {'clips' if clip else 'no clipping'}, bursts {len(ratios)}/{shots}, "
              f"burst/noise {10 * np.log10(score) if score else float('nan'):.1f} dB")
        if not clip and score > best_r:
            best, best_r = g, score
    print(f"-> gain {best}")
    return best


def capture(args):
    b = c.load_bench(args.bench)
    out = args.out or c.default_out("t4-iq")
    os.makedirs(out, exist_ok=True)
    tx = c.open_port(b["tx"], reset=True)       # known state: 2412 MHz, 100 Hz (earlier tests may retune)
    time.sleep(4.0)
    if args.freq != 2412:
        tx.write(b"F%04d" % args.freq)
    tx.write(b"P%04d" % args.tx_rate)
    try:
        sdr = Sdr(c.port_of(args.sdr))
        sdr.cmd("SYNC 1")
        meta = {q: sdr.cmd(q) for q in ("VERSION?", "INFO", "LIMITS?")}
        meta["FREQ"] = sdr.cmd(f"FREQ {args.freq}")
        gain = pick_gain(sdr, args.n) if args.gain == "auto" else int(args.gain)
        meta["GAIN"] = sdr.cmd(f"GAIN MANUAL {gain}")
        if not meta["INFO"].startswith("C6SDR"):
            print(f"warning: INFO = {meta['INFO']!r} (expected an ESP32-C6 ESP-SDR)")
        iq = np.zeros((args.count, args.n), np.complex64)
        t0 = time.time()
        for k in range(args.count):
            iq[k], _ = sdr.capture(args.n)
        dt = time.time() - t0
        sdr.cmd("RELEASE")
    finally:
        tx.write(b"P0100")
        tx.close()
    path = os.path.join(out, "iq.npz")
    np.savez_compressed(path, iq=iq, fs=FS, freq_mhz=args.freq, gain=gain, seconds=dt,
                        tx_rate=args.tx_rate, meta=np.array([f"{k}: {v}" for k, v in meta.items()]))
    print(f"{args.count} captures in {dt:.1f} s -> {path}")
    return path


def analyze(path, native, min_snr):
    d = np.load(path)
    r = reconstruct(d["iq"], float(d["fs"]), min_snr)
    run = os.path.dirname(path) or "."
    tx_rate = int(d["tx_rate"]) if "tx_rate" in d else None          # older captures lack these
    seconds = float(d["seconds"]) if "seconds" in d else None
    lines = [f"capture `{path}`: {len(d['iq'])} snapshots of {d['iq'].shape[1]} samples at "
             f"{float(d['fs']) / 1e6:.0f} MS/s, gain {d['gain']}, TX {tx_rate} Hz, {seconds} s\n"]
    lines += [f"- {x}" for x in summary_lines(r)]
    result = {k: (v.tolist() if isinstance(v, np.ndarray) and v.ndim == 1 and v.dtype != complex else v)
              for k, v in r.items() if not (isinstance(v, np.ndarray) and v.dtype.kind == "c")}
    ok = False
    if "H_he" in r:
        n = len(r["snr_db"])
        ok = n >= 10 and np.mean(r["he_agree"] >= 0.98) >= 0.90
        if seconds:
            lines.append(f"- yield: {n / seconds:.2f} usable packets/s at TX {tx_rate} Hz")
        rows = {"IQ L-LTF (52 tones)": metrics(r["H_lltf"], LEG_4X),
                "IQ 2x HE-LTF (122 tones)": metrics(r["H_he"], HE2X_4X)}
        if native:
            hdrs, H = load(native)
            Hn = H[np.array([h.bb_format == 4 and h.csi_len == 512 for h in hdrs])].astype(complex)
            rows["native HE CSI, measured even tones"] = metrics(Hn[:, 128 + HE2X_4X], HE2X_4X)
        lines += ["\n| set | n | residual | phase std | amp CV |", "|---|---|---|---|---|"]
        for name, (m, _) in rows.items():
            lines.append(f"| {name} | {m['n']} | {m['resid_db']:.1f} dB | {m['phase_std_deg']:.2f}° | {m['amp_cv']:.3f} |")
        if native:
            corr = shape_corr(rows["IQ 2x HE-LTF (122 tones)"][1], rows["native HE CSI, measured even tones"][1])
            lines.append(f"\nmean channel shape correlation IQ HE-LTF vs native: {corr:.3f} (1 = same channel)")
            result["shape_corr"] = corr
        result["quality"] = {k: v[0] for k, v in rows.items()}
    lines.append(f"\n**Result: {'IQ -> CSI reconstruction valid' if ok else 'reconstruction NOT validated'}**")
    result["pass"] = bool(ok)
    c.write_report(run, "Test 4: IQ -> CSI", lines, result)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", default=os.path.join(HERE, "bench.json"))
    ap.add_argument("--sdr", help="USB serial of the ESP-SDR board")
    ap.add_argument("--count", type=int, default=600)
    ap.add_argument("--n", type=int, default=16380)
    ap.add_argument("--gain", default="auto", help="manual gain index, or 'auto' (sweep, no clipping)")
    ap.add_argument("--freq", type=int, default=2412)
    ap.add_argument("--tx-rate", type=int, default=1000)
    ap.add_argument("--min-snr", type=float, default=15.0)
    ap.add_argument("--native", help="raw native CSI stream (.bin) from the same chip/position")
    ap.add_argument("--out")
    ap.add_argument("--analyze", metavar="IQ_NPZ")
    args = ap.parse_args()
    if not args.analyze and not args.sdr:
        ap.error("--sdr is required for a capture")
    sys.exit(0 if analyze(args.analyze or capture(args), args.native, args.min_snr) else 1)


if __name__ == "__main__":
    main()
