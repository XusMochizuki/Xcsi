# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial", "numpy"]
# ///
"""Test 2: does an integer-N PLL frequency make the inter-chip offset discrete?

    uv run research/repro/t2_integer_n.py [--freqs 2412 2440 2442 2420] [--steps 10]
    uv run research/repro/t2_integer_n.py --analyze runs/<dir>

One boot (all boards reset first). For every frequency: TX and RX retune with F<MHz>
(the RX-only interval must lose the link, which proves the RX LO moved), then the
receivers relock --steps times. Per pair and frequency, the segment means theta_s are
tested for a lattice: R_k = |mean exp(j k theta_s)|. R_k ~ 1 means every lock lands on
theta0 + m*360/k.

With a 40 MHz crystal, 2440 MHz = 61 x 40 MHz (integer N) and 2420 MHz = 60.5 x 40 MHz;
the Wi-Fi channels (2412 = 60.3 x 40 MHz) and 2442 are fractional. On the original bench
2440 gave a 90-degree lattice (k = 4), 2420 a 45-degree lattice, 2412 / 2442 none.
Off-grid frequencies only work between our own TX and RX (both retuned); keep them in
the ISM band (2400-2483.5 MHz incl. 10 MHz half bandwidth).
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
R_LATTICE = 0.85            # R_k threshold for "on a 360/k grid"
CRYSTAL_MHZ = 40.0


def capture(args):
    b = c.load_bench(args.bench)
    out = args.out or c.default_out("t2-integer-n")
    with c.Session(b, out, reset=True) as s:
        for f in args.freqs:
            only_rx, after = s.retune(f)
            s.events[-1]["only_rx_frames"], s.events[-1]["after_frames"] = only_rx, after
            print(f"{f} MHz: frames with only RX retuned {only_rx}, after TX {after}", flush=True)
            for k in range(args.steps):
                s.send_rx(b"r")
                s.mark("seg", label=f"{f}:{k + 1}", mhz=f)
                time.sleep(args.settle + args.dwell)
            s.mark("end_freq", mhz=f)
    return out


def analyze(run):
    devs, ev = c.load_run(run)
    events = ev["events"]
    kinds = {"seg", "end_freq", "retune"}
    table = c.agc_phase_table(devs, events, kinds)
    freqs = [e["mhz"] for e in events if e["what"] == "retune"]
    lines, result = [f"run `{run}`\n"], {}
    for variant, tbl in (("raw", None), ("AGC-compensated", table)):
        seg = c.segment_means(devs, events, kinds, agc_table=tbl)
        lines += [f"\n## {variant}\n",
                  "| MHz | N = f/40 MHz | link lost with only RX retuned | pair | n | R1 | R2 | R4 | R8 | lattice |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
        res_v = result[variant] = {}
        prev = 2412                 # boot frequency
        for f in freqs:
            rt = next(e for e in events if e["what"] == "retune" and e["mhz"] == f)
            # a few frames still arrive before the command is applied; same frequency -> no check
            lost = "n/a" if f == prev else ("yes" if all(n <= 15 for n in rt.get("only_rx_frames", [0])) else "NO")
            prev = f
            res_v[f] = {}
            ks_all = []
            for pair, rows in seg.items():
                m = np.array([r[2] for r in rows if r[0].startswith(f"{f}:")])
                if len(m) < 4:
                    continue
                R = c.lattice_stats(m)
                k = next((k for k in (1, 2, 4, 8) if R[k] >= R_LATTICE), None)
                ks_all.append(k)
                t0, res = c.lattice_offset(m, k) if k else (float("nan"), np.array([np.nan]))
                res_v[f][pair] = dict(n=len(m), R=R, k=k, theta0_deg=t0,
                                      residual_rms_deg=float(np.sqrt(np.mean(res ** 2))),
                                      means_deg=np.degrees(m).tolist(),
                                      in_segment_std_deg=[float(np.degrees(r[3])) for r in rows if r[0].startswith(f"{f}:")])
                grid = (f"{360 // k}° grid, θ0 {t0:.1f}°, resid {np.sqrt(np.mean(res ** 2)):.1f}°" if k
                        else "none (continuous)")
                lines.append(f"| {f} | {f / CRYSTAL_MHZ:.3f} | {lost} | {pair} | {len(m)} | "
                             + " | ".join(f"{R[q]:.2f}" for q in (1, 2, 4, 8)) + f" | {grid} |")
            res_v[f]["lattice_k"] = ks_all[0] if ks_all and all(k == ks_all[0] for k in ks_all) else None
    lines.append("\n## AGC phase table used for compensation (deg, relative to each chip's most common code)\n")
    for chip, t in table.items():
        lines.append(f"- {chip}: " + (", ".join(f"{code}: {np.degrees(v):+.1f}" for code, v in sorted(t.items())) or "none"))
    result["agc_table_deg"] = {ch: {int(k): float(np.degrees(v)) for k, v in t.items()} for ch, t in table.items()}
    lines.append(f"\nLattice = smallest k in (1, 2, 4, 8) with R_k ≥ {R_LATTICE}; 'link lost' must be yes, "
                 "otherwise the retune did not take effect. The AGC table is estimated from this run "
                 "(segment means as reference), see common.agc_phase_table.")
    il, intact = c.integrity(devs)
    lines += ["\n## data integrity\n"] + (il or ["- no host-side frame loss"])
    result["data_intact"] = intact
    if not intact:
        lines.append("**DATA INVALID: frames lost between chip and host, rerun.**")
    for variant in ("raw", "AGC-compensated"):
        rv = result[variant]
        summary = ", ".join(f"{f} MHz: " + (f"{360 // rv[f]['lattice_k']}° lattice" if rv[f]["lattice_k"]
                                             else "no common lattice") for f in freqs)
        lines.append(f"**Result ({variant}): {summary}**")
    c.write_report(run, "Test 2: integer-N lattice", lines, result)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", default=os.path.join(HERE, "bench.json"))
    ap.add_argument("--freqs", type=int, nargs="+", default=[2412, 2440, 2442, 2420])
    ap.add_argument("--steps", type=int, default=10)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--out")
    ap.add_argument("--analyze", metavar="RUN_DIR")
    args = ap.parse_args()
    analyze(args.analyze or capture(args))


if __name__ == "__main__":
    main()
