# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial", "numpy"]
# ///
"""Test 3: is the lattice origin theta0 repeatable across reboots?

    uv run research/repro/t3_boots.py [--freq 2440] [--boots 5] [--relocks 3] [--k 4]
    uv run research/repro/t3_boots.py --analyze runs/<dir>

Each boot: reset TX and all RX, retune to --freq, then relock the receivers --relocks
times (every lock = one segment). Per boot and pair, theta0 = angle(mean exp(j k theta_s))/k
over that boot's segments; repeatable if every boot's theta0 is within 360/(4k) deg of
the pooled theta0 (22.5 deg for k = 4, i.e. the state m can never be confused).

The inter-chip phase includes the propagation channel: keep the TX and everything near
the antennas fixed. Per-boot RSSI / AGC / temperature are printed to spot a changed
scene. AGC-compensated values use a per-boot AGC phase table (common.agc_phase_table).
"""
import argparse
import glob
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
KINDS = {"seg", "end"}


def capture(args):
    b = c.load_bench(args.bench)
    out = args.out or c.default_out(f"t3-boots{args.freq}")
    for n in range(args.boots):
        with c.Session(b, os.path.join(out, f"boot{n:02d}"), reset=True) as s:
            if args.freq != 2412:
                s.retune(args.freq)
            s.mark("seg", label="lock0")
            time.sleep(args.dwell)
            for k in range(args.relocks):
                s.send_rx(b"r")
                s.mark("seg", label=f"lock{k + 1}")
                time.sleep(args.settle + args.dwell)
            s.mark("end")
        print(f"boot {n + 1}/{args.boots} done", flush=True)
    return out


def analyze(run, k):
    boots = sorted(glob.glob(os.path.join(run, "boot*")))
    per = {"raw": {}, "AGC-compensated": {}}
    ctx, il_all, intact = [], [], True
    for bd in boots:
        devs, ev = c.load_run(bd)
        il, ok = c.integrity(devs, os.path.basename(bd) + " ")
        il_all += il
        intact &= ok
        table = c.agc_phase_table(devs, ev["events"], KINDS)
        for variant, tbl in (("raw", None), ("AGC-compensated", table)):
            for pair, rows in c.segment_means(devs, ev["events"], KINDS, agc_table=tbl).items():
                per[variant].setdefault(pair, []).append(np.array([r[2] for r in rows]))
        ctx.append(os.path.basename(bd) + ": " + "; ".join(
            f"{n} rssi {d['rssi'].mean():.0f} agc {int(np.bincount(d['agc']).argmax())} T {d['temp'].max()}"
            for n, d in devs.items() if d is not None))
    tol = 360 / k / 4
    lines, result, ok_all = [f"run `{run}`, {len(boots)} boots, lattice k = {k} ({360 // k}°)\n"], {}, True
    for variant, pairs in per.items():
        lines += [f"\n## {variant}\n", "| pair | pooled θ0 | R_k pooled | per-boot θ0 − pooled (deg) | max |dev| | verdict |",
                  "|---|---|---|---|---|---|"]
        result[variant] = {}
        for pair, ms in pairs.items():
            allm = np.concatenate(ms)
            t0, _ = c.lattice_offset(allm, k)
            rk = c.lattice_stats(allm)[k]
            dev = [float(np.degrees(np.angle(np.exp(1j * k * (np.radians(c.lattice_offset(m, k)[0]) - np.radians(t0))))) / k)
                   for m in ms if len(m)]
            ok = max(map(abs, dev)) <= tol
            if variant == "AGC-compensated":
                ok_all &= ok
            lines.append(f"| {pair} | {t0:.1f}° | {rk:.2f} | {' '.join(f'{d:+.1f}' for d in dev)} | "
                         f"{max(map(abs, dev)):.1f}° | {'repeatable' if ok else 'NOT repeatable'} |")
            result[variant][pair] = dict(theta0_deg=t0, Rk=rk, per_boot_dev_deg=dev, pass_=bool(ok))
    lines += ["\n## per-boot context\n"] + [f"- {x}" for x in ctx]
    lines.append(f"\nCriterion: every boot's θ0 within ±{tol:.1f}° of the pooled θ0 (AGC-compensated).")
    lines += ["\n## data integrity\n"] + (il_all or ["- no host-side frame loss"])
    lines.append(f"**Result: {'θ0 repeatable across all boots' if ok_all else 'θ0 NOT repeatable for every pair'}"
                 f"{'' if intact else ' (DATA INVALID: frames lost between chip and host, rerun)'}**")
    ok_all &= intact
    result["data_intact"] = intact
    result["pass"] = bool(ok_all)
    c.write_report(run, "Test 3: reboot repeatability", lines, result)
    return ok_all


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", default=os.path.join(HERE, "bench.json"))
    ap.add_argument("--freq", type=int, default=2440)
    ap.add_argument("--boots", type=int, default=5)
    ap.add_argument("--relocks", type=int, default=3)
    ap.add_argument("--k", type=int, default=4, help="lattice order from test 2 (4 = 90 deg)")
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--out")
    ap.add_argument("--analyze", metavar="RUN_DIR")
    args = ap.parse_args()
    sys.exit(0 if analyze(args.analyze or capture(args), args.k) else 1)


if __name__ == "__main__":
    main()
