# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial", "numpy"]
# ///
"""Test 1: is the inter-chip CSI phase offset set by the RX RF PLL lock?

    uv run research/repro/t1_relock.py [--freq 2412] [--rx-steps 8] [--tx-steps 4]
    uv run research/repro/t1_relock.py --analyze runs/<dir>          # re-run analysis only

Sequence (one boot, all boards reset first): baseline, then the receivers relock their
PLL --rx-steps times ('r' to every RX), then only the transmitter relocks --tx-steps
times (control). Per chip pair, the circular mean of the DC intercept of
angle(H_i conj H_j) is taken in every segment.

Expected if the offset comes from the RX PLL lock:
  RX relocks : segment means spread over the circle (circ std of means > 30 deg)
  TX relocks : no change (every segment within 15 deg of the reference segment), because
               the TX phase is common to all receivers and cancels in H_i conj H_j.
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RX_SPREAD_MIN = 30.0        # deg, circ std of RX-relock segment means
TX_DEV_MAX = 15.0           # deg, max deviation of TX-relock segments from reference


def capture(args):
    b = c.load_bench(args.bench)
    out = args.out or c.default_out(f"t1-relock{args.freq}")
    with c.Session(b, out, reset=True) as s:
        if args.freq != 2412:
            only_rx, after = s.retune(args.freq)
            print(f"retune {args.freq}: frames with only RX moved {only_rx}, after TX {after}")
        s.mark("seg", label="baseline")
        time.sleep(args.dwell)
        for k in range(args.rx_steps):
            s.send_rx(b"r")
            s.mark("seg", label=f"rx{k + 1}")
            time.sleep(args.settle + args.dwell)
            print(f"RX relock {k + 1}/{args.rx_steps}, frames {s.frames()}", flush=True)
        for k in range(args.tx_steps):
            s.send_tx(b"r")
            s.mark("seg", label=f"tx{k + 1}")
            time.sleep(args.settle + args.dwell)
            print(f"TX relock {k + 1}/{args.tx_steps}, frames {s.frames()}", flush=True)
    return out


def analyze(run):
    devs, ev = c.load_run(run)
    seg = c.segment_means(devs, ev["events"], {"seg"})
    lines, result, ok_all = [], {}, True
    lines.append(f"run `{run}`\n")
    lines.append("| pair | RX-relock segment means (deg) | circ std | TX-relock deviation from last RX segment (deg) | verdict |")
    lines.append("|---|---|---|---|---|")
    for pair, rows in seg.items():
        rx = [r for r in rows if r[0] == "baseline" or r[0].startswith("rx")]
        tx = [r for r in rows if r[0].startswith("tx")]
        rx_means = np.array([r[2] for r in rx])
        spread = np.degrees(c.circ_std(rx_means)) if len(rx) > 2 else float("nan")
        ref = rx[-1][2] if rx else 0.0
        dev = [float(np.degrees(np.angle(np.exp(1j * (r[2] - ref))))) for r in tx]
        ok = spread > RX_SPREAD_MIN and len(dev) > 0 and max(map(abs, dev)) < TX_DEV_MAX
        ok_all &= ok
        lines.append(f"| {pair} | {' '.join(f'{np.degrees(m):.0f}' for m in rx_means)} | {spread:.0f}° | "
                     f"{' '.join(f'{d:+.1f}' for d in dev)} | {'PLL origin' if ok else 'not shown'} |")
        result[pair] = dict(rx_means_deg=np.degrees(rx_means).tolist(), rx_spread_deg=spread,
                            tx_dev_deg=dev, in_segment_std_deg=[float(np.degrees(r[3])) for r in rows],
                            pass_=bool(ok))
    lines.append(f"\nCriteria: RX-relock circ std > {RX_SPREAD_MIN:.0f}°, every TX-relock segment within "
                 f"±{TX_DEV_MAX:.0f}° of the last RX segment.")
    il, intact = c.integrity(devs)
    lines += ["\n## data integrity\n"] + (il or ["- no host-side frame loss"])
    ok_all &= intact
    lines.append(f"**Result: {'offset is set by the RX PLL lock (all pairs)' if ok_all else 'NOT confirmed for every pair'}"
                 f"{'' if intact else ' (data invalid, rerun)'}**")
    result["pass"] = bool(ok_all)
    c.write_report(run, "Test 1: PLL relock", lines, result)
    return ok_all


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", default=os.path.join(HERE, "bench.json"))
    ap.add_argument("--freq", type=int, default=2412)
    ap.add_argument("--rx-steps", type=int, default=8)
    ap.add_argument("--tx-steps", type=int, default=4)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--out")
    ap.add_argument("--analyze", metavar="RUN_DIR")
    args = ap.parse_args()
    run = args.analyze or capture(args)
    sys.exit(0 if analyze(run) else 1)


if __name__ == "__main__":
    main()
