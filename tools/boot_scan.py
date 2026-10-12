"""Reboot the receivers N times and collect the inter-chip phase offset of each boot.

    uv run --python 3.12 --with numpy,matplotlib,pyserial tools/boot_scan.py --boots 20 --seconds 15 [--lock-gain]

Each boot: loss_test.py --reset (fresh PLL lock) -> phase_analysis.py -> phase.json.
Output: runs/<stamp>-bootscan/ with one sub-run per boot, plus bootscan.json and a table.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boots", type=int, default=20)
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--lock-gain", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", time.strftime("%Y%m%d-%H%M%S") + "-bootscan"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    per_boot = []
    for k in range(args.boots):
        d = os.path.join(args.out, f"boot{k:02d}")
        cmd = [sys.executable, os.path.join(HERE, "loss_test.py"), "--reset", "--seconds", str(args.seconds),
               "--out", d] + (["--lock-gain"] if args.lock_gain else [])
        r = subprocess.run(cmd, capture_output=True, text=True)
        if "ABORT" in r.stdout or r.returncode:
            print(f"boot{k:02d}: capture failed\n{r.stdout[-500:]}{r.stderr[-500:]}")
            continue
        subprocess.run([sys.executable, os.path.join(HERE, "phase_analysis.py"), d], capture_output=True, check=True)
        with open(os.path.join(d, "phase.json")) as f:
            ph = json.load(f)
        per_boot.append(ph)
        print(f"boot{k:02d}: " + "  ".join(f"{p} {v['mean_deg']:7.1f} (σ{v['std_deg']:4.1f})" for p, v in ph.items()),
              flush=True)

    with open(os.path.join(args.out, "bootscan.json"), "w") as f:
        json.dump(per_boot, f, indent=2)
    if not per_boot:
        return
    print(f"\n{len(per_boot)} boots")
    for pair in per_boot[0]:
        m = np.array([b[pair]["mean_deg"] for b in per_boot if pair in b])
        z = np.exp(1j * np.radians(m))
        r = np.abs(z.mean())
        print(f"{pair}: offsets sorted {np.sort(np.round(m)).astype(int).tolist()}")
        print(f"{'':>13} resultant length R = {r:.2f} (1 = identical every boot, ~0 = uniform)")


if __name__ == "__main__":
    main()
