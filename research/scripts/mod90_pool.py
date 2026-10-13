"""Pool relock_analysis segment means (off-grid LO only) across runs/boots; test the 90-degree lattice.

    uv run --python 3.12 --with numpy research/scripts/mod90_pool.py RUN [RUN ...]

For every run: relock_analysis.analyse() -> segments whose LO is the retuned frequency
(lo != 0). Per run theta0 = angle(mean exp(4j*theta))/4; pooled over all runs: R4 and
the residual of every segment mean from the pooled lattice theta0 + m*90.
"""
import argparse
import contextlib
import io
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import relock_analysis as ra  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--skip-first", action="store_true", help="drop the first retuned segment of each run")
    args = ap.parse_args()
    pooled = {}
    for run in args.runs:
        with contextlib.redirect_stdout(io.StringIO()):
            out = ra.analyse(run, 30, 100)
        line = []
        for pair, v in out.items():
            m = np.radians(np.array(v["means_deg"]))
            lo_ok = np.array(v.get("lo_ok", [True] * len(m)))
            m = m[lo_ok]
            if args.skip_first:
                m = m[1:]
            pooled.setdefault(pair, []).append(m)
            t0 = np.degrees(np.angle(np.mean(np.exp(4j * m))) / 4)
            line.append(f"{pair} θ0 {t0:6.1f} (n={len(m)})")
        print(f"{os.path.basename(run.rstrip('/')):>10}: " + "   ".join(line))
    print()
    for pair, ms in pooled.items():
        allm = np.concatenate(ms)
        z4 = np.mean(np.exp(4j * allm))
        res = np.angle(np.exp(4j * allm) / np.exp(1j * np.angle(z4))) / 4
        per_run = np.array([np.angle(np.mean(np.exp(4j * m))) / 4 for m in ms])
        print(f"{pair}: pooled n={len(allm)} R4={np.abs(z4):.3f} θ0={np.degrees(np.angle(z4) / 4):.1f}°  "
              f"segment residual rms {np.degrees(np.sqrt(np.mean(res ** 2))):.1f}° max {np.degrees(np.abs(res).max()):.1f}°  "
              f"per-run θ0 - pooled θ0: {(np.degrees(np.angle(np.exp(4j * (per_run - np.angle(z4) / 4)))) / 4).round(1).tolist()} deg")


if __name__ == "__main__":
    main()
