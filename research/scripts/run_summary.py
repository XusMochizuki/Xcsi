"""Per-chip RSSI / AGC histogram / temperature of the retuned (lo_mhz != 0) frames of each run.

    uv run --python 3.12 --with numpy research/scripts/run_summary.py RUN [RUN ...]
"""
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from csi_frame import load  # noqa: E402
from phase_analysis import LABELS  # noqa: E402

for run in sys.argv[1:]:
    row = []
    for f in sorted(glob.glob(run + "/*.bin")):
        h = [x for x in load(f)[0] if x.lo_mhz]
        if not h:
            continue
        a = np.array([x.agc_gain for x in h])
        vals, c = np.unique(a, return_counts=True)
        top = vals[np.argsort(-c)][:3].tolist()
        row.append(f"{LABELS[os.path.basename(f)[:-4]]} rssi {np.mean([x.rssi for x in h]):5.1f} "
                   f"agc top {top} T {max(x.temp_c for x in h)}")
    print(f"{os.path.relpath(run, ROOT):>45}  " + " | ".join(row))
