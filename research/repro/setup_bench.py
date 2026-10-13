# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial", "numpy"]
# ///
"""Step 0: find the boards, write bench.json, check the firmware.

    uv run research/repro/setup_bench.py                       # list Espressif devices
    uv run research/repro/setup_bench.py --tx <serial> [--rx <serial> ...] [--names A B C]
                                                               # write bench.json (default: all others are RX)
    uv run research/repro/setup_bench.py --check               # verify frames + F/r command support

--check sends one 'r' (PLL relock at the current frequency) to every receiver and expects
the relock counter in the frame header to increment; this proves the firmware is the
csi_recv build with the PLL commands. TX and RX are reset first so all start at 2412 MHz.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as c  # noqa: E402
from csi_frame import RELOCK_SHIFT  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", default=os.path.join(HERE, "bench.json"))
    ap.add_argument("--tx")
    ap.add_argument("--rx", nargs="+")
    ap.add_argument("--names", nargs="+", help="receiver names in --rx order (default RX0, RX1, ...)")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    found = c.espressif_ports()
    print("Espressif USB devices:")
    for sn, dev in found:
        print(f"  {sn}  {dev}")
    if not found:
        print("  none (WSL: usbipd attach; Linux: check permissions on /dev/ttyACM*)")

    if args.tx:
        rx = args.rx or [sn for sn, _ in found if sn.lower() != args.tx.lower()]
        names = dict(zip(rx, args.names)) if args.names else {}
        bench = dict(tx=args.tx, rx=rx, names=names)
        with open(args.bench, "w") as f:
            json.dump(bench, f, indent=1)
        print(f"wrote {args.bench}: TX {args.tx}, RX {rx}")

    if args.check:
        b = c.load_bench(args.bench)
        out = c.default_out("check")
        with c.Session(b, out, reset=True) as s:
            time.sleep(2)
            n0 = s.frames()
            s.send_rx(b"r")
            time.sleep(2)
            n1 = s.frames()
            relock = {b["names"][r.sn]: ((r.hdrs[-1].flags >> RELOCK_SHIFT) & 255) if r.hdrs else None
                      for r in s.recs}
        ok = True
        for (name, rl), a, z in zip(relock.items(), n0, n1):
            rate = (z - a) / 2.0
            good = rl == 1 and rate > 20
            ok &= good
            print(f"  {name}: {rate:5.0f} frames/s, relock counter {rl}  {'OK' if good else 'FAIL'}")
        print("check passed" if ok else
              "check FAILED: frames missing (TX running? same channel?) or firmware without F/r commands")
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
