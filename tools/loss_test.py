"""Record all receivers simultaneously and report packet loss per device.

    uv run --python 3.12 --with pyserial tools/loss_test.py [--seconds 60] [--out runs/NAME]

Loss is measured against the sender's tx_seq over the window that every receiver
covered (max first tx_seq .. min last tx_seq), so devices started a bit apart are
compared fairly. Per device:
  air_loss    tx_seq missing and not explained by local drops (lost over the air / not decoded)
  local_drop  frame_seq gaps: dropped on the device queue, or rejected here by CRC
  crc_err     frames rejected by CRC on the host
  all_rx      tx_seq received by every device (usable for cross-channel phase)
"""
import argparse
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial  # noqa: E402

from csi_frame import FLAG_GAIN_FORCED, Parser  # noqa: E402

RX_MACS = ["98:88:E0:76:4D:04", "98:88:E0:76:5D:68", "98:88:E0:76:07:C4"]
LABELS = {"98:88:E0:76:4D:04": "COM15", "98:88:E0:76:5D:68": "COM17", "98:88:E0:76:07:C4": "COM16"}


def dev_path(mac):
    return f"/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_{mac}-if00"


class Recorder(threading.Thread):
    def __init__(self, mac, stop, raw_path, reset=False):
        super().__init__(daemon=True)
        self.reset = reset
        self.serial = None
        self.mac, self.stop, self.raw_path = mac, stop, raw_path
        self.parser = Parser()
        self.tx_seqs = []           # received tx_seq, in arrival order
        self.forced = 0             # frames with FLAG_GAIN_FORCED
        self.local_drop = 0
        self.rssi = []
        self.per_sec = 0            # frames since last tick, read+reset by main thread
        self.per_sec_rssi = 0
        self.text = bytearray()
        self.error = None
        self.idle_s = 0

    def run(self):
        try:
            s = serial.Serial()
            s.port, s.timeout = dev_path(self.mac), 0.1
            # Leave DTR/RTS at the open() default (both asserted). Setting dtr=False first
            # passes through RTS=1/DTR=0, which is the USB-Serial-JTAG chip reset.
            s.open()
            self.serial = s
            if self.reset:
                s.dtr = False
                s.rts = True        # RTS=1, DTR=0: hold in reset
                time.sleep(0.1)
                s.rts = False       # release; DTR=0 -> normal boot
        except Exception as e:
            self.error = str(e)
            return
        raw = open(self.raw_path, "wb") if self.raw_path else None
        last_fs = None
        try:
            while not self.stop.is_set():
                data = s.read(65536)
                if raw:
                    raw.write(data)
                for ev in self.parser.feed(data):
                    if ev[0] != "frame":
                        self.text += ev[1]
                        continue
                    h = ev[1]
                    if last_fs is not None:
                        self.local_drop += (h.frame_seq - last_fs - 1) & 0xFFFFFFFF
                    last_fs = h.frame_seq
                    self.tx_seqs.append(h.tx_seq)
                    self.rssi.append(h.rssi)
                    self.forced += bool(h.flags & FLAG_GAIN_FORCED)
                    self.per_sec += 1
                    self.per_sec_rssi += h.rssi
        except Exception as e:      # e.g. USB detached mid-run
            self.error = str(e)
        finally:
            s.close()
            if raw:
                raw.close()


def longest_gap(sorted_seqs):
    gap = 0
    for a, b in zip(sorted_seqs, sorted_seqs[1:]):
        gap = max(gap, b - a - 1)
    return gap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--warmup", type=float, default=2, help="seconds discarded before recording")
    ap.add_argument("--out", help="directory for raw streams (<mac>.bin) and summary.json")
    ap.add_argument("--macs", nargs="+", default=RX_MACS)
    ap.add_argument("--lock-gain", action="store_true",
                    help="after warmup, send 'g': each RX locks its gain at its own median")
    ap.add_argument("--reset", action="store_true",
                    help="reset all receivers at start (fresh boot -> fresh PLL phase offsets)")
    args = ap.parse_args()

    if args.out:
        os.makedirs(args.out, exist_ok=True)
    stop = threading.Event()
    recs = []
    for mac in args.macs:
        raw = os.path.join(args.out, mac.replace(":", "") + ".bin") if args.out else None
        recs.append(Recorder(mac, stop, raw, args.reset))
    for r in recs:
        r.start()

    time.sleep(args.warmup)
    if args.lock_gain:
        for r in recs:
            if r.serial:
                r.serial.write(b"g")
        time.sleep(2)               # 100-frame baseline at 100 Hz, plus margin
    for r in recs:
        if r.error:
            sys.exit(f"{r.mac}: {r.error}")
        r.tx_seqs.clear()
        r.rssi.clear()
        r.local_drop = 0
        r.forced = 0
        r.parser.crc_errors = 0
        r.per_sec = 0
        r.per_sec_rssi = 0

    print("t[s]  " + "  ".join(f"{LABELS.get(r.mac, r.mac[-5:]):>11}" for r in recs) + "   (frames/s rssi)")
    t0 = time.monotonic()
    try:
        for t in range(1, int(args.seconds) + 1):
            time.sleep(max(0, t0 + t - time.monotonic()))
            counts, rssis = [], []
            for r in recs:
                counts.append(r.per_sec)
                rssis.append(r.per_sec_rssi / r.per_sec if r.per_sec else float("nan"))
                r.per_sec = r.per_sec_rssi = 0
            print(f"{t:4d}  " + "  ".join(f"{c:5d} {s:5.0f}" for c, s in zip(counts, rssis)), flush=True)
            idle = [r for r, c in zip(recs, counts) if c == 0]
            for r in idle:
                r.idle_s += 1
            for r in recs:
                if r not in idle:
                    r.idle_s = 0
            dead = [r for r in recs if r.error or r.idle_s >= 3]
            if dead:
                print("ABORT: no data from " + ", ".join(
                    f"{LABELS.get(r.mac, r.mac)}" + (f" ({r.error})" if r.error else "") for r in dead))
                break
    except KeyboardInterrupt:
        pass
    stop.set()
    for r in recs:
        r.join(timeout=2)
    elapsed = time.monotonic() - t0

    got = [r for r in recs if r.tx_seqs]
    if len(got) != len(recs):
        for r in recs:
            if not r.tx_seqs:
                print(f"{r.mac}: no frames received")
    if not got:
        sys.exit(1)
    lo = max(min(r.tx_seqs) for r in got)
    hi = min(max(r.tx_seqs) for r in got)
    expected = hi - lo + 1
    sets = {r.mac: {s for s in r.tx_seqs if lo <= s <= hi} for r in got}
    all_rx = len(set.intersection(*sets.values())) if len(sets) == len(recs) else 0

    print(f"\nwindow tx_seq {lo}..{hi}: {expected} sent in {elapsed:.1f} s ({expected / elapsed:.1f} Hz)")
    print(f"{'device':>8} {'mac':>17} {'recv':>6} {'loss%':>6} {'air':>6} {'local':>6} {'crc':>4} "
          f"{'maxgap':>6} {'rssi':>12}")
    summary = {"window": [lo, hi], "expected": expected, "elapsed_s": elapsed, "devices": {}}
    for r in recs:
        s = sets.get(r.mac, set())
        recv = len(s)
        dup = sum(1 for x in r.tx_seqs if lo <= x <= hi) - recv
        loss = expected - recv
        air = max(loss - r.local_drop, 0)
        rssi = (min(r.rssi), sum(r.rssi) / len(r.rssi), max(r.rssi)) if r.rssi else (0, 0, 0)
        gap = longest_gap(sorted(s))
        label = LABELS.get(r.mac, "")
        print(f"{label:>8} {r.mac:>17} {recv:6d} {100 * loss / expected:6.2f} {air:6d} {r.local_drop:6d} "
              f"{r.parser.crc_errors:4d} {gap:6d} {rssi[0]:4d}/{rssi[1]:.1f}/{rssi[2]}")
        if r.forced:
            print(f"{'':>8} gain locked on {100 * r.forced / max(len(r.tx_seqs), 1):.1f}% of frames")
        if dup:
            print(f"{'':>8} warning: {dup} duplicate tx_seq")
        summary["devices"][r.mac] = dict(label=label, recv=recv, loss=loss, air_loss=air,
                                         local_drop=r.local_drop, crc_err=r.parser.crc_errors,
                                         max_gap=gap, rssi_min=rssi[0], rssi_mean=rssi[1], rssi_max=rssi[2], gain_forced=r.forced,
                                         duplicates=dup)
    print(f"received by all {len(recs)}: {all_rx}/{expected} ({100 * all_rx / expected:.2f}%)")
    summary["all_rx"] = all_rx

    # Boot logs are expected; only surface warnings, errors and crashes.
    for r in recs:
        lines = [ln for ln in r.text.decode(errors="replace").splitlines()
                 if ln.startswith(("W (", "E (")) or "Guru" in ln or "abort" in ln]
        if lines:
            print(f"\n--- {LABELS.get(r.mac, r.mac)} warnings ---\n" + "\n".join(lines[-30:]))

    if args.out:
        with open(os.path.join(args.out, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
