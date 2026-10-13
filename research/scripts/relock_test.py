"""PLL relock / retune experiment: is the inter-chip CSI phase offset set by the RF PLL lock?

    uv run --python 3.12 --with pyserial,numpy research/scripts/relock_test.py \
        --freq 2412 --steps 12 [--target rx|tx|rx0] [--out runs/NAME]

Needs csi_recv/csi_send firmware with the 'F'+MHz / 'r' commands (phy_set_freq).
Sequence: retune RX then TX to --freq (skipped for 2412 unless --force-tune), then
--steps times: send 'r' (relock) to the --target chips, wait --settle, record --dwell.
  rx  : relock all three receivers (inter-chip offset should re-randomize if PLL-set)
  rx0 : relock only the first receiver
  tx  : relock only the transmitter (control: inter-chip offsets must not change)
Raw streams go to <out>/<mac>.bin, events to <out>/events.json; analyse with
research/scripts/relock_analysis.py <out>.
"""
import argparse
import json
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import serial  # noqa: E402

from loss_test import RX_MACS, Recorder, dev_path  # noqa: E402

TX_MAC = "98:88:E0:71:A6:44"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=int, default=2412)
    ap.add_argument("--force-tune", action="store_true", help="send F<freq> even for 2412")
    ap.add_argument("--steps", type=int, default=12)
    ap.add_argument("--target", choices=["rx", "rx0", "tx"], default="rx")
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", time.strftime("%Y%m%d-%H%M%S") + "-relock"))
    ap.add_argument("--reset", action="store_true", help="reset TX and all RX first (new boot)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    stop = threading.Event()
    recs = [Recorder(m, stop, os.path.join(args.out, m.replace(":", "") + ".bin"), reset=args.reset) for m in RX_MACS]
    for r in recs:
        r.start()
    tx = serial.Serial()
    tx.port, tx.timeout = dev_path(TX_MAC), 0.1
    tx.open()                       # default DTR/RTS: no reset (see loss_test.py)
    if args.reset:                  # same sequence as loss_test.py --reset
        tx.dtr = False
        tx.rts = True
        time.sleep(0.1)
        tx.rts = False
    t0 = time.time()
    events = []

    def send(targets, cmd):
        for name, port in targets:
            port.write(cmd)
        events.append(dict(t=time.time() - t0, cmd=cmd.decode(), to=[n for n, _ in targets]))

    time.sleep(4.0 if args.reset else 1.0)   # boot + Wi-Fi start
    for r in recs:
        if r.error or r.serial is None:
            sys.exit(f"{r.mac}: {r.error}")
    rx_ports = [(r.mac, r.serial) for r in recs]
    tx_port = [(TX_MAC, tx)]

    def count():
        return [len(r.tx_seqs) for r in recs]

    if args.freq != 2412 or args.force_tune:
        cmd = b"F%04d" % args.freq
        before = count()
        send(rx_ports, cmd)
        time.sleep(1.0)
        mid = count()
        send(tx_port, cmd)
        time.sleep(1.5)
        after = count()
        # RX retuned alone -> frames stop (proves the RX LO moved); TX follows -> frames resume.
        print(f"retune {args.freq}: frames during RX-only 1 s {[m - b for m, b in zip(mid, before)]}, "
              f"after TX 1.5 s {[a - m for a, m in zip(after, mid)]}")
    targets = {"rx": rx_ports, "rx0": rx_ports[:1], "tx": tx_port}[args.target]
    time.sleep(args.dwell)          # baseline segment
    for k in range(args.steps):
        send(targets, b"r")
        time.sleep(args.settle + args.dwell)
        print(f"step {k + 1}/{args.steps}: frames {count()}", flush=True)
    stop.set()
    for r in recs:
        r.join(timeout=2)
    tx.close()
    for r in recs:                  # firmware log lines (lo:, pll register dumps)
        with open(os.path.join(args.out, r.mac.replace(":", "") + ".txt"), "wb") as f:
            f.write(bytes(r.text))
    with open(os.path.join(args.out, "events.json"), "w") as f:
        json.dump(dict(args=vars(args), events=events), f, indent=1)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
