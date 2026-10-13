"""Shared helpers for the portable reproduction tests (see README.md in this directory).

Bench description: bench.json  {"tx": "<usb serial>", "rx": ["<usb serial>", ...],
                                 "names": {"<usb serial>": "A", ...}}
ESP32-C6 USB-Serial-JTAG reports the chip MAC as USB serial number; ports are looked up
by that serial every time, so the same bench.json works on Linux, WSL and Windows (COMx)
and survives re-enumeration.

Firmware: firmware/csi_recv + firmware/csi_send of this repository (frames per
csi_frame.h, host commands F<MHz>, r, P<Hz>).
"""
import json
import os
import sys
import threading
import time

import numpy as np
import serial
from serial.tools import list_ports

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from csi_frame import RELOCK_SHIFT, Parser, csi_to_complex  # noqa: E402

ESPRESSIF_VID = 0x303A
DF = 78.125e3                                   # HE20 subcarrier spacing (native CSI grid)
SC = np.arange(256) - 128                       # native bin k <-> subcarrier k - 128
VALID = (np.abs(SC) >= 2) & (np.abs(SC) <= 122)  # HE20 242-tone RU without DC (bin 127 spike)
K = SC[VALID].astype(float)


# ---------------------------------------------------------------- devices

def espressif_ports():
    """[(usb serial, port device)] of attached Espressif USB-Serial-JTAG devices."""
    return sorted((p.serial_number, p.device) for p in list_ports.comports()
                  if p.vid == ESPRESSIF_VID and p.serial_number)


def port_of(serial_number, timeout=15.0):
    """Port for a USB serial number; waits for (re-)enumeration."""
    t0 = time.time()
    while True:
        for sn, dev in espressif_ports():
            if sn.lower() == serial_number.lower():
                return dev
        if time.time() - t0 > timeout:
            raise RuntimeError(f"device {serial_number} not found (attached? see README)")
        time.sleep(0.5)


def load_bench(path):
    with open(path) as f:
        b = json.load(f)
    b.setdefault("names", {})
    for i, sn in enumerate(b["rx"]):
        b["names"].setdefault(sn, f"RX{i}")
    b["names"].setdefault(b["tx"], "TX")
    return b


def port_holders(dev):
    """Other processes that have the tty open (Linux /proc scan; [] elsewhere)."""
    if not os.path.isdir("/proc"):
        return []
    target, me, out = os.path.realpath(dev), os.getpid(), []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == me:
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") == target:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        out.append(f"{pid} {f.read().replace(b'\0', b' ').decode(errors='replace')[:100]}")
                    break
        except OSError:
            continue
    return out


def open_port(serial_number, reset=False):
    """Open without resetting (DTR/RTS left asserted). reset=True: RTS=1/DTR=0 pulse =
    USB-Serial-JTAG chip reset (same as esptool's hard reset).

    Two readers on one tty silently split the byte stream (both lose frames, no error),
    so refuse a port another process holds, and lock ours: flock (pyserial exclusive)
    plus TIOCEXCL, which makes any later open() fail with EBUSY. Windows COM ports are
    exclusive anyway."""
    dev = port_of(serial_number)
    busy = port_holders(dev)
    if busy:
        raise RuntimeError(f"{serial_number} ({dev}) is busy, opened by: " + "; ".join(busy))
    s = serial.Serial()
    s.port, s.timeout = dev, 0.1
    if os.name == "posix":
        s.exclusive = True
    s.open()
    if os.name == "posix":
        import fcntl
        import termios
        fcntl.ioctl(s.fileno(), termios.TIOCEXCL)
    if reset:
        s.dtr = False
        s.rts = True
        time.sleep(0.1)
        s.rts = False
    return s


# ---------------------------------------------------------------- recording

class Recorder(threading.Thread):
    """Reads CSI frames from one receiver; keeps headers + complex CSI in memory and
    writes the raw stream to raw_path."""

    def __init__(self, serial_number, raw_path, reset=False):
        super().__init__(daemon=True)
        self.sn, self.raw_path, self.reset = serial_number, raw_path, reset
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.hdrs, self.csi = [], []
        self.text = bytearray()
        self.error = None
        self.port = None

    def run(self):
        try:
            self.port = open_port(self.sn, self.reset)
        except Exception as e:  # noqa: BLE001
            self.error = str(e)
            self.ready.set()
            return
        self.ready.set()
        p = Parser()
        with open(self.raw_path, "wb") as raw:
            while not self.stop.is_set():
                try:
                    data = self.port.read(65536)
                except Exception:  # noqa: BLE001  USB re-enumerated (e.g. after reset): reopen
                    try:
                        self.port.close()
                        time.sleep(0.5)
                        self.port = open_port(self.sn)      # waits up to 15 s for the device
                        self.reconnects = getattr(self, "reconnects", 0) + 1
                        continue
                    except Exception as e:  # noqa: BLE001
                        self.error = str(e)
                        break
                raw.write(data)
                for ev in p.feed(data):
                    if ev[0] == "frame":
                        self.hdrs.append(ev[1])
                        self.csi.append(ev[2])
                    else:
                        self.text += ev[1]
        self.port.close()

    @property
    def last_tx_seq(self):
        return self.hdrs[-1].tx_seq if self.hdrs else None


class Session:
    """TX command port + one Recorder per RX for the duration of a `with` block.
    Events (commands) are logged with the receivers' latest tx_seq so the analysis can
    cut the stream into segments without relying on host timing."""

    def __init__(self, bench, out_dir, reset=False):
        self.b, self.out, self.reset = bench, out_dir, reset
        self.events = []

    def __enter__(self):
        os.makedirs(self.out, exist_ok=True)
        self.recs = [Recorder(sn, os.path.join(self.out, self.b["names"][sn] + ".bin"), self.reset)
                     for sn in self.b["rx"]]
        for r in self.recs:
            r.start()
        self.tx = open_port(self.b["tx"], self.reset)
        for r in self.recs:
            r.ready.wait(20)
            if r.error:
                raise RuntimeError(f"{self.b['names'][r.sn]}: {r.error}")
        time.sleep(4.0 if self.reset else 1.0)      # boot + Wi-Fi start
        self.mark("start")
        return self

    def __exit__(self, *exc):
        for r in self.recs:
            r.stop.set()
        for r in self.recs:
            r.join(timeout=3)
        self.tx.close()
        with open(os.path.join(self.out, "events.json"), "w") as f:
            json.dump(dict(bench=self.b, events=self.events), f, indent=1)
        for r in self.recs:
            with open(os.path.join(self.out, self.b["names"][r.sn] + ".log"), "wb") as f:
                f.write(bytes(r.text))

    def mark(self, what, **kw):
        seqs = [r.last_tx_seq for r in self.recs if r.last_tx_seq is not None]
        self.events.append(dict(t=time.time(), what=what, tx_seq=max(seqs) if seqs else None, **kw))

    def frames(self):
        return [len(r.hdrs) for r in self.recs]

    def send_rx(self, cmd):
        for r in self.recs:
            for _ in range(30):                     # recorder may be reopening a re-enumerated port
                try:
                    r.port.write(cmd)
                    break
                except Exception:  # noqa: BLE001
                    time.sleep(0.5)
            else:
                raise RuntimeError(f"{self.b['names'][r.sn]}: cannot write command")

    def send_tx(self, cmd):
        try:
            self.tx.write(cmd)
        except Exception:  # noqa: BLE001  re-enumerated after reset: reopen once
            self.tx.close()
            self.tx = open_port(self.b["tx"])
            self.tx.write(cmd)

    def retune(self, mhz):
        """RX first (applied on the next received frame), then TX. Returns frames received
        while only the RX had moved (should be ~0: proves the RX LO moved) and after."""
        cmd = b"F%04d" % mhz
        a = self.frames()
        self.send_rx(cmd)
        time.sleep(1.0)
        b = self.frames()
        self.send_tx(cmd)
        time.sleep(1.5)
        c = self.frames()
        self.mark("retune", mhz=mhz)
        return [y - x for x, y in zip(a, b)], [y - x for x, y in zip(b, c)]


# ---------------------------------------------------------------- analysis

def load_run(run_dir):
    """{name: dict(seq, H[valid], relock, lo, agc, rssi, temp)} + events."""
    with open(os.path.join(run_dir, "events.json")) as f:
        ev = json.load(f)
    devs = {}
    for sn in ev["bench"]["rx"]:
        name = ev["bench"]["names"][sn]
        p = Parser()
        with open(os.path.join(run_dir, name + ".bin"), "rb") as f:
            fr = [(e[1], e[2]) for e in p.feed(f.read()) if e[0] == "frame"]
        # frame_seq is the receiver's own counter: gaps = frames lost between chip and host
        # (device queue full, or bytes taken by another reader of the same tty).
        gaps = sum(max(b.frame_seq - a.frame_seq - 1, 0) for (a, _), (b, _) in zip(fr, fr[1:])
                   if b.frame_seq > a.frame_seq)
        n_all = len(fr)
        fr = [(h, c) for h, c in fr if h.csi_len == 512 and h.bb_format == 4]   # HE SU, HE20
        if not fr:
            devs[name] = None
            continue
        seq = np.array([h.tx_seq for h, _ in fr])
        seq, idx = np.unique(seq, return_index=True)
        H = np.array([csi_to_complex(fr[i][1]) for i in idx])[:, VALID]
        g = lambda f: np.array([f(fr[i][0]) for i in idx])  # noqa: E731
        devs[name] = dict(seq=seq, H=H, relock=g(lambda h: (h.flags >> RELOCK_SHIFT) & 255),
                          lo=g(lambda h: h.lo_mhz), agc=g(lambda h: h.agc_gain),
                          rssi=g(lambda h: h.rssi), temp=g(lambda h: h.temp_c),
                          frames=n_all, gaps=gaps)
    return devs, ev


MAX_GAP_FRACTION = 0.01


def integrity(devs, where=""):
    """(report lines, ok): host-side frame loss per receiver from frame_seq gaps."""
    lines, ok = [], True
    for name, d in devs.items():
        if d is None:
            lines.append(f"- {where}{name}: no frames")
            ok = False
            continue
        frac = d["gaps"] / max(d["frames"] + d["gaps"], 1)
        bad = frac > MAX_GAP_FRACTION
        ok &= not bad
        if d["gaps"] or bad:
            lines.append(f"- {where}{name}: {d['gaps']} frames lost between chip and host ({frac * 100:.1f}%)"
                         + (" -> DATA INVALID (another reader on the port? USB overload?)" if bad else ""))
    return lines, ok


def circ_mean(x):
    return float(np.angle(np.mean(np.exp(1j * np.asarray(x)))))


def circ_std(x):
    r = np.abs(np.mean(np.exp(1j * np.asarray(x))))
    return float(np.sqrt(-2 * np.log(max(r, 1e-12))))


def pair_intercepts(di, dj):
    """Common tx_seq, DC intercept b [rad] of unwrap(angle(H_i conj H_j)) per frame."""
    common, ii, jj = np.intersect1d(di["seq"], dj["seq"], return_indices=True)
    X = di["H"][ii] * np.conj(dj["H"][jj])
    ph = np.unwrap(np.angle(X), axis=1)
    A = np.vstack([K, np.ones_like(K)]).T
    coef, *_ = np.linalg.lstsq(A, ph.T, rcond=None)
    return common, np.angle(np.exp(1j * coef[1])), ii, jj


def segments(events, kinds):
    """tx_seq boundaries [(start, end, label)] between consecutive events whose `what` is in kinds."""
    marks = [e for e in events if e["what"] in kinds and e["tx_seq"] is not None]
    out = []
    for a, b in zip(marks, marks[1:] + [None]):
        out.append((a["tx_seq"], b["tx_seq"] if b else 2 ** 32, a.get("label", a["what"])))
    return out


def _pairs(devs):
    names = [n for n in devs if devs[n] is not None]
    return [(names[a], names[b]) for a in range(len(names)) for b in range(a + 1, len(names))]


def _segment_index(seq, events, kinds, skip, min_frames):
    out = []
    for s0, s1, label in segments(events, kinds):
        m = np.flatnonzero((seq > s0) & (seq < s1))[skip:]
        if len(m) >= min_frames:
            out.append((label, m))
    return out


def agc_phase_table(devs, events, kinds, skip=30, min_frames=80, min_code=30, iters=2):
    """Per-chip phase offset of each AGC code [rad], estimated from the run itself.

    Within a segment the LO state is fixed, so after removing the segment mean the
    remaining intercept residual of pair (i, j) is modelled as phi_i[agc_i] - phi_j[agc_j].
    Least squares over all pairs; each chip's most common code is the zero reference.
    Codes seen in fewer than min_code frames are left at 0."""
    table = {n: {} for n in devs if devs[n] is not None}
    for _ in range(iters):
        rows, rhs, cols = [], [], {}
        for a, b in _pairs(devs):
            di, dj = devs[a], devs[b]
            seq, bb, ii, jj = pair_intercepts(di, dj)
            gi, gj = di["agc"][ii], dj["agc"][jj]
            corr = np.array([table[a].get(int(x), 0.0) - table[b].get(int(y), 0.0) for x, y in zip(gi, gj)])
            for _, m in _segment_index(seq, events, kinds, skip, min_frames):
                r = np.angle(np.exp(1j * (bb[m] - corr[m] - circ_mean(bb[m] - corr[m])))) + corr[m]
                for x, y in zip(gi[m], gj[m]):
                    rows.append(((a, int(x)), (b, int(y))))
                rhs.extend(r.tolist())
        count = {}
        for (ca, cb) in rows:
            count[ca] = count.get(ca, 0) + 1
            count[cb] = count.get(cb, 0) + 1
        modal = {}
        for (chip, code), n in count.items():
            if n > count.get((chip, modal.get(chip, -1)), -1):
                modal[chip] = code
        for key, n in count.items():
            if n >= min_code and key[1] != modal[key[0]]:
                cols.setdefault(key, len(cols))
        if not cols:
            return table
        A = np.zeros((len(rows), len(cols)))
        for r, (ca, cb) in enumerate(rows):
            if ca in cols:
                A[r, cols[ca]] += 1
            if cb in cols:
                A[r, cols[cb]] -= 1
        x, *_ = np.linalg.lstsq(A, np.array(rhs), rcond=None)
        table = {n: {} for n in table}
        for (chip, code), k in cols.items():
            table[chip][code] = float(x[k])
    return table


def segment_means(devs, events, kinds, skip=30, min_frames=80, agc_table=None):
    """{pair: [(label, n, mean_rad, std_rad)]} over event-delimited segments.
    agc_table (from agc_phase_table) is subtracted per frame when given."""
    res = {}
    for a, b in _pairs(devs):
        di, dj = devs[a], devs[b]
        seq, bb, ii, jj = pair_intercepts(di, dj)
        if agc_table:
            bb = bb - np.array([agc_table[a].get(int(x), 0.0) - agc_table[b].get(int(y), 0.0)
                                for x, y in zip(di["agc"][ii], dj["agc"][jj])])
        res[f"{a}-{b}"] = [(label, len(m), circ_mean(bb[m]), circ_std(bb[m]))
                           for label, m in _segment_index(seq, events, kinds, skip, min_frames)]
    return res


def lattice_stats(means_rad):
    """Resultant length R_k of k*theta for k = 1, 2, 4, 8 (R_k ~ 1: values on a 360/k grid)."""
    m = np.asarray(means_rad)
    return {k: float(np.abs(np.mean(np.exp(1j * k * m)))) for k in (1, 2, 4, 8)}


def lattice_offset(means_rad, k):
    """Grid origin theta0 in (-180/k, 180/k] deg and residuals of each value from the grid."""
    m = np.asarray(means_rad)
    t0 = np.angle(np.mean(np.exp(1j * k * m))) / k
    res = np.angle(np.exp(1j * k * (m - t0))) / k
    return float(np.degrees(t0)), np.degrees(res)


def write_report(run_dir, title, lines, result):
    with open(os.path.join(run_dir, "result.json"), "w") as f:
        json.dump(result, f, indent=1, default=float)
    with open(os.path.join(run_dir, "report.md"), "w") as f:
        f.write(f"# {title}\n\n" + "\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\nreport: {os.path.join(run_dir, 'report.md')}")


def default_out(tag):
    return os.path.join(ROOT, "runs", time.strftime("%Y%m%d-%H%M%S") + "-" + tag)
