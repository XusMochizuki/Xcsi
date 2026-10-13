"""Web CSI collector: finds the receivers by itself, plots live amplitude, records raw frames.

    uv run --python 3.12 --with pyserial,numpy tools/csi_web.py [--http 8000] [--runs runs]
    then open http://localhost:8000 (WSL forwards localhost to Windows)

Every /dev/serial/by-id Espressif USB-Serial-JTAG port is opened (without resetting the
chip, see csi_frame.py) and rescanned every 2 s, so a re-enumerated device (flash, usbip
re-attach) reconnects by itself. A port that sends valid CSI frames is a receiver; the
port whose MAC is --tx-mac is the sender (only used for the 'F' retune command).

Recording writes every CRC-valid frame unchanged (header + csi + crc32, csi_frame.h) to
<runs>/<time>-<name>/<MAC>.bin -- the same layout as loss_test.py, so csi_frame.load(),
phase_analysis.py and csi_unpack.py all read it -- plus meta.json.
"""
import argparse
import fcntl
import glob
import json
import math
import os
import queue
import re
import socket
import struct
import sys
import termios
import threading
import time
import zlib
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import serial  # noqa: E402

from csi_frame import FLAG_GAIN_FORCED, HDR_FMT, RELOCK_SHIFT, Parser  # noqa: E402

BY_ID = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00"
TX_MAC = "98:88:E0:71:A6:44"
# Board positions (docs/hardware.md); also the fixed display order and colour slot.
LABELS = {"98:88:E0:76:5D:68": "A·COM17", "98:88:E0:76:07:C4": "B·COM16", "98:88:E0:76:4D:04": "C·COM15"}
SC = np.arange(256) - 128                           # HE20 bin k <-> subcarrier k-128
VALID = (np.abs(SC) >= 2) & (np.abs(SC) <= 122)     # 242 tones, DC excluded
TICK_S = 0.05                                       # push rate to the browser
SPEC_EVERY = 5                                      # spectrum every 5 ticks (4 Hz)
LOSS_S = 10                                         # loss% shown over the last 10 s


def mac_of(path):
    return re.search(r"unit_([0-9A-F:]{17})-if00", path).group(1)


def holders(path):
    """Other processes that have this tty open (two readers would split the byte stream)."""
    tty, out = os.path.realpath(path), []
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.readlink(fd) == tty and int(fd.split("/")[2]) != os.getpid():
                pid = fd.split("/")[2]
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    args = f.read().decode(errors="replace").split("\0")
                out.append(f"{pid} " + " ".join([os.path.basename(args[0])] + args[1:])[:80])
        except OSError:
            pass
    return sorted(set(out))


class Device(threading.Thread):
    """One serial port. Reads forever; hands frames to the hub. Exits on any port error."""

    def __init__(self, path, hub):
        super().__init__(daemon=True)
        self.path, self.mac, self.hub = path, mac_of(path), hub
        self.role = "tx" if self.mac == TX_MAC else "?"
        self.label = LABELS.get(self.mac, "TX" if self.role == "tx" else self.mac[-5:])
        self.serial = None
        self.alive = True
        self.parser = Parser()
        self.lock = threading.Lock()
        # Stats, guarded by self.lock.
        self.frames = self.lost = 0
        self.last_seq = None
        self.window = []        # arrival times, last 1 s, for fps
        self.recent = []        # (arrival time, tx_seq gap before it), last LOSS_S, for loss%
        self.pts = []           # [tx_seq, amplitude at the selected bin] since last tick
        self.last_hdr = None
        self.last_csi = None
        self.rec_file = None
        self.rec_frames = 0
        self.rec_seq = None     # [first, last] tx_seq while recording
        self.busy = None        # other process holding the port, if any

    def run(self):
        try:
            self.busy = holders(self.path)
            if self.busy:
                return
            s = serial.Serial()
            # Keep DTR/RTS at the open() default: lowering DTR first resets the C6.
            s.port, s.timeout = self.path, 0.1
            s.open()
            # TIOCEXCL: a script started later gets EBUSY instead of silently sharing the port.
            fcntl.ioctl(s.fileno(), termios.TIOCEXCL)
            self.serial = s
            while self.hub.running:
                data = s.read(65536)
                for ev in self.parser.feed(data):
                    if ev[0] == "frame":
                        self.on_frame(ev[1], ev[2])
        except Exception as e:      # unplugged / re-enumerated; the scanner reopens it
            print(f"{self.label}: {e}", flush=True)
        finally:
            self.alive = False
            if self.serial:
                self.serial.close()
            with self.lock:
                self.close_rec()

    def on_frame(self, h, csi):
        self.role = "rx"
        b = 2 * (self.hub.sc + 128)                       # [imag, real] int8 pair
        amp = math.hypot(*struct.unpack_from("bb", csi, b)) if b + 2 <= len(csi) else 0.0
        now = time.monotonic()
        with self.lock:
            if self.last_seq is not None:
                gap = (h.tx_seq - self.last_seq - 1) & 0xFFFFFFFF
                if gap < 1000:                            # else stale buffer / TX reboot
                    self.lost += gap
                    self.recent.append((now, gap))
            self.last_seq = h.tx_seq
            self.frames += 1
            self.window.append(now)
            self.pts.append([h.tx_seq, round(amp, 2)])
            self.last_hdr, self.last_csi = h, csi
            rec_dir = self.hub.rec_dir
            if rec_dir and self.rec_file is None:
                self.rec_file = open(os.path.join(rec_dir, self.mac.replace(":", "") + ".bin"), "wb")
            if self.rec_file:
                raw = struct.pack(HDR_FMT, *h) + csi
                self.rec_file.write(raw + struct.pack("<I", zlib.crc32(raw)))
                self.rec_frames += 1
                self.rec_seq = [self.rec_seq[0] if self.rec_seq else h.tx_seq, h.tx_seq]

    def close_rec(self):
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = None

    def write(self, data):
        if self.serial and self.alive:
            self.serial.write(data)

    def snapshot(self, with_spec):
        now = time.monotonic()
        with self.lock:
            self.window = [t for t in self.window if now - t < 1.0]
            self.recent = [r for r in self.recent if now - r[0] < LOSS_S]
            n, gaps = len(self.recent), sum(r[1] for r in self.recent)
            pts, self.pts = self.pts, []
            h, csi = self.last_hdr, self.last_csi
            d = dict(mac=self.mac, label=self.label, role="busy" if self.busy else self.role, busy=self.busy,
                     fps=len(self.window),
                     frames=self.frames, lost=self.lost,
                     loss=100 * gaps / (n + gaps) if n else 0,
                     crc=self.parser.crc_errors, rec_frames=self.rec_frames, pts=pts)
        if h:
            d.update(rssi=h.rssi, agc=h.agc_gain, fft=h.fft_gain, temp=h.temp_c, lo=h.lo_mhz or None,
                     gain_forced=bool(h.flags & FLAG_GAIN_FORCED), relock=h.flags >> RELOCK_SHIFT,
                     channel=h.channel, bb_format=h.bb_format)
            if with_spec:
                a = np.frombuffer(csi, np.int8).astype(np.float32).reshape(-1, 2)
                amp = np.hypot(a[:, 0], a[:, 1])
                d["spec"] = np.round(amp[VALID[: len(amp)]], 1).tolist() if len(amp) == 256 else None
        return d


class Hub:
    def __init__(self, runs, ports=BY_ID):
        self.runs, self.ports = runs, ports
        self.running = True
        self.sc = 20                    # selected subcarrier index (-122..122)
        self.devices = {}               # path -> Device
        self.clients = []               # queue.Queue per SSE client
        self.lock = threading.Lock()
        self.rec_dir = None
        self.rec_t0 = None
        self.rec_meta = None
        self.lo_req = None              # last 'F' sent, for the UI
        self.message = ""

    def scan_loop(self):
        while self.running:
            for path in sorted(glob.glob(self.ports)):
                d = self.devices.get(path)
                if d is None or not d.alive:
                    d = Device(path, self)
                    self.devices[path] = d
                    d.start()
            for path in [p for p, d in self.devices.items() if not d.alive and not os.path.exists(p)]:
                del self.devices[path]
            time.sleep(2)

    def ordered(self):
        order = list(LABELS)
        return sorted(list(self.devices.values()),
                      key=lambda d: (d.role != "rx", order.index(d.mac) if d.mac in order else 99, d.mac))

    def tick_loop(self):
        n = 0
        while self.running:
            time.sleep(TICK_S)
            n += 1
            spec = n % SPEC_EVERY == 0
            msg = dict(sc=self.sc, devices=[d.snapshot(spec) for d in self.ordered()],
                       rec=self.rec_status(), lo_req=self.lo_req, message=self.message)
            data = ("data: " + json.dumps(msg, separators=(",", ":")) + "\n\n").encode()
            with self.lock:
                for q in self.clients:
                    try:
                        q.put_nowait(data)
                    except queue.Full:  # slow browser tab: drop, it catches up next tick
                        pass

    def rec_status(self):
        if not self.rec_dir:
            return None
        return dict(dir=self.rec_dir, elapsed=time.monotonic() - self.rec_t0)

    # --- commands from the browser ---

    def cmd_select(self, sc):
        sc = int(sc)
        if not -122 <= sc <= 122:
            raise ValueError("subcarrier must be in -122..122")
        self.sc = sc

    def cmd_record(self, action, name=""):
        if action == "start":
            if self.rec_dir:
                raise ValueError("already recording")
            name = re.sub(r"[^\w.-]+", "_", name.strip())
            d = os.path.join(self.runs, datetime.now().strftime("%Y%m%d-%H%M%S") + (f"-{name}" if name else ""))
            os.makedirs(d)
            rx = [x for x in self.ordered() if x.role == "rx"]
            self.rec_meta = dict(start=datetime.now().isoformat(timespec="seconds"), host=socket.gethostname(),
                                 frame_version=1, tx_mac=TX_MAC, lo_req=self.lo_req,
                                 devices={x.mac: dict(label=x.label) for x in rx})
            for x in rx:
                with x.lock:
                    x.rec_frames, x.rec_seq = 0, None
            self.rec_t0 = time.monotonic()
            self.rec_dir = d      # device threads open their file on the next frame
            self.write_meta()
            self.message = f"recording to {d}"
        elif action == "stop":
            if not self.rec_dir:
                raise ValueError("not recording")
            d, self.rec_dir = self.rec_dir, None
            for x in list(self.devices.values()):
                with x.lock:
                    x.close_rec()
                    if x.rec_frames:
                        m = self.rec_meta["devices"].setdefault(x.mac, dict(label=x.label))
                        m.update(frames=x.rec_frames, tx_seq=x.rec_seq)
            self.rec_meta.update(stop=datetime.now().isoformat(timespec="seconds"),
                                 duration_s=round(time.monotonic() - self.rec_t0, 2))
            self.write_meta(d)
            self.message = f"saved {d}"

    def write_meta(self, d=None):
        with open(os.path.join(d or self.rec_dir, "meta.json"), "w") as f:
            json.dump(self.rec_meta, f, indent=2)

    def cmd_lo(self, mhz):
        """Retune RX first, then TX (RX applies 'F' on its next frame, so the link must still be up)."""
        mhz = int(mhz)
        if not 2300 <= mhz <= 2500:
            raise ValueError("MHz must be in 2300..2500")
        rx = [d for d in list(self.devices.values()) if d.role == "rx"]
        tx = [d for d in list(self.devices.values()) if d.role == "tx" and d.alive]
        if not rx:
            raise ValueError("no receiver connected")
        for d in rx:
            d.write(f"F{mhz}".encode())
        time.sleep(0.3)
        for d in tx:
            d.write(f"F{mhz}".encode())
        self.lo_req = mhz
        self.message = f"F{mhz} sent to {len(rx)} RX" + (" + TX" if tx else "; TX not connected, retune it yourself")

    def cmd_gain(self, mode):
        c = {"lock": b"g", "auto": b"a"}[mode]
        for d in list(self.devices.values()):
            if d.role == "rx":
                d.write(c)
        self.message = "gain lock: median of next 100 frames" if mode == "lock" else "gain: auto"


def make_handler(hub, html_path):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                with open(html_path, "rb") as f:
                    self.send(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                q = queue.Queue(maxsize=40)
                with hub.lock:
                    hub.clients.append(q)
                try:
                    while hub.running:
                        self.wfile.write(q.get())
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with hub.lock:
                        hub.clients.remove(q)
            else:
                self.send(404, b"{}")

        def do_POST(self):
            cmds = {"/api/select": hub.cmd_select, "/api/record": hub.cmd_record,
                    "/api/lo": hub.cmd_lo, "/api/gain": hub.cmd_gain}
            fn = cmds.get(self.path)
            if fn is None:
                return self.send(404, b"{}")
            try:
                args = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                fn(**args)
                self.send(200, b'{"ok":true}')
            except Exception as e:
                self.send(400, json.dumps({"error": str(e)}).encode())

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http", type=int, default=8000, help="HTTP port")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--ports", default=BY_ID, help="glob of serial ports to watch (names must end in unit_<MAC>-if00)")
    ap.add_argument("--runs", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs"))
    args = ap.parse_args()

    hub = Hub(os.path.normpath(args.runs), args.ports)
    threading.Thread(target=hub.scan_loop, daemon=True).start()
    threading.Thread(target=hub.tick_loop, daemon=True).start()
    html = os.path.join(os.path.dirname(os.path.abspath(__file__)), "csi_web.html")
    srv = ThreadingHTTPServer((args.bind, args.http), make_handler(hub, html))
    srv.daemon_threads = True
    print(f"http://localhost:{args.http}  (recordings -> {hub.runs})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if hub.rec_dir:
            hub.cmd_record("stop")
            print(hub.message)
        hub.running = False


if __name__ == "__main__":
    main()
