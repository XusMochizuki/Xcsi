"""Parser for the binary CSI frames from firmware/csi_recv (see csi_frame.h).

Live stats from a receiver, optionally saving the raw byte stream:
    uv run --python 3.12 --with pyserial tools/csi_frame.py /dev/serial/by-id/...-if00 [--out rx.bin]
Offline stats from a saved stream:
    uv run --python 3.12 tools/csi_frame.py --file rx.bin
Load into numpy from Python:
    from csi_frame import load; hdrs, csi = load("rx.bin")   # csi: complex64 [n, subcarriers]
"""
import argparse
import struct
import sys
import time
import zlib
from collections import namedtuple

MAGIC = b"\xC5\x1A"
VERSION = 1
MAX_CSI_LEN = 512
FLAG_FIRST_WORD_INVALID = 1 << 0
FLAG_GAIN_FORCED = 1 << 1

# Must match csi_frame_hdr_t.
HDR_FMT = "<2sBBHHIII6sbbBBBBBbBbHHIHH"
HDR_LEN = struct.calcsize(HDR_FMT)
assert HDR_LEN == 48
Header = namedtuple(
    "Header",
    "magic version hdr_len csi_len flags frame_seq tx_seq timestamp_us mac rssi noise_floor "
    "channel second rate bb_format agc_gain fft_gain rx_state temp_c sig_len rx_seq "
    "he_siga1 he_siga2 reserved1",
)


class Parser:
    """Incremental stream parser; resyncs on magic + CRC, passes other bytes through as text."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        """Yield ("frame", Header, csi_bytes) or ("text", bytes)."""
        self.buf += data
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # Keep a trailing 0xC5 that may be the first half of the next magic.
                keep = 1 if self.buf[-1:] == MAGIC[:1] else 0
                if len(self.buf) > keep:
                    yield ("text", bytes(self.buf[: len(self.buf) - keep]))
                    del self.buf[: len(self.buf) - keep]
                return
            if i > 0:
                yield ("text", bytes(self.buf[:i]))
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                return
            hdr = Header._make(struct.unpack_from(HDR_FMT, self.buf))
            if hdr.version != VERSION or hdr.hdr_len != HDR_LEN or hdr.csi_len > MAX_CSI_LEN:
                yield ("text", bytes(self.buf[:1]))
                del self.buf[:1]
                continue
            total = HDR_LEN + hdr.csi_len + 4
            if len(self.buf) < total:
                return
            (crc,) = struct.unpack_from("<I", self.buf, HDR_LEN + hdr.csi_len)
            if zlib.crc32(self.buf[: HDR_LEN + hdr.csi_len]) != crc:
                self.crc_errors += 1
                yield ("text", bytes(self.buf[:1]))
                del self.buf[:1]
                continue
            csi = bytes(self.buf[HDR_LEN : HDR_LEN + hdr.csi_len])
            del self.buf[:total]
            yield ("frame", hdr, csi)


class Stats:
    def __init__(self):
        self.reset()
        self.last_frame_seq = None
        self.last_tx_seq = None
        self.total = dict(frames=0, local_drop=0, air_loss=0)

    def reset(self):
        self.frames = self.local_drop = self.air_loss = 0
        self.rssi_sum = 0
        self.lens = set()
        self.formats = set()

    def add(self, hdr):
        if self.last_frame_seq is not None:
            local = (hdr.frame_seq - self.last_frame_seq - 1) & 0xFFFFFFFF
            tx_gap = (hdr.tx_seq - self.last_tx_seq - 1) & 0xFFFFFFFF
            self.local_drop += local
            # Frames dropped locally also appear as tx_seq gaps; the rest was lost over the air.
            self.air_loss += max(tx_gap - local, 0)
        self.last_frame_seq, self.last_tx_seq = hdr.frame_seq, hdr.tx_seq
        self.frames += 1
        self.rssi_sum += hdr.rssi
        self.lens.add(hdr.csi_len)
        self.formats.add(hdr.bb_format)

    def line(self, crc_errors):
        for k in self.total:
            self.total[k] += getattr(self, k)
        rssi = self.rssi_sum / self.frames if self.frames else float("nan")
        s = (
            f"frames {self.frames:4d}  local_drop {self.local_drop:3d}  air_loss {self.air_loss:3d}  "
            f"crc_err {crc_errors:3d}  rssi {rssi:6.1f}  len {sorted(self.lens)}  bb_format {sorted(self.formats)}  "
            f"| total frames {self.total['frames']} local_drop {self.total['local_drop']} "
            f"air_loss {self.total['air_loss']}"
        )
        self.reset()
        return s


def csi_to_complex(csi_bytes):
    """[imag0, real0, imag1, real1, ...] int8 -> complex64 array."""
    import numpy as np

    a = np.frombuffer(csi_bytes, dtype=np.int8).astype(np.float32).reshape(-1, 2)
    return (a[:, 1] + 1j * a[:, 0]).astype(np.complex64)


def load(path):
    """Return (list[Header], complex64 array [n_frames, n_subcarriers]) from a saved stream."""
    import numpy as np

    p = Parser()
    with open(path, "rb") as f:
        frames = [(ev[1], ev[2]) for ev in p.feed(f.read()) if ev[0] == "frame"]
    hdrs = [h for h, _ in frames]
    if not frames:
        return hdrs, np.empty((0, 0), np.complex64)
    n = max(h.csi_len for h in hdrs) // 2
    out = np.zeros((len(frames), n), np.complex64)
    for k, (h, c) in enumerate(frames):
        v = csi_to_complex(c)
        out[k, : len(v)] = v
    return hdrs, out


def run(read, out, seconds, quiet_text):
    p = Parser()
    st = Stats()
    t0 = t_last = time.monotonic()
    crc_last = 0
    while seconds is None or time.monotonic() - t0 < seconds:
        data = read()
        if data is None:
            break
        if out:
            out.write(data)
        for ev in p.feed(data):
            if ev[0] == "frame":
                st.add(ev[1])
            elif not quiet_text:
                sys.stdout.write(ev[1].decode(errors="replace"))
        now = time.monotonic()
        if now - t_last >= 1.0:
            print(st.line(p.crc_errors - crc_last), flush=True)
            crc_last = p.crc_errors
            t_last = now
    if st.frames:
        print(st.line(p.crc_errors - crc_last))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/serial/by-id/...-if00")
    ap.add_argument("--file", help="parse a saved raw stream instead of a port")
    ap.add_argument("--out", help="save raw byte stream to this file")
    ap.add_argument("--seconds", type=float, help="stop after N seconds")
    ap.add_argument("--quiet-text", action="store_true", help="don't echo non-frame bytes (logs)")
    args = ap.parse_args()

    out = open(args.out, "wb") if args.out else None
    try:
        if args.file:
            with open(args.file, "rb") as f:
                chunks = iter(lambda: f.read(65536) or None, None)
                p, st = Parser(), Stats()
                for chunk in chunks:
                    for ev in p.feed(chunk):
                        if ev[0] == "frame":
                            st.add(ev[1])
                print(st.line(p.crc_errors))
        elif args.port:
            import serial

            # USB-Serial-JTAG ignores baud rate. Leave DTR/RTS at their open() default (both
            # asserted): RTS=1 with DTR=0 resets the chip, which pyserial passes through if
            # dtr is lowered before rts.
            s = serial.Serial()
            s.port = args.port
            s.timeout = 0.1
            s.open()
            run(lambda: s.read(65536), out, args.seconds, args.quiet_text)
        else:
            ap.error("need a port or --file")
    except KeyboardInterrupt:
        pass
    finally:
        if out:
            out.close()


if __name__ == "__main__":
    main()
