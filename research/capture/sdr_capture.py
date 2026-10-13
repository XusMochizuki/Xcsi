"""Burst IQ capture from an ESP-SDR (ESPARGOS esp-sdr) ESP32-C6 over USB-Serial-JTAG.

    uv run --python 3.12 --with pyserial,numpy research/capture/sdr_capture.py PORT \
        --n 16380 --count 200 [--freq 2412] [--gain hw|<index>] --out runs/NAME.npz

Protocol (esp-sdr README / main/families/c5_c6_c61/receiver.c): newline-terminated ASCII
commands; `CAP20 <n> 0` -> "DATA <n> <crc32-hex> <capture-us>\n" + ceil(n*20/8) bytes of
packed signed 10-bit I/Q (I = bits 0..9, Q = bits 10..19 of each 20-bit sample, two
samples per 5 bytes). C6 supports only rate index 0 = 80 MS/s, n in 256..16380.
Saves complex64 iq[count, n] (raw codes, not scaled), host receive times, capture_us,
and the INFO/CAPS/LIMITS/VERSION replies.
"""
import argparse
import time
import zlib

import numpy as np
import serial

FS = 80e6


def unpack_iq10(buf, n):
    """Packed IQ10 bytes -> complex64[n] of raw signed codes."""
    b = np.frombuffer(buf, dtype=np.uint8)
    pairs = (n + 1) // 2
    b = np.concatenate([b, np.zeros(pairs * 5 - len(b), np.uint8)]).reshape(pairs, 5).astype(np.uint32)
    a = b[:, 0] | (b[:, 1] << 8) | ((b[:, 2] & 0x0F) << 16)
    c = (b[:, 2] >> 4) | (b[:, 3] << 4) | (b[:, 4] << 12)
    w = np.stack([a, c], axis=1).reshape(-1)[:n]

    def s10(x):
        x = (x & 0x3FF).astype(np.int32)
        return np.where(x >= 512, x - 1024, x)

    return (s10(w) + 1j * s10(w >> 10)).astype(np.complex64)


class Sdr:
    def __init__(self, port):
        self.s = serial.Serial()
        self.s.port, self.s.timeout = port, 2.0
        self.s.open()               # default DTR/RTS: no reset
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def line(self):
        out = self.s.readline()
        if not out.endswith(b"\n"):
            raise TimeoutError(f"no reply (got {out!r})")
        return out.decode(errors="replace").strip()

    def cmd(self, text):
        self.s.write(text.encode() + b"\n")
        return self.line()

    def capture(self, n):
        self.s.write(f"CAP20 {n} 0\n".encode())
        hdr = self.line()
        if not hdr.startswith("DATA "):
            raise RuntimeError(hdr)
        _, n_s, crc_s, us_s = hdr.split()
        size = (int(n_s) * 20 + 7) // 8
        buf = self.s.read(size)
        if len(buf) != size:
            raise TimeoutError(f"short payload {len(buf)}/{size}")
        if zlib.crc32(buf) != int(crc_s, 16):
            raise ValueError("payload CRC mismatch")
        return unpack_iq10(buf, int(n_s)), int(us_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--n", type=int, default=16380)
    ap.add_argument("--count", type=int, default=100)
    ap.add_argument("--freq", type=int, default=2412)
    ap.add_argument("--gain", default="hw", help="'hw' (hardware AGC) or a manual gain index")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    sdr = Sdr(args.port)
    meta = {q: sdr.cmd(q) for q in ("SYNC 1", "VERSION?", "INFO", "CAPS", "LIMITS?", "TRANSPORT?")}
    meta["FREQ"] = sdr.cmd(f"FREQ {args.freq}")
    meta["GAIN"] = sdr.cmd("GAIN HARDWARE" if args.gain == "hw" else f"GAIN MANUAL {args.gain}")
    for k, v in meta.items():
        print(f"{k}: {v}")
    iq = np.zeros((args.count, args.n), np.complex64)
    t_host = np.zeros(args.count)
    cap_us = np.zeros(args.count, np.int64)
    t0 = time.time()
    for k in range(args.count):
        iq[k], cap_us[k] = sdr.capture(args.n)
        t_host[k] = time.time() - t0
    dt = time.time() - t0
    print(f"{args.count} captures x {args.n} samples in {dt:.2f} s: {args.count / dt:.1f} captures/s, "
          f"duty {args.count * args.n / FS / dt * 100:.2f}% of air time, capture_us median {np.median(cap_us):.0f}")
    sdr.cmd("RELEASE")
    np.savez_compressed(args.out, iq=iq, t_host=t_host, capture_us=cap_us, fs=FS, freq_mhz=args.freq,
                        gain=args.gain, meta=np.array([f"{k}: {v}" for k, v in meta.items()]))
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
