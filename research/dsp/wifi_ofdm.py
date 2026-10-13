"""802.11 legacy + HE SU (20 MHz) preamble processing: raw IQ -> packet detection, CFO,
timing, L-LTF CSI and 2x HE-LTF CSI.

Input: complex baseband at 80 MS/s (ESP-SDR ESP32-C6) or 20 MS/s. Everything below runs
at 20 MS/s (64-point legacy FFT, 312.5 kHz; 128-point 2x HE-LTF FFT, 156.25 kHz).

HE SU preamble (20 MHz), times from packet start:
  L-STF 0-8 us | L-LTF 8-16 (GI2 1.6 + 2 x 3.2) | L-SIG 16-20 | RL-SIG 20-24 |
  HE-SIG-A 24-32 | HE-STF 32-36 | HE-LTF 36-44 (2x: GI 1.6 + 6.4 us)   [GI+LTF code 2]
Index n_ltf = first sample of the first L-LTF symbol (after GI2) = packet start + 192.

Subcarrier conventions: legacy tone m in -26..26 (0 null) <-> 4x-HE tone 4m;
2x HE-LTF tone q in -61..61 <-> 4x-HE tone 2q, i.e. even tones +-2..+-122 of the native
ESP32-C6 HE CSI grid (bin = tone + 128).

The HE-LTF +-1 sequence is not hard-coded (no verified copy of the 802.11ax table here):
its signs are decided per tone from the L-LTF channel (decision-directed), and callers
should check the decided sign pattern is identical across packets.
"""
import numpy as np
from scipy.signal import remez, resample_poly

FS = 20e6
# L-STF, tones -26..26 (IEEE 802.11-2020 Eq. 17-6), already sqrt(13/6) scaled.
_STF = np.sqrt(13 / 6) * np.array(
    [0, 0, 1 + 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j,
     0, 0, 0, 0, 0, 0, 0, -1 - 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j,
     0, 0, 0, 1 + 1j, 0, 0])
# L-LTF, tones -26..26 (Eq. 17-8).
_LTF = np.array(
    [1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 0,
     1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1], float)
LEG_TONES = np.array([m for m in range(-26, 27) if m != 0])
L_LTF = _LTF[LEG_TONES + 26]
HE2X_TONES = np.array([q for q in range(-61, 62) if q != 0])     # 4x tones 2q = +-2..+-122
N_LTF_OFF = 192                                                  # packet start -> n_ltf


def freq_to_time(tones, values, n):
    """Tone-indexed spectrum -> one OFDM symbol (n samples, unnormalized ifft * n / sqrt(len))."""
    X = np.zeros(n, complex)
    X[np.asarray(tones) % n] = values
    return np.fft.ifft(X) * n / np.sqrt(len(values))


L_STF_SYM = freq_to_time(np.arange(-26, 27), _STF, 64)           # periodic with 16 samples
L_LTF_SYM = freq_to_time(LEG_TONES, L_LTF, 64)


def to_20msps(x, fs):
    if fs == FS:
        return np.asarray(x, complex)
    r = int(round(fs / FS))
    assert abs(fs / r - FS) < 1, "fs must be an integer multiple of 20 MHz"
    # 2x HE-LTF tones reach +-9.53 MHz, next to the 10 MHz Nyquist edge: the default
    # resample_poly filter attenuates them, so use a filter flat to 9.7 MHz.
    h = remez(40 * r + 1, [0, 9.7e6, 10.5e6, fs / 2], [1, 0], fs=fs)
    return resample_poly(np.asarray(x, complex), 1, r, window=h * r)


def detect_stf(x, thresh=0.6, win=64, min_run=48, min_snr_db=6.0):
    """Schmidl-Cox on the 16-sample L-STF period. Returns list of (start, coarse_cfo_hz).
    Windows must also be min_snr_db above the capture noise floor (10th percentile of the
    window power; a packet can fill half a capture, so not the median): residual DC or a CW
    tone is perfectly periodic too and would otherwise pass the M test."""
    lag = 16
    p = x[:-lag] * np.conj(x[lag:])
    e = np.abs(x[lag:]) ** 2
    k = np.ones(win)
    P = np.convolve(p, k, "valid")
    R = np.convolve(e, k, "valid")
    M = np.abs(P) ** 2 / np.maximum(R, 1e-12) ** 2
    floor = np.percentile(R, 10)
    above = (M > thresh) & (R > floor * 10 ** (min_snr_db / 10))
    out, n = [], 0
    while n < len(above):
        if above[n]:
            m = n
            while m < len(above) and above[m]:
                m += 1
            if m - n >= min_run:
                mid = (n + m) // 2
                cfo = -np.angle(P[mid]) * FS / (2 * np.pi * lag)
                out.append((n, cfo))
            n = m
        else:
            n += 1
    return out


def rotate(x, cfo, n0=0):
    """Remove a frequency offset cfo [Hz]; phase referenced to sample index n0."""
    n = np.arange(len(x)) - n0
    return x * np.exp(-2j * np.pi * cfo * n / FS)


def fine_timing(x, start, search=260):
    """Index of the first L-LTF symbol: maximise |c[n]| + |c[n+64]| of the LTF correlation."""
    seg = x[start:start + search + 192]
    ref = np.conj(L_LTF_SYM[::-1])
    c = np.abs(np.convolve(seg, ref, "valid"))          # c[n] = corr with symbol starting at n
    s = c[:-64] + c[64:]
    return start + int(np.argmax(s[:search]))


def ltf_csi(x, n_ltf, backoff=4):
    """L-LTF CSI on LEG_TONES from two symbols. FFT window starts backoff samples early (inside
    the CP, ISI-free); the resulting slope is removed so H refers to timing n_ltf.
    Returns H, per-tone noise variance estimate, fine CFO [Hz]."""
    a = x[n_ltf:n_ltf + 64]
    b = x[n_ltf + 64:n_ltf + 128]
    fine = -np.angle(np.vdot(b, a)) * FS / (2 * np.pi * 64)     # vdot(b,a)=sum conj(b) a
    w0 = n_ltf - backoff
    Y1 = np.fft.fft(x[w0:w0 + 64])[LEG_TONES % 64]
    Y2 = np.fft.fft(x[w0 + 64:w0 + 128])[LEG_TONES % 64]
    ramp = np.exp(2j * np.pi * LEG_TONES * backoff / 64)        # undo window advance
    Y1, Y2 = Y1 * ramp, Y2 * ramp
    scale = 64 / np.sqrt(len(LEG_TONES))                        # inverse of freq_to_time scaling
    H = (Y1 + Y2) / 2 / L_LTF / scale
    noise = np.abs(Y1 - Y2) ** 2 / 4 / scale ** 2               # var of the 2-symbol mean
    return H, noise, fine


def legacy_symbol(x, start, H, backoff=4):
    """Equalised legacy-format symbol (GI 16 + 64) at sample start, on LEG_TONES."""
    w0 = start + 16 - backoff
    Y = np.fft.fft(x[w0:w0 + 64])[LEG_TONES % 64] * np.exp(2j * np.pi * LEG_TONES * backoff / 64)
    return Y / (64 / np.sqrt(len(LEG_TONES))) / H


def he2x_csi(x, n_ltf, H_leg, backoff=8):
    """2x HE-LTF CSI on HE2X_TONES; signs decided from the L-LTF channel (see module doc).
    Returns H_he (sign-resolved), the decided sign pattern, raw Y (tone values / sqrt(N))."""
    start = n_ltf + 528 + 32                                   # HE-LTF FFT window (after 1.6 us GI)
    w0 = start - backoff
    Y = np.fft.fft(x[w0:w0 + 128])[HE2X_TONES % 128] * np.exp(2j * np.pi * HE2X_TONES * backoff / 128)
    Y = Y / (128 / np.sqrt(len(HE2X_TONES)))
    # Predict the channel at legacy tone position q/2 by interpolating the L-LTF CSI
    # (amplitude and unwrapped phase separately), then decide each sign outward from DC.
    m = LEG_TONES.astype(float)
    ph = np.unwrap(np.angle(H_leg))
    pos = HE2X_TONES / 2.0
    pred = np.interp(pos, m, np.abs(H_leg)) * np.exp(1j * np.interp(pos, m, ph))
    # Common gain/phase between the legacy and HE parts (TX scaling, residual CFO phase):
    # fit on the central tones using |.|-insensitive squared decisions first.
    inner = np.abs(pos) <= 20
    z = np.sum((Y[inner] / pred[inner]) ** 2)
    g = np.exp(1j * np.angle(z) / 2)                            # +-pi ambiguity is absorbed by signs
    s = np.zeros(len(Y))
    H = np.zeros(len(Y), complex)
    order = np.argsort(np.abs(HE2X_TONES), kind="stable")
    for i in order:
        q = HE2X_TONES[i]
        if abs(q) / 2 <= 26:
            p = pred[i] * g
        else:                                                   # outside L-LTF: extrapolate HE estimates
            side = np.flatnonzero((np.sign(HE2X_TONES) == np.sign(q)) & (np.abs(HE2X_TONES) < abs(q))
                                  & (s != 0))
            near = side[np.argsort(np.abs(HE2X_TONES[side]))[-3:]]
            k = HE2X_TONES[near].astype(float)
            a_fit = np.polyfit(k, np.unwrap(np.angle(H[near])), 1)
            p = np.mean(np.abs(H[near])) * np.exp(1j * np.polyval(a_fit, q))
        s[i] = 1.0 if np.real(Y[i] * np.conj(p)) >= 0 else -1.0
        H[i] = Y[i] * s[i]
    return H, s, Y


def process_capture(iq, fs=80e6, thresh=0.6, he=True):
    """All complete packets in one capture -> list of dicts (see keys below)."""
    x = to_20msps(iq - np.mean(iq), fs)                          # DC removal before decimation
    out = []
    last_end = -1
    for start, cfo in detect_stf(x, thresh):
        if start < last_end:
            continue
        need = start + 260 + 192 + (528 + 32 + 128 + 16 if he else 128)
        if need > len(x):
            continue
        xc = rotate(x, cfo, start)
        n_ltf = fine_timing(xc, start)
        if n_ltf < 40 or n_ltf + (720 if he else 140) > len(x):     # need GI2 + FFT backoff
            continue
        H0, _, fine = ltf_csi(xc, n_ltf)
        xf = rotate(xc, fine, n_ltf)
        H, noise, _ = ltf_csi(xf, n_ltf)
        sig = legacy_symbol(xf, n_ltf + 128, H)
        evm = np.sqrt(np.mean(np.abs(np.abs(np.real(sig)) - 1) ** 2 + np.imag(sig) ** 2))
        rec = dict(n_ltf=n_ltf, cfo_hz=cfo + fine, H_lltf=H,
                   snr_db=10 * np.log10(np.mean(np.abs(H) ** 2) / np.mean(noise)),
                   lsig_evm=float(evm), lsig_bits=(np.real(sig) > 0).astype(np.uint8))
        if he:
            Hh, s, _ = he2x_csi(xf, n_ltf, H)
            rec.update(H_he=Hh, he_signs=s)
        out.append(rec)
        last_end = n_ltf + 600
    return out
