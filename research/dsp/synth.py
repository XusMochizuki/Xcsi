"""Synthetic HE SU 20 MHz packets with known channel, for testing wifi_ofdm.

The preamble timing and the L-STF/L-LTF fields follow the standard. L-SIG / RL-SIG /
HE-SIG-A carry random BPSK, HE-STF reuses the L-STF waveform, the 2x HE-LTF uses a given
+-1 pattern (stand-in for the 802.11ax sequence) and data symbols are random QPSK on the
4x grid. Only the fields the estimator uses have to be exact.
"""
import numpy as np
from scipy.signal import resample

from wifi_ofdm import FS, HE2X_TONES, L_LTF_SYM, L_STF_SYM, LEG_TONES, freq_to_time


def he_su_packet(rng, he_signs, n_data=4):
    """20 MS/s baseband of one packet; n_ltf = 192 by construction."""
    def leg_bpsk():
        s = freq_to_time(LEG_TONES, rng.choice([-1.0, 1.0], len(LEG_TONES)), 64)
        return np.concatenate([s[-16:], s])

    stf = np.tile(L_STF_SYM, 3)[:160]
    ltf = np.concatenate([L_LTF_SYM[-32:], L_LTF_SYM, L_LTF_SYM])
    lsig = leg_bpsk()
    sig_a = np.concatenate([leg_bpsk(), leg_bpsk()])
    he_stf = np.tile(L_STF_SYM, 2)[:80]
    s = freq_to_time(HE2X_TONES, he_signs, 128)
    he_ltf = np.concatenate([s[-32:], s])
    tones4x = np.array([k for k in range(-122, 123) if abs(k) > 1])
    data = []
    for _ in range(n_data):
        q = (rng.choice([-1, 1], len(tones4x)) + 1j * rng.choice([-1, 1], len(tones4x))) / np.sqrt(2)
        d = freq_to_time(tones4x, q, 256)
        data.append(np.concatenate([d[-32:], d]))
    return np.concatenate([stf, ltf, lsig, lsig, sig_a, he_stf, he_ltf] + data)


def multipath(rng, n_taps=3, spread_ns=100.0):
    """Random taps: delays [s] (first 0) and complex gains, unit total power."""
    tau = np.concatenate([[0.0], np.sort(rng.uniform(0, spread_ns * 1e-9 * 3, n_taps - 1))])
    g = (rng.normal(size=n_taps) + 1j * rng.normal(size=n_taps)) * np.exp(-tau / (spread_ns * 1e-9))
    return tau, g / np.linalg.norm(g)


def channel_response(f_hz, tau, g, delay, phase):
    """Frequency response seen by the receiver incl. bulk delay and carrier phase."""
    f = np.asarray(f_hz, float)[:, None]
    return np.exp(1j * phase) * np.exp(-2j * np.pi * f[:, 0] * delay) * (g * np.exp(-2j * np.pi * f * tau)).sum(1)


def apply_channel(x20, tau, g, delay, phase, cfo, snr_db, rng, fs_out=80e6, offset=0, total=None,
                  adc_bits=10, adc_rms=80.0, dc=0.0):
    """Upsample to fs_out, apply channel (fractional delays in the frequency domain), CFO,
    AWGN at snr_db (relative to in-band signal power), DC and ADC quantisation.
    Returns complex raw codes, length total (packet starts at sample offset)."""
    r = int(round(fs_out / FS))
    x = resample(x20, len(x20) * r)                          # exact band-limited (FFT) interpolation
    total = total or len(x) + offset + 256
    buf = np.zeros(total, complex)
    buf[offset:offset + len(x)] = x[: total - offset]
    n = len(buf)
    F = np.fft.fftfreq(n, 1 / fs_out)
    Hf = channel_response(F, tau, g, delay, phase)
    y = np.fft.ifft(np.fft.fft(buf) * Hf)
    y *= np.exp(2j * np.pi * cfo * np.arange(n) / fs_out)
    p_sig = np.mean(np.abs(x) ** 2)
    # snr_db is defined in the 20 MHz band: white noise over fs_out carries fs_out/20e6 times
    # the in-band noise power, so after decimation to 20 MS/s the SNR is snr_db.
    var = p_sig * (fs_out / FS) / 10 ** (snr_db / 10)
    y = y + np.sqrt(var / 2) * (rng.normal(size=n) + 1j * rng.normal(size=n))
    y = y / np.sqrt(p_sig) * adc_rms + dc
    lim = 2 ** (adc_bits - 1)
    return (np.clip(np.round(y.real), -lim, lim - 1) + 1j * np.clip(np.round(y.imag), -lim, lim - 1))
