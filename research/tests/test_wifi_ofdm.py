"""Synthetic-signal tests of the IQ -> CSI chain (research/dsp/wifi_ofdm.py).

    uv run --python 3.12 --with numpy,scipy,pytest pytest -q research/tests

Fixed seeds; every case builds an 80 MS/s, 10-bit capture with a known multipath channel,
bulk delay, carrier phase, CFO, AWGN and DC, runs process_capture and compares the CSI
with the true channel up to one complex gain and one linear phase slope (the receiver's
timing reference is an integer 20 MS/s sample, so a sub-sample delay remains).
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dsp"))
import synth  # noqa: E402
import wifi_ofdm as w  # noqa: E402

DF_LEG = 312.5e3
DF_HE2X = 156.25e3


def aligned_nmse_db(H, Ht, tones):
    """min over (c, slope) of |H - c e^{j a k} Ht|^2 / |Ht|^2, slope searched finely."""
    best = np.inf
    for a in np.linspace(-0.6, 0.6, 1201):          # rad / tone; covers +-1 sample at 20 MS/s
        r = Ht * np.exp(1j * a * tones)
        c = np.vdot(r, H) / np.vdot(r, r)
        best = min(best, np.sum(np.abs(H - c * r) ** 2) / np.sum(np.abs(c * r) ** 2))
    return 10 * np.log10(best)


def make_case(seed, snr_db=30.0, cfo=0.0, delay=0.0, offset=3000, n_taps=3, total=16380, dc=0j):
    rng = np.random.default_rng(seed)
    signs = rng.choice([-1.0, 1.0], len(w.HE2X_TONES))
    x20 = synth.he_su_packet(rng, signs)
    tau, g = synth.multipath(rng, n_taps)
    phase = rng.uniform(-np.pi, np.pi)
    iq = synth.apply_channel(x20, tau, g, delay, phase, cfo, snr_db, rng, offset=offset, total=total, dc=dc)
    truth = dict(signs=signs, tau=tau, g=g, delay=delay, phase=phase, cfo=cfo,
                 n_ltf=(offset / 4) + w.N_LTF_OFF)
    return iq, truth


def true_H(truth, tones, df):
    return synth.channel_response(tones * df, truth["tau"], truth["g"], truth["delay"], truth["phase"])


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_clean_recovery(seed):
    iq, t = make_case(seed, snr_db=45.0)
    pk = w.process_capture(iq)
    assert len(pk) == 1
    p = pk[0]
    assert abs(p["n_ltf"] - t["n_ltf"]) <= 2
    assert aligned_nmse_db(p["H_lltf"], true_H(t, w.LEG_TONES, DF_LEG), w.LEG_TONES) < -30
    assert aligned_nmse_db(p["H_he"], true_H(t, w.HE2X_TONES, DF_HE2X), w.HE2X_TONES) < -28
    agree = np.mean(p["he_signs"] == t["signs"])
    assert agree in (0.0, 1.0)                       # all correct, up to one global sign
    assert p["lsig_evm"] < 0.05


@pytest.mark.parametrize("seed,cfo,delay", [(10, 80e3, 3.3e-9), (11, -95e3, 31e-9), (12, 20e3, 47e-9),
                                            (13, -50e3, 0.0)])
def test_cfo_delay_multipath(seed, cfo, delay):
    iq, t = make_case(seed, snr_db=25.0, cfo=cfo, delay=delay, dc=-75j + 8)
    pk = w.process_capture(iq)
    assert len(pk) == 1
    p = pk[0]
    assert abs(p["cfo_hz"] - cfo) < 1.5e3
    assert aligned_nmse_db(p["H_lltf"], true_H(t, w.LEG_TONES, DF_LEG), w.LEG_TONES) < -18
    assert aligned_nmse_db(p["H_he"], true_H(t, w.HE2X_TONES, DF_HE2X), w.HE2X_TONES) < -16
    assert np.mean(p["he_signs"] == t["signs"]) in (0.0, 1.0)


def test_nmse_tracks_snr():
    """L-LTF NMSE should follow 1/SNR (two-symbol average: about -3 dB, plus tone/band factors)."""
    res = []
    for snr in (10, 20, 30):
        vals = []
        for seed in range(20, 26):
            iq, t = make_case(seed, snr_db=snr, cfo=30e3)
            p = w.process_capture(iq)[0]
            vals.append(aligned_nmse_db(p["H_lltf"], true_H(t, w.LEG_TONES, DF_LEG), w.LEG_TONES))
        res.append(np.mean(vals))
    assert res[0] > res[1] > res[2]
    for snr, r in zip((10, 20, 30), res):
        assert -snr - 7 < r < -snr + 1, (snr, r)


def test_noise_only_no_detection():
    rng = np.random.default_rng(5)
    iq = np.round(30 * (rng.normal(size=16380) + 1j * rng.normal(size=16380)))
    assert w.process_capture(iq) == []


def test_truncated_packet_rejected():
    iq, _ = make_case(7, snr_db=30.0, offset=16380 - 2000)
    assert w.process_capture(iq) == []


def test_snr_estimate():
    iq, t = make_case(30, snr_db=20.0)
    p = w.process_capture(iq)[0]
    assert 15 < p["snr_db"] < 26
