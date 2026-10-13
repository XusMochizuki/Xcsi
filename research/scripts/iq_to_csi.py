"""Real ESP-SDR captures -> per-packet L-LTF / 2x HE-LTF CSI + quality metrics.

    uv run --python 3.12 --with numpy,scipy research/scripts/iq_to_csi.py runs/<capture>.npz [...]

Writes <capture>.csi.npz: H_lltf[p, 52], H_he[p, 122] (HE sign pattern aligned to the
most common one), cfo_hz, snr_db, lsig_evm, capture index. Prints detection counts, the
HE sign-pattern consistency (a decision-directed estimate is only trustworthy if every
packet decides the same +-1 sequence), CFO/SNR/EVM statistics.
"""
import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dsp"))
import wifi_ofdm as w  # noqa: E402


def reconstruct(iq, fs, min_snr=10.0, lsig_tol=3):
    """Captures [n, samples] -> dict of per-packet CSI and metrics for our transmitter's packets
    (None if none pass). Keys: n_detected, n_snr, H_lltf, H_he, he_ref_signs, he_agree,
    cfo_hz, snr_db, lsig_evm, lsig_same, cap, n_ltf."""
    recs = []
    for i, cap in enumerate(iq):
        # The C6 dump tap has inverted Q polarity (spectrum mirrored, CFO sign flipped,
        # L-LTF CIR scrambled unless conjugated; same convention note in pyespargos
        # espargos/iq_packet.py for the C61). Conjugate at the interface.
        for r in w.process_capture(np.conj(cap.astype(np.complex128)), fs):
            r["cap"] = i
            recs.append(r)
    good = [r for r in recs if r["snr_db"] >= min_snr]
    out = dict(n_detected=len(recs), n_snr=len(good))
    if not good:
        return out
    # Our TX sends a constant-length PSDU, so its L-SIG bits never change: keep packets
    # within lsig_tol bit errors (of 52 hard decisions) of the most common pattern; this
    # drops other transmitters on the channel but tolerates a few weak tones.
    bits = np.array([r["lsig_bits"] for r in good])
    pats, cnt = np.unique(bits, axis=0, return_counts=True)
    mode = pats[np.argmax(cnt)]
    keep = np.sum(bits != mode, axis=1) <= lsig_tol
    good = [r for r, k in zip(good, keep) if k]
    S = np.array([r["he_signs"] for r in good])
    ref = np.sign(np.sum(S * np.sign(S @ S[0])[:, None], axis=0))   # majority, global sign aligned
    flip = np.sign(S @ ref)
    lsig = np.array([r["lsig_bits"] for r in good])
    out.update(H_lltf=np.array([r["H_lltf"] for r in good]),
               H_he=np.array([r["H_he"] for r in good]) * flip[:, None], he_ref_signs=ref,
               he_agree=np.mean((S * flip[:, None]) == ref, axis=1),
               cfo_hz=np.array([r["cfo_hz"] for r in good]), snr_db=np.array([r["snr_db"] for r in good]),
               lsig_evm=np.array([r["lsig_evm"] for r in good]),
               lsig_same=float(np.mean(np.all(lsig == mode, axis=1))),
               cap=np.array([r["cap"] for r in good]), n_ltf=np.array([r["n_ltf"] for r in good]))
    return out


def summary_lines(r):
    if "H_he" not in r:
        return [f"{r['n_detected']} packets detected, {r['n_snr']} above the SNR threshold, none usable"]
    agree, cfo, snr, evm = r["he_agree"], r["cfo_hz"], r["snr_db"], r["lsig_evm"]
    return [f"{r['n_detected']} packets detected, {r['n_snr']} above the SNR threshold, "
            f"{len(snr)} match the majority L-SIG pattern within 3 bits (our transmitter)",
            f"HE sign pattern: {np.mean(agree == 1) * 100:.1f}% of packets identical to the majority pattern; "
            f"worst packet {agree.min() * 100:.1f}% tones agree",
            f"CFO {np.mean(cfo) / 1e3:.2f} kHz (std {np.std(cfo) / 1e3:.2f} kHz); L-LTF SNR median {np.median(snr):.1f} dB "
            f"[{snr.min():.1f}..{snr.max():.1f}]; L-SIG EVM median {np.median(evm) * 100:.1f}% "
            f"({-20 * np.log10(np.median(evm)):.1f} dB)",
            f"L-SIG hard bits exactly identical to the majority pattern: {r['lsig_same'] * 100:.1f}%"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--min-snr", type=float, default=10.0)
    args = ap.parse_args()
    for path in args.captures:
        d = np.load(path)
        r = reconstruct(d["iq"], float(d["fs"]), args.min_snr)
        print(f"\n{path}: {len(d['iq'])} captures")
        print("\n".join("  " + x for x in summary_lines(r)))
        if "H_he" in r:
            out = path.replace(".npz", ".csi.npz")
            np.savez_compressed(out, **{k: v for k, v in r.items() if isinstance(v, np.ndarray)})
            print(f"  saved {out}")


if __name__ == "__main__":
    main()
