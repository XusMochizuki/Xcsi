"""Overview plots of an unpacked run (output of csi_unpack.py).

    uv run --python 3.12 --with numpy,matplotlib tools/csi_plot.py runs/NAME/npy [--sc 20] [--out x.png] [--show]

Panels:
  1  amplitude of subcarrier --sc on every device over time; hollow dots = interpolated frames
  2  missing frames per device (raster) and total loss %
  3  |H| heatmap time x subcarrier, one per device, shared colour scale
  4  mean amplitude spectrum +-1 std | RSSI | AGC gain
  5  phase difference between device pairs at --sc, received-by-both frames only
"""
import argparse
import itertools
import json
import os

import matplotlib
import numpy as np

# Categorical slots 1..3 of the validated palette (fixed per device, same as csi_web.html).
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
ORDER = ["98:88:E0:76:5D:68", "98:88:E0:76:07:C4", "98:88:E0:76:4D:04"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e6e5e0"


def color_of(macs):
    out, spare = [], iter(COLORS[len(ORDER):])
    for m in macs:
        out.append(COLORS[ORDER.index(m)] if m in ORDER else next(spare, INK2))
    return out


def style():
    matplotlib.rcParams.update({
        "axes.edgecolor": "#b9b8b1", "axes.labelcolor": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
        "axes.titlelocation": "left", "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
        "axes.spines.top": False, "axes.spines.right": False, "xtick.color": INK2, "ytick.color": INK2,
        "font.size": 9, "legend.frameon": False, "lines.linewidth": 1.2, "figure.facecolor": "#fcfcfb",
        "axes.facecolor": "#fcfcfb", "text.color": INK,
    })


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npy", help="directory written by csi_unpack.py")
    ap.add_argument("--sc", type=int, default=20, help="subcarrier for the time-series panels (-122..122)")
    ap.add_argument("--out", help="PNG path (default NPY/../analysis_sc<SC>.png)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    L = lambda n: np.load(os.path.join(args.npy, n + ".npy"))  # noqa: E731
    csi, valid, ts = L("csi"), L("valid"), L("ts_us")
    rssi, agc = L("rssi"), L("agc")
    sc, tones = L("subcarrier"), L("tone_mask")
    with open(os.path.join(args.npy, "info.json")) as f:
        info = json.load(f)
    macs = [d["mac"] for d in info["devices"]]
    labels = [d["label"] or d["mac"][-5:] for d in info["devices"]]
    colors = color_of(macs)
    D, T, _ = csi.shape
    k = int(np.flatnonzero(sc == args.sc)[0])
    if not tones[k]:
        raise SystemExit(f"--sc {args.sc} is not a measured HE20 tone (|k| must be 2..122)")

    t = (ts[0] - ts[0, 0]) / 1e6                       # seconds, device 0's clock
    amp = np.abs(csi)
    rate = (T - 1) / t[-1] if T > 1 and t[-1] > 0 else 100.0

    style()
    fig = plt.figure(figsize=(15, 16), layout="constrained")
    gs = fig.add_gridspec(5, D, height_ratios=[1.2, 0.5, 1.3, 1.0, 0.9])
    fig.suptitle(f"{os.path.basename(os.path.dirname(os.path.abspath(args.npy)))}  ·  {T} packets, "
                 f"{t[-1]:.1f} s, ~{rate:.0f} Hz  ·  interp={info['interp']}", x=0.01, ha="left", fontsize=11)

    # 1. amplitude at the selected subcarrier
    ax = fig.add_subplot(gs[0, :])
    for i in range(D):
        ax.plot(t, amp[i, :, k], color=colors[i], label=labels[i])
        miss = ~valid[i]
        if miss.any():
            ax.scatter(t[miss], amp[i, miss, k], s=10, facecolors="none", edgecolors=colors[i], linewidths=0.8)
    ax.set_title(f"|H| at subcarrier {args.sc}  (hollow = interpolated)")
    ax.set_ylabel("amplitude (raw)")
    ax.legend(loc="upper right", ncols=D)
    ax_t = ax

    # 2. loss raster
    ax = fig.add_subplot(gs[1, :], sharex=ax_t)
    for i in range(D):
        ax.eventplot(t[~valid[i]], lineoffsets=i, linelengths=0.6, colors=colors[i], linewidths=0.8)
        ax.text(1.002, i, f"{100 * (~valid[i]).mean():.2f}%", transform=ax.get_yaxis_transform(),
                va="center", color=INK2)
    ax.set_yticks(range(D), labels)
    ax.set_ylim(D - 0.5, -0.5)
    ax.set_title("missing frames (each tick = one lost tx_seq)")
    ax.set_xlabel("time [s]")

    # 3. heatmaps
    vmax = np.percentile(amp[:, :, tones], 99.5)
    sc_t = sc[tones]
    for i in range(D):
        ax = fig.add_subplot(gs[2, i])
        im = ax.imshow(amp[i][:, tones].T, aspect="auto", origin="lower", cmap="Blues", vmin=0, vmax=vmax,
                       extent=[t[0], t[-1], sc_t[0], sc_t[-1]], interpolation="nearest")
        ax.set_title(f"{labels[i]}: |H|")
        ax.set_xlabel("time [s]")
        if i == 0:
            ax.set_ylabel("subcarrier")
        ax.grid(False)
    fig.colorbar(im, ax=fig.axes[-D:], shrink=0.9, label="amplitude")

    # 4. spectrum / RSSI / AGC -- separate panels, never two y-scales on one axis
    sub = gs[3, :].subgridspec(1, 3)
    ax = fig.add_subplot(sub[0])
    for i in range(D):
        a = np.where(valid[i][:, None], amp[i], np.nan)[:, tones]   # received frames only
        m, s = np.nanmean(a, 0), np.nanstd(a, 0)
        for seg in (sc_t < 0, sc_t > 0):
            ax.plot(sc_t[seg], m[seg], color=colors[i], label=labels[i] if seg[0] else None)
            ax.fill_between(sc_t[seg], (m - s)[seg], (m + s)[seg], color=colors[i], alpha=0.15, linewidth=0)
    ax.axvline(args.sc, color=INK2, linewidth=0.8, linestyle="--")
    ax.set_title("mean |H| +-1 std (received frames)")
    ax.set_xlabel("subcarrier")
    ax.legend()
    for j, (arr, name, unit) in enumerate([(rssi, "RSSI", "dBm"), (agc, "AGC gain", "code")], start=1):
        ax = fig.add_subplot(sub[j], sharex=ax_t)
        for i in range(D):
            ax.plot(t, arr[i], color=colors[i], label=labels[i])
        ax.set_title(name)
        ax.set_ylabel(unit)
        ax.set_xlabel("time [s]")

    # 5. pairwise phase difference at the selected subcarrier
    pairs = list(itertools.combinations(range(D), 2))
    sub = gs[4, :].subgridspec(1, max(len(pairs), 1))
    for j, (a, b) in enumerate(pairs):
        ax = fig.add_subplot(sub[j], sharex=ax_t)
        both = valid[a] & valid[b]
        z = csi[a, both, k] * np.conj(csi[b, both, k])
        m = np.mean(z / np.maximum(np.abs(z), 1e-9)) if z.size else 1
        mu = np.degrees(np.angle(m))
        # Wrap around the circular mean so a pair sitting near +-180 deg isn't split in two.
        d = mu + np.degrees(np.angle(z * np.conj(m)))
        ax.scatter(t[both], d, s=2, color=INK2, linewidths=0)
        ax.set_title(f"∠{labels[a]} − ∠{labels[b]}  (mean {mu:.1f}°, R {abs(m):.2f})")
        ax.set_ylim(mu - 180, mu + 180)
        ax.set_yticks(np.round(mu) + np.array([-180, -90, 0, 90, 180]))
        ax.set_xlabel("time [s]")
        if j == 0:
            ax.set_ylabel(f"phase diff @ sc {args.sc} [deg]")

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.npy)), f"analysis_sc{args.sc}.png")
    fig.savefig(out, dpi=110)
    print(f"-> {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
