#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib>=3.8", "numpy"]
# ///
"""family_plot.py — 證明「看起來不像」是 realization/選窗，非模型差。
上排=真實的 4 個獨立窗、下排=生成的 4 個獨立窗（同軸同尺度）。真實彼此之間
也一樣「不像」→ 它們是同一族。用法：uv run family_plot.py <real> <gen> [--axis 0|1|2]
"""
import argparse, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

TNR = "/System/Library/Fonts/Supplemental/Times New Roman.ttf"
if os.path.exists(TNR):
    fm.fontManager.addfont(TNR); FAM = fm.FontProperties(fname=TNR).get_name()
else:
    FAM = "Times New Roman"
plt.rcParams.update({
    "font.family": "serif", "font.serif": [FAM, "Times New Roman", "DejaVu Serif"],
    "font.size": 12, "axes.titlesize": 13, "axes.labelsize": 13,
    "xtick.labelsize": 11, "ytick.labelsize": 11,
    "xtick.direction": "in", "ytick.direction": "in",
    "axes.facecolor": "white", "figure.facecolor": "white",
    "axes.linewidth": 1.2, "savefig.dpi": 300, "mathtext.fontset": "stix",
})
AXN = ["Roll", "Pitch", "Yaw"]
FS = 60.0


def style(ax):
    ax.minorticks_on()
    ax.tick_params(which="major", length=5.0, width=1.1, direction="in", top=True, right=True)
    ax.tick_params(which="minor", length=2.5, width=0.7, direction="in", top=True, right=True)
    for s in ax.spines.values():
        s.set_linewidth(1.1)
    ax.grid(True, linestyle="--", alpha=0.3)


def load(path, typ, axis):
    t, v = [], []
    with open(path) as f:
        for line in f:
            try:
                j = json.loads(line)
            except Exception:
                continue
            if j.get("type") == typ and "rel" in j:
                t.append(j["t"]); v.append(j["rel"][axis])
    t = np.array(t) / 1000.0; v = np.array(v)
    tu = np.arange(t[0], t[-1], 1.0 / FS)
    return tu - tu[0], np.interp(tu, t, v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("real"); ap.add_argument("gen")
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--out", default=os.path.dirname(__file__))
    ap.add_argument("--seg", type=float, default=14.0)
    a = ap.parse_args()
    tr, vr = load(a.real, "imu", a.axis)
    tg, vg = load(a.gen, "pf", a.axis)
    seg = a.seg
    # 4 個不重疊窗（各自序列內均分）
    def starts(t, n):
        hi = t[-1] - seg
        return np.linspace(hi * 0.05, hi * 0.95, n)
    sr = starts(tr, 4); sg = starts(tg, 4)
    ylim = np.nanpercentile(np.abs(vr), 99.5) * 1.1
    ylim = max(ylim, np.nanpercentile(np.abs(vg), 99.5) * 1.1)

    fig, axes = plt.subplots(2, 4, figsize=(15, 6.2), sharey=True)
    fig.subplots_adjust(left=0.06, right=0.99, bottom=0.1, top=0.9, hspace=0.32, wspace=0.12)
    for c in range(4):
        for row, (t, v, s, col, tag) in enumerate([
            (tr, vr, sr[c], "tab:orange", "REAL"),
            (tg, vg, sg[c], "tab:blue", "GEN"),
        ]):
            ax = axes[row, c]
            m = (t >= s[()]) & (t <= s[()] + seg) if np.ndim(s) else (t >= s) & (t <= s + seg)
            ax.plot(t[m] - t[m][0], v[m], color=col, lw=1.0)
            style(ax)
            ax.set_ylim(-ylim, ylim)
            if c == 0:
                ax.set_ylabel(f"{tag}\n{AXN[a.axis]} (deg)")
            ax.set_title(f"window {c+1}", fontsize=11)
            if row == 1:
                ax.set_xlabel("Time (s)")
    fig.suptitle(f"Same family? — 4 independent REAL windows (top) vs 4 GEN windows (bottom), {AXN[a.axis]}",
                 fontsize=15, y=0.97)
    out = os.path.join(a.out, "family_real_vs_gen.png")
    fig.savefig(out)
    print(out)


if __name__ == "__main__":
    main()
