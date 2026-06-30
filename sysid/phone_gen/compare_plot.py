#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib>=3.8", "numpy", "scipy"]
# ///
"""compare_plot.py — 真實手機 capture vs generator 的時域/頻域疊圖（學術規範）。

rel = 濾波前手部角（roll/pitch/yaw）。real 重採樣到 60Hz；gen 本就 60Hz。
時域：同窗 real vs gen 疊圖（不同 realization，看統計同性=幅度/爆發/時間尺度）。
頻域：Welch PSD loglog 疊圖（同譜 → 量化對齊）。
用法：uv run compare_plot.py <real_cap.jsonl> <gen.jsonl> [--out DIR] [--win s0,s1]
"""
import argparse, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from scipy.signal import welch

TNR = "/System/Library/Fonts/Supplemental/Times New Roman.ttf"
if os.path.exists(TNR):
    fm.fontManager.addfont(TNR); FAM = fm.FontProperties(fname=TNR).get_name()
else:
    FAM = "Times New Roman"
plt.rcParams.update({
    "font.family": "serif", "font.serif": [FAM, "Times New Roman", "DejaVu Serif"],
    "font.size": 13, "axes.titlesize": 17, "axes.labelsize": 15,
    "legend.fontsize": 12, "xtick.labelsize": 12, "ytick.labelsize": 12,
    "xtick.direction": "in", "ytick.direction": "in",
    "axes.facecolor": "white", "figure.facecolor": "white",
    "axes.linewidth": 1.2, "savefig.dpi": 300, "mathtext.fontset": "stix",
})
C_REAL = "tab:orange"; C_GEN = "tab:blue"
AX = ["Roll", "Pitch", "Yaw"]
FS = 60.0


def style(ax):
    ax.minorticks_on()
    ax.tick_params(which="major", length=6.0, width=1.2, direction="in", top=True, right=True)
    ax.tick_params(which="minor", length=3.0, width=0.8, direction="in", top=True, right=True)
    for s in ax.spines.values():
        s.set_linewidth(1.2)
    ax.grid(True, linestyle="--", alpha=0.3)


def load(path, typ):
    t, rel = [], []
    with open(path) as f:
        for line in f:
            try:
                j = json.loads(line)
            except Exception:
                continue
            if j.get("type") == typ and "rel" in j:
                t.append(j["t"]); rel.append(j["rel"])
    return np.array(t) / 1000.0, np.array(rel)  # s, [N,3]


def resample(t, y, fs):
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    return tu, np.stack([np.interp(tu, t, y[:, k]) for k in range(3)], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("real"); ap.add_argument("gen")
    ap.add_argument("--out", default=os.path.dirname(__file__))
    ap.add_argument("--win", default="40,70")
    a = ap.parse_args()
    s0, s1 = (float(x) for x in a.win.split(","))

    tr, rr = load(a.real, "imu"); tr, rr = resample(tr, rr, FS)
    tg, rg = load(a.gen, "pf")     # gen 已 60Hz uniform
    # gen t 從 0 起；real t 也從 ~0 起（相對）
    tr = tr - tr[0]; tg = tg - tg[0]

    fig, axes = plt.subplots(3, 2, figsize=(11.5, 9.2))
    fig.subplots_adjust(left=0.085, right=0.975, bottom=0.085, top=0.93, hspace=0.38, wspace=0.24)

    for k in range(3):
        # ── 時域 ──
        axt = axes[k, 0]
        mr = (tr >= s0) & (tr <= s1); mg = (tg >= s0) & (tg <= s1)
        axt.plot(tr[mr], rr[mr, k], color=C_REAL, lw=1.1, label="Real phone")
        axt.plot(tg[mg], rg[mg, k], color=C_GEN, lw=1.1, alpha=0.85, label="Generator")
        style(axt)
        axt.set_ylabel(f"{AX[k]}  (deg)")
        if k == 0:
            axt.set_title("Time domain", fontsize=15)
            axt.legend(loc="upper right", frameon=True, framealpha=0.9, ncol=2)
        if k == 2:
            axt.set_xlabel("Time  (s)")

        # ── 頻域 (Welch PSD) ──
        axf = axes[k, 1]
        nper = min(2048, len(rr) // 2, len(rg) // 2)
        fr, pr = welch(rr[:, k] - rr[:, k].mean(), fs=FS, nperseg=nper, window="hann")
        fg, pg = welch(rg[:, k] - rg[:, k].mean(), fs=FS, nperseg=nper, window="hann")
        axf.loglog(fr, pr, color=C_REAL, lw=1.3, label="Real phone")
        axf.loglog(fg, pg, color=C_GEN, lw=1.3, alpha=0.85, label="Generator")
        style(axf)
        axf.set_xlim(0.02, 30)
        axf.set_ylabel("PSD  (deg$^2$/Hz)")
        if k == 0:
            axf.set_title("Frequency domain", fontsize=15)
            axf.legend(loc="upper right", frameon=True, framealpha=0.9)
        if k == 2:
            axf.set_xlabel("Frequency  (Hz)")

    fig.suptitle("Phone hand-motion (rel): real capture vs generator", fontsize=18, y=0.985)
    out = os.path.join(a.out, "compare_real_vs_gen.png")
    fig.savefig(out)
    print(out)


if __name__ == "__main__":
    main()
