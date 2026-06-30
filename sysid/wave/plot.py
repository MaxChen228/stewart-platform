#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib>=3.8", "numpy"]
# ///
"""wave/plot.py — wave.json → 學術規範波形疊圖（intent vs actual，馬達空間）。

依《學術圖表格式規範》：Times New Roman、刻度朝內+次刻度、虛線格線 alpha 0.3、
DPI 300 PNG、白底、ax 4.0x3.0 inch（fig 5.2x4.35）。plot_style.py 在別專案故此處內聯複刻。

預設輸出 8 張：M1..M6 逐馬達（intent + 兩組態疊圖）+ headline 馬達 zoom + 追蹤誤差。
用法：uv run plot.py <wave.json> [--out DIR] [--before TAG] [--after TAG]
                     [--headline M] [--zoom s0,s1] [--montage]
"""
import argparse, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# ── 規範 rcParams（內聯複刻 plot_style.apply_common_style）──
TNR = "/System/Library/Fonts/Supplemental/Times New Roman.ttf"
if os.path.exists(TNR):
    fm.fontManager.addfont(TNR)
    FAM = fm.FontProperties(fname=TNR).get_name()
else:
    FAM = "Times New Roman"
plt.rcParams.update({
    "font.family": "serif", "font.serif": [FAM, "Times New Roman", "DejaVu Serif"],
    "font.size": 14, "axes.titlesize": 20, "axes.labelsize": 18,
    "legend.fontsize": 14, "xtick.labelsize": 14, "ytick.labelsize": 14,
    "xtick.direction": "in", "ytick.direction": "in",
    "axes.facecolor": "white", "figure.facecolor": "white",
    "axes.linewidth": 1.2, "savefig.dpi": 300, "mathtext.fontset": "stix",
})
# FigureLayout DEFAULT_LAYOUT（物理英吋精確佈局）
AXW, AXH, ML, MR, MB, MT = 4.0, 3.0, 0.8, 0.4, 0.8, 0.55
FIGW, FIGH = ML + AXW + MR, MB + AXH + MT  # 5.2 x 4.35
POS = [ML / FIGW, MB / FIGH, AXW / FIGW, AXH / FIGH]

# 規範色：散佈系列 tab:blue/orange/green
C_INTENT = "#222222"; C_BEFORE = "tab:orange"; C_AFTER = "tab:blue"; C_MID = "tab:green"


def new_ax():
    fig = plt.figure(figsize=(FIGW, FIGH))
    ax = fig.add_axes(POS)
    return fig, ax


def style(ax):
    ax.minorticks_on()
    ax.tick_params(which="major", length=6.0, width=1.2, direction="in", top=True, right=True)
    ax.tick_params(which="minor", length=3.0, width=0.8, direction="in", top=True, right=True)
    for s in ax.spines.values():
        s.set_linewidth(1.2)
    ax.grid(True, linestyle="--", alpha=0.3)


def save(fig, path):
    fig.savefig(path, dpi=300)
    plt.close(fig)
    print("  ", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wave")
    ap.add_argument("--out", default=None)
    ap.add_argument("--before", default=None)
    ap.add_argument("--after", default=None)
    ap.add_argument("--mid", default=None)
    ap.add_argument("--headline", type=int, default=4)  # 0-based 馬達 idx（4 = M5）
    ap.add_argument("--zoom", default=None)             # "s0,s1"
    ap.add_argument("--kind", choices=["time", "fft", "both"], default="both")
    ap.add_argument("--fmax", type=float, default=8.0)  # FFT 顯示上限 Hz
    ap.add_argument("--montage", action="store_true", default=True)
    a = ap.parse_args()

    W = json.load(open(a.wave))
    t = np.array(W["tRel"], float)
    tags = list(W["configs"].keys())
    before = a.before or tags[0]
    after = a.after or tags[-1]
    mid = a.mid or (tags[1] if len(tags) > 2 else None)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.wave)), "wave_png")
    os.makedirs(out, exist_ok=True)
    hl = a.headline
    print(f"before={before}  after={after}  headline=M{hl+1}  → {out}")

    fs = 1000.0 / W["dtMs"]  # 取樣率 Hz

    def arr(tag, kind, m):
        v = np.array(W["configs"][tag][kind][m], float)
        if np.isnan(v).any():  # IK 偶有無效 → 線性內插補洞，FFT 才不破
            i = np.arange(len(v)); ok = ~np.isnan(v)
            v = np.interp(i, i[ok], v[ok])
        return v

    def spec(tag, kind, m):
        """去均值 + Hann 窗 → 單邊幅度譜（freqs Hz, mag）。"""
        x = arr(tag, kind, m); x = x - x.mean()
        w = np.hanning(len(x)); X = np.fft.rfft(x * w)
        f = np.fft.rfftfreq(len(x), 1.0 / fs)
        mag = np.abs(X) * 2.0 / np.sum(w)  # 幅度正規化（窗增益補償）
        return f, mag

    def montage(saved, name):
        if not a.montage:
            return
        imgs = [plt.imread(p) for p in saved]
        rows = (len(imgs) + 1) // 2
        fig = plt.figure(figsize=(2 * FIGW, rows * FIGH))
        for i, im in enumerate(imgs):
            ax = fig.add_subplot(rows, 2, i + 1); ax.imshow(im); ax.axis("off")
        fig.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0.02, hspace=0.02)
        pm = os.path.join(out, name); fig.savefig(pm, dpi=120); plt.close(fig)
        print("montage:", pm)

    def panel_motor(m):
        fig, ax = new_ax()
        ax.plot(t, arr(before, "intent", m), color=C_INTENT, lw=1.8, ls="--", label="Target (intent)", zorder=3)
        ax.plot(t, arr(before, "actual", m), color=C_BEFORE, lw=1.4, label=f"{before}", zorder=2)
        ax.plot(t, arr(after, "actual", m), color=C_AFTER, lw=1.4, label=f"{after}", zorder=2)
        style(ax)
        ax.set_xlabel(r"Time (s)")
        ax.set_ylabel(r"Motor angle $\theta_{%d}$ (deg)" % (m + 1))
        ax.set_title(f"M{m+1} tracking")
        ax.legend(loc="upper right", framealpha=0.9)
        return fig

    order = [before] + ([mid] if mid else []) + [after]
    cols = {before: C_BEFORE, after: C_AFTER}
    if mid: cols[mid] = C_MID

    # ── 時域：6 馬達疊圖 + headline zoom + 追蹤誤差 ──
    if a.kind in ("time", "both"):
        saved = []
        print("時域逐馬達 intent vs (before, after)：")
        for m in range(6):
            p = os.path.join(out, f"wave_M{m+1}.png")
            save(panel_motor(m), p); saved.append(p)
        if a.zoom:
            z0, z1 = map(float, a.zoom.split(","))
        else:
            mid_t = t[0] + (t[-1] - t[0]) * 0.35
            z0, z1 = mid_t, min(mid_t + 2.5, t[-1])
        fig, ax = new_ax(); sel = (t >= z0) & (t <= z1)
        ax.plot(t[sel], arr(before, "intent", hl)[sel], color=C_INTENT, lw=2.0, ls="--", label="Target (intent)")
        ax.plot(t[sel], arr(before, "actual", hl)[sel], color=C_BEFORE, lw=1.8, label=before)
        ax.plot(t[sel], arr(after, "actual", hl)[sel], color=C_AFTER, lw=1.8, label=after)
        style(ax)
        ax.set_xlabel(r"Time (s)"); ax.set_ylabel(r"Motor angle $\theta_{%d}$ (deg)" % (hl + 1))
        ax.set_title(f"M{hl+1} latency (zoom)"); ax.legend(loc="upper right", framealpha=0.9)
        pz = os.path.join(out, f"wave_zoom_M{hl+1}.png"); save(fig, pz); saved.append(pz)
        fig, ax = new_ax()
        for tag in order:
            e = arr(tag, "actual", hl) - arr(tag, "intent", hl)
            ax.plot(t, e, color=cols[tag], lw=1.3, label=f"{tag} (RMS {np.sqrt(np.mean(e**2)):.1f}°)")
        ax.axhline(0, color=C_INTENT, lw=1.0, ls=":")
        style(ax)
        ax.set_xlabel(r"Time (s)"); ax.set_ylabel(r"Tracking error $e_{%d}$ (deg)" % (hl + 1))
        ax.set_title(f"M{hl+1} error"); ax.legend(loc="upper right", framealpha=0.9)
        save(fig, os.path.join(out, f"wave_error_M{hl+1}.png")); saved.append(os.path.join(out, f"wave_error_M{hl+1}.png"))
        montage(saved, "wave_montage.png")
        print(f"時域完成：{len(saved)} 張")

    # ── 頻域：6 馬達幅度譜疊圖 + headline 傳遞函數增益 + 誤差譜 ──
    if a.kind in ("fft", "both"):
        savedF = []
        print("頻域逐馬達幅度譜 |X(f)| intent vs (before, after)：")
        for m in range(6):
            fig, ax = new_ax()
            f, mi = spec(before, "intent", m)
            _, mb = spec(before, "actual", m)
            _, ma = spec(after, "actual", m)
            ax.plot(f, mi, color=C_INTENT, lw=1.8, ls="--", label="Target (intent)", zorder=3)
            ax.plot(f, mb, color=C_BEFORE, lw=1.4, label=before, zorder=2)
            ax.plot(f, ma, color=C_AFTER, lw=1.4, label=after, zorder=2)
            style(ax); ax.set_xlim(0, a.fmax)
            ax.set_xlabel(r"Frequency (Hz)"); ax.set_ylabel(r"$|\Theta_{%d}(f)|$ (deg)" % (m + 1))
            ax.set_title(f"M{m+1} spectrum"); ax.legend(loc="upper right", framealpha=0.9)
            p = os.path.join(out, f"fft_M{m+1}.png"); save(fig, p); savedF.append(p)
        # P7: 傳遞函數增益 |H(f)|=|actual|/|intent|（headline），只在 intent 有能量處畫
        fig, ax = new_ax()
        f, mi = spec(before, "intent", hl)
        thr = 0.05 * mi.max(); band = (f <= a.fmax) & (mi > thr)
        for tag in order:
            _, mx = spec(tag, "actual", hl)
            H = np.full_like(mi, np.nan); H[band] = mx[band] / mi[band]
            ax.plot(f, H, color=cols[tag], lw=1.5, label=tag)
        ax.axhline(1.0, color=C_INTENT, lw=1.0, ls=":", label="ideal (gain=1)")
        style(ax); ax.set_xlim(0, a.fmax); ax.set_ylim(0, 3)
        ax.set_xlabel(r"Frequency (Hz)"); ax.set_ylabel(r"Gain $|H_{%d}(f)|$" % (hl + 1))
        ax.set_title(f"M{hl+1} transfer gain"); ax.legend(loc="upper right", framealpha=0.9)
        p = os.path.join(out, f"fft_gain_M{hl+1}.png"); save(fig, p); savedF.append(p)
        # P8: 誤差譜 |E(f)|（headline，全組態）
        fig, ax = new_ax()
        for tag in order:
            x = arr(tag, "actual", hl) - arr(tag, "intent", hl); x = x - x.mean()
            w = np.hanning(len(x)); E = np.abs(np.fft.rfft(x * w)) * 2.0 / np.sum(w)
            ff = np.fft.rfftfreq(len(x), 1.0 / fs)
            ax.plot(ff, E, color=cols[tag], lw=1.4, label=tag)
        style(ax); ax.set_xlim(0, a.fmax)
        ax.set_xlabel(r"Frequency (Hz)"); ax.set_ylabel(r"$|E_{%d}(f)|$ (deg)" % (hl + 1))
        ax.set_title(f"M{hl+1} error spectrum"); ax.legend(loc="upper right", framealpha=0.9)
        p = os.path.join(out, f"fft_error_M{hl+1}.png"); save(fig, p); savedF.append(p)
        montage(savedF, "fft_montage.png")
        print(f"頻域完成：{len(savedF)} 張")


if __name__ == "__main__":
    main()
