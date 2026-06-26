# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "matplotlib"]
# ///
"""
波形渲染器（agent CLI 全自主）：截任意時間範圍 telemetry → 精緻 PNG，供視覺判讀。
我自己選範圍(--t0/--t1)、自己產圖、自己 Read PNG；使用者也能開同一張 PNG。

用法:
  uv run sysid/plot.py <jsonl> [--t0 S] [--t1 S] [--vel] [--out PNG]
  uv run sysid/plot.py --live <秒> [--vel] [--out PNG]
  uv run sysid/plot.py 'sysid/data/dist_*.jsonl' ...     # glob 取最新
"""
import sys, json, time, glob, argparse, urllib.request
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as _fm
from matplotlib.patheffects import withStroke

# CJK 字型
_CJK = "/System/Library/Fonts/Supplemental/Arial Unicode.ttf"
try:
    _fm.fontManager.addfont(_CJK)
    _CJK_NAME = _fm.FontProperties(fname=_CJK).get_name()
except Exception:
    _CJK_NAME = None

# 配色（高彩度、深底對比）
COLORS = ["#ff6b5e", "#ffa53d", "#ffe14d", "#54e072", "#5aa9ff", "#c87bf0"]  # M1..M6
BG = "#0b0e14"; PANEL = "#11151f"; GRID = "#2a3140"; FG = "#e8edf4"; MUTED = "#8b96a8"
HOST = "localhost:3000"
SETTLE_BAND = 1.0


def load_file(path):
    t, A, H, POS, OK, ev = [], [], [], [], [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        tt = r.get("t", 0) / 1000.0
        if r.get("dir") == "out":
            d = r.get("d", "")
            if isinstance(d, str) and (d.startswith("W ") or d.startswith("U ")):
                ev.append((tt, d.split()[0]))
            continue
        if r.get("dir") != "in":
            continue
        try:
            d = json.loads(r["d"])
        except Exception:
            continue
        if "a" not in d:
            continue
        t.append(tt)
        A.append([float(x) for x in d["a"]])
        H.append([float(x) for x in d["herr"]] if isinstance(d.get("herr"), list) else [np.nan] * 6)
        POS.append(int(d.get("pos", -1)))
        OK.append(int(d.get("ok", -1)))
    return np.array(t), np.array(A), np.array(H), np.array(POS), np.array(OK), ev


def load_live(secs):
    t, A, H, OK = [], [], [], []
    t0 = time.time(); seen = set()
    print(f"即時輪詢 {secs}s ...")
    while time.time() - t0 < secs:
        try:
            with urllib.request.urlopen(f"http://{HOST}/api/latest", timeout=0.5) as r:
                d = json.loads(r.read())
        except Exception:
            time.sleep(0.02); continue
        tk = d.get("t")
        if tk in seen or "a" not in d:
            time.sleep(0.015); continue
        seen.add(tk)
        t.append(tk / 1000.0)
        A.append([float(x) for x in d["a"]])
        H.append([float(x) for x in d["herr"]] if isinstance(d.get("herr"), list) else [np.nan] * 6)
        OK.append(int(d.get("ok", -1)))
        time.sleep(0.015)
    return np.array(t), np.array(A), np.array(H), np.zeros(len(t)), np.array(OK), []


def settling_segments(t, hnorm, events):
    segs = []
    ets = [e[0] for e in events] if events else []
    for k, te in enumerate(ets):
        tend = ets[k + 1] if k + 1 < len(ets) else t[-1]
        m = (t >= te) & (t <= tend)
        if m.sum() < 3:
            continue
        tt, hh = t[m], hnorm[m]
        pk_i = int(np.argmax(hh)); peak = hh[pk_i]; t_peak = tt[pk_i]
        below = np.where(hh < SETTLE_BAND)[0]
        ts = tt[below[0]] - te if len(below) else None
        segs.append((te, ts, peak, t_peak))
    return segs


def _style_ax(ax):
    ax.set_facecolor(PANEL)
    ax.grid(True, alpha=0.5, color=GRID, lw=0.6)
    for s in ax.spines.values():
        s.set_color(GRID); s.set_linewidth(0.8)
    ax.tick_params(colors=MUTED, labelsize=8.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", nargs="?")
    ap.add_argument("--live", type=float)
    ap.add_argument("--t0", type=float)
    ap.add_argument("--t1", type=float)
    ap.add_argument("--vel", action="store_true")
    ap.add_argument("--out", default="sysid/data/plot.png")
    a = ap.parse_args()

    if a.live:
        t, A, H, POS, OK, ev = load_live(a.live)
        title_src = f"LIVE {a.live:.0f}s"
    else:
        if not a.src:
            sys.exit("需 <jsonl> 或 --live N")
        path = sorted(glob.glob(a.src))[-1] if "*" in a.src else a.src
        t, A, H, POS, OK, ev = load_file(path)
        title_src = path.split("/")[-1].replace(".jsonl", "")
        print(f"檔案 {path}")
    if len(t) == 0:
        sys.exit("無資料")

    t0abs = t[0]; t = t - t0abs
    ev = [(e[0] - t0abs, e[1]) for e in ev]
    m = np.ones(len(t), bool)
    if a.t0 is not None: m &= t >= a.t0
    if a.t1 is not None: m &= t <= a.t1
    t, A, H, OK = t[m], A[m], H[m], OK[m]
    ev = [e for e in ev if t[0] <= e[0] <= t[-1]]

    okpct = 100 * np.mean(OK == 6) if len(OK) else 0
    hnorm = np.linalg.norm(np.nan_to_num(H), axis=1)
    segs = settling_segments(t, hnorm, ev)
    peak_axis = int(np.nanargmax(np.nanmax(np.abs(H), axis=0)))
    peak_val = np.nanmax(np.abs(H))

    # ---- console ----
    print(f"樣本 {len(t)}  時長 {t[-1]-t[0]:.1f}s  ok=6 {okpct:.0f}%")
    for i in range(6):
        print(f"M{i+1}  peak|herr|={np.nanmax(np.abs(H[:,i])):6.2f}  末值={H[-1,i]:6.2f}")
    print(f"‖herr‖ peak={hnorm.max():.2f}  末值={hnorm[-1]:.2f}")
    for te, ts, pk, tp in segs:
        print(f"  擾動@{te:.1f}s peak={pk:.1f}°  settling={'%.2fs' % ts if ts else '未回band'}")

    # ---- 繪圖 ----
    plt.style.use("dark_background")
    if _CJK_NAME:
        plt.rcParams["font.family"] = _CJK_NAME
    plt.rcParams["axes.unicode_minus"] = False

    nax = 3 if a.vel else 2
    fig = plt.figure(figsize=(14, 1.4 + 2.9 * nax), facecolor=BG)
    gs = fig.add_gridspec(nax, 1, hspace=0.16, left=0.06, right=0.985, top=0.84, bottom=0.075)
    axes = [fig.add_subplot(gs[i]) for i in range(nax)]
    for ax in axes:
        _style_ax(ax)

    # 標題帶 + 指標（檔名獨佔第一行＋截斷；chips 在下方獨立行，避免重疊）
    short = title_src if len(title_src) <= 50 else title_src[:48] + "…"
    fig.text(0.06, 0.955, short, fontsize=14, color=FG, fontweight="bold", va="center")
    chips = [
        ("峰值 |herr|", f"M{peak_axis+1} {peak_val:.1f}°", COLORS[peak_axis]),
        ("‖herr‖ peak", f"{hnorm.max():.1f}°", "#5aa9ff"),
        ("CAN ok=6", f"{okpct:.0f}%", "#54e072" if okpct > 70 else "#ffa53d"),
        ("樣本·時長", f"{len(t)} · {t[-1]-t[0]:.1f}s", MUTED),
    ]
    x = 0.06
    for label, val, col in chips:
        fig.text(x, 0.908, label, fontsize=8.5, color=MUTED, va="center")
        fig.text(x + 0.072, 0.908, val, fontsize=12, color=col, fontweight="bold", va="center")
        x += 0.175

    glow = [withStroke(linewidth=2.6, foreground="#000000", alpha=0.35)]

    # ax0: herr
    ax = axes[0]
    ax.axhspan(-SETTLE_BAND, SETTLE_BAND, color="#54e072", alpha=0.06, zorder=0)
    ax.axhline(SETTLE_BAND, color="#54e072", lw=0.7, ls=":", alpha=0.45)
    ax.axhline(-SETTLE_BAND, color="#54e072", lw=0.7, ls=":", alpha=0.45)
    ax.axhline(0, color=MUTED, lw=0.7, alpha=0.5)
    for i in range(6):
        ax.plot(t, H[:, i], color=COLORS[i], lw=1.4, label=f"M{i+1}",
                solid_capstyle="round", path_effects=glow)
    leg = ax.legend(ncol=6, fontsize=8.5, loc="upper right", framealpha=0.85,
                    facecolor=PANEL, edgecolor=GRID, handlelength=1.3, columnspacing=1.1)
    for txt in leg.get_texts():
        txt.set_color(FG)
    ax.set_ylabel("herr  (°)", color=FG, fontsize=10)
    ax.set_title("回正誤差 herr — 各軸偏離鎖位點", color=FG, fontsize=10.5, pad=6, loc="left")

    # ax1: angle
    ax = axes[1]
    for i in range(6):
        ax.plot(t, A[:, i], color=COLORS[i], lw=1.3, path_effects=glow)
    ax.set_ylabel("angle  (°)", color=FG, fontsize=10)
    ax.set_title("絕對角度 — 六軸實際位置", color=FG, fontsize=10.5, pad=6, loc="left")

    # ax2: vel
    if a.vel:
        ax = axes[2]
        V = np.vstack([np.zeros((1, 6)), np.diff(A, axis=0)])
        ax.axhline(0, color=MUTED, lw=0.7, alpha=0.5)
        for i in range(6):
            ax.plot(t, V[:, i], color=COLORS[i], lw=1.1)
        ax.set_ylabel("vel  (°/cyc)", color=FG, fontsize=10)
        ax.set_title("速度 Δangle/cycle — 阻尼可視化", color=FG, fontsize=10.5, pad=6, loc="left")

    # 擾動脈衝陰影 + 事件標 + settling
    for ax in axes:
        for te, lbl in ev:
            ax.axvspan(te, te + 0.12, color="#ff5e7a", alpha=0.10, zorder=0)
            ax.axvline(te, color="#ff5e7a", ls="-", lw=1.0, alpha=0.55)
    a0 = axes[0]
    for te, ts, pk, tp in segs:
        a0.annotate("擾動", (te, a0.get_ylim()[1]), color="#ff5e7a", fontsize=8,
                    ha="left", va="top", xytext=(2, -2), textcoords="offset points")
        if ts:
            a0.axvline(te + ts, color="#54e072", ls="--", lw=1.0, alpha=0.7)
            a0.annotate(f"settling {ts:.2f}s", (te + ts, -SETTLE_BAND), color="#54e072",
                        fontsize=8.5, ha="left", va="top", fontweight="bold",
                        xytext=(3, -2), textcoords="offset points")
    axes[-1].set_xlabel("時間  t (s)", color=FG, fontsize=10)
    fig.savefig(a.out, dpi=130, facecolor=BG)
    print(f"→ {a.out}")


if __name__ == "__main__":
    main()
