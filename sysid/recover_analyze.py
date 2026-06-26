# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy"]
# ///
"""
Push-Recovery 健全分析：讀 recover_*.jsonl，量「手推擾動後回正」的衰減特性，
並跨迴圈頻率（L=20/10/6ms）比較——回答「迴圈加速到底治不治發散」。

設計原則（全部事前登記，避免事後挑數字的偏差）：
  • 主訊號 = 純編碼器位移 angdev = ‖a − a_rest‖（任何控制模式都有、免 FK、單位齊一=度）
      a_rest = 整段 a 的中位數（平台多數時間在靜止位 → 中位數=靜止姿態）
  • 次訊號 = pose error ‖err6‖（僅 mode-1/Task-space 有；存在時一併報，當交叉驗證）
  • 主指標 = 主導衰減率 σ（對 |位移| 衰減段擬指數上包絡 → σ=1/τ）
      —— 振幅無關 → 不怕你每次推力道不同；振盪/過阻尼都有定義
  • 次指標 = 阻尼比 ζ / 阻尼頻率 ωd（僅在真有振盪 ≥2 峰時才算；norm 會整流→ωd 偏粗，僅參考）
  • settling 從「峰值」起算（消掉你按開始/推之間的計時誤差），不反應期去重
  • 剔除兩類無效推動：過弱（峰值沒過雜訊）、觸底（峰值平頂=撞到支架）

⚠️ 有效性邊界（務必連同結論一起讀）：
  支架 offload 重力、加了剛度/阻尼 → 「絕對」σ 只描述「被支撐的 plant」，不可外推到無支撐。
  能外推的是「趨勢 σ(迴圈速率)」——那是迴圈本身的性質，較不受支撐污染。所以掃頻才是重點。

用法: uv run sysid/recover_analyze.py 'sysid/data/recover_*.jsonl'
"""
import sys, json, glob, re
import numpy as np

# ===== 事前登記常數（分析前定死，不依資料回調）=====
NOISE_K       = 6.0    # onset 門檻 = baseline + K·noise
REFRACTORY_S  = 1.5    # 兩次 onset 最小間隔（一次推≠多次偵測）
MAX_WIN_S     = 5.0    # 單次推的最長觀察窗
MIN_PEAK_K    = 8.0    # 峰值 < baseline+K·noise → 推太弱，剔除
SAT_FRAC      = 0.02   # 峰值平頂判定：落在峰值 ±2% 內
SAT_N         = 4      # 連續 ≥N 點平頂 → 疑似觸底（撞支架），標記
SETTLE_BAND_K = 3.0    # 回到 baseline+K·noise 內算「穩」
SETTLE_HOLD_S = 0.5    # 須連續停留這麼久才算 settled
OSC_PROM_K    = 1.0    # 振盪峰最小突起 = K·noise
MIN_FIT_PTS   = 4      # 擬合最少點數
FLOOR_FRAC    = 0.10   # σ 擬合只取「峰值→10%峰值」段（避開地板尾巴）
FIT_NOISE_K   = 3.0    # 擬合點下限 = K·noise


def load(path):
    """回傳 dict：t[], en[]=angdev(主), ern[]=errnorm(次,可能全 nan), cm[], fki[]。"""
    t, A, ern, cm, fki = [], [], [], [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        if r.get("dir") != "in":
            continue
        try:
            d = json.loads(r["d"])
        except Exception:
            continue
        if "a" not in d or d.get("pos") != 1:   # 只取控制啟用中的遙測
            continue
        t.append(r["t"] / 1000.0)
        A.append([float(x) for x in d["a"]])
        ern.append(float(np.linalg.norm(d["err"])) if "err" in d else float("nan"))
        cm.append(int(d.get("cm", -1)))
        fki.append(int(d.get("fki", -1)))
    A = np.array(A) if A else np.zeros((0, 6))
    a_rest = np.median(A, axis=0) if len(A) else np.zeros(6)
    en = np.linalg.norm(A - a_rest, axis=1) if len(A) else np.array([])
    return {"t": np.array(t), "en": en, "ern": np.array(ern),
            "cm": np.array(cm), "fki": np.array(fki), "a_rest": a_rest}


def quiet_floor(en):
    """穩健的靜止地板：用低於中位數那半的 MAD 估雜訊（推動的尖峰不污染）。"""
    med = np.median(en)
    low = en[en <= med]
    if len(low) < 3:
        return float(med), max(1e-3, float(np.std(en)))
    noise = 1.4826 * np.median(np.abs(low - np.median(low)))
    return float(med), float(max(noise, 1e-3))


def detect_onsets(t, en, base, noise):
    """en 升越 base+K·noise 的時刻 = 一次推；反應期去重。"""
    thr = base + NOISE_K * noise
    above = en > thr
    onsets, last_t = [], -1e9
    for i in range(1, len(en)):
        if above[i] and not above[i - 1] and (t[i] - last_t) >= REFRACTORY_S:
            onsets.append(i)
            last_t = t[i]
    return onsets


def local_peaks(y, prom):
    return [i for i in range(1, len(y) - 1)
            if y[i] >= y[i - 1] and y[i] > y[i + 1] and y[i] > prom]


def _loglin(tt, yy):
    tt = tt - tt[0]
    b, a = np.polyfit(tt, yy, 1)
    pred = a + b * tt
    ss_res = np.sum((yy - pred) ** 2)
    ss_tot = np.sum((yy - yy.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0
    return float(-b), float(r2)


def fit_sigma(t, sig, base, noise):
    """主導衰減率 σ：優先擬上包絡（峰連峰，振盪/整流都不影響落點）；
    單調衰減退回直接擬。只取「峰值→10%峰值」段避開地板尾巴。回 (σ,r2,npts,method)。"""
    y = sig - base
    ypk = float(np.max(y)) if len(y) else 0.0
    thr = max(FIT_NOISE_K * noise, FLOOR_FRAC * ypk)
    env = sorted(set([0] + local_peaks(y, thr)))
    env = [i for i in env if y[i] > thr]
    if len(env) >= 3:
        env = np.array(env)
        s, r2 = _loglin(t[env], np.log(y[env]))
        return s, r2, len(env), "env"
    mask = y > thr
    if mask.sum() < MIN_FIT_PTS:
        return None, None, int(mask.sum()), "raw"
    s, r2 = _loglin(t[mask], np.log(y[mask]))
    return s, r2, int(mask.sum()), "raw"


def damping(t, sig, base, noise, peak_rel):
    """峰後找振盪峰 → log-decrement ζ + ωd。無 ≥2 峰回 (None,None)。"""
    y = sig[peak_rel:] - base
    tt = t[peak_rel:]
    pk = local_peaks(y, OSC_PROM_K * noise)
    if len(pk) < 2:
        return None, None
    amps, times = y[pk], tt[pk]
    deltas = [np.log(amps[k] / amps[k + 1]) for k in range(len(amps) - 1) if amps[k + 1] > 0]
    Td = np.mean(np.diff(times))
    if not deltas or Td <= 0:
        return None, None
    delta = float(np.mean(deltas))
    zeta = delta / np.sqrt(4 * np.pi ** 2 + delta ** 2)
    return float(zeta), float(2 * np.pi / Td)


def analyze_push(t, en, ern, cm, on, nxt, base, noise):
    w_end = nxt if nxt is not None else len(t)
    tmax = t[on] + MAX_WIN_S
    while w_end - 1 > on and t[w_end - 1] > tmax:
        w_end -= 1
    sl = slice(on, w_end)
    tt, ww = t[sl], en[sl]
    if len(ww) < MIN_FIT_PTS + 2:
        return {"reject": "太短"}

    pk_rel = int(np.argmax(ww))
    peak = float(ww[pk_rel])
    lbase = float(np.median(ww[:max(2, pk_rel)])) if pk_rel >= 2 else base

    if peak < lbase + MIN_PEAK_K * noise:
        return {"reject": "過弱(sub-noise)"}

    near = np.abs(ww - peak) <= SAT_FRAC * peak
    saturated = bool(near.sum() >= SAT_N and
                     np.max(np.where(near)) - np.min(np.where(near)) <= 2 * SAT_N)
    erw = ern[sl]
    has_err = bool(np.isfinite(erw).any())
    fk_fallback = bool(has_err and np.any(cm[sl] == 0))   # 僅 mode-1 cm 掉 0 才算 fallback

    dt, de = tt[pk_rel:], ww[pk_rel:]
    sigma, r2, npts, _ = fit_sigma(dt, de, lbase, noise)

    # 次訊號 errnorm（mode-1 才有）作交叉驗證
    sigma_err = None
    if np.isfinite(erw).all() and len(erw) > pk_rel + MIN_FIT_PTS:
        eb = float(np.median(erw[:max(2, pk_rel)])) if pk_rel >= 2 else 0.0
        sigma_err, _, _, _ = fit_sigma(dt, erw[pk_rel:], eb, max(noise, 1e-3))

    zeta, wd = damping(tt, ww, lbase, noise, pk_rel)

    band = lbase + SETTLE_BAND_K * noise
    settle, i = None, pk_rel
    while i < len(ww):
        if ww[i] <= band:
            j = i
            while j < len(ww) and ww[j] <= band:
                j += 1
            if tt[min(j, len(tt) - 1)] - tt[i] >= SETTLE_HOLD_S:
                settle = float(tt[i] - tt[pk_rel])
                break
            i = j
        else:
            i += 1

    final = float(np.mean(ww[-max(1, len(ww) // 10):]))

    if sigma is None:
        verdict = "資料不足"
    elif sigma <= 0:
        verdict = "發散❌"
    elif settle is not None:
        verdict = "收斂✅"
    else:
        verdict = "邊際〜"

    return {"peak": round(peak, 3), "sigma": _r(sigma), "r2": _r(r2),
            "sigma_err": _r(sigma_err), "zeta": _r(zeta), "wd": _r(wd),
            "settle_s": _r(settle), "final": round(final, 3),
            "npts": npts, "saturated": saturated, "fk_fallback": fk_fallback,
            "verdict": verdict, "t0": round(float(t[on]), 1)}


def _r(x, n=3):
    return None if x is None else round(float(x), n)


def analyze_file(path):
    g = load(path)
    if len(g["t"]) < 20:
        return None
    t, en = g["t"], g["en"]
    base, noise = quiet_floor(en)
    onsets = detect_onsets(t, en, base, noise)
    pushes = []
    for k, on in enumerate(onsets):
        nxt = onsets[k + 1] if k + 1 < len(onsets) else None
        pushes.append(analyze_push(t, en, g["ern"], g["cm"], on, nxt, base, noise))
    valid = [p for p in pushes if "reject" not in p and p["sigma"] is not None]
    rejected = [p["reject"] for p in pushes if "reject" in p]
    has_err = bool(np.isfinite(g["ern"]).any())
    return {"base": round(base, 3), "noise": round(noise, 4),
            "n_onset": len(onsets), "valid": valid, "rejected": rejected,
            "dur": round(float(t[-1] - t[0]), 1), "has_err": has_err}


def agg(valid, key):
    xs = [p[key] for p in valid if p.get(key) is not None]
    return np.array(xs) if xs else np.array([])


def _f(x): return "—" if x is None else (f"{x:.3f}" if abs(x) < 100 else f"{x:.0f}")
def _med(a): return None if len(a) == 0 else round(float(np.median(a)), 2)
def _iqr(a): return "—" if len(a) < 2 else f"{np.percentile(a,25):.2f}–{np.percentile(a,75):.2f}"


def main():
    pat = sys.argv[1] if len(sys.argv) > 1 else "sysid/data/recover_*.jsonl"
    files = sorted(glob.glob(pat))
    if not files:
        print(f"找不到檔案: {pat}")
        sys.exit(1)

    summary = []
    for f in files:
        name = f.split("/")[-1]
        r = analyze_file(f)
        print("\n" + "=" * 70)
        print(f"📄 {name}")
        if r is None:
            print("   資料不足（<20 筆控制啟用遙測）")
            continue
        mode = "mode-1(含err交叉驗證)" if r["has_err"] else "mode-0(純編碼器)"
        print(f"   {mode}  基線={r['base']}°  雜訊={r['noise']}°  時長={r['dur']}s  "
              f"偵測 {r['n_onset']} 推，有效 {len(r['valid'])}，剔除 {len(r['rejected'])}")
        if r["rejected"]:
            from collections import Counter
            print(f"   剔除原因：{dict(Counter(r['rejected']))}")
        if r["valid"]:
            print(f"   {'t0':>6} {'峰值°':>7} {'σ(1/s)':>7} {'r²':>5} {'σ_err':>6} "
                  f"{'ζ':>5} {'ωd':>5} {'settle':>6} {'判定':>7}  旗標")
            for p in r["valid"]:
                flags = []
                if p["saturated"]: flags.append("觸底?")
                if p["fk_fallback"]: flags.append("FKfallback")
                print(f"   {p['t0']:>6} {p['peak']:>7} {_f(p['sigma']):>7} {_f(p['r2']):>5} "
                      f"{_f(p['sigma_err']):>6} {_f(p['zeta']):>5} {_f(p['wd']):>5} "
                      f"{_f(p['settle_s']):>6} {p['verdict']:>7}  {' '.join(flags)}")
            sig = agg(r["valid"], "sigma")
            stl = agg(r["valid"], "settle_s")
            conv = sum(1 for p in r["valid"] if p["verdict"] == "收斂✅") / len(r["valid"])
            sig_med = float(np.median(sig)) if len(sig) else float("nan")
            print(f"   ── 中位 σ={sig_med:.3f} (IQR {_iqr(sig)})  "
                  f"中位 settle={_med(stl)}s  收斂率={conv*100:.0f}%")
            mh = re.search(r"(\d+)Hz", name)
            mm = re.search(r"_m(\d+)_", name)
            if mh:
                md = int(mm.group(1)) if mm else -1
                summary.append((md, int(mh.group(1)), sig_med, _med(stl), conv, len(r["valid"])))

    modes = sorted(set(md for md, *_ in summary))
    if any(len([x for x in summary if x[0] == md]) >= 2 for md in modes):
        print("\n" + "=" * 70)
        print("🔬 掃頻結論：σ（衰減率，越大=回正越快越穩）vs 迴圈頻率")
        for md in modes:
            rows = sorted([x for x in summary if x[0] == md], key=lambda x: x[1])
            label = {0: "mode 0 joint-space", 1: "mode 1 task-space PD"}.get(md, "未標模式")
            print(f"\n   ── {label} ──")
            print(f"   {'Hz':>5} {'中位σ(1/s)':>11} {'settle(s)':>10} {'收斂率':>7} {'N':>4}")
            for _, hz, s, st, c, n in rows:
                print(f"   {hz:>5} {s:>11.3f} {str(st):>10} {c*100:>6.0f}% {n:>4}")
            sigs = [s for _, _, s, _, _, _ in rows if not np.isnan(s)]
            hzs = [hz for _, hz, s, _, _, _ in rows if not np.isnan(s)]
            if len(sigs) >= 2:
                trend = np.polyfit(hzs, sigs, 1)[0]
                span = (max(sigs) - min(sigs)) / (abs(np.mean(sigs)) + 1e-6)
                if trend > 0 and span > 0.25:
                    print("   → σ 隨迴圈變快而增大：頻寬是真槓桿（bandwidth-limited）→ 換 TWAI/1M 有理。")
                elif span <= 0.25:
                    print("   → σ 跨速率持平：迴圈速度非槓桿，問題在控制律/機械 → 換硬體幫助有限。")
                else:
                    print("   → σ 隨迴圈變快而下降：異常，先查高速迴圈丟幀/時序。")
        print("\n   ⚠️ 有效性邊界：絕對 σ 只描述『被支撐的 plant』，不可外推到無支撐；")
        print("     可外推的是趨勢 σ(迴圈)。若 σ 已健康且持平、但無支撐仍垮 →")
        print("     是支撐遮蔽了重力負載/力矩權限問題，迴圈速度與 PD 調參都治不了。")
    elif summary:
        print("\n（單一頻率/模式；要回答頻寬問題需跑完整掃頻）")


if __name__ == "__main__":
    main()
