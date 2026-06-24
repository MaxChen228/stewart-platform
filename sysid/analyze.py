# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "matplotlib"]
# ///
"""
Phase 0 系統指紋分析器。
讀 server.js 記錄器吐的 JSONL，算出「平台指紋」：
  - host 迴圈週期分布（含警告：host 時間戳有序列緩衝抖動，<20ms 延遲不可信）
  - 掉幀率（ok/6）與每顆馬達讀取失敗次數
  - 靜止雜訊地板（每顆 a[i] 的 std / 峰峰值）—— host-only 即準
  - CAN 劣化（tec/rec/eflg 隨時間）
  - 編碼器量化 LSB 驗證
若記錄含 P 指令（激勵），額外做 step/chirp 對齊（reflash 後才有意義）。

用法:  uv run sysid/analyze.py sysid/data/<file>.jsonl
"""
import sys, json, math
from pathlib import Path
import numpy as np

INT64_MIN = -9223372036854775808  # 編碼器讀取失敗 sentinel
LSB_DEG = 360.0 / 16384.0          # 14-bit 編碼器一個 count 的角度


def load(path):
    meta, tele, cmds = {}, [], []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        rec = json.loads(line)
        if "meta" in rec:
            meta = rec["meta"]
            continue
        if rec.get("dir") == "cmd":
            cmds.append((rec["t"], rec["d"]))
            continue
        if rec.get("dir") == "in":
            try:
                d = json.loads(rec["d"])
            except Exception:
                continue
            if "a" in d:  # 只留遙測行
                d["_t"] = rec["t"]
                tele.append(d)
    return meta, tele, cmds


def pct(a, p):
    return float(np.percentile(a, p)) if len(a) else float("nan")


def analyze(path):
    meta, tele, cmds = load(path)
    n = len(tele)
    if n < 2:
        print(f"資料不足：只有 {n} 筆遙測")
        return
    t = np.array([d["_t"] for d in tele])             # host ms
    a = np.array([d["a"] for d in tele], dtype=float)  # n×6 角度
    r = np.array([d["r"] for d in tele], dtype=object)  # n×6 raw（可能含 INT64_MIN）
    ok = np.array([d.get("ok", 0) for d in tele])
    tec = np.array([d.get("tx", 0) for d in tele])
    rec = np.array([d.get("rx", 0) for d in tele])
    eflg = np.array([d.get("ef", 0) for d in tele])

    dur = (t[-1] - t[0]) / 1000.0
    fp = {"file": Path(path).name, "meta": meta, "samples": n, "duration_s": round(dur, 2)}

    # ---- 迴圈週期（host 端，含抖動警告）----
    dt = np.diff(t)
    fp["loop_period_ms"] = {
        "mean": round(float(dt.mean()), 2), "std": round(float(dt.std()), 2),
        "p50": round(pct(dt, 50), 2), "p95": round(pct(dt, 95), 2),
        "p99": round(pct(dt, 99), 2), "max": round(float(dt.max()), 2),
        "missed_cycles(>40ms)": int((dt > 40).sum()),
        "_caveat": "host 時間戳，含序列緩衝抖動；<20ms 延遲不可信，需韌體 micros",
    }

    # ---- 韌體級時間（reflash 後才有 t/dt/cus）----
    if all("cus" in d for d in tele):
        fdt = np.array([d["dt"] for d in tele[1:]], dtype=float)  # 韌體實測週期 ms
        cus = np.array([d["cus"] for d in tele], dtype=float)      # 讀 6 編碼器 µs
        fp["firmware_timing"] = {
            "loop_period_ms": {"mean": round(float(fdt.mean()), 2), "std": round(float(fdt.std()), 2),
                                "p95": round(pct(fdt, 95), 2), "max": round(float(fdt.max()), 2)},
            "can_read_us": {"mean": round(float(cus.mean()), 1), "std": round(float(cus.std()), 1),
                             "p50": round(pct(cus, 50), 1), "p95": round(pct(cus, 95), 1),
                             "p99": round(pct(cus, 99), 1), "max": round(float(cus.max()), 1)},
            "_note": "韌體 micros，精確；CAN 讀取延遲是頻寬受限判別的核心數字",
        }

    # ---- 掉幀 / 每顆讀取失敗 ----
    fail_per_motor = [int(sum(1 for d in tele if d["r"][i] == INT64_MIN)) for i in range(6)]
    fp["dropped_frames"] = {
        "any_motor_fail_rate": round(float((ok < 6).mean()), 4),
        "fail_count_per_motor": fail_per_motor,
        "mean_ok_of_6": round(float(ok.mean()), 3),
    }

    # ---- 靜止雜訊地板（假設整段靜止）----
    rng = a.max(0) - a.min(0)
    moving = float(rng.max()) > 2.0  # 任一軸動超過 2° 視為非靜止
    fp["noise_floor_deg"] = {
        "std_per_motor": [round(float(x), 4) for x in a.std(0)],
        "peak2peak_per_motor": [round(float(x), 4) for x in rng],
        "lsb_deg": round(LSB_DEG, 4),
        "_note": "疑似有運動，雜訊地板不可信" if moving else "整段靜止，雜訊地板有效",
    }

    # ---- 量化 LSB 驗證（最小非零 raw 變化應 = 1 count）----
    min_step = []
    for i in range(6):
        col = np.array([d["r"][i] for d in tele if d["r"][i] != INT64_MIN], dtype=np.int64)
        if len(col) > 1:
            dr = np.abs(np.diff(col))
            dr = dr[dr > 0]
            min_step.append(int(dr.min()) if len(dr) else 0)
        else:
            min_step.append(-1)
    fp["raw_min_step_counts"] = min_step  # 應 ≈ 1

    # ---- CAN 劣化 ----
    fp["can_health"] = {
        "tec_max": int(tec.max()), "tec_final": int(tec[-1]),
        "rec_max": int(rec.max()), "rec_final": int(rec[-1]),
        "frac_error_passive(tec>=128)": round(float((tec >= 128).mean()), 4),
        "eflg_bits_seen": int(np.bitwise_or.reduce(eflg.astype(int))),
    }

    # ---- 激勵對齊（有 P 指令才做）----
    p_cmds = [(tt, c) for tt, c in cmds if c.startswith("P")]
    if p_cmds:
        fp["excitation"] = {
            "p_command_count": len(p_cmds),
            "_note": "偵測到激勵指令；step/chirp 響應分析待韌體時間戳（reflash 後）才精確",
        }

    # ---- 輸出 ----
    out = Path(path).with_suffix(".fingerprint.json")
    out.write_text(json.dumps(fp, ensure_ascii=False, indent=2))
    print(json.dumps(fp, ensure_ascii=False, indent=2))
    print(f"\n→ 已存 {out}")

    # ---- 圖 ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(11, 9))
        ts = (t - t[0]) / 1000.0
        for i in range(6):
            ax[0].plot(ts, a[:, i], lw=0.8, label=f"M{i+1}")
        ax[0].set(title="角度時序 a[i]", xlabel="s", ylabel="deg")
        ax[0].legend(ncol=6, fontsize=7)
        ax[1].hist(dt, bins=60)
        ax[1].set(title="host 迴圈週期分布 (ms) — 含序列抖動", xlabel="ms")
        ax[1].axvline(20, color="r", ls="--", lw=1)
        ax[2].plot(ts, ok, label="ok/6")
        ax[2].plot(ts, tec, label="tec")
        ax[2].plot(ts, rec, label="rec")
        ax[2].set(title="掉幀 / CAN 錯誤", xlabel="s")
        ax[2].legend()
        fig.tight_layout()
        png = Path(path).with_suffix(".png")
        fig.savefig(png, dpi=110)
        print(f"→ 已存 {png}")
    except Exception as e:
        print(f"(略過繪圖: {e})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    analyze(sys.argv[1])
