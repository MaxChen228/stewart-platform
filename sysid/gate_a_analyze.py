# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy"]
# ///
"""
Gate A 穩定度分析：讀 gateA_*.jsonl，對每個迴圈頻率判定階躍後是收斂還是發散。
指標（事前定義，避免事後偏差）：
  - peak_err  : 階躍後 pose error 模長峰值
  - final_err : 最後 0.5s 的 error 模長（趨近 0 = 回正）
  - 趨勢      : 後 1/3 vs 中 1/3 的 error RMS 比值（>1.3 視為發散、<0.7 收斂）
用法: uv run sysid/gate_a_analyze.py 'sysid/data/gateA_*.jsonl'
"""
import sys, json, glob
import numpy as np


def load_err(path):
    """回傳 (t[], errnorm[])：err = mode-1 telemetry 的 poseError 6 維模長。"""
    t, e = [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if r.get("dir") != "in":
            continue
        try:
            d = json.loads(r["d"])
        except Exception:
            continue
        if "err" not in d:
            continue
        t.append(r["t"] / 1000.0)
        e.append(float(np.linalg.norm(d["err"])))
    return np.array(t), np.array(e)


def analyze(path):
    t, e = load_err(path)
    if len(e) < 10:
        return None
    n = len(e)
    mid = e[n // 3:2 * n // 3]
    last = e[2 * n // 3:]
    final = e[-max(1, n // 20):]              # 最後 ~5%
    ratio = (last.mean() / mid.mean()) if mid.mean() > 1e-6 else float("inf")
    verdict = "發散❌" if ratio > 1.3 else ("收斂✅" if ratio < 0.7 else "邊際〜")
    return {
        "peak_err": round(float(e.max()), 3),
        "final_err": round(float(final.mean()), 3),
        "trend_ratio": round(float(ratio), 2),
        "verdict": verdict,
        "samples": n,
    }


if __name__ == "__main__":
    pat = sys.argv[1] if len(sys.argv) > 1 else "sysid/data/gateA_*.jsonl"
    files = sorted(glob.glob(pat))
    if not files:
        print(f"找不到檔案: {pat}")
        sys.exit(1)
    print(f"{'檔案':<28} {'峰值err':>8} {'末端err':>8} {'趨勢比':>7} {'判定':>8}")
    print("-" * 64)
    for f in files:
        r = analyze(f)
        name = f.split("/")[-1].replace(".jsonl", "")
        if r is None:
            print(f"{name:<28} (資料不足/非 mode-1)")
            continue
        print(f"{name:<28} {r['peak_err']:>8} {r['final_err']:>8} {r['trend_ratio']:>7} {r['verdict']:>8}")
