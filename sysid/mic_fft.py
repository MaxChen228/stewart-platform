# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "matplotlib"]
# ///
"""
聲學嘯叫測量：錄馬達 HOLD 時的環境音 → FFT 頻譜 → 量可聞頻段能量 + 主峰。
telemetry(22Hz) 測不到 kHz 嘯叫；耳朵是對的，但要客觀數據就得錄音做頻譜。

用法:
  uv run sysid/mic_fft.py <label> [secs=4] [device=:0]
  錄音 → 分段平均功率譜 → 印各頻段 RMS + 主峰 → 存 PNG（可視覺判讀/疊比）
跨 PID 比較：每個 PID 設好、靜置，跑一次，比 label 之間的頻段能量與主峰。
"""
import sys, subprocess, wave
import numpy as np
import numpy.fft as fft
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABEL = sys.argv[1] if len(sys.argv) > 1 else "rec"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
DEV = sys.argv[3] if len(sys.argv) > 3 else ":0"
WAV = f"/tmp/mic_{LABEL}.wav"
PNG = f"/tmp/mic_{LABEL}.png"

# 錄音（44.1k mono，涵蓋全可聞頻段）
subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                "-f", "avfoundation", "-i", DEV, "-t", str(SECS),
                "-ac", "1", "-ar", "44100", WAV], check=True)

w = wave.open(WAV)
fs = w.getframerate()
x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(float) / 32768.0
k = int(0.3 * fs)                       # 去頭尾 0.3s 避錄音啟動瞬態
x = x[k:-k] if len(x) > 2 * k else x

# 分段平均功率譜（Welch 風格，降方差）
seg, hop = 8192, 4096
win = np.hanning(seg)
acc, cnt = None, 0
for i in range(0, len(x) - seg, hop):
    s = x[i:i + seg] * win
    S = np.abs(fft.rfft(s)) ** 2
    acc = S if acc is None else acc + S
    cnt += 1
P = acc / max(1, cnt)
f = fft.rfftfreq(seg, 1 / fs)

def band(lo, hi):
    m = (f >= lo) & (f < hi)
    return float(np.sqrt(np.mean(P[m]))) if m.any() else 0.0

mask = f > 100                          # 主峰避開低頻嗡（電源/風扇）
pk = f[mask][np.argmax(P[mask])]
b = {"0.1-1k": band(100, 1000), "1-2k": band(1000, 2000), "2-4k": band(2000, 4000),
     "4-8k": band(4000, 8000), "8-15k": band(8000, 15000)}
wide = band(300, 15000)                 # 寬頻可聞總能量（單一比較數）
print(f"{LABEL}: 主峰={pk:.0f}Hz  寬頻RMS(0.3-15k)={wide:.3e}")
print("  頻段RMS  " + "  ".join(f"{k}={v:.2e}" for k, v in b.items()))

# 頻譜圖
plt.figure(figsize=(11, 4), facecolor="#0b0e14")
ax = plt.axes(facecolor="#11151f")
ax.semilogy(f, P + 1e-14, color="#5aa9ff", lw=0.6)
ax.axvline(pk, color="#ff6b5e", ls="--", lw=1, label=f"主峰 {pk:.0f}Hz")
ax.set_xlim(0, 15000)
ax.set_xlabel("Hz", color="#8b96a8")
ax.set_title(f"{LABEL} 聲學功率譜  寬頻RMS={wide:.2e}  主峰{pk:.0f}Hz", color="#e8edf4")
ax.tick_params(colors="#8b96a8")
for s in ax.spines.values():
    s.set_color("#2a3140")
ax.legend(facecolor="#11151f", edgecolor="#2a3140", labelcolor="#e8edf4")
plt.savefig(PNG, dpi=110, bbox_inches="tight", facecolor="#0b0e14")
print(f"→ {PNG}")
