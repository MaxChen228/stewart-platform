// 極限環/回穩研究共用分析函式（純函式，無狀態）。
// ki_matrix.js / amp_sweep.js 各自內嵌過同款邏輯；新腳本一律 require 此檔。
// 定義對齊：σ 用局部極值絕對幅的 ln 線性回歸斜率；峰峰用窗內 max-min。

// pts: [[t_ms, value], ...]
function pp(pts) { if (pts.length < 2) return 0; const v = pts.map(p => p[1]); return Math.max(...v) - Math.min(...v); }
function mean(pts) { return pts.length ? pts.reduce((a, b) => a + b[1], 0) / pts.length : 0; }
function rms(pts) { if (!pts.length) return 0; const m = mean(pts); return Math.sqrt(pts.reduce((a, b) => a + (b[1] - m) ** 2, 0) / pts.length); }

// σ 衰減率：σ>0 衰減(ring-down)，σ≈0 自持。回 {s, r2, n}
function sigma(pts) {
  if (pts.length < 12) return { s: NaN, r2: 0, n: 0 };
  const m = mean(pts), amp = [];
  for (let i = 1; i < pts.length - 1; i++) {
    const a0 = Math.abs(pts[i - 1][1] - m), a1 = Math.abs(pts[i][1] - m), a2 = Math.abs(pts[i + 1][1] - m);
    if (a1 >= a0 && a1 >= a2 && a1 > 0.4) amp.push([(pts[i][0] - pts[0][0]) / 1000, a1]);
  }
  if (amp.length < 4) return { s: NaN, r2: 0, n: amp.length };
  const xs = amp.map(p => p[0]), ys = amp.map(p => Math.log(p[1]));
  const n = xs.length, sx = xs.reduce((a, b) => a + b), sy = ys.reduce((a, b) => a + b);
  const sxx = xs.reduce((a, b) => a + b * b, 0), sxy = xs.reduce((a, b, i) => a + b * ys[i], 0);
  const slope = (n * sxy - sx * sy) / (n * sxx - sx * sx + 1e-9), inter = (sy - slope * sx) / n;
  const my = sy / n, sst = ys.reduce((a, y) => a + (y - my) ** 2, 0);
  const ssr = ys.reduce((a, y, i) => a + (y - (slope * xs[i] + inter)) ** 2, 0);
  return { s: -slope, r2: sst > 0 ? 1 - ssr / sst : 0, n };
}

function med(a) { const s = [...a].sort((x, y) => x - y), m = s.length >> 1; return s.length ? (s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2) : 0; }

const sleep = (ms) => new Promise(r => setTimeout(r, ms));

module.exports = { pp, mean, rms, sigma, med, sleep };
