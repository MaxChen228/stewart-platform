#!/usr/bin/env node
'use strict';
// ===== Mock 手機運動 generator =====
// 從真實 phone-capture 抽出各軸 rel(濾波前手部角) 的 PSD/幅度，用 FFT 頻譜合成任意長度的
// 統計等價手部運動 → 走 phone.html 同款 pipeline(shaped/heading/1€/predict) → PF pose 串流。
// 產物可餵 rig 做「無人長時間」追蹤測試（取代真手機）。複用唯一真相源 ../kin.js 做工作空間守界。
//
// 用法：node sysid/phone_gen/gen.js --ref <capture.jsonl> --dur <sec> [--seed N --out f --rate 60]
//   --ref 不給時掃 sysid/data/phone-capture/ 取最大檔。--validate 印生成 vs 真實對照表。
//
// ⚠️ pipeline 常數是 web/phone.html 的鏡像（SoT=phone.html）。改 phone.html 的 MAP/OEF 要同步這裡，
//    或日後抽 sysid/phone_pipeline.js 共用(如 kin.js)。目前刻意不動正在用的 phone.html。
const fs = require('fs');
const path = require('path');
const Kin = require('../kin.js');

// ---------------- args ----------------
const argv = process.argv.slice(2);
const getArg = (k, d) => { const i = argv.indexOf(k); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : d; };
const hasFlag = (k) => argv.includes(k);
const RATE = Number(getArg('--rate', 60));
const DUR = Number(getArg('--dur', 600));
const SEED = Number(getArg('--seed', 12345));
const MODE = getArg('--mode', 'bootstrap');   // bootstrap=保真實爆發/間歇(三軸聯合連續塊拼接) | iaaft=全合成(匹配譜+邊際,動力學偏平滑)
const VALIDATE = hasFlag('--validate');
if (!['bootstrap', 'iaaft'].includes(MODE)) { console.error(`--mode 須 bootstrap 或 iaaft（收到 ${MODE}）`); process.exit(1); }
const CAP_DIR = path.join(__dirname, '..', 'data', 'phone-capture');
// 多參考池化（語料庫）：--ref 可逗號分隔多檔；不給則用 phone-capture/ 全部 cap_*（含每場≥1024 imu 的）。
// 高階矩(峰度/偏度)session 間天生不穩→單場易過擬合；池化多場 = 更廣 repertoire + 更穩的可重現特徵。
let REFS = getArg('--ref', null);
if (REFS) REFS = REFS.split(',').map(s => s.trim());
else {
  REFS = fs.existsSync(CAP_DIR) ? fs.readdirSync(CAP_DIR).filter(f => f.startsWith('cap_') && f.endsWith('.jsonl')).map(f => path.join(CAP_DIR, f)) : [];
  if (!REFS.length) { console.error('找不到參考 capture（--ref 或放 sysid/data/phone-capture/）'); process.exit(1); }
}
const GEN_DIR = path.join(__dirname, '..', 'data', 'phone-gen');   // 與真實 capture 分開（sysid/data/ 已 gitignore）
const OUT = getArg('--out', path.join(GEN_DIR, `gen_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.jsonl`));
if (OUT.includes(path.sep + 'phone-gen' + path.sep)) fs.mkdirSync(GEN_DIR, { recursive: true });

// ---------------- PRNG (seeded, 可重現) ----------------
function mulberry32(a) { return function () { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
const rng = mulberry32(SEED);
function randn() { let u = 0, v = 0; while (u === 0) u = rng(); while (v === 0) v = rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

// ---------------- FFT (iterative radix-2) ----------------
function fft(re, im, inverse) {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) { let bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { [re[i], re[j]] = [re[j], re[i]];[im[i], im[j]] = [im[j], im[i]]; } }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = (inverse ? 2 : -2) * Math.PI / len;
    for (let i = 0; i < n; i += len) for (let k = 0; k < len / 2; k++) {
      const c = Math.cos(ang * k), s = Math.sin(ang * k);
      const a = i + k, b = i + k + len / 2;
      const vr = re[b] * c - im[b] * s, vi = re[b] * s + im[b] * c;
      re[b] = re[a] - vr; im[b] = im[a] - vi; re[a] += vr; im[a] += vi;
    }
  }
  if (inverse) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
}
const nextPow2 = (n) => { let p = 1; while (p < n) p <<= 1; return p; };

// ---------------- helpers ----------------
const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
const std = a => { const m = mean(a); return Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / a.length); };
const pctl = (a, p) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(s.length * p))]; };
function quantileAt(q, p) { const x = Math.max(0, Math.min(1, p)) * (q.length - 1); const i = Math.floor(x); if (i >= q.length - 1) return q[q.length - 1]; const t = x - i; return q[i] * (1 - t) + q[i + 1] * t; }
function buildQuant(series, Q = 1001) { const s = [...series].sort((a, b) => a - b); const q = new Array(Q); for (let i = 0; i < Q; i++) { const x = i / (Q - 1) * (s.length - 1), j = Math.floor(x), t = x - j; q[i] = j < s.length - 1 ? s[j] * (1 - t) + s[j + 1] * t : s[s.length - 1]; } return q; }
function resample(times, vals, fs) {
  const t0 = times[0], t1 = times[times.length - 1], dt = 1000 / fs, out = []; let j = 0;
  for (let t = t0; t <= t1; t += dt) { while (j < times.length - 2 && times[j + 1] < t) j++; const f = (t - times[j]) / Math.max(1e-6, times[j + 1] - times[j]); out.push(vals[j] + f * (vals[j + 1] - vals[j])); }
  return out;
}

// ---------------- 1. 從參考抽 PSD shape + 幅度（Welch, pow2 分段）----------------
const NFFT = 2048;   // @60Hz → 頻率步階 0.029Hz、段長 34s；解析低頻手部運動
function welchPSD(series, nfft) {
  const hop = nfft >> 1;
  const win = []; for (let i = 0; i < nfft; i++) win.push(0.5 - 0.5 * Math.cos(2 * Math.PI * i / (nfft - 1)));
  const wpow = win.reduce((a, b) => a + b * b, 0);
  const acc = new Array(nfft / 2 + 1).fill(0); let segs = 0;
  for (let st = 0; st + nfft <= series.length; st += hop) {
    const re = new Array(nfft), im = new Array(nfft).fill(0);
    let m = 0; for (let i = 0; i < nfft; i++) m += series[st + i]; m /= nfft;
    for (let i = 0; i < nfft; i++) re[i] = (series[st + i] - m) * win[i];
    fft(re, im, false);
    for (let k = 0; k <= nfft / 2; k++) acc[k] += re[k] * re[k] + im[k] * im[k];
    segs++;
  }
  const psd = acc.map(v => (segs ? v / (segs * RATE * wpow) : 0));
  psd[0] = 0;
  return { psd, segs };
}
function specStat(psd, freqStep) {
  let tot = 0; for (let k = 1; k < psd.length; k++) tot += psd[k];
  const at = q => { let c = 0; for (let k = 1; k < psd.length; k++) { c += psd[k]; if (c >= q * tot) return k * freqStep; } return (psd.length - 1) * freqStep; };
  return { f50: at(.5), f90: at(.9), f95: at(.95) };
}
function loadImuSeries(file) {
  const L = fs.readFileSync(file, 'utf8').trim().split('\n');
  const imu = [];
  for (const l of L) { let j; try { j = JSON.parse(l); } catch { continue; } if (j.type === 'imu') imu.push(j); }
  if (imu.length < NFFT) return null;   // 跳過 imu 不足的舊檔（imu=0）
  const S = [0, 1, 2].map(k => resample(imu.map(x => x.t), imu.map(x => x.rel[k]), RATE));
  return { S, len: S[0].length, durS: (imu[imu.length - 1].t - imu[0].t) / 1000, file: path.basename(file) };
}
// 多場池化：PSD 依長度加權平均；quant/sd/min/max/velP99 對所有場的值池化。bootstrap 用原始 per-session 序列。
function refModel(files) {
  const sessions = [];
  for (const f of files) { const s = loadImuSeries(f); if (s) sessions.push(s); }
  if (!sessions.length) { console.error(`無可用參考（每場需 ≥${NFFT} imu 樣本；舊檔可能 imu=0）`); process.exit(1); }
  const freqStep = RATE / NFFT;
  const axes = [0, 1, 2].map(k => {
    const accPsd = new Array(NFFT / 2 + 1).fill(0); let wsum = 0;
    const allVals = [], allVel = [];
    for (const ss of sessions) {
      const series = ss.S[k];
      const { psd } = welchPSD(series, NFFT);
      for (let i = 0; i < psd.length; i++) accPsd[i] += psd[i] * series.length;
      wsum += series.length;
      for (const v of series) allVals.push(v);
      for (let i = 1; i < series.length; i++) allVel.push(Math.abs(series[i] - series[i - 1]) * RATE);
    }
    const psd = accPsd.map(v => v / wsum); psd[0] = 0;
    const ss = specStat(psd, freqStep);
    return { psd, freqStep, half: NFFT / 2, quant: buildQuant(allVals), sd: std(allVals), min: Math.min(...allVals), max: Math.max(...allVals), velP99: pctl(allVel, .99), f50: ss.f50, f90: ss.f90 };
  });
  return { axes, sessions, durS: sessions.reduce((a, s) => a + s.durS, 0), nSessions: sessions.length };
}

// ---------------- 2. IAAFT 合成 rel（任意長度，同時匹配 振幅譜 + 邊際分布）----------------
// 交替投影：(a) 套真實振幅譜、保留相位 → 對頻譜；(b) rank-remap 到真實邊際分布 → 對幅度/偏度/重尾。
// 迭代收斂 → 幅度、頻譜、速度分布全對（單做其一會顧此失彼：純頻譜合成低估重尾、純直方圖匹配灌大速度）。
const IAAFT_ITER = 20;
function synth(ax, N) {
  const Np = nextPow2(N);
  // 目標振幅譜 A[k]（對稱），由真實 Welch PSD 內插到 Np 格點；DC=0
  const interpPsd = (f) => {
    if (f <= 0) return 0;
    const x = f / ax.freqStep;
    if (x < 1) return (ax.psd[1] || 0) * x;
    const i = Math.floor(x); if (i >= ax.half) return ax.psd[ax.half];
    const t = x - i; return ax.psd[i] * (1 - t) + ax.psd[i + 1] * t;
  };
  const A = new Array(Np); A[0] = 0; A[Np / 2] = Math.sqrt(Math.max(0, interpPsd(RATE / 2)));
  for (let k = 1; k < Np / 2; k++) { const v = Math.sqrt(Math.max(0, interpPsd(k * RATE / Np))); A[k] = v; A[Np - k] = v; }
  // 目標邊際的排序樣本（Np 個真實分位數，升序）
  const targetSorted = new Array(Np);
  for (let i = 0; i < Np; i++) targetSorted[i] = quantileAt(ax.quant, (i + 0.5) / Np);
  // 初始：從真實邊際 i.i.d. 抽（白譜、正確邊際）
  let x = new Array(Np); for (let i = 0; i < Np; i++) x[i] = quantileAt(ax.quant, rng());
  for (let it = 0; it < IAAFT_ITER; it++) {
    // (a) FFT → 保留相位、振幅換成目標譜 → IFFT（Hermitian 由對稱 A + 實輸入保證 → 實輸出）
    const re = x.slice(), im = new Array(Np).fill(0);
    fft(re, im, false);
    for (let k = 0; k < Np; k++) { const mag = Math.hypot(re[k], im[k]) || 1e-12, s = A[k] / mag; re[k] *= s; im[k] *= s; }
    fft(re, im, true);
    // (b) rank-remap：把 IFFT 結果的秩序對到目標邊際 → 強制精確真實分布
    const idx = Array.from({ length: Np }, (_, i) => i).sort((a, b) => re[a] - re[b]);
    const nx = new Array(Np);
    for (let r = 0; r < Np; r++) nx[idx[r]] = targetSorted[r];
    x = nx;
  }
  // 裁到目標長度後再套一次邊際（消 Np→N crop 取樣偏差；單調 remap 保留時序/頻譜）
  const xc = x.slice(0, N);
  const tN = new Array(N); for (let i = 0; i < N; i++) tN[i] = quantileAt(ax.quant, (i + 0.5) / N);
  const idx2 = Array.from({ length: N }, (_, i) => i).sort((a, b) => xc[a] - xc[b]);
  const out = new Array(N); for (let r = 0; r < N; r++) out[idx2[r]] = tN[r];
  return out;
}

// ---------------- 2b. block-bootstrap rel（三軸聯合，保真實爆發/間歇/跨軸協調；多場池化）----------------
// 重採樣真實 60Hz 序列的「連續區塊」拼接：每塊都是真實微動力學(尖銳爆發、停頓叢聚)。三軸取同一場同一時窗
// → 保住跨軸協調手勢。多場語料庫：每塊隨機選一場再選窗(不跨場接=不造假縫)。接縫「連續性匹配(value+slope)」+「0.05s 洩漏」消縫不漂。
function bootstrapRel(sessions, N) {
  const minB = Math.round(1.5 * RATE), maxB = Math.round(4.0 * RATE), K = 32;
  const usable = sessions.map((s, j) => j).filter(j => sessions[j].len > maxB + 2);
  if (!usable.length) { console.error(`所有參考場長度 < 區塊上限 ${(maxB / RATE).toFixed(1)}s`); process.exit(1); }
  const pickJ = () => usable[Math.floor(rng() * usable.length)];
  const pickStart = (j) => 1 + Math.floor(rng() * (sessions[j].len - maxB - 2));
  const out = [[], [], []];
  let j = pickJ(), start = pickStart(j), blk = minB + Math.floor(rng() * (maxB - minB));
  for (let i = 0; i < blk && out[0].length < N; i++) for (let k = 0; k < 3; k++) out[k].push(sessions[j].S[k][start + i]);
  while (out[0].length < N) {
    const e = out[0].length - 1;
    const eVal = [out[0][e], out[1][e], out[2][e]];
    const eVel = [out[0][e] - out[0][e - 1], out[1][e] - out[1][e - 1], out[2][e] - out[2][e - 1]];
    let bj = -1, bs = -1, bestC = Infinity;
    for (let c = 0; c < K; c++) {
      const cj = pickJ(), cs = pickStart(cj), S = sessions[cj].S;
      let cost = 0;
      for (let k = 0; k < 3; k++) { const cv = S[k][cs], cvel = S[k][cs] - S[k][cs - 1]; cost += (cv - eVal[k]) ** 2 + 9 * (cvel - eVel[k]) ** 2; }   // slope ×9 保爆發持續性
      if (cost < bestC) { bestC = cost; bj = cj; bs = cs; }
    }
    j = bj; start = bs; blk = minB + Math.floor(rng() * (maxB - minB));
    const S = sessions[j].S;
    const off = [eVal[0] - S[0][start], eVal[1] - S[1][start], eVal[2] - S[2][start]];   // 接縫殘差
    const blend = Math.min(blk, Math.round(0.05 * RATE));
    for (let i = 1; i < blk && out[0].length < N; i++) for (let k = 0; k < 3; k++) {
      const decay = i < blend ? off[k] * (1 - i / blend) : 0;   // 0.05s 內洩掉偏移 → 連續且不累積漂移
      out[k].push(S[k][start + i] + decay);
    }
  }
  return out.map(a => a.slice(0, N));
}

// ---------------- 3. phone.html pipeline（SoT 鏡像）----------------
const D2R = Math.PI / 180;
const MAP = { tiltGain: 0.5, yawGain: 1.4, headingDeg: 120, deadband: 0.4, rollSign: 1, pitchSign: 1, yawSign: 1 };
const OEF = { cfg: [{ mincut: 0.6, beta: 0.5, dcut: 0.4 }, { mincut: 0.6, beta: 0.5, dcut: 0.4 }, { mincut: 0.4, beta: 0.10, dcut: 0.3 }], predict: 0.08, clamp: 4.0 };
const HOME = [0, 0, 133, 0, 0, 0];
function shaped(v, gain) { const a = Math.abs(v); if (a <= MAP.deadband) return 0; return Math.sign(v) * (a - MAP.deadband) * gain; }
function targetRpy(rel) {
  const r = MAP.rollSign * shaped(rel[0], MAP.tiltGain);
  const p = MAP.pitchSign * shaped(rel[1], MAP.tiltGain);
  const y = MAP.yawSign * shaped(rel[2], MAP.yawGain);
  const h = MAP.headingDeg * D2R, ch = Math.cos(h), sh = Math.sin(h);
  return [r * ch - p * sh, r * sh + p * ch, y];
}
function oefAlpha(cut, dt) { const r = 2 * Math.PI * cut * dt; return r / (r + 1); }
function makeOef() { return [0, 1, 2].map(() => ({ xPrev: 0, dxPrev: 0, started: false })); }
function oneEuro(st, i, x, dt) {
  const s = st[i], c = OEF.cfg[i];
  if (!s.started) { s.xPrev = x; s.dxPrev = 0; s.started = true; return { x, dx: 0 }; }
  const dx = (x - s.xPrev) / dt;
  const dxHat = s.dxPrev + oefAlpha(c.dcut, dt) * (dx - s.dxPrev);
  const cut = c.mincut + c.beta * Math.abs(dxHat);
  const xHat = s.xPrev + oefAlpha(cut, dt) * (x - s.xPrev);
  s.xPrev = xHat; s.dxPrev = dxHat;
  return { x: xHat, dx: dxHat };
}

// ---------------- main ----------------
const ref = refModel(REFS);
const N = Math.round(DUR * RATE);
const rel = MODE === 'bootstrap'
  ? bootstrapRel(ref.sessions, N)
  : [synth(ref.axes[0], N), synth(ref.axes[1], N), synth(ref.axes[2], N)];
const dt = 1 / RATE;
const oef = makeOef();
let lastPose = HOME.slice();
const out = fs.createWriteStream(OUT);
out.write(JSON.stringify({ meta: { kind: 'phone-gen', mode: MODE, refs: ref.sessions.map(s => s.file), durS: DUR, rate: RATE, seed: SEED, wallClock: new Date().toISOString() } }) + '\n');
const genRel = [[], [], []], genCmd = [[], [], []], genPose = [];
let ikDrop = 0;
for (let n = 0; n < N; n++) {
  const rr = [rel[0][n], rel[1][n], rel[2][n]];
  const tgt = targetRpy(rr);
  const cmd = [0, 0, 0];
  for (let i = 0; i < 3; i++) {
    const r = oneEuro(oef, i, tgt[i], dt);
    let p = r.dx * OEF.predict; p = Math.max(-OEF.clamp, Math.min(OEF.clamp, p));
    cmd[i] = r.x + p;
  }
  const cand = HOME.slice(); cand[3] += cmd[0]; cand[4] += cmd[1]; cand[5] += cmd[2];
  const pose = Kin.ik(cand).valid ? cand : (ikDrop++, lastPose);
  lastPose = pose;
  const t = +(n * dt * 1000).toFixed(1);
  out.write(JSON.stringify({ t, type: 'pf', pose: pose.map(v => +v.toFixed(3)), rel: rr.map(v => +v.toFixed(3)), cmd: cmd.map(v => +v.toFixed(3)) }) + '\n');
  for (let i = 0; i < 3; i++) { genRel[i].push(rr[i]); genCmd[i].push(cmd[i]); }
  genPose.push(pose);
}
out.write(JSON.stringify({ end: { records: N, ikDrop } }) + '\n');
out.end();

console.log(`參考: ${ref.nSessions} 場池化 (${ref.durS.toFixed(0)}s 共)  [${ref.sessions.map(s => s.file).join(', ')}]`);
console.log(`生成: ${path.basename(OUT)}  mode=${MODE}  ${DUR}s @ ${RATE}Hz = ${N} frames  seed=${SEED}  ik越界夾限=${ikDrop} (${(ikDrop / N * 100).toFixed(2)}%)`);

if (VALIDATE) {
  const AX = ['roll', 'pitch', 'yaw'];
  console.log('\n=== 驗證：生成 vs 參考（rel，全部應接近）===');
  console.log('軸     | sd        | |max|      | p99速度°/s | f50 Hz    | f90 Hz');
  console.log('       | 生成/參考 | 生成/參考  | 生成/參考  | 生成/參考 | 生成/參考');
  for (let k = 0; k < 3; k++) {
    const g = genRel[k], r = ref.axes[k];
    const vel = []; for (let i = 1; i < g.length; i++) vel.push(Math.abs(g[i] - g[i - 1]) * RATE);
    const { psd } = welchPSD(g, NFFT); const ss = specStat(psd, RATE / NFFT);
    const pad = (a, b, w) => `${a}/${b}`.padEnd(w);
    console.log(`${AX[k].padEnd(6)} | ${pad(std(g).toFixed(2), r.sd.toFixed(2), 9)} | ${pad(Math.max(...g.map(Math.abs)).toFixed(1), Math.max(Math.abs(r.min), Math.abs(r.max)).toFixed(1), 10)} | ${pad(pctl(vel, .99).toFixed(0), r.velP99.toFixed(0), 10)} | ${pad(ss.f50.toFixed(2), r.f50.toFixed(2), 9)} | ${ss.f90.toFixed(2)}/${r.f90.toFixed(2)}`);
  }
  console.log('\n=== 生成 pf.pose 範圍（送 rig 的最終 intent）===');
  for (const k of [3, 4, 5]) { const v = genPose.map(p => p[k]); console.log(`${['x', 'y', 'z', 'roll', 'pitch', 'yaw'][k].padEnd(6)} sd=${std(v).toFixed(2)} range[${pctl(v, 0).toFixed(1)},${pctl(v, 1).toFixed(1)}]`); }
}
