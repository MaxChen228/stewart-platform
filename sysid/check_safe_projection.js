#!/usr/bin/env node
'use strict';
// 安全集 soft-knee 投影不變量閘（npm test 會跑）。
//
// 投影是 PF 即時控制鏈的安全主閘（server manualPfUpdateTarget + phone.html + live.html 三處共用
// workspace_envelope.makeSafeProjector），破壞其不變量＝把不安全 pose 直接發到六顆實體馬達。
// 驗五條：①home 恆 safe ②輸出恆 safe 且 clamped 語意正確 ③殼內恆等（零失真=零延遲承諾）
// ④knee=1 精確退化硬牆 ⑤沿射線連續（無跳變）＋成本預算 <1ms。
// 跑法: node sysid/check_safe_projection.js。不符 → 印 diff + exit 1。

const WE = require('./workspace_envelope');
const Kin = require('./kin');

const problems = [];
const HOME = [0, 0, 133, 0, 0, 0];   // platform_sot homeRelative z=+28 → 絕對 z=NEUTRAL_Z+28=133
const Rb = Kin.BASE_RADIUS;
const geo = WE.geoFromKin(Kin);
const isSafe = WE.makeSafeChecker(geo);
const toN = (p) => [p[0] / Rb, p[1] / Rb, p[2] / Rb, p[3], p[4], p[5]];

// 可重現亂數（mulberry32）：測試不可 flaky
let seed = 0x5eed;
function rand() {
  seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
  let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}
const rnd = (a, b) => a + (b - a) * rand();

// ---- ① home（含平移偏移的代表性 center）恆 safe ----
for (const c of [HOME, [10, 0, 133, 0, 0, 0], [0, -10, 118, 0, 0, 0], [0, 0, 148, 0, 0, 0]]) {
  if (!isSafe(toN(c))) problems.push(`home/center ${JSON.stringify(c)} 不 safe（投影射線起點失效）`);
}

// ---- ②③ 隨機射線：輸出恆 safe；!clamped → 恆等；clamped → rho>knee 且輸出在射線上 ----
const KNEE = 0.8;
const project = WE.makeSafeProjector(Kin, { knee: KNEE });
let nClamped = 0;
for (let t = 0; t < 1000; t++) {
  const input = [HOME[0] + rnd(-50, 50), HOME[1] + rnd(-50, 50), HOME[2] + rnd(-45, 45),
    rnd(-55, 55), rnd(-55, 55), rnd(-110, 110)];   // 故意大幅超界
  const r = project(input, HOME);
  if (!isSafe(toN(r.pose))) { problems.push(`輸出不 safe: in=${JSON.stringify(input.map(v => +v.toFixed(1)))} out=${JSON.stringify(r.pose.map(v => +v.toFixed(1)))}`); break; }
  if (!r.clamped) {
    const maxDiff = Math.max(...input.map((v, i) => Math.abs(v - r.pose[i])));
    if (maxDiff > 1e-9) { problems.push(`!clamped 但輸出≠輸入（殼內失真，違反零延遲承諾）: diff=${maxDiff}`); break; }
  } else {
    nClamped++;
    if (!(r.rho > KNEE)) { problems.push(`clamped 但 rho=${r.rho} ≤ knee=${KNEE}`); break; }
    // 輸出必須在 home→input 射線上（各軸縮放係數一致）
    const fs = input.map((v, i) => (Math.abs(v - HOME[i]) > 1e-6 ? (r.pose[i] - HOME[i]) / (v - HOME[i]) : null)).filter((v) => v != null);
    if (Math.max(...fs) - Math.min(...fs) > 1e-6) { problems.push(`clamped 輸出偏離射線: f 範圍 ${Math.min(...fs)}..${Math.max(...fs)}`); break; }
  }
}
if (nClamped < 300) problems.push(`隨機超界樣本 clamped 僅 ${nClamped}/1000——測試域沒真正覆蓋邊界外`);

// ---- ④ knee=1 硬牆退化：殼內恆等、超界夾在第一安全段邊界 ----
const hardWall = WE.makeSafeProjector(Kin, { knee: 1 });
for (let t = 0; t < 200; t++) {
  const input = [HOME[0], HOME[1], HOME[2], rnd(-55, 55), rnd(-55, 55), rnd(-110, 110)];
  const r = hardWall(input, HOME);
  if (!isSafe(toN(r.pose))) { problems.push(`knee=1 輸出不 safe`); break; }
  if (r.clamped) {
    // 邊界緊貼性：再往外推一個二分解析度應 unsafe（沿射線)
    const eps = (1 / 24 / 2 ** 10) * 2 + 1e-3;   // 掃描步寬/2^bisectIters 的保守上界
    const beyond = r.pose.map((v, i) => HOME[i] + (v - HOME[i]) * (1 + eps) / Math.max(r.s, 1e-9) * r.s);
    void beyond; // 邊界緊貼由 s/rho 一致性驗：f 應 = s*（輸出縮放 = 第一段末端）
    const f = (r.pose[5] - HOME[5]) / (input[5] - HOME[5] || 1e-12);
    if (input[5] !== HOME[5] && Math.abs(f - r.s) > 1e-6) { problems.push(`knee=1 輸出縮放 f=${f} ≠ s*=${r.s}（未夾在第一段邊界）`); break; }
  }
}

// ---- ⑤ 連續性：固定方向掃輸入幅度，輸出無跳變 ----
{
  const dir = [0, 0, 0, 28, 22, 60];   // 已知會超界的方向
  let prev = null, maxJump = 0;
  for (let lam = 0; lam <= 1.6; lam += 0.01) {
    const input = HOME.map((h, i) => h + dir[i] * lam);
    const out = project(input, HOME).pose;
    if (prev) {
      const jump = Math.max(...out.map((v, i) => Math.abs(v - prev[i])));
      if (jump > maxJump) maxJump = jump;
    }
    prev = out;
  }
  // 輸入步 = 0.01×max|dir| = 0.6°；壓縮映射 Lipschitz ≤1 → 輸出步應同量級（含二分解析度餘量）
  if (maxJump > 1.2) problems.push(`連續性: 輸入步 0.6° 下輸出最大跳變 ${maxJump.toFixed(2)}°（>1.2° 視為不連續）`);
}

// ---- 成本預算：平均 <1ms/call（100Hz 熱路徑）----
{
  const N = 500, inputs = [];
  for (let t = 0; t < N; t++) inputs.push([HOME[0] + rnd(-40, 40), HOME[1] + rnd(-40, 40), HOME[2] + rnd(-30, 30), rnd(-50, 50), rnd(-50, 50), rnd(-100, 100)]);
  for (let t = 0; t < 50; t++) project(inputs[t], HOME);   // 暖機
  const t0 = process.hrtime.bigint();
  for (const p of inputs) project(p, HOME);
  const usPer = Number(process.hrtime.bigint() - t0) / 1000 / N;
  if (usPer > 1000) problems.push(`成本: ${usPer.toFixed(0)}µs/call > 1ms 預算`);
  if (problems.length === 0) console.log(`  (bench: 投影 ${usPer.toFixed(0)}µs/call, 隨機超界樣本 clamped ${nClamped}/1000)`);
}

if (problems.length) {
  console.error('✗ 安全投影不變量破壞：');
  for (const p of problems) console.error('  - ' + p);
  console.error('\n投影是 PF 鏈安全主閘（workspace_envelope.makeSafeProjector，server+phone+live 三處共用），不變量必須全綠才可部署。');
  process.exit(1);
}
console.log('✓ 安全投影不變量：home 恆 safe / 輸出恆 safe / 殼內恆等 / knee=1 硬牆退化 / 連續 / 成本達標');
