#!/usr/bin/env node
// 多模態擾動引擎 + 測試 runner。
// 設計：擾動定義在 task-space（platform 6-DOF），單一旋鈕 severity，其餘六軸量由 IK 算出。
//   Δa_ik = ik(pose0 + severity·dir) − ik(pose0)   （task-space → 角度，物理結構正確）
//   Δa_enc = MOTOR_SIGN × Δa_ik                      （IK 慣例 → enc 慣例，見 encoder.h 慣例註解）
//   發 W 六軸同時脈衝（hold 模式疊加 holdAngles）。
// severity 正規化：每模態縮放成「severity = 六軸 enc-Δangle 的 RMS（度）」→ 跨模態強度可比。
//   severity=3 ≈ 平均每軸擾動 3°（不分 roll/heave/yaw，都同一把尺）。
//
// 模態：roll/pitch/tilt(對角傾)/surge(x)/sway(y)/heave(z)/yaw/mixed(複合)/m1..m6(單軸)
//
// 用法:
//   node sysid/disturb.js preview                       # 印所有模態的六軸分配（不發指令）
//   node sysid/disturb.js <mode> <sev> <ms> <reps> [backstop=12] [settle=4]
//     node sysid/disturb.js tilt 3 120 5                # tilt 模態 sev=3°，120ms 脈衝 ×5，每發等 4s
'use strict';
const { encDelta, ALL_MODES, DIRS, POSE0, rms } = require('./disturb_modes.js');   // 擾動數學單一真相源

// ---------- preview ----------
if (require.main === module && process.argv[2] === 'preview') {
  console.log(`工作點 pose0=[${POSE0}]，severity=3 正規化（六軸 RMS=3°）\n`);
  console.log('模態      M1     M2     M3     M4     M5     M6   | rms  max');
  for (const mode of ALL_MODES) {
    const d = encDelta(mode, 3);
    const r = rms(d), mx = Math.max(...d.map(Math.abs));
    console.log(mode.padEnd(7), d.map(x => x.toFixed(2).padStart(6)).join(' '), `| ${r.toFixed(2)} ${mx.toFixed(2)}`);
  }
  process.exit(0);
}

// ---------- 測試 runner ----------
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const MODE = process.argv[2];
const SEV = parseFloat(process.argv[3] ?? '3');
const MS = parseInt(process.argv[4] ?? '120', 10);
const REPS = parseInt(process.argv[5] ?? '5', 10);
const BACKSTOP = parseFloat(process.argv[6] ?? '12');
const SETTLE = parseFloat(process.argv[7] ?? '4') * 1000;
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

if (!MODE) { console.log('用法: node sysid/disturb.js preview | <mode> <sev> <ms> <reps> [backstop] [settle]'); process.exit(1); }

let ws, herr = null, ok = 6, tripped = false, maxAbs = 0, maxAxis = -1;
function onMsg(m) {
  let d; try { d = JSON.parse(m.toString()); } catch { return; }
  if (!Array.isArray(d.a)) return;
  ok = d.ok;
  if (Array.isArray(d.herr)) {
    herr = d.herr;
    for (let i = 0; i < 6; i++) { const a = Math.abs(d.herr[i]); if (a > maxAbs) { maxAbs = a; maxAxis = i; } }
  }
}
async function safeStop(msg) {
  if (msg) console.log(msg);
  try { await rest('rec/stop'); } catch {}
  try { if (ws?.readyState === 1) ws.send('D'); } catch {}
}
process.on('SIGINT', async () => { await safeStop('\n⛔ Ctrl-C：停錄+斷電'); await sleep(300); process.exit(0); });

async function main() {
  const dAngle = encDelta(MODE, SEV);
  console.log(`模態=${MODE} severity=${SEV}° → enc Δangle=[${dAngle.map(x => x.toFixed(2)).join(' ')}] rms=${rms(dAngle).toFixed(2)}°`);

  // 守門：韌體 W handler 會 constrain(d, ±30°) 靜默截斷。若 host 算出的單軸 Δ 超 30°，
  // 實際施加 ≠ 記錄值 → 數據失真。寧可中止要求降 severity，不產生失真數據（真實數據說話）。
  const overAxis = dAngle.findIndex(x => Math.abs(x) > 30);
  if (overAxis >= 0) {
    console.error(`⛔ M${overAxis + 1} Δ=${dAngle[overAxis].toFixed(1)}° 超韌體 W ±30° 上限 → 會被靜默截斷致數據失真。請降低 severity（當前 ${SEV}）後重跑。`);
    process.exit(1);
  }

  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  ws.on('message', onMsg);

  const s = await rest('latest');
  console.log(`當前 pid=[${s.pid}] pos=${s.pos}`);
  if (s.pos !== 1) { console.log('→ 送 H 鎖位'); ws.send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) return safeStop('⚠ hold 未建立，中止');

  const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '').slice(0, 13);
  const name = `dist_${MODE}_s${SEV}_pid${(s.pid || []).join('-')}_${ts}`;
  await rest(`rec/start?name=${name}`);
  console.log(`\n● 錄製 [${name}]  ${MODE} sev=${SEV}° × ${REPS} 發，每發等 ${SETTLE / 1000}s。backstop>${BACKSTOP}°→D。Ctrl-C 可停。\n`);

  const wCmd = `W ${dAngle.map(x => x.toFixed(3)).join(' ')} ${MS}`;
  const watch = setInterval(() => {
    if (!herr || tripped) return;
    const cur = Math.max(...herr.map(Math.abs));
    if (cur > BACKSTOP) { tripped = true; const ax = herr.map(Math.abs).indexOf(cur); safeStop(`\n⛔ M${ax + 1} |herr|=${cur.toFixed(1)}° > ${BACKSTOP}° → 斷電！`); }
  }, 100);

  for (let k = 0; k < REPS && !tripped; k++) {
    ws.send(wCmd);
    await sleep(MS + 80);
    const peak = herr ? Math.max(...herr.map(Math.abs)) : 0;
    console.log(`  脈衝 #${k + 1}: 峰值 max|herr|≈${peak.toFixed(1)}°`);
    await sleep(SETTLE);
  }
  clearInterval(watch);

  if (!tripped) {
    const r = await rest('rec/stop');
    console.log(`\n✓ ${REPS} 發完成，${r.lines || '?'} lines。全程最大漂移 M${maxAxis + 1}=${maxAbs.toFixed(2)}°`);
    console.log(`分析: uv run sysid/recover_analyze.py 'sysid/data/${name}*.jsonl'`);
  }
  ws.close();
}
if (require.main === module) main().catch(async e => { console.error(e); await safeStop(); process.exit(1); });
