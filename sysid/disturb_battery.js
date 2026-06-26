#!/usr/bin/env node
// 標準化擾動 battery：啟動後自動跑一整套廣泛、可重複的擾動序列，作為跨 PID 可比的系統壓力測試。
// 設計原則：固定序列 + 固定 severity + 固定時序 = 控制變因 → 不同 PID 跑同一套，結果直接可比。
// 全程單檔錄製，W 指令注入（單軸=其他軸填 0，單一路徑），事件標記在 W 指令上供 plot.py 標線。
// 複用 disturb.js 的 encDelta（同一份 task-space IK，SoT，不重寫第二真相源）。
//
// 涵蓋（實際會發生的情況）：
//   P1 六軸單馬達均勻覆蓋    — 每顆獨立測，揪個別軸弱點
//   P2 task-space 6-DOF 方向 — roll/pitch/tilt/surge/sway/heave/yaw，協調分配六顆
//   P3 強度階梯              — tilt 小→中→大，看非線性/飽和
//   P4 連續無間隔背靠背      — mixed 連打不等回正，測累積穩定/是否發散
//   P5 最嚴苛複合            — mixed 大 severity，終極壓力
//
// 用法:
//   node sysid/disturb_battery.js preview          # 只印序列與每發六軸分配，不發指令
//   node sysid/disturb_battery.js [tag] [backstop]  # 跑完整 battery，tag 進檔名（預設讀當前 pid）
'use strict';
const { encDelta } = require('./disturb_modes.js');
const WebSocket = require('ws');
const HOST = 'localhost:3000';

// ===== 標準 battery 序列（v1）。{p:phase, mode, sev:六軸RMS度(single為單軸度), ms:脈衝寬, gap:之後等待秒} =====
const SEQ = [
  // P1 六軸單馬達均勻覆蓋（single：sev=單軸度數）
  { p: 'P1-single', mode: 'm1', sev: 5, ms: 120, gap: 3.0 },
  { p: 'P1-single', mode: 'm2', sev: 5, ms: 120, gap: 3.0 },
  { p: 'P1-single', mode: 'm3', sev: 5, ms: 120, gap: 3.0 },
  { p: 'P1-single', mode: 'm4', sev: 5, ms: 120, gap: 3.0 },
  { p: 'P1-single', mode: 'm5', sev: 5, ms: 120, gap: 3.0 },
  { p: 'P1-single', mode: 'm6', sev: 5, ms: 120, gap: 3.0 },
  // P2 task-space 六自由度方向（sev=六軸 RMS 度）
  { p: 'P2-dof', mode: 'roll',  sev: 3, ms: 120, gap: 3.0 },
  { p: 'P2-dof', mode: 'pitch', sev: 3, ms: 120, gap: 3.0 },
  { p: 'P2-dof', mode: 'tilt',  sev: 3, ms: 120, gap: 3.0 },
  { p: 'P2-dof', mode: 'surge', sev: 3, ms: 120, gap: 3.0 },
  { p: 'P2-dof', mode: 'sway',  sev: 3, ms: 120, gap: 3.0 },
  { p: 'P2-dof', mode: 'heave', sev: 3, ms: 120, gap: 3.0 },
  { p: 'P2-dof', mode: 'yaw',   sev: 3, ms: 120, gap: 3.0 },
  // P3 強度階梯（同模態 tilt，小→中→大）
  { p: 'P3-ramp', mode: 'tilt', sev: 2, ms: 120, gap: 3.0 },
  { p: 'P3-ramp', mode: 'tilt', sev: 4, ms: 120, gap: 3.0 },
  { p: 'P3-ramp', mode: 'tilt', sev: 6, ms: 120, gap: 3.5 },
  // P4 連續無間隔背靠背（mixed 連打，gap 短到不等回正 → 累積擾動）
  { p: 'P4-burst', mode: 'mixed', sev: 3, ms: 120, gap: 0.4 },
  { p: 'P4-burst', mode: 'mixed', sev: 3, ms: 120, gap: 0.4 },
  { p: 'P4-burst', mode: 'mixed', sev: 3, ms: 120, gap: 0.4 },
  { p: 'P4-burst', mode: 'mixed', sev: 3, ms: 120, gap: 3.5 },
  // P5 最嚴苛複合
  { p: 'P5-max', mode: 'mixed', sev: 5, ms: 150, gap: 4.0 },
];

const fmt = a => a.map(x => x.toFixed(2).padStart(6)).join(' ');

// ---------- preview ----------
if (process.argv[2] === 'preview') {
  console.log(`battery v1：${SEQ.length} 事件，預估時長 ${(SEQ.reduce((s, e) => s + e.ms / 1000 + e.gap, 0)).toFixed(0)}s\n`);
  console.log('# phase       mode   sev  ms  gap |   M1     M2     M3     M4     M5     M6   | max');
  SEQ.forEach((e, i) => {
    const d = encDelta(e.mode, e.sev), mx = Math.max(...d.map(Math.abs));
    console.log(`${String(i + 1).padStart(2)} ${e.p.padEnd(10)} ${e.mode.padEnd(5)} ${String(e.sev).padStart(3)} ${String(e.ms).padStart(4)} ${e.gap.toFixed(1)} | ${fmt(d)} | ${mx.toFixed(1)}`);
  });
  const allMax = Math.max(...SEQ.map(e => Math.max(...encDelta(e.mode, e.sev).map(Math.abs))));
  console.log(`\n全序列單軸最大注入 = ${allMax.toFixed(1)}°（你驗證單軸 10° 不倒）`);
  process.exit(0);
}

// ---------- runner ----------
const TAG = process.argv[2] || '';
const BACKSTOP = parseFloat(process.argv[3] ?? '15');   // herr 超此 → 發散/翻倒，自動 D+中止
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

let ws, herr = null, tripped = false, maxAbs = 0, maxAxis = -1;
function onMsg(m) {
  let d; try { d = JSON.parse(m.toString()); } catch { return; }
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
process.on('SIGINT', async () => { tripped = true; await safeStop('\n⛔ Ctrl-C：停錄+斷電'); await sleep(300); process.exit(0); });

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  ws.on('message', onMsg);

  const s = await rest('latest');
  const pid = (s.pid || []).join('-');
  console.log(`起始 pid=[${s.pid}] pos=${s.pos}`);
  if (s.pos !== 1) { console.log('→ 送 H 鎖位當前姿態'); ws.send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) return safeStop('⚠ hold 未建立，中止');

  const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '').slice(0, 13);
  const name = `battery_${TAG ? TAG + '_' : ''}pid${pid}_${ts}`;
  await rest(`rec/start?name=${name}`);
  console.log(`\n● battery [${name}]  ${SEQ.length} 事件，backstop>${BACKSTOP}°→D+中止。Ctrl-C 可停。\n`);

  const watch = setInterval(() => {
    if (!herr || tripped) return;
    const cur = Math.max(...herr.map(Math.abs));
    if (cur > BACKSTOP) { tripped = true; const ax = herr.map(Math.abs).indexOf(cur); safeStop(`\n⛔ M${ax + 1} |herr|=${cur.toFixed(1)}° > ${BACKSTOP}° → 斷電+中止 battery！`); }
  }, 80);

  let lastPhase = '';
  for (let i = 0; i < SEQ.length && !tripped; i++) {
    const e = SEQ[i];
    if (e.p !== lastPhase) { console.log(`\n── ${e.p} ──`); lastPhase = e.p; }
    const d = encDelta(e.mode, e.sev);
    ws.send(`W ${d.map(x => x.toFixed(3)).join(' ')} ${e.ms}`);
    await sleep(e.ms + 80);
    const peak = herr ? Math.max(...herr.map(Math.abs)) : 0;
    console.log(`  ${String(i + 1).padStart(2)} ${e.mode.padEnd(5)} sev=${e.sev}  峰值max|herr|≈${peak.toFixed(1)}°`);
    await sleep(e.gap * 1000);
  }
  clearInterval(watch);

  if (!tripped) {
    const r = await rest('rec/stop');
    console.log(`\n✓ battery 完成，${r.lines || '?'} lines。全程最大漂移 M${maxAxis + 1}=${maxAbs.toFixed(2)}°`);
    console.log(`圖: uv run sysid/plot.py 'sysid/data/${name}*.jsonl' --out /tmp/${name}.png`);
  }
  ws.close();
  process.exit(0);
}
main().catch(async e => { console.error(e); await safeStop(); process.exit(1); });
