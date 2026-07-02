#!/usr/bin/env node
// PID 掃描 runner：外層掃 vFOC Kd 或 Kv（Kp/Ki 固定），內層對單軸打可重複 U 脈衝，
// 每格獨立錄一檔（檔名帶 pid）→ 供 recover_analyze 逐檔提取 σ/ζ，刻畫 Kd/Kv→震盪響應面。
// 為何單軸 U：要量 ζ/settling 需振幅完全可重複（控制變因），手推做不到。
// 安全：小振幅起 + backstop(herr 超閾值自動 D 並中止整個掃描) + 你在場 + 隨時 Ctrl-C。
//
// 用法: node sysid/pid_sweep.js <axis 0-5> <deg> <ms> <reps> <settle秒> <var kd|kv> <fixed> <val...>
//   node sysid/pid_sweep.js 3 5 120 5 3.5 kd 0 0 160 320 480 640
//     ↑ M4 打 5°/120ms ×5，每發等 3.5s；掃 Kd，Kv 固定=0，Kd 取 {0,160,320,480,640}
'use strict';
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const KP = 1024, KI = 0;                 // 固定錨點（使用者定的穩定工作點）
const AXIS   = parseInt(process.argv[2] ?? '3', 10);
const DEG    = parseFloat(process.argv[3] ?? '5');
const MS     = parseInt(process.argv[4] ?? '120', 10);
const REPS   = parseInt(process.argv[5] ?? '5', 10);
const SETTLE = parseFloat(process.argv[6] ?? '3.5') * 1000;
const VAR    = (process.argv[7] ?? 'kd').toLowerCase();   // 掃描變數 kd|kv
const FIXED  = parseInt(process.argv[8] ?? '0', 10);      // 另一項固定值
const VALS   = process.argv.slice(9).map(Number);
const BACKSTOP = 12;
const SETTLE_PID = 2000;                 // 改 PID 後等馬達內環穩定
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

if (!['kd', 'kv'].includes(VAR) || !VALS.length) {
  console.log('用法: node sysid/pid_sweep.js <axis> <deg> <ms> <reps> <settle> <kd|kv> <fixed> <val...>');
  process.exit(1);
}

// 組 K 指令：掃 kd → [KP,KI,val,FIXED]；掃 kv → [KP,KI,FIXED,val]
const pidOf = (val) => VAR === 'kd' ? [KP, KI, val, FIXED] : [KP, KI, FIXED, val];

let ws, herr = null, tripped = false;
function onMsg(m) {
  let d; try { d = JSON.parse(m.toString()); } catch { return; }
  if (Array.isArray(d.herr)) herr = d.herr;
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
  console.log(`起始 pid=[${s.pid}] pos=${s.pos}`);
  if (s.pos !== 1) { console.log('→ 送 H 鎖位當前姿態'); ws.send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) return safeStop('⚠ hold 未建立，中止');

  console.log(`\n掃描 ${VAR.toUpperCase()} ∈ {${VALS.join(',')}}（另一項固定=${FIXED}, Kp=${KP} Ki=${KI}）`);
  console.log(`每格 M${AXIS + 1} 打 ${DEG}°/${MS}ms ×${REPS}，每發等 ${SETTLE / 1000}s。backstop>${BACKSTOP}°→D+中止。\n`);

  // 全程 backstop 監看
  const watch = setInterval(() => {
    if (!herr || tripped) return;
    const cur = Math.max(...herr.map(Math.abs));
    if (cur > BACKSTOP) { tripped = true; const ax = herr.map(Math.abs).indexOf(cur); safeStop(`\n⛔ M${ax + 1} |herr|=${cur.toFixed(1)}° > ${BACKSTOP}° → 斷電+中止掃描！`); }
  }, 100);

  const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '').slice(0, 13);
  const names = [];
  for (const val of VALS) {
    if (tripped) break;
    const pid = pidOf(val);
    ws.send(`K ${pid.join(' ')}`);
    await sleep(SETTLE_PID);                 // 等馬達內環吃新 PID 穩定
    const name = `sweep_${VAR}${val}_M${AXIS + 1}_pid${pid.join('-')}_${ts}`;
    await rest(`rec/start?name=${name}`);
    let peakMax = 0;
    for (let k = 0; k < REPS && !tripped; k++) {
      ws.send(`U ${AXIS} ${DEG} ${MS}`);
      await sleep(MS + 80);
      const peak = herr ? Math.max(...herr.map(Math.abs)) : 0;
      if (peak > peakMax) peakMax = peak;
      await sleep(SETTLE);
    }
    await rest('rec/stop');
    names.push(name);
    console.log(`  ${VAR}=${String(val).padStart(4)}  pid=[${pid}]  峰值max|herr|≈${peakMax.toFixed(1)}°  → ${name}`);
  }
  clearInterval(watch);

  if (!tripped) {
    // 掃完恢復安靜工作點 [1024,0,0,0]
    ws.send(`K ${KP} ${KI} 0 0`);
    console.log(`\n✓ ${names.length} 格完成。恢復 K=[${KP},${KI},0,0]`);
    console.log(`分析: uv run sysid/recover_analyze.py 'sysid/data/sweep_${VAR}*_${ts}*.jsonl'`);
  }
  ws.close();
  process.exit(0);
}
main().catch(async e => { console.error(e); await safeStop(); process.exit(1); });
