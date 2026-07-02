#!/usr/bin/env node
// 阻尼系統辨識：hold 模式下打「可重複的軟體脈衝」(U injector)，量擾動後回正的衰減特性。
// 為何用 U 脈衝而非手推(recover.js)：要量 ζ/settling 需「振幅完全可重複」的擾動 → 控制變因。
// 主安全靠：小振幅起 + backstop(herr 超閾值自動 D) + 你在場 spot + 隨時 Ctrl-C。
// 分析：uv run sysid/recover_analyze.py 'sysid/data/damp_*.jsonl'  (主指標 σ 衰減率/ζ/peak 數)
//
// 用法: node sysid/damp_id.js [axis 0-5] [deg] [ms] [reps] [backstop° 預設12] [settle秒 預設4]
//   node sysid/damp_id.js 0 3 120 5         # M1 打 3° 脈衝 120ms，重複 5 次，每次等 4s 回正
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const AXIS = parseInt(process.argv[2] ?? '0', 10);
const DEG = parseFloat(process.argv[3] ?? '3');
const MS = parseInt(process.argv[4] ?? '120', 10);
const REPS = parseInt(process.argv[5] ?? '5', 10);
const BACKSTOP = parseFloat(process.argv[6] ?? '12');
const SETTLE = parseFloat(process.argv[7] ?? '4') * 1000;
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

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
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  ws.on('message', onMsg);

  const s = await rest('latest');
  console.log(`當前 pid=[${s.pid}]  pos=${s.pos}  hold=[${(s.hold||[]).map(x=>x.toFixed(1)).join(' ')}]`);
  if (s.pos !== 1) { console.log('→ 未 hold，送 H 鎖位當前姿態'); ws.send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) { return safeStop('⚠ hold 未建立，中止'); }

  const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '').slice(0, 13);
  const name = `damp_M${AXIS + 1}_${DEG}deg_pid${(s.pid||[]).join('-')}_${ts}`;
  await rest(`rec/start?name=${name}`);
  console.log(`\n● 錄製 [${name}]`);
  console.log(`  對 M${AXIS + 1} 打 ${DEG}° / ${MS}ms 脈衝 × ${REPS} 次，每次等 ${SETTLE / 1000}s 回正。`);
  console.log(`  backstop |herr|>${BACKSTOP}° → 自動 D。你可隨時 Ctrl-C。\n`);

  // backstop 監看（獨立高頻輪詢）
  const watch = setInterval(() => {
    if (!herr || tripped) return;
    const cur = Math.max(...herr.map(Math.abs));
    if (cur > BACKSTOP) { tripped = true; const ax = herr.map(Math.abs).indexOf(cur); safeStop(`\n⛔ M${ax + 1} |herr|=${cur.toFixed(1)}° > ${BACKSTOP}° → 斷電！`); }
  }, 120);

  for (let k = 0; k < REPS && !tripped; k++) {
    ws.send(`U ${AXIS} ${DEG} ${MS}`);
    const t0 = Date.now();
    await sleep(MS + 80);
    const peak = herr ? Math.max(...herr.map(Math.abs)) : 0;
    console.log(`  脈衝 #${k + 1}: 注入 ${DEG}°，峰值 |herr|≈${peak.toFixed(1)}°`);
    await sleep(SETTLE);
    if (tripped) break;
  }
  clearInterval(watch);

  if (!tripped) {
    const r = await rest('rec/stop');
    console.log(`\n✓ ${REPS} 發完成，${r.lines || '?'} lines → ${r.path || name}`);
    console.log(`  全程最大漂移 M${maxAxis + 1}=${maxAbs.toFixed(2)}°（脈衝注入軸=M${AXIS + 1}）`);
    console.log(`\n分析: uv run sysid/recover_analyze.py 'sysid/data/${name}*.jsonl'`);
  }
  ws.close();
}
main().catch(async e => { console.error(e); await safeStop(); process.exit(1); });
