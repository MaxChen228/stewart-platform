#!/usr/bin/env node
// P3 無支架靜止站立 — 安全監控 + 記錄。
// 前提：平台已 Ki=30、在負載姿態。流程：本腳本先 H 鎖位+確認穩 → 提示你「慢慢撤支架」
// → 高頻監看六軸 herr 漂移：5° 警告、10° backstop 自動 D 斷電（明確發散才觸發）。
// 主要安全靠你看 monitor.html + 手動 D 大鈕 + spotter 手扶；本腳本是數值記錄 + 兜底。
//
// 用法: node sysid/p3_standfree.js [時長秒,預設0=持續到Ctrl-C] [backstop°,預設10]
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const DUR = parseFloat(process.argv[2] || '0');
const BACKSTOP = parseFloat(process.argv[3] || '10');
const WARN = 5;
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

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  ws.on('message', onMsg);

  const s = await rest('latest');
  if (!Array.isArray(s.pid) || s.pid[1] > 40) console.log(`⚠ Ki=${s.pid?.[1]} 非 30，建議先設 K 220 30 270 320`);
  if (s.pos !== 1) { console.log('→ H 鎖位'); ws.send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) { console.log('⚠ hold 未建立，中止'); ws.close(); return; }

  await rest('rec/start?name=p3_standfree');
  console.log('\n========================================');
  console.log('✅ 安全網就緒。Ki=30、已 hold 鎖位。');
  console.log(`   現在可【慢慢撤支架】(spotter 手扶、隨時準備接)。`);
  console.log(`   任一軸 |herr| > ${WARN}° 警告、> ${BACKSTOP}° 自動斷電。`);
  console.log('   你也可隨時在 monitor.html 按 D 大鈕，或這裡 Ctrl-C。');
  console.log('========================================\n');

  const t0 = Date.now(); let lastWarn = 0;
  while (!tripped) {
    await sleep(400);
    if (!herr) continue;
    const cur = Math.max(...herr.map(Math.abs));
    const axis = herr.map(Math.abs).indexOf(cur);
    const okStr = ok === 6 ? '' : ` [ok=${ok}!]`;
    if (cur > BACKSTOP) {
      ws.send('D'); tripped = true;
      console.log(`\n⛔ M${axis + 1} |herr|=${cur.toFixed(1)}° > ${BACKSTOP}° → 已斷電(D)！spotter 接住。撐不住。`);
      break;
    }
    const tag = cur > WARN ? '  ⚠️發散中' : '';
    const now = Date.now();
    if (cur > WARN || now - lastWarn > 2000) {
      console.log(`t=${((now - t0) / 1000).toFixed(0)}s  max|herr| M${axis + 1}=${cur.toFixed(2)}°  [${herr.map(h => h.toFixed(1)).join(' ')}]${okStr}${tag}`);
      lastWarn = now;
    }
    if (DUR > 0 && (now - t0) / 1000 >= DUR) { console.log(`\n✓ 守住 ${DUR}s 未發散。最大漂移 M${maxAxis + 1}=${maxAbs.toFixed(2)}°`); break; }
  }

  await rest('rec/stop');
  console.log(`\n錄製檔 sysid/data/p3_standfree_*.jsonl  全程最大漂移 M${maxAxis + 1}=${maxAbs.toFixed(2)}°`);
  ws.close();
}
process.on('SIGINT', async () => { try { await rest('rec/stop'); } catch {} console.log('\n(已停錄；平台保持 hold，未斷電)'); process.exit(0); });
main().catch(e => { console.error(e); process.exit(1); });
