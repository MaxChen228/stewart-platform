#!/usr/bin/env node
// 極限環激發研究：在 HOLD 模式下，把 F5 運動參數 V(speed,acc) 往上掃，
// 找 M4 自激振盪的門檻（Hopf 邊界），量每個參數下的振幅/頻率。
// 安全：連續監看 M4 herr 峰峰值，超過 ABORT_PP 立刻退回安全參數並停。
// 前提：平台已在 HOLD 模式（pos:1, 有 herr）、支架就位、人在電源旁。
//
// 用法: node sysid/lc_excite.js
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const M4 = 3;
const SETTLE_MS = 1500, OBSERVE_MS = 5000;
const ABORT_PP = 14.0;         // M4 峰峰 > 14°(±7°) → 緊急退回（自然振幅 ±3.7° 之上才算失控）
const SAFE = 'V 60 40';        // 退回用的溫和參數

// 掃描序列：先固定 speed 升 acc，再升 speed（找哪個觸發）
const STEPS = [
  'V 131 81',   // 接近原狀（基準）
  'V 131 130',
  'V 131 180',
  'V 131 230',
  'V 131 255',
  'V 160 255',
  'V 200 255',
];

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));

let ws, buf = [], recording = false;
const send = (c) => { ws.send(c); console.log(`  → ${c}`); };

function pp(arr) { return arr.length ? Math.max(...arr) - Math.min(...arr) : 0; }
function zeroCrossFreq(arr, dtS) {            // 粗估頻率：去均值後數過零
  if (arr.length < 4) return 0;
  const m = arr.reduce((a, b) => a + b, 0) / arr.length;
  let cz = 0;
  for (let i = 1; i < arr.length; i++) if ((arr[i - 1] - m) * (arr[i] - m) < 0) cz++;
  return cz / 2 / (arr.length * dtS);
}

async function safeStop(reason) {
  console.log(`\n⛔ ${reason} → 退回 ${SAFE} + 停錄`);
  try { send(SAFE); } catch {}
  try { if (recording) await rest('rec/stop'); } catch {}
}

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  let aborted = false;
  ws.on('message', (m) => {
    let d; try { d = JSON.parse(m.toString()); } catch { return; }
    if (!Array.isArray(d.herr)) return;
    buf.push(d.herr[M4]);
    if (buf.length > 400) buf.shift();
  });

  // 確認 hold
  const s = await rest('latest');
  if (s.pos !== 1 || !Array.isArray(s.herr)) {
    console.log('⚠️ 平台不在 HOLD 模式（需先 H）。中止。'); ws.close(); return;
  }
  console.log('=== 極限環激發掃描（V 參數）===');
  await rest('rec/start?name=lc_Vsweep');
  recording = true;

  for (const step of STEPS) {
    send(step);
    buf = [];
    await sleep(SETTLE_MS);
    buf = [];                                  // 清掉 settle 暫態
    const t0 = Date.now();
    // 觀察窗內持續監看，超標即刻 abort
    while (Date.now() - t0 < OBSERVE_MS) {
      await sleep(200);
      if (pp(buf) > ABORT_PP) { await safeStop(`M4 峰峰 ${pp(buf).toFixed(1)}° 超標`); aborted = true; break; }
    }
    if (aborted) break;
    const amp = pp(buf) / 2;
    const dtS = OBSERVE_MS / 1000 / Math.max(1, buf.length);
    const f = zeroCrossFreq(buf, dtS);
    const flag = pp(buf) > 1.0 ? '  ← 振盪!' : '';
    console.log(`     ${step}: M4 振幅 ±${amp.toFixed(2)}°  ~${f.toFixed(1)}Hz  (n=${buf.length})${flag}`);
  }

  if (!aborted) { send(SAFE); await sleep(500); await rest('rec/stop'); recording = false; }
  console.log('\n完成。分析: uv run sysid/lc_analyze.py');
  ws.close();
}
main().catch(async (e) => { console.error(e); await safeStop('例外'); process.exit(1); });
