#!/usr/bin/env node
// 脈衝幅掃描：固定 Ki，掃 M4 脈衝大小，驗「低 Ki 甜蜜點對更大擾動是否仍殺得住」。
// 通往「受推也能回正」——若殘餘隨幅冒出，門檻與擾動大小有關；若都歸零，Ki 甜蜜點魯棒。
// 對照：可同時測高 Ki 看「同幅下高低 Ki 差異隨幅如何變」。
// 分析邏輯與 ki_matrix.js 相同（第三個 sweep 出現時抽 sysid/lc_lib.js）。
//
// 用法: node sysid/amp_sweep.js [Ki]   (預設 Ki=30；傳 100 可跑對照)
const WebSocket = require('ws');
const { pp, rms } = require('./lc_lib');   // 峰峰/RMS 共用分析函式（SoT，不再內嵌）
const HOST = 'localhost:3000';
const M4 = 3;
const KI = parseInt(process.argv[2] || '30', 10);
const KP = 220, KD = 270, KV = 320;
const AMP_LIST = [5, 8, 12, 16];           // 脈衝幅(°)
const REPS = 3;
const EXCITE_MS = 200;
const PID_SETTLE = 700, OBSERVE_MS = 4500, RESETTLE_MS = 1200;
const ABORT_ABS = 30;
const SAFE_KI = 100;

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));
let ws, abort = false;
const send = (c) => ws.send(c);

function collector(durMs) {
  return new Promise(async (resolve) => {
    const pts = []; let total = 0, bad = 0, txMax = 0;
    const onMsg = (m) => {
      let d; try { d = JSON.parse(m.toString()); } catch { return; }
      if (!Array.isArray(d.a)) return;
      total++; txMax = Math.max(txMax, d.tx || 0);
      if (d.ok !== 6) { bad++; return; }
      if (Array.isArray(d.herr)) { const v = d.herr[M4]; pts.push([performance.now(), v]); if (Math.abs(v) > ABORT_ABS) abort = true; }
    };
    ws.on('message', onMsg); await sleep(durMs); ws.off('message', onMsg); resolve({ pts, total, bad, txMax });
  });
}
async function canHealthy() { const s = await rest('latest'); return s.ok === 6 || (s.ok >= 4 && (s.tx || 0) < 96); }

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  const s0 = await rest('latest'); if (s0.pos !== 1) { send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) { console.log('⚠ hold 未建立'); ws.close(); return; }

  await rest(`rec/start?name=amp_sweep_ki${KI}`);
  console.log('=== 脈衝幅掃描 (M4, Ki=%d, 每格 ×%d) ===', KI, REPS);
  console.log('幅°  rep | 激發峰  殘餘pp 殘RMS | 有效% TEC');
  console.log('-----+-----+----------------------+---------');
  send(`K ${KP} ${KI} ${KD} ${KV}`); await sleep(PID_SETTLE);

  const rows = [];
  outer:
  for (const amp of AMP_LIST) {
    for (let rep = 1; rep <= REPS; rep++) {
      if (!(await canHealthy())) { console.log('  [CAN 恢復…]'); send('D'); await sleep(800); send('H'); await sleep(3200); send(`K ${KP} ${KI} ${KD} ${KV}`); await sleep(PID_SETTLE); }
      send(`U ${M4} ${amp} ${EXCITE_MS}`);
      const ob = await collector(OBSERVE_MS);
      if (abort) { console.log('⛔ |herr|>%d° 中止', ABORT_ABS); break outer; }
      const t0 = ob.pts.length ? ob.pts[0][0] : 0;
      const peak = ob.pts.filter(p => p[0] - t0 < 1200);
      const resid = ob.pts.filter(p => p[0] - t0 > OBSERVE_MS - 2000);
      const validPct = 100 * (ob.total - ob.bad) / Math.max(1, ob.total);
      const r = { amp, rep, excitePeak: pp(peak), residPP: pp(resid), residRMS: rms(resid), validPct, tec: ob.txMax };
      rows.push(r);
      console.log(`${String(amp).padStart(4)}  ${rep}/${REPS}| ${r.excitePeak.toFixed(1).padStart(5)}  ${r.residPP.toFixed(1).padStart(5)}  ${r.residRMS.toFixed(2).padStart(5)} | ${validPct.toFixed(0).padStart(4)}% ${r.tec}`);
      await sleep(RESETTLE_MS);   // 格間 resettle（低 Ki 會自己衰減）
    }
  }
  send(`K ${KP} ${SAFE_KI} ${KD} ${KV}`); await sleep(400);
  await rest('rec/stop');

  console.log('\n=== 每幅彙整（殘餘pp中位 = 是否殺得住）===');
  console.log('幅° | 殘餘pp中位 | 激發峰中位 | 放大比(峰/幅)');
  const med = a => { const s = [...a].sort((x, y) => x - y), m = s.length >> 1; return s.length ? (s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2) : 0; };
  for (const amp of AMP_LIST) {
    const g = rows.filter(r => r.amp === amp); if (!g.length) continue;
    const ep = med(g.map(r => r.excitePeak));
    console.log(`${String(amp).padStart(3)} | ${med(g.map(r => r.residPP)).toFixed(1).padStart(9)} | ${ep.toFixed(1).padStart(9)} | ${(ep / amp).toFixed(2).padStart(11)}`);
  }
  console.log('\n完成。錄製檔 sysid/data/amp_sweep_ki%d_*.jsonl', KI);
  ws.close();
}
main().catch(e => { console.error(e); try { ws.send(`K ${KP} ${SAFE_KI} ${KD} ${KV}`); } catch {} process.exit(1); });
