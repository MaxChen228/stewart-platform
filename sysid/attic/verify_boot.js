#!/usr/bin/env node
// 驗證持久化 / Ki 效果（嚴謹測法，對齊 amp_sweep：殘餘窗=激發後末2s、只取 ok==6、多 rep 中位）。
// mode='boot' → 不送任何 K，測開機預設行為（驗證韌體 setup 下發是否生效）。
// mode=<數字> → 送 K 220 <n> 270 320 對照。
//
// 用法: node sysid/verify_boot.js boot      (測開機態，須先 reset ESP32)
//       node sysid/verify_boot.js 30        (對照 runtime Ki=30)
const WebSocket = require('ws');
const { pp, sigma, med, sleep } = require('./lc_lib');
const HOST = 'localhost:3000';
const M4 = 3, AMP = 12, EXCITE_MS = 200, REPS = 3;
const OBSERVE_MS = 4500, RESID_FROM = OBSERVE_MS - 2000;   // 末2s
const mode = process.argv[2] || 'boot';
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));

let ws;
function collect(durMs) {
  return new Promise(async (res) => {
    const pts = []; let bad = 0, tot = 0;
    const on = (m) => { let d; try { d = JSON.parse(m.toString()); } catch { return; }
      if (!Array.isArray(d.a)) return; tot++; if (d.ok !== 6) { bad++; return; }
      if (Array.isArray(d.herr)) pts.push([performance.now(), d.herr[M4]]); };
    ws.on('message', on); await sleep(durMs); ws.off('message', on); res({ pts, bad, tot });
  });
}

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  const s = await rest('latest');
  console.log(`ESP32 uptime ${s.t}ms  pos=${s.pos} ok=${s.ok}  mode=${mode}`);
  if (s.pos !== 1) { ws.send('H'); await sleep(1800); }
  if (mode !== 'boot') { ws.send(`K 220 ${parseInt(mode, 10)} 270 320`); await sleep(700); }

  const resids = [], peaks = [];
  for (let r = 0; r < REPS; r++) {
    ws.send(`U ${M4} ${AMP} ${EXCITE_MS}`);
    const ob = await collect(OBSERVE_MS);
    const t0 = ob.pts.length ? ob.pts[0][0] : 0;
    const peak = pp(ob.pts.filter(p => p[0] - t0 < 1200));
    const resid = pp(ob.pts.filter(p => p[0] - t0 > RESID_FROM));
    const sg = sigma(ob.pts);
    peaks.push(peak); resids.push(resid);
    console.log(`  rep${r + 1}: 激發峰 ${peak.toFixed(1)}°  殘餘pp ${resid.toFixed(2)}°  σ ${isFinite(sg.s) ? sg.s.toFixed(2) : 'NaN'} r²${sg.r2.toFixed(2)}  (壞幀 ${ob.bad}/${ob.tot})`);
    await sleep(1200);
  }
  const mr = med(resids);
  console.log(`\n${mode === 'boot' ? '開機預設(未送K)' : 'runtime Ki=' + mode}: 激發峰中位 ${med(peaks).toFixed(1)}°  殘餘pp中位 ${mr.toFixed(2)}°`);
  console.log(mr < 2 ? '✓ 殘餘≈0，振盪被殺' : `✗ 殘餘 ${mr.toFixed(1)}° 自持`);
  ws.close();
}
main().catch(e => { console.error(e); process.exit(1); });
