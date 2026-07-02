#!/usr/bin/env node
// Ki 階梯矩陣：HOLD 下對 M4 打固定脈衝(U)，掃 vFOC 位置環積分 Ki，
// 量每個 Ki 的「激發峰 / 殘餘振盪 / 靜態剛性 / σ衰減」，多重複破除單點僥倖。
// 假說：Ki 過高→死區兩側積分泵能→自持；降 Ki→振盪衰減且剛性(Kp)保留。
//
// CAN 品質感知：每格只用 ok==6 的乾淨樣本算振幅；TEC>150 或連續壞幀 → 暫停恢復。
// 指令全經 server echo → monitor 同步畫標記。錄製走 server JSONL（單一真相）。
//
// 用法: node sysid/ki_matrix.js
const WebSocket = require('ws');
const { pp, rms, sigma } = require('./lc_lib');   // 峰峰/RMS/σ衰減率 共用分析函式（SoT，不再內嵌）
const HOST = 'localhost:3000';
const M4 = 3;
const KP = 220, KD = 270, KV = 320;       // 固定，只動 Ki
const KI_LIST = [100, 80, 60, 40, 30, 20, 10];
const REPS = 3;
const EXCITE_DEG = 8, EXCITE_MS = 200;
const PID_SETTLE = 700, STATIC_MS = 1500, OBSERVE_MS = 4500;
const ABORT_ABS = 22;                      // M4 |herr| 超過 → 中止該輪
const SAFE_KI = 100;

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));

let ws, latest = null, abort = false;
const send = (c) => ws.send(c);

// 收集器：時間戳 + M4 herr，只收 ok==6
function collector(durMs) {
  return new Promise(async (resolve) => {
    const pts = []; let total = 0, bad = 0, txMax = 0;
    const onMsg = (m) => {
      let d; try { d = JSON.parse(m.toString()); } catch { return; }
      if (!Array.isArray(d.a)) return;
      total++; txMax = Math.max(txMax, d.tx || 0);
      if (d.ok !== 6) { bad++; return; }
      if (Array.isArray(d.herr)) {
        const v = d.herr[M4];
        pts.push([performance.now(), v]);
        if (Math.abs(v) > ABORT_ABS) abort = true;
      }
    };
    ws.on('message', onMsg);
    await sleep(durMs);
    ws.off('message', onMsg);
    resolve({ pts, total, bad, txMax });
  });
}

async function canHealthy() {
  const s = await rest('latest');
  return s.ok === 6 || (s.ok >= 4 && (s.tx || 0) < 96);
}

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });

  const s0 = await rest('latest');
  if (s0.pos !== 1) { send('H'); await sleep(1800); }
  const s1 = await rest('latest');
  if (s1.pos !== 1) { console.log('⚠ hold 未建立，中止'); ws.close(); return; }

  await rest('rec/start?name=ki_matrix');
  console.log('=== Ki 矩陣（M4, 脈衝 %d°/%dms, 每格 ×%d）===', EXCITE_DEG, EXCITE_MS, REPS);
  console.log('Ki   rep | 靜態pp 靜均 | 激發峰  殘餘pp 殘RMS |  σ    r²  | 有效% TEC');
  console.log('-----+-----+-------------+----------------------+-----------+---------');

  const rows = [];
  outer:
  for (const ki of KI_LIST) {
    for (let rep = 1; rep <= REPS; rep++) {
      if (!(await canHealthy())) {
        console.log(`  [CAN 不健康，暫停 4s 恢復…]`);
        send('D'); await sleep(800); send('H'); await sleep(3200);
      }
      send(`K ${KP} ${ki} ${KD} ${KV}`); await sleep(PID_SETTLE);
      // 靜態剛性（激發前）
      const st = await collector(STATIC_MS);
      // 激發
      send(`U ${M4} ${EXCITE_DEG} ${EXCITE_MS}`);
      const ob = await collector(OBSERVE_MS);
      if (abort) { console.log('⛔ |herr|>%d° 中止，恢復 Ki', ABORT_ABS); break outer; }

      const t0 = ob.pts.length ? ob.pts[0][0] : 0;
      const peak = ob.pts.filter(p => p[0] - t0 < 1200);     // 激發峰窗
      const resid = ob.pts.filter(p => p[0] - t0 > OBSERVE_MS - 2000); // 末 2s 殘餘
      const sig = sigma(ob.pts);
      const validPct = 100 * (ob.total - ob.bad) / Math.max(1, ob.total);
      const r = {
        ki, rep, staticPP: pp(st.pts), staticMean: st.pts.length ? st.pts.reduce((a, b) => a + b[1], 0) / st.pts.length : 0,
        excitePeak: pp(peak), residPP: pp(resid), residRMS: rms(resid),
        sigma: sig.s, r2: sig.r2, validPct, tec: ob.txMax,
      };
      rows.push(r);
      console.log(
        `${String(ki).padStart(4)}  ${rep}/${REPS}| ${r.staticPP.toFixed(2).padStart(5)} ${r.staticMean.toFixed(2).padStart(5)} | ` +
        `${r.excitePeak.toFixed(1).padStart(5)}  ${r.residPP.toFixed(1).padStart(5)}  ${r.residRMS.toFixed(2).padStart(5)} | ` +
        `${(isFinite(r.sigma) ? r.sigma.toFixed(2) : '  NaN').padStart(5)} ${r.r2.toFixed(2)} | ${validPct.toFixed(0).padStart(4)}% ${r.tec}`
      );
    }
  }

  send(`K ${KP} ${SAFE_KI} ${KD} ${KV}`); await sleep(400);
  await rest('rec/stop');

  // 每 Ki 中位數彙整
  console.log('\n=== 每 Ki 彙整（殘餘pp 中位數 = 自持強度）===');
  console.log('Ki  | 殘餘pp中位 | 靜態pp中位(剛性) | 激發峰中位');
  const med = a => { const s = [...a].sort((x, y) => x - y), m = s.length >> 1;
    return s.length ? (s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2) : 0; };
  for (const ki of KI_LIST) {
    const g = rows.filter(r => r.ki === ki);
    if (!g.length) continue;
    console.log(`${String(ki).padStart(3)} | ${med(g.map(r => r.residPP)).toFixed(1).padStart(9)} | ` +
      `${med(g.map(r => r.staticPP)).toFixed(2).padStart(15)} | ${med(g.map(r => r.excitePeak)).toFixed(1).padStart(9)}`);
  }
  console.log('\n完成。錄製檔 sysid/data/ki_matrix_*.jsonl');
  ws.close();
}
main().catch(e => { console.error(e); try { ws.send(`K ${KP} ${SAFE_KI} ${KD} ${KV}`); } catch {} process.exit(1); });
