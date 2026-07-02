#!/usr/bin/env node
// Gate1 雛形：Ki=30 下，六軸輪流被擾動，驗「跨軸通用 + 持續抗擾回穩」。
// 一次回答：(1)降Ki是否只救M4還是全軸通用 (2)反覆擾動下有無任何軸進入自持。
// 常駐 listener 收全六軸 herr；主迴圈定時對輪到的軸打脈衝，每發後量該軸殘餘。
//
// 用法: node sysid/gate1.js [Ki] [脈衝°] [輪數]
const WebSocket = require('ws');
const { pp, rms, sigma, med, sleep } = require('./lc_lib');
const HOST = 'localhost:3000';
const KI = parseInt(process.argv[2] || '30', 10);
const AMP = parseFloat(process.argv[3] || '10');
const ROUNDS = parseInt(process.argv[4] || '2', 10);   // 每軸被打幾次（2輪×6軸=12發）
const KP = 220, KD = 270, KV = 320, SAFE_KI = 100;
const EXCITE_MS = 200, SETTLE_AFTER = 3500, ABORT_ABS = 30;

const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));

let ws;
const log = [];        // 常駐收集：[t, herr[6], ok, tx]
let abort = false, abortAxis = -1;
function onMsg(m) {
  let d; try { d = JSON.parse(m.toString()); } catch { return; }
  if (!Array.isArray(d.a) || d.ok !== 6 || !Array.isArray(d.herr)) return;
  log.push([performance.now(), d.herr.slice(), d.ok, d.tx || 0]);
  for (let i = 0; i < 6; i++) if (Math.abs(d.herr[i]) > ABORT_ABS) { abort = true; abortAxis = i; }
}
// 取軸 i 在 [t0,t1] 的 pts
const seg = (i, t0, t1) => log.filter(r => r[0] >= t0 && r[0] <= t1).map(r => [r[0], r[1][i]]);

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  ws.on('message', onMsg);

  if ((await rest('latest')).pos !== 1) { ws.send('H'); await sleep(1800); }
  if ((await rest('latest')).pos !== 1) { console.log('⚠ hold 未建立'); ws.close(); return; }

  await rest(`rec/start?name=gate1_ki${KI}`);
  ws.send(`K ${KP} ${KI} ${KD} ${KV}`); await sleep(700);
  console.log('=== Gate1 雛形 (Ki=%d, 脈衝%d°, %d輪×6軸) ===', KI, AMP, ROUNDS);
  console.log('輪 軸 | 激發峰 殘餘pp 殘RMS |  σ   r²  | 全軸最大殘餘(殘餘窗) | TEC');
  console.log('---+---+--------------------+---------+----------------------+----');

  const results = [];
  outer:
  for (let round = 1; round <= ROUNDS; round++) {
    for (let axis = 0; axis < 6; axis++) {
      const tPulse = performance.now();
      ws.send(`U ${axis} ${AMP} ${EXCITE_MS}`);
      await sleep(SETTLE_AFTER);
      if (abort) { console.log('⛔ M%d |herr|>%d° 中止', abortAxis + 1, ABORT_ABS); break outer; }
      const t1 = performance.now();
      const own = seg(axis, tPulse, t1);
      const peakWin = own.filter(p => p[0] - tPulse < 1200);
      const residWin = own.filter(p => p[0] - tPulse > SETTLE_AFTER - 1500);   // 末1.5s
      const sg = sigma(own);
      // 殘餘窗內「其他軸」最大殘餘（耦合/連鎖自持偵測）
      let crossMax = 0, crossAxis = -1;
      for (let j = 0; j < 6; j++) {
        const s = seg(j, t1 - 1500, t1).map(p => [p[0], p[1]]);
        const a = pp(s); if (a > crossMax) { crossMax = a; crossAxis = j; }
      }
      const tec = Math.max(0, ...log.filter(r => r[0] >= tPulse && r[0] <= t1).map(r => r[3]));
      const r = { round, axis, peak: pp(peakWin), resid: pp(residWin), residRMS: rms(residWin),
        sigma: sg.s, r2: sg.r2, crossMax, crossAxis, tec };
      results.push(r);
      console.log(`${round}  M${axis + 1}| ${r.peak.toFixed(1).padStart(5)} ${r.resid.toFixed(1).padStart(6)} ${r.residRMS.toFixed(2).padStart(6)} | ` +
        `${(isFinite(r.sigma) ? r.sigma.toFixed(2) : ' NaN').padStart(4)} ${r.r2.toFixed(2)} | ` +
        `M${crossAxis + 1}=${crossMax.toFixed(1).padStart(4)}°            | ${tec}`);
    }
  }

  ws.send(`K ${KP} ${SAFE_KI} ${KD} ${KV}`); await sleep(400);
  await rest('rec/stop');

  // 判定
  console.log('\n=== 每軸彙整（殘餘pp中位）===');
  for (let i = 0; i < 6; i++) {
    const g = results.filter(r => r.axis === i); if (!g.length) continue;
    console.log(`M${i + 1}: 殘餘pp中位 ${med(g.map(r => r.resid)).toFixed(1)}°  激發峰中位 ${med(g.map(r => r.peak)).toFixed(1)}°`);
  }
  const maxResid = Math.max(...results.map(r => r.resid));
  const allSettled = results.every(r => r.resid < 2.0);
  console.log(`\n判定：最大單發殘餘 ${maxResid.toFixed(1)}°  → ${allSettled ? '✓ 全部回穩(<2°)，Ki' + KI + ' 跨軸抗擾通過' : '✗ 有軸未回穩，殘餘 ' + maxResid.toFixed(1) + '°'}`);
  console.log('錄製檔 sysid/data/gate1_ki%d_*.jsonl', KI);
  ws.close();
}
main().catch(e => { console.error(e); try { ws.send(`K ${KP} ${SAFE_KI} ${KD} ${KV}`); } catch {} process.exit(1); });
