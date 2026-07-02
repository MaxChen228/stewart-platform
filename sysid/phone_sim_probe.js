#!/usr/bin/env node
'use strict';
// ===== 深度手機模擬探針：headless sim-owner 驅動實體板 + 全原始落盤 + 追蹤/延遲量測 =====
//
// 一次調用 = 一個 config 的一輪:
//   [--fe/--predict 先設參] → (--arm: H→P landing→P home 起飛) → MODE sim →
//   gen.js 種子固定 60Hz 串流(bootstrap+phone pipe,與真手機同構含 client predict 0.08) →
//   PF 經 server(安全殼投影+100Hz resampler) → 板 → 收 telemetry/followtgt/latency 全落盤 →
//   MODE off → 量測(馬達空間互相關 lag + 追蹤 RMS) → JSON 輸出。掃參由外層迴圈跑多輪(同 seed 可比)。
//
// 用法:
//   node sysid/phone_sim_probe.js --label base --secs 70 --seed 1337 [--fe 0.85] [--predict 0.06]
//        [--arm] [--land] [--host localhost:3000] [--json]
// 安全:意圖經 gen IK 守界+server 安全殼投影;WS 斷線 server 端 MODE off;異常計數(ef/ikFail)增量即中止。
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');
const G = require('./phone_gen/gen.js');
const Kin = require('./kin.js');
const WE = require('./workspace_envelope.js');

const args = process.argv.slice(2);
const opt = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const flag = (k) => args.includes('--' + k);
const HOST = opt('host', 'localhost:3000');
const SECS = Number(opt('secs', 70));
const SEED = Number(opt('seed', 1337));
const LABEL = opt('label', 'run');
const FE = opt('fe', null), PREDICT = opt('predict', null);
const OUT_DIR = path.join(__dirname, 'data', 'sim-drive');

const httpGet = (p) => fetch(`http://${HOST}${p}`).then(r => r.json()).catch(() => null);
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

async function main() {
  fs.mkdirSync(OUT_DIR, { recursive: true });
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const runPath = path.join(OUT_DIR, `sim_${stamp}_${LABEL}.jsonl`);
  const log = fs.createWriteStream(runPath, { flags: 'a' });
  const t0 = performance.now();
  let logOpen = true;
  const W = (type, d) => { if (logOpen) log.write(JSON.stringify({ t: +(performance.now() - t0).toFixed(1), type, ...d }) + '\n'); };

  const ws = new WebSocket(`ws://${HOST}`);
  const acks = [];
  const tele = [];          // {t, a:[6]}
  const lat = [];           // server e2e 互相關樣本
  let ef0 = null, ikFail0 = null, abort = null;

  ws.on('message', (m) => {
    const s = m.toString();
    let d = null; try { d = JSON.parse(s); } catch { return; }
    if (d.evt === 'latency') { if (d.e2e != null) lat.push(d.e2e); W('latency', { e2e: d.e2e, corr: d.corr }); return; }
    if (d.evt === 'followtgt') { W('followtgt', { pose: d.pose }); return; }
    if (d.evt) { W('evt', { evt: d.evt, d }); return; }
    if (Array.isArray(d.a) && d.a.length === 6) {
      tele.push({ t: performance.now() - t0, a: d.a });
      W('tele', { a: d.a, ef: d.ef, ikFail: d.ctl && d.ctl.ikFail });
      if (ef0 == null && Number.isFinite(d.ef)) { ef0 = d.ef; ikFail0 = (d.ctl && d.ctl.ikFail) || 0; }
      else if (ef0 != null) {
        const dEf = (d.ef || 0) - ef0, dIk = ((d.ctl && d.ctl.ikFail) || 0) - ikFail0;
        if (dEf > 20 || dIk > 200) abort = `anomaly: dEf=${dEf} dIkFail=${dIk}`;
      }
      return;
    }
    if (d.status || d.ota) { acks.push(s); W('ack', { s }); }
  });
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  ws.send(JSON.stringify({ role: 'sim' }));

  const send = (c) => { ws.send(c); W('cmd', { c }); };
  const waitAck = async (substr, ms = 2500) => {
    const until = performance.now() + ms;
    while (performance.now() < until) { if (acks.some(a => a.includes(substr))) return true; await sleep(50); }
    return false;
  };

  // ---- 配置 ----
  if (FE != null) { send(`FE ${FE}`); if (!await waitAck('follow tight')) console.error('⚠ FE ack timeout'); }
  if (PREDICT != null) { send(`PREDICT ${PREDICT}`); if (!await waitAck('"predict"')) console.error('⚠ PREDICT ack timeout'); }

  // ---- rec(全原始落盤於 server 端,與本地 jsonl 互補)----
  await httpGet(`/api/rec/start?name=simprobe_${LABEL}`);

  // ---- 起飛 ----
  const cfg = await httpGet('/api/platform-config');
  const home = cfg && cfg.homePose ? cfg.homePose.map(Number) : [0, 0, 133, 0, 0, 0];
  const landing = cfg && cfg.landingPose ? cfg.landingPose.map(Number) : null;
  if (flag('arm')) {
    send('H'); await sleep(900);
    if (landing) { send(`P ${landing.join(' ')} 1200`); await sleep(1600); }
    send(`P ${home.join(' ')} 1500`); await sleep(2100);
  } else if (flag('home')) {
    send(`P ${home.join(' ')} 1200`); await sleep(1700);   // 已使能(HOLD)跑間回 home 對齊起點
  }

  // ---- 進 sim 模式 ----
  send('MODE sim'); await sleep(600);

  // ---- 60Hz 串流(絕對時鐘;意圖=本地同款投影後 pose,與 server 主閘位元一致)----
  const rng = G.mulberry32(SEED);
  const sessions = G.loadSessions(G.defaultRefFiles());
  const stream = G.makeBootstrapStream(sessions, rng);
  const pipe = G.makePhonePipe();
  const project = WE.makeSafeProjector(Kin, { knee: 0.8 });
  const intents = [];   // {t, ang:[6]} 投影後意圖的馬達角(量測用)
  let clampedN = 0, sent = 0;
  const period = 1000 / G.RATE, tEnd = performance.now() + SECS * 1000;
  let nextAt = performance.now();
  await new Promise((done) => {
    const tick = () => {
      if (abort || performance.now() >= tEnd) return done();
      const rel = stream.next();
      const { pose } = pipe.step(rel);
      const pr = project(pose, home);
      if (pr.clamped) clampedN++;
      send(`PF ${pose.map(v => v.toFixed(3)).join(' ')}`);
      sent++;
      const k = Kin.ik(pr.pose);
      if (k.valid) intents.push({ t: performance.now() - t0, ang: k.angles.map(Number) });
      W('intent', { pose: pr.pose.map(v => +v.toFixed(3)), clamped: pr.clamped ? 1 : 0 });
      nextAt += period;
      const now = performance.now();
      if (nextAt <= now) nextAt = now + period;
      setTimeout(tick, Math.max(0, nextAt - now));
    };
    tick();
  });

  send('MODE off'); await sleep(400);
  await httpGet('/api/rec/stop');
  const report = await httpGet('/api/follow/report');
  W('report', { report });

  // ---- 量測:馬達空間互相關 lag + 追蹤 RMS(掐頭 5s settle)----
  const T0 = 5000, interp = (buf, m, t) => {
    let lo = 0, hi = buf.length - 1;
    if (t < buf[0].t || t > buf[hi].t) return null;
    while (hi - lo > 1) { const md = (lo + hi) >> 1; if (buf[md].t <= t) lo = md; else hi = md; }
    const a = buf[lo], b = buf[hi], f = (t - a.t) / Math.max(1e-6, b.t - a.t);
    return a.ang ? a.ang[m] + f * (b.ang[m] - a.ang[m]) : a.a[m] + f * (b.a[m] - a.a[m]);
  };
  const teleB = tele.map(x => ({ t: x.t, a: x.a }));
  const metrics = { motors: [], lagMedMs: null, trkRmsMed: null, rawRmsMed: null };
  if (intents.length > 100 && teleB.length > 50) {
    const tA = Math.max(intents[0].t, teleB[0].t) + T0, tB = Math.min(intents[intents.length - 1].t, teleB[teleB.length - 1].t);
    const grid = []; for (let t = tA + 600; t <= tB; t += 20) grid.push(t);
    for (let m = 0; m < 6; m++) {
      let bestLag = 0, bestC = -2;
      for (let lag = 0; lag <= 600; lag += 10) {
        const A = [], C = [];
        for (const t of grid) { const a = interp(teleB, m, t), c = interp(intents, m, t - lag); if (a != null && c != null) { A.push(a); C.push(c); } }
        if (A.length < 50) continue;
        const n = A.length, ma = A.reduce((x, y) => x + y) / n, mc = C.reduce((x, y) => x + y) / n;
        let saa = 0, scc = 0, sac = 0;
        for (let i = 0; i < n; i++) { const da = A[i] - ma, dc = C[i] - mc; saa += da * da; scc += dc * dc; sac += da * dc; }
        const corr = saa > 0 && scc > 0 ? sac / Math.sqrt(saa * scc) : 0;
        if (corr > bestC) { bestC = corr; bestLag = lag; }
      }
      let se = 0, se0 = 0, n = 0;
      for (const t of grid) {
        const a = interp(teleB, m, t), cL = interp(intents, m, t - bestLag), c0 = interp(intents, m, t);
        if (a != null && cL != null && c0 != null) { se += (a - cL) ** 2; se0 += (a - c0) ** 2; n++; }
      }
      metrics.motors.push({ m: m + 1, lagMs: bestLag, corr: +bestC.toFixed(3), trkRms: +(Math.sqrt(se / Math.max(1, n))).toFixed(3), rawRms: +(Math.sqrt(se0 / Math.max(1, n))).toFixed(3) });
    }
    const med = (xs) => { const s = [...xs].sort((a, b) => a - b); return s[(s.length / 2) | 0]; };
    const act = metrics.motors.filter(x => x.corr > 0.6);
    if (act.length) {
      metrics.lagMedMs = med(act.map(x => x.lagMs));
      metrics.trkRmsMed = +med(act.map(x => x.trkRms)).toFixed(3);
      metrics.rawRmsMed = +med(act.map(x => x.rawRms)).toFixed(3);
    }
  }
  const latMed = lat.length ? [...lat].sort((a, b) => a - b)[(lat.length / 2) | 0] : null;

  // ---- 降落(可選)----
  if (flag('land')) {
    if (landing) { send(`P ${home.join(' ')} 1500`); await sleep(2000); send(`P ${landing.join(' ')} 1500`); await sleep(2200); }
    send('D'); await sleep(400);
  }

  const out = {
    label: LABEL, secs: SECS, seed: SEED, fe: FE, predict: PREDICT,
    sent, clampedPct: +(100 * clampedN / Math.max(1, sent)).toFixed(1),
    teleN: tele.length, latency: { e2eMedMs: latMed, n: lat.length },
    metrics, abort, runPath,
    reportVerdict: report && report.verdict ? report.verdict : null,
  };
  W('summary', out);
  logOpen = false;
  ws.terminate();
  log.end();
  console.log(JSON.stringify(out, null, flag('json') ? 0 : 2));
  if (abort) process.exit(2);
  process.exit(0);
}
main().catch(e => { console.error('probe error:', e.message); process.exit(1); });
