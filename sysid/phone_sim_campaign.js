#!/usr/bin/env node
'use strict';
// ===== 深度手機模擬戰役總指揮:等東風(馬達上電)→ 全自動跑完 → 降落 → 出報告 =====
//
// 流程:
//   1. [--wait-bus] 輪詢 /api/latest 直到 6/6 馬達回應(連續 3 次)= 馬達已 power-cycle 回歸
//   2. ESP32 重啟(esp32_reset.js;鐵律:馬達 power-cycle 後映射須重建)
//   3. 驗收:ok=6、TEC<10、a[] 非 placeholder 且 |a|<120
//   4. 掃參矩陣(phone_sim_probe.js 逐輪,同 seed 可比):
//        r0b  FE0.70 P0     (--arm 起飛)
//        r1   FE0.85 P0     (--home)
//        r2   FE1.00 P0     (--home)      → 選 FE*(trkRms 最低,平手取 lag 低)
//        r3   FE*   P0.06   (--home)
//        r4   FE*   P0.10   (--home)      → 選 P*(同準則;PREDICT 設值即存 NVS)
//        r5   FE*   P*      (--home --land, seed 4242 換軌跡驗穩健) → 落地斷電
//   5. 任一輪 abort(異常計數/預檢失敗)→ 中止 + 嘗試降落
//   6. 匯總 campaign_<stamp>.json + console 對比表
//
// 用法: node sysid/phone_sim_campaign.js [--wait-bus] [--secs 70] [--seed 1337] [--host localhost:3000]
const { spawnSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const args = process.argv.slice(2);
const opt = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const flag = (k) => args.includes('--' + k);
const HOST = opt('host', 'localhost:3000');
const SECS = Number(opt('secs', 70));
const SEED = Number(opt('seed', 1337));
const OUT_DIR = path.join(__dirname, 'data', 'sim-drive');

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const latest = () => fetch(`http://${HOST}/api/latest`).then(r => r.json()).catch(() => null);
const log = (m) => console.log(`[campaign ${new Date().toISOString().slice(11, 19)}] ${m}`);

function runProbe(label, extra) {
  const argv = [path.join(__dirname, 'phone_sim_probe.js'), '--label', label,
    '--secs', String(SECS), '--seed', String(SEED), '--host', HOST, '--json', ...extra];
  log(`▶ ${label}: node ${argv.slice(1).join(' ')}`);
  const r = spawnSync('node', argv, { encoding: 'utf8', timeout: (SECS + 90) * 1000 });
  const lines = (r.stdout || '').trim().split('\n');
  let summary = null;
  for (let i = lines.length - 1; i >= 0; i--) {
    try { const d = JSON.parse(lines[i]); if (d.label || d.abort) { summary = d; break; } } catch {}
  }
  if (r.stderr && r.stderr.trim()) log(`  stderr: ${r.stderr.trim().slice(0, 200)}`);
  return { status: r.status, summary };
}

async function landViaWs(reason) {
  log(`嘗試降落(${reason})`);
  try {
    const cfg = await fetch(`http://${HOST}/api/platform-config`).then(r => r.json());
    const WebSocket = require('ws');
    const ws = new WebSocket(`ws://${HOST}`);
    await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
    ws.send(`P ${cfg.homePose.join(' ')} 1800`); await sleep(2400);
    ws.send(`P ${cfg.landingPose.join(' ')} 1800`); await sleep(2500);
    ws.send('D'); await sleep(500);
    ws.terminate();
    log('降落序列已送出');
  } catch (e) { log(`降落失敗: ${e.message}(改用 /api/land)`); await fetch(`http://${HOST}/api/land`, { method: 'POST' }).catch(() => {}); }
}

async function main() {
  const results = [];
  const t0 = Date.now();

  // ---- 1. 等東風 ----
  if (flag('wait-bus')) {
    log('等待馬達上電(輪詢 /api/latest,ok=6 連續 3 次)…');
    let streak = 0;
    for (;;) {
      const d = await latest();
      streak = d && d.ok === 6 ? streak + 1 : 0;
      if (streak >= 3) break;
      await sleep(3000);
    }
    log('馬達回應 6/6——東風已到');
    await sleep(3000);   // 上電穩定
  }

  // ---- 2. ESP32 重啟(映射鐵律)----
  log('重啟 ESP32(重建 sessionZeroRaw 映射)…');
  const rr = spawnSync('node', [path.join(__dirname, 'esp32_reset.js'), '--host', HOST], { encoding: 'utf8', timeout: 120000 });
  log(`  reset: ${(rr.stdout || rr.stderr || '').trim()}`);
  if (rr.status !== 0) { log('✗ ESP32 重啟失敗,中止'); process.exit(1); }
  await sleep(3000);

  // ---- 3. 驗收 ----
  const d = await latest();
  const aOk = d && Array.isArray(d.a) && d.a.every(v => Math.abs(v) < 120) && new Set(d.a.map(v => v.toFixed(2))).size > 1;
  const tec = d && d.can ? d.can.tx : 999;
  if (!d || d.ok !== 6 || tec >= 10 || !aOk) {
    log(`✗ 驗收失敗: ok=${d && d.ok} tec=${tec} aOk=${aOk} —— bus 未健康,中止(不驅動)`);
    process.exit(1);
  }
  log(`✓ 驗收: ok=6 tec=${tec} a=[${d.a.map(v => v.toFixed(1)).join(',')}]`);

  // ---- 4. 掃參矩陣 ----
  const pick = (cands) => cands.filter(x => x && x.metrics && x.metrics.trkRmsMed != null)
    .sort((x, y) => (x.metrics.trkRmsMed - y.metrics.trkRmsMed) || (x.metrics.lagMedMs - y.metrics.lagMedMs))[0];
  const abortAll = async (why, r) => {
    log(`✗ 戰役中止: ${why} ${r && r.summary ? JSON.stringify(r.summary.abort || r.summary) : ''}`);
    await landViaWs(why);
    finishReport(results, null, null, `aborted: ${why}`);
    process.exit(2);
  };

  const feMatrix = [
    { label: 'r0b_fe070_p000', fe: '0.70', predict: '0', extra: ['--arm'] },
    { label: 'r1_fe085_p000', fe: '0.85', predict: '0', extra: ['--home'] },
    { label: 'r2_fe100_p000', fe: '1.00', predict: '0', extra: ['--home'] },
  ];
  for (const m of feMatrix) {
    const r = runProbe(m.label, ['--fe', m.fe, '--predict', m.predict, ...m.extra]);
    if (r.status !== 0 || !r.summary || r.summary.abort) return abortAll(m.label, r);
    r.summary._fe = m.fe; r.summary._predict = m.predict;
    results.push(r.summary);
    log(`  ✓ ${m.label}: lag=${r.summary.metrics.lagMedMs}ms trkRms=${r.summary.metrics.trkRmsMed}° e2e=${r.summary.latency.e2eMedMs}ms`);
    await sleep(2000);
  }
  const feBest = pick(results);
  if (!feBest) return abortAll('FE 掃參無有效結果', null);
  const FE_STAR = feBest._fe;
  log(`★ FE* = ${FE_STAR}(來自 ${feBest.label})`);

  const pMatrix = [
    { label: `r3_fe${FE_STAR.replace('.', '')}_p006`, predict: '0.06' },
    { label: `r4_fe${FE_STAR.replace('.', '')}_p010`, predict: '0.10' },
  ];
  for (const m of pMatrix) {
    const r = runProbe(m.label, ['--fe', FE_STAR, '--predict', m.predict, '--home']);
    if (r.status !== 0 || !r.summary || r.summary.abort) return abortAll(m.label, r);
    r.summary._fe = FE_STAR; r.summary._predict = m.predict;
    results.push(r.summary);
    log(`  ✓ ${m.label}: lag=${r.summary.metrics.lagMedMs}ms trkRms=${r.summary.metrics.trkRmsMed}° e2e=${r.summary.latency.e2eMedMs}ms`);
    await sleep(2000);
  }
  const pBest = pick(results.filter(x => x._fe === FE_STAR));
  const P_STAR = pBest._predict;
  log(`★ P* = ${P_STAR}(來自 ${pBest.label})`);

  // ---- 5. 確認輪(換 seed 驗穩健;PREDICT 設值即存 NVS → 此輪落定持久值)+ 降落 ----
  const confirm = spawnSync('node', [path.join(__dirname, 'phone_sim_probe.js'),
    '--label', `r5_confirm_fe${FE_STAR.replace('.', '')}_p${P_STAR.replace('.', '')}`,
    '--secs', String(SECS), '--seed', '4242', '--host', HOST, '--json',
    '--fe', FE_STAR, '--predict', P_STAR, '--home', '--land'], { encoding: 'utf8', timeout: (SECS + 120) * 1000 });
  let confirmSummary = null;
  for (const line of (confirm.stdout || '').trim().split('\n').reverse()) {
    try { const x = JSON.parse(line); if (x.label || x.abort) { confirmSummary = x; break; } } catch {}
  }
  if (confirm.status !== 0 || !confirmSummary || confirmSummary.abort) {
    log('✗ 確認輪失敗'); await landViaWs('confirm failed');
  } else {
    confirmSummary._fe = FE_STAR; confirmSummary._predict = P_STAR;
    results.push(confirmSummary);
    log(`  ✓ 確認輪: lag=${confirmSummary.metrics.lagMedMs}ms trkRms=${confirmSummary.metrics.trkRmsMed}°`);
  }

  finishReport(results, FE_STAR, P_STAR, null);
  log(`戰役完成,總時長 ${Math.round((Date.now() - t0) / 60000)} 分鐘`);

  function finishReport(rs, feStar, pStar, aborted) {
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const report = { stamp, feStar, pStar, aborted, secs: SECS, seed: SEED, runs: rs };
    const p = path.join(OUT_DIR, `campaign_${stamp}.json`);
    fs.mkdirSync(OUT_DIR, { recursive: true });
    fs.writeFileSync(p, JSON.stringify(report, null, 2));
    log(`報告: ${p}`);
    console.log('\n| run | FE | PREDICT | e2e(ms) | lag(ms) | trkRms(°) | rawRms(°) | clamp% |');
    console.log('|---|---|---|---|---|---|---|---|');
    for (const r of rs) {
      console.log(`| ${r.label} | ${r._fe} | ${r._predict} | ${r.latency.e2eMedMs} | ${r.metrics.lagMedMs} | ${r.metrics.trkRmsMed} | ${r.metrics.rawRmsMed} | ${r.clampedPct} |`);
    }
  }
}
main().catch(e => { console.error('campaign error:', e.message); process.exit(1); });
