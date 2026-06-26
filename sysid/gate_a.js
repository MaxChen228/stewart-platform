#!/usr/bin/env node
// Gate A：量「控制迴圈速度 vs 穩定度」——整個專案的核心問題。
// 在各迴圈頻率下 enable（Task-space PD，holds 當前姿態）→ 注入小 z 階躍擾動
// → 記錄 pose error 是收斂(穩)還是發散(不穩)。
//
// ⚠️ 前置（必須人在硬體旁親手做，腳本不代勞）：
//   1. 把平台擺到中位（各下腿朝上 ≈ 90°）
//   2. 送 Z 重新歸零（斷電被手轉過，舊校正失效）
//   3. 確認周圍淨空、手放在電源開關旁可隨時切斷
//   4. 確認 server 在跑、ok:6
// 然後: node sysid/gate_a.js
//
// 安全設計：每次 run 之間 D（斷電）；擾動僅 z +4mm（工作空間內）；每段短；
//   韌體 coord>8192 會自動 posStop。任何一段看到發散，Ctrl-C 後送 D。

const WebSocket = require('ws');
const HOST = 'localhost:3000';
const LOOPS_MS = [20, 10, 6];     // 50 / 100 / ~167 Hz
const NEUTRAL = [0, 0, 105, 0, 0, 0];
const STEP_Z = 4;                  // mm 階躍擾動
const SETTLE = 3.0;                // 每段觀察秒數

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then((r) => r.json());
const P = (pose) => `P ${pose.join(' ')}`;

async function main() {
  const ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  const send = (c) => { ws.send(c); console.log(`  → ${c}`); };

  // 確認校正 + 模式（輪詢、乾淨 bus；Task-space PD = controlMode 1）
  send('A0'); await sleep(300);      // 用乾淨輪詢，不用 redline 的 auto-return
  send('CM 1'); await sleep(300);    // 明確進 task-space PD：韌體預設 controlMode=0，不送就跑 joint-space（本實驗要測的是 PD）
  console.log('\n⚠️ 確認平台已置中位且已 Z 歸零、周圍淨空。3 秒後開始...');
  await sleep(3000);

  for (const loop of LOOPS_MS) {
    const hz = Math.round(1000 / loop);
    console.log(`\n===== ${hz}Hz (loop ${loop}ms) =====`);
    send(`L ${loop}`); await sleep(300);
    send('E'); await sleep(1500);                 // enable，target=當前姿態（不暴衝）
    // 真實數據說話：確認本段確實在 task-space PD（cm===1）。FK 連續失敗 5 次韌體會自動
    // fallback 回 joint-space（cm→0），那段數據就不是 PD，標籤會錯——當場標記存疑。
    const cm0 = (await rest('latest')).cm;
    if (cm0 !== 1) console.warn(`  ⚠ 本段 cm=${cm0}≠1：task-space PD 未生效或已 fallback joint-space，此段數據模式存疑`);
    const name = `gateA_${hz}Hz`;
    await rest(`rec/start?name=${name}`);
    send(P([0, 0, 105 + STEP_Z, 0, 0, 0]));       // 注入 +4mm z 階躍
    await sleep(SETTLE * 1000);                    // 觀察收斂/發散
    send(P(NEUTRAL));                              // 回中位
    await sleep(SETTLE * 1000);
    const r = await rest('rec/stop');
    console.log(`      ${r.lines} lines → ${r.path}`);
    send('D'); await sleep(800);                   // 每段間斷電，安全
  }

  send('L 20');
  console.log('\n完成。所有段已 disable。用 analyze.py 看各檔的 err 包絡（收斂=穩/發散=不穩）。');
  ws.close();
}
main().catch((e) => { console.error(e); process.exit(1); });
