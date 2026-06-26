#!/usr/bin/env node
// Push-Recovery 量測：操作員親手推平台，量「擾動後回正」的衰減特性。
// 你是操作員（推的人）；腳本只負責安全地 包夾 enable→錄製→disable，不代你決定何時推。
//
// 為何這實驗比 gate_a.js 好：擾動是「真實物理脈衝」（你的手），不是軟體階躍。
// 受限條件（有支架 + holding 力矩）下平台能回正 → 正好可量到衰減率 σ / 阻尼 ζ。
// 真正的問題答案來自「掃頻」：同一套推法在 L=20/10/6ms 各做一段，比較 σ 隨迴圈速度怎麼變。
//   σ 隨迴圈變快而增大 → 頻寬是真槓桿（bandwidth-limited，值得換 TWAI/1M）
//   σ 跨速率持平       → 迴圈速度沒用，是控制律/機械問題
//
// ⚠️ 前置（人在硬體旁親手做）：
//   1. 支架就位、平台置中位、確認 holding 下推一下能回正（你已實測 OK）
//   2. server 在跑、ok:6、姿態對稱
//   3. 手放電源旁，任何一段失控 → Ctrl-C（會自動 D 斷電 + 停錄）
//
// 用法:
//   node sysid/recover.js            # 掃頻：依序跑 20/10/6ms，每段你按 Enter 開始/結束
//   node sysid/recover.js 10         # 只跑單一迴圈 10ms

const WebSocket = require('ws');
const readline = require('readline');

const HOST = 'localhost:3000';
const ALL_LOOPS = [20, 10, 6];           // 50 / 100 / ~167 Hz
const NEUTRAL = [0, 0, 105, 0, 0, 0];    // 與 gate_a 一致的中位姿態

const arg = process.argv[2] ? parseInt(process.argv[2], 10) : null;
const LOOPS = arg ? [arg] : ALL_LOOPS;

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then((r) => r.json());

const rl = readline.createInterface({ input: process.stdin, output: process.stdout });
const ask = (q) => new Promise((res) => rl.question(q, () => res()));

let ws = null;
let recording = false;

async function safeStop() {
  try { if (recording) { await rest('rec/stop'); recording = false; } } catch {}
  try { if (ws && ws.readyState === 1) { ws.send('D'); ws.send('L 20'); } } catch {}
}

process.on('SIGINT', async () => {
  console.log('\n⛔ Ctrl-C：斷電 + 停錄...');
  await safeStop();
  await sleep(400);
  process.exit(0);
});

async function main() {
  ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  const send = (c) => { ws.send(c); console.log(`  → ${c}`); };

  send('A0'); await sleep(300);   // 乾淨輪詢（TEC=0），不用會 redline 的 auto-return
  send('P ' + NEUTRAL.join(' ')); await sleep(300);

  const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '').slice(0, 13);
  console.log(`\n=== Push-Recovery 量測 (session ${ts}) ===`);
  console.log('每段流程：Enter 開始 → 推 6~8 次（每次推完等它穩定 ~3s）→ Enter 結束。\n');

  for (const loop of LOOPS) {
    const hz = Math.round(1000 / loop);
    console.log(`\n----- ${hz}Hz (loop ${loop}ms) -----`);
    send(`L ${loop}`); await sleep(300);
    await ask(`  [Enter] 開始 ${hz}Hz：會 enable 閉迴圈（target=當前中位，不暴衝）... `);

    send('E'); await sleep(1500);               // enable，hold 當前姿態
    const name = `recover_${hz}Hz_${ts}`;
    await rest(`rec/start?name=${name}`);
    recording = true;
    console.log(`  ● 錄製中 [${name}]。現在推它 6~8 次，每次推完等 ~3s 讓它回正/發散。`);

    await ask('  [Enter] 結束本段 ');
    const r = await rest('rec/stop');
    recording = false;
    console.log(`  ${r.lines} lines → ${r.path}`);
    send('D'); await sleep(800);                // 段間斷電，安全
  }

  send('L 20');
  console.log('\n完成。分析：uv run sysid/recover_analyze.py');
  rl.close();
  ws.close();
}

main().catch(async (e) => { console.error(e); await safeStop(); process.exit(1); });
