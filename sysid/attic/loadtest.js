#!/usr/bin/env node
// 負載矩陣測試：auto-return 模式下，掃各控制迴圈週期 × {disable, hold}，
// 量 CAN bus 負載（掉幀率、RX overflow、TEC/REC、實測 lhz）。
// hold(H) = 馬達鎖位純送 F5，是最安全的「通電」狀態（不跑會發散的 PD）。
//
// 用法: node sysid/loadtest.js
// 產出: sysid/data/load_ar_L<ms>_<state>_*.jsonl（每個再用 analyze 看）

const WebSocket = require('ws');
const HOST = 'localhost:3000';
const LOOPS = [20, 10, 5, 3];   // 控制迴圈週期 ms
const SECS = 6;                  // 每段錄製秒數
const AR_MS = 3;                 // auto-return 上報週期

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then((r) => r.json());

async function main() {
  const ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  const send = (c) => { ws.send(c); console.log(`  → ${c}`); };

  // 啟用 auto-return
  send(`AR ${AR_MS}`); await sleep(300);
  send('A1'); await sleep(800);

  async function run(loop, state) {
    send(`L ${loop}`); await sleep(300);
    send(state);       await sleep(1200);  // 切狀態 + 穩定
    const name = `load_ar_L${loop}_${state === 'H' ? 'hold' : 'dis'}`;
    console.log(`[REC] ${name}`);
    await rest(`rec/start?name=${name}`);
    await sleep(SECS * 1000);
    const r = await rest('rec/stop');
    console.log(`      ${r.lines} lines`);
  }

  // 先全部 disable（完全安全，馬達不通電）
  console.log('\n===== DISABLE 矩陣 =====');
  for (const loop of LOOPS) { send('D'); await sleep(500); await run(loop, 'D'); }

  // 再全部 hold（馬達鎖位通電）
  console.log('\n===== HOLD 矩陣（馬達通電鎖位）=====');
  for (const loop of LOOPS) await run(loop, 'H');

  // 收尾：回安全狀態
  send('D');    await sleep(500);
  send('L 20'); await sleep(300);
  console.log('\n完成。回到 disable / 20ms。');
  ws.close();
}
main().catch((e) => { console.error(e); process.exit(1); });
