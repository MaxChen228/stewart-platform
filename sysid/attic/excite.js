#!/usr/bin/env node
// Phase 0 姿態級激勵器（次要實驗：閉迴圈系統響應，含現有控制器）。
// 透過 WebSocket 送 P 指令，並用 REST 控制記錄器在每段實驗前後 start/stop。
// 基礎 plant 隔離（開迴圈單馬達 F5 波形）需韌體新指令，不在此檔。
//
// ⚠️ 跑之前：馬達須上電、平台須能 Enable；且建議等韌體時間戳 reflash 後再跑，
//    否則 host 時間戳量不到 <20ms 致動延遲。
//
// 用法:
//   node sysid/excite.js step  <axis> <delta> [holdSec=2] [baseZ=105]
//   node sysid/excite.js chirp <axis> <amp> <f0> <f1> <durSec=30> [baseZ=105]
//   axis ∈ x y z roll pitch yaw

const WebSocket = require('ws');

const HOST = 'localhost:3000';
const NEUTRAL_Z = 105.0;
const AXES = { x: 0, y: 1, z: 2, roll: 3, pitch: 4, yaw: 5 };
// 安全上限（避免命令超出工作空間 / 撞限位）；--force 解除
const LIMIT = { x: 20, y: 20, z: 15, roll: 12, pitch: 12, yaw: 12 };

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then((r) => r.json());

function poseStr(base, axisIdx, val) {
  const p = base.slice();
  p[axisIdx] = val;
  return `P ${p.map((x) => x.toFixed(3)).join(' ')}`;
}

async function main() {
  const [mode, axisName, ...rest_] = process.argv.slice(2);
  const force = process.argv.includes('--force');
  if (!mode || !(axisName in AXES)) {
    console.log('用法: node sysid/excite.js step|chirp <axis> ...   axis ∈ x y z roll pitch yaw');
    process.exit(1);
  }
  const axisIdx = AXES[axisName];

  const ws = new WebSocket(`ws://${HOST}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  const send = (c) => { ws.send(c); console.log(`→ ${c}`); };

  const base = [0, 0, NEUTRAL_Z, 0, 0, 0];

  if (mode === 'step') {
    const delta = parseFloat(rest_[0]);
    const hold = parseFloat(rest_[1] || '2');
    if (!force && Math.abs(delta) > LIMIT[axisName]) {
      console.error(`delta ${delta} 超過 ${axisName} 安全上限 ${LIMIT[axisName]}（--force 解除）`); process.exit(1);
    }
    const name = `step_${axisName}_${delta}`;
    console.log(await rest(`rec/start?name=${name}`));
    send('E'); await sleep(800);
    send(poseStr(base, axisIdx, base[axisIdx])); await sleep(hold * 1000);      // home 穩定
    send(poseStr(base, axisIdx, base[axisIdx] + delta)); await sleep(hold * 1000); // 階躍
    send(poseStr(base, axisIdx, base[axisIdx])); await sleep(hold * 1000);       // 回 home
    send('S');
    console.log(await rest('rec/stop'));
  } else if (mode === 'chirp') {
    const amp = parseFloat(rest_[0]);
    const f0 = parseFloat(rest_[1]);
    const f1 = parseFloat(rest_[2]);
    const dur = parseFloat(rest_[3] || '30');
    if (!force && amp > LIMIT[axisName]) {
      console.error(`amp ${amp} 超過 ${axisName} 安全上限 ${LIMIT[axisName]}（--force 解除）`); process.exit(1);
    }
    const name = `chirp_${axisName}_a${amp}_${f0}-${f1}Hz`;
    console.log(await rest(`rec/start?name=${name}`));
    send('E'); await sleep(800);
    send(poseStr(base, axisIdx, base[axisIdx])); await sleep(1500);
    const dt = 0.02, n = Math.round(dur / dt);
    let phase = 0;
    for (let k = 0; k < n; k++) {
      const tt = k * dt;
      const f = f0 + (f1 - f0) * (tt / dur);  // 線性掃頻
      phase += 2 * Math.PI * f * dt;
      send(poseStr(base, axisIdx, base[axisIdx] + amp * Math.sin(phase)));
      await sleep(dt * 1000);
    }
    send(poseStr(base, axisIdx, base[axisIdx])); await sleep(1500);
    send('S');
    console.log(await rest('rec/stop'));
  } else {
    console.error(`未知模式: ${mode}`); process.exit(1);
  }
  ws.close();
}

main().catch((e) => { console.error(e); process.exit(1); });
