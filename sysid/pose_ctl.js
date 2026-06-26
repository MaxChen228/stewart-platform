#!/usr/bin/env node
// 姿態調控 host：把「相對中位的 6-DOF 偏移」用 kin.js IK 算成六軸 Δenc，發 G 指令（HOLD 下平滑移動）。
// SoT：IK 走 kin.js（與 disturb_modes/disturb 同源）；MOTOR_SIGN/POSE0 取自 disturb_modes，不重定義。
// 前提：平台須在 HOLD（pos=1）；Δ 相對 H 鎖位時的基準姿態、不累積（再發即覆蓋目標）。
//
// 用法: node sysid/pose_ctl.js <dx> <dy> <dz> <droll> <dpitch> <dyaw> [ms=1500]
//   node sysid/pose_ctl.js 0 0 5 0 0 0          # heave +5mm（升）
//   node sysid/pose_ctl.js 0 0 0 3 0 0          # roll +3°
//   node sysid/pose_ctl.js 0 0 0 0 0 0          # 回中位
'use strict';
const { deltaAngle } = require('./kin.js');
const { MOTOR_SIGN, POSE0 } = require('./disturb_modes.js');
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));

const off = [0, 1, 2, 3, 4, 5].map(i => parseFloat(process.argv[2 + i] ?? '0'));   // [dx,dy,dz,droll,dpitch,dyaw]
const MS = parseInt(process.argv[8] ?? '1500', 10);

// 相對中位偏移 → IK 慣例 Δangle → ×MOTOR_SIGN → enc 慣例 Δenc
function poseOffsetToEncDelta(offset) {
  const p = deltaAngle(POSE0, offset);                  // ik(POSE0+offset) − ik(POSE0)
  if (!p.valid) throw new Error('IK invalid（offset 超出工作空間）');
  return p.d.map((v, i) => v * MOTOR_SIGN[i]);
}

(async () => {
  const d = poseOffsetToEncDelta(off);
  console.log(`偏移 [${off.join(' ')}] → Δenc=[${d.map(x => x.toFixed(2)).join(' ')}]  ${MS}ms`);

  const s = await rest('latest');
  if (s.pos !== 1) { console.log('⚠ 不在 HOLD（pos≠1），先送 H 鎖位再用 G'); process.exit(1); }

  const ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  ws.send(`G ${d.map(x => x.toFixed(3)).join(' ')} ${MS}`);
  await new Promise(r => setTimeout(r, 400));
  ws.close();
  console.log('✓ G 已送');
  process.exit(0);
})().catch(e => { console.error('✗', e.message); process.exit(1); });
