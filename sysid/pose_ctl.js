#!/usr/bin/env node
// 姿態調控 host CLI：把「相對中位的 6-DOF 偏移」換算成絕對 pose，發 P 指令（HOLD 下韌體 IK→死咬）。
// SoT：IK 預檢走 kin.js（與網頁/disturb 同源）。前端不算馬達角度，韌體用 inverse_kinematics 解。
// 前提：平台須在 HOLD（pos=1）；P 為絕對姿態，再發即覆蓋目標（非累積）。
//
// 用法: node sysid/pose_ctl.js <dx> <dy> <dz> <droll> <dpitch> <dyaw> [ms=1500]
//   node sysid/pose_ctl.js 0 0 10 0 0 0         # heave +10mm（升）
//   node sysid/pose_ctl.js 0 0 0 3 0 0          # roll +3°
//   node sysid/pose_ctl.js 0 0 0 0 0 0          # 回中位
'use strict';
const { ik, NEUTRAL_Z } = require('./kin.js');
const WebSocket = require('ws');
const HOST = 'localhost:3000';
const rest = (p) => fetch(`http://${HOST}/api/${p}`).then(r => r.json()).catch(() => ({}));

const off = [0, 1, 2, 3, 4, 5].map(i => parseFloat(process.argv[2 + i] ?? '0'));   // [dx,dy,dz,droll,dpitch,dyaw]
const MS = parseInt(process.argv[8] ?? '1500', 10);

// 相對中位偏移 → 絕對 pose（xy/rpy 中位=0、z 疊 NEUTRAL_Z）
function offsetToAbsPose(offset) {
  const pose = [offset[0], offset[1], NEUTRAL_Z + offset[2], offset[3], offset[4], offset[5]];
  if (!ik(pose).valid) throw new Error('IK invalid（pose 超出工作空間）');
  return pose;
}

(async () => {
  const pose = offsetToAbsPose(off);
  console.log(`偏移 [${off.join(' ')}] → 絕對 pose [${pose.map(x => x.toFixed(2)).join(' ')}]  ${MS}ms`);

  const s = await rest('latest');
  if (s.pos !== 1) { console.log('⚠ 不在 HOLD（pos≠1），先送 H 鎖位再用 P'); process.exit(1); }

  const ws = new WebSocket(`ws://${HOST}`);
  await new Promise((r, j) => { ws.on('open', r); ws.on('error', j); });
  ws.send(`P ${pose.map(x => x.toFixed(3)).join(' ')} ${MS}`);
  await new Promise(r => setTimeout(r, 400));
  ws.close();
  console.log('✓ P 已送');
  process.exit(0);
})().catch(e => { console.error('✗', e.message); process.exit(1); });
