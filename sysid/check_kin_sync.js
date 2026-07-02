#!/usr/bin/env node
'use strict';
// 幾何真相源 parity 閘：韌體 C++（src/kinematics.h + src/servo42d.h）↔ JS（sysid/kin.js + sysid/disturb_modes.js）。
//
// 為何需要：mm 域幾何常數是跨語言雙份（C++ 韌體 / JS 前端+工具），改一處漏一處會讓 IK 靜默分歧
// → 幾何錯誤直接發到六顆實體馬達。這支做「常數 parity」（零依賴、免編譯的第一道防線）；
// 行為級 golden-vector 比對需編韌體，留待有動機再做。
// 跑法: node sysid/check_kin_sync.js（npm test 會跑）。不符 → 印 diff + exit 1。

const fs = require('fs');
const path = require('path');
const Kin = require('./kin');
const DM = require('./disturb_modes');

const KIN_H = path.join(__dirname, '..', 'src', 'kinematics.h');
const SERVO_H = path.join(__dirname, '..', 'src', 'servo42d.h');

const problems = [];
const near = (a, b, tol = 1e-4) => Math.abs(a - b) <= tol;

// ---- 解析 kinematics.h ----
const kh = fs.readFileSync(KIN_H, 'utf8');
function hScalar(name) {
  const m = kh.match(new RegExp(`constexpr\\s+float\\s+${name}\\s*=\\s*([-0-9.]+)f?`));
  if (!m) { problems.push(`kinematics.h: 找不到 constexpr float ${name}`); return NaN; }
  return parseFloat(m[1]);
}
function hArray(name) {
  const m = kh.match(new RegExp(`${name}\\s*\\[6\\]\\s*=\\s*\\{([^}]*)\\}`));
  if (!m) { problems.push(`kinematics.h: 找不到陣列 ${name}[6]`); return null; }
  return m[1].split(',').map((s) => parseFloat(s.trim()));
}

// 純量常數（kin.js 直接 export 者）
const scalarPairs = [
  ['BASE_RADIUS', Kin.BASE_RADIUS],
  ['PLATFORM_RADIUS', Kin.PLATFORM_RADIUS],
  ['LOWER_LEG', Kin.LOWER_LEG],
  ['UPPER_LEG', Kin.UPPER_LEG],
  ['NEUTRAL_Z', Kin.NEUTRAL_Z],
];
for (const [name, jsVal] of scalarPairs) {
  const hVal = hScalar(name);
  if (!near(hVal, jsVal)) problems.push(`${name}: kinematics.h=${hVal} ≠ kin.js=${jsVal}`);
}

// MOTOR_PLANE_ANGLE 陣列（字面數值）
const hMPA = hArray('MOTOR_PLANE_ANGLE');
if (hMPA) {
  if (hMPA.length !== 6 || Kin.MOTOR_PLANE_ANGLE.length !== 6) problems.push('MOTOR_PLANE_ANGLE 長度非 6');
  else for (let i = 0; i < 6; i++) if (!near(hMPA[i], Kin.MOTOR_PLANE_ANGLE[i])) problems.push(`MOTOR_PLANE_ANGLE[${i}]: kinematics.h=${hMPA[i]} ≠ kin.js=${Kin.MOTOR_PLANE_ANGLE[i]}`);
}

// BASE_ANGLES / PLATFORM_ANGLES：.h 是純量的表達式（BASE_ANGLE/2+210 …），kin.js 已算好。
// 用 .h 純量 + CW 生成規則算預期值，比對 kin.js 的陣列 → 同時驗證 BASE_ANGLE/PLATFORM_ANGLE 純量 + 排列規則。
const SPOKES = [210, 210, 90, 90, -30, -30], SGN = [1, -1, 1, -1, 1, -1];
function expectAngles(halfAngleName) {
  const half = hScalar(halfAngleName) / 2;
  return SPOKES.map((s, i) => s + SGN[i] * half);
}
for (const [name, hAngle, jsArr] of [
  ['BASE_ANGLES', 'BASE_ANGLE', Kin.BASE_ANGLES],
  ['PLATFORM_ANGLES', 'PLATFORM_ANGLE', Kin.PLATFORM_ANGLES],
]) {
  const exp = expectAngles(hAngle);
  for (let i = 0; i < 6; i++) if (!near(exp[i], jsArr[i])) problems.push(`${name}[${i}]: kinematics.h 推算=${exp[i]} ≠ kin.js=${jsArr[i]}`);
}

// ---- MOTOR_SIGN：servo42d.h ↔ disturb_modes.js（kin.js 不 export 此常數）----
const sh = fs.readFileSync(SERVO_H, 'utf8');
const mSign = sh.match(/MOTOR_SIGN\[NUM_MOTORS\]\s*=\s*\{([^}]*)\}/);
if (!mSign) problems.push('servo42d.h: 找不到 MOTOR_SIGN[NUM_MOTORS]');
else {
  const hSign = mSign[1].split(',').map((s) => parseInt(s.trim(), 10));
  for (let i = 0; i < 6; i++) if (hSign[i] !== DM.MOTOR_SIGN[i]) problems.push(`MOTOR_SIGN[${i}]: servo42d.h=${hSign[i]} ≠ disturb_modes.js=${DM.MOTOR_SIGN[i]}`);
}

if (problems.length) {
  console.error('✗ 幾何真相源不同步：');
  for (const p of problems) console.error('  - ' + p);
  console.error('\n改幾何時 src/kinematics.h（+ MOTOR_SIGN 在 src/servo42d.h）與 sysid/kin.js（+ disturb_modes.js）須一起改。');
  process.exit(1);
}
console.log('✓ 幾何真相源同步：kinematics.h ↔ kin.js（+ MOTOR_SIGN servo42d.h ↔ disturb_modes.js）一致');
