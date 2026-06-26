// Stewart IK — 從 web/index.html 抽出的 host 端純函數版（無 THREE 依賴）。
// 幾何常數與前端/韌體同步；CW 排列。改幾何時三處（kinematics.h / index.html / 此檔）須一致。
'use strict';
// IIFE 隔離：瀏覽器當 classic script 載入時不污染全域（否則 ik/deltaAngle/NEUTRAL_Z 等
// 與 disturb_modes.js 的 const 撞名 → "already declared"）。只 leak window.Kin / module.exports。
(function () {
const DEG = Math.PI / 180;
const BASE_RADIUS = 152, BASE_ANGLE = 18.92;
const PLATFORM_RADIUS = 103, PLATFORM_ANGLE = 28.07;
const LOWER_LEG = 65, UPPER_LEG = 165;
const NEUTRAL_Z = 105;
const MOTOR_PLANE_ANGLE = [300, 120, 180, 0, 60, 240];

const BASE_ANGLES = [
  BASE_ANGLE / 2 + 210, -BASE_ANGLE / 2 + 210,
  BASE_ANGLE / 2 + 90, -BASE_ANGLE / 2 + 90,
  BASE_ANGLE / 2 - 30, -BASE_ANGLE / 2 - 30,
];
const PLATFORM_ANGLES = [
  PLATFORM_ANGLE / 2 + 210, -PLATFORM_ANGLE / 2 + 210,
  PLATFORM_ANGLE / 2 + 90, -PLATFORM_ANGLE / 2 + 90,
  PLATFORM_ANGLE / 2 - 30, -PLATFORM_ANGLE / 2 - 30,
];

const B = [], P = [];
for (let i = 0; i < 6; i++) {
  const ba = BASE_ANGLES[i] * DEG, pa = PLATFORM_ANGLES[i] * DEG;
  B.push({ x: BASE_RADIUS * Math.cos(ba), y: BASE_RADIUS * Math.sin(ba), z: 0 });
  P.push({ x: PLATFORM_RADIUS * Math.cos(pa), y: PLATFORM_RADIUS * Math.sin(pa), z: 0 });
}

// pose = [x,y,z,roll,pitch,yaw]（mm/度）→ 六馬達角度（度）。回傳 {angles, valid}
function ik(pose) {
  const [x, y, z, roll, pitch, yaw] = pose;
  const cr = Math.cos(roll * DEG), sr = Math.sin(roll * DEG);
  const cp = Math.cos(pitch * DEG), sp = Math.sin(pitch * DEG);
  const cy = Math.cos(yaw * DEG), sy = Math.sin(yaw * DEG);
  const R = [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
  const angles = []; let valid = true;
  for (let i = 0; i < 6; i++) {
    const px = P[i].x, py = P[i].y, pz = P[i].z;
    const qx = R[0][0] * px + R[0][1] * py + R[0][2] * pz + x;
    const qy = R[1][0] * px + R[1][1] * py + R[1][2] * pz + y;
    const qz = R[2][0] * px + R[2][1] * py + R[2][2] * pz + z;
    const dx = qx - B[i].x, dy = qy - B[i].y, dz = qz - B[i].z;
    const L = dx * dx + dy * dy + dz * dz - (UPPER_LEG * UPPER_LEG - LOWER_LEG * LOWER_LEG);
    const M = 2 * LOWER_LEG * dz;
    const th = MOTOR_PLANE_ANGLE[i] * DEG;
    const N = 2 * LOWER_LEG * (Math.cos(th) * dx + Math.sin(th) * dy);
    const d = Math.sqrt(M * M + N * N);
    const ratio = L / d;
    if (ratio < -1 || ratio > 1) valid = false;     // asin clamp = 工作空間邊界
    const sa = Math.max(-1, Math.min(1, ratio));
    angles.push((Math.asin(sa) - Math.atan2(N, M)) / DEG);
  }
  return { angles, valid };
}

// 擾動 Δangle：在工作點 pose0 施加 task-space Δpose，回傳六軸角度增量（度）。
// 非線性（直接 ik 差），故大 severity 也是合法 IK 配置差，非線性化近似。
function deltaAngle(pose0, dpose) {
  const a0 = ik(pose0), a1 = ik(pose0.map((v, k) => v + dpose[k]));
  return { d: a1.angles.map((v, i) => v - a0.angles[i]), valid: a0.valid && a1.valid };
}

function gaussSolve(A, b) {
  const n = A.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let i = 0; i < n; i++) {
    let mx = i;
    for (let k = i + 1; k < n; k++) if (Math.abs(M[k][i]) > Math.abs(M[mx][i])) mx = k;
    [M[i], M[mx]] = [M[mx], M[i]];
    if (Math.abs(M[i][i]) < 1e-10) return null;
    for (let k = i + 1; k < n; k++) {
      const c = M[k][i] / M[i][i];
      for (let j = i; j <= n; j++) M[k][j] -= c * M[i][j];
    }
  }
  const x = new Array(n);
  for (let i = n - 1; i >= 0; i--) {
    x[i] = M[i][n];
    for (let j = i + 1; j < n; j++) x[i] -= M[i][j] * x[j];
    x[i] /= M[i][i];
  }
  return x;
}

function computeA(angles) {
  return angles.map((ang, i) => {
    const a = ang * DEG;
    const th = MOTOR_PLANE_ANGLE[i] * DEG;
    return {
      x: LOWER_LEG * Math.cos(a) * Math.cos(th) + B[i].x,
      y: LOWER_LEG * Math.cos(a) * Math.sin(th) + B[i].y,
      z: LOWER_LEG * Math.sin(a) + B[i].z,
    };
  });
}

let lastFKPose = [0, 0, NEUTRAL_Z, 0, 0, 0];

function resetFK(pose = [0, 0, NEUTRAL_Z, 0, 0, 0]) {
  lastFKPose = [...pose];
}

function solveFK(measured, opts = {}) {
  const A = computeA(measured);
  const warmStart = opts.warmStart || lastFKPose;
  let pose = warmStart.some((v) => !Number.isFinite(v)) ? [0, 0, NEUTRAL_Z, 0, 0, 0] : [...warmStart];
  let finalIter = 0;
  let finalErr = 999;
  let converged = false;

  for (let iter = 0; iter < 50; iter++) {
    const [x, y, z, roll, pitch, yaw] = pose;
    const cr = Math.cos(roll * DEG), sr = Math.sin(roll * DEG);
    const cp = Math.cos(pitch * DEG), sp = Math.sin(pitch * DEG);
    const cy = Math.cos(yaw * DEG), sy = Math.sin(yaw * DEG);
    const R = [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
    ];
    const dRr = [
      [0, cy * sp * cr + sy * sr, -cy * sp * sr + sy * cr],
      [0, sy * sp * cr - cy * sr, -sy * sp * sr - cy * cr],
      [0, cp * cr, -cp * sr],
    ];
    const dRp = [
      [-cy * sp, cy * cp * sr, cy * cp * cr],
      [-sy * sp, sy * cp * sr, sy * cp * cr],
      [-cp, -sp * sr, -sp * cr],
    ];
    const dRy = [
      [-sy * cp, -sy * sp * sr - cy * cr, -sy * sp * cr + cy * sr],
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [0, 0, 0],
    ];

    const f = [];
    const J = [];
    for (let i = 0; i < 6; i++) {
      const px = P[i].x, py = P[i].y;
      const qx = R[0][0] * px + R[0][1] * py + x;
      const qy = R[1][0] * px + R[1][1] * py + y;
      const qz = R[2][0] * px + R[2][1] * py + z;
      const vx = qx - A[i].x, vy = qy - A[i].y, vz = qz - A[i].z;
      f.push(vx * vx + vy * vy + vz * vz - UPPER_LEG * UPPER_LEG);

      const drx = dRr[0][0] * px + dRr[0][1] * py;
      const dry = dRr[1][0] * px + dRr[1][1] * py;
      const drz = dRr[2][0] * px + dRr[2][1] * py;
      const dpx = dRp[0][0] * px + dRp[0][1] * py;
      const dpy = dRp[1][0] * px + dRp[1][1] * py;
      const dpz = dRp[2][0] * px + dRp[2][1] * py;
      const dyx = dRy[0][0] * px + dRy[0][1] * py;
      const dyy = dRy[1][0] * px + dRy[1][1] * py;
      J.push([
        2 * vx,
        2 * vy,
        2 * vz,
        2 * (vx * drx + vy * dry + vz * drz) * DEG,
        2 * (vx * dpx + vy * dpy + vz * dpz) * DEG,
        2 * (vx * dyx + vy * dyy) * DEG,
      ]);
    }

    const maxF = Math.max(...f.map(Math.abs));
    finalIter = iter;
    finalErr = maxF;
    if (maxF < 1.0) {
      converged = true;
      lastFKPose = [...pose];
      return { pose, converged, iterations: iter, residual: maxF };
    }

    const dp = gaussSolve(J, f.map((v) => -v));
    if (!dp || dp.some((v) => !Number.isFinite(v))) break;

    const stepLimit = [30, 30, 30, 15, 15, 15];
    for (let j = 0; j < 6; j++) {
      dp[j] = Math.max(-stepLimit[j], Math.min(stepLimit[j], dp[j]));
      pose[j] += dp[j];
    }
    if (pose.some((v) => !Number.isFinite(v))) break;
  }

  if (pose.every((v) => Number.isFinite(v))) lastFKPose = [...pose];
  return { pose: [...lastFKPose], converged, iterations: finalIter, residual: finalErr };
}

const _K = { ik, deltaAngle, solveFK, resetFK, NEUTRAL_Z, LOWER_LEG, UPPER_LEG, MOTOR_PLANE_ANGLE,
             BASE_RADIUS, PLATFORM_RADIUS, BASE_ANGLES, PLATFORM_ANGLES };   // 幾何原語：前端 render 取同源、免重列
if (typeof module !== 'undefined' && module.exports) module.exports = _K;   // Node
if (typeof window !== 'undefined') window.Kin = _K;                          // 瀏覽器 <script src>（SoT 共用）
})();
