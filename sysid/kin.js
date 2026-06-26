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

const _K = { ik, deltaAngle, NEUTRAL_Z, LOWER_LEG, UPPER_LEG, MOTOR_PLANE_ANGLE,
             BASE_RADIUS, PLATFORM_RADIUS, BASE_ANGLES, PLATFORM_ANGLES };   // 幾何原語：前端 render 取同源、免重列
if (typeof module !== 'undefined' && module.exports) module.exports = _K;   // Node
if (typeof window !== 'undefined') window.Kin = _K;                          // 瀏覽器 <script src>（SoT 共用）
})();
