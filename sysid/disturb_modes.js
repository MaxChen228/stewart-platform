'use strict';
// 擾動模態數學（純，無副作用）：task-space 方向 → 六軸 enc 慣例 Δangle，severity 正規化。
// SoT：disturb.js（單模態 runner）、disturb_battery.js（壓力測試序列）共用此一份；
//      未來前端 UI 化也載入同一邏輯（kin.js 已可雙載），不重寫第二真相源。
const { deltaAngle, NEUTRAL_Z } = (typeof require !== 'undefined')
  ? require('./kin.js')                                    // Node
  : window.Kin;                                            // 瀏覽器（需先載 kin.js）

const MOTOR_SIGN = [1, -1, 1, -1, 1, -1];
const POSE0 = [0, 0, NEUTRAL_Z, 0, 0, 0];   // 工作點（中位姿態）

// task-space 單位方向 [x,y,z,roll,pitch,yaw]（mm/度混用，正規化後消量綱）
const DIRS = {
  roll:  [0, 0, 0, 1, 0, 0],
  pitch: [0, 0, 0, 0, 1, 0],
  tilt:  [0, 0, 0, 0.7071, 0.7071, 0],   // 對角傾＝任意水平方向傾倒
  surge: [1, 0, 0, 0, 0, 0],             // +x 側推
  sway:  [0, 1, 0, 0, 0, 0],             // +y 側推
  heave: [0, 0, 1, 0, 0, 0],             // +z 升降（六軸近同號）
  yaw:   [0, 0, 0, 0, 0, 1],             // 扭轉（六軸奇偶交替）
  mixed: [0.3, 0.3, 0, 1, 1, 1],         // 複合：平移+三轉，最嚴苛
};
const rms = (a) => Math.sqrt(a.reduce((s, x) => s + x * x, 0) / a.length);

// 該模態在 severity 下的「enc 慣例六軸 Δangle」(度)。single 模態(m1..m6) 直接單軸。
// severity 正規化：task-space 模態縮放成「六軸 enc-Δangle 的 RMS = severity」→ 跨模態強度可比。
function encDelta(mode, severity) {
  const m = /^m([1-6])$/.exec(mode);
  if (m) { const d = [0, 0, 0, 0, 0, 0]; d[+m[1] - 1] = severity; return d; }
  const dir = DIRS[mode];
  if (!dir) throw new Error(`未知模態 ${mode}；可用: ${Object.keys(DIRS).join(',')},m1..m6`);
  const eps = 0.5;                                  // 小 probe 取線性比例（ik 非線性，eps 小才準）
  const p = deltaAngle(POSE0, dir.map(v => v * eps));
  const enc = p.d.map((v, i) => v * MOTOR_SIGN[i]); // IK 慣例 → enc 慣例
  const ur = rms(enc) || 1e-9;
  return enc.map(v => v * severity / ur);            // 線性縮放使最終 rms = severity
}

const ALL_MODES = [...Object.keys(DIRS), 'm1', 'm2', 'm3', 'm4', 'm5', 'm6'];

const _DM = { encDelta, ALL_MODES, DIRS, MOTOR_SIGN, POSE0, rms };
if (typeof module !== 'undefined' && module.exports) module.exports = _DM;   // Node
if (typeof window !== 'undefined') window.DisturbModes = _DM;                 // 瀏覽器（需先載 kin.js）
