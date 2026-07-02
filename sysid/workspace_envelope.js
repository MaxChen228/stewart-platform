'use strict';
// ===== Stewart 工作空間包絡 — 泛用 robust 函數（任意 center r=(x,y,z)、任意幾何構型）=====
//
// 這是 sysid/phone_gen/workspace_viz.js 舊 computeWorkspaceData 的「泛化 + 強化」母函數：
//   舊版：center 寫死 [0,0,133]、幾何寫死 kin.js、safe 只用 Type-I（asin clamp 的 1.25× 徑向裕度）。
//   本版：center / 幾何 / 網格 / 門檻 全參數化；safe 重構為 Type-I 徑向裕度 ∧ Type-II(σmin(Jx)≥ε ∧ κ(Ja)≤κ̄)。
//
// 關鍵：6-RSS 有「雙奇異」。Type-I（串聯）= IK asin clamp，舊版 valid 抓的就是這個。
// Type-II（並聯）= 連桿 Plücker 線相依、載重崩潰，σmin(J_x)→0 —— 舊「safe」完全沒抓 → 本版補上。
//
// 全程在「無量綱座標（R_b≡1，所有長度÷R_b）」內計算 → ratio/角度尺度不變（與 kin.js 位元一致，
// 見自測），且 Type-II 門檻 ε=0.15 / κ̄=12 不隨整體縮放飄移、跨幾何通用。
// 方位（roll/pitch/yaw 度）本就 scale-free，輸出殼仍是「度」，與舊版 live.html 完全相容。
//
// 用法：
//   const WE = require('./workspace_envelope');
//   const Kin = require('./kin.js');
//   const geo = WE.geoFromKin(Kin);                       // 現役幾何（SoT，不分叉）
//   const env = WE.computeEnvelope(geo, { center: [0,0,133/Kin.BASE_RADIUS] });
//   // 任意幾何：WE.geoFromParams({ r, alphaB, alphaP, l, u, beta })

const DEG = Math.PI / 180;

// ---------- 向量基礎 ----------
const dot3 = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const sub3 = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const cross3 = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const nrm3 = a => Math.sqrt(dot3(a, a));
const ap3 = (R, v) => [
  R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
  R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
  R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2],
];
function Rmat(roll, pitch, yaw) {
  const cr=Math.cos(roll*DEG), sr=Math.sin(roll*DEG), cp=Math.cos(pitch*DEG), sp=Math.sin(pitch*DEG), cy=Math.cos(yaw*DEG), sy=Math.sin(yaw*DEG);
  return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
          [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
          [-sp, cp*sr, cp*cr]];
}

// ---------- 對稱 6×6 特徵值（Jacobi）→ 升序 ----------
function eigSym6(Ain) {
  const n = 6, A = Ain.map(r => r.slice());
  for (let s = 0; s < 100; s++) {
    let off = 0; for (let p = 0; p < n; p++) for (let q = p+1; q < n; q++) off += A[p][q]*A[p][q];
    if (off < 1e-22) break;
    for (let p = 0; p < n; p++) for (let q = p+1; q < n; q++) {
      if (Math.abs(A[p][q]) < 1e-18) continue;
      const t0 = (A[q][q]-A[p][p])/(2*A[p][q]), t = Math.sign(t0||1)/(Math.abs(t0)+Math.sqrt(t0*t0+1)), c = 1/Math.sqrt(t*t+1), si = t*c;
      for (let k = 0; k < n; k++) { const akp = A[k][p], akq = A[k][q]; A[k][p] = c*akp-si*akq; A[k][q] = si*akp+c*akq; }
      for (let k = 0; k < n; k++) { const apk = A[p][k], aqk = A[q][k]; A[p][k] = c*apk-si*aqk; A[q][k] = si*apk+c*aqk; }
    }
  }
  return Array.from({length: n}, (_, i) => A[i][i]).sort((a, b) => a-b);
}
// 奇異值（升序）= sqrt(eig(MᵀM))
function singv(M) {
  const G = Array.from({length: 6}, () => Array(6).fill(0));
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) { let s = 0; for (let k = 0; k < 6; k++) s += M[k][i]*M[k][j]; G[i][j] = s; }
  return eigSym6(G).map(v => Math.sqrt(Math.max(0, v)));
}

// ---------- 無量綱運動學核心：geo = { B,P(÷R_b),theta(rad),l,u(÷R_b),Rb } ----------
// IK：逐字搬 kin.js 的 asin 解，但在 R_b≡1 座標（pose 平移用 R_b 單位）。回 {angles(rad),valid,ratios}
function ikN(geo, pose) {
  const [x, y, z, roll, pitch, yaw] = pose, R = Rmat(roll, pitch, yaw);
  const angles = [], ratios = []; let valid = true;
  for (let i = 0; i < 6; i++) {
    const q = ap3(R, geo.P[i]); q[0] += x; q[1] += y; q[2] += z;
    const d = sub3(q, geo.B[i]);
    const L = dot3(d, d) - (geo.u*geo.u - geo.l*geo.l);
    const M = 2*geo.l*d[2];
    const N = 2*geo.l*(Math.cos(geo.theta[i])*d[0] + Math.sin(geo.theta[i])*d[1]);
    const den = Math.sqrt(M*M + N*N), ratio = L/den;
    ratios.push(Math.abs(ratio));
    if (ratio < -1 || ratio > 1) valid = false;
    angles.push(Math.asin(Math.max(-1, Math.min(1, ratio))) - Math.atan2(N, M));   // rad
  }
  return { angles, valid, ratios };
}
// 解析 Jacobian（無量綱）：Jx 列=[ŝ; q×ŝ]，Ja = Jq⁻¹Jx（Jq = L·sinψ = crank⟂rod 扭矩項）
function jacN(geo, pose, angles) {
  const R = Rmat(pose[3], pose[4], pose[5]), T = [pose[0], pose[1], pose[2]], Jx = [], Jq = [];
  for (let i = 0; i < 6; i++) {
    const a = angles[i], th = geo.theta[i], ca = Math.cos(a), sa = Math.sin(a), ct = Math.cos(th), st = Math.sin(th);
    const A = [geo.l*ca*ct + geo.B[i][0], geo.l*ca*st + geo.B[i][1], geo.l*sa + geo.B[i][2]];
    const qr = ap3(R, geo.P[i]), Q = [qr[0]+T[0], qr[1]+T[1], qr[2]+T[2]];
    let s = sub3(Q, A); const ns = nrm3(s) || 1e-9; s = [s[0]/ns, s[1]/ns, s[2]/ns];
    const m = cross3(qr, s);
    Jx.push([s[0], s[1], s[2], m[0], m[1], m[2]]);
    Jq.push(dot3(s, [-geo.l*sa*ct, -geo.l*sa*st, geo.l*ca]));
  }
  return { Jx, Ja: Jx.map((row, i) => row.map(v => v/(Jq[i] || 1e-12))) };
}

// ---------- 一個 pose 的全奇異裕度（無量綱）----------
// 回 { valid(Type-I), radialOk(Type-I 徑向裕度), t2=σmin(Jx), kappa=κ(Ja), minRatioMargin }
function poseMargins(geo, pose, opts = {}) {
  const radialMargin = opts.radialMargin == null ? 1.25 : opts.radialMargin;
  const sol = ikN(geo, pose);
  const out = { valid: sol.valid, radialOk: false, t2: 0, kappa: Infinity, minRatioMargin: 0 };
  if (!sol.valid) return out;
  out.minRatioMargin = 1 - Math.max(...sol.ratios);     // 內生 Type-I 裕度（離 asin clamp 多遠）
  // Type-I 徑向裕度：把方位向量放大 radialMargin 倍仍可達（離邊界有餘裕，方向感知）
  const c = pose.slice(0, 3);
  const rad = ikN(geo, [c[0], c[1], c[2], pose[3]*radialMargin, pose[4]*radialMargin, pose[5]*radialMargin]);
  out.radialOk = rad.valid;
  // Type-II：σmin(Jx)、κ(Ja)
  const J = jacN(geo, pose, sol.angles), sx = singv(J.Jx), sa = singv(J.Ja);
  out.t2 = sx[0];
  out.kappa = sa[5] / (sa[0] || 1e-12);
  // 物理機械約束（傳 opts.phys 才算）：逐腿 J1/J2/J3 + crank 碰撞
  if (opts.phys) {
    const phys = _physOf(opts.phys), thetaLim = solveThetaLim(geo, phys);
    out.phys = { ...legPhysics(geo, pose, sol.angles, phys, thetaLim), thetaLim };
  }
  return out;
}

// ================= 物理機械約束（理想運動學之外，實體機構真實限制）=================
// 預設硬體（mm；a,b,c 為 J2 關節幾何，Rb 為物理底座半徑做無量綱換算）+ 關節行程（度）+ 曲柄碰撞（R_b）。
// J1 = 曲柄仰角(base plate 平面=0、法線朝上=+90)；J2 = 關節內角 ∠(B-A,Q-A) ≥ θ_lim(由式解，隨曲柄長變)；
// J3 = rod 對「平台(動)法線」夾角(外傾為正)；crank = 6 個 crank-tip 兩兩中心距 ≥ crankClear（各半徑 ≈0.2）。
const PHYS_DEFAULT = { Rb: 152, j2a: 39.3, j2b: 12.65, j2c: 5.2, j1Min: -90, j1Max: 90, j3Min: -75, j3Max: 90, crankClear: 0.4 };
const _physOf = (p) => (p === true ? { ...PHYS_DEFAULT } : { ...PHYS_DEFAULT, ...p });

// crank tip A_i（無量綱）。a = IK 曲柄角(rad)
function crankTip(geo, i, a) {
  const th = geo.theta[i];
  return [geo.l*Math.cos(a)*Math.cos(th) + geo.B[i][0], geo.l*Math.cos(a)*Math.sin(th) + geo.B[i][1], geo.l*Math.sin(a) + geo.B[i][2]];
}
// J2 極限角：解 b/tanθ + c/sinθ = l1 - a（全部÷Rb 無量綱）。LHS 對 θ 單調遞減。回 deg；曲柄太短閉不起來→null
function solveThetaLim(geo, phys) {
  const Rb = phys.Rb || 152, aN = phys.j2a/Rb, bN = phys.j2b/Rb, cN = phys.j2c/Rb, target = geo.l - aN;
  if (target <= cN) return null;                       // l1-a ≤ c → 無解（關節閉不起來，幾何不可行）
  let lo = 1e-3, hi = 89.999;
  for (let it = 0; it < 60; it++) { const m = (lo+hi)/2, lhs = bN/Math.tan(m*DEG) + cN/Math.sin(m*DEG); if (lhs > target) lo = m; else hi = m; }
  return (lo+hi)/2;
}
// 一個 pose 的逐腿物理量 + 四閘是否通過。thetaLim 由 solveThetaLim 預算好傳入
function legPhysics(geo, pose, angles, phys, thetaLim) {
  const R = Rmat(pose[3], pose[4], pose[5]), T = [pose[0], pose[1], pose[2]], nUp = [R[0][2], R[1][2], R[2][2]];
  const A = [], j1 = [], j2 = [], j3 = [];
  for (let i = 0; i < 6; i++) {
    const a = angles[i], Ai = crankTip(geo, i, a); A.push(Ai);
    const qr = ap3(R, geo.P[i]), Q = [qr[0]+T[0], qr[1]+T[1], qr[2]+T[2]];
    j1.push(a/DEG);                                                          // J1 仰角
    const BA = sub3(geo.B[i], Ai), QA = sub3(Q, Ai);                         // J2 關節內角
    j2.push(Math.acos(Math.max(-1, Math.min(1, dot3(BA, QA)/((nrm3(BA)||1e-9)*(nrm3(QA)||1e-9)))))/DEG);
    const rod = sub3(Ai, Q), nr = nrm3(rod)||1e-9;                           // J3 rod 對平台法線(朝下)，外傾為正
    let a3 = Math.acos(Math.max(-1, Math.min(1, -dot3(rod, nUp)/nr)))/DEG;
    if (dot3(rod, qr) < 0) a3 = -a3;                                         // rod 與外徑向反 → 內傾為負
    j3.push(a3);
  }
  let crankMin = Infinity;
  for (let i = 0; i < 6; i++) for (let k = i+1; k < 6; k++) { const d = nrm3(sub3(A[i], A[k])); if (d < crankMin) crankMin = d; }
  const okJ1 = j1.every(v => v >= phys.j1Min && v <= phys.j1Max);
  const okJ2 = thetaLim != null && j2.every(v => v >= thetaLim);
  const okJ3 = j3.every(v => v >= phys.j3Min && v <= phys.j3Max);
  const okCrank = crankMin >= phys.crankClear;
  return { j1, j2, j3, crankMin, okJ1, okJ2, okJ3, okCrank, ok: okJ1 && okJ2 && okJ3 && okCrank };
}

// ---------- 幾何 provider ----------
const SPOKES = [210, 90, -30], SIGN = [1, -1, 1, -1, 1, -1];
// 任意幾何（無量綱參數，與 web/geometry.html 同義）：r=R_p/R_b, alphaB/alphaP(度), l=L/R_b, u=U/R_b, beta(度), clocking(度)
function geoFromParams(p) {
  const clocking = p.clocking || 0, B = [], P = [], theta = [];
  for (let i = 0; i < 6; i++) {
    const spoke = SPOKES[(i/2)|0];
    const ba = (spoke + SIGN[i]*p.alphaB/2)*DEG, pa = (spoke + clocking + SIGN[i]*p.alphaP/2)*DEG;
    B.push([Math.cos(ba), Math.sin(ba), 0]);
    P.push([p.r*Math.cos(pa), p.r*Math.sin(pa), 0]);
    theta.push((spoke + SIGN[i]*(90 - (p.beta || 0)))*DEG);
  }
  return { B, P, theta, l: p.l, u: p.u, Rb: 1 };
}
// 現役幾何：讀 kin.js 暴露的 mm 常數 ÷ R_b（不分叉 ik、不另立幾何源）
function geoFromKin(Kin) {
  const Rb = Kin.BASE_RADIUS, B = [], P = [], theta = [];
  for (let i = 0; i < 6; i++) {
    const ba = Kin.BASE_ANGLES[i]*DEG, pa = Kin.PLATFORM_ANGLES[i]*DEG;
    B.push([Math.cos(ba), Math.sin(ba), 0]);
    P.push([(Kin.PLATFORM_RADIUS/Rb)*Math.cos(pa), (Kin.PLATFORM_RADIUS/Rb)*Math.sin(pa), 0]);
    theta.push(Kin.MOTOR_PLANE_ANGLE[i]*DEG);
  }
  return { B, P, theta, l: Kin.LOWER_LEG/Rb, u: Kin.UPPER_LEG/Rb, Rb };
}

// ---------- 軸工具 ----------
const ax = (a) => { const o = []; for (let v = a[0]; v <= a[1] + 1e-9; v += a[2]) o.push(+v.toFixed(2)); return o; };
const samp = (a, n) => { if (a.length <= n) return a; const o = [], s = a.length/n; for (let t = 0; t < n; t++) o.push(a[Math.floor(t*s)]); return o; };

// ================= 母函數：在 center 周圍掃方位，分類 工作空間 / 安全空間 =================
// geo    : geoFromKin / geoFromParams 的輸出（無量綱）
// center : [x,y,z]（R_b 單位）平台中心。預設 [0,0,NEUTRAL_Z/Rb]≈現役 home
// grid   : { roll:[lo,hi,step], pitch:[...], yaw:[...] }（度）
// safe 門檻：radialMargin(Type-I 徑向)、epsT2(σmin(Jx) 下限)、kappaThr(κ(Ja) 上限)
// 回 { workspace, safe, lim, center, grid, stats }；殼點 = [roll,pitch,yaw]（度，scale-free）
function computeEnvelope(geo, opts = {}) {
  const center = opts.center || [0, 0, 105/(geo.Rb || 1)];
  const grid = opts.grid || { roll: [-42, 42, 1.5], pitch: [-42, 42, 1.5], yaw: [-92, 92, 2.5] };
  const radialMargin = opts.radialMargin == null ? 1.25 : opts.radialMargin;
  const epsT2 = opts.epsT2 == null ? 0.15 : opts.epsT2;
  const kappaThr = opts.kappaThr == null ? 12 : opts.kappaThr;
  const sampWS = opts.sampleWorkspace || 4500, sampSafe = opts.sampleSafe || 2800;
  const phys = opts.phys ? _physOf(opts.phys) : null;          // 物理機械約束（不傳=純運動學，live.html 退化版不變）
  const thetaLim = phys ? solveThetaLim(geo, phys) : null;

  const rs = ax(grid.roll), ps = ax(grid.pitch), ys = ax(grid.yaw);
  const key = (i, j, k) => i + ',' + j + ',' + k;
  const valid = new Set(), physical = new Set(), safe = new Set();
  let nValid = 0, cutRadial = 0, cutT2 = 0, cutKappa = 0, cutCrank = 0, cutJ1 = 0, cutJ2 = 0, cutJ3 = 0;

  for (let i = 0; i < rs.length; i++) for (let j = 0; j < ps.length; j++) for (let k = 0; k < ys.length; k++) {
    const pose = [center[0], center[1], center[2], rs[i], ps[j], ys[k]];
    const sol = ikN(geo, pose);
    if (!sol.valid) continue;                       // Type-I 不可達 → 不在工作空間
    valid.add(key(i, j, k)); nValid++;
    // 物理可達：四閘（曲柄不撞 ∧ J1 ∧ J2 ∧ J3）。不通過 → 不在物理可達、亦不可能 safe
    if (phys) {
      const lp = legPhysics(geo, pose, sol.angles, phys, thetaLim);
      if (lp.ok) physical.add(key(i, j, k));
      else { if (!lp.okCrank) cutCrank++; else if (!lp.okJ1) cutJ1++; else if (!lp.okJ2) cutJ2++; else cutJ3++; continue; }
    }
    // 安全：先 Type-I 徑向裕度（便宜）→ 再 Type-II（貴，只對通過者算）
    const rad = ikN(geo, [center[0], center[1], center[2], rs[i]*radialMargin, ps[j]*radialMargin, ys[k]*radialMargin]);
    if (!rad.valid) { cutRadial++; continue; }
    const J = jacN(geo, pose, sol.angles), sx = singv(J.Jx), sa = singv(J.Ja);
    if (sx[0] < epsT2) { cutT2++; continue; }        // 太靠 Type-II（並聯奇異）
    if (sa[5] / (sa[0] || 1e-12) > kappaThr) { cutKappa++; continue; }  // 條件數爛（病態）
    safe.add(key(i, j, k));
  }

  // 邊界殼（任一 6-鄰居不在集合內 = 邊界體素）
  function shell(set) {
    const out = [];
    for (const kk of set) {
      const [i, j, k] = kk.split(',').map(Number);
      const nb = [[i-1,j,k],[i+1,j,k],[i,j-1,k],[i,j+1,k],[i,j,k-1],[i,j,k+1]];
      if (nb.some(([a, b, c]) => !set.has(key(a, b, c)))) out.push([rs[i], ps[j], ys[k]]);
    }
    return out;
  }
  const voxVol = grid.roll[2]*grid.pitch[2]*grid.yaw[2];
  // 單軸極限（沿純 roll/pitch/yaw 掃到 Type-I 邊界）
  const axMax = (axis, hi = 92) => {
    let m = 0, mn = 0;
    for (let v = 0; v <= hi; v += 0.5) { const a = [0,0,0]; a[axis] = v; if (ikN(geo, [center[0],center[1],center[2],a[0],a[1],a[2]]).valid) m = v; else break; }
    for (let v = 0; v >= -hi; v -= 0.5) { const a = [0,0,0]; a[axis] = v; if (ikN(geo, [center[0],center[1],center[2],a[0],a[1],a[2]]).valid) mn = v; else break; }
    return [mn, m];
  };
  const rollAt = (y) => { let m = 0; for (let v = 0; v <= 60; v += 0.5) { if (ikN(geo, [center[0],center[1],center[2],v,0,y]).valid) m = v; else break; } return m; };

  const stats = {
    rollLim: axMax(0), pitchLim: axMax(1), yawLim: axMax(2),
    coupling: { y0: rollAt(0), y30: rollAt(30), y50: rollAt(50), y70: rollAt(70) },
    wsVol: Math.round(valid.size*voxVol), physVol: Math.round(physical.size*voxVol), safeVol: Math.round(safe.size*voxVol),
    safeFrac: valid.size ? +(safe.size/valid.size).toFixed(3) : 0,
    physFrac: phys && valid.size ? +(physical.size/valid.size).toFixed(3) : 0,
    nValid, nPhys: physical.size, nSafe: safe.size,
    // safe 被各機制砍掉多少（透明化）；物理閘細項僅 phys 時有
    cuts: { radial: cutRadial, typeII: cutT2, kappa: cutKappa, crank: cutCrank, j1: cutJ1, j2: cutJ2, j3: cutJ3 },
    thetaLim, thresholds: { radialMargin, epsT2, kappaThr, phys },
  };
  return {
    workspace: samp(shell(valid), sampWS),
    physical: phys ? samp(shell(physical), sampWS) : [],
    safe: samp(shell(safe), sampSafe),
    lim: { r: grid.roll, p: grid.pitch, y: grid.yaw },
    center, grid, stats,
  };
}

const _M = { DEG, Rmat, eigSym6, singv, ikN, jacN, poseMargins, geoFromParams, geoFromKin, computeEnvelope, ax, samp,
  PHYS_DEFAULT, crankTip, solveThetaLim, legPhysics };
if (typeof module !== 'undefined' && module.exports) module.exports = _M;
if (typeof self !== 'undefined') self.WorkspaceEnvelope = _M;   // 瀏覽器 window 或 Web Worker（self 兩者皆涵蓋）
