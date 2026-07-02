'use strict';
// Stewart 6-RSS 幾何最佳化(無量綱):雙奇異 + 條件數 + 多起點 Nelder-Mead。
// 第一性原理:R_b=1 鎖尺度,只搜 8 個無量綱形狀變數 {r,α_b,α_p,L,U,h,β,γ}。
// 目標 = 平滑 margin 代理(避開 binary-valid 不可微);順帶示範 FD 梯度為何在此失效。
// 跑法: node sysid/geometry_optimize.js [sanity|asbuilt|scan|grad|opt|all]

const DEG = Math.PI / 180;
const WE = require('./workspace_envelope');   // 幾何/IK/Jacobian/奇異值 SoT 同源,不重寫本地版

// 運動學委派 WE(消除逐字搬的 geometry/Rmat/ik/jacobians/eigSym6/singvals6)。
// 參數映射:optimize 命名 {ab,ap,L,U,gamma} → WE.geoFromParams 命名 {alphaB,alphaP,l,u,clocking}。
// ik/jacobians 保留 (p,geo,...) 簽名(p 忽略,l/u 已在 geo)→ 呼叫點不動。
const geometry = (p) => WE.geoFromParams({ r: p.r, alphaB: p.ab, alphaP: p.ap, l: p.L, u: p.U, beta: p.beta, clocking: p.gamma });
const ik = (p, geo, pose) => WE.ikN(geo, pose);
const jacobians = (p, geo, pose, angles) => WE.jacN(geo, pose, angles);
const singvals6 = WE.singv;

// ---------- 參考工作空間 W(無量綱、任務無關:home 周圍對稱球)----------
// Halton 低差異序列(可重現,免 Math.random)
function halton(idx, base) { let f = 1, r = 0, i = idx; while (i > 0) { f /= base; r += f*(i % base); i = (i/base)|0; } return r; }
function makeW(p, n = 160, dT = 0.10, dA = 12) {  // 平移 ±0.10·R_b、轉動 ±12°
  const bases = [2, 3, 5, 7, 11, 13], W = [];
  for (let k = 1; k <= n; k++) {
    const u = bases.map(b => 2*halton(k, b) - 1);   // [-1,1]^6
    W.push([u[0]*dT, u[1]*dT, p.h + u[2]*dT, u[3]*dA, u[4]*dA, u[5]*dA]);
  }
  return W;
}

// ---------- 度量 ----------
const EPS_T2 = 0.15;   // Type-II 安全門檻(σ_min(J_x))
function metrics(p) {
  const geo = geometry(p), W = makeW(p);
  let reach = 0, sumInvK = 0, worstSig = Infinity, worstT2 = Infinity, sumMargin = 0, nK = 0;
  for (const pose of W) {
    const sol = ik(p, geo, pose);
    if (!sol.valid) continue;                        // Type-I 不可達
    reach++;
    const J = jacobians(p, geo, pose, sol.angles);
    const svJa = singvals6(J.Ja), svJx = singvals6(J.Jx);
    const sMin = svJa[0], sMax = svJa[5], kappa = sMax / (sMin || 1e-12);
    const t2 = svJx[0];
    sumInvK += 1/kappa; nK++;
    worstSig = Math.min(worstSig, sMin);
    worstT2 = Math.min(worstT2, t2);
    sumMargin += Math.max(0, t2 - EPS_T2);           // 平滑 margin 代理(可微地形)
  }
  const N = W.length;
  return {
    reachFrac: reach / N,
    GCI: nK ? sumInvK / nK : 0,
    worstSig: worstSig === Infinity ? 0 : worstSig,
    worstT2: worstT2 === Infinity ? 0 : worstT2,
    smoothMargin: sumMargin / N,                     // ← 最佳化用(連續)
  };
}
// 純量目標(最大化):平滑 margin × 各向同性,Type-II 不安全則重罰
function objective(p) {
  const m = metrics(p);
  const safety = m.worstT2 < EPS_T2 ? (m.worstT2 / EPS_T2) : 1;   // 軟障壁
  return m.smoothMargin * (0.5 + 0.5*m.GCI) * safety;
}

// ===== Scale-free 目標:角度 dexterous 工作空間 =====
// 角度不隨整體縮放改變 → 不可被「放大機構」game。
// θ* = 最大傾角半徑,使「該半徑球面上所有姿態」皆 reachable ∧ Type-II 安全 ∧ κ<門檻。
const KAPPA_THR = 12;          // 良態門檻(rotary Stewart κ 偏高,12 為寬鬆良態)
function metricsAng(p) {
  const geo = geometry(p), bases = [2, 3, 5];
  const Ndir = 40, levels = [4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30];
  let thetaStar = 0, invKsum = 0, invKn = 0, worstT2 = Infinity;
  for (const th of levels) {
    let allOK = true;
    for (let k = 1; k <= Ndir; k++) {
      // 在 (roll,pitch,yaw) 單位球上取方向(Halton)→ 乘半徑 th
      let d = [2*halton(k,2)-1, 2*halton(k,3)-1, 2*halton(k,5)-1];
      const dn = Math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]) || 1; d = d.map(v => v/dn);
      const pose = [0, 0, p.h, d[0]*th, d[1]*th, d[2]*th];
      const sol = ik(p, geo, pose);
      if (!sol.valid) { allOK = false; break; }
      const J = jacobians(p, geo, pose, sol.angles);
      const svJa = singvals6(J.Ja), svJx = singvals6(J.Jx);
      const kappa = svJa[5] / (svJa[0] || 1e-12);
      worstT2 = Math.min(worstT2, svJx[0]);
      if (svJx[0] < EPS_T2 || kappa > KAPPA_THR) { allOK = false; break; }
      invKsum += 1/kappa; invKn++;
    }
    if (allOK) thetaStar = th; else break;
  }
  return { thetaStar, meanInvK: invKn ? invKsum/invKn : 0, worstT2: worstT2 === Infinity ? 0 : worstT2 };
}
// 角度工作空間(度)× 各向同性 → scale-free,不可被尺度 game
function objectiveAng(p) {
  const m = metricsAng(p);
  return m.thetaStar * (0.5 + 0.5*m.meanInvK);
}

// ===== 製造約束(從 as-built 反推:pair-chord ≈ 50mm = 接頭/馬達硬體寬)=====
const CMIN = 0.3287;   // = 49.96mm / 152mm,base 與 platform 同弦長
function pairChords(p) {
  return { base: 2*Math.sin(p.ab/2*DEG), plat: 2*p.r*Math.sin(p.ap/2*DEG) };  // R_b=1
}
function mfgPenalty(p) {
  const c = pairChords(p); let pen = 0;
  if (c.base < CMIN) pen += (CMIN - c.base) * 100;   // 接頭互撞 → 不可製造
  if (c.plat < CMIN) pen += (CMIN - c.plat) * 100;
  return pen;
}
function objectiveAngMfg(p) { return objectiveAng(p) - mfgPenalty(p); }

// 只對 freeKeys 子集做 Nelder-Mead,其餘鎖在 base(asbuilt)
function optimizeSubset(freeKeys, baseP, objFn, { iters = 250 } = {}) {
  const x0 = freeKeys.map(k => baseP[k]);
  const expand = (xr) => { const p = { ...baseP }; freeKeys.forEach((k, i) => p[k] = xr[i]); return clampP(p); };
  const f = (xr) => objFn(expand(xr));
  // 暫時把 NM 的步長按 freeKeys 範圍縮放
  const n = freeKeys.length;
  let simplex = [x0.slice()];
  for (let i = 0; i < n; i++) { const x = x0.slice(); x[i] += 0.12 * (UB[freeKeys[i]] - LB[freeKeys[i]]); simplex.push(x); }
  let fv = simplex.map(f);
  const order = () => { const idx = fv.map((v, i) => i).sort((a, b) => fv[b] - fv[a]); simplex = idx.map(i => simplex[i]); fv = idx.map(i => fv[i]); };
  for (let it = 0; it < iters; it++) {
    order();
    const best = simplex[0], worst = simplex[n], cen = Array(n).fill(0);
    for (let i = 0; i < n; i++) simplex[i].forEach((v, j) => cen[j] += v/n);
    const refl = cen.map((c, j) => c + (c - worst[j])), fr = f(refl);
    if (fr > fv[0]) { const exp = cen.map((c, j) => c + 2*(c - worst[j])), fe = f(exp); if (fe > fr) { simplex[n]=exp; fv[n]=fe; } else { simplex[n]=refl; fv[n]=fr; } }
    else if (fr > fv[n-1]) { simplex[n]=refl; fv[n]=fr; }
    else { const con = cen.map((c, j) => c + 0.5*(worst[j]-c)), fc = f(con);
      if (fc > fv[n]) { simplex[n]=con; fv[n]=fc; }
      else { for (let i=1;i<=n;i++){ simplex[i]=simplex[i].map((v,j)=>best[j]+0.5*(v-best[j])); fv[i]=f(simplex[i]); } } }
  }
  order();
  return { p: expand(simplex[0]), f: fv[0] };
}

const RB_MM = 152;   // 維持底座半徑不變做 mm 還原
function showMfg(tag, p, cost) {
  const m = metricsAng(p), c = pairChords(p);
  const mm = `R_b=${RB_MM} R_p=${(p.r*RB_MM).toFixed(0)} α_b=${p.ab.toFixed(1)}° α_p=${p.ap.toFixed(1)}° L=${(p.L*RB_MM).toFixed(0)} U=${(p.U*RB_MM).toFixed(0)} h=${(p.h*RB_MM).toFixed(0)} β=${p.beta.toFixed(1)}° γ=${p.gamma.toFixed(1)}°`;
  console.log(`\n[${tag}] (改動成本: ${cost})`);
  console.log('  mm: ' + mm);
  console.log(`  θ*=${m.thetaStar}°  meanInvK=${m.meanInvK.toFixed(3)}  Type-II裕度=${m.worstT2.toFixed(3)}  pair-chord(base/plat)=${(c.base*RB_MM).toFixed(0)}/${(c.plat*RB_MM).toFixed(0)}mm`);
  return m;
}

// ---------- 變數打包 ----------
const KEYS = ['r', 'ab', 'ap', 'L', 'U', 'h', 'beta', 'gamma'];
const LB = { r:0.30, ab:2, ap:2, L:0.20, U:0.60, h:0.30, beta:0, gamma:-60 };
const UB = { r:1.00, ab:50, ap:50, L:0.80, U:1.60, h:1.00, beta:80, gamma:60 };
const ASBUILT = { r:103/152, ab:18.92, ap:28.07, L:65/152, U:165/152, h:105/152, beta:0, gamma:0 };
const clampP = (p) => { const q = {}; for (const k of KEYS) q[k] = Math.max(LB[k], Math.min(UB[k], p[k])); return q; };
const toVec = (p) => KEYS.map(k => p[k]);
const toP = (v) => { const p = {}; KEYS.forEach((k, i) => p[k] = v[i]); return p; };

// ---------- Nelder–Mead(8D,含邊界 clamp)----------
function nelderMead(f, x0, { iters = 400, step = 0.12 } = {}) {
  const n = x0.length;
  let simplex = [x0.slice()];
  for (let i = 0; i < n; i++) { const x = x0.slice(); x[i] += step * (UB[KEYS[i]] - LB[KEYS[i]]); simplex.push(x); }
  let fv = simplex.map(f);
  const order = () => { const idx = fv.map((v, i) => i).sort((a, b) => fv[b] - fv[a]); simplex = idx.map(i => simplex[i]); fv = idx.map(i => fv[i]); };
  for (let it = 0; it < iters; it++) {
    order();
    const best = simplex[0], worst = simplex[n], cen = Array(n).fill(0);
    for (let i = 0; i < n; i++) simplex[i].forEach((v, j) => cen[j] += v/n);   // 不含 worst
    const reflect = cen.map((c, j) => c + 1.0*(c - worst[j])), fr = f(reflect);
    if (fr > fv[0]) { const exp = cen.map((c, j) => c + 2.0*(c - worst[j])), fe = f(exp);
      if (fe > fr) { simplex[n] = exp; fv[n] = fe; } else { simplex[n] = reflect; fv[n] = fr; } }
    else if (fr > fv[n-1]) { simplex[n] = reflect; fv[n] = fr; }
    else { const con = cen.map((c, j) => c + 0.5*(worst[j] - c)), fc = f(con);
      if (fc > fv[n]) { simplex[n] = con; fv[n] = fc; }
      else { for (let i = 1; i <= n; i++) { simplex[i] = simplex[i].map((v, j) => best[j] + 0.5*(v - best[j])); fv[i] = f(simplex[i]); } } }
  }
  order();
  return { x: simplex[0], f: fv[0] };
}

// ---------- 報表 ----------
const fmtP = (p) => KEYS.map(k => `${k}=${p[k].toFixed(k==='ab'||k==='ap'||k==='beta'||k==='gamma'?2:4)}`).join('  ');
function show(tag, p) {
  const m = metrics(p);
  console.log(`\n[${tag}]`);
  console.log('  ' + fmtP(p));
  console.log(`  reachFrac=${m.reachFrac.toFixed(3)}  GCI=${m.GCI.toFixed(4)}  worstσmin(Ja)=${m.worstSig.toFixed(4)}  worstσmin(Jx/T2)=${m.worstT2.toFixed(4)}  obj=${objective(p).toFixed(5)}`);
  return m;
}

// ---------- 主程式 ----------
const mode = process.argv[2] || 'all';
const wrapObj = (v) => objective(clampP(toP(v)));
const wrapAng = (v) => objectiveAng(clampP(toP(v)));

if (mode === 'sanity' || mode === 'all') {
  // 自檢 1:正六邊形對齊(r=1,α_b=α_p,γ=0)→ Type-II 近奇異 σ_min(J_x)→0
  const reg = { r:1, ab:20, ap:20, L:0.43, U:1.05, h:0.69, beta:0, gamma:0 };
  const geo = geometry(reg);
  // 在 home 取 σ_min(J_x)
  const sol = ik(reg, geo, [0,0,reg.h,0,0,0]);
  const t2reg = sol.valid ? singvals6(jacobians(reg, geo, [0,0,reg.h,0,0,0], sol.angles).Jx)[0] : NaN;
  // 對照:錯開 γ=30
  const off = { ...reg, gamma: 30 }, geo2 = geometry(off), s2 = ik(off, geo2, [0,0,off.h,0,0,0]);
  const t2off = s2.valid ? singvals6(jacobians(off, geo2, [0,0,off.h,0,0,0], s2.angles).Jx)[0] : NaN;
  console.log('[sanity] 正六邊形對齊(γ=0) σ_min(Jx)=' + t2reg.toFixed(4) + '  vs 錯開(γ=30) σ_min(Jx)=' + t2off.toFixed(4));
  console.log('         預期:對齊明顯較小(更靠 Type-II)。' + (t2reg < t2off ? ' ✓ 通過' : ' ✗ 檢查'));
}

if (mode === 'asbuilt' || mode === 'all') {
  show('AS-BUILT (你現在的幾何)', ASBUILT);
  const ma = metricsAng(ASBUILT);
  console.log(`  [角度] θ*=${ma.thetaStar}°  meanInvK=${ma.meanInvK.toFixed(4)}  worstT2=${ma.worstT2.toFixed(4)}  objAng=${objectiveAng(ASBUILT).toFixed(3)}`);
}

if (mode === 'optang' || mode === 'all') {
  // Scale-free 最佳化:角度 dexterous 工作空間
  const bases = [2,3,5,7,11,13,17,19];
  const starts = [toVec(ASBUILT)];
  const scored = [];
  for (let k = 1; k <= 300; k++) { const v = KEYS.map((key,i)=>LB[key]+(UB[key]-LB[key])*halton(k,bases[i])); scored.push({v,f:wrapAng(v)}); }
  scored.sort((a,b)=>b.f-a.f);
  for (let i = 0; i < 4; i++) starts.push(scored[i].v);
  let best = null;
  starts.forEach((x0)=>{ const r = nelderMead(wrapAng, x0, { iters: 250, step: 0.10 }); if (!best || r.f > best.f) best = r; });
  const bp = clampP(toP(best.x)), m = metricsAng(bp);
  console.log('\n[OPTIMUM-ANG (scale-free 角度工作空間)]');
  console.log('  ' + fmtP(bp));
  console.log(`  θ*=${m.thetaStar}°  meanInvK=${m.meanInvK.toFixed(4)}  worstT2=${m.worstT2.toFixed(4)}  objAng=${best.f.toFixed(3)}`);
}

if (mode === 'grad' || mode === 'all') {
  // 示範:FD 梯度在 binary-valid 目標上失效 vs 平滑代理可用
  const reachObj = (p) => metrics(p).reachFrac;        // 階梯 → 梯度多半=0
  const base = ASBUILT, eps = 1e-3;
  console.log('\n[grad] FD 梯度比較(±1e-3 微擾):');
  for (const k of KEYS) {
    const pp = { ...base, [k]: base[k]+eps }, pm = { ...base, [k]: base[k]-eps };
    const gReach = (reachObj(pp) - reachObj(pm)) / (2*eps);
    const gSmooth = (objective(pp) - objective(pm)) / (2*eps);
    console.log(`  ∂/∂${k.padEnd(6)}  reachFrac=${gReach.toFixed(4).padStart(9)}   平滑obj=${gSmooth.toFixed(4).padStart(9)}`);
  }
  console.log('  → reachFrac 梯度大量為 0/跳變(不可微);平滑 margin 代理才有穩定方向。');
}

if (mode === 'mfg' || mode === 'all') {
  console.log('\n========== 可落地最佳化(製造約束 pair-chord ≥ 50mm)==========');
  showMfg('AS-BUILT 基準', ASBUILT, '無');
  // A) 只重新 clock 平台(最便宜:重鑽平台孔位,不動馬達/接頭)
  const A = optimizeSubset(['gamma'], ASBUILT, objectiveAngMfg);
  showMfg('A 只重新 clock(只動 γ)', A.p, '重鑽平台孔位');
  // B) 不換硬體:α 鎖死(接頭間距固定),retune 半徑/腿長/高度/clock/曲柄偏轉
  const B = optimizeSubset(['r','L','U','h','gamma','beta'], ASBUILT, objectiveAngMfg, { iters: 300 });
  showMfg('B 不換馬達/接頭(α 鎖死)', B.p, '換平台盤+曲柄+連桿+標準件');
  // C) 全重製:全自由,僅守 pair-chord≥50mm
  let bestC = null;
  const bases = [2,3,5,7,11,13,17,19];
  const starts = [toVec(ASBUILT)];
  const sc = [];
  for (let k=1;k<=300;k++){ const v=KEYS.map((key,i)=>LB[key]+(UB[key]-LB[key])*halton(k,bases[i])); sc.push({v,f:objectiveAngMfg(clampP(toP(v)))}); }
  sc.sort((a,b)=>b.f-a.f); for (let i=0;i<4;i++) starts.push(sc[i].v);
  starts.forEach(x0=>{ const r=optimizeSubset(KEYS, toP(x0), objectiveAngMfg, { iters: 250 }); if(!bestC||r.f>bestC.f) bestC=r; });
  showMfg('C 全重製(守 pair-chord≥50mm)', bestC.p, '整台重做');
}

if (mode === 'scan' || mode === 'opt') {
  // 多起點:as-built + LHS 取樣的前幾名
  const bases = [2,3,5,7,11,13,17,19];
  const starts = [toVec(ASBUILT)];
  const scored = [];
  for (let k = 1; k <= 400; k++) {
    const v = KEYS.map((key, i) => LB[key] + (UB[key]-LB[key]) * halton(k, bases[i]));
    scored.push({ v, f: wrapObj(v) });
  }
  scored.sort((a, b) => b.f - a.f);
  for (let i = 0; i < 4; i++) starts.push(scored[i].v);

  if (mode === 'scan') { console.log('\n[scan] LHS 前 5 名:'); scored.slice(0,5).forEach((s,i)=>show('scan#'+i, clampP(toP(s.v)))); }

  if (mode === 'opt' || mode === 'all') {
    let best = null;
    starts.forEach((x0, i) => {
      const res = nelderMead(wrapObj, x0, { iters: 350, step: 0.10 });
      if (!best || res.f > best.f) best = { ...res, start: i };
    });
    const bp = clampP(toP(best.x));
    show('OPTIMUM (Nelder-Mead 多起點)', bp);
    // Pareto:沿 reachFrac↔GCI,掃權重看取捨
    console.log('\n[pareto] 目標權重掃描(margin 廣度 vs GCI 各向同性):');
    for (const w of [0, 0.25, 0.5, 0.75, 1.0]) {
      const obj = (v) => { const p = clampP(toP(v)); const m = metrics(p);
        const safety = m.worstT2 < EPS_T2 ? m.worstT2/EPS_T2 : 1;
        return (w*m.GCI + (1-w)*m.smoothMargin*4) * safety; };
      const r = nelderMead(obj, toVec(ASBUILT), { iters: 200, step: 0.10 });
      const p = clampP(toP(r.x)), m = metrics(p);
      console.log(`  w_GCI=${w.toFixed(2)}  →  reachFrac=${m.reachFrac.toFixed(3)} GCI=${m.GCI.toFixed(4)} T2=${m.worstT2.toFixed(3)} | ${fmtP(p)}`);
    }
  }
}
