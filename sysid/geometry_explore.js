'use strict';
// Stewart 6-RSS 幾何深度探索 — 與 web/geometry.html 數學「逐字一致」。
// 核心區別於舊 geometry_optimize.js:home(z*,γ*) 不是自由變數,而是「由幾何算出」
// (C3 閉式定 x=y=roll=pitch=0;(z*,γ*)=argmax 各方向安全可達傾角平均)。θ* 在算出的 home 上評。
// 目標:在「7 幾何 DOF {r,αb,αp,l,u,β}(R_b≡1)」空間找 高活動度(θ*) + Type-II 穩定(σmin(Jx)/κ)。
// 產出多樣化候選(每類一個最佳)→ 直接貼進 geometry.html 成可點選方案。
// 跑法: node sysid/geometry_explore.js

const DEG = Math.PI/180;
const EPS_T2 = 0.15, KAPPA_THR = 12;
const DELTA_JOINT = 0.329;     // pair-chord 製造下限 = 50mm/152
const WE = require('./workspace_envelope');   // 幾何/IK/Jacobian/奇異值/物理約束 SoT 同源,不重寫

const PHYS = WE.PHYS_DEFAULT;                 // {Rb,a/b/c(J2),j1/j3 行程,crankClear}

// 向量(方向 normalize 用)+ 運動學委派 WE：消除 geom/Rmat/ik/jac/eigSym6/singv 逐字搬的重複。
// geom 保留薄包裝(轉接 clocking 參數);ik/jac/singv 直接委派 WE 的無量綱版(位元一致)。
const dot3=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2], nrm3=a=>Math.sqrt(dot3(a,a));
const geom=(p,clocking)=>WE.geoFromParams({...p,clocking});
const ik=WE.ikN, jac=WE.jacN, singv=WE.singv;

// ---- home(幾何):逐字搬頁面 computeHome ----
const _hal=(idx,b)=>{let f=1,r=0,i=idx;while(i>0){f/=b;r+=f*(i%b);i=(i/b)|0;}return r;};
function evalHome(p,z,yaw){
  const g=geom(p,yaw), sol=ik(g,[0,0,z,0,0,0]); if(!sol.valid) return null;
  const J=jac(g,[0,0,z,0,0,0],sol.angles), sx=singv(J.Jx), sa=singv(J.Ja);
  return { t2:sx[0], kappa:sa[5]/(sa[0]||1e-12) };
}
function meanReachAt(p,z,yaw){
  const g=geom(p,yaw), thetaLim=WE.solveThetaLim(g,PHYS); let sum=0; const N=12;
  for(let k=1;k<=N;k++){
    let d=[2*_hal(k,2)-1,2*_hal(k,3)-1,2*_hal(k,5)-1]; const dn=nrm3(d)||1; d=d.map(v=>v/dn);
    let last=0;
    for(let r=3;r<=30;r+=3){ const ps=[0,0,z,d[0]*r,d[1]*r,d[2]*r], sol=ik(g,ps); if(!sol.valid) break;
      if(!WE.legPhysics(g,ps,sol.angles,PHYS,thetaLim).ok) break;                 // 物理閘:撞曲柄/J1/J2/J3 即止
      const J=jac(g,ps,sol.angles), sa=singv(J.Ja), sx=singv(J.Jx);
      if(sx[0]<EPS_T2 || sa[5]/(sa[0]||1e-12)>KAPPA_THR) break; last=r; }
    sum+=last;
  }
  return sum/N;
}
function computeHome(p){
  let best={ z:0.6, yaw:0, reach:-1, valid:false };
  const scan=(yaws,zs)=>{ for(const yaw of yaws) for(const z of zs){ const m=meanReachAt(p,z,yaw);
    if(m>best.reach) best={ z, yaw, reach:m, valid:true }; } };
  const yawsC=[]; for(let y=0;y<120;y+=20) yawsC.push(y);
  const zsC=[];   for(let z=0.35;z<=1.25;z+=0.07) zsC.push(z);
  scan(yawsC, zsC);
  if(best.valid){ const yy=[],zz=[];
    for(let y=best.yaw-20;y<=best.yaw+20;y+=5) yy.push((y+120)%120);
    for(let z=Math.max(0.25,best.z-0.07);z<=best.z+0.07;z+=0.025) zz.push(z);
    scan(yy,zz); }
  const m=evalHome(p,best.z,best.yaw)||{t2:0,kappa:Infinity};
  best.t2=m.t2; best.kappa=m.kappa;
  best.yawDisp=((best.yaw+60)%120)-60;
  best.valid = best.reach>=0 && Number.isFinite(best.kappa);
  return best;
}
// θ*(在算出的 home 上)— 逐字搬頁面 thetaStar
const THETA_LEVELS=[4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46];
function thetaStar(p, home){
  const g=geom(p,home.yaw), thetaLim=WE.solveThetaLim(g,PHYS), Ndir=36, levels=THETA_LEVELS;
  let th=0,invK=0,nK=0;
  for(const lvl of levels){ let ok=true;
    for(let k=1;k<=Ndir;k++){ let d=[2*_hal(k,2)-1,2*_hal(k,3)-1,2*_hal(k,5)-1]; const dn=nrm3(d)||1; d=d.map(v=>v/dn);
      const ps=[0,0,home.z,d[0]*lvl,d[1]*lvl,d[2]*lvl], sol=ik(g,ps); if(!sol.valid){ok=false;break;}
      if(!WE.legPhysics(g,ps,sol.angles,PHYS,thetaLim).ok){ok=false;break;}        // 物理閘
      const J=jac(g,ps,sol.angles), sa=singv(J.Ja), sx=singv(J.Jx), kap=sa[5]/(sa[0]||1e-12);
      if(sx[0]<EPS_T2||kap>KAPPA_THR){ok=false;break;} invK+=1/kap; nK++; }
    if(ok) th=lvl; else break; }
  return { thetaStar:th, meanInvK:nK?invK/nK:0 };
}

// ---- 完整評估一組幾何 ----
function pairChords(p){ return { base:2*Math.sin(p.alphaB/2*DEG), plat:2*p.r*Math.sin(p.alphaP/2*DEG) }; }
function evalGeom(p){
  const home=computeHome(p);
  if(!home.valid) return { p, home, theta:0, meanInvK:0, chord:pairChords(p), feasible:false };
  // home 本身須物理可行（J2 不過彎、曲柄不撞）— 否則這組幾何連坐都坐不住
  const gh=geom(p,home.yaw), tl=WE.solveThetaLim(gh,PHYS), solH=ik(gh,[0,0,home.z,0,0,0]);
  const lpH = solH.valid ? WE.legPhysics(gh,[0,0,home.z,0,0,0],solH.angles,PHYS,tl) : { ok:false, crankMin:0 };
  if(!lpH.ok) return { p:{...p}, home, theta:0, meanInvK:0, chord:pairChords(p), homeCrank:lpH.crankMin, feasible:false };
  const ts=thetaStar(p,home), c=pairChords(p);
  return { p:{...p}, home, theta:ts.thetaStar, meanInvK:ts.meanInvK, chord:c, homeCrank:lpH.crankMin,
           buildable: c.base>=DELTA_JOINT-1e-3 && c.plat>=DELTA_JOINT-1e-3, feasible:true };
}

// ---- 參數空間 ----
const ASBUILT = { r:103/152, alphaB:18.92, alphaP:28.07, l:65/152, u:165/152, beta:0 };
const KEYS = ['r','alphaB','alphaP','l','u','beta'];
const LB = { r:0.35, alphaB:6,  alphaP:6,  l:0.20, u:0.60, beta:-60 };
const UB = { r:1.00, alphaB:48, alphaP:48, l:0.85, u:1.55, beta:60 };
const clampP=p=>{ const q={}; for(const k of KEYS) q[k]=Math.max(LB[k],Math.min(UB[k],p[k])); return q; };

// Halton 多基底取樣(可重現)
const HB=[2,3,5,7,11,13];
function sampleHalton(k){ const p={}; KEYS.forEach((key,i)=>p[key]=LB[key]+(UB[key]-LB[key])*_hal(k,HB[i])); return p; }

// 座標下降 polish(θ* 是階梯 → derivative-free;沿各軸掃細格,接受改善)
function polish(p0, scoreFn, rounds=3){
  let p={...p0}, best=scoreFn(evalGeom(p));
  const span=k=>(UB[k]-LB[k]);
  for(let r=0;r<rounds;r++){
    for(const k of KEYS){
      const step=span(k)*(r===0?0.06:r===1?0.03:0.015);
      for(const dir of [step,-step,2*step,-2*step]){
        const q=clampP({...p,[k]:p[k]+dir}), e=evalGeom(q), sc=scoreFn(e);
        if(sc>best){ best=sc; p=q; }
      }
    }
  }
  return { p, e:evalGeom(p), score:best };
}

// ---- 目標函數族 ----
// 主分數:θ* 為主(活動度),isotropy 細打破平手;Type-II 不足重罰
const scoreMobility = e => e.feasible ? e.theta + 0.8*e.meanInvK*100*0 + e.meanInvK*0 + (e.theta>0? e.meanInvK*2:0) : -1;
// 簡化:θ* + 小 isotropy bonus(避免 θ* 平台上亂跳)
const sMob   = e => e.feasible ? e.theta + 2*e.meanInvK : -1;
const sBal   = e => e.feasible ? e.theta*0.6 + 30*e.meanInvK : -1;            // 平衡:重各向同性
const sT2    = e => e.feasible ? e.theta*0.3 + 60*e.home.t2 + 10*e.meanInvK : -1; // Type-II 穩健:重 home σmin
const sBuild = e => (e.feasible && e.buildable) ? e.theta + 2*e.meanInvK : -1;    // 可製造 max θ*
// 微調(離 as-built 近):L2 球懲罰
function sNear(e){ if(!e.feasible) return -1; let d2=0; for(const k of KEYS){ const dn=(e.p[k]-ASBUILT[k])/(UB[k]-LB[k]); d2+=dn*dn; }
  return e.theta + 2*e.meanInvK - 9*d2; }

console.log('=== 取樣探索 (Halton 1500 點) ===');
const t0=Date.now();
const samples=[];
for(let k=1;k<=1500;k++){ const e=evalGeom(sampleHalton(k)); if(e.feasible) samples.push(e); }
console.log(`可行樣本 ${samples.length}/1500  耗時 ${((Date.now()-t0)/1000).toFixed(1)}s`);

const asb=evalGeom(ASBUILT);
console.log(`\nAS-BUILT: θ*=${asb.theta}°  meanInvK=${asb.meanInvK.toFixed(3)}  home z*=${asb.home.z.toFixed(3)} γ*=${asb.home.yawDisp.toFixed(1)}° reach=${asb.home.reach.toFixed(1)}° σmin=${asb.home.t2.toFixed(3)} κ=${asb.home.kappa.toFixed(1)} chord=${asb.chord.base.toFixed(3)}/${asb.chord.plat.toFixed(3)}`);

// 每類:取樣前幾名為起點 → polish
function bestOf(scoreFn, topSeed=6){
  const seeds=[...samples].sort((a,b)=>scoreFn(b)-scoreFn(a)).slice(0,topSeed).map(e=>e.p);
  seeds.push(ASBUILT);
  let best=null;
  for(const s of seeds){ const r=polish(s,scoreFn); if(!best||r.score>best.score) best=r; }
  return best;
}

const cats=[
  ['mobility','活動度優先(max θ*)',sMob],
  ['buildable','可製造 max θ*(弦≥0.329)',sBuild],
  ['balanced','平衡(θ*×各向同性)',sBal],
  ['t2robust','Type-II 穩健(max home σmin)',sT2],
  ['near','微調(離 as-built 最近)',sNear],
];
const results={};
for(const [key,label,fn] of cats){
  const b=bestOf(fn);
  results[key]={label, ...b.e};
  const e=b.e;
  console.log(`\n[${key}] ${label}`);
  console.log(`  r=${e.p.r.toFixed(3)} αb=${e.p.alphaB.toFixed(1)} αp=${e.p.alphaP.toFixed(1)} l=${e.p.l.toFixed(3)} u=${e.p.u.toFixed(3)} β=${e.p.beta.toFixed(1)}`);
  console.log(`  θ*=${e.theta}°  meanInvK=${e.meanInvK.toFixed(3)}  home z*=${e.home.z.toFixed(3)} γ*=${e.home.yawDisp.toFixed(1)}° reach=${e.home.reach.toFixed(1)}° σmin=${e.home.t2.toFixed(3)} κ=${e.home.kappa.toFixed(1)}  chord=${e.chord.base.toFixed(3)}/${e.chord.plat.toFixed(3)} ${e.buildable?'✓可造':'✗需重做接頭'}`);
}

// 輸出可貼進頁面的 JSON
console.log('\n=== PRESETS_JSON ===');
const presets=[];
const fmtNum=(n,d)=>Number(n.toFixed(d));
for(const [key,label] of cats){
  const e=results[key];
  presets.push({ id:key, label, p:{ r:fmtNum(e.p.r,4), alphaB:fmtNum(e.p.alphaB,2), alphaP:fmtNum(e.p.alphaP,2), l:fmtNum(e.p.l,4), u:fmtNum(e.p.u,4), beta:fmtNum(e.p.beta,1) },
    theta:e.theta, kappa:fmtNum(e.home.kappa,1), sigma:fmtNum(e.home.t2,3), reach:fmtNum(e.home.reach,1),
    z:fmtNum(e.home.z,3), gamma:fmtNum(e.home.yawDisp,1), buildable:e.buildable, crankMM:fmtNum((e.homeCrank||0)*PHYS.Rb,0) });
}
console.log(JSON.stringify(presets,null,2));
