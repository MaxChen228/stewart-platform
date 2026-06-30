#!/usr/bin/env node
'use strict';
// ===== home 旋轉工作空間 3D 視覺化（roll/pitch/yaw 耦合）=====
// 用 ../kin.js 在 home(x=y=0,z=133) 掃 (roll,pitch,yaw) 網格判 IK-valid → 工作空間殼；
// 25% 徑向裕度子集 = 安全估計殼；疊真實手機 PF 實際用到的範圍雲。吐自包含 three.js HTML（軌道操作）。
// 用法：node sysid/phone_gen/workspace_viz.js [--out f.html]
const fs = require('fs'), path = require('path');
const Kin = require(path.join(__dirname, '..', 'kin.js'));
const HOME = [0, 0, 133, 0, 0, 0];
const ik = (r, p, y) => Kin.ik([HOME[0], HOME[1], HOME[2], r, p, y]).valid;

const OUT = (() => { const i = process.argv.indexOf('--out'); return i >= 0 ? process.argv[i + 1] : path.join(__dirname, 'workspace_3d.html'); })();
const CAP_DIR = path.join(__dirname, '..', 'data', 'phone-capture');

// ---- 網格 (deg) ----
const RR = [-42, 42, 1.5], PR = [-42, 42, 1.5], YR = [-92, 92, 2.5];
const ax = (a) => { const o = []; for (let v = a[0]; v <= a[1]; v += a[2]) o.push(+v.toFixed(2)); return o; };
const rs = ax(RR), ps = ax(PR), ys = ax(YR);
const key = (i, j, k) => i + ',' + j + ',' + k;

const valid = new Set(), safe = new Set();
for (let i = 0; i < rs.length; i++) for (let j = 0; j < ps.length; j++) for (let k = 0; k < ys.length; k++) {
  const r = rs[i], p = ps[j], y = ys[k];
  if (!ik(r, p, y)) continue;
  valid.add(key(i, j, k));
  if (ik(r * 1.25, p * 1.25, y * 1.25)) safe.add(key(i, j, k));   // 25% 徑向裕度 = 離幾何邊界有餘裕
}
function shell(set) {
  const out = [];
  for (const kk of set) {
    const [i, j, k] = kk.split(',').map(Number);
    const nb = [[i - 1, j, k], [i + 1, j, k], [i, j - 1, k], [i, j + 1, k], [i, j, k - 1], [i, j, k + 1]];
    if (nb.some(([a, b, c]) => !set.has(key(a, b, c)))) out.push([rs[i], ps[j], ys[k]]);
  }
  return out;
}
const samp = (a, n) => { if (a.length <= n) return a; const o = [], s = a.length / n; for (let t = 0; t < n; t++) o.push(a[Math.floor(t * s)]); return o; };

// ---- 手機 PF 實際用到的雲（兩場 pf.pose 的 roll/pitch/yaw）----
function phoneCloud(file, n) {
  const L = fs.readFileSync(file, 'utf8').trim().split('\n'); const pts = [];
  for (const l of L) { let j; try { j = JSON.parse(l); } catch { continue; } if (j.type === 'pf' && Array.isArray(j.pose)) pts.push([+j.pose[3].toFixed(2), +j.pose[4].toFixed(2), +j.pose[5].toFixed(2)]); }
  return samp(pts, n);
}
const caps = fs.existsSync(CAP_DIR) ? fs.readdirSync(CAP_DIR).filter(f => f.startsWith('cap_') && f.endsWith('.jsonl')).map(f => path.join(CAP_DIR, f)) : [];
let phone = []; for (const c of caps) phone = phone.concat(phoneCloud(c, 1100));

// ---- 統計 ----
const axMax = (axis) => { let m = 0, mn = 0; for (let v = 0; v <= 92; v += 0.5) { const a = [0, 0, 0]; a[axis] = v; if (ik(a[0], a[1], a[2])) m = v; else break; } for (let v = 0; v >= -92; v -= 0.5) { const a = [0, 0, 0]; a[axis] = v; if (ik(a[0], a[1], a[2])) mn = v; else break; } return [mn, m]; };
const rollAt = (y) => { let m = 0; for (let v = 0; v <= 60; v += 0.5) { if (ik(v, 0, y)) m = v; else break; } return m; };
const pfRange = () => { const mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9]; for (const p of phone) for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[k]); mx[k] = Math.max(mx[k], p[k]); } return { mn: mn.map(v => +v.toFixed(1)), mx: mx.map(v => +v.toFixed(1)) }; };
const voxVol = RR[2] * PR[2] * YR[2];
const stats = {
  rollLim: axMax(0), pitchLim: axMax(1), yawLim: axMax(2),
  coupling: { y0: rollAt(0), y30: rollAt(30), y50: rollAt(50), y70: rollAt(70) },
  wsVol: Math.round(valid.size * voxVol), safeVol: Math.round(safe.size * voxVol), safeFrac: +(safe.size / valid.size).toFixed(2),
  pf: pfRange(), nCaps: caps.length, nPhone: phone.length,
};
const DATA = {
  workspace: samp(shell(valid), 4500), safe: samp(shell(safe), 2800), phone: samp(phone, 2000),
  lim: { r: RR, p: PR, y: YR }, stats,
};

// ---- 自包含 three.js HTML ----
const html = `<!DOCTYPE html><html lang="zh-TW"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Stewart 工作空間 (roll/pitch/yaw 耦合)</title>
<style>
html,body{margin:0;height:100%;background:#0d1014;color:#e8eaed;font-family:-apple-system,"Noto Sans TC",sans-serif;overflow:hidden}
#c{position:fixed;inset:0}
.panel{position:fixed;background:rgba(20,24,30,.82);border:1px solid #2a313c;border-radius:10px;padding:12px 14px;font-size:13px;line-height:1.7;backdrop-filter:blur(6px)}
#legend{top:14px;left:14px}
#stats{top:14px;right:14px;max-width:300px}
#hint{bottom:12px;left:14px;font-size:12px;color:#8b95a3;background:none;border:none}
.sw{display:inline-block;width:11px;height:11px;border-radius:50%;margin-right:7px;vertical-align:-1px}
.k{color:#8b95a3}.v{color:#fff;font-variant-numeric:tabular-nums}
h3{margin:0 0 8px;font-size:13px;font-weight:600;letter-spacing:.03em}
hr{border:0;border-top:1px solid #2a313c;margin:9px 0}
</style></head><body>
<canvas id="c"></canvas>
<div id="legend" class="panel">
 <h3>Home 旋轉工作空間</h3>
 <div><span class="sw" style="background:#3d7fd6"></span>工作空間殼（IK 可解）</div>
 <div><span class="sw" style="background:#33c06a"></span>安全估計（25% 裕度）</div>
 <div><span class="sw" style="background:#ff8a1e"></span>手機實際用到 (PF)</div>
 <hr><div class="k" style="font-size:12px">軸：<span style="color:#e0564f">Roll(X)</span> · <span style="color:#5bcd6b">Pitch(Y)</span> · <span style="color:#5aa0e6">Yaw(Z)</span></div>
</div>
<div id="stats" class="panel">
 <h3>數據（kin.js 實算）</h3>
 <div><span class="k">單軸 Roll</span> <span class="v">${stats.rollLim[0]}° ~ +${stats.rollLim[1]}°</span></div>
 <div><span class="k">單軸 Pitch</span> <span class="v">${stats.pitchLim[0]}° ~ +${stats.pitchLim[1]}°</span></div>
 <div><span class="k">單軸 Yaw</span> <span class="v">${stats.yawLim[0]}° ~ +${stats.yawLim[1]}°</span></div>
 <hr><div class="k" style="font-size:12px;margin-bottom:3px">耦合：Roll 上限隨 Yaw 崩塌</div>
 <div><span class="k">Yaw 0°→</span><span class="v">Roll ${stats.coupling.y0}°</span> &nbsp; <span class="k">50°→</span><span class="v">${stats.coupling.y50}°</span> &nbsp; <span class="k">70°→</span><span class="v">${stats.coupling.y70}°</span></div>
 <hr><div><span class="k">安全 / 工作空間</span> <span class="v">${stats.safeFrac * 100}%</span></div>
 <div><span class="k">手機 Roll</span> <span class="v">${stats.pf.mn[0]}° ~ ${stats.pf.mx[0]}°</span></div>
 <div><span class="k">手機 Pitch</span> <span class="v">${stats.pf.mn[1]}° ~ ${stats.pf.mx[1]}°</span></div>
 <div><span class="k">手機 Yaw</span> <span class="v">${stats.pf.mn[2]}° ~ ${stats.pf.mx[2]}°</span></div>
</div>
<div id="hint" class="panel">拖曳旋轉 · 滾輪縮放 · 右鍵平移　|　軸單位 = 度，原點 = home</div>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
const DATA=${JSON.stringify(DATA)};
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x0d1014);
const cam=new THREE.PerspectiveCamera(48,innerWidth/innerHeight,1,2000);
cam.position.set(120,95,150);
const rdr=new THREE.WebGLRenderer({canvas:document.getElementById('c'),antialias:true});
rdr.setPixelRatio(devicePixelRatio); rdr.setSize(innerWidth,innerHeight);
const ctl=new THREE.OrbitControls(cam,rdr.domElement); ctl.enableDamping=true;
function cloud(pts,color,size,op){
  const g=new THREE.BufferGeometry(); const a=new Float32Array(pts.length*3);
  for(let i=0;i<pts.length;i++){a[i*3]=pts[i][0];a[i*3+1]=pts[i][1];a[i*3+2]=pts[i][2];}
  g.setAttribute('position',new THREE.BufferAttribute(a,3));
  const m=new THREE.PointsMaterial({color,size,transparent:true,opacity:op,sizeAttenuation:true,depthWrite:false});
  return new THREE.Points(g,m);
}
scene.add(cloud(DATA.workspace,0x3d7fd6,2.0,0.30));
scene.add(cloud(DATA.safe,0x33c06a,2.2,0.34));
scene.add(cloud(DATA.phone,0xff8a1e,3.0,0.85));
// 軸 + 標籤
function axis(dir,len,color){
  const g=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),dir.clone().multiplyScalar(len)]);
  scene.add(new THREE.Line(g,new THREE.LineBasicMaterial({color})));
}
const L=Math.max(DATA.lim.r[1],DATA.lim.p[1])+8, LY=DATA.lim.y[1]+8;
axis(new THREE.Vector3(1,0,0),L,0xe0564f); axis(new THREE.Vector3(-1,0,0),L,0x5a2a28);
axis(new THREE.Vector3(0,1,0),L,0x5bcd6b); axis(new THREE.Vector3(0,-1,0),L,0x2c5a34);
axis(new THREE.Vector3(0,0,1),LY,0x5aa0e6); axis(new THREE.Vector3(0,0,-1),LY,0x2a4c66);
function label(txt,pos,color){
  const cv=document.createElement('canvas'); cv.width=256;cv.height=64; const x=cv.getContext('2d');
  x.fillStyle=color; x.font='bold 34px sans-serif'; x.textAlign='center'; x.textBaseline='middle'; x.fillText(txt,128,32);
  const tx=new THREE.CanvasTexture(cv); const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:tx,transparent:true,depthWrite:false,depthTest:false}));
  sp.position.copy(pos); sp.scale.set(28,7,1); scene.add(sp);
}
label('Roll +'+DATA.lim.r[1]+'°',new THREE.Vector3(L+6,0,0),'#e0564f');
label('Pitch +'+DATA.lim.p[1]+'°',new THREE.Vector3(0,L+6,0),'#5bcd6b');
label('Yaw +'+DATA.lim.y[1]+'°',new THREE.Vector3(0,0,LY+6),'#5aa0e6');
// 原點 home 標記
const home=new THREE.Mesh(new THREE.SphereGeometry(2,16,16),new THREE.MeshBasicMaterial({color:0xffffff}));
scene.add(home);
// 淡格線（roll-pitch 平面，yaw=0）
const grid=new THREE.GridHelper(80,8,0x2a313c,0x1b2129); grid.rotation.x=Math.PI/2; scene.add(grid);
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();rdr.setSize(innerWidth,innerHeight);});
(function loop(){requestAnimationFrame(loop);ctl.update();rdr.render(scene,cam);})();
</script></body></html>`;

fs.writeFileSync(OUT, html);
console.log(JSON.stringify(stats, null, 1));
console.log('shell ws=' + DATA.workspace.length + ' safe=' + DATA.safe.length + ' phone=' + DATA.phone.length + '  → ' + OUT + ' (' + (html.length / 1024).toFixed(0) + ' KB)');
