#!/usr/bin/env node
'use strict';
// wave/extract.js — rig run 目錄 → wave.json（intent vs actual 馬達空間波形 + 指標）。
// 複用唯一真相源 ../kin.js 做 IK（intent_motor=IK(commanded base)），actual=遙測 a[6]。FK 在邊緣不可靠，故全程馬達空間。
// 用法：node extract.js <run_dir> [--win s0,s1] [--out wave.json] [--configs tagA,tagB]
//   run_dir 須含 rawlog 產物 raw_ws.jsonl + sent.jsonl，且 sent 有 cand_start/cand_end marker、PF 帶 base。
const fs = require('fs');
const path = require('path');
const kin = require(path.join(__dirname, '..', 'kin.js'));

function arg(name, def) { const i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }
const dir = process.argv[2];
if (!dir || dir.startsWith('--')) { console.error('用法: node extract.js <run_dir> [--win s0,s1] [--out f] [--configs a,b]'); process.exit(1); }
const [W0, W1] = (arg('--win', '3,11')).split(',').map(Number);
const outFile = arg('--out', path.join(dir, 'wave.json'));
const cfgFilter = arg('--configs', null) ? new Set(arg('--configs', '').split(',')) : null;
const DT = 25; // ms grid

const readJsonl = f => fs.readFileSync(f, 'utf8').trim().split('\n').map(l => { try { return JSON.parse(l); } catch (e) { return null; } }).filter(Boolean);
const ws = readJsonl(path.join(dir, 'raw_ws.jsonl'));
const sent = readJsonl(path.join(dir, 'sent.jsonl'));

const pf = [];
for (const s of sent) if (s.msg && typeof s.msg.cmd === 'string' && s.msg.cmd.startsWith('PF ') && Array.isArray(s.msg.base)) pf.push({ t: s.t, base: s.msg.base });
const tele = [];
for (const e of ws) { let o; try { o = JSON.parse(e.raw); } catch (_) { continue; } const d = o.data || o; if (Array.isArray(d.a) && d.a.length === 6) tele.push({ t: e.t, fl: d.fl, a: d.a }); }
const segs = [];
for (const s of sent) { if (s.marker === 'cand_start') segs.push({ tag: s.info.tag, t0: s.t }); if (s.marker === 'cand_end' && segs.length) segs[segs.length - 1].t1 = s.t; }
let cands = segs.filter(s => s.t1 != null);
if (cfgFilter) cands = cands.filter(s => cfgFilter.has(s.tag));
if (!pf.length || !tele.length || !cands.length) { console.error(`資料不足: pf=${pf.length} tele=${tele.length} cands=${cands.length}`); process.exit(1); }

function zohBase(t) { let lo = 0, hi = pf.length - 1, r = pf[0]; while (lo <= hi) { const m = (lo + hi) >> 1; if (pf[m].t <= t) { r = pf[m]; lo = m + 1; } else hi = m - 1; } return r.base; }
function actualAt(arr, m, t) { let lo = 0, hi = arr.length - 1; while (lo < hi) { const k = (lo + hi) >> 1; if (arr[k].t < t) lo = k + 1; else hi = k; } if (lo === 0) return arr[0].a[m]; const a = arr[lo - 1], b = arr[lo]; if (b.t === a.t) return a.a[m]; const f = (t - a.t) / (b.t - a.t); return a.a[m] + (b.a[m] - a.a[m]) * f; }
const rms = v => Math.sqrt(v.reduce((a, b) => a + b * b, 0) / v.length);
const std = v => { const m = v.reduce((a, b) => a + b, 0) / v.length; return Math.sqrt(v.reduce((a, b) => a + (b - m) ** 2, 0) / v.length); };
function bestLag(intent, actual) { const im = intent.reduce((a, b) => a + b) / intent.length, am = actual.reduce((a, b) => a + b) / actual.length; let bl = 0, bc = -2; for (let L = 0; L <= 20; L++) { let n = 0, d1 = 0, d2 = 0; for (let i = L; i < intent.length; i++) { const x = intent[i - L] - im, y = actual[i] - am; n += x * y; d1 += x * x; d2 += y * y; } const c = n / Math.sqrt(d1 * d2 || 1); if (c > bc) { bc = c; bl = L; } } return bl; }

const gridRel = []; for (let t = W0 * 1000; t <= W1 * 1000; t += DT) gridRel.push(t);
const out = { run: path.basename(dir.replace(/\/$/, '')), winSec: [W0, W1], dtMs: DT, tRel: gridRel.map(t => t / 1000), configs: {} };
for (const sg of cands) {
  const segTele = tele.filter(x => x.t >= sg.t0 && x.t <= sg.t1);
  const cfg = { intent: [], actual: [], rawRms: [], lagMs: [], trkRms: [], gain: [] };
  for (let m = 0; m < 6; m++) {
    const intent = gridRel.map(rt => { const b = zohBase(sg.t0 + rt); const ik = kin.ik([0, 0, 133, b[0], b[1], b[2]]); return ik.valid ? ik.angles[m] : null; });
    const actual = gridRel.map(rt => actualAt(segTele, m, sg.t0 + rt));
    cfg.intent.push(intent.map(v => v == null ? null : +v.toFixed(3)));
    cfg.actual.push(actual.map(v => +v.toFixed(3)));
    const ok = intent.map((v, i) => v == null ? null : actual[i] - v).filter(v => v != null);
    cfg.rawRms.push(+rms(ok).toFixed(3));
    const vi = intent.map((v, i) => [v, i]).filter(p => p[0] != null);
    const I = vi.map(p => p[0]), A = vi.map(p => actual[p[1]]);
    const L = bestLag(I, A); cfg.lagMs.push(L * DT);
    const ed = []; for (let i = L; i < I.length; i++) ed.push(A[i] - I[i - L]); cfg.trkRms.push(+rms(ed).toFixed(3));
    cfg.gain.push(+(std(A) / (std(I) || 1)).toFixed(3));
  }
  // 哨兵：掃窗內原始幀的不合理尖刺（|a|>120° 絕對越界，或單幀 >50° 跳出再回彈的編碼器 glitch）。記錄不改資料。
  const winRaw = segTele.filter(x => x.t >= sg.t0 + W0 * 1000 && x.t <= sg.t0 + W1 * 1000);
  cfg.spikes = [];
  for (let k = 0; k < winRaw.length; k++) for (let m = 0; m < 6; m++) {
    const v = winRaw[k].a[m], pv = winRaw[Math.max(0, k - 1)].a[m], nv = winRaw[Math.min(winRaw.length - 1, k + 1)].a[m];
    if (Math.abs(v) > 120 || (Math.abs(v - pv) > 50 && Math.abs(v - nv) > 50 && Math.sign(v - pv) === Math.sign(v - nv)))
      cfg.spikes.push({ tRel: +((winRaw[k].t - sg.t0) / 1000).toFixed(2), m: m + 1, v: +v.toFixed(1) });
  }
  out.configs[sg.tag] = cfg;
}
fs.writeFileSync(outFile, JSON.stringify(out));
console.log(`寫出 ${outFile}  窗 ${W0}-${W1}s  ${gridRel.length} 點  組態 ${Object.keys(out.configs).join(', ')}`);
console.log('組態                | rawRMS° M1..M6                      | mean | lag ms | trk°');
for (const tag of Object.keys(out.configs)) { const c = out.configs[tag]; console.log(tag.padEnd(19), '|', c.rawRms.map(v => v.toFixed(1).padStart(5)).join(' '), '|', (c.rawRms.reduce((a, b) => a + b) / 6).toFixed(2), '|', (c.lagMs.reduce((a, b) => a + b) / 6).toFixed(0).padStart(5), '|', (c.trkRms.reduce((a, b) => a + b) / 6).toFixed(2)); }
const totSpk = Object.values(out.configs).reduce((s, c) => s + c.spikes.length, 0);
if (totSpk) { console.log(`\n⚠️ 窗內偵測到 ${totSpk} 個編碼器尖刺（已記錄於 wave.json，未改資料）：`); for (const tag of Object.keys(out.configs)) for (const s of out.configs[tag].spikes) console.log(`   ${tag} t=${s.tRel}s M${s.m}=${s.v}°`); }
else console.log('\n✅ 窗內無編碼器尖刺異常');
