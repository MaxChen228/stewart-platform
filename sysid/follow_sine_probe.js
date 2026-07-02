#!/usr/bin/env node
'use strict';
// Hardware truth-test for follow tracking. Drives a PURE-HEAVE (z-axis) sinusoid
// through the real PF pipeline and measures how faithfully the platform tracks it.
//
// Pipeline exercised end-to-end:
//   probe -> WS {cmd:"PF"} -> server 100Hz resampler -> ESP32 followStep
//        -> motor PID -> encoders -> telemetry a[] -> FK -> actual z(t)
//
// Output: amplitude ratio (gain), phase lag in ms (= effective dead-time/transport
// lag), per-sample tracking RMS/peak error, and the control-loop rate (lhz) seen
// during the run. Use it to compare control rates (L 10ms vs L 2.5ms) and to
// confirm fixes: a serial-starved board shows large, frequency-growing lag; a
// healthy one tracks with small constant lag.
//
// Usage:
//   node sysid/follow_sine_probe.js [--amp 6] [--freq 0.5] [--secs 12]
//        [--intent 60] [--settle 2] [--host localhost:3000] [--json]
// Safety: keep 2*pi*freq*amp below the VF translation limit (default 60mm/s) to
// measure tracking rather than slew saturation; the probe warns if it would clip.

const path = require('path');
const { performance } = require('perf_hooks');
const WS = require(path.join(__dirname, '..', 'node_modules', 'ws'));
const Kin = require('./kin.js');

function arg(name, def) {
  const i = process.argv.indexOf('--' + name);
  return i >= 0 && process.argv[i + 1] != null && !process.argv[i + 1].startsWith('--')
    ? process.argv[i + 1] : def;
}
const AMP = parseFloat(arg('amp', '6'));
const FREQ = parseFloat(arg('freq', '0.5'));
const SECS = parseFloat(arg('secs', '12'));
const INTENT_HZ = parseFloat(arg('intent', '60'));
const SETTLE = parseFloat(arg('settle', '2'));
const HOST = arg('host', 'localhost:3000');
const JSON_OUT = process.argv.includes('--json');
// Step mode: --step <deltaMm> drives a single z step (hold z0, then jump z0+delta)
// and reports rise time / overshoot% / settle time -- the right probe for the
// "drag to a point then stop" motion and for catching the tight-knob overshoot ring.
const STEP = process.argv.includes('--step') ? parseFloat(arg('step', '10')) : null;
const STEP_AT = parseFloat(arg('stepAt', '1.5'));   // when the step fires (s)

const W = 2 * Math.PI * FREQ;
const peakVel = W * AMP;
const log = (...a) => { if (!JSON_OUT) console.log(...a); };

// Safety gate: this probe drives the platform (FOLLOW 1 + z-sine PF stream) the
// instant the socket opens. Same contract as research_trial: a live run needs
// both --live and --i-am-at-rig so nothing moves hardware by accident.
const LIVE = process.argv.includes('--live');
const AT_RIG = process.argv.includes('--i-am-at-rig');
if (!LIVE) {
  console.log('[probe] dry-run (no hardware). Re-run with --live --i-am-at-rig to drive the rig.');
  console.log(`[probe] would stream z-sine amp=${AMP}mm freq=${FREQ}Hz peakVel=${peakVel.toFixed(1)}mm/s secs=${SECS} intent=${INTENT_HZ}Hz`);
  if (peakVel > 58) console.log(`[probe] ⚠️ peakVel ${peakVel.toFixed(1)} near/over VF limit (60mm/s) → would slew-clip`);
  process.exit(0);
}
if (LIVE && !AT_RIG) { console.error('[probe] --live requires --i-am-at-rig (human supervision + reachable power cut)'); process.exit(1); }

log(`[probe] z-sine amp=${AMP}mm freq=${FREQ}Hz peakVel=${peakVel.toFixed(1)}mm/s secs=${SECS} intent=${INTENT_HZ}Hz settle=${SETTLE}s`);
if (peakVel > 58) log(`[probe] ⚠️ peakVel ${peakVel.toFixed(1)} near/over VF limit (60mm/s) → expect slew clipping, not pure tracking`);

const ws = new WS(`ws://${HOST}`);
const samples = [];      // {t, a:[6], lhz}
let basePose = null;     // [x,y,z,r,p,y] captured from FOLLOW 1
let t0 = null;
let streamTimer = null;
let finished = false;

const send = (cmd) => ws.send(JSON.stringify({ cmd }));

ws.on('open', () => {
  // Crash safety net: if this probe dies (throw / SIGKILL) mid-stream, the board
  // stays in FOLLOW with motors enabled. Declaring autoLandOnClose hands the
  // WS-close to the server's authoritative safe-land (home->landing->power-cut),
  // which survives even SIGKILL. Safe here because the probe is one-shot (no batch
  // race) and its normal finish() ends by landing anyway.
  ws.send(JSON.stringify({ autoLandOnClose: true }));
  send('FOLLOW 1');
});

ws.on('message', (m) => {
  let d; try { d = JSON.parse(m.toString()); } catch { return; }
  if (!basePose && d.status && /follow on/i.test(d.status) && Array.isArray(d.pose)) {
    basePose = d.pose.slice(0, 6).map(Number);
    log('[probe] base pose =', basePose.map((v) => v.toFixed(2)).join(', '));
    startStreaming();
    return;
  }
  if (Array.isArray(d.a) && d.a.length === 6 && t0 != null && !finished) {
    samples.push({ t: (performance.now() - t0) / 1000, a: d.a.map(Number), lhz: d.lhz });
  }
});
ws.on('error', (e) => { console.error('[probe] ws error', e.message); process.exit(1); });

function startStreaming() {
  t0 = performance.now();
  const dtMs = 1000 / INTENT_HZ;
  streamTimer = setInterval(() => {
    const t = (performance.now() - t0) / 1000;
    if (t >= SECS) return finish();
    const z = STEP != null
      ? basePose[2] + (t >= STEP_AT ? STEP : 0)
      : basePose[2] + AMP * Math.sin(W * t);
    const p = [basePose[0], basePose[1], z, basePose[3], basePose[4], basePose[5]];
    send(`PF ${p.map((v) => v.toFixed(3)).join(' ')}`);
  }, dtMs);
}

function finish() {
  if (finished) return; finished = true;
  clearInterval(streamTimer);
  send('FOLLOW 0');
  setTimeout(() => { analyze(); try { ws.close(); } catch {} process.exit(0); }, 400);
}

// 3x3 linear solve (Gaussian elimination, partial pivot)
function solve3(A, b) {
  const M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let piv = c; for (let r = c + 1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < 3; r++) { if (r === c) continue; const f = M[r][c] / M[c][c]; for (let k = c; k <= 3; k++) M[r][k] -= f * M[c][k]; }
  }
  return [M[0][3] / M[0][0], M[1][3] / M[1][1], M[2][3] / M[2][2]];
}

function analyze() {
  if (!basePose || samples.length < 10) { console.log('[probe] insufficient data:', samples.length); return; }
  Kin.resetFK(basePose);
  if (STEP != null) return analyzeStep();
  const win = [];
  for (const s of samples) {
    if (s.t < SETTLE || s.t > SECS) continue;
    const fk = Kin.solveFK(s.a, { warmStart: basePose });
    if (fk && fk.converged) win.push({ t: s.t, z: fk.pose[2], lhz: s.lhz });
  }
  if (win.length < 10) { console.log('[probe] too few converged FK samples:', win.length); return; }

  // least-squares: z(t) ≈ c0 + c1 sin(Wt) + c2 cos(Wt)
  let S00 = 0, S01 = 0, S02 = 0, S11 = 0, S12 = 0, S22 = 0, b0 = 0, b1 = 0, b2 = 0;
  for (const p of win) {
    const s = Math.sin(W * p.t), c = Math.cos(W * p.t);
    S00 += 1; S01 += s; S02 += c; S11 += s * s; S12 += s * c; S22 += c * c;
    b0 += p.z; b1 += p.z * s; b2 += p.z * c;
  }
  const [c0, c1, c2] = solve3([[S00, S01, S02], [S01, S11, S12], [S02, S12, S22]], [b0, b1, b2]);
  const ampOut = Math.hypot(c1, c2);
  const psi = Math.atan2(c2, c1);          // output phase rel to sin(Wt)
  const lagMs = (-psi / W) * 1000;          // positive = lag
  const gain = ampOut / AMP;

  let sse = 0, maxErr = 0;
  for (const p of win) {
    const fitv = c0 + c1 * Math.sin(W * p.t) + c2 * Math.cos(W * p.t);
    const cmd = basePose[2] + AMP * Math.sin(W * p.t);
    sse += (p.z - fitv) ** 2;
    maxErr = Math.max(maxErr, Math.abs(p.z - cmd));
  }
  const fitRms = Math.sqrt(sse / win.length);
  const dts = []; for (let i = 1; i < win.length; i++) dts.push((win[i].t - win[i - 1].t) * 1000);
  dts.sort((a, b) => a - b);
  const teleHz = 1000 / (dts.reduce((a, b) => a + b, 0) / dts.length);
  const lhzs = win.map((p) => p.lhz).filter((v) => Number.isFinite(v));
  const lhzMin = lhzs.length ? Math.min(...lhzs) : null;
  const lhzAvg = lhzs.length ? Math.round(lhzs.reduce((a, b) => a + b, 0) / lhzs.length) : null;

  const result = {
    amp: AMP, freq: FREQ, peakVel: round(peakVel, 1),
    gain: round(gain, 3), lagMs: round(lagMs, 1),
    fitRmsMm: round(fitRms, 3), maxErrMm: round(maxErr, 3),
    samples: win.length, teleHz: round(teleHz, 1),
    teleDtP95Ms: round(dts[Math.floor(dts.length * 0.95)], 1), teleDtMaxMs: round(dts[dts.length - 1], 1),
    lhzAvg, lhzMin,
  };
  if (JSON_OUT) { console.log(JSON.stringify(result)); return; }
  console.log('\n=== follow tracking @ z-sine ===');
  console.log(`  gain (ampOut/ampIn)  : ${result.gain}   ${result.gain < 0.9 ? '⚠️ attenuated' : 'ok'}`);
  console.log(`  phase lag            : ${result.lagMs} ms   ${result.lagMs > 120 ? '⚠️ large dead-time' : ''}`);
  console.log(`  fit residual RMS     : ${result.fitRmsMm} mm   (non-sinusoid distortion / jitter)`);
  console.log(`  peak track error     : ${result.maxErrMm} mm`);
  console.log(`  telemetry            : ${result.teleHz} Hz  (dtP95=${result.teleDtP95Ms} dtMax=${result.teleDtMaxMs} ms, n=${result.samples})`);
  console.log(`  control loop lhz     : avg ${result.lhzAvg}  min ${result.lhzMin}`);
}
function analyzeStep() {
  const pts = [];
  for (const s of samples) {
    const fk = Kin.solveFK(s.a, { warmStart: basePose });
    if (fk && fk.converged) pts.push({ t: s.t, z: fk.pose[2], lhz: s.lhz });
  }
  if (pts.length < 20) { console.log('[probe] too few converged FK samples:', pts.length); return; }
  const z0 = mean(pts.filter((p) => p.t >= 0.3 && p.t < STEP_AT - 0.1).map((p) => p.z));
  const zf = mean(pts.filter((p) => p.t >= SECS - 1.0).map((p) => p.z));
  const delta = zf - z0;
  const dir = STEP >= 0 ? 1 : -1;
  const norm = (z) => (z - z0) / (delta || 1e-9);   // 0 at start, 1 at final
  const post = pts.filter((p) => p.t >= STEP_AT);
  // rise time 10->90%
  let t10 = null, t90 = null;
  for (const p of post) { const n = norm(p.z); if (t10 == null && n >= 0.1) t10 = p.t; if (t90 == null && n >= 0.9) { t90 = p.t; break; } }
  const riseMs = t10 != null && t90 != null ? (t90 - t10) * 1000 : null;
  const deadMs = t10 != null ? (t10 - STEP_AT) * 1000 : null;   // dead-time before motion
  // overshoot
  let peakN = -Infinity; for (const p of post) peakN = Math.max(peakN, norm(p.z));
  const overshootPct = round((peakN - 1) * 100, 1);
  // settle to +-2%
  let settleMs = null;
  for (let i = post.length - 1; i >= 0; i--) { if (Math.abs(norm(post[i].z) - 1) > 0.02) { settleMs = (post[i].t - STEP_AT) * 1000; break; } }
  const lhzs = pts.map((p) => p.lhz).filter(Number.isFinite);
  const result = {
    mode: 'step', stepMm: STEP, deltaActualMm: round(delta, 2),
    deadMs: deadMs != null ? round(deadMs, 0) : null,
    riseMs: riseMs != null ? round(riseMs, 0) : null,
    overshootPct, settleMs: settleMs != null ? round(settleMs, 0) : null,
    lhzAvg: lhzs.length ? Math.round(mean(lhzs)) : null, lhzMin: lhzs.length ? Math.min(...lhzs) : null,
    teleHz: round(1000 / mean(diffs(post.map((p) => p.t * 1000))), 1),
  };
  if (JSON_OUT) { console.log(JSON.stringify(result)); return; }
  console.log('\n=== follow STEP response ===');
  console.log(`  commanded step       : ${STEP} mm   (achieved ${result.deltaActualMm} mm)`);
  console.log(`  dead-time before move: ${result.deadMs} ms`);
  console.log(`  rise time (10-90%)   : ${result.riseMs} ms`);
  console.log(`  overshoot            : ${result.overshootPct} %   ${result.overshootPct > 8 ? '⚠️ ringing' : 'ok'}`);
  console.log(`  settle (+-2%)        : ${result.settleMs} ms`);
  console.log(`  control loop lhz     : avg ${result.lhzAvg}  min ${result.lhzMin}   telemetry ${result.teleHz} Hz`);
}
function mean(a) { return a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0; }
function diffs(a) { const d = []; for (let i = 1; i < a.length; i++) d.push(a[i] - a[i - 1]); return d.length ? d : [1]; }
function round(v, d) { const m = 10 ** d; return Math.round(v * m) / m; }

setTimeout(() => { if (!finished) { console.error('[probe] timeout, no follow-on/telemetry'); finish(); } }, (SECS + SETTLE + 8) * 1000);
