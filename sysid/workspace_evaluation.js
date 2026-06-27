#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { solveFK, resetFK, NEUTRAL_Z } = require('./kin');

const AXES = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'];
const MOTOR_LABELS = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6'];

function usage() {
  console.log(`Usage:
  node sysid/workspace_evaluation.js sysid/data/run.jsonl [more.jsonl...] [--out report.json] [--md-out report.md]

Full-lifecycle Workspace evaluation derived from raw motor angles, FK pose,
phase timing, command tracking, and telemetry health.
`);
}

function round(v, d = 4) {
  return Number.isFinite(v) ? Number(v.toFixed(d)) : null;
}

function mean(xs) {
  return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : NaN;
}

function rms(xs) {
  return xs.length ? Math.sqrt(mean(xs.map((x) => x * x))) : NaN;
}

function percentile(xs, p) {
  const a = xs.filter(Number.isFinite).sort((x, y) => x - y);
  if (!a.length) return NaN;
  const i = Math.max(0, Math.min(a.length - 1, Math.ceil(p * a.length) - 1));
  return a[i];
}

function counterDelta(values) {
  let total = 0;
  for (let i = 1; i < values.length; i++) {
    const prev = values[i - 1], cur = values[i];
    if (!Number.isFinite(prev) || !Number.isFinite(cur)) continue;
    total += cur >= prev ? cur - prev : cur;
  }
  return total;
}

function parsePoseCommand(cmd) {
  const p = String(cmd || '').trim().split(/\s+/);
  if (p[0] !== 'P' && p[0] !== 'PF') return null;
  const pose = p.slice(1, 7).map(Number);
  if (pose.length !== 6 || pose.some((x) => !Number.isFinite(x))) return null;
  return { kind: p[0], pose, ms: Number(p[7]) || (p[0] === 'P' ? 1500 : 0) };
}

function minJerk01(u) {
  const x = Math.max(0, Math.min(1, u));
  return x * x * x * (10 + x * (-15 + 6 * x));
}

function load(file) {
  resetFK([0, 0, NEUTRAL_Z, 0, 0, 0]);
  const cmds = [];
  const events = [];
  const rows = [];
  let meta = null;
  for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
    if (!line.trim()) continue;
    let rec;
    try { rec = JSON.parse(line); } catch { continue; }
    if (rec.meta) { meta = rec.meta; continue; }
    if (rec.dir === 'cmd') {
      const cmd = String(rec.d || '').trim();
      cmds.push({ t: Number(rec.t), cmd, pose: parsePoseCommand(cmd) });
      continue;
    }
    if (rec.dir === 'event') {
      const d = rec.d && typeof rec.d === 'object' ? rec.d : {};
      events.push({ t: Number(rec.t), ...d });
      continue;
    }
    if (rec.dir !== 'in') continue;
    let d;
    try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
    if (!d || !Array.isArray(d.a) || d.a.length !== 6) continue;
    const fk = solveFK(d.a);
    rows.push({
      t: Number(rec.t),
      motor: d.a.map(Number),
      pose: fk.pose.map(Number),
      fkOk: fk.converged,
      raw: d,
    });
  }
  rows.sort((a, b) => a.t - b.t);
  cmds.sort((a, b) => a.t - b.t);
  events.sort((a, b) => a.t - b.t);
  return { file, meta, cmds, events, rows };
}

function phaseWindows(events, rows) {
  if (!rows.length) return [];
  const phases = events.filter((e) => e.type === 'session_phase').map((e) => ({ t: e.t, phase: e.phase }));
  const out = [];
  if (!phases.length) return [{ phase: 'full', start: rows[0].t, end: rows[rows.length - 1].t }];
  if (phases[0].t > rows[0].t) out.push({ phase: 'pre-session', start: rows[0].t, end: phases[0].t });
  for (let i = 0; i < phases.length; i++) {
    const end = i + 1 < phases.length ? phases[i + 1].t : rows[rows.length - 1].t;
    if (end > phases[i].t) out.push({ phase: phases[i].phase, start: phases[i].t, end });
  }
  return out;
}

function rowsIn(rows, start, end) {
  return rows.filter((r) => r.t >= start && r.t <= end);
}

function targetSegments(cmds, rows) {
  const segs = [];
  let last = rows[0]?.pose || [0, 0, NEUTRAL_Z, 0, 0, 0];
  for (const c of cmds) {
    if (!c.pose) continue;
    if (c.pose.kind === 'PF') {
      segs.push({ kind: 'hold', start: c.t, end: Infinity, from: c.pose.pose, to: c.pose.pose, cmd: c.cmd });
      last = c.pose.pose;
      continue;
    }
    const before = [...rows].reverse().find((r) => r.t <= c.t)?.pose || last;
    const end = c.t + c.pose.ms;
    segs.push({ kind: 'minjerk', start: c.t, end, from: before, to: c.pose.pose, ms: c.pose.ms, cmd: c.cmd });
    segs.push({ kind: 'hold', start: end, end: Infinity, from: c.pose.pose, to: c.pose.pose, cmd: `hold after ${c.cmd}` });
    last = c.pose.pose;
  }
  segs.sort((a, b) => a.start - b.start);
  for (let i = 0; i < segs.length - 1; i++) segs[i].end = Math.min(segs[i].end, segs[i + 1].start);
  return segs;
}

function targetAt(segs, t) {
  let seg = null;
  for (const s of segs) {
    if (s.start <= t && t <= s.end) seg = s;
    if (s.start > t) break;
  }
  if (!seg) return null;
  if (seg.kind === 'hold') return seg.to;
  const u = minJerk01((t - seg.start) / Math.max(1, seg.ms));
  return seg.from.map((v, i) => v + (seg.to[i] - v) * u);
}

function commandedAxesForWindow(segs, start, end) {
  const axes = new Set();
  for (const s of segs) {
    if (s.end < start || s.start > end) continue;
    for (let i = 0; i < 6; i++) {
      if (Math.abs(s.to[i] - s.from[i]) > 0.5) axes.add(i);
    }
  }
  return axes;
}

function diffs(rows, key, idx) {
  const out = [];
  for (let i = 1; i < rows.length; i++) {
    const dt = (rows[i].t - rows[i - 1].t) / 1000;
    if (dt <= 0) continue;
    out.push({
      t: rows[i].t,
      dt,
      step: rows[i][key][idx] - rows[i - 1][key][idx],
      rate: (rows[i][key][idx] - rows[i - 1][key][idx]) / dt,
    });
  }
  return out;
}

function secondDiff(rows, key, idx) {
  const out = [];
  const v = diffs(rows, key, idx);
  for (let i = 1; i < v.length; i++) {
    const dt = v[i].t - v[i - 1].t;
    if (dt <= 0) continue;
    out.push({ t: v[i].t, accel: (v[i].rate - v[i - 1].rate) / (dt / 1000) });
  }
  return out;
}

function thirdDiff(rows, key, idx) {
  const out = [];
  const a = secondDiff(rows, key, idx);
  for (let i = 1; i < a.length; i++) {
    const dt = a[i].t - a[i - 1].t;
    if (dt <= 0) continue;
    out.push({ t: a[i].t, jerk: (a[i].accel - a[i - 1].accel) / (dt / 1000) });
  }
  return out;
}

function movingAverage(values, times, windowMs) {
  const out = [];
  let j = 0, sum = 0;
  for (let i = 0; i < values.length; i++) {
    sum += values[i];
    while (times[i] - times[j] > windowMs) sum -= values[j++];
    out.push(sum / Math.max(1, i - j + 1));
  }
  return out;
}

function countVelocityPeaks(rows, key, idx) {
  const v = diffs(rows, key, idx).map((x) => Math.abs(x.rate));
  if (v.length < 5) return 0;
  const threshold = Math.max(0.02, percentile(v, 0.35));
  let peaks = 0;
  for (let i = 1; i < v.length - 1; i++) {
    if (v[i] > threshold && v[i] > v[i - 1] && v[i] >= v[i + 1]) peaks++;
  }
  return peaks;
}

function spectralArcLength(rows, key, idx) {
  const vel = diffs(rows, key, idx).map((x) => Math.abs(x.rate));
  if (vel.length < 16) return null;
  const n = vel.length;
  const avgDt = mean(diffs(rows, key, idx).map((x) => x.dt));
  if (!Number.isFinite(avgDt) || avgDt <= 0) return null;
  const maxK = Math.min(Math.floor(n / 2), 80);
  const mags = [];
  for (let k = 0; k <= maxK; k++) {
    let re = 0, im = 0;
    for (let j = 0; j < n; j++) {
      const ang = -2 * Math.PI * k * j / n;
      re += vel[j] * Math.cos(ang);
      im += vel[j] * Math.sin(ang);
    }
    mags.push(Math.sqrt(re * re + im * im));
  }
  const m0 = Math.max(...mags) || 1;
  const norm = mags.map((m) => m / m0);
  let sal = 0;
  for (let k = 1; k < norm.length; k++) {
    const df = 1 / (n * avgDt);
    sal -= Math.sqrt(df * df + (norm[k] - norm[k - 1]) ** 2);
  }
  return sal;
}

function axisSignalStats(rows, key, labels) {
  return labels.map((label, idx) => {
    const values = rows.map((r) => r[key][idx]).filter(Number.isFinite);
    const times = rows.map((r) => r.t);
    const hpBase = movingAverage(values, times, 700);
    const hp = values.map((v, i) => v - hpBase[i]);
    const step = diffs(rows, key, idx).map((x) => Math.abs(x.step));
    const rate = diffs(rows, key, idx).map((x) => Math.abs(x.rate));
    const accel = secondDiff(rows, key, idx).map((x) => Math.abs(x.accel));
    const jerk = thirdDiff(rows, key, idx).map((x) => Math.abs(x.jerk));
    const maxStepIdx = step.length ? step.indexOf(Math.max(...step)) + 1 : -1;
    return {
      label,
      range: round(Math.max(...values) - Math.min(...values), 4),
      hpRms: round(rms(hp), 4),
      hpP95: round(percentile(hp.map(Math.abs), 0.95), 4),
      hpPeak: round(Math.max(0, ...hp.map(Math.abs)), 4),
      stepP95: round(percentile(step, 0.95), 4),
      stepP99: round(percentile(step, 0.99), 4),
      stepMax: round(Math.max(0, ...step), 4),
      stepMaxAtS: maxStepIdx >= 0 ? round((rows[maxStepIdx].t - rows[0].t) / 1000, 3) : null,
      rateP95: round(percentile(rate, 0.95), 4),
      rateMax: round(Math.max(0, ...rate), 4),
      accelP95: round(percentile(accel, 0.95), 4),
      accelMax: round(Math.max(0, ...accel), 4),
      jerkP95: round(percentile(jerk, 0.95), 4),
      jerkMax: round(Math.max(0, ...jerk), 4),
      velocityPeaks: countVelocityPeaks(rows, key, idx),
      spectralArcLength: round(spectralArcLength(rows, key, idx), 4),
    };
  });
}

function phaseByName(phases, name) {
  return phases.find((p) => p.phase === name) || null;
}

function phaseLike(phases, names) {
  return phases.filter((p) => names.includes(p.phase));
}

function waveformQuality(result) {
  const takeoff = phaseByName(result.phases, 'takeoff');
  const landing = phaseByName(result.phases, 'landing');
  const motionPhases = phaseLike(result.phases, ['takeoff', 'landing', 'follow', 'block 1/1', 'close loop']);
  const transitionPhases = [takeoff, landing].filter(Boolean);
  const transitionStepMax = Math.max(0, ...transitionPhases.map((p) => Number(p.motorStepMaxDeg)).filter(Number.isFinite));
  const transitionCrossPeak = Math.max(0, ...transitionPhases.map((p) => Number(p.crossHpPeak)).filter(Number.isFinite));
  const transitionPeaks = transitionPhases.reduce((sum, p) => sum + (Number(p.velocityPeaksTotal) || 0), 0);
  const fullStep = Number(result.fullBadness.motorStepMaxDeg);
  const fullP99 = Number(result.fullBadness.motorStepP99MeanDeg);
  const fullCross = Number(result.fullBadness.poseCrossHpPeak);

  const failures = [];
  if (fullStep > 6) failures.push(`full max motor step ${round(fullStep, 2)}deg > 6deg`);
  if (transitionStepMax > 4) failures.push(`takeoff/landing max motor step ${round(transitionStepMax, 2)}deg > 4deg`);
  if (fullP99 > 2) failures.push(`mean motor step p99 ${round(fullP99, 2)}deg > 2deg`);
  if (transitionCrossPeak > 4) failures.push(`takeoff/landing cross-axis peak ${round(transitionCrossPeak, 2)} > 4`);
  if (result.health.efOr) failures.push(`CAN EFLG non-zero (${result.health.efOr})`);
  if (result.health.rxDrop > 0) failures.push(`CAN rxDrop ${result.health.rxDrop}`);

  const verdict = failures.length ? 'reject' : 'no-hard-fail';
  return {
    verdict,
    failures,
    transitionStepMaxDeg: round(transitionStepMax, 3),
    transitionCrossPeak: round(transitionCrossPeak, 3),
    transitionVelocityPeaks: transitionPeaks,
    phaseCount: motionPhases.length,
    takeoff: takeoff ? {
      motorStepMaxDeg: takeoff.motorStepMaxDeg,
      crossHpPeak: takeoff.crossHpPeak,
      worstMotor: takeoff.worstMotorStep?.label || null,
      worstCrossAxis: takeoff.worstCrossAxis?.label || null,
    } : null,
    landing: landing ? {
      motorStepMaxDeg: landing.motorStepMaxDeg,
      crossHpPeak: landing.crossHpPeak,
      worstMotor: landing.worstMotorStep?.label || null,
      worstCrossAxis: landing.worstCrossAxis?.label || null,
    } : null,
  };
}

function phaseMetrics(rows, segs, events) {
  return phaseWindows(events, rows).map((w) => {
    const part = rowsIn(rows, w.start, w.end);
    const commanded = commandedAxesForWindow(segs, w.start, w.end);
    const poseStats = axisSignalStats(part, 'pose', AXES);
    const motorStats = axisSignalStats(part, 'motor', MOTOR_LABELS);
    const cross = poseStats.filter((_, i) => !commanded.has(i));
    return {
      phase: w.phase,
      startS: round((w.start - rows[0].t) / 1000, 3),
      endS: round((w.end - rows[0].t) / 1000, 3),
      samples: part.length,
      commandedAxes: [...commanded].map((i) => AXES[i]),
      motorStepMaxDeg: round(Math.max(0, ...motorStats.map((s) => s.stepMax)), 4),
      motorJerkP95: round(mean(motorStats.map((s) => s.jerkP95).filter(Number.isFinite)), 4),
      crossHpPeak: round(Math.max(0, ...cross.map((s) => s.hpPeak)), 4),
      crossHpP95Mean: round(mean(cross.map((s) => s.hpP95).filter(Number.isFinite)), 4),
      velocityPeaksTotal: motorStats.reduce((sum, s) => sum + s.velocityPeaks, 0),
      worstMotorStep: motorStats.reduce((best, s) => (s.stepMax || 0) > (best.stepMax || 0) ? s : best, {}),
      worstCrossAxis: cross.reduce((best, s) => (s.hpPeak || 0) > (best.hpPeak || 0) ? s : best, {}),
    };
  });
}

function phaseAt(events, t) {
  let phase = 'pre-session';
  for (const e of events) {
    if (e.t > t) break;
    if (e.type === 'session_phase' && e.phase) phase = e.phase;
  }
  return phase;
}

function lastCommandBefore(cmds, t) {
  let out = null;
  for (const c of cmds) {
    if (c.t > t) break;
    out = c;
  }
  return out;
}

function topMotorStepEvents(rows, events, cmds, limit = 10) {
  const out = [];
  for (let i = 1; i < rows.length; i++) {
    const before = rows[i - 1];
    const after = rows[i];
    const dtMs = after.t - before.t;
    if (!(dtMs > 0)) continue;
    for (let m = 0; m < 6; m++) {
      const step = after.motor[m] - before.motor[m];
      if (!Number.isFinite(step)) continue;
      out.push({
        tS: round((after.t - rows[0].t) / 1000, 4),
        phase: phaseAt(events, after.t),
        motor: MOTOR_LABELS[m],
        stepDeg: round(step, 4),
        absStepDeg: round(Math.abs(step), 4),
        dtMs: round(dtMs, 2),
        lastCmd: lastCommandBefore(cmds, after.t)?.cmd || null,
        before: {
          motorDeg: round(before.motor[m], 4),
          pose: Object.fromEntries(AXES.map((axis, idx) => [axis, round(before.pose[idx], 4)])),
        },
        after: {
          motorDeg: round(after.motor[m], 4),
          pose: Object.fromEntries(AXES.map((axis, idx) => [axis, round(after.pose[idx], 4)])),
        },
        health: {
          ok: after.raw.ok ?? null,
          loopHz: after.raw.lhz ?? null,
          loopDtMs: after.raw.dt ?? null,
          canEf: after.raw.can?.ef ?? after.raw.ef ?? null,
          canTx: after.raw.can?.tx ?? after.raw.tx ?? null,
          rxDrop: after.raw.can?.rxDrop ?? null,
          arAge: Array.isArray(after.raw.arAge) ? after.raw.arAge.map((x) => round(Number(x), 1)) : null,
        },
      });
    }
  }
  out.sort((a, b) => b.absStepDeg - a.absStepDeg);
  return out.slice(0, limit);
}

function analyze(file) {
  const data = load(file);
  const segs = targetSegments(data.cmds, data.rows);
  const targets = data.rows.map((r) => targetAt(segs, r.t));
  const errRows = data.rows.map((r, i) => ({ ...r, err: targets[i] ? r.pose.map((v, j) => v - targets[i][j]) : null }));
  const durationS = data.rows.length >= 2 ? (data.rows[data.rows.length - 1].t - data.rows[0].t) / 1000 : 0;
  const motorStats = axisSignalStats(data.rows, 'motor', MOTOR_LABELS);
  const poseStats = axisSignalStats(data.rows, 'pose', AXES);
  const errStats = AXES.map((label, idx) => {
    const values = errRows.map((r) => r.err?.[idx]).filter(Number.isFinite);
    return {
      label,
      rms: round(rms(values), 4),
      p95: round(percentile(values.map(Math.abs), 0.95), 4),
      peak: round(Math.max(0, ...values.map(Math.abs)), 4),
    };
  });
  const rxDrop = counterDelta(data.rows.map((r) => Number(r.raw.can?.rxDrop)).filter(Number.isFinite));
  const arAges = data.rows.flatMap((r) => Array.isArray(r.raw.arAge) ? r.raw.arAge.map(Number) : []);
  const efOr = data.rows.reduce((s, r) => s | (Number(r.raw.can?.ef ?? r.raw.ef ?? 0) & 0xff), 0);
  const txMax = Math.max(0, ...data.rows.map((r) => Number(r.raw.can?.tx ?? r.raw.tx ?? 0)).filter(Number.isFinite));
  const lhz = data.rows.map((r) => Number(r.raw.lhz)).filter(Number.isFinite);
  const fullBadness = {
    motorStepMaxDeg: round(Math.max(0, ...motorStats.map((s) => s.stepMax)), 4),
    motorStepP99MeanDeg: round(mean(motorStats.map((s) => s.stepP99).filter(Number.isFinite)), 4),
    motorJerkP95Mean: round(mean(motorStats.map((s) => s.jerkP95).filter(Number.isFinite)), 4),
    poseCrossHpPeak: round(Math.max(0, ...poseStats.filter((_, i) => ![2, 5].includes(i)).map((s) => s.hpPeak)), 4),
    velocityPeaksTotal: motorStats.reduce((sum, s) => sum + s.velocityPeaks, 0),
    worstMotorStep: motorStats.reduce((best, s) => (s.stepMax || 0) > (best.stepMax || 0) ? s : best, {}),
  };
  const result = {
    file: path.basename(file),
    path: file,
    meta: data.meta,
    durationS: round(durationS, 3),
    samples: data.rows.length,
    commands: data.cmds.map((c) => ({ tS: round((c.t - data.rows[0].t) / 1000, 3), cmd: c.cmd })),
    fullBadness,
    topEvents: {
      motorSteps: topMotorStepEvents(data.rows, data.events, data.cmds, 12),
    },
    motor: motorStats,
    pose: poseStats,
    trackingErrorToCommand: errStats,
    phases: phaseMetrics(data.rows, segs, data.events),
    health: {
      okFailFrac: round(data.rows.filter((r) => Number(r.raw.ok) < 6).length / Math.max(1, data.rows.length), 5),
      fkFailFrac: round(data.rows.filter((r) => !r.fkOk).length / Math.max(1, data.rows.length), 5),
      efOr,
      txMax,
      rxDrop,
      rxDropPerS: durationS > 0 ? round(rxDrop / durationS, 4) : null,
      arAgeMaxMs: round(Math.max(0, ...arAges), 3),
      loopHzMin: round(Math.min(...lhz), 3),
      loopHzMax: round(Math.max(...lhz), 3),
    },
  };
  result.quality = waveformQuality(result);
  return result;
}

function summarizeRuns(results) {
  const rows = results.map((r) => ({
    file: r.file,
    verdict: r.quality.verdict,
    fullStep: r.fullBadness.motorStepMaxDeg,
    p99: r.fullBadness.motorStepP99MeanDeg,
    cross: r.fullBadness.poseCrossHpPeak,
    worstMotor: r.fullBadness.worstMotorStep?.label || null,
    takeoffStep: r.quality.takeoff?.motorStepMaxDeg ?? null,
    landingStep: r.quality.landing?.motorStepMaxDeg ?? null,
    rxDropPerS: r.health.rxDropPerS,
    efOr: r.health.efOr,
  }));
  const nums = (key) => rows.map((r) => Number(r[key])).filter(Number.isFinite);
  return {
    count: rows.length,
    verdicts: rows.reduce((acc, r) => {
      acc[r.verdict] = (acc[r.verdict] || 0) + 1;
      return acc;
    }, {}),
    fullMotorStepMaxDeg: { mean: round(mean(nums('fullStep')), 3), max: round(Math.max(0, ...nums('fullStep')), 3) },
    poseCrossHpPeak: { mean: round(mean(nums('cross')), 3), max: round(Math.max(0, ...nums('cross')), 3) },
    rxDropPerS: { mean: round(mean(nums('rxDropPerS')), 3), max: round(Math.max(0, ...nums('rxDropPerS')), 3) },
    rows,
  };
}

function markdownReport(results) {
  const summary = summarizeRuns(results);
  const lines = [];
  lines.push('# Workspace Execution Feature Report');
  lines.push('');
  lines.push('This report is source-data-first: it uses full lifecycle telemetry and raw waveform evidence.');
  lines.push('');
  lines.push('## Summary');
  lines.push('');
  lines.push(`- runs: ${summary.count}`);
  lines.push(`- verdicts: ${Object.entries(summary.verdicts).map(([k, v]) => `${k}=${v}`).join(', ')}`);
  lines.push(`- max motor step mean/max: ${summary.fullMotorStepMaxDeg.mean} / ${summary.fullMotorStepMaxDeg.max} deg`);
  lines.push(`- cross-axis peak mean/max: ${summary.poseCrossHpPeak.mean} / ${summary.poseCrossHpPeak.max}`);
  lines.push(`- rxDrop/s mean/max: ${summary.rxDropPerS.mean} / ${summary.rxDropPerS.max}`);
  lines.push('');
  lines.push('| run | verdict | max step | p99 step | cross peak | worst | takeoff | landing | rxDrop/s | ef |');
  lines.push('|---|---:|---:|---:|---:|---|---:|---:|---:|---:|');
  for (const r of summary.rows) {
    lines.push(`| ${r.file} | ${r.verdict} | ${r.fullStep ?? ''} | ${r.p99 ?? ''} | ${r.cross ?? ''} | ${r.worstMotor ?? ''} | ${r.takeoffStep ?? ''} | ${r.landingStep ?? ''} | ${r.rxDropPerS ?? ''} | ${r.efOr ?? ''} |`);
  }
  lines.push('');
  lines.push('## Failures');
  lines.push('');
  for (const r of results) {
    lines.push(`### ${r.file}`);
    lines.push(`- verdict: ${r.quality.verdict}`);
    for (const f of r.quality.failures) lines.push(`- ${f}`);
    const takeoffStep = r.quality.takeoff?.motorStepMaxDeg;
    const landingStep = r.quality.landing?.motorStepMaxDeg;
    lines.push(`- takeoff: max step ${takeoffStep == null ? 'n/a' : `${takeoffStep}deg`} (${r.quality.takeoff?.worstMotor ?? 'n/a'}), cross peak ${r.quality.takeoff?.crossHpPeak ?? 'n/a'} (${r.quality.takeoff?.worstCrossAxis ?? 'n/a'})`);
    lines.push(`- landing: max step ${landingStep == null ? 'n/a' : `${landingStep}deg`} (${r.quality.landing?.worstMotor ?? 'n/a'}), cross peak ${r.quality.landing?.crossHpPeak ?? 'n/a'} (${r.quality.landing?.worstCrossAxis ?? 'n/a'})`);
    for (const e of (r.topEvents?.motorSteps || []).slice(0, 3)) {
      lines.push(`- event ${e.tS}s ${e.phase} ${e.motor}: ${e.stepDeg}deg over ${e.dtMs}ms; pose ${e.before.pose.X},${e.before.pose.Y},${e.before.pose.Z},${e.before.pose.Roll},${e.before.pose.Pitch},${e.before.pose.Yaw} -> ${e.after.pose.X},${e.after.pose.Y},${e.after.pose.Z},${e.after.pose.Roll},${e.after.pose.Pitch},${e.after.pose.Yaw}; CAN ef=${e.health.canEf} tx=${e.health.canTx} rxDrop=${e.health.rxDrop}`);
    }
    lines.push('');
  }
  return lines.join('\n');
}

function main() {
  const outIdx = process.argv.indexOf('--out');
  const outFile = outIdx >= 0 ? process.argv[outIdx + 1] : null;
  const mdIdx = process.argv.indexOf('--md-out');
  const mdFile = mdIdx >= 0 ? process.argv[mdIdx + 1] : null;
  const files = process.argv.slice(2).filter((x, i, arr) => x !== '--out' && arr[i - 1] !== '--out' && x !== '--md-out' && arr[i - 1] !== '--md-out');
  if (!files.length) { usage(); process.exit(1); }
  const result = files.map(analyze);
  const body = JSON.stringify(result.length === 1 ? result[0] : result, null, 2);
  if (outFile) fs.writeFileSync(outFile, body);
  if (mdFile) fs.writeFileSync(mdFile, markdownReport(result));
  console.log(body);
}

if (require.main === module) main();
else module.exports = { analyze, summarizeRuns, markdownReport };
