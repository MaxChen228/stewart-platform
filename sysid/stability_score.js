#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { solveFK, resetFK, NEUTRAL_Z } = require('./kin');

const AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];
const LINEAR = new Set(['x', 'y', 'z']);

function usage() {
  console.log(`Usage:
  node sysid/stability_score.js <record.jsonl> [--axis z] [--out file.json]

Trajectory-agnostic stability scorer.

Principles:
  - Reconstruct commanded P/PF targets when available.
  - Separate normal slow trajectory motion from oscillation using high-pass
    residuals, so "moving upward" is not misread as wobble.
  - Score tracking, cross-axis coupling, residual oscillation, disturbance
    recovery, and data quality separately before producing one stability score.
`);
}

function arg(name, fallback = null) {
  const i = process.argv.indexOf(name);
  return i >= 0 ? process.argv[i + 1] : fallback;
}

function round(x, d = 4) {
  return Number.isFinite(x) ? Number(x.toFixed(d)) : null;
}

function mean(xs) {
  return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : NaN;
}

function rms(xs) {
  return xs.length ? Math.sqrt(mean(xs.map((x) => x * x))) : NaN;
}

function percentile(xs, p) {
  if (!xs.length) return NaN;
  const s = [...xs].sort((a, b) => a - b);
  const i = Math.max(0, Math.min(s.length - 1, Math.ceil(p * s.length) - 1));
  return s[i];
}

function parsePoseCommand(cmd) {
  const parts = String(cmd || '').trim().split(/\s+/);
  if (parts[0] !== 'P' && parts[0] !== 'PF') return null;
  const pose = parts.slice(1, 7).map(Number);
  if (pose.length !== 6 || pose.some((x) => !Number.isFinite(x))) return null;
  return { kind: parts[0], pose, ms: Number(parts[7]) || (parts[0] === 'P' ? 1500 : 0) };
}

function parseWCommand(cmd) {
  const parts = String(cmd || '').trim().split(/\s+/);
  if (parts[0] !== 'W') return null;
  const deg = parts.slice(1, 7).map(Number);
  const ms = Number(parts[7]) || 0;
  if (deg.length !== 6 || deg.some((x) => !Number.isFinite(x))) return null;
  return { deg, ms };
}

function readJson(file) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); }
  catch { return null; }
}

function companionManifest(file) {
  const bundle = readJson(file.replace(/\.jsonl$/, '.bundle.json'));
  if (bundle?.manifest && fs.existsSync(bundle.manifest)) return readJson(bundle.manifest);
  const dir = path.dirname(file);
  const stem = path.basename(file).replace(/_heave-step_.*$/, '');
  const candidates = fs.readdirSync(dir)
    .filter((x) => x.startsWith(stem) && x.endsWith('.manifest.json'))
    .sort()
    .map((x) => path.join(dir, x));
  return candidates.length ? readJson(candidates[candidates.length - 1]) : null;
}

function smoothstep(u) {
  const x = Math.max(0, Math.min(1, u));
  return x * x * (3 - 2 * x);
}

function load(file) {
  const cmds = [];
  const tele = [];
  resetFK([0, 0, NEUTRAL_Z, 0, 0, 0]);

  for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
    if (!line.trim()) continue;
    let rec;
    try { rec = JSON.parse(line); } catch { continue; }
    if (rec.dir === 'cmd') {
      const cmd = String(rec.d || '').trim();
      cmds.push({ t: Number(rec.t), cmd, pose: parsePoseCommand(cmd), disturb: parseWCommand(cmd) });
      continue;
    }
    if (rec.dir !== 'in') continue;
    let d;
    try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
    if (!d || !Array.isArray(d.a)) continue;
    const fk = solveFK(d.a);
    tele.push({ t: Number(rec.t), raw: d, pose: fk.pose, fk });
  }
  tele.sort((a, b) => a.t - b.t);
  cmds.sort((a, b) => a.t - b.t);
  return { cmds, tele };
}

function applyReferenceCompensation(pose, referenceComp) {
  const out = [...pose];
  if (Number.isFinite(referenceComp.zBias)) out[2] -= referenceComp.zBias;
  return out;
}

function buildTargetModel(cmds, tele, referenceComp = {}) {
  const poseCmds = cmds.filter((x) => x.pose);
  const segments = [];
  let lastTarget = null;
  let lastReference = null;

  for (const c of poseCmds) {
    const referencePose = applyReferenceCompensation(c.pose.pose, referenceComp);
    if (c.pose.kind === 'PF') {
      segments.push({ kind: 'hold', t0: c.t, t1: Infinity, pose: c.pose.pose, referencePose });
      lastTarget = c.pose.pose;
      lastReference = referencePose;
      continue;
    }
    const prev = lastTarget || nearestPoseBefore(tele, c.t) || c.pose.pose;
    const refPrev = lastReference || applyReferenceCompensation(prev, referenceComp);
    const t1 = c.t + c.pose.ms;
    segments.push({
      kind: 'smoothstep',
      t0: c.t,
      t1,
      from: prev,
      to: c.pose.pose,
      refFrom: refPrev,
      refTo: referencePose,
      ms: c.pose.ms,
    });
    segments.push({ kind: 'hold', t0: t1, t1: Infinity, pose: c.pose.pose, referencePose });
    lastTarget = c.pose.pose;
    lastReference = referencePose;
  }

  segments.sort((a, b) => a.t0 - b.t0);
  for (let i = 0; i < segments.length - 1; i++) segments[i].t1 = Math.min(segments[i].t1, segments[i + 1].t0);

  function commandTargetAt(t) {
    let seg = null;
    for (const s of segments) {
      if (s.t0 <= t && t <= s.t1) seg = s;
      if (s.t0 > t) break;
    }
    if (!seg) return null;
    if (seg.kind === 'hold') return seg.pose;
    const u = smoothstep((t - seg.t0) / Math.max(1, seg.ms));
    return seg.from.map((v, i) => v + (seg.to[i] - v) * u);
  }

  function referenceTargetAt(t) {
    let seg = null;
    for (const s of segments) {
      if (s.t0 <= t && t <= s.t1) seg = s;
      if (s.t0 > t) break;
    }
    if (!seg) return null;
    if (seg.kind === 'hold') return seg.referencePose || seg.pose;
    const u = smoothstep((t - seg.t0) / Math.max(1, seg.ms));
    const from = seg.refFrom || seg.from;
    const to = seg.refTo || seg.to;
    return from.map((v, i) => v + (to[i] - v) * u);
  }

  if (!segments.length && tele.length) {
    const pose = tele[0].pose;
    segments.push({ kind: 'hold', t0: tele[0].t, t1: Infinity, pose, referencePose: pose });
  }

  return { segments, commandTargetAt, referenceTargetAt, referenceComp };
}

function nearestPoseBefore(tele, t) {
  let last = null;
  for (const row of tele) {
    if (row.t > t) break;
    last = row.pose;
  }
  return last;
}

function movingAverage(values, times, windowMs) {
  const out = [];
  let j0 = 0;
  let sum = 0;
  for (let i = 0; i < values.length; i++) {
    sum += values[i];
    while (times[i] - times[j0] > windowMs) sum -= values[j0++];
    out.push(sum / Math.max(1, i - j0 + 1));
  }
  return out;
}

function dominantFreq(values, times) {
  if (values.length < 12) return null;
  const durationS = (times[times.length - 1] - times[0]) / 1000;
  if (durationS <= 0) return null;
  const avgDt = durationS / (values.length - 1);
  const nyq = 0.5 / avgDt;
  let best = { hz: 0, amp: 0 };
  for (let hz = 0.2; hz <= Math.min(12, nyq); hz += 0.1) {
    let c = 0, s = 0;
    for (let i = 0; i < values.length; i++) {
      const t = (times[i] - times[0]) / 1000;
      c += values[i] * Math.cos(2 * Math.PI * hz * t);
      s += values[i] * Math.sin(2 * Math.PI * hz * t);
    }
    const amp = Math.sqrt(c * c + s * s) / values.length;
    if (amp > best.amp) best = { hz, amp };
  }
  return { hz: round(best.hz, 2), amp: round(best.amp) };
}

function axisStats(samples, axisIndex) {
  const values = samples.map((s) => s.pose[axisIndex]);
  const targets = samples.map((s) => s.target?.[axisIndex]).filter(Number.isFinite);
  const errors = samples.map((s) => s.err?.[axisIndex]).filter(Number.isFinite);
  const times = samples.map((s) => s.t);
  const hpBase = movingAverage(values, times, 700);
  const hp = values.map((v, i) => v - hpBase[i]);
  const absErr = errors.map(Math.abs);
  return {
    min: round(Math.min(...values)),
    max: round(Math.max(...values)),
    p2p: round(Math.max(...values) - Math.min(...values)),
    targetRange: targets.length ? round(Math.max(...targets) - Math.min(...targets)) : null,
    errRms: round(rms(errors)),
    errP95: round(percentile(absErr, 0.95)),
    errPeak: round(Math.max(0, ...absErr)),
    hpRms: round(rms(hp)),
    hpP95: round(percentile(hp.map(Math.abs), 0.95)),
    hpPeak: round(Math.max(0, ...hp.map(Math.abs))),
    dominant: dominantFreq(hp, times),
  };
}

function splitSamples(tele, targetModel) {
  return tele.map((row) => {
    const commandTarget = targetModel.commandTargetAt(row.t);
    const target = targetModel.referenceTargetAt(row.t);
    const err = target ? row.pose.map((v, i) => v - target[i]) : null;
    return { ...row, commandTarget, target, err };
  });
}

function inferCommandedAxes(samples, requestedAxis) {
  if (requestedAxis && AXES.includes(requestedAxis)) return [requestedAxis];
  const ranges = AXES.map((axis, i) => {
    const ts = samples.map((s) => s.target?.[i]).filter(Number.isFinite);
    return ts.length ? Math.max(...ts) - Math.min(...ts) : 0;
  });
  const threshold = 0.5;
  const axes = AXES.filter((_, i) => Math.abs(ranges[i]) > threshold);
  return axes.length ? axes : ['z'];
}

function quality(tele) {
  const okFrac = tele.length ? tele.filter((x) => x.raw.ok === 6).length / tele.length : 0;
  const fkFailFrac = tele.length ? tele.filter((x) => !x.fk.converged).length / tele.length : 1;
  const dts = [];
  for (let i = 1; i < tele.length; i++) dts.push(tele[i].t - tele[i - 1].t);
  const latest = tele[tele.length - 1]?.raw || {};
  const rxDropSum = tele.reduce((s, x) => s + (x.raw.can?.rxDrop || 0), 0);
  const txFailSum = tele.reduce((s, x) => s + (x.raw.can?.txFail || 0), 0);
  const efOr = tele.reduce((s, x) => s | (x.raw.can?.ef || x.raw.ef || 0), 0);
  const durationS = tele.length >= 2 ? (tele[tele.length - 1].t - tele[0].t) / 1000 : 0;
  return {
    samples: tele.length,
    durationS: round(durationS, 3),
    okFrac: round(okFrac, 4),
    fkFailFrac: round(fkFailFrac, 4),
    hostDtP95Ms: round(percentile(dts, 0.95), 2),
    hostDtMaxMs: round(Math.max(0, ...dts), 2),
    can: {
      backend: latest.can?.backend,
      bitrate: latest.can?.bitrate,
      efOr,
      txMax: Math.max(0, ...tele.map((x) => x.raw.can?.tx || x.raw.tx || 0)),
      rxDropSum,
      rxDropPerS: durationS > 0 ? round(rxDropSum / durationS, 3) : 0,
      txFailSum,
      txFailPerS: durationS > 0 ? round(txFailSum / durationS, 3) : 0,
    },
  };
}

function disturbanceRecovery(samples, cmds) {
  const events = cmds.filter((x) => x.disturb);
  if (!events.length) return { events: 0 };
  const out = [];
  for (const e of events) {
    const before = samples.filter((s) => s.t >= e.t - 500 && s.t < e.t);
    const base = AXES.map((_, i) => mean(before.map((s) => s.pose[i])));
    const after = samples.filter((s) => s.t >= e.t && s.t <= e.t + 3000);
    const mag = after.map((s) => Math.sqrt(AXES.reduce((sum, _, i) => {
      const scale = LINEAR.has(AXES[i]) ? 1 : 3;
      return sum + ((s.pose[i] - base[i]) / scale) ** 2;
    }, 0)));
    const peak = Math.max(0, ...mag);
    const threshold = Math.max(0.25, peak * 0.15);
    const recovered = after.find((s, idx) => idx > 2 && mag[idx] < threshold);
    out.push({ t: round(e.t), peak: round(peak), recoverMs: recovered ? round(recovered.t - e.t, 1) : null });
  }
  return {
    events: events.length,
    peakMax: round(Math.max(...out.map((x) => x.peak))),
    recoverMsP95: round(percentile(out.map((x) => x.recoverMs).filter(Number.isFinite), 0.95), 1),
    items: out,
  };
}

function score(file, opts = {}) {
  const { cmds, tele } = load(file);
  if (!tele.length) throw new Error(`${file}: no telemetry`);
  const manifest = companionManifest(file);
  const referenceComp = {
    zBias: opts.zBias != null && Number.isFinite(Number(opts.zBias))
      ? Number(opts.zBias)
      : Number(manifest?.condition?.zBias || 0),
  };
  const targetModel = buildTargetModel(cmds, tele, referenceComp);
  const samples = splitSamples(tele, targetModel);
  const commandedAxes = inferCommandedAxes(samples, opts.axis);
  const commanded = new Set(commandedAxes);
  const stats = Object.fromEntries(AXES.map((axis, i) => [axis, axisStats(samples, i)]));
  const q = quality(tele);
  const recovery = disturbanceRecovery(samples, cmds);

  const commandedErr = commandedAxes.map((axis) => stats[axis].errP95).filter(Number.isFinite);
  const crossAxes = AXES.filter((axis) => !commanded.has(axis));
  const crossHp = crossAxes.map((axis) => stats[axis].hpP95).filter(Number.isFinite);
  const crossPeak = crossAxes.map((axis) => stats[axis].hpPeak).filter(Number.isFinite);
  const oscillHp = AXES.map((axis) => stats[axis].hpP95).filter(Number.isFinite);

  const trackingCost = mean(commandedErr) || 0;
  const crossCost = mean(crossHp) || 0;
  const oscillCost = mean(oscillHp) || 0;
  const recoveryCost = recovery.recoverMsP95 ? recovery.recoverMsP95 / 1000 : 0;
  const qualityPenalty =
    (1 - (q.okFrac || 0)) * 30 +
    (q.fkFailFrac || 0) * 30 +
    Math.min(20, (q.can.rxDropPerS || 0) * 0.8) +
    Math.min(20, (q.can.txFailPerS || 0) * 5) +
    (q.can.efOr ? 2 : 0);
  const cost = trackingCost * 1.2 + crossCost * 2.5 + oscillCost * 1.5 + recoveryCost * 1.0 + qualityPenalty;
  const stabilityScore = Math.max(0, 100 - cost);

  return {
    file: path.basename(file),
    path: file,
    commandedAxes,
    score: {
      stability: round(stabilityScore, 2),
      cost: round(cost, 3),
      trackingCost: round(trackingCost),
      crossAxisCost: round(crossCost),
      oscillationCost: round(oscillCost),
      recoveryCost: round(recoveryCost),
      qualityPenalty: round(qualityPenalty),
    },
    targetModel: {
      commandCount: cmds.length,
      poseCommandCount: cmds.filter((x) => x.pose).length,
      disturbanceCount: cmds.filter((x) => x.disturb).length,
      referenceComp,
      segments: targetModel.segments.map((s) => ({
        kind: s.kind,
        t0: round(s.t0, 1),
        t1: Number.isFinite(s.t1) ? round(s.t1, 1) : null,
        commandPose: s.pose || s.to,
        referencePose: s.referencePose || s.refTo || s.pose || s.to,
      })),
    },
    axes: stats,
    crossAxis: {
      axes: crossAxes,
      hpP95Mean: round(mean(crossHp)),
      hpPeakMax: round(Math.max(0, ...crossPeak)),
    },
    recovery,
    quality: q,
  };
}

function main() {
  const file = process.argv.find((x, i) => i > 1 && !x.startsWith('--') && process.argv[i - 1] !== '--out');
  if (!file) { usage(); process.exit(1); }
  const result = score(file, { axis: arg('--axis'), zBias: arg('--z-bias') });
  const out = arg('--out');
  if (out) fs.writeFileSync(out, JSON.stringify(result, null, 2));
  console.log(JSON.stringify(result, null, 2));
}

if (require.main === module) main();
else module.exports = { score };
