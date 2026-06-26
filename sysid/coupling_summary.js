#!/usr/bin/env node
// Summarize coupling-probe recordings using shared FK.

const fs = require('fs');
const path = require('path');
const { solveFK, resetFK, NEUTRAL_Z } = require('./kin');

const AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];

function usage() {
  console.log(`Usage:
  node sysid/coupling_summary.js sysid/data/coupling_probe_*.jsonl
`);
}

function parsePoseCommand(cmd) {
  const parts = String(cmd).trim().split(/\s+/);
  if (parts[0] !== 'PF' && parts[0] !== 'P') return null;
  const pose = parts.slice(1, 7).map(Number);
  if (pose.length !== 6 || pose.some((x) => !Number.isFinite(x))) return null;
  return pose;
}

function load(file) {
  const tele = [];
  const targets = [];
  const cmds = [];
  let meta = null;
  resetFK([0, 0, NEUTRAL_Z, 0, 0, 0]);

  for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
    if (!line.trim()) continue;
    let rec;
    try { rec = JSON.parse(line); } catch { continue; }
    if (rec.meta) { meta = rec.meta; continue; }
    if (rec.dir === 'cmd') {
      cmds.push({ t: rec.t, cmd: rec.d });
      const pose = parsePoseCommand(rec.d);
      if (pose) targets.push({ t: rec.t, pose, cmd: rec.d.split(/\s+/)[0] });
      continue;
    }
    if (rec.dir !== 'in') continue;
    let d;
    try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
    if (!d || !Array.isArray(d.a)) continue;
    const fk = solveFK(d.a);
    tele.push({ t: rec.t, raw: d, pose: fk.pose, fk });
  }
  return { meta, tele, targets, cmds };
}

function avg(xs) {
  return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : null;
}

function rms(xs) {
  return xs.length ? Math.sqrt(avg(xs.map((x) => x * x))) : null;
}

function min(xs) {
  return xs.length ? Math.min(...xs) : null;
}

function max(xs) {
  return xs.length ? Math.max(...xs) : null;
}

function round(x, d = 3) {
  return Number.isFinite(x) ? Number(x.toFixed(d)) : null;
}

function nearestTarget(targets, t) {
  let last = null;
  for (const target of targets) {
    if (target.t > t) break;
    last = target;
  }
  return last;
}

function summarize(file) {
  const { meta, tele, targets, cmds } = load(file);
  const durationS = tele.length >= 2 ? (tele[tele.length - 1].t - tele[0].t) / 1000 : 0;
  const initial = tele[0]?.pose || [0, 0, NEUTRAL_Z, 0, 0, 0];
  const target0 = targets[0]?.pose || initial;
  const targetRange = AXES.map((_, i) => {
    const xs = targets.map((x) => x.pose[i]);
    return (max(xs) ?? target0[i]) - (min(xs) ?? target0[i]);
  });
  const commandedIdx = targetRange.reduce((best, v, i) => Math.abs(v) > Math.abs(targetRange[best]) ? i : best, 0);

  const poseStats = AXES.map((axis, i) => {
    const xs = tele.map((x) => x.pose[i]);
    const rel = xs.map((x) => x - initial[i]);
    return {
      axis,
      min: round(min(xs)),
      max: round(max(xs)),
      p2p: round((max(xs) ?? 0) - (min(xs) ?? 0)),
      peakFromInitial: round(Math.max(...rel.map((x) => Math.abs(x)))),
      rmsFromInitial: round(rms(rel)),
    };
  });

  const errors = AXES.map(() => []);
  for (const sample of tele) {
    const target = nearestTarget(targets, sample.t);
    if (!target) continue;
    for (let i = 0; i < 6; i++) errors[i].push(target.pose[i] - sample.pose[i]);
  }
  const errorStats = AXES.map((axis, i) => ({
    axis,
    rms: round(rms(errors[i])),
    peak: round(Math.max(0, ...errors[i].map((x) => Math.abs(x)))),
  }));

  const commandPeak = poseStats[commandedIdx]?.peakFromInitial || 0;
  const crossPeak = Math.max(0, ...poseStats.filter((_, i) => i !== commandedIdx).map((s) => s.peakFromInitial || 0));

  return {
    file: path.basename(file),
    meta,
    samples: tele.length,
    durationS: round(durationS, 2),
    commands: cmds.map((x) => x.cmd),
    targetSamples: targets.length,
    commandedAxis: AXES[commandedIdx],
    targetRange: Object.fromEntries(AXES.map((axis, i) => [axis, round(targetRange[i])])),
    coupling: {
      commandPeak: round(commandPeak),
      crossPeak: round(crossPeak),
      crossOverCommand: commandPeak > 1e-9 ? round(crossPeak / commandPeak, 4) : null,
    },
    poseStats,
    errorStats,
    fk: {
      failCount: tele.filter((x) => !x.fk.converged).length,
      residualMax: round(max(tele.map((x) => x.fk.residual))),
    },
  };
}

function main() {
  const files = process.argv.slice(2);
  if (!files.length) { usage(); process.exit(1); }
  const out = files.map(summarize);
  console.log(JSON.stringify(out.length === 1 ? out[0] : out, null, 2));
}

main();
