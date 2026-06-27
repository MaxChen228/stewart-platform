#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { solveFK, resetFK, NEUTRAL_Z } = require('./kin');

const AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];

function usage() {
  console.log(`Usage:
  node sysid/heave_step_metrics.js sysid/data/*heave-step*.jsonl [...]

Computes local heave-step metrics from a clean research_trial recording:
baseline is the window before the final P target command, and metrics only use
the main target/settle window so safe landing/release do not contaminate data.
`);
}

function round(x, d = 4) {
  return Number.isFinite(x) ? Number(x.toFixed(d)) : null;
}

function parsePoseCommand(cmd) {
  const parts = String(cmd).trim().split(/\s+/);
  if (parts[0] !== 'P') return null;
  const pose = parts.slice(1, 7).map(Number);
  const ms = Number(parts[7]);
  if (pose.length !== 6 || pose.some((x) => !Number.isFinite(x))) return null;
  return { pose, ms: Number.isFinite(ms) ? ms : null };
}

function stats(xs) {
  if (!xs.length) return {};
  const min = Math.min(...xs);
  const max = Math.max(...xs);
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  const rms = Math.sqrt(xs.reduce((a, b) => a + b * b, 0) / xs.length);
  return {
    min: round(min),
    max: round(max),
    p2p: round(max - min),
    mean: round(mean),
    rms: round(rms),
    peak: round(Math.max(Math.abs(min), Math.abs(max))),
  };
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
      const parsed = parsePoseCommand(rec.d);
      cmds.push({ t: rec.t, cmd: rec.d, parsed });
      continue;
    }
    if (rec.dir !== 'in') continue;
    let d;
    try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
    if (!d || !Array.isArray(d.a)) continue;
    const fk = solveFK(d.a);
    tele.push({ t: rec.t, raw: d, pose: fk.pose, fk });
  }
  return { cmds, tele };
}

function summarize(file) {
  const { cmds, tele } = load(file);
  const poseCmds = cmds.filter((x) => x.parsed);
  const targetCmd = poseCmds[poseCmds.length - 1];
  const homeCmd = poseCmds[poseCmds.length - 2] || null;
  if (!targetCmd) throw new Error(`${file}: no P target command found`);

  const moveMs = targetCmd.parsed.ms || 5000;
  const t0 = targetCmd.t;
  const baseline = tele.filter((x) => x.t > t0 - 700 && x.t < t0 - 100);
  const main = tele.filter((x) => x.t >= t0 && x.t <= t0 + moveMs + 1800);
  const finalWin = tele.filter((x) => x.t > t0 + moveMs + 1100 && x.t < t0 + moveMs + 1800);
  if (!baseline.length || !main.length) throw new Error(`${file}: not enough samples around target command`);

  const base = AXES.map((_, i) => baseline.reduce((a, s) => a + s.pose[i], 0) / baseline.length);
  const rel = AXES.map((_, i) => main.map((s) => s.pose[i] - base[i]));
  const final = AXES.map((_, i) => finalWin.length
    ? finalWin.reduce((a, s) => a + s.pose[i], 0) / finalWin.length - base[i]
    : null);
  const axisStats = Object.fromEntries(AXES.map((axis, i) => [axis, stats(rel[i])]));
  const peaks = AXES.map((axis) => axisStats[axis].peak || 0);
  const crossPeak = Math.max(peaks[0], peaks[1], peaks[3], peaks[4], peaks[5]);
  const zPeak = peaks[2];

  let health = {};
  const summaryPath = file.replace(/\.jsonl$/, '.summary.json');
  try {
    const summary = JSON.parse(fs.readFileSync(summaryPath, 'utf8'));
    health = {
      canEfOr: summary.canHealth?.efOr,
      txMax: summary.canHealth?.txMax,
      rxDropSum: summary.canHealth?.rxDropSum,
      hmaxMax: summary.hold?.hmaxMax,
      herrP95: summary.hold?.herrAbsP95,
      okFailFrac: summary.encoders?.failFrac,
    };
  } catch {}

  return {
    file: path.basename(file),
    path: file,
    samples: tele.length,
    targetCommand: targetCmd.cmd,
    homeCommand: homeCmd?.cmd || null,
    windows: {
      baselineSamples: baseline.length,
      mainSamples: main.length,
      finalSamples: finalWin.length,
    },
    baseline: Object.fromEntries(AXES.map((axis, i) => [axis, round(base[i])])),
    final: Object.fromEntries(AXES.map((axis, i) => [axis, round(final[i])])),
    absolute: {
      commandedHomeZ: round(homeCmd?.parsed?.pose?.[2]),
      actualHomeZ: round(base[2]),
      homeErrorVsCommand: round(Number.isFinite(homeCmd?.parsed?.pose?.[2]) ? base[2] - homeCmd.parsed.pose[2] : NaN),
      commandedTargetZ: round(targetCmd.parsed.pose[2]),
      actualFinalZ: round(Number.isFinite(final[2]) ? base[2] + final[2] : NaN),
      finalErrorVsCommand: round(Number.isFinite(final[2]) ? base[2] + final[2] - targetCmd.parsed.pose[2] : NaN),
      finalErrorVsNominal143: round(Number.isFinite(final[2]) ? base[2] + final[2] - 143 : NaN),
      actualStepZ: round(final[2]),
    },
    axisStats,
    coupling: {
      zPeak: round(zPeak),
      crossPeak: round(crossPeak),
      crossOverZ: zPeak > 1e-9 ? round(crossPeak / zPeak) : null,
    },
    health,
  };
}

function main() {
  const files = process.argv.slice(2);
  if (!files.length) { usage(); process.exit(1); }
  const out = files.map(summarize);
  console.log(JSON.stringify(out.length === 1 ? out[0] : out, null, 2));
}

if (require.main === module) main();
else module.exports = { summarize };
