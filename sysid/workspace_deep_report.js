#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { score, load, cropToWindow, buildTargetModel, companionManifest, AXES } = require('./stability_score');
const { summarize } = require('./telemetry_summary');

const ROOT = path.resolve(__dirname, '..');
const DATA_DIR = path.join(ROOT, 'sysid', 'data');
const OUT_DIR = path.join(DATA_DIR, 'reports');

function arg(name, fallback = null) {
  const i = process.argv.indexOf(name);
  return i >= 0 ? process.argv[i + 1] : fallback;
}

function has(name) {
  return process.argv.includes(name);
}

function round(v, d = 3) {
  const n = Number(v);
  return Number.isFinite(n) ? Number(n.toFixed(d)) : null;
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
  const idx = Math.max(0, Math.min(s.length - 1, Math.ceil(p * s.length) - 1));
  return s[idx];
}

function range(xs) {
  const values = xs.filter(Number.isFinite);
  if (!values.length) return { min: null, max: null, p2p: null };
  const min = Math.min(...values);
  const max = Math.max(...values);
  return { min: round(min), max: round(max), p2p: round(max - min) };
}

function dominantFreq(values, times) {
  if (values.length < 18) return null;
  const durationS = (times[times.length - 1] - times[0]) / 1000;
  if (!(durationS > 0)) return null;
  const avgDt = durationS / Math.max(1, values.length - 1);
  const nyq = 0.5 / avgDt;
  let best = { hz: 0, amp: 0 };
  for (let hz = 0.2; hz <= Math.min(8, nyq); hz += 0.1) {
    let c = 0;
    let s = 0;
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

function runFiles() {
  return fs.readdirSync(DATA_DIR)
    .filter((x) => x.endsWith('.jsonl'))
    .map((x) => path.join(DATA_DIR, x))
    .filter((file) => fs.statSync(file).isFile())
    .sort();
}

function conditionLabel(c) {
  if (!c) return 'default';
  const parts = [];
  if (Number(c.zBias)) parts.push(`zBias=${Number(c.zBias)}`);
  if (Number.isFinite(Number(c.vmaxT))) parts.push(`vT=${Number(c.vmaxT)}`);
  if (Number.isFinite(Number(c.vmaxR))) parts.push(`vR=${Number(c.vmaxR)}`);
  return parts.join(' ') || 'default';
}

function phaseInfo(events) {
  const phases = events
    .filter((e) => e.type === 'session_phase')
    .map((e) => ({ t: round(e.t, 1), phase: e.phase }));
  const names = phases.map((x) => x.phase);
  const required = ['recording', 'landing', 'release'];
  return {
    phases,
    completeLifecycle: required.every((p) => names.includes(p)),
    hasRecordingWindow: names.includes('recording') && names.includes('landing'),
  };
}

function telemetryHealth(tele) {
  const ok = tele.map((x) => Number(x.raw?.ok)).filter(Number.isFinite);
  const ef = tele.map((x) => Number(x.raw?.can?.ef ?? x.raw?.ef)).filter(Number.isFinite);
  const rxDrop = tele.map((x) => Number(x.raw?.can?.rxDrop)).filter(Number.isFinite);
  const txFail = tele.map((x) => Number(x.raw?.can?.txFail)).filter(Number.isFinite);
  const dts = [];
  for (let i = 1; i < tele.length; i++) dts.push(tele[i].t - tele[i - 1].t);
  return {
    samples: tele.length,
    okFailFrac: ok.length ? round(ok.filter((x) => x < 6).length / ok.length, 4) : null,
    okFailCount: ok.filter((x) => x < 6).length,
    efOr: ef.reduce((s, x) => s | (x & 0xff), 0),
    rxDropDelta: counterDelta(rxDrop),
    txFailDelta: counterDelta(txFail),
    hostDtP99Ms: round(percentile(dts, 0.99), 2),
    hostDtMaxMs: round(Math.max(0, ...dts), 2),
  };
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

function axisAnalysis(scoped, targetModel, commandedAxes) {
  const commanded = new Set(commandedAxes || []);
  const samples = scoped.tele.map((row) => {
    const target = targetModel.referenceTargetAt(row.t);
    const err = target ? row.pose.map((v, i) => v - target[i]) : null;
    return { ...row, target, err };
  });
  const times = samples.map((s) => s.t);
  return AXES.map((axis, i) => {
    const actual = samples.map((s) => s.pose[i]).filter(Number.isFinite);
    const target = samples.map((s) => s.target?.[i]).filter(Number.isFinite);
    const err = samples.map((s) => s.err?.[i]).filter(Number.isFinite);
    const base = movingAverage(actual, times, 700);
    const hp = actual.map((v, idx) => v - base[idx]);
    return {
      axis,
      commanded: commanded.has(axis),
      actual: range(actual),
      target: range(target),
      errRms: round(rms(err)),
      errP95: round(percentile(err.map(Math.abs), 0.95)),
      errPeak: round(Math.max(0, ...err.map(Math.abs))),
      hpRms: round(rms(hp)),
      hpP95: round(percentile(hp.map(Math.abs), 0.95)),
      hpPeak: round(Math.max(0, ...hp.map(Math.abs))),
      dominant: dominantFreq(hp, times),
    };
  });
}

function top(items, key, n = 3) {
  return [...items]
    .filter((x) => Number.isFinite(Number(x[key])))
    .sort((a, b) => Number(b[key]) - Number(a[key]))
    .slice(0, n);
}

function visualReading(run) {
  const axes = run.axes;
  const commanded = axes.filter((a) => a.commanded);
  const cross = axes.filter((a) => !a.commanded);
  const worstTrack = top(commanded, 'errP95', 2);
  const worstCross = top(cross, 'hpPeak', 3);
  const worstOsc = top(axes, 'hpP95', 3);
  const notes = [];
  if (worstTrack.length) {
    notes.push(`tracking主要卡在 ${worstTrack.map((a) => `${a.axis} P95=${a.errP95}`).join(', ')}`);
  }
  if (worstCross.length && Number(worstCross[0].hpPeak) > 1.5) {
    notes.push(`cross-axis峰值集中在 ${worstCross.map((a) => `${a.axis} peak=${a.hpPeak}`).join(', ')}`);
  } else {
    notes.push('cross-axis波形峰值低，耦合不是主矛盾');
  }
  if (worstOsc.length) {
    const osc = worstOsc.map((a) => `${a.axis} hpP95=${a.hpP95}${a.dominant?.hz ? ` @${a.dominant.hz}Hz` : ''}`).join(', ');
    notes.push(`高頻殘餘主要看 ${osc}`);
  }
  if (run.health.scoreWindow.okFailCount > 0) notes.push(`評分窗口內 encoder ok<6 ${run.health.scoreWindow.okFailCount} 次，這組控制判讀需降權`);
  else if (run.health.full.okFailCount > 0) notes.push(`評分窗口 ok=6；全流程其他段落有 ${run.health.full.okFailCount} 次 ok<6，主要作 lifecycle 風險觀察`);
  if (run.summary.canHealth.efOr || run.summary.canHealth.rxDropSum || run.summary.canHealth.txFailSum) notes.push('CAN健康有事件，需先排資料面再解讀控制效果');
  if (run.summary.hostDtMs.p99 > 140) notes.push(`host telemetry p99=${run.summary.hostDtMs.p99}ms 偏大，波形時間解析度有限`);
  return notes;
}

function analyze(file) {
  const manifest = companionManifest(file);
  const loaded = load(file);
  const scoped = cropToWindow(loaded.cmds, loaded.tele, loaded.events);
  const result = score(file);
  const referenceComp = result.targetModel.referenceComp || {};
  const targetModel = buildTargetModel(scoped.cmds, scoped.tele, referenceComp);
  const summary = summarize(file);
  const axes = axisAnalysis(scoped, targetModel, result.commandedAxes);
  const lifecycle = phaseInfo(loaded.events);
  const fullDurationS = loaded.tele.length >= 2 ? (loaded.tele.at(-1).t - loaded.tele[0].t) / 1000 : 0;
  const scoreDurationS = scoped.tele.length >= 2 ? (scoped.tele.at(-1).t - scoped.tele[0].t) / 1000 : 0;
  const run = {
    id: path.basename(file, '.jsonl'),
    file: path.relative(ROOT, file),
    program: manifest?.program || summary.meta?.owner?.program || null,
    condition: manifest?.condition || summary.meta?.owner?.condition || null,
    score: result.score,
    commandedAxes: result.commandedAxes,
    lifecycle,
    durations: {
      fullS: round(fullDurationS, 2),
      scoreWindowS: round(scoreDurationS, 2),
      samplesFull: loaded.tele.length,
      samplesScoreWindow: scoped.tele.length,
    },
    summary: {
      hostDtMs: summary.hostDtMs,
      loopHz: summary.loopHz,
      canReadUs: summary.canReadUs,
      canHealth: summary.canHealth,
      encoders: summary.encoders,
      controller: summary.controller,
      hold: summary.hold,
    },
    health: {
      scoreWindow: telemetryHealth(scoped.tele),
      full: telemetryHealth(loaded.tele),
    },
    axes,
  };
  run.visualReading = visualReading(run);
  return run;
}

function groupByProgram(runs) {
  const groups = new Map();
  for (const run of runs) {
    const key = run.program?.hash || run.program?.id || '__unknown__';
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(run);
  }
  return [...groups.entries()].map(([key, items]) => ({
    key,
    label: items[0].program?.name || key,
    runs: items.sort((a, b) => Number(b.score.stability) - Number(a.score.stability)),
  })).sort((a, b) => a.label.localeCompare(b.label));
}

function statusMark(run) {
  if (!run.lifecycle.completeLifecycle) return 'INCOMPLETE';
  const h = run.health.scoreWindow;
  if (h.okFailCount > 0 || h.efOr || h.rxDropDelta || h.txFailDelta) return 'DATA-WATCH';
  return 'OK';
}

function markdown(report) {
  const lines = [];
  lines.push(`# Workspace Deep Report`);
  lines.push('');
  lines.push(`Generated: ${report.generatedAt}`);
  lines.push('');
  lines.push('## Program Summary');
  lines.push('');
  lines.push('| Program | Condition | Status | Stability | Track | Cross | Osc | Quality | Samples | Health |');
  lines.push('|---|---:|---|---:|---:|---:|---:|---:|---:|---|');
  for (const group of report.groups) {
    for (const run of group.runs) {
      const s = run.score;
      const h = [];
      if (run.health.scoreWindow.okFailCount) h.push(`window okFail ${run.health.scoreWindow.okFailCount}`);
      if (run.health.scoreWindow.efOr) h.push(`window ef ${run.health.scoreWindow.efOr}`);
      if (run.health.scoreWindow.rxDropDelta) h.push(`window rxDrop ${run.health.scoreWindow.rxDropDelta}`);
      if (!h.length && run.health.full.okFailCount) h.push(`window clean; lifecycle okFail ${run.health.full.okFailCount}`);
      if (!h.length) h.push('clean');
      lines.push(`| ${group.label} | ${conditionLabel(run.condition)} | ${statusMark(run)} | ${s.stability} | ${s.trackingCost} | ${s.crossAxisCost} | ${s.oscillationCost} | ${s.qualityPenalty} | ${run.durations.samplesScoreWindow}/${run.durations.samplesFull} | ${h.join(', ')} |`);
    }
  }
  lines.push('');
  lines.push('## Waveform Reading');
  for (const group of report.groups) {
    lines.push('');
    lines.push(`### ${group.label}`);
    for (const run of group.runs) {
      lines.push(`- \`${run.id}\` (${conditionLabel(run.condition)}): ${run.visualReading.join('；')}`);
      const axisLine = run.axes
        .filter((a) => a.commanded || a.hpPeak > 1.5 || a.errP95 > 3)
        .map((a) => `${a.axis}${a.commanded ? '*' : ''}: range ${a.actual.p2p}, errP95 ${a.errP95}, hpP95 ${a.hpP95}, hpPeak ${a.hpPeak}`)
        .join(' | ');
      if (axisLine) lines.push(`  - axes: ${axisLine}`);
    }
  }
  lines.push('');
  lines.push('## Engineering Read');
  lines.push('');
  lines.push(...report.conclusions.map((x) => `- ${x}`));
  lines.push('');
  return lines.join('\n');
}

function conclusions(groups) {
  const runs = groups.flatMap((g) => g.runs);
  const out = [];
  const dirty = runs.filter((r) => statusMark(r) !== 'OK');
  if (dirty.length) out.push(`${dirty.length} run(s) need data caution before control interpretation: ${dirty.map((r) => r.id).join(', ')}.`);
  const low = [...runs].sort((a, b) => a.score.stability - b.score.stability).slice(0, 3);
  out.push(`Lowest stability groups are ${low.map((r) => `${r.program?.name || r.id}=${r.score.stability}`).join(', ')}; optimize within those programs only.`);
  const zRuns = runs.filter((r) => r.commandedAxes.includes('z'));
  if (zRuns.length) {
    const zTrack = zRuns.map((r) => `${r.program?.name || r.id}: track=${r.score.trackingCost}, cross=${r.score.crossAxisCost}, osc=${r.score.oscillationCost}`);
    out.push(`Z-dominant runs remain tracking-limited more than cross-limited: ${zTrack.join('; ')}.`);
  }
  const clean = runs.every((r) => !r.summary.canHealth.efOr && !r.summary.canHealth.rxDropSum && !r.summary.canHealth.txFailSum);
  if (clean) out.push('CAN error counters were clean in active successful runs; current comparisons can focus on trajectory/compensation, not bus faults.');
  out.push('Next experiment should test a small same-program condition matrix and inspect waveform overlays after each run, not rank by scalar score alone.');
  return out;
}

function main() {
  const filter = arg('--program');
  let files = runFiles();
  if (filter) files = files.filter((file) => file.includes(filter));
  if (!files.length) throw new Error('no active JSONL runs found');
  fs.mkdirSync(OUT_DIR, { recursive: true });
  const runs = files.map(analyze);
  const groups = groupByProgram(runs);
  const report = {
    generatedAt: new Date().toISOString(),
    runs,
    groups,
    conclusions: conclusions(groups),
  };
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const jsonPath = path.join(OUT_DIR, `workspace-deep-report_${stamp}.json`);
  const mdPath = path.join(OUT_DIR, `workspace-deep-report_${stamp}.md`);
  fs.writeFileSync(jsonPath, JSON.stringify(report, null, 2));
  fs.writeFileSync(mdPath, markdown(report));
  if (has('--json')) console.log(JSON.stringify(report, null, 2));
  else {
    console.log(`Wrote ${path.relative(ROOT, jsonPath)}`);
    console.log(`Wrote ${path.relative(ROOT, mdPath)}`);
    for (const group of groups) {
      const best = group.runs[0];
      console.log(`${group.label}: best ${best.score.stability} (${conditionLabel(best.condition)}) · ${statusMark(best)}`);
    }
  }
}

if (require.main === module) {
  try { main(); }
  catch (err) {
    console.error(`[workspace:deep-report] ${err.message}`);
    process.exit(1);
  }
}
