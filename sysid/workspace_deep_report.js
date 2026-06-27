#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { analyze, summarizeRuns } = require('./workspace_evaluation');

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

function readJson(file) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); }
  catch { return null; }
}

function runFiles() {
  return fs.readdirSync(DATA_DIR)
    .filter((x) => x.endsWith('.jsonl'))
    .map((x) => path.join(DATA_DIR, x))
    .filter((file) => fs.statSync(file).isFile())
    .sort();
}

function manifestFor(file) {
  const stem = file.replace(/\.jsonl$/, '');
  const direct = readJson(`${stem}.manifest.json`);
  if (direct) return direct;
  const bundle = readJson(`${stem}.bundle.json`);
  return bundle?.manifest ? readJson(bundle.manifest) : null;
}

function conditionLabel(condition) {
  if (!condition || typeof condition !== 'object') return 'default';
  const parts = [];
  for (const key of ['zBias', 'vmaxT', 'vmaxR', 'warmupMs', 'homeMs', 'landMs']) {
    if (Number.isFinite(Number(condition[key]))) parts.push(`${key}=${condition[key]}`);
  }
  return parts.join(' ') || 'default';
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
    summary: summarizeRuns(items),
    runs: items.sort((a, b) => Number(b.path.includes('queue')) - Number(a.path.includes('queue')) || String(b.file).localeCompare(String(a.file))),
  })).sort((a, b) => a.label.localeCompare(b.label));
}

function readRun(file) {
  const evaluation = analyze(file);
  const manifest = manifestFor(file) || {};
  return {
    ...evaluation,
    id: path.basename(file, '.jsonl'),
    program: manifest.program || evaluation.meta?.owner?.program || null,
    condition: manifest.condition || evaluation.meta?.owner?.condition || null,
  };
}

function markdown(report) {
  const lines = [];
  lines.push('# Workspace Evaluation Report');
  lines.push('');
  lines.push(`Generated: ${report.generatedAt}`);
  lines.push('');
  lines.push('## Program Summary');
  lines.push('');
  lines.push('| Program | Condition | Verdict | Max step | P99 step | Cross peak | Worst | Takeoff | Landing | rxDrop/s | ef |');
  lines.push('|---|---|---:|---:|---:|---:|---|---:|---:|---:|---:|');
  for (const group of report.groups) {
    for (const run of group.runs) {
      const q = run.quality;
      const b = run.fullBadness;
      const h = run.health;
      lines.push(`| ${group.label} | ${conditionLabel(run.condition)} | ${q.verdict} | ${b.motorStepMaxDeg} | ${b.motorStepP99MeanDeg} | ${b.poseCrossHpPeak} | ${b.worstMotorStep?.label || ''} | ${q.takeoff?.motorStepMaxDeg ?? ''} | ${q.landing?.motorStepMaxDeg ?? ''} | ${h.rxDropPerS} | ${h.efOr} |`);
    }
  }
  lines.push('');
  lines.push('## Failure Detail');
  lines.push('');
  for (const group of report.groups) {
    lines.push(`### ${group.label}`);
    for (const run of group.runs) {
      lines.push(`- \`${run.id}\`: ${run.quality.verdict}; ${run.quality.failures.join('; ') || 'no hard failures'}`);
      lines.push(`  - takeoff maxStep=${run.quality.takeoff?.motorStepMaxDeg ?? 'n/a'} worst=${run.quality.takeoff?.worstMotor ?? 'n/a'} cross=${run.quality.takeoff?.crossHpPeak ?? 'n/a'} axis=${run.quality.takeoff?.worstCrossAxis ?? 'n/a'}`);
      lines.push(`  - landing maxStep=${run.quality.landing?.motorStepMaxDeg ?? 'n/a'} worst=${run.quality.landing?.worstMotor ?? 'n/a'} cross=${run.quality.landing?.crossHpPeak ?? 'n/a'} axis=${run.quality.landing?.worstCrossAxis ?? 'n/a'}`);
      for (const e of (run.topEvents?.motorSteps || []).slice(0, 3)) {
        lines.push(`  - event ${e.tS}s ${e.phase} ${e.motor}: ${e.stepDeg}deg / ${e.dtMs}ms; pose Y/Roll ${e.before.pose.Y}/${e.before.pose.Roll} -> ${e.after.pose.Y}/${e.after.pose.Roll}; CAN ef=${e.health.canEf} tx=${e.health.canTx} rxDrop=${e.health.rxDrop}`);
      }
    }
    lines.push('');
  }
  lines.push('## Engineering Read');
  lines.push('');
  lines.push(...report.conclusions.map((x) => `- ${x}`));
  lines.push('');
  return lines.join('\n');
}

function conclusions(groups) {
  const runs = groups.flatMap((g) => g.runs);
  const out = [];
  const rejects = runs.filter((r) => r.quality.verdict === 'reject');
  if (rejects.length) out.push(`${rejects.length}/${runs.length} run(s) are lifecycle rejects; do not optimize from scalar success claims.`);
  const worst = [...runs].sort((a, b) => Number(b.fullBadness.motorStepMaxDeg) - Number(a.fullBadness.motorStepMaxDeg)).slice(0, 3);
  if (worst.length) out.push(`Worst motor steps: ${worst.map((r) => `${r.id} ${r.fullBadness.worstMotorStep?.label || '?'}=${r.fullBadness.motorStepMaxDeg}deg`).join(', ')}.`);
  const canDirty = runs.filter((r) => r.health.efOr || r.health.rxDrop);
  if (canDirty.length) out.push(`${canDirty.length}/${runs.length} run(s) have CAN EFLG/rxDrop evidence; treat controller conclusions as data-plane-coupled.`);
  out.push('Next comparisons should inspect verdict, max motor step, cross-axis peak, phase detail, and repeatability before any controller-model claims.');
  return out;
}

function main() {
  const filter = arg('--program');
  let files = runFiles();
  if (filter) files = files.filter((file) => file.includes(filter));
  if (!files.length) throw new Error('no active JSONL runs found');
  fs.mkdirSync(OUT_DIR, { recursive: true });
  const runs = files.map(readRun);
  const groups = groupByProgram(runs);
  const report = {
    generatedAt: new Date().toISOString(),
    runs,
    groups,
    conclusions: conclusions(groups),
  };
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const jsonPath = path.join(OUT_DIR, `workspace-evaluation-report_${stamp}.json`);
  const mdPath = path.join(OUT_DIR, `workspace-evaluation-report_${stamp}.md`);
  fs.writeFileSync(jsonPath, JSON.stringify(report, null, 2));
  fs.writeFileSync(mdPath, markdown(report));
  if (has('--json')) console.log(JSON.stringify(report, null, 2));
  else {
    console.log(`Wrote ${path.relative(ROOT, jsonPath)}`);
    console.log(`Wrote ${path.relative(ROOT, mdPath)}`);
    for (const group of groups) {
      const latest = group.runs[0];
      console.log(`${group.label}: latest ${latest.quality.verdict} maxStep=${latest.fullBadness.motorStepMaxDeg} cross=${latest.fullBadness.poseCrossHpPeak} (${conditionLabel(latest.condition)})`);
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
