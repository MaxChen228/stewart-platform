#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const DATA_DIR = path.join(ROOT, 'sysid', 'data');

function usage() {
  console.log(`Usage:
  node sysid/program_features.js [--program ID_OR_HASH] [--json]

Lists run feature summaries only within the same Workspace program/reference.
It intentionally avoids scalar run ratings.
`);
}

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

function runIdFromJsonl(file) {
  return path.basename(file, '.jsonl');
}

function summarizeJsonlMeta(file) {
  let meta = null, firstT = null, lastT = null, samples = 0, cmdCount = 0;
  try {
    for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
      if (!line.trim()) continue;
      let rec;
      try { rec = JSON.parse(line); } catch { continue; }
      if (rec.meta) { meta = rec.meta; continue; }
      if (typeof rec.t === 'number') {
        if (firstT == null) firstT = rec.t;
        lastT = rec.t;
      }
      if (rec.dir === 'cmd') cmdCount++;
      if (rec.dir === 'in') {
        let d;
        try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
        if (Array.isArray(d?.a)) samples++;
      }
    }
  } catch {}
  return { meta, samples, cmdCount, durationS: firstT != null && lastT != null ? (lastT - firstT) / 1000 : null };
}

function collectRuns() {
  let files = [];
  try {
    files = fs.readdirSync(DATA_DIR)
      .filter((x) => x.endsWith('.jsonl'))
      .map((x) => path.join(DATA_DIR, x))
      .filter((file) => fs.statSync(file).isFile())
      .sort((a, b) => fs.statSync(b).mtimeMs - fs.statSync(a).mtimeMs);
  } catch {}
  return files.map((file) => {
    const stem = file.replace(/\.jsonl$/, '');
    const id = runIdFromJsonl(file);
    const meta = summarizeJsonlMeta(file);
    const summary = readJson(`${stem}.summary.json`);
    const evaluation = readJson(`${stem}.evaluation.json`);
    const bundle = readJson(`${stem}.bundle.json`);
    const manifest = bundle?.manifest ? readJson(bundle.manifest) : null;
    const program = meta.meta?.owner?.program || manifest?.program || null;
    return {
      id,
      file: path.relative(ROOT, file),
      name: meta.meta?.name || manifest?.name || id,
      wallClock: meta.meta?.wallClock || summary?.meta?.wallClock || manifest?.wallClock || null,
      mtime: fs.statSync(file).mtimeMs,
      durationS: summary?.durationS ?? meta.durationS,
      samples: summary?.samples ?? meta.samples,
      cmdCount: meta.cmdCount,
      program,
      condition: manifest?.condition || meta.meta?.owner?.condition || null,
      evaluation,
      verdict: evaluation?.quality?.verdict ?? null,
      health: {
        canEfOr: evaluation?.health?.efOr ?? summary?.canHealth?.efOr ?? null,
        rxDropPerS: evaluation?.health?.rxDropPerS ?? null,
        rxDropSum: evaluation?.health?.rxDrop ?? summary?.canHealth?.rxDropSum ?? null,
        maxStepDeg: evaluation?.fullBadness?.motorStepMaxDeg ?? null,
        crossPeak: evaluation?.fullBadness?.poseCrossHpPeak ?? null,
      },
    };
  });
}

function programKey(run) {
  const p = run.program;
  if (!p) return '__unclassified__';
  if (p.hash) return p.draft ? `draft:${p.hash}` : `program:${p.hash}`;
  if (p.id) return p.draft ? `draft:${p.id}` : `program:${p.id}`;
  return '__unclassified__';
}

function programLabel(run) {
  const p = run.program;
  if (!p) return 'Unclassified';
  return p.name || p.id || p.hash || 'Unnamed program';
}

function number(v) {
  if (v === null || v === undefined || v === '') return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function rankRuns(runs) {
  return [...runs].sort((a, b) => Number(b.mtime || 0) - Number(a.mtime || 0));
}

function latestRun(runs) {
  return [...runs].sort((a, b) => Number(b.mtime || 0) - Number(a.mtime || 0))[0] || null;
}

function delta(latest, reference) {
  if (!latest || !reference || latest.id === reference.id) return null;
  const fields = ['maxStepDeg', 'p99StepDeg', 'crossPeak', 'rxDropPerS'];
  const out = {};
  for (const field of fields) {
    const a = latest.health?.[field];
    const b = reference.health?.[field];
    const an = number(a), bn = number(b);
    out[field] = an != null && bn != null ? Number((an - bn).toFixed(3)) : null;
  }
  return out;
}

function summarize(programRuns) {
  const ranked = rankRuns(programRuns);
  const reference = ranked[1] || null;
  const latest = latestRun(programRuns);
  return {
    key: programRuns[0] ? programKey(programRuns[0]) : '',
    label: programRuns[0] ? programLabel(programRuns[0]) : '',
    runs: programRuns.length,
    reference,
    latest,
    latestVsReference: delta(latest, reference),
    ranked,
  };
}

function fmt(v, d = 2) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(d) : '-';
}

function conditionLabel(condition) {
  if (!condition || typeof condition !== 'object') return '';
  const parts = [];
  if (Number.isFinite(Number(condition.zBias)) && Number(condition.zBias) !== 0) parts.push(`zBias=${fmt(condition.zBias, 2)}`);
  if (Number.isFinite(Number(condition.vmaxT))) parts.push(`vT=${fmt(condition.vmaxT, 0)}`);
  if (Number.isFinite(Number(condition.vmaxR))) parts.push(`vR=${fmt(condition.vmaxR, 0)}`);
  return parts.length ? ` [${parts.join(' ')}]` : '';
}

function printHuman(groups) {
  if (!groups.length) {
    console.log('No runs found.');
    return;
  }
  for (const group of groups) {
    console.log(`\n${group.label} (${group.key})`);
    console.log(`runs=${group.runs}`);
    if (group.latest) {
      const h = group.latest.health || {};
      console.log(`latest ${group.latest.verdict || ''}  maxStep=${fmt(h.maxStepDeg)} p99=${fmt(h.p99StepDeg)} cross=${fmt(h.crossPeak)} rxDrop/s=${fmt(h.rxDropPerS)}${conditionLabel(group.latest.condition)}  ${group.latest.id}`);
    }
    if (group.latestVsReference) {
      const d = group.latestVsReference;
      console.log(`delta latest-prev: maxStep=${fmt(d.maxStepDeg)} p99=${fmt(d.p99StepDeg)} cross=${fmt(d.crossPeak)} rxDrop/s=${fmt(d.rxDropPerS)}`);
    }
    for (const [i, run] of group.ranked.slice(0, 8).entries()) {
      const h = run.health || {};
      console.log(`${String(i + 1).padStart(2)}  ${run.verdict || ''}  maxStep=${fmt(h.maxStepDeg)} p99=${fmt(h.p99StepDeg)} cross=${fmt(h.crossPeak)} rxDrop/s=${fmt(h.rxDropPerS)}${conditionLabel(run.condition)}  ${run.id}`);
    }
  }
}

function main() {
  if (has('--help') || has('-h')) {
    usage();
    return;
  }
  const filter = arg('--program');
  const runs = collectRuns();
  const groups = new Map();
  for (const run of runs) {
    const key = programKey(run);
    if (filter) {
      const p = run.program || {};
      const hay = [key, p.id, p.hash, p.name].filter(Boolean).join(' ');
      if (!hay.includes(filter)) continue;
    }
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(run);
  }
  const out = [...groups.values()]
    .map(summarize)
    .sort((a, b) => a.label.localeCompare(b.label));
  if (has('--json')) console.log(JSON.stringify(out, null, 2));
  else printHuman(out);
}

if (require.main === module) main();
else module.exports = { collectRuns, summarize, programKey, programLabel };
