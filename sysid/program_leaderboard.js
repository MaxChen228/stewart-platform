#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const DATA_DIR = path.join(ROOT, 'sysid', 'data');

function usage() {
  console.log(`Usage:
  node sysid/program_leaderboard.js [--program ID_OR_HASH] [--json]

Ranks runs only within the same Workspace program/reference. Cross-program
scores are intentionally not compared.
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
    const stability = readJson(`${stem}.stability.json`);
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
      score: stability?.score || null,
      stability: stability?.score?.stability ?? null,
      health: {
        canEfOr: summary?.canHealth?.efOr ?? stability?.quality?.can?.efOr ?? null,
        rxDropPerS: stability?.quality?.can?.rxDropPerS ?? null,
        rxDropSum: summary?.canHealth?.rxDropSum ?? stability?.quality?.can?.rxDropSum ?? null,
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
  return [...runs].sort((a, b) => {
    const as = number(a.stability);
    const bs = number(b.stability);
    if (as != null && bs != null && bs !== as) return bs - as;
    return Number(b.mtime || 0) - Number(a.mtime || 0);
  });
}

function latestRun(runs) {
  return [...runs].sort((a, b) => Number(b.mtime || 0) - Number(a.mtime || 0))[0] || null;
}

function delta(latest, best) {
  if (!latest || !best || latest.id === best.id) return null;
  const fields = ['stability', 'trackingCost', 'crossAxisCost', 'oscillationCost', 'qualityPenalty'];
  const out = {};
  for (const field of fields) {
    const a = field === 'stability' ? latest.stability : latest.score?.[field];
    const b = field === 'stability' ? best.stability : best.score?.[field];
    const an = number(a), bn = number(b);
    out[field] = an != null && bn != null ? Number((an - bn).toFixed(3)) : null;
  }
  return out;
}

function summarize(programRuns) {
  const ranked = rankRuns(programRuns);
  const best = ranked[0] || null;
  const latest = latestRun(programRuns);
  return {
    key: programRuns[0] ? programKey(programRuns[0]) : '',
    label: programRuns[0] ? programLabel(programRuns[0]) : '',
    runs: programRuns.length,
    best,
    latest,
    latestVsBest: delta(latest, best),
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
    if (group.best) {
      console.log(`best   ${fmt(group.best.stability, 2)}  ${group.best.id}`);
    }
    if (group.latest) {
      const s = group.latest.score || {};
      console.log(`latest ${fmt(group.latest.stability, 2)}  track=${fmt(s.trackingCost)} cross=${fmt(s.crossAxisCost)} osc=${fmt(s.oscillationCost)} quality=${fmt(s.qualityPenalty)}${conditionLabel(group.latest.condition)}  ${group.latest.id}`);
    }
    if (group.latestVsBest) {
      const d = group.latestVsBest;
      console.log(`delta latest-best: stability=${fmt(d.stability)} tracking=${fmt(d.trackingCost)} cross=${fmt(d.crossAxisCost)} osc=${fmt(d.oscillationCost)} quality=${fmt(d.qualityPenalty)}`);
    }
    for (const [i, run] of group.ranked.slice(0, 8).entries()) {
      const s = run.score || {};
      console.log(`${String(i + 1).padStart(2)}  ${fmt(run.stability, 2)}  track=${fmt(s.trackingCost)} cross=${fmt(s.crossAxisCost)} osc=${fmt(s.oscillationCost)} quality=${fmt(s.qualityPenalty)}${conditionLabel(run.condition)}  ${run.id}`);
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
