#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const {
  collectRuns,
  summarize,
  programKey,
  programLabel,
} = require('./program_leaderboard');

const ROOT = path.resolve(__dirname, '..');
const CONFIG_FILE = path.join(ROOT, 'sysid', 'config', 'platform.json');

function usage() {
  console.log(`Usage:
  node sysid/program_recommend.js --program PROGRAM_ID_OR_HASH [--json]

Recommends the next same-program optimization queue. It never compares scores
across different programs.
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

function workspacePrograms() {
  const cfg = readJson(CONFIG_FILE) || {};
  return cfg.uiState?.workspace?.programs || [];
}

function findProgram(idOrHash) {
  const q = String(idOrHash || '');
  return workspacePrograms().find((p) => p.id === q || p.hash === q || String(p.name || '').includes(q)) || null;
}

function scoreValue(run, key) {
  if (!run) return null;
  const v = key === 'stability' ? run.stability : run.score?.[key];
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function conditionNumber(run, key, fallback = 0) {
  const n = Number(run?.condition?.[key]);
  return Number.isFinite(n) ? n : fallback;
}

function programKind(program) {
  const ids = (program?.blocks || []).map((b) => b.id).join(' ');
  if (/\bz-(step|sine)\b/.test(ids)) return 'heave';
  if (/\b(x|y)-(step|sine)\b/.test(ids)) return 'xy';
  if (/\b(roll|pitch|yaw)-(step|sine)\b/.test(ids)) return 'attitude';
  if (/\bbuoy\b|\bhelix\b|\bfig8\b/.test(ids)) return 'combined';
  return 'generic';
}

function uniqVariants(variants) {
  const seen = new Set();
  return variants.filter((v) => {
    const key = JSON.stringify({
      zBias: Number(v.zBias || 0),
      vmaxT: Number(v.vmaxT || 0),
      vmaxR: Number(v.vmaxR || 0),
    });
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });
}

function recommendation(program, runs) {
  const group = runs.length ? summarize(runs) : null;
  const kind = programKind(program);
  const best = group?.best || null;
  const latest = group?.latest || null;
  const bestScore = scoreValue(best, 'stability');
  const tracking = scoreValue(best, 'trackingCost');
  const cross = scoreValue(best, 'crossAxisCost');
  const osc = scoreValue(best, 'oscillationCost');
  const quality = scoreValue(best, 'qualityPenalty');
  const reasons = [];
  let variants = [];
  let preset = 'baseline';

  if (!runs.length) {
    reasons.push('no same-program baseline exists yet');
    if (kind === 'heave') {
      preset = 'heave-zbias';
      variants = [
        { label: 'baseline', zBias: 0 },
        { label: 'zbias3', zBias: 3 },
        { label: 'zbias4p5', zBias: 4.5 },
        { label: 'zbias6', zBias: 6 },
      ];
    } else {
      preset = 'baseline';
      variants = [{ label: 'baseline', zBias: 0 }];
    }
  } else if (quality != null && quality > 6) {
    reasons.push(`quality penalty is high (${quality.toFixed(2)}); first reduce command pressure`);
    preset = 'vf-sweep';
    variants = [
      { label: 'vf60_45', zBias: conditionNumber(best, 'zBias'), vmaxT: 60, vmaxR: 45 },
      { label: 'vf45_35', zBias: conditionNumber(best, 'zBias'), vmaxT: 45, vmaxR: 35 },
      { label: 'vf30_25', zBias: conditionNumber(best, 'zBias'), vmaxT: 30, vmaxR: 25 },
    ];
  } else if (kind === 'heave' && tracking != null && (cross == null || tracking >= cross * 0.8)) {
    const z = conditionNumber(best, 'zBias');
    reasons.push(`tracking cost is a main limiter (${tracking.toFixed(2)}); refine zBias around best=${z}`);
    preset = 'custom-zbias-refine';
    variants = [
      { label: 'zbias_low', zBias: Math.max(0, z - 1.5) },
      { label: 'zbias_best', zBias: z },
      { label: 'zbias_high', zBias: z + 1.5 },
    ];
  } else if ((cross != null && cross > 2.5) || (osc != null && osc > 2.5)) {
    reasons.push(`cross/oscillation cost is limiting (cross=${fmt(cross)}, osc=${fmt(osc)}); try slower follow limits`);
    preset = 'vf-sweep';
    variants = [
      { label: 'vf60_45', zBias: conditionNumber(best, 'zBias'), vmaxT: 60, vmaxR: 45 },
      { label: 'vf45_35', zBias: conditionNumber(best, 'zBias'), vmaxT: 45, vmaxR: 35 },
      { label: 'vf30_25', zBias: conditionNumber(best, 'zBias'), vmaxT: 30, vmaxR: 25 },
    ];
  } else {
    reasons.push(`current best is reasonably balanced (stability=${fmt(bestScore)}); repeat best once for reproducibility`);
    preset = 'repeat-best';
    variants = [{
      label: 'repeat_best',
      zBias: conditionNumber(best, 'zBias'),
      vmaxT: conditionNumber(best, 'vmaxT', 60),
      vmaxR: conditionNumber(best, 'vmaxR', 45),
    }];
  }

  variants = uniqVariants(variants);
  const variantArgs = variants.map((v) => `--variant '${JSON.stringify(v)}'`).join(' ');
  const command = preset.startsWith('custom') || preset === 'repeat-best'
    ? `npm run workspace:optimize -- --program ${program.id} ${variantArgs}`
    : `npm run workspace:optimize -- --program ${program.id} --preset ${preset}`;
  const liveCommand = `${command} -- --live --i-am-at-rig`;
  return {
    program: {
      id: program.id,
      name: program.name || program.id,
      hash: program.hash || null,
      kind,
    },
    runs: runs.length,
    best,
    latest,
    reasons,
    preset,
    variants,
    command,
    liveCommand,
  };
}

function fmt(v, d = 2) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(d) : '-';
}

function printHuman(rec) {
  console.log(`${rec.program.name} (${rec.program.id})`);
  console.log(`kind=${rec.program.kind} runs=${rec.runs}`);
  if (rec.best) {
    const s = rec.best.score || {};
    console.log(`best ${fmt(rec.best.stability)} track=${fmt(s.trackingCost)} cross=${fmt(s.crossAxisCost)} osc=${fmt(s.oscillationCost)} quality=${fmt(s.qualityPenalty)}`);
  } else {
    console.log('best -');
  }
  for (const reason of rec.reasons) console.log(`- ${reason}`);
  console.log('variants:');
  for (const v of rec.variants) console.log(`  ${v.label || 'variant'} ${JSON.stringify(v)}`);
  console.log(`dry-run: ${rec.command}`);
  console.log(`live:    ${rec.liveCommand}`);
}

function main() {
  if (has('--help') || has('-h')) {
    usage();
    return;
  }
  const programQuery = arg('--program');
  if (!programQuery) throw new Error('--program is required');
  const program = findProgram(programQuery);
  if (!program) throw new Error(`unknown program: ${programQuery}`);
  const runs = collectRuns().filter((run) => {
    const p = run.program || {};
    const key = programKey(run);
    return p.id === program.id || p.hash === program.hash || key.includes(program.id) || key.includes(program.hash || '');
  });
  const rec = recommendation(program, runs);
  if (has('--json')) console.log(JSON.stringify(rec, null, 2));
  else printHuman(rec);
}

if (require.main === module) {
  main();
}
