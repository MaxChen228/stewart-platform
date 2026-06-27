#!/usr/bin/env node
'use strict';

// Same-program optimization queue. Dry-run by default; live requires
// --live --i-am-at-rig and uses workspace_session.js for the actual motion.

const { execFileSync } = require('child_process');

function usage() {
  console.log(`Usage:
  node sysid/workspace_optimize.js --program PROGRAM_ID [--preset heave-zbias|vf-sweep|baseline]
  node sysid/workspace_optimize.js --program PROGRAM_ID --variant '{"zBias":4.5,"vmaxT":45,"vmaxR":35}'
  node sysid/workspace_optimize.js --program PROGRAM_ID --live --i-am-at-rig --preset heave-zbias

Rules:
  - All variants run against one Workspace program id.
  - Program id/hash is preserved, so feature comparisons stay same-reference.
  - Dry-run is default.
`);
}

function parseArgs(argv) {
  const out = {
    base: 'http://localhost:3000',
    program: '',
    preset: 'baseline',
    variants: [],
    live: false,
    atRig: false,
    namePrefix: '',
  };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) throw new Error(`missing value after ${a}`);
      return argv[++i];
    };
    if (a === '--help' || a === '-h') out.help = true;
    else if (a === '--base') out.base = next().replace(/\/+$/, '');
    else if (a === '--program') out.program = next();
    else if (a === '--preset') out.preset = next();
    else if (a === '--variant') out.variants.push(JSON.parse(next()));
    else if (a === '--name-prefix') out.namePrefix = next();
    else if (a === '--live') out.live = true;
    else if (a === '--i-am-at-rig') out.atRig = true;
    else throw new Error(`unknown arg: ${a}`);
  }
  return out;
}

function presetVariants(name) {
  if (name === 'baseline') return [{ label: 'baseline', zBias: 0 }];
  if (name === 'heave-zbias') {
    return [
      { label: 'baseline', zBias: 0 },
      { label: 'zbias3', zBias: 3 },
      { label: 'zbias4p5', zBias: 4.5 },
      { label: 'zbias6', zBias: 6 },
    ];
  }
  if (name === 'vf-sweep') {
    return [
      { label: 'vf60_45', zBias: 0, vmaxT: 60, vmaxR: 45 },
      { label: 'vf45_35', zBias: 0, vmaxT: 45, vmaxR: 35 },
      { label: 'vf30_25', zBias: 0, vmaxT: 30, vmaxR: 25 },
    ];
  }
  throw new Error(`unknown preset: ${name}`);
}

function variantLabel(v, index) {
  if (v.label) return String(v.label).replace(/[^a-zA-Z0-9_.-]/g, '_');
  const parts = [];
  if (Number.isFinite(Number(v.zBias))) parts.push(`zb${String(v.zBias).replace('-', 'm').replace('.', 'p')}`);
  if (Number.isFinite(Number(v.vmaxT))) parts.push(`vt${Math.round(v.vmaxT)}`);
  if (Number.isFinite(Number(v.vmaxR))) parts.push(`vr${Math.round(v.vmaxR)}`);
  return parts.join('_') || `variant${index + 1}`;
}

async function api(base, pathName, body = null) {
  const res = await fetch(`${base}${pathName}`, {
    method: body ? 'POST' : 'GET',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(`${pathName}: ${data.error || res.statusText}`);
  return data;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function apiRetry(base, pathName, body = null, tries = 4) {
  let last = null;
  for (let i = 0; i < tries; i++) {
    try {
      return await api(base, pathName, body);
    } catch (err) {
      last = err;
      if (i + 1 < tries) await sleep(250 + i * 500);
    }
  }
  throw last;
}

async function setProgram(base, programId) {
  const current = await apiRetry(base, '/api/platform-config');
  const programs = current.uiState?.workspace?.programs || [];
  if (!programs.some((p) => p.id === programId)) throw new Error(`unknown Workspace program: ${programId}`);
  const uiState = {
    ...(current.uiState || {}),
    workspace: {
      ...((current.uiState && current.uiState.workspace) || {}),
      currentProgramId: programId,
      blocks: [],
    },
  };
  await apiRetry(base, '/api/platform-config', { uiState });
}

function runSession(opts, variant, index) {
  const args = ['sysid/workspace_session.js', '--base', opts.base];
  const label = `${opts.namePrefix || opts.program}_${variantLabel(variant, index)}`;
  args.push('--name', label);
  if (Number.isFinite(Number(variant.zBias))) args.push('--z-bias', String(variant.zBias));
  if (Number.isFinite(Number(variant.vmaxT))) args.push('--vmaxT', String(variant.vmaxT));
  if (Number.isFinite(Number(variant.vmaxR))) args.push('--vmaxR', String(variant.vmaxR));
  if (opts.live) args.push('--live', '--i-am-at-rig');
  console.log(`\n=== ${label} ===`);
  execFileSync(process.execPath, args, { stdio: 'inherit' });
}

function printFeatureReport(programId) {
  execFileSync(process.execPath, ['sysid/program_features.js', '--program', programId], { stdio: 'inherit' });
}

function printDeepReport(programId) {
  execFileSync(process.execPath, ['sysid/workspace_deep_report.js', '--program', programId], { stdio: 'inherit' });
}

function printWaveforms(programId) {
  execFileSync(process.execPath, ['sysid/workspace_waveform_report.js', '--program', programId], { stdio: 'inherit' });
}

(async () => {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.help) {
    usage();
    return;
  }
  if (!opts.program) throw new Error('--program is required');
  if (opts.live && !opts.atRig) throw new Error('--live requires --i-am-at-rig');
  const variants = opts.variants.length ? opts.variants : presetVariants(opts.preset);
  await setProgram(opts.base, opts.program);
  console.log(`Workspace optimize: program=${opts.program}, variants=${variants.length}, live=${opts.live ? 'yes' : 'no'}`);
  for (let i = 0; i < variants.length; i++) {
    await setProgram(opts.base, opts.program);
    runSession(opts, variants[i], i);
    if (opts.live) {
      printFeatureReport(opts.program);
      printDeepReport(opts.program);
      printWaveforms(opts.program);
    }
  }
  if (!opts.live) {
    console.log('\nDry-run only. Add --live --i-am-at-rig to execute the queue.');
  }
})().catch((err) => {
  console.error(`[workspace:optimize] ${err.message}`);
  process.exit(1);
});
