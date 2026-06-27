#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const Kin = require('./kin');
const PlatformSoT = require('./platform_sot');

const ROOT = path.resolve(__dirname, '..');
const MOTION_LIB = path.join(ROOT, 'web', 'shared', 'motion-library.js');
const WORKSPACE_CORE = path.join(ROOT, 'web', 'shared', 'workspace-core.js');
const CONFIG_FILE = path.join(ROOT, 'sysid', 'config', 'platform.json');

async function loadEsm(file) {
  const source = fs.readFileSync(file, 'utf8');
  return import(`data:text/javascript;charset=utf-8,${encodeURIComponent(source)}`);
}

function maxPoseErr(a, b) {
  return Math.max(...a.map((v, i) => Math.abs(Number(v) - Number(b[i]))));
}

function finitePose(pose) {
  return Array.isArray(pose) && pose.length === 6 && pose.every((v) => Number.isFinite(Number(v)));
}

function readConfig() {
  try {
    return PlatformSoT.mergeConfig(JSON.parse(fs.readFileSync(CONFIG_FILE, 'utf8')));
  } catch {
    return PlatformSoT.mergeConfig({});
  }
}

(async () => {
  const [lib, core] = await Promise.all([loadEsm(MOTION_LIB), loadEsm(WORKSPACE_CORE)]);
  const issues = [];
  const ids = new Set();
  const homePose = PlatformSoT.relToAbs(readConfig().homeRelative, Kin.NEUTRAL_Z);

  if (!Array.isArray(lib.MOTIONS) || !lib.MOTIONS.length) issues.push('MOTIONS must be a non-empty array');

  for (const [index, def] of (lib.MOTIONS || []).entries()) {
    const where = def?.id || `motion[${index}]`;
    if (!def || typeof def !== 'object') {
      issues.push(`motion[${index}] must be an object`);
      continue;
    }
    if (!/^[a-z][a-z0-9-]*$/.test(def.id || '')) issues.push(`${where}: id must be kebab-case ascii`);
    if (ids.has(def.id)) issues.push(`${where}: duplicate id`);
    ids.add(def.id);
    if (typeof def.label !== 'string' || !def.label.trim()) issues.push(`${where}: label is required`);
    if (typeof def.per !== 'function') issues.push(`${where}: per(params) function is required`);
    if (typeof def.fn !== 'function') issues.push(`${where}: fn(t, params) function is required`);
    if (!Array.isArray(def.params)) issues.push(`${where}: params must be an array`);

    const keys = new Set();
    for (const param of def.params || []) {
      if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(param?.k || '')) issues.push(`${where}: param key must be ascii identifier`);
      if (keys.has(param.k)) issues.push(`${where}: duplicate param ${param.k}`);
      keys.add(param.k);
      for (const field of ['min', 'max', 'def', 'step']) {
        if (!Number.isFinite(Number(param[field]))) issues.push(`${where}.${param.k}: ${field} must be numeric`);
      }
      if (Number(param.min) > Number(param.max)) issues.push(`${where}.${param.k}: min > max`);
      if (Number(param.def) < Number(param.min) || Number(param.def) > Number(param.max)) {
        issues.push(`${where}.${param.k}: def outside min/max`);
      }
    }

    if (typeof def.per !== 'function' || typeof def.fn !== 'function' || !Array.isArray(def.params)) continue;
    const params = lib.motionParams(def, lib.motionDefaults(def));
    const per = lib.motionPeriod(def, params);
    if (!Number.isFinite(per) || per <= 0) {
      issues.push(`${where}: default period must be > 0`);
      continue;
    }

    for (let i = 0; i <= 24; i++) {
      const t = (per * i) / 24;
      const raw = def.fn(t, params);
      if (!finitePose(raw)) issues.push(`${where}: fn(${t.toFixed(3)}) must return 6 finite numbers`);
      const sample = lib.motionPoseAt(def, params, t, homePose, Kin);
      if (!finitePose(sample.pose)) issues.push(`${where}: sample pose is not finite at t=${t.toFixed(3)}`);
      if (!sample.ok) issues.push(`${where}: default sample IK invalid at t=${t.toFixed(3)}`);
    }

    const start = lib.motionPoseAt(def, params, 0, homePose, Kin);
    const end = lib.motionPoseAt(def, params, per, homePose, Kin);
    if (maxPoseErr(start.pose, homePose) > 1e-6) issues.push(`${where}: t=0 must be HOME after envelope`);
    if (maxPoseErr(end.pose, homePose) > 1e-6) issues.push(`${where}: t=period must be HOME after envelope`);
  }

  const config = readConfig();
  const blocks = core.normalizeBlocks(config, lib.MOTIONS, lib);
  const audit = core.closedLoopAudit(blocks, homePose, Kin, lib.MOTIONS, lib);
  if (!audit.ok) issues.push(...audit.issues.map((x) => `saved workspace: ${x}`));
  const programs = typeof core.normalizePrograms === 'function'
    ? core.normalizePrograms(config, lib.MOTIONS, lib)
    : [];
  for (const program of programs) {
    const programAudit = core.closedLoopAudit(program.blocks || [], homePose, Kin, lib.MOTIONS, lib);
    if (!programAudit.ok) {
      issues.push(...programAudit.issues.map((x) => `program ${program.id}: ${x}`));
    }
  }

  if (issues.length) {
    console.error(`[motion-library] FAIL (${issues.length})`);
    for (const issue of issues) console.error(`- ${issue}`);
    process.exit(1);
  }
  console.log(`[motion-library] PASS · ${lib.MOTIONS.length} motions · ${blocks.length} scratch blocks · ${programs.length} programs`);
  if (audit.warnings?.length) console.log(`[workspace] ${audit.warnings.join('; ')}`);
})().catch((err) => {
  console.error(`[motion-library] ${err.message}`);
  process.exit(1);
});
