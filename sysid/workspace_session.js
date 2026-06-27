#!/usr/bin/env node
'use strict';

// Workspace session runner for Codex/CLI.
// Dry-run is default. Live motion requires both --live and --i-am-at-rig.

const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');
const Kin = require('./kin');
const PlatformSoT = require('./platform_sot');

const ROOT = path.resolve(__dirname, '..');
const MOTION_LIB = path.join(ROOT, 'web', 'shared', 'motion-library.js');
const WORKSPACE_CORE = path.join(ROOT, 'web', 'shared', 'workspace-core.js');

function parseArgs(argv) {
  const out = {
    base: 'http://localhost:3000',
    live: false,
    atRig: false,
    name: '',
    closeMs: 1200,
    warmupMs: null,
    homeMs: null,
    landMs: null,
    zBias: 0,
    vmaxT: null,
    vmaxR: null,
    releaseObserveMs: 1200,
  };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) throw new Error(`missing value after ${a}`);
      return argv[++i];
    };
    if (a === '--base') out.base = next().replace(/\/+$/, '');
    else if (a === '--name') out.name = next();
    else if (a === '--close-ms') out.closeMs = Math.max(300, Number(next()) || out.closeMs);
    else if (a === '--warmup-ms') out.warmupMs = Math.max(0, Number(next()) || 0);
    else if (a === '--home-ms') out.homeMs = Math.max(300, Number(next()) || 0);
    else if (a === '--land-ms') out.landMs = Math.max(300, Number(next()) || 0);
    else if (a === '--z-bias') out.zBias = Number(next()) || 0;
    else if (a === '--vmaxT') out.vmaxT = Math.max(1, Number(next()) || 0);
    else if (a === '--vmaxR') out.vmaxR = Math.max(1, Number(next()) || 0);
    else if (a === '--release-observe-ms') out.releaseObserveMs = Math.max(0, Number(next()) || 0);
    else if (a === '--live') out.live = true;
    else if (a === '--i-am-at-rig') out.atRig = true;
    else if (a === '--help' || a === '-h') out.help = true;
    else throw new Error(`unknown arg: ${a}`);
  }
  return out;
}

function usage() {
  console.log(`Usage:
  npm run workspace:session -- [--base http://localhost:3000] [--name NAME]
  npm run workspace:session -- --live --i-am-at-rig --name NAME [--warmup-ms MS] [--home-ms MS] [--land-ms MS] [--z-bias MM] [--vmaxT MM_S] [--vmaxR DEG_S]

Dry-run validates the saved Workspace flow and closed-loop contract.
Live runs: record full lifecycle -> TAKE OFF -> HOME -> FOLLOW -> score window -> LANDING -> RELEASE -> analyze.
`);
}

async function loadMotionLibrary() {
  const source = fs.readFileSync(MOTION_LIB, 'utf8');
  return import(`data:text/javascript;charset=utf-8,${encodeURIComponent(source)}`);
}

async function loadWorkspaceCore() {
  const source = fs.readFileSync(WORKSPACE_CORE, 'utf8');
  return import(`data:text/javascript;charset=utf-8,${encodeURIComponent(source)}`);
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

async function apiRetry(base, pathName, body = null, tries = 3) {
  let last = null;
  for (let i = 0; i < tries; i++) {
    try {
      return await api(base, pathName, body);
    } catch (err) {
      last = err;
      if (i + 1 < tries) await sleep(200 + i * 300);
    }
  }
  throw last;
}

function wsUrl(base) {
  const u = new URL(base);
  u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
  u.pathname = '/';
  u.search = '';
  return u.toString();
}

function poseLine(pose) {
  return pose.map((x) => Number(x || 0).toFixed(3)).join(' ');
}

function commandPose(pose, opts) {
  const out = pose.map(Number);
  if (Number.isFinite(opts.zBias) && opts.zBias !== 0) out[2] += opts.zBias;
  return out;
}

function runCondition(opts, config) {
  const timing = config?.trialDefaults || {};
  const condition = {
    zBias: Number.isFinite(opts.zBias) ? opts.zBias : 0,
    vmaxT: Number.isFinite(opts.vmaxT) ? opts.vmaxT : Number(config.followLimits?.vmaxT || 60),
    vmaxR: Number.isFinite(opts.vmaxR) ? opts.vmaxR : Number(config.followLimits?.vmaxR || 45),
    warmupMs: Number.isFinite(opts.warmupMs) ? opts.warmupMs : Math.max(0, Number(timing.warmupMs) || 0),
    homeMs: Number.isFinite(opts.homeMs) ? opts.homeMs : Math.max(300, Number(timing.homeMs) || 1500),
    landMs: Number.isFinite(opts.landMs) ? opts.landMs : Math.max(300, Number(timing.landMs) || 1500),
  };
  return Object.fromEntries(Object.entries(condition).filter(([, v]) => Number.isFinite(v)));
}

function runIdFromRecordingPath(p) {
  const name = String(p || '').split('/').pop() || '';
  return name.endsWith('.jsonl') ? name.slice(0, -6) : null;
}

function describePlan(blocks, homePose, landingPose, lib, core, condition = null) {
  console.log('Workspace plan');
  console.log(`  blocks: ${blocks.length}`);
  console.log(`  motion window: ${core.totalSeconds(blocks, lib.MOTIONS, lib).toFixed(1)}s`);
  console.log(`  start/end: HOME [${homePose.map((x) => x.toFixed(1)).join(', ')}]`);
  console.log(`  landing: [${landingPose.map((x) => x.toFixed(1)).join(', ')}]`);
  blocks.forEach((b, i) => {
    const def = core.motionById(lib.MOTIONS, b.id);
    console.log(`  ${i + 1}. ${def.label} x${b.loops} · ${lib.formatMotionParams(def, b.params)}`);
  });
  if (condition) {
    console.log(`  condition: zBias=${condition.zBias || 0}mm · VF ${condition.vmaxT}/${condition.vmaxR} · warmup ${condition.warmupMs || 0}ms · takeoff ${condition.homeMs || 0}ms · land ${condition.landMs || 0}ms`);
  }
}

function currentProgram(config) {
  const ws = config?.uiState?.workspace || {};
  return (ws.programs || []).find((program) => program.id === ws.currentProgramId) || null;
}

function activePlan(config, lib, core) {
  const program = currentProgram(config);
  if (program) {
    return {
      program,
      blocks: core.normalizeBlockList(program.blocks || [], lib.MOTIONS, lib),
    };
  }
  return {
    program: null,
    blocks: core.normalizeBlocks(config, lib.MOTIONS, lib),
  };
}

function programMeta(config, blocks, lib, core) {
  const program = currentProgram(config);
  if (typeof core.workspaceProgramMeta === 'function') {
    return core.workspaceProgramMeta({
      program,
      blocks,
      name: program ? program.name : 'Scratch Draft',
      motions: lib.MOTIONS,
      motionLib: lib,
    });
  }
  if (!program) return null;
  return {
    id: program.id,
    name: program.name || program.id,
    hash: typeof core.programHash === 'function' ? core.programHash(program, lib) : program.hash || null,
    blocks: blocks.length,
    durationS: core.totalSeconds(blocks, lib.MOTIONS, lib),
  };
}

function slug(value) {
  return String(value || 'workspace')
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '_')
    .replace(/^_+|_+$/g, '')
    .slice(0, 48) || 'workspace';
}

async function preflight(base) {
  const [latest, transport, rec, config] = await Promise.all([
    api(base, '/api/latest').catch(() => ({})),
    api(base, '/api/transport').catch(() => ({})),
    api(base, '/api/rec/status').catch(() => ({})),
    api(base, '/api/platform-config'),
  ]);
  if (transport.state !== 'connected') throw new Error(`transport not connected (${transport.state || 'unknown'})`);
  if (rec.recording) throw new Error('recorder already running');
  if (!Array.isArray(latest.a) || latest.a.length !== 6) throw new Error('no fresh telemetry');
  if (Number(latest.ok) !== 6) throw new Error(`encoder ok=${latest.ok ?? 'missing'}, expected 6`);
  const ageMs = Number(latest._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
  if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
  return PlatformSoT.mergeConfig(config);
}

function assertLivePacket(d, label) {
  if (!d || !Array.isArray(d.a) || d.a.length !== 6) throw new Error(`${label}: no telemetry packet`);
  if (Number(d.ok) !== 6) throw new Error(`${label}: encoder ok=${d.ok ?? 'missing'}, expected 6`);
  const txFail = Number(d.can?.txFail ?? 0);
  const cmdDrop = Number(d.sched?.cmd_drop ?? 0);
  if (txFail) throw new Error(`${label}: CAN txFail=${txFail}`);
  if (cmdDrop) throw new Error(`${label}: command drops=${cmdDrop}`);
  return d;
}

async function verifyLiveApi(base, label, expect = {}) {
  const timeoutMs = Number(expect.timeoutMs) || 1800;
  const t0 = Date.now();
  let lastReason = '';
  while (Date.now() - t0 <= timeoutMs) {
    const [transport, latest, session] = await Promise.all([
      apiRetry(base, '/api/transport'),
      apiRetry(base, '/api/latest'),
      apiRetry(base, '/api/session/status').catch(() => ({})),
    ]);
    if (transport.state !== 'connected') throw new Error(`${label}: transport ${transport.state || 'unknown'}`);
    const ageMs = Number(latest._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
    if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`${label}: telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
    assertLivePacket(latest, label);
    if (Number.isFinite(expect.pos) && Number(latest.pos) !== expect.pos) {
      lastReason = `pos=${latest.pos}, expected ${expect.pos}`;
    } else if (Number.isFinite(expect.fl) && Number(latest.fl) !== expect.fl) {
      lastReason = `follow=${latest.fl}, expected ${expect.fl}`;
    } else if (expect.phase && session.phase !== expect.phase) {
      lastReason = `session phase=${session.phase || 'unknown'}, expected ${expect.phase}`;
    } else {
      return latest;
    }
    await sleep(120);
  }
  throw new Error(`${label}: ${lastReason || 'condition not reached'}`);
}

function openWs(base) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(wsUrl(base));
    const state = { latest: null, follow: false };
    const timer = setTimeout(() => reject(new Error('websocket connect timeout')), 4000);
    ws.on('open', () => {
      clearTimeout(timer);
      resolve({ ws, state });
    });
    ws.on('message', (buf) => {
      let d;
      try { d = JSON.parse(buf.toString()); } catch { return; }
      if (Array.isArray(d.a)) state.latest = { data: d, at: Date.now() };
      if (d.status === 'follow on') state.follow = true;
      if (d.status === 'follow off' || d.fl === 0) state.follow = false;
    });
    ws.on('error', reject);
  });
}

function waitUntil(fn, timeoutMs, label) {
  return new Promise((resolve, reject) => {
    const t0 = Date.now();
    const tick = () => {
      if (fn()) return resolve();
      if (Date.now() - t0 > timeoutMs) return reject(new Error(label || 'timeout'));
      setTimeout(tick, 80);
    };
    tick();
  });
}

function healthy(state) {
  if (!state.latest || Date.now() - state.latest.at > 1500) throw new Error('live telemetry stale');
  return assertLivePacket(state.latest.data, 'live telemetry');
}

async function runLive(opts, blocks, homePose, landingPose, config, lib, core) {
  if (!opts.live || !opts.atRig) throw new Error('live motion requires --live --i-am-at-rig');
  const { ws, state } = await openWs(opts.base);
  const send = (cmd, token) => ws.send(JSON.stringify({ cmd, sessionToken: token }));
  const program = programMeta(config, blocks, lib, core);
  const condition = runCondition(opts, config);
  const prefix = program ? program.id : slug(config?.uiState?.workspace?.currentProgramId || 'workspace');
  const suffix = condition.zBias ? `_zbias${String(condition.zBias).replace('-', 'm').replace('.', 'p')}` : '';
  const label = opts.name || `${prefix}${suffix}_workspace_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}`;
  let token = null;
  let recording = false;
  try {
    const started = await api(opts.base, '/api/session/start', { label, phase: 'takeoff', program, condition });
    token = started.token;
    const rec = await api(opts.base, '/api/session/rec/start', { token, name: label });
    recording = true;
    await verifyLiveApi(opts.base, 'after recorder start');
    send('H', token);
    await waitUntil(() => {
      const d = state.latest?.data;
      return d && d.pos === 1 && Array.isArray(d.hold);
    }, 3000, 'HOLD did not arm');
    await verifyLiveApi(opts.base, 'after HOLD', { pos: 1 });
    send(`VF ${condition.vmaxT || config.followLimits?.vmaxT || 60} ${condition.vmaxR || config.followLimits?.vmaxR || 45}`, token);
    if (condition.warmupMs > 0) {
      await api(opts.base, '/api/session/phase', { token, phase: 'holding warmup' });
      await sleep(condition.warmupMs);
    }
    await api(opts.base, '/api/session/phase', { token, phase: 'takeoff' });
    send(`P ${poseLine(homePose)} ${condition.homeMs}`, token);
    await sleep(condition.homeMs + 400);
    await verifyLiveApi(opts.base, 'after HOME settle', { pos: 1 });
    send('FOLLOW 1', token);
    await waitUntil(() => state.follow, 3500, 'FOLLOW did not arm');
    await verifyLiveApi(opts.base, 'after FOLLOW on', { pos: 1, fl: 1 });
    await api(opts.base, '/api/session/phase', { token, phase: 'recording' });
    await verifyLiveApi(opts.base, 'recording window opened', { pos: 1, fl: 1, phase: 'recording' });

    for (const [i, block] of blocks.entries()) {
      const def = core.motionById(lib.MOTIONS, block.id);
      if (!def) throw new Error(`unknown motion: ${block.id}`);
      const per = lib.motionPeriod(def, block.params);
      const totalMs = per * block.loops * 1000;
      const t0 = Date.now();
      await api(opts.base, '/api/session/phase', { token, phase: `block ${i + 1}/${blocks.length}` });
      while (Date.now() - t0 <= totalMs) {
        healthy(state);
        const t = (Date.now() - t0) / 1000;
        const sample = lib.motionPoseAt(def, block.params, t, homePose, Kin);
        if (sample.ok) send(`PF ${poseLine(commandPose(sample.pose, opts))}`, token);
        await sleep(1000 / lib.MOTION_HZ);
      }
    }

    await api(opts.base, '/api/session/phase', { token, phase: 'close loop' });
    const closeT0 = Date.now();
    while (Date.now() - closeT0 < opts.closeMs) {
      healthy(state);
      send(`PF ${poseLine(commandPose(homePose, opts))}`, token);
      await sleep(1000 / lib.MOTION_HZ);
    }
    await api(opts.base, '/api/session/phase', { token, phase: 'landing' });
    await verifyLiveApi(opts.base, 'landing phase marked', { pos: 1, fl: 1, phase: 'landing' });
    send('FOLLOW 0', token);
    await waitUntil(() => !state.follow, 2500, 'FOLLOW did not disarm');
    await verifyLiveApi(opts.base, 'after FOLLOW off', { pos: 1, fl: 0, phase: 'landing' });
    send(`P ${poseLine(landingPose)} ${condition.landMs}`, token);
    await sleep(condition.landMs + 300);
    await verifyLiveApi(opts.base, 'after landing settle', { pos: 1, fl: 0, phase: 'landing' });
    send('D', token);
    await api(opts.base, '/api/session/phase', { token, phase: 'release' });
    await sleep(opts.releaseObserveMs);
    await verifyLiveApi(opts.base, 'after release observe', { pos: 0, fl: 0, phase: 'release' });
    const stopped = await api(opts.base, '/api/session/rec/stop', { token, analyze: false }).catch(() => ({}));
    recording = false;
    const runId = runIdFromRecordingPath(stopped.path);
    await api(opts.base, '/api/session/finish', { token });
    if (runId) await api(opts.base, `/api/runs/${encodeURIComponent(runId)}/analyze`, { program, condition }).catch(() => ({}));
    console.log(`Workspace session complete: ${label}`);
  } catch (err) {
    if (token) {
      send(`PF ${poseLine(homePose)}`, token);
      send('FOLLOW 0', token);
      await new Promise((r) => setTimeout(r, 150));
      const landMs = Number.isFinite(condition?.landMs) ? condition.landMs : 1500;
      send(`P ${poseLine(landingPose)} ${landMs}`, token);
      await new Promise((r) => setTimeout(r, landMs + 300));
      send('D', token);
      if (recording) await api(opts.base, '/api/session/rec/stop', { token, analyze: false }).catch(() => ({}));
      await api(opts.base, '/api/session/abort', { token, reason: err.message }).catch(() => ({}));
    }
    throw err;
  } finally {
    ws.close();
  }
}

(async () => {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.help) {
    usage();
    return;
  }
  if (opts.live && !opts.atRig) throw new Error('--live requires --i-am-at-rig');
  const [lib, core] = await Promise.all([loadMotionLibrary(), loadWorkspaceCore()]);
  const config = opts.live ? await preflight(opts.base) : PlatformSoT.mergeConfig(await api(opts.base, '/api/platform-config').catch(() => ({})));
  const plan = activePlan(config, lib, core);
  const blocks = plan.blocks;
  const homePose = PlatformSoT.relToAbs(config.homeRelative, Kin.NEUTRAL_Z);
  const landingPose = PlatformSoT.relToAbs(config.landingRelative, Kin.NEUTRAL_Z);
  const condition = runCondition(opts, config);
  describePlan(blocks, homePose, landingPose, lib, core, condition);
  const program = programMeta(config, blocks, lib, core);
  if (program) console.log(`  program: ${program.name} (${program.id}) hash ${program.hash || 'none'}`);
  const audit = core.closedLoopAudit(blocks, homePose, Kin, lib.MOTIONS, lib);
  if (!audit.ok) throw new Error(`closed-loop audit failed:\n- ${audit.issues.join('\n- ')}`);
  if (audit.warnings?.length) console.log(`Closed-loop warning: ${audit.warnings.join('; ')}`);
  console.log('Closed-loop audit: PASS (motion window starts and ends at HOME)');
  if (!opts.live) {
    console.log('Dry-run only. Add --live --i-am-at-rig to execute.');
    return;
  }
  await runLive(opts, blocks, homePose, landingPose, config, lib, core);
})().catch((err) => {
  console.error(`[workspace:session] ${err.message}`);
  process.exit(1);
});
