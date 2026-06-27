'use strict';

const fs = require('fs');
const path = require('path');
const Kin = require('./kin');
const PlatformSoT = require('./platform_sot');

const ROOT = path.resolve(__dirname, '..');
const MOTION_LIB = path.join(ROOT, 'web', 'shared', 'motion-library.js');
const WORKSPACE_CORE = path.join(ROOT, 'web', 'shared', 'workspace-core.js');

let sharedModules = null;

async function loadEsm(file) {
  const source = fs.readFileSync(file, 'utf8');
  return import(`data:text/javascript;charset=utf-8,${encodeURIComponent(source)}`);
}

async function loadWorkspaceModules() {
  if (!sharedModules) {
    const [lib, core] = await Promise.all([loadEsm(MOTION_LIB), loadEsm(WORKSPACE_CORE)]);
    sharedModules = { lib, core };
  }
  return sharedModules;
}

function sleep(ms, signal = null) {
  const end = Date.now() + Math.max(0, Number(ms) || 0);
  return new Promise((resolve, reject) => {
    const tick = () => {
      if (signal?.aborted) return reject(new Error('aborted'));
      const left = end - Date.now();
      if (left <= 0) return resolve();
      setTimeout(tick, Math.min(80, left));
    };
    tick();
  });
}

function poseLine(pose) {
  return pose.map((x) => Number(x || 0).toFixed(3)).join(' ');
}

function commandPose(pose, condition) {
  const out = pose.map(Number);
  if (Number.isFinite(condition.zBias) && condition.zBias !== 0) out[2] += condition.zBias;
  return out;
}

function runIdFromRecordingPath(p) {
  const name = String(p || '').split('/').pop() || '';
  return name.endsWith('.jsonl') ? name.slice(0, -6) : null;
}

function slug(value) {
  return String(value || 'workspace')
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '_')
    .replace(/^_+|_+$/g, '')
    .slice(0, 48) || 'workspace';
}

function currentProgram(config) {
  const ws = config?.uiState?.workspace || {};
  return (ws.programs || []).find((program) => program.id === ws.currentProgramId) || null;
}

function activePlan(config, lib, core, opts = {}) {
  if (Array.isArray(opts.blocks)) {
    return {
      program: opts.program || null,
      blocks: core.normalizeBlockList(opts.blocks, lib.MOTIONS, lib),
    };
  }
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

function programMeta(config, blocks, lib, core, opts = {}) {
  if (opts.program?.id) return opts.program;
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
    draft: false,
  };
}

function runCondition(opts = {}, config = {}) {
  const timing = config?.trialDefaults || {};
  const normalized = typeof opts.normalizeTiming === 'function' ? opts.normalizeTiming(config) : null;
  const condition = {
    zBias: Number.isFinite(Number(opts.zBias)) ? Number(opts.zBias) : Number(timing.zBias || 0),
    vmaxT: Number.isFinite(Number(opts.vmaxT)) ? Number(opts.vmaxT) : Number(config.followLimits?.vmaxT || 60),
    vmaxR: Number.isFinite(Number(opts.vmaxR)) ? Number(opts.vmaxR) : Number(config.followLimits?.vmaxR || 45),
    warmupMs: Number.isFinite(Number(opts.warmupMs)) ? Number(opts.warmupMs) : Number(normalized?.warmupMs ?? timing.warmupMs ?? 0),
    homeMs: Number.isFinite(Number(opts.homeMs)) ? Number(opts.homeMs) : Number(normalized?.homeMs ?? timing.homeMs ?? 1500),
    followSettleMs: Number.isFinite(Number(opts.followSettleMs)) ? Number(opts.followSettleMs) : Number(timing.followSettleMs ?? 800),
    landMs: Number.isFinite(Number(opts.landMs)) ? Number(opts.landMs) : Number(normalized?.landMs ?? timing.landMs ?? 1500),
    closeMs: Number.isFinite(Number(opts.closeMs)) ? Number(opts.closeMs) : 1200,
    releaseObserveMs: Number.isFinite(Number(opts.releaseObserveMs)) ? Number(opts.releaseObserveMs) : 1200,
  };
  condition.warmupMs = Math.max(0, condition.warmupMs || 0);
  condition.homeMs = Math.max(300, condition.homeMs || 1500);
  condition.followSettleMs = Math.max(0, condition.followSettleMs || 0);
  condition.landMs = Math.max(300, condition.landMs || 1500);
  condition.closeMs = Math.max(300, condition.closeMs || 1200);
  condition.releaseObserveMs = Math.max(0, condition.releaseObserveMs || 0);
  return Object.fromEntries(Object.entries(condition).filter(([, v]) => Number.isFinite(v)));
}

function describePlan(blocks, homePose, landingPose, lib, core, condition = null, executionMode = null, blockModes = null) {
  const lines = [];
  lines.push('Workspace plan');
  lines.push(`  blocks: ${blocks.length}`);
  lines.push(`  motion window: ${core.totalSeconds(blocks, lib.MOTIONS, lib).toFixed(1)}s`);
  lines.push(`  start/end: HOME [${homePose.map((x) => x.toFixed(1)).join(', ')}]`);
  lines.push(`  landing: [${landingPose.map((x) => x.toFixed(1)).join(', ')}]`);
  blocks.forEach((b, i) => {
    const def = core.motionById(lib.MOTIONS, b.id);
    const mode = Array.isArray(blockModes) ? ` · ${blockModes[i] || 'hold'}` : '';
    lines.push(`  ${i + 1}. ${def.label} x${b.loops} · ${lib.formatMotionParams(def, b.params)}${mode}`);
  });
  if (condition) {
    lines.push(`  condition: zBias=${condition.zBias || 0}mm · VF ${condition.vmaxT}/${condition.vmaxR} · warmup ${condition.warmupMs || 0}ms · takeoff ${condition.homeMs || 0}ms · follow settle ${condition.followSettleMs || 0}ms · land ${condition.landMs || 0}ms`);
  }
  if (executionMode) {
    const label = executionMode === 'hold'
      ? 'HOLD/P only (no FOLLOW/PF)'
      : executionMode === 'p'
        ? 'P waypoint blocks (no FOLLOW/PF)'
      : executionMode === 'follow'
        ? 'FOLLOW/PF streaming'
        : 'hybrid P waypoints + FOLLOW/PF streaming';
    lines.push(`  execution: ${label}`);
  }
  return lines;
}

function blockDurationMs(block, lib, core) {
  const def = core.motionById(lib.MOTIONS, block.id);
  if (!def) throw new Error(`unknown motion: ${block.id}`);
  return lib.motionPeriod(def, block.params) * block.loops * 1000;
}

function poseDeltaMax(a, b) {
  return Math.max(...a.map((v, i) => Math.abs(Number(v || 0) - Number(b[i] || 0))));
}

function blockNeedsStreaming(block, homePose, lib, core) {
  const def = core.motionById(lib.MOTIONS, block.id);
  if (!def) throw new Error(`unknown motion: ${block.id}`);
  const per = lib.motionPeriod(def, block.params);
  const total = per * block.loops;
  const samples = Math.max(3, Math.min(80, Math.ceil(total * 10)));
  for (let i = 0; i <= samples; i++) {
    const t = total * i / samples;
    const sample = lib.motionPoseAt(def, block.params, t, homePose, Kin);
    if (!sample.ok) return true;
    if (poseDeltaMax(sample.pose, homePose) > 1e-4) return true;
  }
  return false;
}

function blockPKeyframes(block, lib, core) {
  const def = core.motionById(lib.MOTIONS, block.id);
  if (!def || typeof def.pKeyframes !== 'function') return null;
  const params = lib.motionParams(def, block.params);
  const frames = def.pKeyframes(params);
  if (!Array.isArray(frames) || frames.length < 2) return null;
  const per = lib.motionPeriod(def, params);
  const normalized = frames.map((frame) => ({
    t: Number(frame?.t),
    pose: Array.isArray(frame?.pose) ? frame.pose.map(Number) : null,
  }));
  if (!normalized.every((frame) => Number.isFinite(frame.t) && frame.pose?.length === 6 && frame.pose.every(Number.isFinite))) return null;
  if (normalized[0].t !== 0 || Math.abs(normalized[normalized.length - 1].t - per) > 1e-6) return null;
  for (let i = 1; i < normalized.length; i++) {
    if (normalized[i].t <= normalized[i - 1].t) return null;
  }
  return normalized;
}

function blockExecutionMode(block, homePose, lib, core) {
  const def = core.motionById(lib.MOTIONS, block.id);
  if (!def) throw new Error(`unknown motion: ${block.id}`);
  if (blockPKeyframes(block, lib, core)) return 'p';
  return blockNeedsStreaming(block, homePose, lib, core) ? 'follow' : 'hold';
}

function planExecutionMode(blockModes) {
  if (!blockModes.some((mode) => mode !== 'hold')) return 'hold';
  if (!blockModes.some((mode) => mode !== 'p')) return 'p';
  if (!blockModes.some((mode) => mode !== 'follow')) return 'follow';
  return 'hybrid';
}

function addPose(basePose, deltaPose) {
  return basePose.map((v, i) => Number(v || 0) + Number(deltaPose[i] || 0));
}

function assertLivePacket(d, label) {
  if (!d || !Array.isArray(d.a) || d.a.length !== 6) throw new Error(`${label}: no telemetry packet`);
  if (Number(d.ok) !== 6) throw new Error(`${label}: encoder ok=${d.ok ?? 'missing'}, expected 6`);
  const ar = Number(d.ar || 0);
  if (ar > 0) {
    const ages = d.arAge;
    if (Array.isArray(ages) && ages.length === 6) {
      const stale = ages.map((v, i) => (Number(v) <= 1500 ? null : i + 1)).filter(Boolean);
      if (stale.length) throw new Error(`${label}: auto-return old motors M${stale.join(',M')}`);
    }
  }
  const txFail = Number(d.can?.txFail ?? 0);
  const cmdDrop = Number(d.sched?.cmd_drop ?? 0);
  if (txFail) throw new Error(`${label}: CAN txFail=${txFail}`);
  if (cmdDrop) throw new Error(`${label}: command drops=${cmdDrop}`);
  return d;
}

async function waitFor(deps, label, predicate, timeoutMs = 2500, signal = null) {
  const end = Date.now() + timeoutMs;
  let lastReason = '';
  while (Date.now() <= end) {
    if (signal?.aborted) throw new Error('aborted');
    const transport = deps.getTransport();
    if (transport.state !== 'connected') throw new Error(`${label}: transport ${transport.state || 'unknown'}`);
    const latest = deps.getLatest();
    const ageMs = Number(latest?._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
    if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`${label}: telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
    assertLivePacket(latest, label);
    const ok = predicate(latest);
    if (ok === true) return latest;
    lastReason = typeof ok === 'string' ? ok : 'condition not reached';
    await sleep(120, signal);
  }
  throw new Error(`${label}: ${lastReason}`);
}

function healthy(deps, label) {
  const transport = deps.getTransport();
  if (transport.state !== 'connected') throw new Error(`${label}: transport ${transport.state || 'unknown'}`);
  const latest = deps.getLatest();
  const ageMs = Number(latest?._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
  if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`${label}: telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
  return assertLivePacket(latest, label);
}

function preflightSnapshot(deps) {
  const transport = deps.getTransport();
  if (transport.state !== 'connected') throw new Error(`transport not connected (${transport.state || 'unknown'})`);
  const rec = deps.getRecorder();
  if (rec.recording) throw new Error('recorder already running');
  const latest = deps.getLatest();
  const ageMs = Number(latest?._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
  if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
  assertLivePacket(latest, 'preflight');
}

async function buildWorkspacePlan(config, opts = {}) {
  const { lib, core } = await loadWorkspaceModules();
  const mergedConfig = PlatformSoT.mergeConfig(config || {});
  const plan = activePlan(mergedConfig, lib, core, opts);
  const blocks = plan.blocks;
  const homePose = PlatformSoT.relToAbs(mergedConfig.homeRelative, Kin.NEUTRAL_Z);
  const landingPose = Array.isArray(mergedConfig.landingPose)
    ? mergedConfig.landingPose
    : PlatformSoT.relToAbs(mergedConfig.landingRelative, Kin.NEUTRAL_Z);
  const condition = runCondition({ ...opts, normalizeTiming: core.normalizeWorkspaceTiming }, mergedConfig);
  const program = programMeta(mergedConfig, blocks, lib, core, opts);
  const audit = core.closedLoopAudit(blocks, homePose, Kin, lib.MOTIONS, lib);
  const blockModes = blocks.map((block) => blockExecutionMode(block, homePose, lib, core));
  const result = {
    lib,
    core,
    config: mergedConfig,
    program,
    blocks,
    homePose,
    landingPose,
    condition,
    audit,
    blockModes,
  };
  result.executionMode = planExecutionMode(blockModes);
  result.description = describePlan(blocks, homePose, landingPose, lib, core, condition, result.executionMode, blockModes);
  return result;
}

async function runWorkspaceSession({ config, opts = {}, deps, signal = null }) {
  if (!deps) throw new Error('workspace executor deps required');
  const plan = await buildWorkspacePlan(config, opts);
  if (!plan.audit.ok) throw new Error(`closed-loop audit failed: ${plan.audit.issues.join('; ')}`);
  preflightSnapshot(deps);
  const sessionCondition = { ...plan.condition, executionMode: plan.executionMode, blockModes: plan.blockModes };

  const prefix = plan.program ? plan.program.id : slug(plan.config?.uiState?.workspace?.currentProgramId || 'workspace');
  const suffix = plan.condition.zBias ? `_zbias${String(plan.condition.zBias).replace('-', 'm').replace('.', 'p')}` : '';
  const label = opts.name || `${prefix}${suffix}_workspace_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}`;
  let token = null;
  let recording = false;
  let runId = null;
  let followOn = false;

  const send = (cmd) => deps.send(cmd, token);
  const ensureFollowOn = async () => {
    if (followOn) return;
    send('FOLLOW 1');
    await waitFor(deps, 'after FOLLOW on', (d) => (d.pos === 1 && d.fl === 1 ? true : `pos=${d.pos}, fl=${d.fl}`), 3500, signal);
    followOn = true;
  };
  const ensureFollowOff = async () => {
    if (!followOn) return;
    send('FOLLOW 0');
    await waitFor(deps, 'after FOLLOW off', (d) => (d.pos === 1 && d.fl === 0 ? true : `pos=${d.pos}, fl=${d.fl}`), 2500, signal);
    followOn = false;
  };
  const runPKeyframeBlock = async (block, modeLabel) => {
    const frames = blockPKeyframes(block, plan.lib, plan.core);
    if (!frames) throw new Error(`${modeLabel}: missing P keyframes`);
    for (let loop = 0; loop < block.loops; loop++) {
      for (let j = 1; j < frames.length; j++) {
        healthy(deps, modeLabel);
        const ms = Math.max(1, Math.round((frames[j].t - frames[j - 1].t) * 1000));
        send(`P ${poseLine(commandPose(addPose(plan.homePose, frames[j].pose), plan.condition))} ${ms}`);
        await sleep(ms, signal);
      }
    }
  };
  try {
    const started = deps.startSession({ label, phase: 'takeoff', program: plan.program, condition: sessionCondition });
    token = started.token;
    deps.startRecording(token, label);
    recording = true;

    send('H');
    await waitFor(deps, 'after HOLD', (d) => (
      d.pos === 1 && Array.isArray(d.hold) ? true : `pos=${d.pos}, hold=${Array.isArray(d.hold) ? 'yes' : 'no'}`
    ), 3500, signal);

    send(`VF ${plan.condition.vmaxT || plan.config.followLimits?.vmaxT || 60} ${plan.condition.vmaxR || plan.config.followLimits?.vmaxR || 45}`);
    if (plan.condition.warmupMs > 0) {
      deps.phase(token, 'holding warmup');
      await sleep(plan.condition.warmupMs, signal);
    }

    deps.phase(token, 'takeoff');
    send(`P ${poseLine(plan.homePose)} ${plan.condition.homeMs}`);
    await sleep(plan.condition.homeMs + 400, signal);
    await waitFor(deps, 'after HOME settle', (d) => (d.pos === 1 ? true : `pos=${d.pos}`), 1800, signal);

    if (plan.condition.followSettleMs > 0) {
      if (plan.executionMode === 'follow') {
        await ensureFollowOn();
        deps.phase(token, 'follow settle');
        const settleT0 = Date.now();
        while (Date.now() - settleT0 < plan.condition.followSettleMs) {
          healthy(deps, 'follow settle');
          send(`PF ${poseLine(commandPose(plan.homePose, plan.condition))}`);
          await sleep(1000 / plan.lib.MOTION_HZ, signal);
        }
      } else {
        deps.phase(token, 'hold settle');
        await sleep(plan.condition.followSettleMs, signal);
      }
    }

    deps.phase(token, 'recording');
    for (const [i, block] of plan.blocks.entries()) {
      const def = plan.core.motionById(plan.lib.MOTIONS, block.id);
      if (!def) throw new Error(`unknown motion: ${block.id}`);
      const totalMs = blockDurationMs(block, plan.lib, plan.core);
      const t0 = Date.now();
      const mode = plan.blockModes[i] || 'hold';
      deps.phase(token, `block ${i + 1}/${plan.blocks.length}`);
      if (mode === 'p') {
        await ensureFollowOff();
        await runPKeyframeBlock(block, 'P keyframe block');
      } else if (mode === 'follow') {
        await ensureFollowOn();
        while (Date.now() - t0 <= totalMs) {
          healthy(deps, 'motion block');
          const t = (Date.now() - t0) / 1000;
          const sample = plan.lib.motionPoseAt(def, block.params, t, plan.homePose, Kin);
          if (sample.ok) send(`PF ${poseLine(commandPose(sample.pose, plan.condition))}`);
          await sleep(1000 / plan.lib.MOTION_HZ, signal);
        }
      } else {
        await ensureFollowOff();
        while (Date.now() - t0 <= totalMs) {
          healthy(deps, 'hold block');
          await sleep(Math.min(80, Math.max(1, totalMs - (Date.now() - t0))), signal);
        }
      }
    }

    deps.phase(token, 'close loop');
    if (followOn) {
      const closeT0 = Date.now();
      while (Date.now() - closeT0 < plan.condition.closeMs) {
        healthy(deps, 'close loop');
        send(`PF ${poseLine(commandPose(plan.homePose, plan.condition))}`);
        await sleep(1000 / plan.lib.MOTION_HZ, signal);
      }
      await ensureFollowOff();
    } else if (plan.executionMode === 'hybrid' || plan.executionMode === 'p') {
      send(`P ${poseLine(commandPose(plan.homePose, plan.condition))} ${plan.condition.closeMs}`);
      await sleep(plan.condition.closeMs, signal);
    } else {
      await sleep(plan.condition.closeMs, signal);
    }

    deps.phase(token, 'landing');
    await ensureFollowOff();
    send(`P ${poseLine(plan.landingPose)} ${plan.condition.landMs}`);
    await sleep(plan.condition.landMs + 300, signal);
    await waitFor(deps, 'after landing settle', (d) => (d.pos === 1 && d.fl === 0 ? true : `pos=${d.pos}, fl=${d.fl}`), 1800, signal);

    send('D');
    deps.phase(token, 'release');
    await sleep(plan.condition.releaseObserveMs, signal);
    const stopped = await deps.stopRecording(token, false).catch(() => ({}));
    recording = false;
    runId = runIdFromRecordingPath(stopped.path);
    deps.finishSession(token);
    if (runId) deps.analyzeRun(runId, { program: plan.program, condition: sessionCondition });
    return { label, runId, program: plan.program, condition: sessionCondition };
  } catch (err) {
    if (token) {
      if (followOn) {
        try { send(`PF ${poseLine(commandPose(plan.homePose, plan.condition))}`); } catch {}
        try { send('FOLLOW 0'); } catch {}
        followOn = false;
        await sleep(150).catch(() => {});
      }
      try { send(`P ${poseLine(plan.landingPose)} ${plan.condition.landMs}`); } catch {}
      await sleep(plan.condition.landMs + 300).catch(() => {});
      try { send('D'); } catch {}
      if (recording) await deps.stopRecording(token, false).catch(() => {});
      deps.abortSession(token, err.message);
    }
    throw err;
  }
}

module.exports = {
  loadWorkspaceModules,
  buildWorkspacePlan,
  runWorkspaceSession,
  describePlan,
  runCondition,
  poseLine,
};
