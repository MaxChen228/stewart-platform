export function motionById(motions, id) {
  return motions.find((m) => m.id === id) || null;
}

export function normalizeBlocks(config, motions, motionLib) {
  const raw = config?.uiState?.workspace?.blocks || [];
  return normalizeBlockList(raw, motions, motionLib);
}

export function normalizeBlockList(raw, motions, motionLib) {
  return (Array.isArray(raw) ? raw : [])
    .filter((block) => block && motionById(motions, block.id))
    .map((block) => {
      const def = motionById(motions, block.id);
      const params = typeof motionLib.motionParams === 'function'
        ? motionLib.motionParams(def, block.params)
        : { ...motionLib.motionDefaults(def), ...(block.params || {}) };
      return {
        id: def.id,
        params,
        loops: motionLib.clampLoops(block.loops),
      };
    });
}

export function serializeBlocks(blocks, motionLib) {
  return blocks.map((block) => ({
    id: block.id,
    params: { ...(block.params || {}) },
    loops: motionLib.clampLoops(block.loops),
  }));
}

export function slugifyProgramName(name) {
  const slug = String(name || 'program')
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-+|-+$/g, '')
    .slice(0, 36);
  return slug || 'program';
}

export function stableJson(value) {
  if (Array.isArray(value)) return `[${value.map(stableJson).join(',')}]`;
  if (value && typeof value === 'object') {
    return `{${Object.keys(value).sort().map((k) => `${JSON.stringify(k)}:${stableJson(value[k])}`).join(',')}}`;
  }
  return JSON.stringify(value);
}

export function hashString(input) {
  let h = 2166136261;
  const s = String(input || '');
  for (let i = 0; i < s.length; i++) {
    h ^= s.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  return (h >>> 0).toString(16).padStart(8, '0');
}

export function programHash(program, motionLib) {
  return hashString(stableJson(serializeBlocks(program?.blocks || [], motionLib)));
}

export function workspaceProgramMeta({ program = null, blocks = [], name = 'Scratch Draft', motions = [], motionLib } = {}) {
  const blockList = program?.blocks || blocks || [];
  const hash = programHash({ blocks: blockList }, motionLib);
  const saved = !!program;
  return {
    id: saved ? program.id : `scratch_${hash}`,
    name: saved ? (program.name || program.id) : (String(name || '').trim() || 'Scratch Draft'),
    hash,
    blocks: blockList.length,
    durationS: totalSeconds(blockList, motions, motionLib),
    draft: !saved,
  };
}

export function normalizePrograms(config, motions, motionLib) {
  const raw = config?.uiState?.workspace?.programs || [];
  return (Array.isArray(raw) ? raw : [])
    .filter((program) => program && typeof program.id === 'string')
    .map((program) => ({
      id: program.id,
      name: typeof program.name === 'string' && program.name.trim() ? program.name.trim() : program.id,
      notes: typeof program.notes === 'string' ? program.notes : '',
      blocks: normalizeBlockList(program.blocks, motions, motionLib),
      hash: typeof program.hash === 'string' ? program.hash : null,
      createdAt: typeof program.createdAt === 'string' ? program.createdAt : null,
      updatedAt: typeof program.updatedAt === 'string' ? program.updatedAt : null,
    }));
}

export function serializePrograms(programs, motionLib) {
  return (Array.isArray(programs) ? programs : []).map((program) => ({
    id: program.id,
    name: program.name,
    notes: program.notes || '',
    blocks: serializeBlocks(program.blocks || [], motionLib),
    hash: program.hash || programHash(program, motionLib),
    createdAt: program.createdAt || null,
    updatedAt: program.updatedAt || null,
  }));
}

export function programDuration(program, motions, motionLib) {
  return totalSeconds(program?.blocks || [], motions, motionLib);
}

export function totalSeconds(blocks, motions, motionLib) {
  return blocks.reduce((sum, block) => {
    const def = motionById(motions, block.id);
    if (!def) return sum;
    return sum + motionLib.motionPeriod(def, block.params) * motionLib.clampLoops(block.loops);
  }, 0);
}

export function normalizeWorkspaceTiming(config = {}) {
  const defaults = config?.trialDefaults || {};
  const ms = (key, fallback, min, max) => {
    const v = Number(defaults[key]);
    const n = Number.isFinite(v) ? v : fallback;
    return Math.max(min, Math.min(max, n));
  };
  return {
    warmupMs: ms('warmupMs', 1000, 0, 30000),
    homeMs: ms('homeMs', 1500, 300, 60000),
    landMs: ms('landMs', 1500, 300, 60000),
  };
}

export function lerpPose(from, to, u, motionLib = {}) {
  const s = typeof motionLib.smoothstep === 'function'
    ? motionLib.smoothstep(u)
    : Math.max(0, Math.min(1, u));
  return from.map((v, i) => v + (to[i] - v) * s);
}

export function workspacePreviewTimeline({
  blocks = [],
  startPose,
  homePose,
  landingPose,
  motions,
  motionLib,
  kin = null,
  warmupSec = 1.0,
  takeoffSec = 1.5,
  closeSec = 1.2,
  landSec = 1.5,
  smokeSec = 0.8,
} = {}) {
  const start = Array.isArray(startPose) && startPose.length === 6 ? startPose : homePose;
  const home = Array.isArray(homePose) && homePose.length === 6 ? homePose : start;
  const landing = Array.isArray(landingPose) && landingPose.length === 6 ? landingPose : start;
  const segments = [];
  let cursor = 0;
  const add = (segment) => {
    const duration = Math.max(0.05, Number(segment.duration) || 0);
    segments.push({ ...segment, t0: cursor, duration });
    cursor += duration;
  };

  if (Number(warmupSec) > 0) add({ kind: 'warmup', label: 'HOLD WARMUP', from: start, to: start, duration: warmupSec });
  add({ kind: 'takeoff', label: 'TAKE OFF', from: start, to: home, duration: takeoffSec });
  if (blocks.length) {
    for (const [index, block] of blocks.entries()) {
      const def = motionById(motions, block.id);
      if (!def) continue;
      const params = typeof motionLib.motionParams === 'function'
        ? motionLib.motionParams(def, block.params)
        : { ...motionLib.motionDefaults(def), ...(block.params || {}) };
      const per = motionLib.motionPeriod(def, params);
      add({
        kind: 'block',
        label: `${index + 1}. ${def.label}`,
        block,
        def,
        params,
        loops: motionLib.clampLoops(block.loops),
        duration: per * motionLib.clampLoops(block.loops),
      });
    }
  } else {
    add({ kind: 'smoke', label: 'HOME SMOKE', from: home, to: home, duration: smokeSec });
  }
  add({ kind: 'close', label: 'CLOSE LOOP', from: home, to: home, duration: closeSec });
  add({ kind: 'land', label: 'LAND', from: home, to: landing, duration: landSec });

  const sample = (time) => {
    const total = Math.max(0.05, cursor);
    const t = Math.max(0, Math.min(total, Number(time) || 0));
    const segment = segments.find((s) => t <= s.t0 + s.duration) || segments[segments.length - 1];
    const local = Math.max(0, t - segment.t0);
    const frac = segment.duration > 0 ? Math.max(0, Math.min(1, local / segment.duration)) : 1;
    let pose;
    if (segment.kind === 'block') {
      const sampleAt = motionLib.motionPoseAt(segment.def, segment.params, local, home, kin);
      pose = sampleAt.pose;
    } else {
      pose = lerpPose(segment.from, segment.to, frac, motionLib);
    }
    const ik = kin && typeof kin.ik === 'function' ? kin.ik(pose) : { valid: true };
    return {
      pose,
      ok: !!ik.valid,
      t,
      total,
      local,
      frac,
      segment,
      label: segment.label,
    };
  };

  return { duration: cursor, segments, sample };
}

export function closedLoopAudit(blocks, homePose, kin, motions, motionLib) {
  const issues = [];
  const warnings = [];
  if (!blocks.length) warnings.push('no motion blocks; session will validate TAKE OFF → HOME → LAND');
  for (const [i, block] of blocks.entries()) {
    const def = motionById(motions, block.id);
    if (!def) {
      issues.push(`block ${i + 1} has unknown motion: ${block.id}`);
      continue;
    }
    const params = typeof motionLib.motionParams === 'function'
      ? motionLib.motionParams(def, block.params)
      : { ...motionLib.motionDefaults(def), ...(block.params || {}) };
    const loops = motionLib.clampLoops(block.loops);
    const per = motionLib.motionPeriod(def, params);
    if (!Number.isFinite(per) || per <= 0) issues.push(`block ${i + 1} has invalid period`);
    const start = motionLib.motionPoseAt(def, params, 0, homePose, kin);
    const end = motionLib.motionPoseAt(def, params, per * loops, homePose, kin);
    const startErr = Math.max(...start.pose.map((v, k) => Math.abs(v - homePose[k])));
    const endErr = Math.max(...end.pose.map((v, k) => Math.abs(v - homePose[k])));
    if (!start.ok || !end.ok) issues.push(`block ${i + 1} endpoint IK invalid`);
    if (startErr > 1e-6 || endErr > 1e-6) issues.push(`block ${i + 1} endpoints are not HOME`);
  }
  return {
    ok: issues.length === 0,
    issues,
    warnings,
  };
}

export function blockMeta(block, motions, motionLib) {
  const def = motionById(motions, block.id);
  return def ? `${motionLib.clampLoops(block.loops)} loop · ${motionLib.formatMotionParams(def, block.params)}` : 'unknown motion';
}
