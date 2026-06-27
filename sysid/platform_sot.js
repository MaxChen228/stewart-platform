'use strict';

// Shared platform configuration schema/defaults.
// Mutable values live in server-side sysid/config/platform.json and are exposed
// by /api/platform-config. This file is only the schema/default adapter used by
// server, UI, and sysid tools.
(function () {
const DEFAULT_PLATFORM_CONFIG = Object.freeze({
  version: 1,
  homeRelative: [0, 0, 28, 0, 0, 0],
  landingRelative: [0, 0, -10, 0, 0, 34],
  followLimits: { vmaxT: 60, vmaxR: 45 },
  trialDefaults: {
    profile: 'heave-step',
    heave: 60,
    zBias: 0,
    ms: 5000,
    settleMs: 800,
    warmupMs: 1000,
    homeMs: 1500,
    followSettleMs: 800,
    landMs: 1500,
    safeLand: true,
  },
  uiState: {
    poseTargetRelative: [0, 0, 28, 0, 0, 0],
    poseMoveMs: 1500,
    motion: {
      selectedId: 'helix',
      loops: 0,
      paramsById: {},
    },
    workspace: {
      blocks: [],
      currentProgramId: 'scratch',
      programs: [],
    },
    disturbance: {
      randomSev: 3,
    },
    recorderName: 'run',
  },
  safety: {
    requireHumanForLive: true,
    requireOk6: true,
    purePReference: [1024, 0, 0, 0],
    herrBackstopDeg: 15,
  },
});

function finitePose(p) {
  return Array.isArray(p) && p.length === 6 && p.every(Number.isFinite);
}

function clone(v) {
  return JSON.parse(JSON.stringify(v));
}

function mergeConfig(input = {}) {
  const out = clone(DEFAULT_PLATFORM_CONFIG);
  if (finitePose(input.homeRelative)) out.homeRelative = input.homeRelative.map(Number);
  if (finitePose(input.landingRelative)) out.landingRelative = input.landingRelative.map(Number);
  if (input.followLimits && typeof input.followLimits === 'object') {
    for (const k of ['vmaxT', 'vmaxR']) {
      const v = Number(input.followLimits[k]);
      if (Number.isFinite(v) && v > 0) out.followLimits[k] = v;
    }
  }
  if (input.trialDefaults && typeof input.trialDefaults === 'object') {
    const t = input.trialDefaults;
    if (typeof t.profile === 'string') out.trialDefaults.profile = t.profile;
    for (const k of ['heave', 'zBias', 'ms', 'settleMs', 'warmupMs', 'homeMs', 'followSettleMs', 'landMs']) {
      const v = Number(t[k]);
      if (Number.isFinite(v)) out.trialDefaults[k] = v;
    }
    if (typeof t.safeLand === 'boolean') out.trialDefaults.safeLand = t.safeLand;
  }
  if (input.uiState && typeof input.uiState === 'object') {
    const u = input.uiState;
    if (finitePose(u.poseTargetRelative)) out.uiState.poseTargetRelative = u.poseTargetRelative.map(Number);
    const poseMoveMs = Number(u.poseMoveMs);
    if (Number.isFinite(poseMoveMs)) out.uiState.poseMoveMs = poseMoveMs;
    if (u.motion && typeof u.motion === 'object') {
      if (typeof u.motion.selectedId === 'string') out.uiState.motion.selectedId = u.motion.selectedId;
      const loops = Number(u.motion.loops);
      if (Number.isFinite(loops)) out.uiState.motion.loops = loops;
      if (u.motion.paramsById && typeof u.motion.paramsById === 'object') {
        out.uiState.motion.paramsById = clone(u.motion.paramsById);
      }
    }
    if (u.workspace && typeof u.workspace === 'object') {
      const normalizeBlocks = (blocks) => (Array.isArray(blocks) ? blocks : [])
        .filter((b) => b && typeof b.id === 'string')
        .map((b) => ({
          id: b.id,
          params: b.params && typeof b.params === 'object' ? clone(b.params) : {},
          loops: Number.isFinite(Number(b.loops)) ? Number(b.loops) : 1,
        }));
      if (Array.isArray(u.workspace.blocks)) out.uiState.workspace.blocks = normalizeBlocks(u.workspace.blocks);
      if (typeof u.workspace.currentProgramId === 'string') out.uiState.workspace.currentProgramId = u.workspace.currentProgramId;
      if (Array.isArray(u.workspace.programs)) {
        out.uiState.workspace.programs = u.workspace.programs
          .filter((p) => p && typeof p.id === 'string')
          .map((p) => ({
            id: p.id,
            name: typeof p.name === 'string' && p.name.trim() ? p.name.trim() : p.id,
            notes: typeof p.notes === 'string' ? p.notes : '',
            blocks: normalizeBlocks(p.blocks),
            hash: typeof p.hash === 'string' ? p.hash : null,
            createdAt: typeof p.createdAt === 'string' ? p.createdAt : null,
            updatedAt: typeof p.updatedAt === 'string' ? p.updatedAt : null,
          }));
      }
    }
    if (u.disturbance && typeof u.disturbance === 'object') {
      const d = u.disturbance;
      const randomSev = Number.isFinite(Number(d.randomSev)) ? Number(d.randomSev) : Number(d.wSev);
      if (Number.isFinite(randomSev)) out.uiState.disturbance.randomSev = Math.max(0.5, Math.min(8, randomSev));
    }
    if (typeof u.recorderName === 'string') out.uiState.recorderName = u.recorderName;
  }
  if (input.safety && typeof input.safety === 'object') {
    const s = input.safety;
    for (const k of ['requireHumanForLive', 'requireOk6']) {
      if (typeof s[k] === 'boolean') out.safety[k] = s[k];
    }
    if (finitePose(s.purePReference)) out.safety.purePReference = s.purePReference.map(Number);
    const b = Number(s.herrBackstopDeg);
    if (Number.isFinite(b) && b > 0) out.safety.herrBackstopDeg = b;
  }
  out.version = 1;
  return out;
}

function relToAbs(rel, neutralZ) {
  if (!finitePose(rel)) throw new Error('expected relative pose[6]');
  return [rel[0], rel[1], neutralZ + rel[2], rel[3], rel[4], rel[5]];
}

function absToRel(pose, neutralZ) {
  if (!finitePose(pose)) throw new Error('expected absolute pose[6]');
  return [pose[0], pose[1], pose[2] - neutralZ, pose[3], pose[4], pose[5]];
}

const api = { DEFAULT_PLATFORM_CONFIG, finitePose, mergeConfig, relToAbs, absToRel };
if (typeof module !== 'undefined' && module.exports) module.exports = api;
if (typeof window !== 'undefined') window.PlatformSoT = api;
})();
