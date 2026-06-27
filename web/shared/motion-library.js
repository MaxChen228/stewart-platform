const TAU = Math.PI * 2;

export const MOTION_HZ = 30;

export function smoothstep(x) {
  x = Math.max(0, Math.min(1, x));
  return x * x * (3 - 2 * x);
}

export function cycleEnvelope(t, per) {
  return per > 0 ? Math.sin(Math.PI * ((t % per) / per)) ** 2 : 1;
}

const AXIS_INDEX = { x: 0, y: 1, z: 2, roll: 3, pitch: 4, yaw: 5 };
const ZERO_POSE = [0, 0, 0, 0, 0, 0];

function axisPose(axis, value) {
  const pose = [...ZERO_POSE];
  pose[AXIS_INDEX[axis]] = value;
  return pose;
}

function step01(t, duration) {
  const T = Math.max(0.1, Number(duration) || 0.1);
  const ph = ((t % T) + T) % T;
  const half = T / 2;
  if (ph < half) return smoothstep(ph / half);
  return 1 - smoothstep((ph - half) / half);
}

function axisStepMotion({ id, label, axis, unit, min, max, def, step, duration = 3 }) {
  return {
    id,
    label,
    category: '基礎 / 單軸 step',
    envelope: false,
    per: (p) => p.T,
    pKeyframes: (p) => [
      { t: 0, pose: [...ZERO_POSE] },
      { t: p.T / 2, pose: axisPose(axis, p.A) },
      { t: p.T, pose: [...ZERO_POSE] },
    ],
    params: [
      { k: 'A', l: '幅度', min, max, def, step, u: unit },
      { k: 'T', l: '時長', min: 0.5, max: 20, def: duration, step: 0.5, u: 's' },
    ],
    fn: (t, p) => axisPose(axis, p.A * step01(t, p.T)),
  };
}

function axisSineMotion({ id, label, axis, unit, min, max, def, step, period = 4 }) {
  return {
    id,
    label,
    category: '基礎 / 單軸 sine',
    per: (p) => p.T,
    params: [
      { k: 'A', l: '幅度', min, max, def, step, u: unit },
      { k: 'T', l: '週期', min: 1, max: 20, def: period, step: 0.5, u: 's' },
    ],
    fn: (t, p) => axisPose(axis, p.A * Math.sin(TAU * t / p.T)),
  };
}

const BASIC_STEP_MOTIONS = [
  axisStepMotion({ id: 'z-step', label: 'Z step / heave', axis: 'z', unit: 'mm', min: -30, max: 30, def: 10, step: 1 }),
  axisStepMotion({ id: 'x-step', label: 'X step / surge', axis: 'x', unit: 'mm', min: -30, max: 30, def: 10, step: 1 }),
  axisStepMotion({ id: 'y-step', label: 'Y step / sway', axis: 'y', unit: 'mm', min: -30, max: 30, def: 10, step: 1 }),
  axisStepMotion({ id: 'roll-step', label: 'Roll step', axis: 'roll', unit: 'deg', min: -12, max: 12, def: 5, step: 0.5 }),
  axisStepMotion({ id: 'pitch-step', label: 'Pitch step', axis: 'pitch', unit: 'deg', min: -12, max: 12, def: 5, step: 0.5 }),
  axisStepMotion({ id: 'yaw-step', label: 'Yaw step', axis: 'yaw', unit: 'deg', min: -30, max: 30, def: 12, step: 1 }),
];

const BASIC_SINE_MOTIONS = [
  axisSineMotion({ id: 'z-sine', label: 'Z sine / heave', axis: 'z', unit: 'mm', min: -25, max: 25, def: 8, step: 1 }),
  axisSineMotion({ id: 'x-sine', label: 'X sine / surge', axis: 'x', unit: 'mm', min: -25, max: 25, def: 8, step: 1 }),
  axisSineMotion({ id: 'y-sine', label: 'Y sine / sway', axis: 'y', unit: 'mm', min: -25, max: 25, def: 8, step: 1 }),
  axisSineMotion({ id: 'roll-sine', label: 'Roll sine', axis: 'roll', unit: 'deg', min: -10, max: 10, def: 4, step: 0.5 }),
  axisSineMotion({ id: 'pitch-sine', label: 'Pitch sine', axis: 'pitch', unit: 'deg', min: -10, max: 10, def: 4, step: 0.5 }),
  axisSineMotion({ id: 'yaw-sine', label: 'Yaw sine', axis: 'yaw', unit: 'deg', min: -25, max: 25, def: 10, step: 1 }),
];

export const MOTIONS = [
  {
    id: 'hold',
    label: '保持 hold',
    category: '基礎 / 時間',
    envelope: false,
    per: (p) => p.T,
    params: [
      { k: 'T', l: '時間', min: 0.5, max: 30, def: 3, step: 0.5, u: 's' },
    ],
    fn: () => [...ZERO_POSE],
  },
  ...BASIC_STEP_MOTIONS,
  ...BASIC_SINE_MOTIONS,
  {
    id: 'helix',
    label: '螺旋升降',
    category: '組合 / 軌跡',
    per: (p) => p.T,
    params: [
      { k: 'r', l: '半徑', min: 0, max: 40, def: 15, step: 1, u: 'mm' },
      { k: 'zlo', l: '低點', min: -15, max: 0, def: -8, step: 1, u: 'mm' },
      { k: 'zhi', l: '高點', min: 0, max: 30, def: 8, step: 1, u: 'mm' },
      { k: 'T', l: '週期', min: 1, max: 20, def: 6, step: 0.5, u: 's' },
      { k: 'dir', l: '方向', min: -1, max: 1, def: 1, step: 2, u: '' },
    ],
    fn: (t, p) => {
      const w = TAU / p.T;
      const th = p.dir * w * t;
      const z = p.zlo + (p.zhi - p.zlo) * (0.5 - 0.5 * Math.cos(w * t));
      return [p.r * Math.cos(th), p.r * Math.sin(th), z, 0, 0, 0];
    },
  },
  {
    id: 'poke',
    label: '法向推擠回彈',
    category: '組合 / 事件',
    per: (p) => p.push + p.ret + p.rest,
    params: [
      { k: 'd', l: '距離', min: 0, max: 30, def: 18, step: 1, u: 'mm' },
      { k: 'push', l: '推時長', min: 0.1, max: 1, def: 0.25, step: 0.05, u: 's' },
      { k: 'ret', l: '回時長', min: 0.1, max: 2, def: 0.6, step: 0.05, u: 's' },
      { k: 'rest', l: '間隔', min: 0, max: 3, def: 0.6, step: 0.1, u: 's' },
    ],
    fn: (t, p) => {
      const T = p.push + p.ret + p.rest;
      const ph = t % T;
      let a = 0;
      if (ph < p.push) a = smoothstep(ph / p.push);
      else if (ph < p.push + p.ret) a = 1 - smoothstep((ph - p.push) / p.ret);
      return [0, 0, p.d * a, 0, 0, 0];
    },
  },
  {
    id: 'yaw',
    label: '偏航擺盪',
    category: '組合 / 軸向',
    per: (p) => p.T,
    params: [
      { k: 'A', l: '幅度', min: 0, max: 60, def: 30, step: 1, u: 'deg' },
      { k: 'T', l: '週期', min: 0.5, max: 10, def: 3, step: 0.5, u: 's' },
      { k: 'sym', l: '雙向', min: 0, max: 1, def: 0, step: 1, u: '' },
    ],
    fn: (t, p) => {
      const w = TAU / p.T;
      const yaw = p.sym ? p.A * Math.sin(w * t) : p.A * (0.5 - 0.5 * Math.cos(w * t));
      return [0, 0, 0, 0, 0, yaw];
    },
  },
  {
    id: 'coning',
    label: '進動 coning',
    category: '組合 / 姿態',
    per: (p) => p.T,
    params: [
      { k: 'A', l: '傾角', min: 0, max: 12, def: 6, step: 0.5, u: 'deg' },
      { k: 'T', l: '週期', min: 1, max: 12, def: 4, step: 0.5, u: 's' },
      { k: 'dir', l: '方向', min: -1, max: 1, def: 1, step: 2, u: '' },
    ],
    fn: (t, p) => {
      const w = TAU / p.T;
      const th = p.dir * w * t;
      return [0, 0, 0, p.A * Math.cos(th), p.A * Math.sin(th), 0];
    },
  },
  {
    id: 'idle',
    label: '漂浮 idle',
    category: '組合 / 軌跡',
    per: (p) => p.T,
    params: [
      { k: 'tr', l: '平移範圍', min: 0, max: 20, def: 8, step: 1, u: 'mm' },
      { k: 'rr', l: '旋轉範圍', min: 0, max: 12, def: 4, step: 0.5, u: 'deg' },
      { k: 'spd', l: '速度', min: 0.2, max: 3, def: 1, step: 0.1, u: 'x' },
      { k: 'T', l: '週期', min: 4, max: 60, def: 16, step: 1, u: 's' },
    ],
    fn: (t, p) => {
      const s = p.spd;
      const d = (f1, f2, ph) => 0.5 * Math.sin(s * f1 * t) + 0.5 * Math.sin(s * f2 * t + ph);
      return [
        p.tr * d(0.17, 0.31, 1.3),
        p.tr * d(0.13, 0.27, 2.1),
        p.tr * 0.6 * d(0.11, 0.23, 0.7),
        p.rr * d(0.19, 0.29, 0.4),
        p.rr * d(0.23, 0.37, 1.9),
        p.rr * d(0.15, 0.33, 2.7),
      ];
    },
  },
  {
    id: 'fig8',
    label: '8字 Lissajous',
    category: '組合 / 軌跡',
    per: (p) => p.T,
    params: [
      { k: 'ax', l: 'X幅', min: 0, max: 40, def: 20, step: 1, u: 'mm' },
      { k: 'ay', l: 'Y幅', min: 0, max: 40, def: 15, step: 1, u: 'mm' },
      { k: 'ratio', l: '頻比', min: 1, max: 4, def: 2, step: 1, u: 'x' },
      { k: 'T', l: '週期', min: 1, max: 15, def: 6, step: 0.5, u: 's' },
    ],
    fn: (t, p) => {
      const w = TAU / p.T;
      return [p.ax * Math.sin(w * t), p.ay * Math.sin(p.ratio * w * t), 0, 0, 0, 0];
    },
  },
  {
    id: 'buoy',
    label: '海浪 buoy',
    category: '組合 / 軌跡',
    per: (p) => p.T,
    params: [
      { k: 'A', l: '傾角', min: 0, max: 12, def: 5, step: 0.5, u: 'deg' },
      { k: 'H', l: '起伏', min: 0, max: 25, def: 4, step: 1, u: 'mm' },
      { k: 'T', l: '週期', min: 1, max: 12, def: 5, step: 0.5, u: 's' },
      { k: 'dir', l: '方向', min: -1, max: 1, def: 1, step: 2, u: '' },
    ],
    fn: (t, p) => {
      const w = TAU / p.T;
      const th = p.dir * w * t;
      return [0, 0, p.H * Math.sin(w * t), p.A * Math.cos(th), p.A * Math.sin(th), 0];
    },
  },
  {
    id: 'heartbeat',
    label: '心跳 heartbeat',
    category: '組合 / 事件',
    per: (p) => p.T,
    params: [
      { k: 'd', l: '強度', min: 0, max: 25, def: 12, step: 1, u: 'mm' },
      { k: 'T', l: '節律', min: 0.6, max: 3, def: 1.2, step: 0.1, u: 's' },
    ],
    fn: (t, p) => {
      const ph = t % p.T;
      const w = 0.05 * p.T;
      const g = (tc, amp) => amp * Math.exp(-(((ph - tc) / w) ** 2));
      return [0, 0, p.d * (g(0.08 * p.T, 1.0) + g(0.22 * p.T, 0.6)), 0, 0, 0];
    },
  },
];

export function motionDefaults(def) {
  return Object.fromEntries(def.params.map((param) => [param.k, param.def]));
}

export function motionParams(def, raw = {}) {
  return Object.fromEntries(def.params.map((param) => {
    const n = Number(raw?.[param.k]);
    const fallback = Number(param.def);
    const value = Number.isFinite(n) ? n : fallback;
    return [param.k, Math.max(param.min, Math.min(param.max, value))];
  }));
}

export function motionPeriod(def, params = motionDefaults(def)) {
  if (!def) return 0;
  const per = Number(def.per(motionParams(def, params)));
  return Number.isFinite(per) && per > 0 ? per : 0;
}

export function motionPoseAt(def, params, t, basePose, kin, forceHome = false) {
  const p = motionParams(def, params);
  const per = motionPeriod(def, p) || 0.1;
  const tCycle = per > 0 ? t % per : t;
  const frac = per > 0 ? tCycle / per : 0;
  const env = forceHome ? 0 : def.envelope === false ? 1 : cycleEnvelope(t, per);
  const d = def.fn(t, p).map((v) => v * env);
  const pose = basePose.map((h, i) => h + d[i]);
  const ok = kin && typeof kin.ik === 'function' ? !!kin.ik(pose).valid : true;
  return { pose, per, tCycle, frac, ok };
}

export function formatMotionParams(def, params) {
  const normalized = motionParams(def, params);
  return def.params.map((p) => `${p.l}${Number(normalized[p.k])}${p.u}`).join(' · ');
}

export function clampLoops(value) {
  return Math.max(1, Math.min(999, Number.parseInt(value, 10) || 1));
}
