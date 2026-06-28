const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { WebSocketServer } = require('ws');
const http = require('http');
const https = require('https');
const net = require('net');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { execFileSync } = require('child_process');
const { NEUTRAL_Z, solveFK, resetFK } = require('./sysid/kin');
const PlatformSoT = require('./sysid/platform_sot');
const WorkspaceExecutor = require('./sysid/workspace_executor');

const HTTP_PORT = Number(process.env.HTTP_PORT) || 3000;
const HTTPS_PORT = Number(process.env.HTTPS_PORT) || 3443;
const HTTPS_ENABLED = process.env.STEWART_HTTPS === '1' || process.env.HTTPS === '1';
const BAUD = 460800;
// 綁定位址：預設 0.0.0.0（方便站在機台旁用手機/平板驅動平台）。
// 這是會動的實體機械——若在不可信網段，設環境變數 LOOPBACK_ONLY=1 只綁本機。
const BIND_HOST = process.env.LOOPBACK_ONLY === '1' ? '127.0.0.1' : '0.0.0.0';

// ===== 傳輸：自動偵測（有線→serial、沒線→WiFi/TCP）。單一 port、liveness-gated、無 silent fallback =====
const ESP32_HOST_OVERRIDE = process.env.ESP32_HOST || '';     // 顯式指定 WiFi 主機（跳過自動學 IP）
const ESP32_PORT = Number(process.env.ESP32_PORT) || 3333;
const IP_CACHE = path.join(__dirname, '.esp32-ip');           // serial 連線時經 WIFI? 學到的 ESP32 WiFi IP（直連最可靠，避本機 Tailscale 對 .local 不穩解析）
const TRANSPORT_LOCK_PATH = path.join(os.tmpdir(), `stewart-platform-esp32-${ESP32_PORT}.lock`);
const TRANSPORT_LOCK_DISABLED = process.env.STEWART_TRANSPORT_LOCK === '0';
const SERIAL_PATTERN = /usbserial|usbmodem|SLAB|wchusbserial/i;
const LIVENESS_MS  = 5000;   // 連上後須在此時間內收到有效遙測，否則判死換路（埠開/socket 開 ≠ 會講話）
const IDLE_MS      = 2500;   // 連線中遙測中斷逾此 → 判斷線
const RECONNECT_MS = 1500;   // 換路/重連退避
const RECONNECT_MAX_MS = 12000;
const RECOVERY_WATCH_MS = 1000; // 修正 transport 狀態機卡在 down/connecting 但未排重連的情況
const TCP_CONNECT_TIMEOUT_MS = 2500;
const SERIAL_POLL_MS = 3000; // WiFi 運行時偵測 USB 插入（有線就用線）
const TCP_HB_MS    = 1000;   // 對 ESP32 心跳（韌體 failsafe 基準）
const RELEASE_HOLD_MS = Number(process.env.RELEASE_HOLD_MS) || 30000;
const RELEASE_HOLD_MAX_MS = 180000;

// 最新一筆「遙測」資料（供 REST API 用）。ack/status 不可覆蓋這裡，否則 /api/latest 會失去
// 「即時姿態」語意。
let lastTelemetry = null;
let lastTelemetryAt = 0;
let lastLine = null;
let lastParseWarningAt = 0;
const lineParseStats = {
  errors: 0,
  lastAt: 0,
  lastLen: 0,
  lastError: null,
  lastHead: null,
  lastTail: null,
};

// ===== Phase 0 系統辨識：資料記錄器 =====
// 把每筆進來的遙測（dir:"in"）與每筆送出的指令（dir:"cmd"）寫成 JSONL，
// 時間戳用 performance.now()（單調、sub-ms），供離線分析對齊。
const REC_DIR = path.join(__dirname, 'sysid', 'data');
const CONFIG_DIR = path.join(__dirname, 'sysid', 'config');
const HOME_POSE_PATH = path.join(CONFIG_DIR, 'pose-home.json');
const PLATFORM_CONFIG_PATH = path.join(CONFIG_DIR, 'platform.json');
const HTTPS_KEY_PATH = process.env.HTTPS_KEY || path.join(CONFIG_DIR, 'https-key.pem');
const HTTPS_CERT_PATH = process.env.HTTPS_CERT || path.join(CONFIG_DIR, 'https-cert.pem');
let recStream = null;
let recPath = null;
let recCount = 0;
let recT0 = 0;
let recOwner = null;

function numberEnv(name, fallback) {
  const v = Number(process.env[name]);
  return Number.isFinite(v) ? v : fallback;
}

// Always-on follow diagnostics. This is intentionally separate from the formal
// recorder: it is low-volume, survives "just restart server and try it", and
// keeps enough host-side timing to diagnose jerky FOLLOW/PF behavior.
const FOLLOW_DIAG_DIR = path.join(REC_DIR, 'follow-diagnostics');
const FOLLOW_DIAG_ENABLED = process.env.FOLLOW_DIAG !== '0';
const FOLLOW_DIAG_WINDOW_MS = 120000;
const FOLLOW_DIAG_MAX_SAMPLES = Math.max(300, Math.floor(numberEnv('FOLLOW_DIAG_MAX_SAMPLES', 5000)));
const PF_MAILBOX_MIN_MS = Math.max(0, numberEnv('PF_MAILBOX_MIN_MS', 10));
// Manual FOLLOW resampler: server owns a fixed-rate PF emission clock so browser/
// phone input jitter never becomes the physical reference cadence. Default 100 Hz.
const PF_RESAMPLE_HZ = Math.max(1, Math.min(200, Math.round(numberEnv('PF_RESAMPLE_HZ', 100))));
const PF_RESAMPLE_MS = Math.max(1, Math.round(1000 / PF_RESAMPLE_HZ));
const MANUAL_PF_MODE = `server-resample-${PF_RESAMPLE_HZ}hz`;
let followDiagStream = null;
let followDiagPath = null;
let followDiagT0 = performance.now();
let followDiagLastSummaryAt = 0;
let followDiagSeq = 0;
const followDiag = {
  startedAt: new Date().toISOString(),
  lastOp: null,
  lastPfAt: 0,
  lastPfPose: null,
  lastTeleAt: 0,
  lastFl: null,
  lastProfile: null,
  pf: [],
  tele: [],
  state: [],
  warnings: [],
};

// PF is a high-rate setpoint stream, not a chatty command. Treat it as a
// latest-wins mailbox so producers cannot build a backlog in Node, WebSocket
// clients, recorder, or the ESP32 transport. The firmware still owns the real
// motion clock and velocity limits; this only bounds host-side delivery noise.
const pfMailbox = {
  pending: null,
  timer: null,
  timerKind: null,
  received: 0,
  flushed: 0,
  coalesced: 0,
  dropped: 0,
  writeFail: 0,
  gateDrop: 0,
  invalidDrop: 0,
  transportDrop: 0,
  lastEnqueueAt: 0,
  lastFlushAt: 0,
  lastDropAt: 0,
  lastError: null,
  lastVia: null,
  lastQueuedPose: null,
  lastFlushedPose: null,
};

// Manual FOLLOW is not allowed to use browser/input events as the physical
// reference clock. Browser/phone PF messages are treated as latest intent and
// forwarded through the host latest-wins mailbox. ESP32 followStep owns the
// fixed-rate acceleration/jerk-limited reference. Workspace/session PF keeps
// its own deterministic path and bypasses this manual accounting.
const manualPf = {
  target: null,
  output: null,
  lastInputAt: 0,
  lastEmitAt: 0,
  targetUpdates: 0,
  emitted: 0,
  inputWindow: [],
  emitWindow: [],
  vmaxT: 60,
  vmaxR: 45,
  lastReason: null,
  // resampler emit-side accounting (Part A); emitDtMaxSession reset at session start
  writeFail: 0,
  transportSkip: 0,
  emitErrors: 0,   // manualPfEmit() threw (unexpected); persistent so a recovered one-shot stays visible
  emitDtMaxSession: 0,
};

// Manual PF emission scheduler (server is the fixed-rate reference clock).
// Absolute-clock setTimeout chain, not setInterval: see manualPfResamplerStart.
let manualPfTimer = null;
let manualPfNextAt = 0;   // absolute wall-clock (ms) of the next scheduled emit slot

// ===== Follow session report thresholds + state (Part B) =====
// One report per FOLLOW session (fl 0->1 .. 1->0) with a PASS/FAIL verdict, so a
// run can be confirmed after the fact via curl /api/follow/report.
const FOLLOW_SESSION_MIN_MS = numberEnv('FOLLOW_SESSION_MIN_MS', 500);
const FOLLOW_STALE_MS = numberEnv('FOLLOW_STALE_MS', 3000);
const FOLLOW_RESERVOIR_MAX = Math.max(100, Math.floor(numberEnv('FOLLOW_RESERVOIR_MAX', 2000)));
const FOLLOW_SAT_FRAC = numberEnv('FOLLOW_SAT_FRAC', 0.95);
const FOLLOW_VERDICT = {
  // Emission band tracks the resampler rate so control rate + emit rate move as ONE
  // coupled knob: raise the firmware control loop (runtime `L <ms>`) and set
  // PF_RESAMPLE_HZ to match -> this pass window follows automatically. The CAN bus
  // is not the binding constraint (150Hz F5+encoder = 47% of 500kbps per
  // `node sysid/can_budget.js --sweep`); the real limit is per-tick ctrlMaxUs,
  // which the report's `control` check measures empirically. Both band and p95 are
  // derived from the EFFECTIVE rate 1000/PF_RESAMPLE_MS (the actual setTimeout
  // cadence after ms quantization), not nominal PF_RESAMPLE_HZ -- at e.g. 150Hz the
  // period quantizes to 7ms (142.86Hz), so the band must center there to stay honest.
  // At 100Hz this reduces exactly to the prior 90-110Hz / 13ms; env overrides win.
  emitHzLo: numberEnv('FOLLOW_VERDICT_EMIT_HZ_LO', round((1000 / PF_RESAMPLE_MS) * 0.9, 1)),
  emitHzHi: numberEnv('FOLLOW_VERDICT_EMIT_HZ_HI', round((1000 / PF_RESAMPLE_MS) * 1.1, 1)),
  emitDtP95Ms: numberEnv('FOLLOW_VERDICT_EMIT_DTP95_MS', round(PF_RESAMPLE_MS * 1.3, 1)),
  emitDtMaxMs: numberEnv('FOLLOW_VERDICT_EMIT_DTMAX_MS', 50),
  missedMax: numberEnv('FOLLOW_VERDICT_MISSED_MAX', 5),
  ctrlBudgetUs: numberEnv('FOLLOW_VERDICT_CTRL_BUDGET_US', 0),   // 0 = auto (0.8 x loop period)
  satPct: numberEnv('FOLLOW_VERDICT_SAT_PCT', 20),
  lhzFloorFrac: numberEnv('FOLLOW_LHZ_FLOOR_FRAC', 0.9),
};
let lastFollowReport = null;
let followReportSeq = 0;
const followSession = {
  active: false, seq: 0, startedAt: null, startedAtMs: 0, endedAt: null,
  trigger: null, limits: null, vfChanges: [], acc: null,
};

// UI/session ownership gate. Manual pose controls and program sessions share the
// same WebSocket transport, so the server owns the final arbitration. Workspace
// execution is owned by the server-side executor; browser and CLI clients only
// request a run and observe session events.
let activeSession = null;
let sessionEvents = [];
let sessionSeq = 0;
let workspaceRun = null;

function makeSessionToken() {
  return `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
}

function publicSessionState() {
  return activeSession
    ? {
        active: true,
        owner: activeSession.owner,
        label: activeSession.label,
        phase: activeSession.phase,
        startedAt: activeSession.startedAt,
        program: activeSession.program || null,
        condition: activeSession.condition || null,
      }
    : { active: false, owner: 'manual', phase: 'idle' };
}

function broadcastSessionState() {
  broadcast(JSON.stringify({ evt: 'session', ...publicSessionState() }));
}

function pushSessionEvent(type, detail = {}) {
  const event = {
    seq: ++sessionSeq,
    at: new Date().toISOString(),
    type,
    session: publicSessionState(),
    ...detail,
  };
  sessionEvents.push(event);
  if (sessionEvents.length > 400) sessionEvents = sessionEvents.slice(-400);
  broadcast(JSON.stringify({ evt: 'session_event', event }));
  return event;
}

function publicRecOwner(owner = recOwner) {
  if (!owner) return null;
  return {
    type: owner.type || 'manual',
    label: owner.label || null,
    startedAt: owner.startedAt || null,
    program: owner.program || null,
    condition: owner.condition || null,
  };
}

function sanitizeProgramMeta(program) {
  if (!program || typeof program !== 'object') return null;
  const id = typeof program.id === 'string' ? program.id.replace(/[^a-zA-Z0-9_.-]/g, '_').slice(0, 80) : '';
  if (!id) return null;
  return {
    id,
    name: typeof program.name === 'string' ? program.name.slice(0, 120) : id,
    hash: typeof program.hash === 'string' ? program.hash.replace(/[^a-zA-Z0-9_.-]/g, '').slice(0, 40) : null,
    blocks: Number.isFinite(Number(program.blocks)) ? Number(program.blocks) : null,
    durationS: Number.isFinite(Number(program.durationS)) ? Number(program.durationS) : null,
    draft: program.draft === true,
  };
}

function sanitizeConditionMeta(condition) {
  if (!condition || typeof condition !== 'object') return null;
  const out = {};
  for (const [key, value] of Object.entries(condition)) {
    const safeKey = key.replace(/[^a-zA-Z0-9_.-]/g, '_').slice(0, 48);
    if (!safeKey) continue;
    if (typeof value === 'number' && Number.isFinite(value)) out[safeKey] = value;
    else if (typeof value === 'boolean') out[safeKey] = value;
    else if (typeof value === 'string') out[safeKey] = value.slice(0, 120);
  }
  return Object.keys(out).length ? out : null;
}

function startSessionState({ label = 'program', phase = 'starting', program = null, condition = null } = {}) {
  if (activeSession) throw new Error('session already active');
  manualPfStop('session starting');
  activeSession = {
    token: makeSessionToken(),
    owner: 'session',
    label: typeof label === 'string' ? label.slice(0, 80) : 'program',
    phase: typeof phase === 'string' ? phase.slice(0, 40) : 'starting',
    program: sanitizeProgramMeta(program),
    condition: sanitizeConditionMeta(condition),
    startedAt: new Date().toISOString(),
  };
  pushSessionEvent('session_start', {
    label: activeSession.label,
    phase: activeSession.phase,
    program: activeSession.program,
    condition: activeSession.condition,
  });
  broadcastSessionState();
  return { ...publicSessionState(), token: activeSession.token };
}

function requireSessionToken(token) {
  if (!activeSession) throw new Error('no active session');
  if (token !== activeSession.token) throw new Error('bad session token');
}

function setSessionPhase(token, phase) {
  requireSessionToken(token);
  activeSession.phase = typeof phase === 'string' ? phase.slice(0, 40) : activeSession.phase;
  pushSessionEvent('phase', { phase: activeSession.phase });
  recEvent('session_phase', { phase: activeSession.phase });
  broadcastSessionState();
  return publicSessionState();
}

function finishSessionState(token, { abort = false, reason = null } = {}) {
  requireSessionToken(token);
  clearPfMailbox(abort ? 'session aborted' : 'session finished');
  const endedSession = publicSessionState();
  activeSession = null;
  pushSessionEvent(abort ? 'session_abort' : 'session_finish', {
    reason,
    session: { ...endedSession, active: false, endedAt: new Date().toISOString() },
  });
  broadcastSessionState();
  return publicSessionState();
}

function publicRecState() {
  return { recording: !!recStream, path: recPath, lines: recCount, owner: publicRecOwner() };
}

function commandOp(cmd) {
  return String(cmd || '').trim().split(/\s+/)[0].toUpperCase();
}

function movementCommand(cmd) {
  const op = commandOp(cmd);
  return [
    'H', 'E', 'P', 'PF', 'FOLLOW',
    'U', 'W', 'K', 'KS', 'KRESET', 'V', 'VF', 'FE', 'J', 'M',
    'L', 'AR', 'A', 'C', 'OD', 'CM', 'T0', 'T1', 'T2', 'T3', 'T4', 'T5',
    'Z', 'Z0', 'Z1', 'Z2', 'Z3', 'Z4', 'Z5',
  ].includes(op);
}

function commandAllowed(cmd, token) {
  if (!activeSession) return { ok: true };
  const op = commandOp(cmd);
  if (token && token === activeSession.token) return { ok: true };
  if (op === 'D' || op === 'S') return { ok: true, emergency: true };
  if (!movementCommand(cmd)) return { ok: true };
  return { ok: false, reason: `session active: ${activeSession.label || activeSession.owner}` };
}

function markPfMailboxDrop(reason, kind = 'drop') {
  pfMailbox.dropped++;
  pfMailbox.lastDropAt = Date.now();
  pfMailbox.lastError = reason || kind;
  if (kind === 'write') pfMailbox.writeFail++;
  else if (kind === 'gate') pfMailbox.gateDrop++;
  else if (kind === 'invalid') pfMailbox.invalidDrop++;
  else if (kind === 'transport') pfMailbox.transportDrop++;
}

function clearPfMailboxTimer() {
  if (!pfMailbox.timer) return;
  if (pfMailbox.timerKind === 'immediate') clearImmediate(pfMailbox.timer);
  else clearTimeout(pfMailbox.timer);
  pfMailbox.timer = null;
  pfMailbox.timerKind = null;
}

function clearPfMailbox(reason = 'cleared') {
  clearPfMailboxTimer();
  if (pfMailbox.pending) {
    pfMailbox.pending = null;
    markPfMailboxDrop(reason, 'drop');
  }
}

function schedulePfMailboxFlush(delayMs = 0) {
  if (pfMailbox.timer) return;
  const delay = Math.max(0, Math.ceil(Number(delayMs) || 0));
  if (delay > 0) {
    pfMailbox.timerKind = 'timeout';
    pfMailbox.timer = setTimeout(flushPfMailbox, delay);
  } else {
    pfMailbox.timerKind = 'immediate';
    pfMailbox.timer = setImmediate(flushPfMailbox);
  }
}

function pfCommandCancelsMailbox(op) {
  return ['D', 'S', 'H', 'E', 'P', 'FOLLOW'].includes(op);
}

function commandStopsManualPf(cmd, op) {
  if (['D', 'S', 'H', 'E', 'P'].includes(op)) return true;
  if (op !== 'FOLLOW') return false;
  const parts = String(cmd || '').trim().split(/\s+/);
  return Number(parts[1]) === 0;
}

function pfMailboxSummary(now = Date.now()) {
  const pendingAgeMs = pfMailbox.pending ? now - pfMailbox.pending.enqueuedAt : null;
  return {
    minMs: PF_MAILBOX_MIN_MS,
    pending: !!pfMailbox.pending,
    pendingAgeMs,
    received: pfMailbox.received,
    flushed: pfMailbox.flushed,
    coalesced: pfMailbox.coalesced,
    dropped: pfMailbox.dropped,
    writeFail: pfMailbox.writeFail,
    gateDrop: pfMailbox.gateDrop,
    invalidDrop: pfMailbox.invalidDrop,
    transportDrop: pfMailbox.transportDrop,
    lastVia: pfMailbox.lastVia,
    lastError: pfMailbox.lastError,
    msSinceEnqueue: pfMailbox.lastEnqueueAt ? now - pfMailbox.lastEnqueueAt : null,
    msSinceFlush: pfMailbox.lastFlushAt ? now - pfMailbox.lastFlushAt : null,
    lastQueuedPose: pfMailbox.lastQueuedPose,
    lastFlushedPose: pfMailbox.lastFlushedPose,
  };
}

function enqueuePfCommand(cmd, sessionToken = null) {
  const pose = parsePoseCommand(cmd, 'PF');
  if (!pose) {
    markPfMailboxDrop('invalid PF pose', 'invalid');
    return { ok: false, reason: 'invalid PF pose' };
  }
  if (currentTransport.state !== 'connected') {
    const reason = `transport ${currentTransport.state || 'unknown'}`;
    markPfMailboxDrop(reason, 'transport');
    return { ok: false, reason };
  }

  const now = Date.now();
  pfMailbox.received++;
  if (pfMailbox.pending) pfMailbox.coalesced++;
  pfMailbox.pending = {
    cmd,
    pose,
    sessionToken,
    enqueuedAt: now,
    seq: pfMailbox.received,
  };
  pfMailbox.lastEnqueueAt = now;
  pfMailbox.lastQueuedPose = pose;
  observeCommand(cmd, 'pf-mailbox', sessionToken);

  const sinceFlush = pfMailbox.lastFlushAt ? now - pfMailbox.lastFlushAt : PF_MAILBOX_MIN_MS;
  schedulePfMailboxFlush(Math.max(0, PF_MAILBOX_MIN_MS - sinceFlush));
  return { ok: true, via: 'pf-mailbox' };
}

function flushPfMailbox() {
  pfMailbox.timer = null;
  pfMailbox.timerKind = null;
  const item = pfMailbox.pending;
  if (!item) return;

  const now = Date.now();
  const sinceFlush = pfMailbox.lastFlushAt ? now - pfMailbox.lastFlushAt : PF_MAILBOX_MIN_MS;
  if (sinceFlush < PF_MAILBOX_MIN_MS) {
    schedulePfMailboxFlush(PF_MAILBOX_MIN_MS - sinceFlush);
    return;
  }

  const gate = commandAllowed(item.cmd, item.sessionToken);
  if (!gate.ok) {
    pfMailbox.pending = null;
    markPfMailboxDrop(gate.reason, 'gate');
    return;
  }
  if (currentTransport.state !== 'connected') {
    pfMailbox.pending = null;
    markPfMailboxDrop(`transport ${currentTransport.state || 'unknown'}`, 'transport');
    return;
  }

  pfMailbox.pending = null;
  const wr = transport.write(item.cmd);
  if (wr.ok) {
    pfMailbox.flushed++;
    pfMailbox.lastFlushAt = Date.now();
    pfMailbox.lastVia = wr.kind;
    pfMailbox.lastError = null;
    pfMailbox.lastFlushedPose = item.pose;
    recWrite('cmd', item.cmd);
    return;
  }

  markPfMailboxDrop(wr.reason, 'write');
  recWrite('drop', { cmd: item.cmd, reason: wr.reason, transport: currentTransport, highRate: true });
}

function sendCommand(cmd, sessionToken = null) {
  const op = commandOp(cmd);
  const highRate = op === 'PF';
  const gate = commandAllowed(cmd, sessionToken);
  if (!gate.ok) {
    if (highRate) {
      markPfMailboxDrop(gate.reason, 'gate');
      return { ok: false, reason: gate.reason };
    }
    console.warn(`[WS] blocked cmd (${gate.reason}): ${cmd}`);
    pushSessionEvent('command_blocked', { cmd, reason: gate.reason, transport: currentTransport });
    recWrite('drop', { cmd, reason: gate.reason, transport: currentTransport, session: publicSessionState() });
    broadcast(JSON.stringify({ evt: 'cmd_dropped', c: cmd, reason: gate.reason, transport: currentTransport, session: publicSessionState() }));
    return { ok: false, reason: gate.reason };
  }
  if (highRate) {
    if (sessionToken) return enqueuePfCommand(cmd, sessionToken);
    return manualPfUpdateTarget(cmd);
  }

  if (pfCommandCancelsMailbox(op)) clearPfMailbox(`cancelled by ${op}`);
  console.log(`[WS] cmd: ${cmd}`);
  const wr = transport.write(cmd);
  if (wr.ok) {
    if (!sessionToken && commandStopsManualPf(cmd, op)) manualPfStop(`cancelled by ${op}`);
    if (!sessionToken && op === 'VF') manualPfSetLimits(cmd);
    observeCommand(cmd, wr.kind, sessionToken);
    if (activeSession && sessionToken === activeSession.token) pushSessionEvent('command_sent', { cmd, via: wr.kind });
    recWrite('cmd', cmd);
    broadcast(JSON.stringify({ evt: 'cmd', c: cmd, via: wr.kind }));
    return { ok: true, via: wr.kind };
  }
  console.warn(`[WS] dropped cmd (${wr.reason}): ${cmd}`);
  if (activeSession && sessionToken === activeSession.token) pushSessionEvent('command_dropped', { cmd, reason: wr.reason, transport: currentTransport });
  recWrite('drop', { cmd, reason: wr.reason, transport: currentTransport });
  broadcast(JSON.stringify({ evt: 'cmd_dropped', c: cmd, reason: wr.reason, transport: currentTransport }));
  return { ok: false, reason: wr.reason };
}

function sendRequiredCommand(cmd, sessionToken = null) {
  const result = sendCommand(cmd, sessionToken);
  if (!result.ok) throw new Error(`command dropped: ${cmd} (${result.reason || 'unknown'})`);
  return result;
}

function recWrite(dir, payload) {
  if (!recStream) return;
  recStream.write(JSON.stringify({ t: performance.now() - recT0, dir, d: payload }) + '\n');
  recCount++;
}

function recEvent(type, payload = {}) {
  if (!recStream) return;
  recWrite('event', { type, ...payload });
}

function percentile(values, p) {
  const a = values.filter(Number.isFinite).sort((x, y) => x - y);
  if (!a.length) return null;
  const idx = (a.length - 1) * (p / 100);
  const lo = Math.floor(idx), hi = Math.ceil(idx);
  if (lo === hi) return a[lo];
  return a[lo] + (a[hi] - a[lo]) * (idx - lo);
}

function avg(values) {
  const a = values.filter(Number.isFinite);
  return a.length ? a.reduce((x, y) => x + y, 0) / a.length : null;
}

function round(v, d = 3) {
  return Number.isFinite(v) ? Number(v.toFixed(d)) : null;
}

function maxFinite(values) {
  const a = values.filter(Number.isFinite);
  return a.length ? Math.max(...a) : null;
}

function pushWindow(arr, item, now = Date.now()) {
  arr.push(item);
  while (arr.length && now - arr[0].at > FOLLOW_DIAG_WINDOW_MS) arr.shift();
  while (arr.length > FOLLOW_DIAG_MAX_SAMPLES) arr.shift();
}

function parsePoseCommand(cmd, op) {
  const parts = String(cmd || '').trim().split(/\s+/);
  if (parts[0]?.toUpperCase() !== op) return null;
  const pose = parts.slice(1, 7).map(Number);
  return pose.length === 6 && pose.every(Number.isFinite) ? pose : null;
}

function poseDelta(prev, next) {
  if (!Array.isArray(prev) || !Array.isArray(next)) return null;
  const d = next.map((x, i) => x - prev[i]);
  return {
    maxT: Math.max(Math.abs(d[0]), Math.abs(d[1]), Math.abs(d[2])),
    maxR: Math.max(Math.abs(d[3]), Math.abs(d[4]), Math.abs(d[5])),
    d,
  };
}

function finiteOrNull(v) {
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function parseFollowReference(fref) {
  if (!fref || typeof fref !== 'object') return null;
  let errT = null;
  let errR = null;
  if (Array.isArray(fref.e) && fref.e.length >= 2) {
    errT = finiteOrNull(fref.e[0]);
    errR = finiteOrNull(fref.e[1]);
  } else if (Array.isArray(fref.cur) && Array.isArray(fref.tgt)) {
    const frefErr = poseDelta(fref.cur, fref.tgt);
    errT = frefErr ? frefErr.maxT : null;
    errR = frefErr ? frefErr.maxR : null;
  }
  errT = errT ?? finiteOrNull(fref.errT);
  errR = errR ?? finiteOrNull(fref.errR);
  const out = {
    gen: fref.gen || (Number(fref.g) === 1 ? 'esp32_jerk_limited' : null),
    errT,
    errR,
    vel: Array.isArray(fref.v) ? fref.v : (Array.isArray(fref.vel) ? fref.vel : null),
    vmax: Array.isArray(fref.lim) ? fref.lim : (Array.isArray(fref.vmax) ? fref.vmax : null),
  };
  if (Array.isArray(fref.cur)) out.cur = fref.cur;
  if (Array.isArray(fref.tgt)) out.tgt = fref.tgt;
  if (Array.isArray(fref.amax)) out.amax = fref.amax;
  if (Array.isArray(fref.jmax)) out.jmax = fref.jmax;
  return out;
}

function parseVfCommand(cmd) {
  const parts = String(cmd || '').trim().split(/\s+/);
  if (parts[0]?.toUpperCase() !== 'VF') return null;
  const vmaxT = Number(parts[1]);
  const vmaxR = Number(parts[2]);
  return Number.isFinite(vmaxT) && Number.isFinite(vmaxR) && vmaxT > 0 && vmaxR > 0
    ? { vmaxT, vmaxR }
    : null;
}

function manualPfStop(reason = 'stopped') {
  const wasActive = !!manualPf.target;
  manualPfResamplerStop(reason);
  manualPf.target = null;
  manualPf.output = null;
  manualPf.lastReason = reason;
  if (wasActive) followDiagWrite('manual_pf_stop', { reason });
}

function manualPfSummary(now = Date.now()) {
  const inputDt = [];
  for (let i = 1; i < manualPf.inputWindow.length; i++) {
    inputDt.push(manualPf.inputWindow[i].at - manualPf.inputWindow[i - 1].at);
  }
  const emitDt = [];
  for (let i = 1; i < manualPf.emitWindow.length; i++) {
    emitDt.push(manualPf.emitWindow[i].at - manualPf.emitWindow[i - 1].at);
  }
  const inputSpan = manualPf.inputWindow.length >= 2
    ? (manualPf.inputWindow[manualPf.inputWindow.length - 1].at - manualPf.inputWindow[0].at) / 1000
    : 0;
  const emitSpan = manualPf.emitWindow.length >= 2
    ? (manualPf.emitWindow[manualPf.emitWindow.length - 1].at - manualPf.emitWindow[0].at) / 1000
    : 0;
  return {
    enabled: true,
    mode: MANUAL_PF_MODE,
    hz: PF_RESAMPLE_HZ,
    intervalMs: PF_RESAMPLE_MS,
    running: !!manualPfTimer,
    active: !!manualPf.target,
    targetUpdates: manualPf.targetUpdates,
    emitted: manualPf.emitted,
    inputHz: inputSpan > 0 ? round((manualPf.inputWindow.length - 1) / inputSpan, 2) : null,
    emitHz: emitSpan > 0 ? round((manualPf.emitWindow.length - 1) / emitSpan, 2) : null,
    inputDtP95: round(percentile(inputDt, 95), 2),
    emitDtP95: round(percentile(emitDt, 95), 2),
    msSinceInput: manualPf.lastInputAt ? now - manualPf.lastInputAt : null,
    msSinceEmit: manualPf.lastEmitAt ? now - manualPf.lastEmitAt : null,
    vmaxT: manualPf.vmaxT,
    vmaxR: manualPf.vmaxR,
    target: manualPf.target,
    output: manualPf.output,
    writeFail: manualPf.writeFail,
    transportSkip: manualPf.transportSkip,
    emitErrors: manualPf.emitErrors,
    lastReason: manualPf.lastReason,
  };
}

function manualPfSetLimits(cmd) {
  const vf = parseVfCommand(cmd);
  if (!vf) return;
  manualPf.vmaxT = Math.max(1, Math.min(500, vf.vmaxT));
  manualPf.vmaxR = Math.max(1, Math.min(360, vf.vmaxR));
  followDiagWrite('manual_pf_limits', { vmaxT: manualPf.vmaxT, vmaxR: manualPf.vmaxR });
  if (followSession.active && followSession.vfChanges.length < 50) {
    followSession.vfChanges.push({ at: new Date().toISOString(), vmaxT: manualPf.vmaxT, vmaxR: manualPf.vmaxR });
  }
}

// ===== Manual FOLLOW resampler (configurable rate, default 100 Hz) =====
// browser/phone only post the latest intent (manualPf.target). This fixed-rate
// timer is the single physical PF clock: each tick re-emits the latest target,
// fully decoupled from browser main-thread jitter. The ESP32 control loop (default
// 100 Hz, runtime-settable via `L <ms>`) owns the jerk/accel/vel-limited followStep
// reference; this resampler only guarantees a steady target-update cadence on the
// wire. Keep PF_RESAMPLE_HZ matched to the firmware loop rate (coupled knob).
// We drive a self-correcting setTimeout chain against an ABSOLUTE tick schedule
// (manualPfNextAt += period) rather than setInterval. setInterval silently slips
// under event-loop congestion (telemetry parse + WS broadcast share this loop)
// and gives no hook to fold a fresh input into the cadence. Absolute-clock gives:
//   - drift-free cadence: each tick aims at its ideal wall-clock slot, so a late
//     callback is corrected on the next hop instead of accumulating.
//   - spiral protection: once a tick fires late enough to skip a slot, drop the
//     missed slots (resync to now+period) so a stalled loop emits the latest target
//     ONCE on recovery instead of firing a back-to-back catch-up burst onto the wire.
//   - leading-edge fold-in: a fresh target whose slot is already due fires now via
//     manualPfUpdateTarget, cutting up-to-one-period of input->wire latency.
function manualPfArm() {
  // (re)schedule next tick toward its absolute slot; clamp delay to [0, period].
  if (!manualPf.target || activeSession) return;
  const delay = Math.min(PF_RESAMPLE_MS, Math.max(0, manualPfNextAt - Date.now()));
  manualPfTimer = setTimeout(manualPfTick, delay);
}

function manualPfResamplerStart() {
  if (manualPfTimer || activeSession) return;
  manualPf.lastEmitAt = 0;   // fresh emit run: first tick must not measure the cross-session idle gap
  manualPfNextAt = Date.now();   // first slot is now (emit asap)
  manualPfArm();
  followDiagWrite('manual_pf_resampler', { state: 'start', hz: PF_RESAMPLE_HZ });
}

function manualPfResamplerStop(reason = 'stopped') {
  if (!manualPfTimer) return;
  clearTimeout(manualPfTimer);
  manualPfTimer = null;
  followDiagWrite('manual_pf_resampler', { state: 'stop', reason });
}

// Advance the absolute schedule one slot, dropping missed slots on stall. The
// resync test is AFTER the increment: if the next slot is already in the past, the
// tick fired >=1 period late (skipped a slot), so resync to now+period rather than
// re-arming at delay 0 for each missed slot (which would emit a duplicate burst).
function manualPfAdvance() {
  manualPfNextAt += PF_RESAMPLE_MS;
  const now = Date.now();
  if (manualPfNextAt < now) {
    manualPfNextAt = now + PF_RESAMPLE_MS;   // fell behind: drop missed slots, emit latest once, resync
  }
}

// One scheduled tick: emit, advance the absolute clock, re-arm. A throwing emit
// must never escape to uncaughtException (killing the process) nor leave the chain
// un-armed (silently freezing the stream) -- hence try/finally.
function manualPfTick() {
  manualPfTimer = null;
  try {
    manualPfEmit();
  } catch (err) {
    manualPf.emitErrors++;   // persistent counter: a one-shot throw that recovers must stay observable
    manualPf.lastReason = `emit error: ${err && err.message ? err.message : err}`;
  } finally {
    manualPfAdvance();
    manualPfArm();
  }
}

// One emission. Inline transport write (NOT enqueuePfCommand): the mailbox is a
// latest-wins coalescer for bursty browser input and its own ~10ms flush clock
// would beat against this scheduler, corrupting both the steady cadence and the
// emit accounting (mailbox flush is shared with session PF). Mirrors the
// gate/transport checks of flushPfMailbox(). Called from the scheduled tick and
// from the leading-edge fold-in; never schedules -- the caller owns cadence.
function manualPfEmit() {
  const target = manualPf.target;
  if (!Array.isArray(target) || target.length !== 6) return;
  const cmd = `PF ${target.map((x) => Number(x).toFixed(3)).join(' ')}`;
  const gate = commandAllowed(cmd, null);
  if (!gate.ok) { manualPf.lastReason = gate.reason; return; }
  if (currentTransport.state !== 'connected') {
    manualPf.transportSkip++;
    manualPf.lastReason = `transport ${currentTransport.state || 'unknown'}`;
    return;
  }
  const wr = transport.write(cmd);
  if (!wr.ok) {
    manualPf.writeFail++;
    manualPf.lastReason = wr.reason;
    recWrite('drop', { cmd, reason: wr.reason, transport: currentTransport, highRate: true });
    return;
  }
  const now = Date.now();
  if (manualPf.lastEmitAt) {
    const dt = now - manualPf.lastEmitAt;
    if (dt > manualPf.emitDtMaxSession) manualPf.emitDtMaxSession = dt;
    if (followSession.active && followSession.acc) {
      reservoirAdd(followSession.acc.emitDt, dt, ++followSession.acc.emitDtSeen, FOLLOW_RESERVOIR_MAX);
    }
  }
  manualPf.output = target.slice();
  manualPf.emitted++;
  manualPf.lastEmitAt = now;
  manualPf.lastReason = null;
  pushWindow(manualPf.emitWindow, { at: now, pose: target.slice() }, now);
  observeCommand(cmd, 'manual-resample', null);   // feeds followDiag.pf + lastPfAt (nearFollow gate)
  recWrite('cmd', cmd);
}

function manualPfUpdateTarget(cmd) {
  const pose = parsePoseCommand(cmd, 'PF');
  if (!pose) {
    markPfMailboxDrop('invalid manual PF target', 'invalid');
    return { ok: false, reason: 'invalid PF pose' };
  }
  const now = Date.now();
  manualPf.target = pose;
  manualPf.lastInputAt = now;
  manualPf.targetUpdates++;
  manualPf.lastReason = null;
  pushWindow(manualPf.inputWindow, { at: now, pose }, now);
  followDiagWrite('manual_pf_target', {
    pose,
    targetUpdates: manualPf.targetUpdates,
    output: manualPf.output,
  });
  manualPfResamplerStart();
  // Leading edge: if the steady slot is already due (input landed after its slot),
  // fold this target in NOW instead of waiting up to one period. manualPfTick
  // advances on the absolute clock, so the steady phase stays intact and no double
  // emit occurs (the overdue slot is consumed, not added to).
  if (manualPfTimer && Date.now() >= manualPfNextAt) {
    clearTimeout(manualPfTimer);
    manualPfTimer = null;
    manualPfTick();
  }
  return { ok: true, via: 'manual-pf-resample' };
}

function followDiagWrite(type, data = {}) {
  if (!FOLLOW_DIAG_ENABLED) return;
  try {
    if (!followDiagStream) {
      fs.mkdirSync(FOLLOW_DIAG_DIR, { recursive: true });
      const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
      followDiagPath = path.join(FOLLOW_DIAG_DIR, `follow_${stamp}.jsonl`);
      followDiagStream = fs.createWriteStream(followDiagPath, { flags: 'a' });
      followDiagT0 = performance.now();
      followDiagStream.write(JSON.stringify({
        meta: {
          kind: 'follow-diagnostics',
          wallClock: new Date().toISOString(),
          windowMs: FOLLOW_DIAG_WINDOW_MS,
          maxSamples: FOLLOW_DIAG_MAX_SAMPLES,
          pfMailboxMinMs: PF_MAILBOX_MIN_MS,
          manualPfMode: MANUAL_PF_MODE,
        },
      }) + '\n');
      console.log(`[FollowDiag] writing ${followDiagPath}`);
    }
    followDiagStream.write(JSON.stringify({
      seq: ++followDiagSeq,
      t: round(performance.now() - followDiagT0, 3),
      wall: new Date().toISOString(),
      type,
      ...data,
    }) + '\n');
  } catch (err) {
    if (!followDiag.warnings.some((w) => w.type === 'diag_write_failed')) {
      followDiag.warnings.push({ type: 'diag_write_failed', error: err.message, at: Date.now() });
      console.warn(`[FollowDiag] write failed: ${err.message}`);
    }
  }
}

function followDiagSummary(now = Date.now()) {
  const pf = followDiag.pf;
  const tele = followDiag.tele;
  const pfDt = pf.map((x) => x.dtMs).filter(Number.isFinite);
  const teleDt = tele.map((x) => x.dtMs).filter(Number.isFinite);
  const recentTele = tele[tele.length - 1] || null;
  const durationPf = pf.length >= 2 ? (pf[pf.length - 1].at - pf[0].at) / 1000 : 0;
  const durationTele = tele.length >= 2 ? (tele[tele.length - 1].at - tele[0].at) / 1000 : 0;
  return {
    enabled: FOLLOW_DIAG_ENABLED,
    path: followDiagPath,
    windowMs: FOLLOW_DIAG_WINDOW_MS,
    ageMs: recentTele ? now - recentTele.at : null,
    state: {
      fl: followDiag.lastFl,
      profile: followDiag.lastProfile,
      lastOp: followDiag.lastOp,
      msSincePf: followDiag.lastPfAt ? now - followDiag.lastPfAt : null,
    },
    pf: {
      count: pf.length,
      hz: durationPf > 0 ? round((pf.length - 1) / durationPf, 2) : null,
      dtAvg: round(avg(pfDt), 2),
      dtP95: round(percentile(pfDt, 95), 2),
      dtMax: round(maxFinite(pfDt), 2),
      stepTMax: round(maxFinite(pf.map((x) => x.stepT)), 3),
      stepRMax: round(maxFinite(pf.map((x) => x.stepR)), 3),
    },
    tele: {
      count: tele.length,
      hz: durationTele > 0 ? round((tele.length - 1) / durationTele, 2) : null,
      dtAvg: round(avg(teleDt), 2),
      dtP95: round(percentile(teleDt, 95), 2),
      dtMax: round(maxFinite(teleDt), 2),
    },
    manual: manualPfSummary(now),
    mailbox: pfMailboxSummary(now),
    parse: {
      errors: lineParseStats.errors,
      lastAt: lineParseStats.lastAt || null,
      lastAgeMs: lineParseStats.lastAt ? now - lineParseStats.lastAt : null,
      lastLen: lineParseStats.lastLen || null,
      lastError: lineParseStats.lastError,
      lastHead: lineParseStats.lastHead,
      lastTail: lineParseStats.lastTail,
    },
    latest: recentTele ? recentTele.sample : null,
    warnings: followDiag.warnings.slice(-12),
  };
}

// ===== Follow session report (Part B) =====
// Streaming accumulator over one FOLLOW session. Independent of the rolling
// followDiag.tele window (caps at 120s/5000 samples and evicts early samples on
// long sessions). Hard metrics are O(1) exact; advisory percentiles use bounded
// reservoirs so memory is independent of session length.
function newFollowAcc() {
  return {
    errT: [], errR: [], errSeenT: 0, errSeenR: 0,
    emitDt: [], emitDtSeen: 0,
    satCount: 0, frefSamples: 0, satAxis: [0, 0, 0, 0, 0, 0],
    counters: {},
    ctrlMaxUs: null, jitterUsMax: null, herrMax: null, lhzMin: null, lhzMax: null,
    teleSamples: 0, counterResetDetected: false,
    emittedAtStart: 0, writeFailAtStart: 0, transportSkipAtStart: 0, emitErrorsAtStart: 0,
    mailbox: { received: 0, flushed: 0, coalesced: 0, dropped: 0, writeFail: 0, transportDrop: 0 },
  };
}

function runMax(cur, v) {
  return Number.isFinite(v) ? (cur == null ? v : Math.max(cur, v)) : cur;
}

function axisVmax(vmax, i) {
  if (!Array.isArray(vmax)) return null;
  if (vmax.length >= 6) return Number(vmax[i]);
  if (vmax.length >= 2) return Number(i < 3 ? vmax[0] : vmax[1]);
  return null;
}

// Monotonic firmware counter -> session delta, preserving accumulated delta across
// a firmware reboot (counter resets toward 0) and flagging it.
function accCounter(acc, key, val) {
  if (!Number.isFinite(val)) return;
  const c = acc.counters[key];
  if (!c) { acc.counters[key] = { base: val, last: val }; return; }
  if (val < c.last) { c.base = val - (c.last - c.base); acc.counterResetDetected = true; }
  c.last = val;
}

function counterDelta(acc, key) {
  const c = acc.counters[key];
  return c ? c.last - c.base : null;
}

function reservoirAdd(arr, val, seen, cap) {
  if (arr.length < cap) { arr.push(val); return; }
  const j = Math.floor(Math.random() * seen);
  if (j < cap) arr[j] = val;
}

function followSessionStart(now, sample, trigger) {
  if (followSession.active) {
    // stale session never finalized (transport loss) -> close before starting anew
    if (now - followSession.startedAtMs > FOLLOW_SESSION_MIN_MS
        && followDiag.lastTeleAt && now - followDiag.lastTeleAt > FOLLOW_STALE_MS) {
      followSessionEnd(now, 'stale-restart');
    } else {
      return;
    }
  }
  const acc = newFollowAcc();
  acc.emittedAtStart = manualPf.emitted;
  acc.writeFailAtStart = manualPf.writeFail;
  acc.transportSkipAtStart = manualPf.transportSkip;
  acc.emitErrorsAtStart = manualPf.emitErrors;
  acc.mailbox = {
    received: pfMailbox.received, flushed: pfMailbox.flushed, coalesced: pfMailbox.coalesced,
    dropped: pfMailbox.dropped, writeFail: pfMailbox.writeFail, transportDrop: pfMailbox.transportDrop,
  };
  manualPf.emitDtMaxSession = 0;
  const vmax = sample && sample.fref && Array.isArray(sample.fref.vmax) ? sample.fref.vmax : null;
  const limits = (vmax && vmax.length >= 2)
    ? { vmaxT: Number(vmax[0]), vmaxR: Number(vmax.length >= 6 ? vmax[3] : vmax[1]), source: 'fref' }
    : { vmaxT: manualPf.vmaxT, vmaxR: manualPf.vmaxR, source: 'manualPf' };
  followSession.active = true;
  followSession.seq = ++followReportSeq;
  followSession.startedAt = new Date(now).toISOString();
  followSession.startedAtMs = now;
  followSession.endedAt = null;
  followSession.trigger = trigger;
  followSession.limits = limits;
  followSession.vfChanges = [];
  followSession.acc = acc;
  followDiagWrite('follow_session', { state: 'start', seq: followSession.seq, trigger, limits });
}

function followSessionAccumulate(now, sample) {
  const acc = followSession.acc;
  if (!acc || !sample) return;
  acc.teleSamples++;
  accCounter(acc, 'ikFail', sample.ikFail);
  accCounter(acc, 'fkFail', sample.fkFail);
  accCounter(acc, 'coordLimit', sample.coordLimit);
  accCounter(acc, 'missed', sample.missed);
  accCounter(acc, 'cmdDrop', sample.cmdDrop);
  accCounter(acc, 'teleDrop', sample.teleDrop);
  accCounter(acc, 'rxDrop', sample.rxDrop);
  accCounter(acc, 'txFail', sample.txFail);
  accCounter(acc, 'busOvr', sample.busOvr);
  acc.ctrlMaxUs = runMax(acc.ctrlMaxUs, sample.ctrlMaxUs);
  acc.jitterUsMax = runMax(acc.jitterUsMax, sample.jitterUs);
  acc.herrMax = runMax(acc.herrMax, sample.herrMax);
  if (Number.isFinite(sample.lhz)) {
    acc.lhzMin = acc.lhzMin == null ? sample.lhz : Math.min(acc.lhzMin, sample.lhz);
    acc.lhzMax = acc.lhzMax == null ? sample.lhz : Math.max(acc.lhzMax, sample.lhz);
  }
  const fr = sample.fref;
  if (fr) {
    if (Number.isFinite(fr.errT)) { acc.errSeenT++; reservoirAdd(acc.errT, fr.errT, acc.errSeenT, FOLLOW_RESERVOIR_MAX); }
    if (Number.isFinite(fr.errR)) { acc.errSeenR++; reservoirAdd(acc.errR, fr.errR, acc.errSeenR, FOLLOW_RESERVOIR_MAX); }
    if (Array.isArray(fr.vel) && Array.isArray(fr.vmax)) {
      acc.frefSamples++;
      let sat = false;
      for (let i = 0; i < 6; i++) {
        const v = Number(fr.vel[i]);
        const lim = axisVmax(fr.vmax, i);
        if (Number.isFinite(v) && Number.isFinite(lim) && lim > 0 && Math.abs(v) >= FOLLOW_SAT_FRAC * lim) {
          acc.satAxis[i]++; sat = true;
        }
      }
      if (sat) acc.satCount++;
    }
  }
}

function followSessionEnd(now, trigger) {
  if (!followSession.active) return;
  followSession.active = false;
  followSession.endedAt = new Date(now).toISOString();
  const durationMs = now - followSession.startedAtMs;
  if (durationMs < FOLLOW_SESSION_MIN_MS) {
    followDiagWrite('follow_session', { state: 'abort', seq: followSession.seq, durationMs, trigger });
    return;
  }
  const report = buildFollowReport(now, 'complete', trigger, durationMs);
  lastFollowReport = report;
  followDiagWrite('follow_report', report);
}

function pushCounterCheck(checks, name, hasTele, reset, clean, detail) {
  if (!hasTele) { checks.push({ check: name, pass: null, reason: `n/a: no telemetry - ${detail}` }); return; }
  if (!clean) { checks.push({ check: name, pass: false, reason: `FAIL: ${detail}` }); return; }
  if (reset) { checks.push({ check: name, pass: null, reason: `WARN(counter reset): ${detail}` }); return; }
  checks.push({ check: name, pass: true, reason: `PASS: ${detail}` });
}

function computeFollowVerdict(emit, wire, fw, limits) {
  const V = FOLLOW_VERDICT;
  const checks = [];
  const advisory = [];
  const hasTele = fw.teleSamples > 0;

  if (!emit.applicable) {
    checks.push({ check: 'emission', pass: null, reason: 'n/a: session-PF follow (manual resampler did not emit)' });
  } else {
    const ok = emit.emitHz != null && emit.emitHz >= V.emitHzLo && emit.emitHz <= V.emitHzHi
      && emit.emitDtP95 != null && emit.emitDtP95 <= V.emitDtP95Ms
      && (emit.emitDtMaxSession == null || emit.emitDtMaxSession <= V.emitDtMaxMs);
    checks.push({ check: 'emission', pass: ok,
      reason: `${ok ? 'PASS' : 'FAIL'}: hz=${emit.emitHz} (want ${V.emitHzLo}-${V.emitHzHi}), dtP95=${emit.emitDtP95}ms (<=${V.emitDtP95Ms}), dtMax=${emit.emitDtMaxSession}ms (<=${V.emitDtMaxMs})` });
  }

  if (!emit.applicable) {
    checks.push({ check: 'wire', pass: null, reason: 'n/a: no resampler emits this session' });
  } else {
    const ok = (wire.resampler.writeFail || 0) === 0 && (wire.resampler.transportSkip || 0) === 0
      && (wire.resampler.emitErrors || 0) === 0;
    checks.push({ check: 'wire', pass: ok,
      reason: `${ok ? 'PASS' : 'FAIL'}: writeFail=${wire.resampler.writeFail}, transportSkip=${wire.resampler.transportSkip}, emitErrors=${wire.resampler.emitErrors}` });
  }

  pushCounterCheck(checks, 'kinematics', hasTele, fw.counterResetDetected,
    (fw.counters.ikFailDelta || 0) === 0 && (fw.counters.fkFailDelta || 0) === 0 && (fw.counters.coordLimitDelta || 0) === 0,
    `ikFail +${fw.counters.ikFailDelta ?? 0}, fkFail +${fw.counters.fkFailDelta ?? 0}, coordLimit +${fw.counters.coordLimitDelta ?? 0}`);

  {
    const lhzNom = fw.lhzNominal;
    const budget = V.ctrlBudgetUs > 0 ? V.ctrlBudgetUs : (Number.isFinite(lhzNom) && lhzNom > 0 ? Math.round(0.8 * 1e6 / lhzNom) : 8000);
    const md = fw.counters.missedDelta;
    const clean = (md == null || md <= V.missedMax) && (fw.ctrlMaxUs == null || fw.ctrlMaxUs < budget);
    pushCounterCheck(checks, 'control', hasTele, fw.counterResetDetected, clean,
      `missed +${md ?? 0} (<=${V.missedMax}), ctrlMaxUs=${fw.ctrlMaxUs} (budget ${budget}us)`);
  }

  pushCounterCheck(checks, 'bus', hasTele, fw.counterResetDetected,
    (fw.counters.rxDropDelta || 0) === 0 && (fw.counters.txFailDelta || 0) === 0,
    `rxDrop +${fw.counters.rxDropDelta ?? 0}, txFail +${fw.counters.txFailDelta ?? 0}`);

  if (!hasTele || fw.lhzNominal == null) {
    checks.push({ check: 'looprate', pass: null, reason: 'n/a: no loop-rate telemetry' });
  } else {
    const floor = V.lhzFloorFrac * fw.lhzNominal;
    const ok = fw.lhzMin != null && fw.lhzMin >= floor;
    checks.push({ check: 'looprate', pass: ok,
      reason: `${ok ? 'PASS' : 'FAIL'}: lhzMin=${fw.lhzMin} (floor ${round(floor, 0)}, nominal ${fw.lhzNominal})` });
  }

  if (Number.isFinite(fw.saturation.pct) && fw.saturation.pct > V.satPct) {
    advisory.push(`tracking velocity-limited ${fw.saturation.pct}% of session (>${V.satPct}%); consider raising VF (current vmaxT=${limits?.vmaxT ?? '?'} vmaxR=${limits?.vmaxR ?? '?'})`);
  }
  if (fw.counterResetDetected) {
    advisory.push('firmware counter reset detected mid-session (reboot?); affected checks downgraded to warn');
  }
  // Coupled-knob guard: emit rate and firmware control rate should match. If they
  // diverge (operator raised PF_RESAMPLE_HZ without the firmware `L`, or vice versa)
  // the server over/under-feeds the loop. Benign (followStep coalesces to latest) so
  // it stays advisory, not a FAIL -- but surface it rather than leaving the operator
  // to eyeball emitHz vs lhzNominal.
  if (emit.applicable && emit.emitHz != null && Number.isFinite(fw.lhzNominal) && fw.lhzNominal > 0) {
    const ratio = emit.emitHz / fw.lhzNominal;
    if (ratio > 1.2 || ratio < 0.8) {
      advisory.push(`emit rate ${emit.emitHz}Hz vs firmware control rate ${fw.lhzNominal}Hz mismatch (ratio ${round(ratio, 2)}); match PF_RESAMPLE_HZ to the firmware loop (\`L <ms>\`) so the coupled rate knob stays consistent`);
    }
  }

  // tri-state: any FAIL -> false; else any real PASS -> true; else null (inconclusive,
  // e.g. status-triggered session with no telemetry and no resampler emits).
  const anyFalse = checks.some((c) => c.pass === false);
  const anyTrue = checks.some((c) => c.pass === true);
  const pass = anyFalse ? false : (anyTrue ? true : null);
  return { pass, checks, advisory };
}

function buildFollowReport(now = Date.now(), status = null, trigger = null, durationMs = null) {
  if (!status) {
    if (followSession.active) { status = 'in-progress'; trigger = followSession.trigger; durationMs = now - followSession.startedAtMs; }
    else if (lastFollowReport) return lastFollowReport;
    else return { status: 'none', verdict: null, message: 'no follow session since startup' };
  }
  const acc = followSession.acc;
  if (!acc) return lastFollowReport || { status: 'none', verdict: null, message: 'no follow session data' };
  const durS = durationMs > 0 ? durationMs / 1000 : 0;
  const emitted = manualPf.emitted - acc.emittedAtStart;
  const emit = {
    applicable: emitted > 0,
    mode: MANUAL_PF_MODE,
    emitted,
    emitHz: durS > 0 ? round(emitted / durS, 2) : null,
    emitDtP95: round(percentile(acc.emitDt, 95), 2),   // session-scoped (independent of rolling window)
    emitDtMaxSession: round(manualPf.emitDtMaxSession, 2),
  };
  const wire = {
    resampler: {
      writeFail: manualPf.writeFail - acc.writeFailAtStart,
      transportSkip: manualPf.transportSkip - acc.transportSkipAtStart,
      emitErrors: manualPf.emitErrors - acc.emitErrorsAtStart,
    },
    mailbox: {
      received: pfMailbox.received - acc.mailbox.received,
      flushed: pfMailbox.flushed - acc.mailbox.flushed,
      coalesced: pfMailbox.coalesced - acc.mailbox.coalesced,
      dropped: pfMailbox.dropped - acc.mailbox.dropped,
      writeFail: pfMailbox.writeFail - acc.mailbox.writeFail,
      transportDrop: pfMailbox.transportDrop - acc.mailbox.transportDrop,
    },
  };
  const firmware = {
    errT: { p95: round(percentile(acc.errT, 95), 2), max: round(maxFinite(acc.errT), 2), n: acc.errSeenT },
    errR: { p95: round(percentile(acc.errR, 95), 3), max: round(maxFinite(acc.errR), 3), n: acc.errSeenR },
    saturation: {
      pct: acc.frefSamples ? round(100 * acc.satCount / acc.frefSamples, 1) : null,
      satCount: acc.satCount, n: acc.frefSamples,
      perAxis: acc.frefSamples ? acc.satAxis.map((c) => round(100 * c / acc.frefSamples, 1)) : null,
    },
    counters: {
      ikFailDelta: counterDelta(acc, 'ikFail'), fkFailDelta: counterDelta(acc, 'fkFail'),
      coordLimitDelta: counterDelta(acc, 'coordLimit'), missedDelta: counterDelta(acc, 'missed'),
      cmdDropDelta: counterDelta(acc, 'cmdDrop'), teleDropDelta: counterDelta(acc, 'teleDrop'),
      rxDropDelta: counterDelta(acc, 'rxDrop'), txFailDelta: counterDelta(acc, 'txFail'),
      busOvrDelta: counterDelta(acc, 'busOvr'),
    },
    ctrlMaxUs: acc.ctrlMaxUs, jitterUsMax: acc.jitterUsMax,
    lhzMin: acc.lhzMin, lhzNominal: acc.lhzMax, herrMax: round(acc.herrMax, 3),
    teleHz: durS > 0 ? round(acc.teleSamples / durS, 2) : null, teleSamples: acc.teleSamples,
    counterResetDetected: acc.counterResetDetected,
  };
  const v = computeFollowVerdict(emit, wire, firmware, followSession.limits);
  return {
    status,
    generatedAt: new Date(now).toISOString(),
    session: {
      seq: followSession.seq,
      startedAt: followSession.startedAt,
      endedAt: status === 'complete' ? followSession.endedAt : null,
      durationMs: Math.round(durationMs || 0),
      trigger,
      limits: followSession.limits,
      vfChanges: followSession.vfChanges.slice(-20),
    },
    emit, wire, firmware,
    verdict: { pass: v.pass, checks: v.checks },
    advisory: v.advisory,
  };
}

function observeCommand(cmd, via = null, sessionToken = null) {
  const op = String(cmd || '').trim().split(/\s+/)[0]?.toUpperCase();
  if (!op) return;
  const now = Date.now();
  if (['FOLLOW', 'PF', 'VF', 'FE', 'P', 'H', 'D', 'S', 'E'].includes(op)) {
    followDiag.lastOp = op;
    const entry = { op, cmd, via, session: !!sessionToken };
    if (op === 'PF') {
      const pose = parsePoseCommand(cmd, 'PF');
      const dtMs = followDiag.lastPfAt ? now - followDiag.lastPfAt : null;
      const delta = poseDelta(followDiag.lastPfPose, pose);
      followDiag.lastPfAt = now;
      followDiag.lastPfPose = pose || followDiag.lastPfPose;
      pushWindow(followDiag.pf, {
        at: now,
        dtMs,
        stepT: delta ? delta.maxT : null,
        stepR: delta ? delta.maxR : null,
      }, now);
      entry.pose = pose;
      entry.dtMs = dtMs;
      entry.step = delta;
    }
    if (op !== 'PF') followDiagWrite('cmd', entry);
  }
}

function observeIncoming(d) {
  const now = Date.now();
  const isTelemetry = d && Array.isArray(d.a);
  const isFollowRelatedStatus = d && (d.status === 'follow on' || d.status === 'follow off' || d.status === 'follow vmax');
  if (isFollowRelatedStatus) {
    followDiagWrite('status', {
      status: d.status,
      pose: d.pose || null,
      fk: d.fk,
      vmaxT: d.t,
      vmaxR: d.r,
      raw: d,
    });
    if (d.status === 'follow on') followSessionStart(now, null, 'status');
    else if (d.status === 'follow off') followSessionEnd(now, 'status');
  }
  if (!isTelemetry) return;

  const fl = Number.isFinite(Number(d.fl)) ? Number(d.fl) : null;
  const profile = d.ctl?.profile || null;
  const prevFl = followDiag.lastFl;
  if (fl !== followDiag.lastFl || profile !== followDiag.lastProfile) {
    followDiag.lastFl = fl;
    followDiag.lastProfile = profile;
    const state = { at: now, fl, profile, pos: d.pos, mode: d.ctl?.mode || null };
    pushWindow(followDiag.state, state, now);
    followDiagWrite('state', state);
  }
  // FOLLOW exit must finalize even when the nearFollow gate below has lapsed.
  if (fl !== 1 && prevFl === 1) followSessionEnd(now, 'fl');

  const nearFollow = fl === 1 || profile === 'follow' || (followDiag.lastPfAt && now - followDiag.lastPfAt < 5000);
  if (!nearFollow) return;

  const dtMs = followDiag.lastTeleAt ? now - followDiag.lastTeleAt : null;
  followDiag.lastTeleAt = now;
  const sample = {
    fl,
    profile,
    pos: d.pos,
    ok: d.ok,
    lhz: d.lhz,
    fwDt: d.dt,
    hmax: d.hmax,
    herrMax: Array.isArray(d.herr) ? maxFinite(d.herr.map((x) => Math.abs(Number(x)))) : null,
    coordMax: d.ctl?.coordMax,
    ikFail: d.ctl?.ikFail,
    fkFail: d.ctl?.fkFail,
    coordLimit: d.ctl?.coordLimit,
    ef: d.can?.ef ?? d.ef,
    tx: d.can?.tx ?? d.tx,
    rxDrop: d.can?.rxDrop,
    txFail: d.can?.txFail,
    busOvr: d.bus?.ovr,
    f5us: d.bus?.f5us,
    maxDrain: d.bus?.mxd,
    ctrlUs: d.sched?.ctrl_us,
    ctrlMaxUs: d.sched?.ctrl_max_us,
    jitterUs: d.sched?.jitter_us,
    missed: d.sched?.missed,
    cmdDrop: d.sched?.cmd_drop,
    teleDrop: d.sched?.tele_drop,
    ar: d.ar,
    arAgeMax: Array.isArray(d.arAge) ? maxFinite(d.arAge.map(Number)) : null,
  };
  sample.fref = parseFollowReference(d.fref);
  if (!sample.fref) delete sample.fref;
  pushWindow(followDiag.tele, { at: now, dtMs, sample }, now);

  if (fl === 1 && prevFl !== 1) followSessionStart(now, sample, 'fl');
  if (followSession.active) followSessionAccumulate(now, sample);

  if (now - followDiagLastSummaryAt > 2000) {
    followDiagLastSummaryAt = now;
    const s = followDiagSummary(now);
    followDiagWrite('summary', s);
    console.log(`[FollowDiag] fl=${s.state.fl} profile=${s.state.profile} pf=${s.pf.hz ?? '-'}Hz pfDt95=${s.pf.dtP95 ?? '-'}ms manualIn=${s.manual.inputHz ?? '-'}Hz manualOut=${s.manual.emitHz ?? '-'}Hz mb=${s.mailbox.flushed}/${s.mailbox.received} coal=${s.mailbox.coalesced} drop=${s.mailbox.dropped} tele=${s.tele.hz ?? '-'}Hz teleDt95=${s.tele.dtP95 ?? '-'}ms frefErr=${round(sample.fref?.errT, 2) ?? '-'}/${round(sample.fref?.errR, 2) ?? '-'} herr=${round(sample.herrMax, 2) ?? '-'} ok=${sample.ok ?? '-'} ef=${sample.ef ?? '-'} rxDrop=${sample.rxDrop ?? '-'} jitter=${sample.jitterUs ?? '-'}us`);
  }
}

function observeLineParseError(line, err) {
  const now = Date.now();
  lineParseStats.errors++;
  lineParseStats.lastAt = now;
  lineParseStats.lastLen = line.length;
  lineParseStats.lastError = err?.message || String(err || 'parse failed');
  lineParseStats.lastHead = line.slice(0, 120);
  lineParseStats.lastTail = line.slice(-120);
  const nearFollow = followDiag.lastFl === 1 || (followDiag.lastPfAt && now - followDiag.lastPfAt < 5000);
  if (!nearFollow && !line.startsWith('{"a":')) return;
  if (now - lastParseWarningAt < 1000) return;
  lastParseWarningAt = now;
  const warning = {
    type: 'json_parse_failed',
    count: lineParseStats.errors,
    len: lineParseStats.lastLen,
    error: lineParseStats.lastError,
    head: lineParseStats.lastHead,
    tail: lineParseStats.lastTail,
    at: now,
  };
  followDiag.warnings.push(warning);
  while (followDiag.warnings.length > 30) followDiag.warnings.shift();
  followDiagWrite('parse_error', warning);
  console.warn(`[FollowDiag] incoming JSON parse failed len=${warning.len}: ${warning.error}`);
}

function recStart(name, owner = null) {
  if (recStream) throw new Error('recorder already running');
  fs.mkdirSync(REC_DIR, { recursive: true });
  const safe = (name || 'capture').replace(/[^a-zA-Z0-9_-]/g, '_');
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  recPath = path.join(REC_DIR, `${safe}_${stamp}.jsonl`);
  recStream = fs.createWriteStream(recPath);
  recT0 = performance.now();
  recCount = 0;
  recOwner = owner ? { ...owner, startedAt: new Date().toISOString() } : null;
  // 第一行寫 meta（牆鐘時間 + 標籤），方便事後辨識
  recStream.write(JSON.stringify({ meta: { name: safe, wallClock: new Date().toISOString(), baud: BAUD, owner: publicRecOwner() } }) + '\n');
  console.log(`[Rec] started: ${recPath}`);
  return recPath;
}

function recStop() {
  if (!recStream) return null;
  const p = recPath, n = recCount, owner = publicRecOwner(), stream = recStream;
  recStream = null;
  recOwner = null;
  return new Promise((resolve, reject) => {
    stream.once('error', reject);
    stream.end(() => {
      console.log(`[Rec] stopped: ${p} (${n} lines)`);
      resolve({ path: p, lines: n, owner });
    });
  });
}

function runSysidNode(script, args) {
  return execFileSync(process.execPath, [path.join(__dirname, 'sysid', script), ...args], {
    cwd: __dirname,
    encoding: 'utf8',
    stdio: ['ignore', 'pipe', 'pipe'],
  }).trim();
}

function writeJson(file, value) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, JSON.stringify(value, null, 2));
}

function createRecordingArtifacts(recordingPath, { session = null, program = null, condition = null } = {}) {
  if (!recordingPath || !fs.existsSync(recordingPath)) return null;
  const stem = recordingPath.replace(/\.jsonl$/, '');
  const manifestPath = `${stem}.manifest.json`;
  const summaryPath = `${stem}.summary.json`;
  const evaluationPath = `${stem}.evaluation.json`;
  const bundlePath = `${stem}.bundle.json`;
  const existingManifest = safeReadJson(manifestPath);
  const manifest = {
    name: path.basename(stem),
    wallClock: new Date().toISOString(),
    kind: 'workspace-session',
    program: program || existingManifest?.program || null,
    session: session || existingManifest?.session || null,
    condition: sanitizeConditionMeta(condition) || existingManifest?.condition || null,
    platformConfig: (() => {
      try { return PlatformSoT.mergeConfig(loadPlatformConfig()); } catch { return null; }
    })(),
  };
  writeJson(manifestPath, manifest);

  const summary = JSON.parse(runSysidNode('telemetry_summary.js', [recordingPath]));
  writeJson(summaryPath, summary);
  const plot = JSON.parse(runSysidNode('plot_motion6.js', [recordingPath, '--out-dir', path.join(REC_DIR, 'plots')]));
  const evaluation = JSON.parse(runSysidNode('workspace_evaluation.js', [recordingPath]));
  writeJson(evaluationPath, evaluation);
  const bundle = { recording: recordingPath, manifest: manifestPath, summary: summaryPath, evaluation: evaluationPath, plot };
  writeJson(bundlePath, bundle);
  return { manifest: manifestPath, summary: summaryPath, evaluation: evaluationPath, bundle: bundlePath, plot };
}

function analyzeRun(id, opts = {}) {
  const file = findRunJsonl(id);
  if (!file) return null;
  return createRecordingArtifacts(file, opts);
}

function finitePose(p) {
  return PlatformSoT.finitePose(p);
}

function relativeToAbsolutePose(rel) {
  return PlatformSoT.relToAbs(rel, NEUTRAL_Z);
}

function absoluteToRelativePose(pose) {
  return PlatformSoT.absToRel(pose, NEUTRAL_Z);
}

function loadPlatformConfig() {
  let disk = {};
  try { disk = JSON.parse(fs.readFileSync(PLATFORM_CONFIG_PATH, 'utf8')); } catch {}
  let legacy = {};
  try {
    const d = JSON.parse(fs.readFileSync(HOME_POSE_PATH, 'utf8'));
    if (finitePose(d.relative)) legacy.homeRelative = d.relative;
    else if (finitePose(d.pose)) legacy.homeRelative = absoluteToRelativePose(d.pose);
  } catch {}
  return PlatformSoT.mergeConfig({ ...legacy, ...disk });
}

function savePlatformConfig(config) {
  const body = { ...PlatformSoT.mergeConfig(config), updatedAt: new Date().toISOString() };
  fs.mkdirSync(CONFIG_DIR, { recursive: true });
  fs.writeFileSync(PLATFORM_CONFIG_PATH, JSON.stringify(body, null, 2));
  return body;
}

function loadHomePose() {
  return relativeToAbsolutePose(loadPlatformConfig().homeRelative);
}

function saveHomePose(pose) {
  const cfg = loadPlatformConfig();
  cfg.homeRelative = absoluteToRelativePose(pose);
  const saved = savePlatformConfig(cfg);
  const body = { pose, relative: saved.homeRelative, updatedAt: saved.updatedAt };
  fs.writeFileSync(HOME_POSE_PATH, JSON.stringify(body, null, 2));
  return body;
}

function readRequestBody(req, limit = 65536) {
  return new Promise((resolve, reject) => {
    let body = '';
    req.on('data', (chunk) => {
      body += chunk;
      if (body.length > limit) {
        reject(new Error('request body too large'));
        req.destroy();
      }
    });
    req.on('end', () => resolve(body));
    req.on('error', reject);
  });
}

// ===== HTTP 靜態檔案伺服器 =====
const MIME = {
  '.html':'text/html',
  '.js':'application/javascript',
  '.css':'text/css',
  '.json':'application/json',
  '.jsonl':'application/x-ndjson',
  '.svg':'image/svg+xml',
};

function safeRelPath(file) {
  return path.relative(__dirname, file).replaceAll(path.sep, '/');
}

function safeReadJson(file) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); } catch { return null; }
}

function dataFileFromRel(rel) {
  if (typeof rel !== 'string' || !rel) return null;
  const p = path.resolve(__dirname, rel);
  const roots = [REC_DIR, path.join(REC_DIR, 'plots')].map((x) => path.resolve(x));
  return roots.some((root) => p === root || p.startsWith(root + path.sep)) ? p : null;
}

function latestTelemetryBody() {
  if (!lastTelemetry) return '{}';
  try {
    const d = JSON.parse(lastTelemetry);
    d._server = {
      receivedAt: lastTelemetryAt || null,
      telemetryAgeMs: lastTelemetryAt ? Date.now() - lastTelemetryAt : null,
    };
    return JSON.stringify(d);
  } catch {
    return lastTelemetry;
  }
}

function latestTelemetryObject() {
  if (!lastTelemetry) return {};
  try {
    const d = JSON.parse(lastTelemetry);
    d._server = {
      receivedAt: lastTelemetryAt || null,
      telemetryAgeMs: lastTelemetryAt ? Date.now() - lastTelemetryAt : null,
    };
    return d;
  } catch {
    return {};
  }
}

function publicWorkspaceRunState() {
  return workspaceRun
    ? {
        active: !!workspaceRun.active,
        label: workspaceRun.label || null,
        startedAt: workspaceRun.startedAt,
        aborting: !!workspaceRun.signal?.aborted,
        result: workspaceRun.result || null,
        error: workspaceRun.error || null,
      }
    : { active: false };
}

function runIdFromJsonl(file) {
  return path.basename(file, '.jsonl');
}

function findRunJsonl(id) {
  const base = path.basename(String(id || ''));
  if (!/^[a-zA-Z0-9_.-]+$/.test(base)) return null;
  const p = path.join(REC_DIR, `${base}.jsonl`);
  return fs.existsSync(p) ? p : null;
}

function summarizeJsonlMeta(file) {
  let meta = null, firstT = null, lastT = null, samples = 0, cmdCount = 0;
  try {
    for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
      if (!line.trim()) continue;
      let rec; try { rec = JSON.parse(line); } catch { continue; }
      if (rec.meta) { meta = rec.meta; continue; }
      if (typeof rec.t === 'number') {
        if (firstT == null) firstT = rec.t;
        lastT = rec.t;
      }
      if (rec.dir === 'cmd') cmdCount++;
      if (rec.dir === 'in') {
        let d; try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
        if (Array.isArray(d?.a)) samples++;
      }
    }
  } catch {}
  return { meta, samples, cmdCount, durationS: firstT != null && lastT != null ? (lastT - firstT) / 1000 : null };
}

function collectRuns() {
  let files = [];
  try {
    files = fs.readdirSync(REC_DIR)
      .filter((x) => x.endsWith('.jsonl'))
      .map((x) => path.join(REC_DIR, x))
      .sort((a, b) => fs.statSync(b).mtimeMs - fs.statSync(a).mtimeMs);
  } catch {}
  return files.map((file) => {
    const id = runIdFromJsonl(file);
    const stem = file.replace(/\.jsonl$/, '');
    const summary = safeReadJson(`${stem}.summary.json`);
    const evaluation = safeReadJson(`${stem}.evaluation.json`);
    const bundle = safeReadJson(`${stem}.bundle.json`);
    const directManifest = `${stem}.manifest.json`;
    const manifest = fs.existsSync(directManifest)
      ? safeReadJson(directManifest)
      : (bundle?.manifest ? safeReadJson(bundle.manifest) : null);
    const meta = summarizeJsonlMeta(file);
    const program = meta.meta?.owner?.program || manifest?.program || null;
    const plotBase = path.basename(stem);
    const planePlot = bundle?.plot?.planeOut || path.join(REC_DIR, 'plots', `${plotBase}_plane6.svg`);
    const motorPlot = bundle?.plot?.motorOut || path.join(REC_DIR, 'plots', `${plotBase}_motor6.svg`);
    const wallClock = meta.meta?.wallClock || summary?.meta?.wallClock || manifest?.wallClock || null;
    return {
      id,
      name: meta.meta?.name || manifest?.name || id,
      wallClock,
      mtime: fs.statSync(file).mtimeMs,
      durationS: summary?.durationS ?? meta.durationS,
      samples: summary?.samples ?? meta.samples,
      cmdCount: meta.cmdCount,
      verdict: evaluation?.quality?.verdict ?? null,
      features: {
        maxStepDeg: evaluation?.fullBadness?.motorStepMaxDeg ?? null,
        p99StepDeg: evaluation?.fullBadness?.motorStepP99MeanDeg ?? null,
        crossPeak: evaluation?.fullBadness?.poseCrossHpPeak ?? null,
        worstMotor: evaluation?.fullBadness?.worstMotorStep?.label ?? null,
        takeoffStepDeg: evaluation?.quality?.takeoff?.motorStepMaxDeg ?? null,
        landingStepDeg: evaluation?.quality?.landing?.motorStepMaxDeg ?? null,
      },
      profile: manifest?.condition?.profile || (id.includes('z_sweep') ? 'z-sweep' : id.includes('heave-step') ? 'heave-step' : null),
      program,
      condition: manifest?.condition || meta.meta?.owner?.condition || null,
      health: {
        okFailFrac: evaluation?.health?.okFailFrac ?? summary?.encoders?.failFrac ?? null,
        canEfOr: evaluation?.health?.efOr ?? summary?.canHealth?.efOr ?? null,
        rxDropSum: evaluation?.health?.rxDrop ?? summary?.canHealth?.rxDropSum ?? null,
        rxDropPerS: evaluation?.health?.rxDropPerS ?? null,
      },
      files: {
        recording: safeRelPath(file),
        summary: fs.existsSync(`${stem}.summary.json`) ? safeRelPath(`${stem}.summary.json`) : null,
        evaluation: fs.existsSync(`${stem}.evaluation.json`) ? safeRelPath(`${stem}.evaluation.json`) : null,
        manifest: fs.existsSync(directManifest) ? safeRelPath(directManifest) : (bundle?.manifest && fs.existsSync(bundle.manifest) ? safeRelPath(bundle.manifest) : null),
        bundle: fs.existsSync(`${stem}.bundle.json`) ? safeRelPath(`${stem}.bundle.json`) : null,
        planePlot: fs.existsSync(planePlot) ? safeRelPath(planePlot) : null,
        motorPlot: fs.existsSync(motorPlot) ? safeRelPath(motorPlot) : null,
      },
    };
  });
}

function loadRunDetail(id) {
  const file = findRunJsonl(id);
  if (!file) return null;
  const stem = file.replace(/\.jsonl$/, '');
  const summary = safeReadJson(`${stem}.summary.json`);
  const evaluation = safeReadJson(`${stem}.evaluation.json`);
  const bundle = safeReadJson(`${stem}.bundle.json`);
  const directManifest = `${stem}.manifest.json`;
  const manifest = fs.existsSync(directManifest)
    ? safeReadJson(directManifest)
    : (bundle?.manifest ? safeReadJson(bundle.manifest) : null);
  const commands = [];
  const samples = [];
  const rawRows = [];
  resetFK([0, 0, NEUTRAL_Z, 0, 0, 0]);
  try {
    for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
      if (!line.trim()) continue;
      let rec; try { rec = JSON.parse(line); } catch { continue; }
      if (rec.dir === 'cmd') {
        commands.push({ t: rec.t, cmd: rec.d });
        continue;
      }
      if (rec.dir !== 'in') continue;
      let d; try { d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d; } catch { continue; }
      if (!Array.isArray(d?.a)) continue;
      const fk = solveFK(d.a);
      rawRows.push({
        t: rec.t,
        a: d.a.map(Number),
        pose: fk.pose.map(Number),
        ok: d.ok,
        lhz: d.lhz,
        ef: d.can?.ef ?? d.ef,
        tx: d.can?.tx ?? d.tx,
        rxDrop: d.can?.rxDrop ?? 0,
      });
    }
  } catch {}
  const maxSamples = 900;
  const step = Math.max(1, Math.ceil(rawRows.length / maxSamples));
  for (let i = 0; i < rawRows.length; i += step) samples.push(rawRows[i]);
  const run = collectRuns().find((x) => x.id === id);
  return { run, summary, evaluation, manifest, bundle, commands, samples, sampleStep: step };
}

function relatedRunFiles(id) {
  const file = findRunJsonl(id);
  if (!file) return null;
  const stem = file.replace(/\.jsonl$/, '');
  const bundle = safeReadJson(`${stem}.bundle.json`);
  const candidates = new Set([
    file,
    `${stem}.manifest.json`,
    `${stem}.summary.json`,
    `${stem}.evaluation.json`,
    `${stem}.bundle.json`,
    bundle?.manifest,
    bundle?.summary,
    bundle?.evaluation,
    bundle?.plot?.planeOut,
    bundle?.plot?.motorOut,
    path.join(REC_DIR, 'plots', `${path.basename(stem)}_plane6.svg`),
    path.join(REC_DIR, 'plots', `${path.basename(stem)}_motor6.svg`),
  ].filter(Boolean));
  const out = [];
  const seen = new Set();
  for (const candidate of candidates) {
    const resolved = path.resolve(candidate);
    if (seen.has(resolved)) continue;
    seen.add(resolved);
    if (fs.existsSync(resolved) && !fs.statSync(resolved).isDirectory()) out.push(resolved);
  }
  return out;
}

function moveRunFiles(id, bucket) {
  if (!['archive', 'trash'].includes(bucket)) throw new Error('bad bucket');
  if (recStream && recPath && runIdFromJsonl(recPath) === id) throw new Error('cannot move active recording');
  const files = relatedRunFiles(id);
  if (!files) return null;
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const safeId = path.basename(id).replace(/[^a-zA-Z0-9_.-]/g, '_');
  const destDir = path.join(REC_DIR, bucket, `${stamp}_${safeId}`);
  fs.mkdirSync(destDir, { recursive: true });
  const moved = [];
  for (const file of files) {
    if (!fs.existsSync(file)) continue;
    const dest = path.join(destDir, path.basename(file));
    fs.renameSync(file, dest);
    moved.push({ from: safeRelPath(file), to: safeRelPath(dest) });
  }
  fs.writeFileSync(path.join(destDir, '_run-op.json'), JSON.stringify({
    op: bucket,
    id,
    movedAt: new Date().toISOString(),
    moved,
  }, null, 2));
  return { id, op: bucket, dest: safeRelPath(destDir), moved };
}

async function startWorkspaceRun(incoming = {}) {
  if (workspaceRun?.active) throw new Error('workspace run already active');
  if (activeSession) throw new Error('session already active');
  const config = loadPlatformConfig();
  const signal = { aborted: false };
  const opts = {
    name: typeof incoming.name === 'string' ? incoming.name.slice(0, 80) : '',
    blocks: Array.isArray(incoming.blocks) ? incoming.blocks : undefined,
    program: sanitizeProgramMeta(incoming.program) || null,
    zBias: Number(incoming.zBias),
    vmaxT: Number(incoming.vmaxT),
    vmaxR: Number(incoming.vmaxR),
    warmupMs: Number(incoming.warmupMs),
    homeMs: Number(incoming.homeMs),
    followSettleMs: Number(incoming.followSettleMs),
    landMs: Number(incoming.landMs),
    closeMs: Number(incoming.closeMs),
    releaseObserveMs: Number(incoming.releaseObserveMs),
  };
  const plan = await WorkspaceExecutor.buildWorkspacePlan(config, opts);
  if (!plan.audit.ok) throw new Error(`closed-loop audit failed: ${plan.audit.issues.join('; ')}`);
  const label = opts.name || `${plan.program ? plan.program.id : 'workspace'}_workspace_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}`;
  workspaceRun = {
    active: true,
    label,
    startedAt: new Date().toISOString(),
    signal,
    result: null,
    error: null,
  };
  const deps = {
    getLatest: latestTelemetryObject,
    getTransport: () => ({
      ...currentTransport,
      lastLineAt: lastLine ? lastLine.t : null,
      lastTelemetryAt: lastTelemetryAt || null,
      telemetryAgeMs: lastTelemetryAt ? Date.now() - lastTelemetryAt : null,
    }),
    getRecorder: publicRecState,
    startSession: (session) => startSessionState(session),
    phase: (token, phase) => setSessionPhase(token, phase),
    startRecording: (token, name) => {
      requireSessionToken(token);
      const labelName = typeof name === 'string' ? name.slice(0, 80) : activeSession.label;
      const p = recStart(labelName, {
        type: 'session',
        label: activeSession.label,
        token: activeSession.token,
        program: activeSession.program,
        condition: activeSession.condition,
      });
      pushSessionEvent('rec_start', { path: safeRelPath(p), recorder: publicRecState() });
      return publicRecState();
    },
    stopRecording: async (token, analyze = false) => {
      requireSessionToken(token);
      if (recOwner && recOwner.type === 'session' && recOwner.token !== activeSession.token) {
        throw new Error('recorder belongs to another session');
      }
      const stopped = await recStop();
      let artifacts = null;
      let artifactError = null;
      if (analyze !== false) {
        try {
          artifacts = stopped?.path ? createRecordingArtifacts(stopped.path, { session: publicSessionState(), program: activeSession.program || null, condition: activeSession.condition || null }) : null;
        } catch (err) {
          artifactError = err.message;
        }
      }
      pushSessionEvent('rec_stop', { recorder: stopped, artifacts, artifactError });
      return { recording: false, ...(stopped || {}), artifacts, artifactError };
    },
    finishSession: (token) => finishSessionState(token),
    abortSession: (token, reason) => {
      try { return finishSessionState(token, { abort: true, reason }); }
      catch { return publicSessionState(); }
    },
    send: (cmd, token) => sendRequiredCommand(cmd, token),
    analyzeRun: (id, optsForRun) => analyzeRun(id, optsForRun),
  };
  workspaceRun.promise = WorkspaceExecutor.runWorkspaceSession({ config, opts: { ...opts, name: label }, deps, signal })
    .then((result) => {
      if (workspaceRun) workspaceRun.result = result;
      pushSessionEvent('workspace_complete', { result });
      return result;
    })
    .catch((err) => {
      if (workspaceRun) workspaceRun.error = err.message;
      pushSessionEvent('workspace_error', { error: err.message });
      throw err;
    })
    .finally(() => {
      const done = workspaceRun;
      if (done) done.active = false;
      setTimeout(() => {
        if (workspaceRun === done) workspaceRun = null;
      }, 3000);
    });
  workspaceRun.promise.catch(() => {});
  return { state: publicWorkspaceRunState(), plan: { program: plan.program, condition: plan.condition, blocks: plan.blocks.length, description: plan.description } };
}

const requestHandler = (req, res) => {
  // REST: 最新資料
  if (req.url === '/api/latest') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(latestTelemetryBody());
    return;
  }
  if (req.url === '/api/transport') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({
      ...currentTransport,
      lastLineAt: lastLine ? lastLine.t : null,
      lastTelemetryAt: lastTelemetryAt || null,
      telemetryAgeMs: lastTelemetryAt ? Date.now() - lastTelemetryAt : null,
    }));
    return;
  }
  if (req.url === '/api/follow/diag') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(followDiagSummary(), null, 2));
    return;
  }
  if (req.url.split('?')[0] === '/api/follow/report') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(buildFollowReport(), null, 2));
    return;
  }
  if (req.url.split('?')[0] === '/api/transport/reconnect') {
    transport.forceReconnect('api');
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ reconnecting: true, transport: transport.getState() }));
    return;
  }
  if (req.url === '/api/runs') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ runs: collectRuns() }));
    return;
  }
  {
    const urlPath = req.url.split('?')[0];
    const m = urlPath.match(/^\/api\/runs\/([^/]+)\/(archive|delete)$/);
    if (m && req.method === 'POST') {
      const id = decodeURIComponent(m[1]);
      const op = m[2] === 'delete' ? 'trash' : 'archive';
      try {
        const result = moveRunFiles(id, op);
        if (!result) {
          res.writeHead(404, { 'Content-Type': 'application/json' });
          res.end(JSON.stringify({ error: 'run not found' }));
          return;
        }
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(result));
      } catch (err) {
        res.writeHead(400, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: err.message }));
      }
      return;
    }
  }
  {
    const urlPath = req.url.split('?')[0];
    const m = urlPath.match(/^\/api\/runs\/([^/]+)\/analyze$/);
    if (m && req.method === 'POST') {
      readRequestBody(req).then((body) => {
        let incoming = {};
        try { incoming = JSON.parse(body || '{}'); } catch {}
        const id = decodeURIComponent(m[1]);
        try {
          const artifacts = analyzeRun(id, {
            session: incoming.session || null,
            program: sanitizeProgramMeta(incoming.program) || null,
            condition: sanitizeConditionMeta(incoming.condition) || null,
          });
          if (!artifacts) {
            res.writeHead(404, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: 'run not found' }));
            return;
          }
          res.writeHead(200, { 'Content-Type': 'application/json' });
          res.end(JSON.stringify({ artifacts, detail: loadRunDetail(id) }));
        } catch (err) {
          res.writeHead(500, { 'Content-Type': 'application/json' });
          res.end(JSON.stringify({ error: err.message }));
        }
      }).catch((err) => {
        res.writeHead(400, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: err.message }));
      });
      return;
    }
  }
  if (req.url.startsWith('/api/runs/')) {
    const id = decodeURIComponent(req.url.split('?')[0].slice('/api/runs/'.length));
    const detail = loadRunDetail(id);
    if (!detail) {
      res.writeHead(404, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: 'run not found' }));
      return;
    }
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(detail));
    return;
  }
  if (req.url.split('?')[0] === '/api/run-file') {
    const u = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
    const p = dataFileFromRel(u.searchParams.get('p'));
    if (!p || !fs.existsSync(p) || fs.statSync(p).isDirectory()) {
      res.writeHead(404, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: 'file not found' }));
      return;
    }
    const ext = path.extname(p);
    fs.readFile(p, (err, data) => {
      if (err) { res.writeHead(404); res.end('Not found'); return; }
      res.writeHead(200, {
        'Content-Type': MIME[ext] || 'text/plain',
        'Cache-Control': 'no-store',
      });
      res.end(data);
    });
    return;
  }
  if (req.url === '/api/platform-config' && req.method === 'GET') {
    const config = loadPlatformConfig();
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({
      ...config,
      neutralZ: NEUTRAL_Z,
      homePose: relativeToAbsolutePose(config.homeRelative),
      landingPose: relativeToAbsolutePose(config.landingRelative),
    }));
    return;
  }
  if (req.url === '/api/platform-config' && req.method === 'POST') {
    readRequestBody(req).then(async (body) => {
      const current = loadPlatformConfig();
      const incoming = JSON.parse(body || '{}');
      const wantsHomeUpdate = finitePose(incoming.homePose);
      const wantsLandingUpdate = finitePose(incoming.landingPose);
      if (wantsHomeUpdate) incoming.homeRelative = absoluteToRelativePose(incoming.homePose);
      else delete incoming.homeRelative;
      if (wantsLandingUpdate) incoming.landingRelative = absoluteToRelativePose(incoming.landingPose);
      else delete incoming.landingRelative;
      delete incoming.homePose;
      delete incoming.landingPose;
      const saved = savePlatformConfig({ ...current, ...incoming });
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({
        ...saved,
        neutralZ: NEUTRAL_Z,
        homePose: relativeToAbsolutePose(saved.homeRelative),
        landingPose: relativeToAbsolutePose(saved.landingRelative),
      }));
    }).catch((err) => {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message }));
    });
    return;
  }
  if (req.url === '/api/home' && req.method === 'GET') {
    const pose = loadHomePose();
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ pose, relative: absoluteToRelativePose(pose), neutralZ: NEUTRAL_Z }));
    return;
  }
  if (req.url === '/api/home' && req.method === 'POST') {
    readRequestBody(req).then((body) => {
      const d = JSON.parse(body || '{}');
      const pose = finitePose(d.pose) ? d.pose : (finitePose(d.relative) ? relativeToAbsolutePose(d.relative) : null);
      if (!pose) {
        res.writeHead(400, { 'Content-Type': 'application/json' });
        res.end('{"error":"expected pose[6] or relative[6]"}');
        return;
      }
      const saved = saveHomePose(pose);
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ...saved, neutralZ: NEUTRAL_Z }));
    }).catch((err) => {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message }));
    });
    return;
  }

  // 系統辨識記錄器控制
  if (req.url.startsWith('/api/rec/start')) {
    if (activeSession) {
      res.writeHead(409, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: 'session active; use /api/session/rec/start', session: publicSessionState(), recorder: publicRecState() }));
      return;
    }
    try {
      const name = new URL(req.url, 'http://x').searchParams.get('name');
      const p = recStart(name, { type: 'manual', label: name || 'capture' });
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ...publicRecState(), path: p }));
    } catch (err) {
      res.writeHead(409, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message, recorder: publicRecState() }));
    }
    return;
  }
  if (req.url === '/api/rec/stop') {
    if (recOwner && recOwner.type === 'session') {
      res.writeHead(409, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: 'recorder is owned by active session', recorder: publicRecState() }));
      return;
    }
    Promise.resolve(recStop()).then((r) => {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ recording: false, ...(r || {}) }));
    }).catch((err) => {
      res.writeHead(500, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message, recording: false }));
    });
    return;
  }
  if (req.url === '/api/rec/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(publicRecState()));
    return;
  }

  if (req.url === '/api/workspace/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(publicWorkspaceRunState()));
    return;
  }
  if (req.url === '/api/workspace/run' && req.method === 'POST') {
    readRequestBody(req).then(async (body) => {
      let incoming = {};
      try { incoming = JSON.parse(body || '{}'); } catch {}
      const result = await startWorkspaceRun(incoming);
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(result));
    }).catch((err) => {
      res.writeHead(409, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message, workspace: publicWorkspaceRunState(), session: publicSessionState(), recorder: publicRecState() }));
    });
    return;
  }
  if (req.url === '/api/workspace/abort' && req.method === 'POST') {
    if (workspaceRun?.signal) {
      workspaceRun.signal.aborted = true;
      pushSessionEvent('workspace_abort_requested', { label: workspaceRun.label || null });
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(publicWorkspaceRunState()));
    } else {
      res.writeHead(409, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: 'no active workspace run', workspace: publicWorkspaceRunState() }));
    }
    return;
  }

  if (req.url === '/api/session/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(publicSessionState()));
    return;
  }
  if (req.url === '/api/session/events') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ events: sessionEvents.slice(-120), session: publicSessionState() }));
    return;
  }
  if (req.url === '/api/session/start' && req.method === 'POST') {
    readRequestBody(req).then(async (body) => {
      let incoming = {};
      try { incoming = JSON.parse(body || '{}'); } catch {}
      const started = startSessionState({
        label: typeof incoming.label === 'string' ? incoming.label.slice(0, 80) : 'program',
        phase: typeof incoming.phase === 'string' ? incoming.phase.slice(0, 40) : 'starting',
        program: sanitizeProgramMeta(incoming.program),
        condition: sanitizeConditionMeta(incoming.condition),
      });
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(started));
    }).catch((err) => {
      res.writeHead(activeSession ? 409 : 400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message, session: publicSessionState() }));
    });
    return;
  }
  if (req.url === '/api/session/phase' && req.method === 'POST') {
    readRequestBody(req).then((body) => {
      let incoming = {};
      try { incoming = JSON.parse(body || '{}'); } catch {}
      setSessionPhase(incoming.token, incoming.phase);
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(publicSessionState()));
    }).catch((err) => {
      res.writeHead(err.message === 'bad session token' ? 403 : 409, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message, session: publicSessionState() }));
    });
    return;
  }
  if (req.url === '/api/session/rec/start' && req.method === 'POST') {
    readRequestBody(req).then((body) => {
      let incoming = {};
      try { incoming = JSON.parse(body || '{}'); } catch {}
      if (!activeSession) {
        res.writeHead(409, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: 'no active session', session: publicSessionState(), recorder: publicRecState() }));
        return;
      }
      if (incoming.token !== activeSession.token) {
        res.writeHead(403, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: 'bad session token', session: publicSessionState(), recorder: publicRecState() }));
        return;
      }
      try {
        const label = typeof incoming.name === 'string' ? incoming.name.slice(0, 80) : activeSession.label;
        const p = recStart(label, { type: 'session', label: activeSession.label, token: activeSession.token, program: activeSession.program, condition: activeSession.condition });
        pushSessionEvent('rec_start', { path: safeRelPath(p), recorder: publicRecState() });
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(publicRecState()));
      } catch (err) {
        res.writeHead(409, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: err.message, recorder: publicRecState() }));
      }
    }).catch((err) => {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message }));
    });
    return;
  }
  if (req.url === '/api/session/rec/stop' && req.method === 'POST') {
    readRequestBody(req).then(async (body) => {
      let incoming = {};
      try { incoming = JSON.parse(body || '{}'); } catch {}
      if (!activeSession) {
        res.writeHead(409, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: 'no active session', session: publicSessionState(), recorder: publicRecState() }));
        return;
      }
      if (incoming.token !== activeSession.token) {
        res.writeHead(403, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: 'bad session token', session: publicSessionState(), recorder: publicRecState() }));
        return;
      }
      if (recOwner && recOwner.type === 'session' && recOwner.token !== activeSession.token) {
        res.writeHead(403, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: 'recorder belongs to another session', recorder: publicRecState() }));
        return;
      }
      const stopped = await recStop();
      let artifacts = null;
      let artifactError = null;
      if (incoming.analyze !== false) {
        try {
          artifacts = stopped?.path ? createRecordingArtifacts(stopped.path, { session: publicSessionState(), program: activeSession.program || null, condition: activeSession.condition || null }) : null;
        } catch (err) {
          artifactError = err.message;
        }
      }
      pushSessionEvent('rec_stop', { recorder: stopped, artifacts, artifactError });
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ recording: false, ...(stopped || {}), artifacts, artifactError }));
    }).catch((err) => {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message }));
    });
    return;
  }
  if ((req.url === '/api/session/finish' || req.url === '/api/session/abort') && req.method === 'POST') {
    readRequestBody(req).then(async (body) => {
      let incoming = {};
      try { incoming = JSON.parse(body || '{}'); } catch {}
      if (!activeSession) {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(publicSessionState()));
        return;
      }
      const abortReq = req.url === '/api/session/abort';
      const emergencyAbort = abortReq && incoming.emergency === true;
      if (!emergencyAbort && incoming.token !== activeSession.token) {
        res.writeHead(403, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: 'bad session token', session: publicSessionState() }));
        return;
      }
      const endedSession = publicSessionState();
      let stopped = null;
      let artifacts = null;
      let artifactError = null;
      if (recOwner && recOwner.type === 'session' && (!recOwner.token || recOwner.token === activeSession.token)) {
        stopped = await recStop();
        try {
          artifacts = stopped?.path ? createRecordingArtifacts(stopped.path, { session: endedSession, program: activeSession.program || null, condition: activeSession.condition || null }) : null;
        } catch (err) {
          artifactError = err.message;
        }
      }
      activeSession = null;
      pushSessionEvent(abortReq ? (emergencyAbort ? 'session_emergency_abort' : 'session_abort') : 'session_finish', {
        reason: incoming.reason || null,
        session: { ...endedSession, active: false, endedAt: new Date().toISOString() },
        recorder: stopped,
        artifacts,
        artifactError,
      });
      broadcastSessionState();
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(publicSessionState()));
    }).catch((err) => {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message }));
    });
    return;
  }

  // 釋放傳輸（供韌體上傳用，暫停重連）。語意 transport-aware（見 transport.release）。
  if (req.url.split('?')[0] === '/api/release') {
    const u = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
    const requestedMs = Number(u.searchParams.get('ms')) || RELEASE_HOLD_MS;
    const holdMs = Math.max(1000, Math.min(RELEASE_HOLD_MAX_MS, requestedMs));
    transport.release(holdMs);
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ released: true, holdMs }));
    return;
  }

  // HTTP-pull OTA：板子 outbound 抓 firmware.bin（繞 host 防火牆，比 ArduinoOTA 回連可靠、可全自動）。
  // 不 release transport（控制鏈保持；板子走獨立 HTTP 連線抓檔，flash 完才 reboot）。
  if (req.url.split('?')[0] === '/firmware.bin') {
    const fwPath = path.join(__dirname, '.pio', 'build', 'esp32', 'firmware.bin');
    fs.stat(fwPath, (err, st) => {
      if (err) { res.writeHead(404, { 'Content-Type': 'text/plain' }); res.end('firmware.bin not found; run `pio run` first'); return; }
      res.writeHead(200, { 'Content-Type': 'application/octet-stream', 'Content-Length': st.size });
      fs.createReadStream(fwPath).pipe(res);
    });
    return;
  }
  if (req.url.split('?')[0] === '/api/ota/http') {
    const info = transport.otaPullInfo();
    if (info.state !== 'connected') {
      res.writeHead(503, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: false, reason: `transport ${info.state}; board not reachable` }));
      return;
    }
    const host = (info.kind === 'wifi' && info.localAddress) ? info.localAddress : lanIpForBoard();
    if (!host) {
      res.writeHead(503, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: false, reason: 'cannot determine server LAN IP; use `npm run upload` (USB)' }));
      return;
    }
    const url = `http://${host}:${HTTP_PORT}/firmware.bin`;
    const wr = transport.write(`OTA ${url}`);
    console.log(`[OTA] HTTP-pull trigger ${url} → ${wr.ok ? 'sent' : wr.reason}`);
    res.writeHead(wr.ok ? 200 : 502, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ ok: wr.ok, url, transport: info.kind, reason: wr.reason }));
    return;
  }

  // 靜態檔案（去查詢字串；目錄/結尾 → index.html）
  let urlPath = req.url.split('?')[0];
  if (urlPath === '/') urlPath = '/index.html';
  else if (urlPath.endsWith('/')) urlPath += 'index.html';
  // /sysid/*.js 從 sysid/ serve → 瀏覽器與 Node 共用同一份 kin.js/disturb_modes.js（SoT，不造第二真相源）
  const fromSysid = urlPath.startsWith('/sysid/') && urlPath.endsWith('.js');
  // 各自的服務沙箱：sysid 只到 sysid/，其餘只到 web/。
  const rootDir = fromSysid ? path.join(__dirname, 'sysid') : path.join(__dirname, 'web');
  const relPath = fromSysid ? urlPath.slice('/sysid'.length) : urlPath;
  const filePath = path.join(rootDir, relPath);
  // 防目錄遍歷：正規化後必須仍在自己的沙箱內（擋 /sysid/../server.js 之類 ../ 逃逸）
  if (filePath !== rootDir && !filePath.startsWith(rootDir + path.sep)) {
    res.writeHead(403); res.end('Forbidden'); return;
  }
  const ext = path.extname(filePath);
  fs.readFile(filePath, (err, data) => {
    if (err) { res.writeHead(404); res.end('Not found'); return; }
    res.writeHead(200, {
      'Content-Type': MIME[ext] || 'text/plain',
      'Permissions-Policy': 'accelerometer=(self), gyroscope=(self), magnetometer=(self)',
    });
    res.end(data);
  });
};

const server = http.createServer(requestHandler);

// ===== WebSocket =====
const wsServers = [];
let clientCount = 0;

// 對等廣播匯流排：所有 client（monitor 頁、腳本）共用同一條送出路徑。
// telemetry（序列埠回覆）與低頻指令 echo 走這裡；PF 這種資料面串流不廣播，避免拖慢 UI。
function broadcast(line) {
  for (const wss of wsServers) {
    for (const client of wss.clients) if (client.readyState === 1) client.send(line);
  }
}

function handleWsConnection(ws, req) {
  // 安全：拒絕跨站瀏覽器連線（惡意網頁經 DNS rebinding / CSWSH 可直送指令驅動實體馬達）。
  // 放行本機 + RFC1918 私網來源（保住站機台旁用手機/平板從 LAN IP 控制）；
  // 跨站攻擊頁的 Origin 是攻擊者公網域名、不符此段 → 擋掉。無 Origin 的 Node 腳本放行。
  const origin = req.headers.origin;
  const ALLOW_ORIGIN = /^https?:\/\/(localhost|127\.0\.0\.1|10\.\d+\.\d+\.\d+|192\.168\.\d+\.\d+|172\.(1[6-9]|2\d|3[01])\.\d+\.\d+)(:\d+)?$/;
  if (origin && !ALLOW_ORIGIN.test(origin)) {
    console.warn(`[WS] rejected cross-origin connection: ${origin}`);
    ws.close(1008, 'origin not allowed');
    return;
  }
  clientCount++;
  console.log(`[WS] client connected (${clientCount})`);

  // 立即送出最新資料 + 當前傳輸鏈狀態（新 client 不必等下次切換才知道走哪條路）
  if (lastTelemetry) ws.send(lastTelemetry);
  ws.send(JSON.stringify({ evt: 'transport', ...currentTransport }));
  ws.send(JSON.stringify({ evt: 'session', ...publicSessionState() }));

  ws.on('message', (msg) => {
    const raw = msg.toString().trim();
    let cmd = raw;
    let sessionToken = null;
    if (raw.startsWith('{')) {
      try {
        const d = JSON.parse(raw);
        if (typeof d.cmd === 'string') cmd = d.cmd.trim();
        if (typeof d.sessionToken === 'string') sessionToken = d.sessionToken;
      } catch {}
    }
    if (!cmd) return;
    sendCommand(cmd, sessionToken);
  });

  ws.on('close', () => {
    clientCount--;
    console.log(`[WS] client disconnected (${clientCount})`);
    if (clientCount <= 0) manualPfStop('all clients disconnected');
  });
}

function attachWebSocketServer(httpServer) {
  const wss = new WebSocketServer({ server: httpServer });
  wsServers.push(wss);
  wss.on('connection', handleWsConnection);
  return wss;
}

attachWebSocketServer(server);

// ===== 傳輸層 =====
// 傳輸無關的「收到一行」處理：telemetry/狀態/ack 都走這 → broadcast 給所有 WS client。
// Serial 與 TCP（P6）共用，確保前端對底層傳輸零感知。
function onLine(line) {
  line = line.trim();
  if (!line) return;
  lastLine = { t: Date.now(), line };
  let d;
  try {
    d = JSON.parse(line);
  } catch (err) {
    observeLineParseError(line, err);
    recWrite('in', line);
    return;
  }
  if (d && Array.isArray(d.a)) {
    lastTelemetry = line;
    lastTelemetryAt = Date.now();
  }
  observeIncoming(d);
  recWrite('in', line);
  broadcast(line);
}

// 學到的 ESP32 WiFi IP 持久化：避開本機 Tailscale 對 .local 的不穩解析，直連 IP 最可靠。
function loadCachedIp() { try { return fs.readFileSync(IP_CACHE, 'utf8').trim() || null; } catch { return null; } }
function saveCachedIp(ip) { try { fs.writeFileSync(IP_CACHE, ip); } catch {} }

// HTTP-pull OTA：找一個「板子能到達」的本機 LAN IPv4（serial transport 或無 socket localAddress 時的後備）。
// 優先取與快取板子 IP 同 /24 網段者；否則排除 Tailscale CGNAT(100.x) 取第一個非內部位址。
function lanIpForBoard() {
  const all = [];
  const ifaces = os.networkInterfaces();
  for (const name of Object.keys(ifaces)) {
    for (const ni of ifaces[name] || []) {
      if (ni.family === 'IPv4' && !ni.internal) all.push(ni.address);
    }
  }
  const boardIp = loadCachedIp();
  if (boardIp) {
    const sub = boardIp.split('.').slice(0, 3).join('.') + '.';
    const m = all.find(a => a.startsWith(sub));
    if (m) return m;
  }
  return all.find(a => !a.startsWith('100.')) || all[0] || null;
}

function pidAlive(pid) {
  if (!Number.isInteger(pid) || pid <= 0) return false;
  try { process.kill(pid, 0); return true; }
  catch { return false; }
}

function acquireTransportLock() {
  if (TRANSPORT_LOCK_DISABLED) return { ok: true, disabled: true };
  let existing = null;
  try { existing = JSON.parse(fs.readFileSync(TRANSPORT_LOCK_PATH, 'utf8')); } catch {}
  if (existing && pidAlive(existing.pid)) {
    return { ok: false, existing };
  }
  const lock = {
    pid: process.pid,
    cwd: process.cwd(),
    httpPort: HTTP_PORT,
    esp32Port: ESP32_PORT,
    startedAt: new Date().toISOString(),
  };
  fs.writeFileSync(TRANSPORT_LOCK_PATH, JSON.stringify(lock, null, 2));
  const release = () => {
    try {
      const cur = JSON.parse(fs.readFileSync(TRANSPORT_LOCK_PATH, 'utf8'));
      if (cur.pid === process.pid) fs.unlinkSync(TRANSPORT_LOCK_PATH);
    } catch {}
  };
  process.once('exit', release);
  process.once('SIGINT', () => { release(); process.exit(130); });
  process.once('SIGTERM', () => { release(); process.exit(143); });
  return { ok: true, lock };
}

// ---- attempt-once 啞連線：只負責「開連線/吐行/報一次生死」，不自排重連（policy 全歸 manager）----
function openSerial({ path: portPath, baud, onLine, onClose }) {
  let closed = false;
  const gone = (why) => { if (closed) return; closed = true; onClose(why); };
  let sp;
  try { sp = new SerialPort({ path: portPath, baudRate: baud }); }
  catch (e) { setImmediate(() => gone('open-fail:' + e.message)); return { write() {}, close() {} }; }
  sp.pipe(new ReadlineParser({ delimiter: '\n' })).on('data', onLine);
  sp.on('close', () => gone('disconnected'));
  sp.on('error', (e) => { console.error(`[Serial] ${e.message}`); gone('error'); });
  return {
    write(line) { if (!sp.isOpen) return false; return sp.write(line + '\n'); },
    close() { if (closed) return; closed = true; try { sp.close(); } catch {} },
  };
}

// TCP 啞連線：連 ESP32 :3333（協議與 serial 逐字相同）。心跳 write 獨立於 liveness，
// 供韌體可選 HB 失效保護；半開/idle 偵測由 manager 統一管（單一 owner，不雙頭）。
function openTcp({ host, port, onLine, onClose }) {
  let closed = false, hb = null;
  const sock = net.connect({ host, port });
  const gone = (why) => {
    if (closed) return;
    closed = true;
    if (hb) clearInterval(hb);
    try { if (!sock.destroyed) sock.destroy(); } catch {}
    onClose(why);
  };
  sock.setNoDelay(true);
  sock.setKeepAlive(true, 5000);
  sock.setTimeout(TCP_CONNECT_TIMEOUT_MS);
  sock.pipe(new ReadlineParser({ delimiter: '\n' })).on('data', onLine);
  sock.on('connect', () => {
    sock.setTimeout(0);
    hb = setInterval(() => { if (!sock.destroyed && sock.writable) sock.write('\n'); }, TCP_HB_MS);
  });
  sock.on('timeout', () => gone('connect-timeout'));
  sock.on('close', () => gone('closed'));
  sock.on('error', (e) => {
    console.error(`[TCP] ${e.message}`);
    gone(`error:${e.code || e.message}`);
  });
  return {
    write(line) { if (sock.destroyed || !sock.writable) return false; return sock.write(line + '\n'); },
    close() { if (closed) return; closed = true; if (hb) clearInterval(hb); try { sock.destroy(); } catch {} },
    localAddress() { return sock.localAddress || null; },   // HTTP-pull OTA：此連線本端位址 = 板子能到達的 server IP
  };
}

// ===== TransportManager：單一 reconnect/選路擁有者 =====
// 偵測：有線(USB)→serial，沒線→WiFi(直連學到的 IP)。每次切換/連上/斷線都廣播狀態（無 silent fallback）。
// liveness gate：埠開/socket 開 ≠ 連上——須在 LIVENESS_MS 內收到有效遙測才算 connected，否則判死換路。
function createTransportManager({ onLine, onState }) {
  let conn = null, kind = null, state = 'init', detail = '';
  let suppressReconnect = false, switching = false;
  let pendingTimer = null;          // 單一重連 timer（互斥、不堆疊）—— 沿用 e0ecae7 不變式
  let pendingDueAt = 0;
  let livenessTimer = null, idleTimer = null, serialPollTimer = null, serialSeen = 0;
  let recoveryTimer = null;
  let lastRx = 0, lastPos = 0;
  let reconnectDelayMs = RECONNECT_MS;
  let wifiHostIndex = 0;

  function resetReconnectDelay() { reconnectDelayMs = RECONNECT_MS; }
  function clearPending() {
    if (pendingTimer) clearTimeout(pendingTimer);
    pendingTimer = null;
    pendingDueAt = 0;
  }
  function scheduleSelect() {
    clearPending();
    const delay = reconnectDelayMs;
    reconnectDelayMs = Math.min(RECONNECT_MAX_MS, Math.round(reconnectDelayMs * 1.7));
    pendingDueAt = Date.now() + delay;
    pendingTimer = setTimeout(() => {
      pendingTimer = null;
      pendingDueAt = 0;
      select();
    }, delay);
  }

  function setState(s, k, d) {
    state = s; if (k !== undefined) kind = k; if (d !== undefined) detail = d;
    console.log(`[Transport] ${state}${kind ? ' ' + kind : ''}${detail ? ' (' + detail + ')' : ''}`);
    onState({ kind, state, detail });
  }
  function clearWatches() {
    if (livenessTimer) { clearTimeout(livenessTimer); livenessTimer = null; }
    if (idleTimer) { clearInterval(idleTimer); idleTimer = null; }
    if (serialPollTimer) { clearInterval(serialPollTimer); serialPollTimer = null; serialSeen = 0; }
  }

  function startRecoveryWatch() {
    if (recoveryTimer) clearInterval(recoveryTimer);
    recoveryTimer = setInterval(() => {
      if (suppressReconnect || switching || conn) return;
      if (pendingTimer) {
        if (pendingDueAt && Date.now() > pendingDueAt + RECOVERY_WATCH_MS * 2) {
          console.warn('[Transport] reconnect timer stale; rescheduling');
          scheduleSelect();
        }
        return;
      }
      if (state !== 'connected') {
        console.warn(`[Transport] recovery watchdog scheduling select from ${state}${detail ? ` (${detail})` : ''}`);
        scheduleSelect();
      }
    }, RECOVERY_WATCH_MS);
  }

  async function findSerial() {
    const ports = await SerialPort.list();
    const p = ports.find(x => SERIAL_PATTERN.test(x.path));
    return p ? p.path.replace('/dev/tty.', '/dev/cu.') : null;
  }
  function wifiCandidates() {
    if (ESP32_HOST_OVERRIDE) return [ESP32_HOST_OVERRIDE];
    return [...new Set([loadCachedIp(), 'stewart.local'].filter(Boolean))];
  }
  function wifiHost() {
    const hosts = wifiCandidates();
    return hosts[wifiHostIndex % hosts.length] || 'stewart.local';
  }
  function advanceWifiHost(why) {
    if (ESP32_HOST_OVERRIDE) return;
    const hosts = wifiCandidates();
    if (hosts.length <= 1) return;
    const prev = hosts[wifiHostIndex % hosts.length];
    wifiHostIndex = (wifiHostIndex + 1) % hosts.length;
    const next = hosts[wifiHostIndex % hosts.length];
    console.warn(`[Transport] WiFi host ${prev} failed (${why}); trying ${next}`);
  }

  // manager 統一「收到一行」：liveness/idle 計時、學 ESP32 IP、再轉發給匯流排
  function handleLine(line) {
    line = (line || '').trim();
    if (!line) return;
    lastRx = Date.now();
    let d = null; try { d = JSON.parse(line); } catch {}
    if (d && Array.isArray(d.a)) {                 // 有效遙測 = liveness 證據
      if (typeof d.pos === 'number') lastPos = d.pos;
      if (state === 'connecting') {                // 首筆遙測 → 確認 connected
        if (livenessTimer) { clearTimeout(livenessTimer); livenessTimer = null; }
        resetReconnectDelay();
        setState('connected');
        startIdleWatch();
        if (kind === 'wifi') startSerialPoll();    // WiFi 時才需偵測 USB 插入
      }
    }
    if (d && d.wifi && /^\d+\.\d+\.\d+\.\d+$/.test(d.wifi.ip || '') && d.wifi.ip !== '0.0.0.0') {
      if (loadCachedIp() !== d.wifi.ip) { saveCachedIp(d.wifi.ip); wifiHostIndex = 0; console.log(`[Transport] learned ESP32 IP ${d.wifi.ip}`); }
    }
    onLine(line);
  }

  function startIdleWatch() {
    if (idleTimer) clearInterval(idleTimer);
    idleTimer = setInterval(() => {
      if (state === 'connected' && Date.now() - lastRx > IDLE_MS) { console.log('[Transport] telemetry idle'); dropAndReselect('idle'); }
    }, 500);
  }

  // 有線就用線：WiFi 運行時低頻偵測 USB；連 2 次穩定且平台閒置(!pos)→切回 serial（announced）。
  // 平台控制中不熱切換（關 WiFi socket 會觸發韌體 failsafe）——等閒置。
  function startSerialPoll() {
    if (serialPollTimer) clearInterval(serialPollTimer);
    serialSeen = 0;
    serialPollTimer = setInterval(async () => {
      if (kind !== 'wifi' || switching) return;
      const path = await findSerial();
      if (!path) { serialSeen = 0; return; }
      if (++serialSeen < 2) return;                // 防抖
      if (lastPos) { return; }                     // 平台運動中 → 待閒置
      console.log(`[Transport] USB 出現且平台閒置 → 切回 serial (${path})`);
      switching = true; clearWatches();
      if (conn) { conn.close(); conn = null; }
      setState('switching', 'serial', path);
      clearPending();
      resetReconnectDelay();
      pendingDueAt = Date.now() + RECONNECT_MS;
      pendingTimer = setTimeout(() => {
        pendingTimer = null;
        pendingDueAt = 0;
        switching = false;
        select();
      }, RECONNECT_MS);
    }, SERIAL_POLL_MS);
  }

  function dropAndReselect(why) {
    const failedKind = kind;
    clearWatches();
    if (conn) { try { conn.close(); } catch {} conn = null; }
    if (failedKind === 'wifi') advanceWifiHost(why);
    setState('down', kind, why);
    if (suppressReconnect) return;
    scheduleSelect();
  }

  async function select() {
    if (suppressReconnect || conn) return;
    clearPending();
    setState('selecting', null, '');
    const path = await findSerial();
    if (path) { setState('connecting', 'serial', path); conn = openSerial({ path, baud: BAUD, onLine: handleLine, onClose: (w) => dropAndReselect(w) }); }
    else {
      const host = wifiHost();
      const hosts = wifiCandidates();
      const suffix = hosts.length > 1 ? ` host ${wifiHostIndex % hosts.length + 1}/${hosts.length}` : '';
      setState('connecting', 'wifi', `${host}:${ESP32_PORT}${suffix}`);
      conn = openTcp({ host, port: ESP32_PORT, onLine: handleLine, onClose: (w) => dropAndReselect(w) });
    }
    lastRx = Date.now();
    if (livenessTimer) clearTimeout(livenessTimer);
    livenessTimer = setTimeout(() => {              // liveness gate：開了但不講話 → 判死換路（非 silent 坐死）
      if (state === 'connecting') { console.log(`[Transport] ${kind} 開了但 ${LIVENESS_MS}ms 無遙測`); dropAndReselect('no-telemetry'); }
    }, LIVENESS_MS);
    setTimeout(() => { if (conn) conn.write('WIFI?'); }, 800);   // 學/刷新 ESP32 的 WiFi IP（供下次直連）
  }

  return {
    start() { startRecoveryWatch(); select(); },
    isReady() { return state === 'connected'; },
    write(line) {
      if (!conn) return { ok: false, reason: 'no transport' };
      if (state !== 'connected') return { ok: false, reason: `transport ${state}` };
      const ok = conn.write(line);
      return ok ? { ok: true, kind } : { ok: false, reason: `${kind || 'transport'} not writable` };
    },
    release(holdMs) {                               // 燒錄：暫停重連、放掉連線、holdMs 後重選
      suppressReconnect = true;
      clearWatches();
      clearPending();
      if (conn) { conn.close(); conn = null; }
      setState('down', kind, `released ${holdMs}ms`);
      resetReconnectDelay();
      pendingDueAt = Date.now() + holdMs;
      pendingTimer = setTimeout(() => {
        pendingTimer = null;
        pendingDueAt = 0;
        suppressReconnect = false;
        select();
      }, holdMs);
    },
    forceReconnect(reason = 'manual') {
      suppressReconnect = false;
      clearWatches();
      clearPending();
      if (conn) { try { conn.close(); } catch {} conn = null; }
      setState('down', kind, `force-reconnect:${reason}`);
      resetReconnectDelay();
      scheduleSelect();
    },
    otaPullInfo() {
      return { kind, state, localAddress: (conn && typeof conn.localAddress === 'function') ? conn.localAddress() : null };
    },
    getState() { return { kind, state, detail }; },
  };
}

// 當前傳輸鏈狀態（server↔ESP32）；廣播給 UI、新 client 連上立即補送 → UI 永遠知道走哪條路
let currentTransport = { kind: null, state: 'init', detail: '' };
const transport = createTransportManager({
  onLine,
  onState: (st) => {
    currentTransport = st;
    if (st.state !== 'connected') clearPfMailbox(`transport ${st.state}`);
    broadcast(JSON.stringify({ evt: 'transport', ...st }));
  },
});

function localHttpsSanList() {
  const sans = new Set(['DNS:localhost', 'IP:127.0.0.1']);
  for (const nets of Object.values(os.networkInterfaces())) {
    for (const n of nets || []) {
      if (n.family === 'IPv4' && !n.internal) sans.add(`IP:${n.address}`);
    }
  }
  return [...sans];
}

function localHttpsSans() {
  return localHttpsSanList().join(',');
}

function certHasCurrentSans() {
  if (!fs.existsSync(HTTPS_CERT_PATH)) return false;
  try {
    const out = execFileSync('openssl', ['x509', '-in', HTTPS_CERT_PATH, '-noout', '-ext', 'subjectAltName'], { encoding: 'utf8' });
    return localHttpsSanList().every((san) => {
      const needle = san.startsWith('IP:') ? `IP Address:${san.slice(3)}` : san;
      return out.includes(needle);
    });
  } catch {
    return false;
  }
}

function loadHttpsCredentials() {
  try {
    fs.mkdirSync(CONFIG_DIR, { recursive: true });
    if (!fs.existsSync(HTTPS_KEY_PATH) || !certHasCurrentSans()) {
      execFileSync('openssl', [
        'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-sha256',
        '-days', '3650',
        '-keyout', HTTPS_KEY_PATH,
        '-out', HTTPS_CERT_PATH,
        '-subj', '/CN=stewart-platform.local',
        '-addext', `subjectAltName=${localHttpsSans()}`,
      ], { stdio: 'ignore' });
    }
    return {
      key: fs.readFileSync(HTTPS_KEY_PATH),
      cert: fs.readFileSync(HTTPS_CERT_PATH),
    };
  } catch (err) {
    console.warn(`[Server] HTTPS disabled: ${err.message}`);
    console.warn('[Server] Install openssl or provide HTTPS_KEY/HTTPS_CERT to use phone IMU sensors.');
    return null;
  }
}

function startTransport() {
  const lock = acquireTransportLock();
  if (!lock.ok) {
    const e = lock.existing || {};
    console.error(`[Server] ESP32 transport lock is already held by pid ${e.pid || '?'} (${e.cwd || 'unknown cwd'}, HTTP ${e.httpPort || '?'})`);
    console.error(`[Server] Refusing to connect to ESP32 :${ESP32_PORT}; stop the other dashboard server or run with STEWART_TRANSPORT_LOCK=0 only for deliberate debugging.`);
    process.exit(1);
  }
  if (lock.disabled) console.warn('[Server] ⚠ ESP32 transport lock disabled by STEWART_TRANSPORT_LOCK=0');
  transport.start();
}

// ===== 啟動 =====
server.listen(HTTP_PORT, BIND_HOST, () => {
  console.log(`[Server] http://localhost:${HTTP_PORT}  (bind ${BIND_HOST})`);
  console.log(`[Server] REST: http://localhost:${HTTP_PORT}/api/latest`);
  console.log('[Server] transport: 自動偵測（有線→serial / 沒線→WiFi）');
  if (BIND_HOST === '0.0.0.0')
    console.log('[Server] ⚠ 全網段可達。不可信網路請以 LOOPBACK_ONLY=1 啟動只綁本機。');
  startTransport();
});

if (HTTPS_ENABLED) {
  const creds = loadHttpsCredentials();
  if (creds) {
    const httpsServer = https.createServer(creds, requestHandler);
    attachWebSocketServer(httpsServer);
    httpsServer.listen(HTTPS_PORT, BIND_HOST, () => {
      console.log(`[Server] https://localhost:${HTTPS_PORT}/phone.html  (bind ${BIND_HOST})`);
      console.log(`[Server] Phone IMU page: https://<this-computer-LAN-IP>:${HTTPS_PORT}/phone.html`);
    });
  }
}
