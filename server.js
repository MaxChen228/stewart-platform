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

function movementCommand(cmd) {
  const op = String(cmd || '').trim().split(/\s+/)[0].toUpperCase();
  return [
    'H', 'E', 'P', 'PF', 'FOLLOW',
    'U', 'W', 'K', 'KS', 'KRESET', 'V', 'VF', 'J', 'M',
    'L', 'AR', 'A', 'C', 'OD', 'CM', 'T0', 'T1', 'T2', 'T3', 'T4', 'T5',
    'Z', 'Z0', 'Z1', 'Z2', 'Z3', 'Z4', 'Z5',
  ].includes(op);
}

function commandAllowed(cmd, token) {
  if (!activeSession) return { ok: true };
  const op = String(cmd || '').trim().split(/\s+/)[0].toUpperCase();
  if (token && token === activeSession.token) return { ok: true };
  if (op === 'D' || op === 'S') return { ok: true, emergency: true };
  if (!movementCommand(cmd)) return { ok: true };
  return { ok: false, reason: `session active: ${activeSession.label || activeSession.owner}` };
}

function sendCommand(cmd, sessionToken = null) {
  const gate = commandAllowed(cmd, sessionToken);
  if (!gate.ok) {
    console.warn(`[WS] blocked cmd (${gate.reason}): ${cmd}`);
    pushSessionEvent('command_blocked', { cmd, reason: gate.reason, transport: currentTransport });
    recWrite('drop', { cmd, reason: gate.reason, transport: currentTransport, session: publicSessionState() });
    broadcast(JSON.stringify({ evt: 'cmd_dropped', c: cmd, reason: gate.reason, transport: currentTransport, session: publicSessionState() }));
    return { ok: false, reason: gate.reason };
  }
  console.log(`[WS] cmd: ${cmd}`);
  const wr = transport.write(cmd);
  if (wr.ok) {
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
// telemetry（序列埠回覆）與指令 echo 都走這裡 → 任何來源的操作對所有人可見。
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
  try {
    const d = JSON.parse(line);
    if (d && Array.isArray(d.a)) {
      lastTelemetry = line;
      lastTelemetryAt = Date.now();
    }
  } catch {}
  recWrite('in', line);
  broadcast(line);
}

// 學到的 ESP32 WiFi IP 持久化：避開本機 Tailscale 對 .local 的不穩解析，直連 IP 最可靠。
function loadCachedIp() { try { return fs.readFileSync(IP_CACHE, 'utf8').trim() || null; } catch { return null; } }
function saveCachedIp(ip) { try { fs.writeFileSync(IP_CACHE, ip); } catch {} }

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
  function wifiHost() { return ESP32_HOST_OVERRIDE || loadCachedIp() || 'stewart.local'; }

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
      if (loadCachedIp() !== d.wifi.ip) { saveCachedIp(d.wifi.ip); console.log(`[Transport] learned ESP32 IP ${d.wifi.ip}`); }
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
    clearWatches();
    if (conn) { try { conn.close(); } catch {} conn = null; }
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
    else { const host = wifiHost(); setState('connecting', 'wifi', `${host}:${ESP32_PORT}`); conn = openTcp({ host, port: ESP32_PORT, onLine: handleLine, onClose: (w) => dropAndReselect(w) }); }
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
    getState() { return { kind, state, detail }; },
  };
}

// 當前傳輸鏈狀態（server↔ESP32）；廣播給 UI、新 client 連上立即補送 → UI 永遠知道走哪條路
let currentTransport = { kind: null, state: 'init', detail: '' };
const transport = createTransportManager({
  onLine,
  onState: (st) => { currentTransport = st; broadcast(JSON.stringify({ evt: 'transport', ...st })); },
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
