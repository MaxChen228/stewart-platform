#!/usr/bin/env node
'use strict';

// One-shot research trial runner.
//
// Default is dry-run. A live run intentionally requires both --live and
// --i-am-at-rig so automation cannot accidentally move hardware.

const fs = require('fs');
const path = require('path');
const http = require('http');
const { execFileSync } = require('child_process');
const WebSocket = require('ws');
const { NEUTRAL_Z } = require('./kin');
const PlatformSoT = require('./platform_sot');

const DATA_DIR = path.join(__dirname, 'data');
const DEFAULTS = PlatformSoT.DEFAULT_PLATFORM_CONFIG;

function usage() {
  console.log(`Usage:
  node sysid/research_trial.js [options]

Profiles:
  heave-step          Return to configured home, move to relative heave target, safe-land.

Options:
  --live              Actually send motor commands. Default: dry-run.
  --i-am-at-rig       Required with --live; confirms human supervision and reachable power cut.
  --host HOST         Server host (default localhost:3000)
  --name NAME         Recording/manifest prefix (default research_trial)
  --profile NAME      Profile name (default heave-step)
  --heave MM          Relative heave target for heave-step (default 60)
  --z-bias MM         Add command-side Z bias to recorded home/target only (default 0)
  --ms N              Main move duration, ms (default 5000)
  --settle-ms N       Settle time after main move, ms (default 800)
  --home-ms N         Move duration to configured home, ms (default 1500)
  --land-ms N         Move duration to support landing pose, ms (default 1500)
  --landing POSE      Relative support landing pose (default 0,0,10,0,0,0)
  --no-safe-land      End at target and keep HOLD instead of landing/release.
  --record-main-only  Stop recording after the main move/settle, then safe-land outside the dataset.
  --no-plot           Skip plot generation.

Example:
  node sysid/research_trial.js
  node sysid/research_trial.js --live --i-am-at-rig --name heave60 --heave 60 --ms 5000
`);
}

function parsePose(s) {
  const p = String(s).split(',').map((x) => Number(x.trim()));
  if (p.length !== 6 || p.some((x) => !Number.isFinite(x))) throw new Error(`Bad pose: ${s}`);
  return p;
}

function parseArgs(argv) {
  const opts = {
    live: false,
    atRig: false,
    host: 'localhost:3000',
    name: 'research_trial',
    profile: DEFAULTS.trialDefaults.profile,
    heave: DEFAULTS.trialDefaults.heave,
    zBias: 0,
    ms: DEFAULTS.trialDefaults.ms,
    settleMs: DEFAULTS.trialDefaults.settleMs,
    homeMs: DEFAULTS.trialDefaults.homeMs,
    landMs: DEFAULTS.trialDefaults.landMs,
    landing: [...DEFAULTS.landingRelative],
    safeLand: DEFAULTS.trialDefaults.safeLand,
    recordMainOnly: false,
    plot: true,
    _set: new Set(),
  };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--live') opts.live = true;
    else if (a === '--i-am-at-rig') opts.atRig = true;
    else if (a === '--host') opts.host = next();
    else if (a === '--name') opts.name = next();
    else if (a === '--profile') { opts.profile = next(); opts._set.add('profile'); }
    else if (a === '--heave') { opts.heave = Number(next()); opts._set.add('heave'); }
    else if (a === '--z-bias') { opts.zBias = Number(next()); opts._set.add('zBias'); }
    else if (a === '--ms') { opts.ms = Number(next()); opts._set.add('ms'); }
    else if (a === '--settle-ms') { opts.settleMs = Number(next()); opts._set.add('settleMs'); }
    else if (a === '--home-ms') { opts.homeMs = Number(next()); opts._set.add('homeMs'); }
    else if (a === '--land-ms') { opts.landMs = Number(next()); opts._set.add('landMs'); }
    else if (a === '--landing') { opts.landing = parsePose(next()); opts._set.add('landing'); }
    else if (a === '--no-safe-land') { opts.safeLand = false; opts._set.add('safeLand'); }
    else if (a === '--record-main-only') opts.recordMainOnly = true;
    else if (a === '--no-plot') opts.plot = false;
    else throw new Error(`Unknown option: ${a}`);
  }
  if (opts.profile !== 'heave-step') throw new Error(`Unsupported profile: ${opts.profile}`);
  for (const k of ['heave', 'zBias', 'ms', 'settleMs', 'homeMs', 'landMs']) {
    if (!Number.isFinite(opts[k])) throw new Error(`Bad numeric option: ${k}`);
  }
  if (opts.live && !opts.atRig) throw new Error('Live run requires --i-am-at-rig');
  return opts;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function safeName(s) {
  return String(s || 'trial').replace(/[^a-zA-Z0-9_-]/g, '_');
}

function httpBase(host) {
  return host.startsWith('http://') || host.startsWith('https://') ? host : `http://${host}`;
}

function wsUrl(host) {
  const u = new URL(httpBase(host));
  u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
  u.pathname = '/';
  return u.toString();
}

function getJson(host, apiPath) {
  return new Promise((resolve, reject) => {
    http.get(new URL(apiPath, httpBase(host)), (res) => {
      let data = '';
      res.on('data', (c) => { data += c; });
      res.on('end', () => {
        try { resolve(JSON.parse(data || '{}')); }
        catch (e) { reject(e); }
      });
    }).on('error', reject);
  });
}

async function rest(host, apiPath) {
  return getJson(host, apiPath.startsWith('/api/') ? apiPath : `/api/${apiPath}`);
}

async function openWs(host) {
  const ws = new WebSocket(wsUrl(host));
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('WebSocket timeout')), 3000);
    ws.on('open', () => { clearTimeout(timer); resolve(); });
    ws.on('error', reject);
  });
  return ws;
}

function poseLine(pose) {
  return pose.map((x) => Number(x).toFixed(3)).join(' ');
}

function withZBias(pose, bias) {
  const out = [...pose];
  out[2] += bias;
  return out;
}

function relPose(rel) {
  return [rel[0], rel[1], NEUTRAL_Z + rel[2], rel[3], rel[4], rel[5]];
}

async function loadHome(host) {
  try {
    const home = await rest(host, '/api/platform-config');
    if (PlatformSoT.finitePose(home.homePose)) return home.homePose;
    if (PlatformSoT.finitePose(home.homeRelative)) return relPose(home.homeRelative);
  } catch {}
  return relPose(DEFAULTS.homeRelative);
}

async function preflight(opts) {
  const [latest, transport, rec, configRaw] = await Promise.all([
    rest(opts.host, '/api/latest'),
    rest(opts.host, '/api/transport').catch((e) => ({ error: e.message })),
    rest(opts.host, '/api/rec/status'),
    rest(opts.host, '/api/platform-config').catch(() => ({})),
  ]);
  const config = PlatformSoT.mergeConfig(configRaw);
  if (!opts._set.has('profile')) opts.profile = config.trialDefaults.profile;
  if (!opts._set.has('heave')) opts.heave = config.trialDefaults.heave;
  if (!opts._set.has('zBias')) opts.zBias = config.trialDefaults.zBias;
  if (!opts._set.has('ms')) opts.ms = config.trialDefaults.ms;
  if (!opts._set.has('settleMs')) opts.settleMs = config.trialDefaults.settleMs;
  if (!opts._set.has('homeMs')) opts.homeMs = config.trialDefaults.homeMs;
  if (!opts._set.has('landMs')) opts.landMs = config.trialDefaults.landMs;
  if (!opts._set.has('safeLand')) opts.safeLand = config.trialDefaults.safeLand;
  if (!opts._set.has('landing')) opts.landing = config.landingRelative;
  const home = PlatformSoT.finitePose(configRaw.homePose) ? configRaw.homePose : relPose(config.homeRelative);
  const failures = [];
  const warnings = [];
  if (!latest || latest.error || !Array.isArray(latest.a)) failures.push('no live telemetry from /api/latest');
  if (latest && latest.ok !== 6) failures.push(`encoder ok must be 6, got ${latest.ok}`);
  if (rec && rec.recording) failures.push(`recorder already running: ${rec.path}`);
  if (latest?.can?.backend === 'mcp2515' && latest?.can?.bitrate === 500000) {
    warnings.push('MCP2515/500K active; keep motion conservative and prefer clean recordings over high-rate stress.');
  }
  if (latest?.pid && latest.pid.join(',') !== '1024,0,0,0') {
    warnings.push(`motor PID is [${latest.pid.join(',')}], not pure-P [1024,0,0,0].`);
  }
  return { latest, transport, rec, home, config, failures, warnings };
}

function buildPlan(opts, home) {
  const target = relPose([0, 0, opts.heave, 0, 0, 0]);
  const landing = relPose(opts.landing);
  const recordedHome = withZBias(home, opts.zBias);
  const recordedTarget = withZBias(target, opts.zBias);
  const main = [
    { cmd: 'H', wait: 700, note: 'enter HOLD' },
    { cmd: `P ${poseLine(recordedHome)} ${opts.homeMs}`, wait: opts.homeMs + 500, note: `repeatable start from configured home${opts.zBias ? ` + zBias ${opts.zBias}` : ''}` },
    { cmd: `P ${poseLine(recordedTarget)} ${opts.ms}`, wait: opts.ms + opts.settleMs, note: `main heave step to relative +${opts.heave}${opts.zBias ? ` + zBias ${opts.zBias}` : ''}` },
  ];
  const landingPlan = [];
  if (opts.safeLand) {
    landingPlan.push(
      { cmd: `P ${poseLine(home)} ${opts.homeMs}`, wait: opts.homeMs + 500, note: 'return home before release' },
      { cmd: `P ${poseLine(landing)} ${opts.landMs}`, wait: opts.landMs + 600, note: 'support landing pose' },
      { cmd: 'D', wait: 400, note: 'release onto support' },
    );
  }
  return opts.recordMainOnly ? [...main, ...landingPlan] : [...main, ...landingPlan];
}

function splitPlanForRecording(opts, plan) {
  if (!opts.recordMainOnly || !opts.safeLand) return { recorded: plan, after: [] };
  return { recorded: plan.slice(0, 3), after: plan.slice(3) };
}

function printPlan(opts, ctx, plan) {
  console.log(`${opts.live ? '[live]' : '[dry-run]'} ${opts.profile} name=${opts.name}`);
  console.log(`home=${poseLine(ctx.home)} heaveTarget=${poseLine(relPose([0, 0, opts.heave, 0, 0, 0]))}`);
  if (ctx.warnings.length) for (const w of ctx.warnings) console.log(`[warn] ${w}`);
  if (ctx.failures.length) for (const f of ctx.failures) console.log(`[block] ${f}`);
  console.log('Plan:');
  for (const step of plan) console.log(`  -> ${step.cmd.padEnd(48)} # ${step.note}`);
}

function runNode(script, args, opts = {}) {
  return execFileSync(process.execPath, [script, ...args], {
    cwd: path.join(__dirname, '..'),
    encoding: 'utf8',
    stdio: opts.stdio || ['ignore', 'pipe', 'pipe'],
  }).trim();
}

function writeJson(file, value) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, JSON.stringify(value, null, 2));
}

async function createManifest(opts, condition) {
  const out = runNode('sysid/experiment_manifest.js', [
    '--host', opts.host,
    '--name', opts.name,
    '--condition', JSON.stringify(condition),
  ]);
  return out.split(/\r?\n/).filter(Boolean).pop();
}

function summarizeAndPlot(opts, recPath, manifestPath) {
  const summaryText = runNode('sysid/telemetry_summary.js', [recPath]);
  const summary = JSON.parse(summaryText);
  const summaryPath = recPath.replace(/\.jsonl$/, '.summary.json');
  writeJson(summaryPath, summary);

  let plot = null;
  if (opts.plot) {
    plot = JSON.parse(runNode('sysid/plot_motion6.js', [recPath, '--out-dir', path.join(DATA_DIR, 'plots')]));
  }
  const bundle = { recording: recPath, manifest: manifestPath, summary: summaryPath, plot };
  const bundlePath = recPath.replace(/\.jsonl$/, '.bundle.json');
  writeJson(bundlePath, bundle);
  return { summary, bundle, bundlePath };
}

async function liveRun(opts, ctx, plan) {
  const { recorded, after } = splitPlanForRecording(opts, plan);
  const condition = {
    profile: opts.profile,
    heave: opts.heave,
    zBias: opts.zBias,
    ms: opts.ms,
    home: ctx.home,
    landing: opts.safeLand ? opts.landing : null,
    safeLand: opts.safeLand,
    recordMainOnly: opts.recordMainOnly,
  };
  const manifestPath = await createManifest(opts, condition);
  const recName = `${safeName(opts.name)}_${opts.profile}_h${opts.heave}_ms${opts.ms}`;
  const ws = await openWs(opts.host);
  let recPath = null;
  let started = false;
  try {
    const rec = await rest(opts.host, `/api/rec/start?name=${encodeURIComponent(recName)}`);
    recPath = rec.path;
    started = true;
    console.log(`Recording: ${recPath}`);
    for (const step of recorded) {
      console.log(`> ${step.cmd}`);
      ws.send(step.cmd);
      await sleep(step.wait);
    }
    if (opts.recordMainOnly && started) {
      const stopped = await rest(opts.host, '/api/rec/stop');
      recPath = stopped.path || recPath;
      started = false;
      console.log(`Stopped recording before safe landing: ${recPath} (${stopped.lines ?? '?'} lines)`);
    }
    for (const step of after) {
      console.log(`> ${step.cmd}`);
      ws.send(step.cmd);
      await sleep(step.wait);
    }
  } finally {
    if (ws.readyState === WebSocket.OPEN) ws.close();
    if (started) {
      const stopped = await rest(opts.host, '/api/rec/stop');
      recPath = stopped.path || recPath;
      console.log(`Stopped recording: ${recPath} (${stopped.lines ?? '?'} lines)`);
    }
  }
  if (!recPath) throw new Error('No recording path returned');
  const result = summarizeAndPlot(opts, recPath, manifestPath);
  console.log(JSON.stringify({
    recording: recPath,
    manifest: manifestPath,
    summary: result.bundle.summary,
    motorPlot: result.bundle.plot?.motorOut,
    planePlot: result.bundle.plot?.planeOut,
    bundle: result.bundlePath,
    samples: result.summary.samples,
    durationS: result.summary.durationS,
    okFailFrac: result.summary.encoders.failFrac,
    canTxMax: result.summary.canHealth.txMax,
    canEfOr: result.summary.canHealth.efOr,
    hmaxMax: result.summary.hold.hmaxMax,
  }, null, 2));
}

async function main() {
  const opts = parseArgs(process.argv);
  const ctx = await preflight(opts);
  if (opts.profile !== 'heave-step') throw new Error(`Unsupported profile: ${opts.profile}`);
  const plan = buildPlan(opts, ctx.home);
  printPlan(opts, ctx, plan);
  if (!opts.live) {
    console.log('\nDry-run only. Add --live --i-am-at-rig to execute with recording and plots.');
    return;
  }
  if (ctx.failures.length) throw new Error('Preflight blocked live run.');
  await liveRun(opts, ctx, plan);
}

main().catch((err) => {
  console.error(`Error: ${err.message}`);
  process.exit(1);
});
