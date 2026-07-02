#!/usr/bin/env node
'use strict';

const path = require('path');
const fs = require('fs');
const WebSocket = require('ws');
const { execFileSync } = require('child_process');
const { NEUTRAL_Z } = require('./kin');
const PlatformSoT = require('./platform_sot');

function usage() {
  console.log(`Usage:
  node sysid/z_sweep_trial.js [options]

Options:
  --live --i-am-at-rig   Required for motion.
  --name NAME            Recording prefix.
  --low MM               Relative low heave (default 10).
  --high MM              Relative high heave (default 62).
  --cycles N             Up/down cycles (default 1).
  --ms N                 One-way P duration ms (default 6000).
  --settle-ms N          Settle after each move ms (default 1200).
  --pid a,b,c,d          Temporary motor PID for the run; restored after.
  --z-bias MM            Command-side bias for moving segment only.
  --host HOST            Default localhost:3000.

Sequence:
  H -> configured home -> low -> high -> low ... -> stop recording
  -> configured home -> landing -> D
`);
}

function parseArgs(argv) {
  const o = {
    live: false,
    atRig: false,
    name: 'z_sweep',
    low: 10,
    high: 62,
    cycles: 1,
    ms: 6000,
    settleMs: 1200,
    zBias: 0,
    pid: null,
    host: 'localhost:3000',
  };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--live') o.live = true;
    else if (a === '--i-am-at-rig') o.atRig = true;
    else if (a === '--name') o.name = next();
    else if (a === '--low') o.low = Number(next());
    else if (a === '--high') o.high = Number(next());
    else if (a === '--cycles') o.cycles = Number(next());
    else if (a === '--ms') o.ms = Number(next());
    else if (a === '--settle-ms') o.settleMs = Number(next());
    else if (a === '--z-bias') o.zBias = Number(next());
    else if (a === '--pid') o.pid = String(next()).split(',').map(Number);
    else if (a === '--host') o.host = next();
    else throw new Error(`Unknown option: ${a}`);
  }
  for (const k of ['low', 'high', 'cycles', 'ms', 'settleMs', 'zBias']) {
    if (!Number.isFinite(o[k])) throw new Error(`Bad ${k}`);
  }
  if (o.pid && (o.pid.length !== 4 || o.pid.some((x) => !Number.isFinite(x)))) throw new Error('Bad --pid');
  if (o.live && !o.atRig) throw new Error('Live run requires --i-am-at-rig');
  if (o.low < 9 || o.high > 62 || o.low >= o.high) throw new Error('Expected 9 <= low < high <= 62');
  return o;
}

function httpBase(host) {
  return host.startsWith('http://') ? host : `http://${host}`;
}

function wsUrl(host) {
  const u = new URL(httpBase(host));
  u.protocol = 'ws:';
  u.pathname = '/';
  return u.toString();
}

async function rest(host, api, method = 'GET') {
  const r = await fetch(new URL(api.startsWith('/api/') ? api : `/api/${api}`, httpBase(host)), { method });
  return r.json();
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function nowMs() {
  return Date.now();
}

function safeName(s) {
  return String(s || 'run').replace(/[^a-zA-Z0-9_-]/g, '_');
}

function relPose(zRel, zBias = 0) {
  return [0, 0, NEUTRAL_Z + zRel + zBias, 0, 0, 0];
}

function poseLine(pose) {
  return pose.map((x) => Number(x).toFixed(3)).join(' ');
}

async function openWs(host) {
  const ws = new WebSocket(wsUrl(host));
  ws._acks = [];
  ws._drops = [];
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('WebSocket timeout')), 3000);
    ws.on('open', () => { clearTimeout(timer); resolve(); });
    ws.on('error', reject);
    ws.on('message', (buf) => {
      let d;
      try { d = JSON.parse(buf.toString()); } catch { return; }
      if (d?.evt === 'cmd') ws._acks.push(d);
      else if (d?.evt === 'cmd_dropped') ws._drops.push(d);
    });
  });
  return ws;
}

async function waitForTransport(host, timeoutMs = 20000) {
  const deadline = nowMs() + timeoutMs;
  let last = null;
  while (nowMs() < deadline) {
    last = await rest(host, '/api/transport').catch((err) => ({ error: err.message }));
    if (last.state === 'connected') return last;
    if (last.state === 'down' || last.state === 'connecting') {
      await rest(host, '/api/transport/reconnect', 'POST').catch(() => {});
    }
    await sleep(700);
  }
  throw new Error(`transport not connected after ${timeoutMs}ms: ${JSON.stringify(last)}`);
}

async function waitForFreshTelemetry(host, timeoutMs = 6000) {
  const deadline = nowMs() + timeoutMs;
  let last = null;
  while (nowMs() < deadline) {
    last = await rest(host, '/api/latest').catch((err) => ({ error: err.message }));
    const transport = await rest(host, '/api/transport').catch(() => ({}));
    const ageMs = Number.isFinite(transport.lastLineAt) ? nowMs() - transport.lastLineAt : Infinity;
    if (transport.state === 'connected' && ageMs < 1500 && Array.isArray(last.a) && last.ok === 6 && Number(last.lhz) >= 20) return last;
    await sleep(250);
  }
  throw new Error(`no fresh ok=6 telemetry after ${timeoutMs}ms: ${JSON.stringify(last)}`);
}

async function sendChecked(ws, host, cmd, timeoutMs = 2500) {
  await waitForTransport(host);
  const ackStart = ws._acks.length;
  const dropStart = ws._drops.length;
  ws.send(cmd);
  const deadline = nowMs() + timeoutMs;
  while (nowMs() < deadline) {
    const drop = ws._drops.slice(dropStart).find((x) => x.c === cmd);
    if (drop) throw new Error(`command dropped: ${cmd} (${drop.reason})`);
    const ack = ws._acks.slice(ackStart).find((x) => x.c === cmd);
    if (ack) return ack;
    await sleep(25);
  }
  throw new Error(`command not acknowledged: ${cmd}`);
}

async function platformConfig(host) {
  return PlatformSoT.mergeConfig(await rest(host, '/api/platform-config').catch(() => ({})));
}

function absFromRel(rel) {
  return PlatformSoT.relToAbs(rel, NEUTRAL_Z);
}

function runNode(script, args) {
  return execFileSync(process.execPath, [script, ...args], {
    cwd: path.join(__dirname, '..'),
    encoding: 'utf8',
  }).trim();
}

async function createManifest(opts, condition) {
  const out = runNode('sysid/experiment_manifest.js', [
    '--host', opts.host,
    '--name', opts.name,
    '--condition', JSON.stringify(condition),
  ]);
  return out.split(/\r?\n/).filter(Boolean).pop();
}

function summarize(recPath, manifestPath) {
  const summary = JSON.parse(runNode('sysid/telemetry_summary.js', [recPath]));
  const summaryPath = recPath.replace(/\.jsonl$/, '.summary.json');
  fs.writeFileSync(summaryPath, JSON.stringify(summary, null, 2));
  const plot = JSON.parse(runNode('sysid/plot_motion6.js', [recPath, '--out-dir', path.join(__dirname, 'data', 'plots')]));
  const evaluation = JSON.parse(runNode('sysid/workspace_evaluation.js', [recPath]));
  const evaluationPath = recPath.replace(/\.jsonl$/, '.evaluation.json');
  fs.writeFileSync(evaluationPath, JSON.stringify(evaluation, null, 2));
  const bundle = { recording: recPath, manifest: manifestPath, summary: summaryPath, evaluation: evaluationPath, plot };
  const bundlePath = recPath.replace(/\.jsonl$/, '.bundle.json');
  fs.writeFileSync(bundlePath, JSON.stringify(bundle, null, 2));
  return { summary, evaluation, bundlePath, plot };
}

function buildPlan(opts, cfg) {
  const home = absFromRel(cfg.homeRelative);
  const landing = absFromRel(cfg.landingRelative);
  const setup = [
    { cmd: 'H', wait: 700, note: 'hold current' },
    { cmd: `P ${poseLine(home)} 2000`, wait: 2500, note: 'configured home' },
  ];
  const recorded = [
    { cmd: `P ${poseLine(relPose(opts.low, opts.zBias))} ${opts.ms}`, wait: opts.ms + opts.settleMs, note: 'low reference' },
  ];
  for (let i = 0; i < opts.cycles; i++) {
    recorded.push({ cmd: `P ${poseLine(relPose(opts.high, opts.zBias))} ${opts.ms}`, wait: opts.ms + opts.settleMs, note: `cycle ${i + 1} high` });
    recorded.push({ cmd: `P ${poseLine(relPose(opts.low, opts.zBias))} ${opts.ms}`, wait: opts.ms + opts.settleMs, note: `cycle ${i + 1} low` });
  }
  const after = [
    { cmd: `P ${poseLine(home)} 2000`, wait: 2500, note: 'home before release' },
    { cmd: `P ${poseLine(landing)} 1800`, wait: 2300, note: 'landing' },
    { cmd: 'D', wait: 400, note: 'release' },
  ];
  return { setup, recorded, after };
}

async function main() {
  const opts = parseArgs(process.argv);
  const cfg = await platformConfig(opts.host);
  if (opts.live) await waitForTransport(opts.host);
  const latest = opts.live ? await waitForFreshTelemetry(opts.host) : await rest(opts.host, '/api/latest');
  const rec = await rest(opts.host, '/api/rec/status');
  if (latest.ok !== 6) throw new Error(`Need ok=6, got ${latest.ok}`);
  if (rec.recording) throw new Error(`Recorder already running: ${rec.path}`);
  const plan = buildPlan(opts, cfg);
  console.log(`${opts.live ? '[live]' : '[dry-run]'} ${opts.name}: Z relative ${opts.low}..${opts.high}mm cycles=${opts.cycles} ms=${opts.ms} zBias=${opts.zBias}`);
  if (opts.pid) console.log(`temporary PID=[${opts.pid.join(',')}]`);
  for (const step of [...plan.setup, ...plan.recorded, ...plan.after]) console.log(`  -> ${step.cmd.padEnd(52)} # ${step.note}`);
  if (!opts.live) return;

  const condition = {
    profile: 'z-sweep',
    low: opts.low,
    high: opts.high,
    cycles: opts.cycles,
    ms: opts.ms,
    settleMs: opts.settleMs,
    zBias: opts.zBias,
    pid: opts.pid,
    homeRelative: cfg.homeRelative,
    landingRelative: cfg.landingRelative,
  };
  const manifestPath = await createManifest(opts, condition);
  const ws = await openWs(opts.host);
  const originalPid = latest.pid;
  let recPath = null;
  let started = false;
  try {
    if (opts.pid) {
      await sendChecked(ws, opts.host, `K ${opts.pid.join(' ')}`);
      await sleep(900);
    }
    for (const step of plan.setup) {
      console.log(`> ${step.cmd}`);
      await sendChecked(ws, opts.host, step.cmd);
      await sleep(step.wait);
      await waitForFreshTelemetry(opts.host);
    }
    await waitForFreshTelemetry(opts.host);
    const recName = `${safeName(opts.name)}_z${opts.low}-${opts.high}_c${opts.cycles}_ms${opts.ms}`;
    const startedRec = await rest(opts.host, `/api/rec/start?name=${encodeURIComponent(recName)}`, 'POST');
    recPath = startedRec.path;
    started = true;
    console.log(`Recording: ${recPath}`);
    for (const step of plan.recorded) {
      console.log(`> ${step.cmd}`);
      await sendChecked(ws, opts.host, step.cmd);
      await sleep(step.wait);
      await waitForFreshTelemetry(opts.host);
    }
    const stopped = await rest(opts.host, '/api/rec/stop', 'POST');
    recPath = stopped.path || recPath;
    started = false;
    console.log(`Stopped recording before safe landing: ${recPath} (${stopped.lines ?? '?'} lines)`);
    if ((stopped.lines ?? 0) < 10) throw new Error(`recording too short: ${stopped.lines ?? 0} lines`);
    for (const step of plan.after) {
      console.log(`> ${step.cmd}`);
      await sendChecked(ws, opts.host, step.cmd);
      await sleep(step.wait);
    }
  } finally {
    if (started) await rest(opts.host, '/api/rec/stop', 'POST').catch(() => {});
    if (originalPid && opts.pid && ws.readyState === WebSocket.OPEN) {
      await sendChecked(ws, opts.host, `K ${originalPid.join(' ')}`).catch(() => {});
      await sleep(800);
    }
    if (ws.readyState === WebSocket.OPEN) ws.close();
  }
  const result = summarize(recPath, manifestPath);
  console.log(JSON.stringify({
    recording: recPath,
    evaluation: result.bundlePath.replace(/\.bundle\.json$/, '.evaluation.json'),
    inspection: result.evaluation.quality,
    fullBadness: result.evaluation.fullBadness,
    summary: result.summary.canHealth,
    planePlot: result.plot.planeOut,
    motorPlot: result.plot.motorOut,
    bundle: result.bundlePath,
  }, null, 2));
}

main().catch((err) => {
  console.error(`Error: ${err.message}`);
  process.exit(1);
});
