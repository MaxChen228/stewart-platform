#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');
const WebSocket = require('ws');

const ROOT = path.resolve(__dirname, '..');
const QUEUE_DIR = path.join(__dirname, 'data', 'queue');

function stamp() {
  return new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
}

function slug(value) {
  return String(value || 'workspace_queue')
    .trim()
    .replace(/[^a-zA-Z0-9_-]+/g, '_')
    .replace(/^_+|_+$/g, '')
    .slice(0, 64) || 'workspace_queue';
}

function parseArgs(argv) {
  const out = {
    base: 'http://localhost:3000',
    live: false,
    atRig: false,
    count: 3,
    prefix: 'workspace_queue',
    gapMs: 1500,
    notify: false,
    setupCmds: [],
    stopOnHealth: false,
    warmupMs: null,
    homeMs: null,
    followSettleMs: null,
    landMs: null,
    closeMs: null,
    releaseObserveMs: null,
    zBias: null,
    vmaxT: null,
    vmaxR: null,
  };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) throw new Error(`missing value after ${a}`);
      return argv[++i];
    };
    if (a === '--base') out.base = next().replace(/\/+$/, '');
    else if (a === '--count') out.count = Math.max(1, Math.min(100, Number(next()) || 1));
    else if (a === '--prefix') out.prefix = slug(next());
    else if (a === '--gap-ms') out.gapMs = Math.max(0, Number(next()) || 0);
    else if (a === '--setup-cmd') out.setupCmds.push(next());
    else if (a === '--pure-p') out.setupCmds.push('K 1024 0 0 0');
    else if (a === '--ar') out.setupCmds.push(`AR ${Math.max(1, Number(next()) || 20)}`);
    else if (a === '--warmup-ms') out.warmupMs = Math.max(0, Number(next()) || 0);
    else if (a === '--home-ms') out.homeMs = Math.max(300, Number(next()) || 0);
    else if (a === '--follow-settle-ms') out.followSettleMs = Math.max(0, Number(next()) || 0);
    else if (a === '--land-ms') out.landMs = Math.max(300, Number(next()) || 0);
    else if (a === '--close-ms') out.closeMs = Math.max(300, Number(next()) || 0);
    else if (a === '--release-observe-ms') out.releaseObserveMs = Math.max(0, Number(next()) || 0);
    else if (a === '--z-bias') out.zBias = Number(next()) || 0;
    else if (a === '--vmaxT') out.vmaxT = Math.max(1, Number(next()) || 0);
    else if (a === '--vmaxR') out.vmaxR = Math.max(1, Number(next()) || 0);
    else if (a === '--stop-on-health') out.stopOnHealth = true;
    else if (a === '--notify') out.notify = true;
    else if (a === '--live') out.live = true;
    else if (a === '--i-am-at-rig') out.atRig = true;
    else if (a === '--help' || a === '-h') out.help = true;
    else throw new Error(`unknown arg: ${a}`);
  }
  return out;
}

function usage() {
  console.log(`Usage:
  npm run workspace:queue -- --count 3 --prefix program00_test
  npm run workspace:queue -- --live --i-am-at-rig --count 6 --prefix program00_ar20 --pure-p --ar 20 --notify

Runs saved Workspace program jobs one at a time through the server-side runner.
Each completed run is analyzed by server.js and collected into a queue report.
`);
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

function finiteRunOptions(opts, name) {
  const out = { name };
  for (const key of ['warmupMs', 'homeMs', 'followSettleMs', 'landMs', 'closeMs', 'releaseObserveMs', 'zBias', 'vmaxT', 'vmaxR']) {
    if (Number.isFinite(opts[key])) out[key] = opts[key];
  }
  return out;
}

function setupUrl(base) {
  const u = new URL(base);
  u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
  u.pathname = '/';
  u.search = '';
  return u.toString();
}

async function sendSetupCommands(base, commands) {
  if (!commands.length) return [];
  const sent = [];
  await new Promise((resolve, reject) => {
    const ws = new WebSocket(setupUrl(base));
    const timer = setTimeout(() => {
      ws.close();
      reject(new Error('setup websocket timeout'));
    }, Math.max(5000, commands.length * 1000));
    ws.on('open', async () => {
      try {
        for (const cmd of commands) {
          ws.send(cmd);
          sent.push(cmd);
          await sleep(450);
        }
        clearTimeout(timer);
        ws.close();
        resolve();
      } catch (err) {
        clearTimeout(timer);
        reject(err);
      }
    });
    ws.on('error', (err) => {
      clearTimeout(timer);
      reject(err);
    });
  });
  return sent;
}

async function preflight(base) {
  const [latest, transport, rec, workspace] = await Promise.all([
    api(base, '/api/latest').catch(() => ({})),
    api(base, '/api/transport').catch(() => ({})),
    api(base, '/api/rec/status').catch(() => ({})),
    api(base, '/api/workspace/status').catch(() => ({})),
  ]);
  if (transport.state !== 'connected') throw new Error(`transport not connected (${transport.state || 'unknown'})`);
  if (workspace.active) throw new Error(`workspace already active (${workspace.label || 'unknown'})`);
  if (rec.recording) throw new Error('recorder already running');
  if (!Array.isArray(latest.a) || latest.a.length !== 6) throw new Error('no fresh telemetry');
  if (Number(latest.ok) !== 6) throw new Error(`encoder ok=${latest.ok ?? 'missing'}, expected 6`);
  const ageMs = Number(latest._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
  if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
  if (Array.isArray(latest.arAge) && latest.arAge.some((v) => Number(v) > 1500)) {
    throw new Error(`auto-return old: [${latest.arAge.join(',')}]ms`);
  }
  return { latest, transport };
}

async function waitForWorkspace(base, label) {
  const started = Date.now();
  let lastPhase = '';
  while (Date.now() - started < 10 * 60 * 1000) {
    const [workspace, session] = await Promise.all([
      api(base, '/api/workspace/status').catch(() => ({ active: false })),
      api(base, '/api/session/status').catch(() => ({})),
    ]);
    if (session.phase && session.phase !== lastPhase) {
      lastPhase = session.phase;
      console.log(`  ${label}: ${lastPhase}`);
    }
    if (workspace.error) throw new Error(workspace.error);
    if (!workspace.active) return workspace;
    await sleep(500);
  }
  throw new Error(`workspace run timeout (${label})`);
}

function healthStopReason(detail) {
  const h = detail?.run?.health || detail?.summary?.canHealth || {};
  const summary = detail?.summary || {};
  if (Number(summary.encoders?.failFrac || 0) > 0) return `encoder failFrac=${summary.encoders.failFrac}`;
  if (Number(summary.canHealth?.txFailSum || 0) > 0) return `CAN txFailSum=${summary.canHealth.txFailSum}`;
  if (Number(summary.hostDtMs?.max || 0) > 2000) return `host dt max=${summary.hostDtMs.max}ms`;
  if (Number(h.okFailFrac || 0) > 0) return `okFailFrac=${h.okFailFrac}`;
  return null;
}

async function runOne(base, opts, index, total) {
  const runName = `${opts.prefix}_${String(index + 1).padStart(2, '0')}_${stamp()}`;
  console.log(`[queue] ${index + 1}/${total} ${runName}`);
  await preflight(base);
  const runOpts = finiteRunOptions(opts, runName);
  const started = await api(base, '/api/workspace/run', runOpts);
  if (!started?.state) throw new Error('workspace did not start');
  const finalState = await waitForWorkspace(base, runName);
  const runId = finalState.result?.runId;
  if (!runId) throw new Error('workspace completed without runId');
  const detail = await api(base, `/api/runs/${encodeURIComponent(runId)}`).catch(() => null);
  return {
    index: index + 1,
    name: runName,
    runId,
    state: finalState,
    detail,
    healthStopReason: healthStopReason(detail),
  };
}

function writeReport(opts, report) {
  fs.mkdirSync(QUEUE_DIR, { recursive: true });
  const base = path.join(QUEUE_DIR, `${opts.prefix}_${stamp()}`);
  const jsonPath = `${base}.json`;
  const mdPath = `${base}.md`;
  fs.writeFileSync(jsonPath, JSON.stringify(report, null, 2));
  const lines = [];
  lines.push(`# ${opts.prefix}`);
  lines.push('');
  lines.push(`- started: ${report.startedAt}`);
  lines.push(`- ended: ${report.endedAt}`);
  lines.push(`- status: ${report.status}`);
  lines.push(`- setup: ${report.setupCmds.length ? report.setupCmds.map((x) => `\`${x}\``).join(', ') : 'none'}`);
  lines.push('');
  lines.push('| # | run | verdict | max step | p99 step | cross peak | worst | rxDrop/s | ef | note |');
  lines.push('|---:|---|---:|---:|---:|---:|---|---:|---:|---|');
  for (const item of report.results) {
    const run = item.detail?.run || {};
    const summary = item.detail?.summary || {};
    const evaluation = item.detail?.evaluation || {};
    const health = run.health || {};
    const quality = evaluation.quality || {};
    const full = evaluation.fullBadness || {};
    const rxDrop = health.rxDropPerS ?? '';
    const ef = health.canEfOr ?? summary.canHealth?.efOr ?? '';
    lines.push(`| ${item.index} | \`${item.runId || item.name}\` | ${quality.verdict || run.verdict || ''} | ${full.motorStepMaxDeg ?? ''} | ${full.motorStepP99MeanDeg ?? ''} | ${full.poseCrossHpPeak ?? ''} | ${full.worstMotorStep?.label ?? ''} | ${rxDrop} | ${ef} | ${item.error || item.healthStopReason || ''} |`);
  }
  fs.writeFileSync(mdPath, `${lines.join('\n')}\n`);
  return { jsonPath, mdPath };
}

function notify(title, message) {
  process.stdout.write('\u0007');
  spawnSync('osascript', ['-e', `display notification ${JSON.stringify(message)} with title ${JSON.stringify(title)}`], {
    stdio: 'ignore',
  });
}

(async () => {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.help) {
    usage();
    return;
  }
  if (opts.live && !opts.atRig) throw new Error('--live requires --i-am-at-rig');

  const report = {
    kind: 'workspace-queue',
    startedAt: new Date().toISOString(),
    endedAt: null,
    status: 'running',
    base: opts.base,
    live: opts.live,
    count: opts.count,
    setupCmds: opts.setupCmds,
    runOptions: finiteRunOptions(opts, null),
    results: [],
  };

  if (!opts.live) {
    await preflight(opts.base);
    report.status = 'dry-run';
    report.endedAt = new Date().toISOString();
    const paths = writeReport(opts, report);
    console.log(`[queue] dry-run ok: ${paths.mdPath}`);
    return;
  }

  try {
    await sendSetupCommands(opts.base, opts.setupCmds);
    await sleep(800);
    for (let i = 0; i < opts.count; i++) {
      const item = await runOne(opts.base, opts, i, opts.count);
      report.results.push(item);
      if (opts.stopOnHealth && item.healthStopReason) {
        report.status = 'stopped-health';
        break;
      }
      if (i + 1 < opts.count) await sleep(opts.gapMs);
    }
    if (report.status === 'running') report.status = 'complete';
  } catch (err) {
    report.status = 'error';
    report.error = err.message;
    report.results.push({ index: report.results.length + 1, error: err.message });
    throw err;
  } finally {
    report.endedAt = new Date().toISOString();
    const paths = writeReport(opts, report);
    console.log(`[queue] report: ${paths.mdPath}`);
    if (opts.notify) notify('Workspace queue done', `${report.status}: ${report.results.length}/${opts.count} jobs`);
  }
})().catch((err) => {
  console.error(`[workspace:queue] ${err.message}`);
  process.exit(1);
});
