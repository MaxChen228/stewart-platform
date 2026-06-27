#!/usr/bin/env node
'use strict';

// Workspace session runner for Codex/CLI.
// Dry-run validates the shared Workspace plan. Live execution is owned by server.js.

const PlatformSoT = require('./platform_sot');
const WorkspaceExecutor = require('./workspace_executor');

function parseArgs(argv) {
  const out = {
    base: 'http://localhost:3000',
    live: false,
    atRig: false,
    name: '',
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
    else if (a === '--name') out.name = next();
    else if (a === '--warmup-ms') out.warmupMs = Math.max(0, Number(next()) || 0);
    else if (a === '--home-ms') out.homeMs = Math.max(300, Number(next()) || 0);
    else if (a === '--follow-settle-ms') out.followSettleMs = Math.max(0, Number(next()) || 0);
    else if (a === '--land-ms') out.landMs = Math.max(300, Number(next()) || 0);
    else if (a === '--close-ms') out.closeMs = Math.max(300, Number(next()) || 0);
    else if (a === '--release-observe-ms') out.releaseObserveMs = Math.max(0, Number(next()) || 0);
    else if (a === '--z-bias') out.zBias = Number(next()) || 0;
    else if (a === '--vmaxT') out.vmaxT = Math.max(1, Number(next()) || 0);
    else if (a === '--vmaxR') out.vmaxR = Math.max(1, Number(next()) || 0);
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
  npm run workspace:session -- --live --i-am-at-rig --name NAME [--warmup-ms MS] [--home-ms MS] [--follow-settle-ms MS] [--land-ms MS] [--z-bias MM] [--vmaxT MM_S] [--vmaxR DEG_S]

Dry-run validates the saved Workspace flow and closed-loop contract.
Live runs are executed by the server-side Workspace runner and record the full lifecycle.
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

function finiteOptions(opts) {
  const out = {};
  if (opts.name) out.name = opts.name;
  for (const key of ['warmupMs', 'homeMs', 'followSettleMs', 'landMs', 'closeMs', 'releaseObserveMs', 'zBias', 'vmaxT', 'vmaxR']) {
    if (Number.isFinite(opts[key])) out[key] = opts[key];
  }
  return out;
}

async function preflight(base) {
  const [latest, transport, rec] = await Promise.all([
    api(base, '/api/latest').catch(() => ({})),
    api(base, '/api/transport').catch(() => ({})),
    api(base, '/api/rec/status').catch(() => ({})),
  ]);
  if (transport.state !== 'connected') throw new Error(`transport not connected (${transport.state || 'unknown'})`);
  if (rec.recording) throw new Error('recorder already running');
  if (!Array.isArray(latest.a) || latest.a.length !== 6) throw new Error('no fresh telemetry');
  if (Number(latest.ok) !== 6) throw new Error(`encoder ok=${latest.ok ?? 'missing'}, expected 6`);
  if (Number(latest.ar || 0) > 0) {
    const ages = latest.arAge;
    if (Array.isArray(ages) && ages.length === 6) {
      const stale = ages.map((v, i) => (Number(v) <= 1500 ? null : i + 1)).filter(Boolean);
      if (stale.length) throw new Error(`auto-return old motors M${stale.join(',M')}`);
    }
  }
  const ageMs = Number(latest._server?.telemetryAgeMs ?? transport.telemetryAgeMs);
  if (!Number.isFinite(ageMs) || ageMs > 1500) throw new Error(`telemetry stale (${Number.isFinite(ageMs) ? Math.round(ageMs) : 'unknown'}ms)`);
}

async function waitForWorkspace(base) {
  let lastPhase = '';
  let lastState = null;
  const started = Date.now();
  while (Date.now() - started < 10 * 60 * 1000) {
    const [workspace, session] = await Promise.all([
      api(base, '/api/workspace/status').catch(() => ({ active: false })),
      api(base, '/api/session/status').catch(() => ({})),
    ]);
    lastState = workspace;
    if (session.phase && session.phase !== lastPhase) {
      lastPhase = session.phase;
      console.log(`  phase: ${lastPhase}`);
    }
    if (workspace.error) throw new Error(workspace.error);
    if (!workspace.active) return workspace;
    await sleep(500);
  }
  throw new Error(`workspace run timeout${lastState?.label ? ` (${lastState.label})` : ''}`);
}

(async () => {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.help) {
    usage();
    return;
  }
  if (opts.live && !opts.atRig) throw new Error('--live requires --i-am-at-rig');

  const config = PlatformSoT.mergeConfig(await api(opts.base, '/api/platform-config').catch(() => ({})));
  const runOpts = finiteOptions(opts);
  const plan = await WorkspaceExecutor.buildWorkspacePlan(config, runOpts);
  plan.description.forEach((line) => console.log(line));
  if (plan.program) console.log(`  program: ${plan.program.name} (${plan.program.id}) hash ${plan.program.hash || 'none'}`);
  if (!plan.audit.ok) throw new Error(`closed-loop audit failed:\n- ${plan.audit.issues.join('\n- ')}`);
  if (plan.audit.warnings?.length) console.log(`Closed-loop warning: ${plan.audit.warnings.join('; ')}`);
  console.log('Closed-loop audit: PASS (motion window starts and ends at HOME)');

  if (!opts.live) {
    console.log('Dry-run only. Add --live --i-am-at-rig to execute.');
    return;
  }

  await preflight(opts.base);
  const started = await api(opts.base, '/api/workspace/run', runOpts);
  console.log(`Workspace session started: ${started.state?.label || runOpts.name || 'workspace'}`);
  const finalState = await waitForWorkspace(opts.base);
  const runId = finalState.result?.runId || null;
  console.log(`Workspace session complete${runId ? `: ${runId}` : ''}`);
})().catch((err) => {
  console.error(`[workspace:session] ${err.message}`);
  process.exit(1);
});
