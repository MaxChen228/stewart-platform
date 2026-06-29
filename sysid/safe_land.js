#!/usr/bin/env node
// Move to the user-provided support landing pose, then release motors.
//
// Dry-run by default. Add --live only when the support fixture is in place and
// a human is beside the platform.

const WebSocket = require('ws');
const http = require('http');
const { NEUTRAL_Z } = require('./kin');
const PlatformSoT = require('./platform_sot');

const DEFAULT_HOME_REL = PlatformSoT.DEFAULT_PLATFORM_CONFIG.homeRelative;
const DEFAULT_LANDING_REL = PlatformSoT.DEFAULT_PLATFORM_CONFIG.landingRelative;

function usage() {
  console.log(`Usage:
  node sysid/safe_land.js [options]

Options:
  --live              Actually send commands. Default: dry-run.
  --host HOST         WebSocket host (default localhost:3000)
  --home-ms N         Move duration to home, ms (default 1500)
  --land-ms N         Move duration to landing pose, ms (default 1500)
  --home POSE         Fallback relative home if server has none (default 0,0,28,0,0,0)
  --landing POSE      Relative support landing pose (default 0,0,10,0,0,0)

Sequence:
  P <server home pose> <home-ms>   # to home
  P <landing pose> <land-ms>       # to release pose
  D                                # power off (release motors)
`);
}

function parseArgs(argv) {
  const o = {
    live: false,
    host: 'localhost:3000',
    homeMs: 1500,
    landMs: 1500,
    home: [...DEFAULT_HOME_REL],
    landing: [...DEFAULT_LANDING_REL],
    landingWasSet: false,
  };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--live') o.live = true;
    else if (a === '--host') o.host = next();
    else if (a === '--home-ms') o.homeMs = Number(next());
    else if (a === '--land-ms') o.landMs = Number(next());
    else if (a === '--home') o.home = parsePose(next());
    else if (a === '--landing') { o.landing = parsePose(next()); o.landingWasSet = true; }
    else throw new Error(`Unknown option: ${a}`);
  }
  if (!Number.isFinite(o.homeMs) || !Number.isFinite(o.landMs)) throw new Error('Bad move duration');
  return o;
}

function parsePose(s) {
  const p = String(s).split(',').map((x) => Number(x.trim()));
  if (p.length !== 6 || p.some((x) => !Number.isFinite(x))) throw new Error(`Bad pose: ${s}`);
  return p;
}

function sleep(ms) {
  return new Promise((res) => setTimeout(res, ms));
}

function absolutePose(rel) {
  return [rel[0], rel[1], NEUTRAL_Z + rel[2], rel[3], rel[4], rel[5]];
}

function poseLine(pose) {
  return pose.map((x) => x.toFixed(3)).join(' ');
}

function httpBase(host) {
  return host.startsWith('http://') || host.startsWith('https://') ? host : `http://${host}`;
}

function getJson(base, apiPath) {
  return new Promise((resolve, reject) => {
    http.get(new URL(apiPath, httpBase(base)), (res) => {
      let data = '';
      res.on('data', (c) => { data += c; });
      res.on('end', () => {
        try { resolve(JSON.parse(data || '{}')); }
        catch (e) { reject(e); }
      });
    }).on('error', reject);
  });
}

async function loadHomePose(opts) {
  try {
    const d = await getJson(opts.host, '/api/platform-config');
    if (PlatformSoT.finitePose(d.homePose)) return d.homePose;
    if (PlatformSoT.finitePose(d.homeRelative)) return absolutePose(d.homeRelative);
  } catch {}
  return absolutePose(opts.home);
}

async function loadLandingPose(opts) {
  if (opts.landingWasSet) return absolutePose(opts.landing);
  try {
    const d = await getJson(opts.host, '/api/platform-config');
    if (PlatformSoT.finitePose(d.landingPose)) return d.landingPose;
    if (PlatformSoT.finitePose(d.landingRelative)) return absolutePose(d.landingRelative);
  } catch {}
  return absolutePose(opts.landing);
}

async function buildPlan(opts) {
  const home = await loadHomePose(opts);
  const landing = await loadLandingPose(opts);
  return [
    { cmd: `P ${poseLine(home)} ${opts.homeMs}`, wait: opts.homeMs + 400, note: 'to home' },
    { cmd: `P ${poseLine(landing)} ${opts.landMs}`, wait: opts.landMs + 500, note: 'to release pose' },
    { cmd: 'D', wait: 300, note: 'power off (release motors)' },
  ];
}

async function openWs(host) {
  const ws = new WebSocket(`ws://${host}`);
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('WebSocket timeout')), 3000);
    ws.on('open', () => { clearTimeout(timer); resolve(); });
    ws.on('error', reject);
  });
  return ws;
}

async function main() {
  const opts = parseArgs(process.argv);
  const plan = await buildPlan(opts);
  console.log('Safe landing plan:');
  for (const step of plan) console.log(`  -> ${step.cmd.padEnd(42)} # ${step.note}`);
  if (!opts.live) {
    console.log('\nDry-run only. Add --live after confirming the support fixture is in place.');
    return;
  }

  const ws = await openWs(opts.host);
  try {
    for (const step of plan) {
      console.log(`sent: ${step.cmd}`);
      ws.send(step.cmd);
      await sleep(step.wait);
    }
  } finally {
    ws.close();
  }
  console.log('Safe landing sequence complete.');
}

main().catch((err) => {
  console.error(`Error: ${err.message}`);
  process.exit(1);
});
