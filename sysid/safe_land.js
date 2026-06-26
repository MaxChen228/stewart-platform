#!/usr/bin/env node
// Move to the user-provided support landing pose, then release motors.
//
// Dry-run by default. Add --live only when the support fixture is in place and
// a human is beside the platform.

const PlatformSoT = require('./platform_sot');
const {
  loadHomePose,
  loadLandingPose,
  openWs,
  parsePose,
  poseLine,
  send,
  sleep,
} = require('./rig_client');

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
  H
  P <server home pose> <home-ms>
  P <landing pose> <land-ms>
  D
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

async function buildPlan(opts) {
  const home = await loadHomePose(opts.host, opts.home);
  const landing = await loadLandingPose(opts.host, opts.landing, opts.landingWasSet ? opts.landing : null);
  return [
    { cmd: 'H', wait: 600, note: 'hold current pose' },
    { cmd: `P ${poseLine(home)} ${opts.homeMs}`, wait: opts.homeMs + 400, note: 'return to configured home' },
    { cmd: `P ${poseLine(landing)} ${opts.landMs}`, wait: opts.landMs + 500, note: 'move to support landing pose' },
    { cmd: 'D', wait: 300, note: 'release onto support' },
  ];
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
      send(ws, step.cmd);
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
