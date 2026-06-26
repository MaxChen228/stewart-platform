#!/usr/bin/env node
// Return the rig to a conservative known baseline.
//
// Dry-run by default. Add --live to send commands.
// Commands:
//   D      disable position/motors
//   A0     stop encoder auto-return, return to polling
//   C 1 0  restore immediate command response mode
//   L 20   50Hz conservative control loop

const WebSocket = require('ws');

const DEFAULT_HOST = 'localhost:3000';
const COMMANDS = ['D', 'A0', 'C 1 0', 'L 20'];

function usage() {
  console.log(`Usage:
  node sysid/safe_baseline.js [--host localhost:3000] [--live]

Dry-run is default. Add --live to send:
  ${COMMANDS.join(' -> ')}
`);
}

function parseArgs(argv) {
  const o = { host: DEFAULT_HOST, live: false };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--host') o.host = next();
    else if (a === '--live') o.live = true;
    else throw new Error(`Unknown option: ${a}`);
  }
  return o;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function openWs(host) {
  const ws = new WebSocket(`ws://${host}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  return ws;
}

async function main() {
  const opts = parseArgs(process.argv);
  console.log('Safe baseline plan:');
  for (const c of COMMANDS) console.log(`  -> ${c}`);
  if (!opts.live) {
    console.log('\nDry-run only. Add --live to send commands.');
    return;
  }

  const ws = await openWs(opts.host);
  try {
    for (const c of COMMANDS) {
      ws.send(c);
      console.log(`sent: ${c}`);
      await sleep(400);
    }
  } finally {
    ws.close();
  }
  console.log('Safe baseline commands sent.');
}

main().catch((e) => {
  console.error(e.message || e);
  process.exit(1);
});
