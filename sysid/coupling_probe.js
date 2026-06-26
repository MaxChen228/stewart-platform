#!/usr/bin/env node
// Coupling probe for early MIMO/MPC identification.
// Default is dry-run. Use --live only when a human is next to the platform.

const {
  WebSocket,
  PlatformSoT,
  loadHomePose,
  loadLandingPose,
  openWs,
  parsePose,
  poseLine,
  rest,
  send,
  sleep,
  startRecording,
  stopRecording,
} = require('./rig_client');

const AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];

function usage() {
  console.log(`Usage:
  node sysid/coupling_probe.js [options]

Options:
  --live              Actually send commands and record data. Default: dry-run.
  --host URL          Server base URL (default http://localhost:3000)
  --axis NAME         x|y|z|roll|pitch|yaw|all (default z)
  --amp N             Probe amplitude, mm or deg (default 8 for z, 2 otherwise)
  --base POSE         Base absolute pose CSV (default 0,0,133,0,0,0)
  --hz N              PF stream rate (default 50)
  --duration S        One-way motion duration (default 1.2)
  --settle S          Settle time before/after each segment (default 1.0)
  --vmaxT N           FOLLOW translational limit, mm/s (default 50)
  --vmaxR N           FOLLOW rotational limit, deg/s (default 35)
  --name NAME         Recording label prefix (default coupling_probe)
  --safe-land         After the probe, go home, move to relative heave +10, then D.
                      Enabled by default in --live mode.
  --no-safe-land      End at base pose and keep hold instead.
  --landing POSE      Relative support landing pose (default 0,0,10,0,0,0).

Example:
  node sysid/coupling_probe.js --axis z
  node sysid/coupling_probe.js --live --axis z --amp 10 --duration 1.5
`);
}

function parseArgs(argv) {
  const o = {
    live: false,
    host: 'http://localhost:3000',
    axis: 'z',
    amp: null,
    base: [0, 0, 133, 0, 0, 0],
    hz: 50,
    duration: 1.2,
    settle: 1.0,
    vmaxT: 50,
    vmaxR: 35,
    name: 'coupling_probe',
    safeLand: true,
    landing: [...PlatformSoT.DEFAULT_PLATFORM_CONFIG.landingRelative],
    landingWasSet: false,
  };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--live') o.live = true;
    else if (a === '--host') o.host = next();
    else if (a === '--axis') o.axis = next();
    else if (a === '--amp') o.amp = Number(next());
    else if (a === '--base') o.base = parsePose(next());
    else if (a === '--hz') o.hz = Number(next());
    else if (a === '--duration') o.duration = Number(next());
    else if (a === '--settle') o.settle = Number(next());
    else if (a === '--vmaxT') o.vmaxT = Number(next());
    else if (a === '--vmaxR') o.vmaxR = Number(next());
    else if (a === '--name') o.name = next();
    else if (a === '--safe-land') o.safeLand = true;
    else if (a === '--no-safe-land') o.safeLand = false;
    else if (a === '--landing') { o.landing = parsePose(next()); o.landingWasSet = true; }
    else throw new Error(`Unknown option: ${a}`);
  }
  if (o.axis !== 'all' && !AXES.includes(o.axis)) throw new Error(`Bad axis: ${o.axis}`);
  if (!Number.isFinite(o.amp)) o.amp = o.axis === 'z' ? 8 : 2;
  return o;
}

function smootherstep(u) {
  const x = Math.max(0, Math.min(1, u));
  return x * x * x * (x * (x * 6 - 15) + 10);
}

function buildAxisCommands(opts, axis) {
  const idx = AXES.indexOf(axis);
  const cmds = [];
  const samples = Math.max(2, Math.round(opts.duration * opts.hz));
  const dt = 1 / opts.hz;

  const segment = (from, to) => {
    for (let k = 0; k <= samples; k++) {
      const s = smootherstep(k / samples);
      const p = [...opts.base];
      p[idx] += from + (to - from) * s;
      cmds.push({ dt, cmd: `PF ${poseLine(p)}` });
    }
  };

  segment(0, opts.amp);
  cmds.push({ dt: opts.settle, cmd: null });
  segment(opts.amp, 0);
  cmds.push({ dt: opts.settle, cmd: null });
  segment(0, -opts.amp);
  cmds.push({ dt: opts.settle, cmd: null });
  segment(-opts.amp, 0);
  return cmds;
}

async function dryRun(opts) {
  const axes = opts.axis === 'all' ? AXES : [opts.axis];
  console.log('[dry-run] No commands will be sent.');
  console.log(`Base pose: ${opts.base.join(', ')}`);
  console.log(`Axes: ${axes.join(', ')}, amp=${opts.amp}, hz=${opts.hz}, duration=${opts.duration}s`);
  console.log('Setup commands:');
  console.log(`  H`);
  console.log(`  P ${poseLine(opts.base)} 1500`);
  console.log(`  VF ${opts.vmaxT} ${opts.vmaxR}`);
  console.log(`  FOLLOW 1`);
  for (const axis of axes) {
    const cmds = buildAxisCommands(opts, axis).filter((x) => x.cmd);
    console.log(`Axis ${axis}: ${cmds.length} PF samples, about ${(cmds.length / opts.hz).toFixed(1)}s streaming plus settles`);
    console.log(`  first: ${cmds[0].cmd}`);
    console.log(`  last:  ${cmds[cmds.length - 1].cmd}`);
  }
  if (opts.safeLand) console.log(`End commands: FOLLOW 0, P configured home, P landing ${opts.landing.join(',')}, D`);
  else console.log('End commands: FOLLOW 0, P base 1200');
}

async function safeLand(ws, opts) {
  const home = await loadHomePose(opts.host);
  const landing = await loadLandingPose(opts.host, opts.landing, opts.landingWasSet ? opts.landing : null);
  send(ws, 'FOLLOW 0');
  await sleep(300);
  send(ws, `P ${poseLine(home)} 1500`);
  await sleep(1900);
  send(ws, `P ${poseLine(landing)} 1500`);
  await sleep(2000);
  send(ws, 'D');
  await sleep(300);
}

async function liveRun(opts) {
  const latest = await rest(opts.host, '/api/latest');
  if (latest.ok !== 6) throw new Error(`Need ok=6 before live probe, got ok=${latest.ok}`);
  if (latest.can && latest.can.backend === 'mcp2515' && latest.can.bitrate === 500000) {
    console.log('[warn] Running on MCP2515/500K. Keep amplitudes conservative.');
  }

  const axes = opts.axis === 'all' ? AXES : [opts.axis];
  const ws = await openWs(opts.host);
  const recName = `${opts.name}_${axes.join('')}_amp${opts.amp}`;
  const rec = await startRecording(opts.host, recName);
  console.log(`Recording: ${rec.path}`);
  let landed = false;

  try {
    send(ws, 'H');
    await sleep(800);
    send(ws, `P ${poseLine(opts.base)} 1500`);
    await sleep(2200);
    send(ws, `VF ${opts.vmaxT} ${opts.vmaxR}`);
    await sleep(200);
    send(ws, 'FOLLOW 1');
    await sleep(800);

    for (const axis of axes) {
      console.log(`--- axis ${axis} ---`);
      for (const item of buildAxisCommands(opts, axis)) {
        if (item.cmd) send(ws, item.cmd);
        await sleep(Math.round(item.dt * 1000));
      }
      await sleep(Math.round(opts.settle * 1000));
    }

    if (opts.safeLand) {
      await safeLand(ws, opts);
      landed = true;
    } else {
      send(ws, 'FOLLOW 0');
      await sleep(300);
      send(ws, `P ${poseLine(opts.base)} 1200`);
      await sleep(1600);
    }
  } finally {
    if (opts.safeLand && !landed && ws.readyState === WebSocket.OPEN) {
      try { await safeLand(ws, opts); } catch (err) { console.error(`[warn] safe landing failed: ${err.message}`); }
    }
    ws.close();
    const stopped = await stopRecording(opts.host);
    console.log(`Stopped recording: ${stopped.path} (${stopped.lines} lines)`);
  }
}

async function main() {
  const opts = parseArgs(process.argv);
  if (!opts.live) return dryRun(opts);
  return liveRun(opts);
}

main().catch((err) => {
  console.error(`Error: ${err.message}`);
  process.exit(1);
});
