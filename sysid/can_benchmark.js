#!/usr/bin/env node
// Repeatable CAN/HOLD benchmark.
//
// Default mode is dry-run: prints the planned sequence and exits.
// Add --live only when a human is beside the rig and power cutoff is reachable.

const WebSocket = require('ws');
const { execFileSync } = require('child_process');
const path = require('path');

const DEFAULTS = {
  host: 'localhost:3000',
  loops: [20, 10, 6, 5, 4, 3],
  ar: [20, 10, 5, 3],
  resp: ['0 0'],
  seconds: 20,
  settleMs: 1500,
  enable: false,
  live: false,
  name: 'canbench',
};

function usage() {
  console.log(`Usage:
  node sysid/can_benchmark.js [options]

Dry-run is the default. Add --live to send commands.

Options:
  --host HOST          Server host (default ${DEFAULTS.host})
  --loops LIST         Loop periods in ms, comma-separated (default ${DEFAULTS.loops.join(',')})
  --ar LIST            Auto-return periods in ms, comma-separated (default ${DEFAULTS.ar.join(',')})
  --resp LIST          Response modes, comma-separated pairs: "0 0,1 0"
  --seconds N          Record seconds per condition (default ${DEFAULTS.seconds})
  --enable             Send E before each condition and D after it
  --live               Actually send commands and record data
  --name NAME          Recording name prefix (default ${DEFAULTS.name})
`);
}

function listNums(s) {
  return String(s).split(',').map((x) => Number(x.trim())).filter((x) => Number.isFinite(x) && x > 0);
}

function parseArgs(argv) {
  const o = { ...DEFAULTS, loops: [...DEFAULTS.loops], ar: [...DEFAULTS.ar], resp: [...DEFAULTS.resp] };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--host') o.host = next();
    else if (a === '--loops') o.loops = listNums(next());
    else if (a === '--ar') o.ar = listNums(next());
    else if (a === '--resp') o.resp = String(next()).split(',').map((x) => x.trim()).filter(Boolean);
    else if (a === '--seconds') o.seconds = Number(next());
    else if (a === '--enable') o.enable = true;
    else if (a === '--live') o.live = true;
    else if (a === '--name') o.name = next();
    else throw new Error(`Unknown option: ${a}`);
  }
  return o;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function rest(host, path) {
  const r = await fetch(`http://${host}/api/${path}`);
  if (!r.ok) throw new Error(`HTTP ${r.status} for /api/${path}`);
  return r.json();
}

function manifestPathForRecording(recPath, label) {
  if (!recPath) return null;
  const dir = path.dirname(recPath);
  const base = path.basename(recPath, '.jsonl');
  return path.join(dir, `${base}_${label}.manifest.json`);
}

function writeManifest(opts, label, condition, recPath, latest) {
  const out = manifestPathForRecording(recPath, 'start');
  if (!out) return null;
  let git = {};
  try {
    git = {
      branch: execFileSync('git', ['rev-parse', '--abbrev-ref', 'HEAD'], { encoding: 'utf8' }).trim(),
      commit: execFileSync('git', ['rev-parse', 'HEAD'], { encoding: 'utf8' }).trim(),
      statusShort: execFileSync('git', ['status', '--short'], { encoding: 'utf8' }).trim(),
    };
  } catch {}
  const manifest = {
    manifestVersion: 1,
    name: `${opts.name}_${label}`,
    wallClock: new Date().toISOString(),
    host: opts.host,
    condition,
    git,
    latestAtStart: latest,
    recording: { path: recPath },
  };
  require('fs').writeFileSync(out, JSON.stringify(manifest, null, 2));
  return out;
}

async function openWs(host) {
  const ws = new WebSocket(`ws://${host}`);
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  return ws;
}

function send(ws, cmd) {
  ws.send(cmd);
  console.log(`  -> ${cmd}`);
}

function summarize(rows) {
  const tele = rows.filter((d) => d && Array.isArray(d.a));
  if (!tele.length) return null;
  const bus = tele.map((d) => d.bus || {});
  const sum = (arr) => arr.reduce((a, b) => a + (Number(b) || 0), 0);
  const avg = (arr) => sum(arr) / arr.length;
  const flatPer = bus.flatMap((b) => Array.isArray(b.per) ? b.per : []);
  return {
    samples: tele.length,
    efMax: Math.max(...tele.map((d) => Number(d.ef) || 0)),
    txMax: Math.max(...tele.map((d) => Number(d.tx) || 0)),
    rxMax: Math.max(...tele.map((d) => Number(d.rx) || 0)),
    ovrSum: sum(bus.map((b) => b.ovr)),
    busRxAvg: avg(bus.map((b) => b.rx)),
    lhzAvg: avg(tele.map((d) => d.lhz)),
    f5usMax: Math.max(...bus.map((b) => Number(b.f5us) || 0)),
    perMin: flatPer.length ? Math.min(...flatPer) : null,
    perAvg: flatPer.length ? avg(flatPer) : null,
    hmaxMax: Math.max(...tele.map((d) => Number(d.hmax) || 0)),
  };
}

function conditionName(resp, loop, ar) {
  return `C${resp.replace(/\s+/g, '')}_L${loop}_AR${ar}`;
}

async function runCondition(opts, ws, resp, loop, ar) {
  const label = conditionName(resp, loop, ar);
  console.log(`\n=== ${label} ===`);
  send(ws, `C ${resp}`); await sleep(300);
  send(ws, `L ${loop}`); await sleep(300);
  send(ws, `AR ${ar}`); await sleep(300);
  send(ws, 'A1'); await sleep(opts.settleMs);

  if (opts.enable) {
    send(ws, 'E');
    await sleep(opts.settleMs);
  }

  const rec = await rest(opts.host, `rec/start?name=${opts.name}_${label}`);
  const latestAtStart = await rest(opts.host, 'latest').catch(() => null);
  const manifestPath = writeManifest(opts, label, { resp, loop, ar, seconds: opts.seconds, enable: opts.enable }, rec.path, latestAtStart);
  const rows = [];
  const onMessage = (m) => {
    try {
      const d = JSON.parse(String(m));
      if (d && Array.isArray(d.a)) rows.push(d);
    } catch {}
  };
  ws.on('message', onMessage);
  await sleep(opts.seconds * 1000);
  ws.off('message', onMessage);
  const stopped = await rest(opts.host, 'rec/stop');

  if (opts.enable) {
    send(ws, 'D');
    await sleep(700);
  }

  const s = summarize(rows) || {};
  console.log(`  file: ${stopped.path || rec.path}`);
  if (manifestPath) console.log(`  manifest: ${manifestPath}`);
  console.log(`  summary: samples=${s.samples || 0} efMax=${s.efMax} txMax=${s.txMax} ovrSum=${s.ovrSum} busRxAvg=${Number(s.busRxAvg || 0).toFixed(1)} lhzAvg=${Number(s.lhzAvg || 0).toFixed(1)} perMin=${s.perMin} hmaxMax=${s.hmaxMax}`);
  return { label, file: stopped.path || rec.path, ...s };
}

async function main() {
  const opts = parseArgs(process.argv);
  const conditions = [];
  for (const resp of opts.resp) {
    for (const loop of opts.loops) {
      for (const ar of opts.ar) conditions.push({ resp, loop, ar });
    }
  }

  console.log(`CAN benchmark plan (${conditions.length} conditions, ${opts.seconds}s each):`);
  for (const c of conditions) console.log(`  ${conditionName(c.resp, c.loop, c.ar)}`);
  if (!opts.live) {
    console.log('\nDry-run only. Add --live when a human is beside the rig.');
    return;
  }

  const latest = await rest(opts.host, 'latest');
  if (!latest || latest.ok !== 6) throw new Error(`Need ok:6 before live benchmark; latest ok=${latest && latest.ok}`);
  if (opts.enable && !latest.z) throw new Error('Need calibrated z:1 before --enable benchmark');

  const ws = await openWs(opts.host);
  const results = [];
  try {
    for (const c of conditions) results.push(await runCondition(opts, ws, c.resp, c.loop, c.ar));
  } finally {
    if (opts.enable) send(ws, 'D');
    send(ws, 'L 20');
    ws.close();
  }

  console.log('\n=== Results ===');
  console.table(results.map((r) => ({
    label: r.label,
    samples: r.samples,
    efMax: r.efMax,
    txMax: r.txMax,
    ovrSum: r.ovrSum,
    busRxAvg: Number(r.busRxAvg || 0).toFixed(1),
    lhzAvg: Number(r.lhzAvg || 0).toFixed(1),
    perMin: r.perMin,
    hmaxMax: r.hmaxMax,
  })));
}

main().catch((e) => {
  console.error(e.message || e);
  process.exit(1);
});
