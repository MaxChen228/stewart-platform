#!/usr/bin/env node
// One-shot machine health check for experiments.

const DEFAULT_HOST = 'localhost:3000';

function usage() {
  console.log(`Usage:
  node sysid/health_check.js [--host localhost:3000] [--json]
`);
}

function parseArgs(argv) {
  const o = { host: DEFAULT_HOST, json: false };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--host') o.host = next();
    else if (a === '--json') o.json = true;
    else throw new Error(`Unknown option: ${a}`);
  }
  return o;
}

async function getJson(host, path) {
  const r = await fetch(`http://${host}/api/${path}`);
  if (!r.ok) throw new Error(`HTTP ${r.status} for /api/${path}`);
  return r.json();
}

function severityRank(s) {
  return { ok: 0, warn: 1, fail: 2 }[s] ?? 1;
}

function add(checks, status, name, detail) {
  checks.push({ status, name, detail });
}

function analyze(latest, transport, rec) {
  const checks = [];
  const hasTele = latest && Array.isArray(latest.a);
  add(checks, hasTele ? 'ok' : 'fail', 'telemetry', hasTele ? `t=${latest.t}` : 'no telemetry from /api/latest');
  add(checks, transport && transport.state === 'connected' ? 'ok' : 'fail', 'transport', transport ? `${transport.kind || '?'} ${transport.state || '?'}` : 'unavailable');
  add(checks, rec && !rec.recording ? 'ok' : 'warn', 'recording', rec && rec.recording ? `already recording: ${rec.path}` : 'idle');

  if (hasTele) {
    const can = latest.can || {};
    const ef = Number.isFinite(Number(can.ef)) ? Number(can.ef) : Number(latest.ef);
    const tx = Number.isFinite(Number(can.tx)) ? Number(can.tx) : Number(latest.tx);
    const rx = Number.isFinite(Number(can.rx)) ? Number(can.rx) : Number(latest.rx);
    const rxDrop = Number(can.rxDrop || 0);
    add(checks, latest.ok === 6 ? 'ok' : 'fail', 'encoders', `ok=${latest.ok}/6`);
    add(checks, latest.z ? 'ok' : 'warn', 'calibration', latest.z ? 'z=1' : 'z=0');
    add(checks, latest.pos ? 'warn' : 'ok', 'motion_enabled', latest.pos ? 'pos=1 (motors may be holding)' : 'pos=0');
    add(checks, can.backend ? 'ok' : 'warn', 'can_backend', can.backend ? `${can.backend} ${can.bitrate || ''}` : 'legacy telemetry');
    add(checks, ef === 0 ? 'ok' : 'warn', 'can_eflg', `ef=${ef}`);
    add(checks, tx < 128 ? 'ok' : 'fail', 'can_tec', `tx=${tx}`);
    add(checks, rx < 128 ? 'ok' : 'fail', 'can_rec', `rx=${rx}`);
    add(checks, rxDrop === 0 ? 'ok' : 'warn', 'can_rx_drop', `rxDrop=${rxDrop}`);
    const ovr = latest.bus && Number(latest.bus.ovr);
    add(checks, !ovr ? 'ok' : 'warn', 'rx_overflow', `ovr=${ovr || 0}`);
    add(checks, latest.lhz > 0 ? 'ok' : 'warn', 'loop_rate', `lhz=${latest.lhz}${latest.tim ? ` dtMax=${latest.tim.dtMax}` : ''}`);
    if (latest.ctl) {
      const ctlWarn = latest.ctl.ikFail || latest.ctl.fkFail || latest.ctl.coordLimit;
      add(checks, ctlWarn ? 'warn' : 'ok', 'controller', `${latest.ctl.mode} ik=${latest.ctl.ikFail} fk=${latest.ctl.fkFail} limit=${latest.ctl.coordLimit}`);
    }
    if (latest.pid) add(checks, 'ok', 'pid', `[${latest.pid.join(',')}]`);
  }

  const overall = checks.reduce((worst, c) => severityRank(c.status) > severityRank(worst) ? c.status : worst, 'ok');
  return { overall, latest, transport, recording: rec, checks };
}

function printReport(r) {
  console.log(`Health: ${r.overall.toUpperCase()}`);
  for (const c of r.checks) {
    const mark = c.status === 'ok' ? 'OK ' : c.status === 'warn' ? 'WARN' : 'FAIL';
    console.log(`  ${mark}  ${c.name}: ${c.detail}`);
  }
}

async function main() {
  const opts = parseArgs(process.argv);
  const [latest, transport, rec] = await Promise.all([
    getJson(opts.host, 'latest').catch((e) => ({ _error: e.message })),
    getJson(opts.host, 'transport').catch((e) => ({ _error: e.message })),
    getJson(opts.host, 'rec/status').catch((e) => ({ _error: e.message })),
  ]);
  const report = analyze(latest, transport, rec);
  if (opts.json) console.log(JSON.stringify(report, null, 2));
  else printReport(report);
  process.exit(report.overall === 'fail' ? 2 : 0);
}

main().catch((e) => {
  console.error(e.message || e);
  process.exit(2);
});
