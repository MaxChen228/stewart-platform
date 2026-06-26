#!/usr/bin/env node
// Create an experiment manifest capturing software state and current machine state.

const fs = require('fs');
const path = require('path');
const { execFileSync } = require('child_process');

const DEFAULT_HOST = 'localhost:3000';
const DATA_DIR = path.join(__dirname, 'data');

function usage() {
  console.log(`Usage:
  node sysid/experiment_manifest.js [options]

Options:
  --host HOST          Server host (default ${DEFAULT_HOST})
  --name NAME          Experiment name (default manifest)
  --out PATH           Output path; default sysid/data/<name>_<stamp>.manifest.json
  --condition JSON     Extra condition object as JSON
  --json               Print manifest JSON to stdout
`);
}

function parseArgs(argv) {
  const o = { host: DEFAULT_HOST, name: 'manifest', out: null, condition: null, json: false };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--host') o.host = next();
    else if (a === '--name') o.name = next();
    else if (a === '--out') o.out = next();
    else if (a === '--condition') o.condition = JSON.parse(next());
    else if (a === '--json') o.json = true;
    else throw new Error(`Unknown option: ${a}`);
  }
  return o;
}

function sh(cmd, args) {
  try { return execFileSync(cmd, args, { cwd: path.join(__dirname, '..'), encoding: 'utf8' }).trim(); }
  catch (e) { return null; }
}

async function getJson(host, api) {
  try {
    const r = await fetch(`http://${host}/api/${api}`);
    if (!r.ok) return { error: `HTTP ${r.status}` };
    return r.json();
  } catch (e) {
    return { error: e.message };
  }
}

function safeName(s) {
  return String(s || 'manifest').replace(/[^a-zA-Z0-9_-]/g, '_');
}

async function buildManifest(opts) {
  const [latest, transport, recording] = await Promise.all([
    getJson(opts.host, 'latest'),
    getJson(opts.host, 'transport'),
    getJson(opts.host, 'rec/status'),
  ]);
  return {
    manifestVersion: 1,
    name: safeName(opts.name),
    wallClock: new Date().toISOString(),
    host: opts.host,
    condition: opts.condition,
    git: {
      branch: sh('git', ['rev-parse', '--abbrev-ref', 'HEAD']),
      commit: sh('git', ['rev-parse', 'HEAD']),
      statusShort: sh('git', ['status', '--short']),
    },
    server: {
      transport,
      recording,
    },
    latest,
    notes: [
      'Manifest records state at experiment start; telemetry JSONL is the source for time-varying values.',
    ],
  };
}

async function main() {
  const opts = parseArgs(process.argv);
  const manifest = await buildManifest(opts);
  const stamp = manifest.wallClock.replace(/[:.]/g, '-').slice(0, 19);
  const out = opts.out || path.join(DATA_DIR, `${safeName(opts.name)}_${stamp}.manifest.json`);
  fs.mkdirSync(path.dirname(out), { recursive: true });
  fs.writeFileSync(out, JSON.stringify(manifest, null, 2));
  if (opts.json) console.log(JSON.stringify(manifest, null, 2));
  else console.log(out);
}

main().catch((e) => {
  console.error(e.message || e);
  process.exit(1);
});
