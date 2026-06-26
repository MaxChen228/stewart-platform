#!/usr/bin/env node
// Offline pose motion-profile generator.
// Produces PF/P command samples for smooth testing and future firmware parity.

const AXES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];

function usage() {
  console.log(`Usage:
  node sysid/motion_profile.js --from "0,0,105,0,0,0" --to "0,0,115,0,0,0" [options]

Options:
  --hz N             Sample rate (default 100)
  --duration S       Duration seconds (default auto from max axis travel / velocity)
  --vmaxT N          Translational velocity limit mm/s (default 60)
  --vmaxR N          Rotational velocity limit deg/s (default 45)
  --profile NAME     smoothstep|smootherstep|linear (default smootherstep)
  --format NAME      json|csv|pf (default json)

Examples:
  node sysid/motion_profile.js --to "0,0,115,0,0,0" --format pf
  node sysid/motion_profile.js --from "0,0,105,0,0,0" --to "10,0,110,0,5,0" --hz 200 --profile smootherstep
`);
}

function parsePose(s, fallback) {
  if (!s) return [...fallback];
  const a = String(s).split(',').map((x) => Number(x.trim()));
  if (a.length !== 6 || a.some((x) => !Number.isFinite(x))) throw new Error(`Bad pose: ${s}`);
  return a;
}

function parseArgs(argv) {
  const o = {
    from: [0, 0, 105, 0, 0, 0],
    to: null,
    hz: 100,
    duration: null,
    vmaxT: 60,
    vmaxR: 45,
    profile: 'smootherstep',
    format: 'json',
  };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--from') o.from = parsePose(next(), o.from);
    else if (a === '--to') o.to = parsePose(next(), o.from);
    else if (a === '--hz') o.hz = Number(next());
    else if (a === '--duration') o.duration = Number(next());
    else if (a === '--vmaxT') o.vmaxT = Number(next());
    else if (a === '--vmaxR') o.vmaxR = Number(next());
    else if (a === '--profile') o.profile = next();
    else if (a === '--format') o.format = next();
    else throw new Error(`Unknown option: ${a}`);
  }
  if (!o.to) throw new Error('--to is required');
  return o;
}

function shape(name, u) {
  const x = Math.max(0, Math.min(1, u));
  if (name === 'linear') return x;
  if (name === 'smoothstep') return x * x * (3 - 2 * x);
  if (name === 'smootherstep') return x * x * x * (x * (x * 6 - 15) + 10);
  throw new Error(`Unknown profile: ${name}`);
}

function autoDuration(opts) {
  const d = opts.to.map((v, i) => Math.abs(v - opts.from[i]));
  const tTrans = Math.max(d[0], d[1], d[2]) / Math.max(1e-6, opts.vmaxT);
  const tRot = Math.max(d[3], d[4], d[5]) / Math.max(1e-6, opts.vmaxR);
  return Math.max(0.2, tTrans, tRot);
}

function generate(opts) {
  const duration = opts.duration || autoDuration(opts);
  const n = Math.max(2, Math.round(duration * opts.hz) + 1);
  const samples = [];
  for (let k = 0; k < n; k++) {
    const t = k / opts.hz;
    const u = Math.min(1, t / duration);
    const s = shape(opts.profile, u);
    const pose = opts.from.map((v, i) => v + (opts.to[i] - v) * s);
    samples.push({ k, t, u, s, pose });
  }
  return { meta: { ...opts, duration, samples: n, axes: AXES }, samples };
}

function fmtPose(p) {
  return p.map((x) => Number(x).toFixed(3)).join(' ');
}

function print(out, format) {
  if (format === 'json') {
    console.log(JSON.stringify(out, null, 2));
  } else if (format === 'csv') {
    console.log(['t', ...AXES].join(','));
    for (const s of out.samples) console.log([s.t.toFixed(4), ...s.pose.map((x) => x.toFixed(5))].join(','));
  } else if (format === 'pf') {
    for (const s of out.samples) console.log(`PF ${fmtPose(s.pose)}`);
  } else {
    throw new Error(`Unknown format: ${format}`);
  }
}

function main() {
  const opts = parseArgs(process.argv);
  print(generate(opts), opts.format);
}

try {
  main();
} catch (err) {
  console.error(`Error: ${err.message}`);
  usage();
  process.exit(1);
}
