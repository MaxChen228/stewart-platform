#!/usr/bin/env node
// Print SERVO42D 0x8A CAN bitrate configuration frames.
// This does not send anything; it is a migration planning/check tool.

const SPEEDS = {
  '125k': 0x00,
  '250k': 0x01,
  '500k': 0x02,
  '1m': 0x03,
};

function usage() {
  console.log(`Usage:
  node sysid/can_bitrate_frames.js --speed 1m [--ids 1,2,3,4,5,6]

SERVO42D 0x8A bitRate values:
  125k -> 0x00
  250k -> 0x01
  500k -> 0x02
  1m   -> 0x03
`);
}

function parseArgs(argv) {
  const o = { speed: null, ids: [1, 2, 3, 4, 5, 6] };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--speed') o.speed = next().toLowerCase();
    else if (a === '--ids') o.ids = next().split(',').map((x) => Number(x.trim())).filter((x) => x >= 1 && x <= 0x7ff);
    else throw new Error(`Unknown option: ${a}`);
  }
  if (!(o.speed in SPEEDS)) throw new Error('Missing or invalid --speed');
  return o;
}

function hex(n, width = 2) {
  return `0x${n.toString(16).toUpperCase().padStart(width, '0')}`;
}

function main() {
  const o = parseArgs(process.argv);
  const bitRate = SPEEDS[o.speed];
  const rows = o.ids.map((id) => {
    const data = [0x8A, bitRate, (id + 0x8A + bitRate) & 0xFF];
    return {
      id: hex(id),
      speed: o.speed,
      data: data.map((b) => hex(b)).join(' '),
      bytes: data,
    };
  });
  console.table(rows.map((r) => ({ CAN_ID: r.id, speed: r.speed, data: r.data })));
}

main();
