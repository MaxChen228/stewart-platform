#!/usr/bin/env node
// CAN traffic budget for the Stewart platform.
// Estimates classical 11-bit CAN load for F5 position commands, encoder
// auto-return frames, and optional command ACKs.

const DEFAULTS = {
  motors: 6,
  controlHz: 100,
  encoderHz: 100,
  bitrate: 500000,
  ack: false,
  f5Dlc: 8,
  encoderDlc: 8,
  ackDlc: 2,
  stuff: 1.18,
};

function usage() {
  console.log(`Usage:
  node sysid/can_budget.js [options]

Options:
  --motors N          Motor count (default ${DEFAULTS.motors})
  --control HZ        F5 control frequency per motor (default ${DEFAULTS.controlHz})
  --encoder HZ        Encoder auto-return frequency per motor (default ${DEFAULTS.encoderHz})
  --bitrate BPS       CAN bitrate, e.g. 500000 or 1000000 (default ${DEFAULTS.bitrate})
  --ack 0|1           Include one ACK frame per F5 command (default 0)
  --stuff FACTOR      Bit-stuffing/safety multiplier (default ${DEFAULTS.stuff})
  --sweep             Print common 50-300Hz scenarios for 500K and 1M
  --json              Emit JSON
`);
}

function parseArgs(argv) {
  const o = { ...DEFAULTS, json: false, sweep: false };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--help' || a === '-h') { usage(); process.exit(0); }
    else if (a === '--motors') o.motors = Number(next());
    else if (a === '--control') o.controlHz = Number(next());
    else if (a === '--encoder') o.encoderHz = Number(next());
    else if (a === '--bitrate') o.bitrate = Number(next());
    else if (a === '--ack') o.ack = next() !== '0';
    else if (a === '--stuff') o.stuff = Number(next());
    else if (a === '--json') o.json = true;
    else if (a === '--sweep') o.sweep = true;
    else throw new Error(`Unknown option: ${a}`);
  }
  return o;
}

function frameBits(dlc, stuffFactor = DEFAULTS.stuff) {
  // Standard 11-bit data frame, including intermission:
  // SOF 1 + arbitration 12 + control 6 + data + CRC 16 + ACK 2 + EOF 7 + IFS 3.
  // Bit stuffing applies from SOF through CRC sequence; using a multiplier is a
  // useful engineering estimate and intentionally conservative.
  const rawBits = 1 + 12 + 6 + (Math.max(0, Math.min(8, dlc)) * 8) + 16 + 2 + 7 + 3;
  return rawBits * stuffFactor;
}

function classify(loadPct) {
  if (loadPct < 45) return 'comfortable';
  if (loadPct < 65) return 'usable';
  if (loadPct < 80) return 'tight';
  return 'redline';
}

function calc(opts) {
  const f5Fps = opts.motors * opts.controlHz;
  const encFps = opts.motors * opts.encoderHz;
  const ackFps = opts.ack ? opts.motors * opts.controlHz : 0;
  const f5Bits = frameBits(opts.f5Dlc, opts.stuff);
  const encBits = frameBits(opts.encoderDlc, opts.stuff);
  const ackBits = frameBits(opts.ackDlc, opts.stuff);
  const bitsPerSec = f5Fps * f5Bits + encFps * encBits + ackFps * ackBits;
  const loadPct = (bitsPerSec / opts.bitrate) * 100;
  return {
    motors: opts.motors,
    controlHz: opts.controlHz,
    encoderHz: opts.encoderHz,
    bitrate: opts.bitrate,
    ack: opts.ack,
    stuff: opts.stuff,
    framesPerSec: {
      f5: f5Fps,
      encoder: encFps,
      ack: ackFps,
      total: f5Fps + encFps + ackFps,
    },
    frameBits: {
      f5: f5Bits,
      encoder: encBits,
      ack: ackBits,
    },
    bitsPerSec,
    loadPct,
    class: classify(loadPct),
    frameTimeUs: {
      f5: (f5Bits / opts.bitrate) * 1e6,
      encoder: (encBits / opts.bitrate) * 1e6,
      ack: (ackBits / opts.bitrate) * 1e6,
    },
  };
}

function fmt(n, d = 1) {
  return Number.isFinite(n) ? n.toFixed(d) : 'NaN';
}

function printOne(r) {
  console.log(`CAN budget: ${r.motors} motors, F5 ${r.controlHz}Hz, encoder ${r.encoderHz}Hz, bitrate ${(r.bitrate / 1000).toFixed(0)}K, ACK ${r.ack ? 'on' : 'off'}`);
  console.log(`  frames/s: F5=${r.framesPerSec.f5}, encoder=${r.framesPerSec.encoder}, ack=${r.framesPerSec.ack}, total=${r.framesPerSec.total}`);
  console.log(`  load: ${fmt(r.loadPct)}% (${r.class}), ${Math.round(r.bitsPerSec)} bit/s`);
  console.log(`  estimated 8-byte frame time: ${fmt(r.frameTimeUs.f5, 0)} us`);
}

function printSweep(base) {
  const rows = [];
  for (const bitrate of [500000, 1000000]) {
    for (const hz of [50, 100, 150, 200, 250, 300]) {
      for (const ack of [false, true]) {
        const r = calc({ ...base, bitrate, controlHz: hz, encoderHz: hz, ack });
        rows.push({
          bitrate: `${bitrate / 1000}K`,
          hz,
          ack: ack ? 'on' : 'off',
          fps: r.framesPerSec.total,
          load: `${fmt(r.loadPct)}%`,
          class: r.class,
        });
      }
    }
  }
  console.table(rows);
}

function main() {
  const opts = parseArgs(process.argv);
  if (opts.sweep) {
    if (opts.json) {
      const out = [];
      for (const bitrate of [500000, 1000000]) {
        for (const hz of [50, 100, 150, 200, 250, 300]) {
          for (const ack of [false, true]) out.push(calc({ ...opts, bitrate, controlHz: hz, encoderHz: hz, ack }));
        }
      }
      console.log(JSON.stringify(out, null, 2));
    } else {
      printSweep(opts);
    }
    return;
  }
  const r = calc(opts);
  if (opts.json) console.log(JSON.stringify(r, null, 2));
  else printOne(r);
}

main();
