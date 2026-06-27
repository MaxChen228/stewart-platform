#!/usr/bin/env node
// Summarize server.js JSONL recordings into control/CAN health metrics.

const fs = require('fs');
const path = require('path');

function usage() {
  console.log(`Usage:
  node sysid/telemetry_summary.js sysid/data/file.jsonl [more.jsonl...]

Outputs JSON summary for each recording.
`);
}

function percentile(arr, p) {
  if (!arr.length) return null;
  const a = [...arr].sort((x, y) => x - y);
  const idx = (a.length - 1) * (p / 100);
  const lo = Math.floor(idx), hi = Math.ceil(idx);
  if (lo === hi) return a[lo];
  return a[lo] + (a[hi] - a[lo]) * (idx - lo);
}

function avg(arr) {
  return arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : null;
}

function max(arr) {
  return arr.length ? Math.max(...arr) : null;
}

function min(arr) {
  return arr.length ? Math.min(...arr) : null;
}

function round(v, d = 3) {
  if (v === null || v === undefined || !Number.isFinite(v)) return null;
  return Number(v.toFixed(d));
}

function counterDelta(values) {
  let total = 0;
  for (let i = 1; i < values.length; i++) {
    const prev = values[i - 1], cur = values[i];
    if (!Number.isFinite(prev) || !Number.isFinite(cur)) continue;
    total += cur >= prev ? cur - prev : cur;
  }
  return total;
}

function load(file) {
  const tele = [];
  const cmds = [];
  let meta = null;
  for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
    if (!line.trim()) continue;
    let rec;
    try { rec = JSON.parse(line); } catch { continue; }
    if (rec.meta) { meta = rec.meta; continue; }
    if (rec.dir === 'cmd') { cmds.push({ t: rec.t, cmd: rec.d }); continue; }
    if (rec.dir !== 'in') continue;
    try {
      const d = typeof rec.d === 'string' ? JSON.parse(rec.d) : rec.d;
      if (d && Array.isArray(d.a)) {
        d._hostT = rec.t;
        tele.push(d);
      }
    } catch {}
  }
  return { meta, tele, cmds };
}

function summarize(file) {
  const { meta, tele, cmds } = load(file);
  const hostDt = [];
  for (let i = 1; i < tele.length; i++) hostDt.push(tele[i]._hostT - tele[i - 1]._hostT);
  const fwDt = tele.map((d) => Number(d.dt)).filter(Number.isFinite);
  const lhz = tele.map((d) => Number(d.lhz)).filter(Number.isFinite);
  const canReadUs = tele.map((d) => Number(d.cus)).filter(Number.isFinite);
  const ef = tele.map((d) => Number(d.can?.ef ?? d.ef)).filter(Number.isFinite);
  const tx = tele.map((d) => Number(d.can?.tx ?? d.tx)).filter(Number.isFinite);
  const rx = tele.map((d) => Number(d.can?.rx ?? d.rx)).filter(Number.isFinite);
  const rxDrop = tele.map((d) => Number(d.can?.rxDrop)).filter(Number.isFinite);
  const txFail = tele.map((d) => Number(d.can?.txFail)).filter(Number.isFinite);
  const qDepth = tele.map((d) => Number(d.can?.qDepth)).filter(Number.isFinite);
  const qMax = tele.map((d) => Number(d.can?.qMax)).filter(Number.isFinite);
  const timDtMax = tele.map((d) => Number(d.tim?.dtMax)).filter(Number.isFinite);
  const timF5Max = tele.map((d) => Number(d.tim?.f5max)).filter(Number.isFinite);
  const ctlIk = tele.map((d) => Number(d.ctl?.ikFail)).filter(Number.isFinite);
  const ctlFk = tele.map((d) => Number(d.ctl?.fkFail)).filter(Number.isFinite);
  const ctlLimit = tele.map((d) => Number(d.ctl?.coordLimit)).filter(Number.isFinite);
  const ctlCoordMax = tele.map((d) => Number(d.ctl?.coordMax)).filter(Number.isFinite);
  const ok = tele.map((d) => Number(d.ok)).filter(Number.isFinite);
  const bus = tele.map((d) => d.bus || {});
  const busRx = bus.map((b) => Number(b.rx)).filter(Number.isFinite);
  const busOvr = bus.map((b) => Number(b.ovr)).filter(Number.isFinite);
  const f5us = bus.map((b) => Number(b.f5us)).filter(Number.isFinite);
  const mxd = bus.map((b) => Number(b.mxd)).filter(Number.isFinite);
  const per = bus.flatMap((b) => Array.isArray(b.per) ? b.per.map(Number).filter(Number.isFinite) : []);
  const hmax = tele.map((d) => Number(d.hmax)).filter(Number.isFinite);
  const herrAbs = tele.flatMap((d) => Array.isArray(d.herr) ? d.herr.map((x) => Math.abs(Number(x))).filter(Number.isFinite) : []);

  const durationS = tele.length >= 2 ? (tele[tele.length - 1]._hostT - tele[0]._hostT) / 1000 : 0;
  return {
    file: path.basename(file),
    meta,
    samples: tele.length,
    durationS: round(durationS, 2),
    commands: cmds.map((c) => c.cmd),
    hostDtMs: {
      avg: round(avg(hostDt), 2),
      p95: round(percentile(hostDt, 95), 2),
      p99: round(percentile(hostDt, 99), 2),
      max: round(max(hostDt), 2),
    },
    firmwareDtMs: {
      avg: round(avg(fwDt), 2),
      p95: round(percentile(fwDt, 95), 2),
      max: round(max(fwDt), 2),
    },
    loopHz: {
      avg: round(avg(lhz), 2),
      min: round(min(lhz), 2),
      max: round(max(lhz), 2),
    },
    canReadUs: {
      avg: round(avg(canReadUs), 1),
      p95: round(percentile(canReadUs, 95), 1),
      max: round(max(canReadUs), 1),
    },
    canHealth: {
      backend: tele.find((d) => d.can?.backend)?.can?.backend || null,
      bitrate: tele.find((d) => d.can?.bitrate)?.can?.bitrate || null,
      efOr: ef.reduce((a, b) => a | (b & 0xff), 0),
      efMax: max(ef),
      txMax: max(tx),
      rxMax: max(rx),
      rxDropSum: counterDelta(rxDrop),
      txFailSum: counterDelta(txFail),
      qDepthMax: max(qDepth),
      qMaxMax: max(qMax),
      errorPassiveFrac: tx.length ? round(tx.filter((x) => x >= 128).length / tx.length, 4) : null,
    },
    timingV2: {
      dtMaxMax: max(timDtMax),
      f5maxMax: max(timF5Max),
    },
    controller: {
      modes: [...new Set(tele.map((d) => d.ctl?.mode).filter(Boolean))],
      ikFailSum: ctlIk.reduce((a, b) => a + b, 0),
      fkFailSum: ctlFk.reduce((a, b) => a + b, 0),
      coordLimitSum: ctlLimit.reduce((a, b) => a + b, 0),
      coordMaxMax: max(ctlCoordMax),
    },
    bus: {
      rxAvg: round(avg(busRx), 2),
      ovrSum: busOvr.reduce((a, b) => a + b, 0),
      f5usP95: round(percentile(f5us, 95), 1),
      f5usMax: max(f5us),
      maxDrainMax: max(mxd),
      perMin: min(per),
      perAvg: round(avg(per), 2),
    },
    encoders: {
      okAvg: round(avg(ok), 3),
      failFrac: ok.length ? round(ok.filter((x) => x < 6).length / ok.length, 4) : null,
    },
    hold: {
      hmaxMax: round(max(hmax), 3),
      herrAbsP95: round(percentile(herrAbs, 95), 3),
      herrAbsMax: round(max(herrAbs), 3),
    },
  };
}

function main() {
  const files = process.argv.slice(2);
  if (!files.length) { usage(); process.exit(1); }
  const out = files.map(summarize);
  console.log(JSON.stringify(out.length === 1 ? out[0] : out, null, 2));
}

if (require.main === module) main();
else module.exports = { summarize, load };
