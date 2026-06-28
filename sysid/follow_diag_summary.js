#!/usr/bin/env node
// Summarize always-on FOLLOW/PF diagnostics emitted by server.js.

const fs = require('fs');
const path = require('path');

function usage() {
  console.log(`Usage:
  node sysid/follow_diag_summary.js sysid/data/follow-diagnostics/follow_*.jsonl
  node sysid/follow_diag_summary.js latest
`);
}

function percentile(values, p) {
  const a = values.filter(Number.isFinite).sort((x, y) => x - y);
  if (!a.length) return null;
  const idx = (a.length - 1) * (p / 100);
  const lo = Math.floor(idx), hi = Math.ceil(idx);
  if (lo === hi) return a[lo];
  return a[lo] + (a[hi] - a[lo]) * (idx - lo);
}

function avg(values) {
  const a = values.filter(Number.isFinite);
  return a.length ? a.reduce((x, y) => x + y, 0) / a.length : null;
}

function max(values) {
  const a = values.filter(Number.isFinite);
  return a.length ? Math.max(...a) : null;
}

function min(values) {
  const a = values.filter(Number.isFinite);
  return a.length ? Math.min(...a) : null;
}

function round(v, d = 3) {
  return Number.isFinite(v) ? Number(v.toFixed(d)) : null;
}

function load(file) {
  const out = { meta: null, cmds: [], tele: [], states: [], statuses: [], summaries: [], manualTargets: [], manualForwards: [], manualStops: [], manualLimits: [], parseErrors: [] };
  for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
    if (!line.trim()) continue;
    let rec;
    try { rec = JSON.parse(line); } catch { continue; }
    if (rec.meta) { out.meta = rec.meta; continue; }
    if (rec.type === 'cmd') out.cmds.push(rec);
    else if (rec.type === 'tele') out.tele.push(rec);
    else if (rec.type === 'state') out.states.push(rec);
    else if (rec.type === 'status') out.statuses.push(rec);
    else if (rec.type === 'summary') out.summaries.push(rec);
    else if (rec.type === 'manual_pf_target') out.manualTargets.push(rec);
    else if (rec.type === 'manual_pf_forward') out.manualForwards.push(rec);
    else if (rec.type === 'manual_pf_stop') out.manualStops.push(rec);
    else if (rec.type === 'manual_pf_limits') out.manualLimits.push(rec);
    else if (rec.type === 'parse_error') out.parseErrors.push(rec);
  }
  return out;
}

function frefVelTOf(x) {
  if (!Array.isArray(x.vel)) return null;
  if (x.vel.length === 2) return Math.abs(Number(x.vel[0]));
  return max(x.vel.slice(0, 3).map((v) => Math.abs(Number(v))));
}

function frefVelROf(x) {
  if (!Array.isArray(x.vel)) return null;
  if (x.vel.length === 2) return Math.abs(Number(x.vel[1]));
  return max(x.vel.slice(3, 6).map((v) => Math.abs(Number(v))));
}

function latestFollowDiagFile() {
  const dir = path.join(__dirname, 'data', 'follow-diagnostics');
  const files = fs.readdirSync(dir)
    .filter((x) => /^follow_.*\.jsonl$/.test(x))
    .map((x) => path.join(dir, x))
    .sort((a, b) => fs.statSync(b).mtimeMs - fs.statSync(a).mtimeMs);
  if (!files.length) throw new Error(`no follow diagnostics found in ${dir}`);
  return files[0];
}

function summarize(file) {
  const d = load(file);
  const firstSummary = d.summaries[0] || null;
  const lastSummary = d.summaries[d.summaries.length - 1] || null;
  const pf = d.cmds.filter((x) => x.op === 'PF');
  const followCmds = d.cmds.filter((x) => x.op === 'FOLLOW');
  const vfCmds = d.cmds.filter((x) => x.op === 'VF');
  const pCmds = d.cmds.filter((x) => x.op === 'P');
  const pfDt = pf.map((x) => Number(x.dtMs)).filter(Number.isFinite);
  const stepT = pf.map((x) => Number(x.step?.maxT)).filter(Number.isFinite);
  const stepR = pf.map((x) => Number(x.step?.maxR)).filter(Number.isFinite);
  const teleDt = d.tele.map((x) => Number(x.dtMs)).filter(Number.isFinite);
  const samples = d.tele.length ? d.tele.map((x) => x.sample || {}) : d.summaries.map((x) => x.latest || {}).filter((x) => x && Object.keys(x).length);
  const durationS = d.tele.length >= 2
    ? (d.tele[d.tele.length - 1].t - d.tele[0].t) / 1000
    : (d.summaries.length >= 2 ? (d.summaries[d.summaries.length - 1].t - d.summaries[0].t) / 1000 : 0);
  const pfDurationS = pf.length >= 2 ? (pf[pf.length - 1].t - pf[0].t) / 1000 : 0;
  const herr = samples.map((x) => Number(x.herrMax)).filter(Number.isFinite);
  const ok = samples.map((x) => Number(x.ok)).filter(Number.isFinite);
  const jitter = samples.map((x) => Number(x.jitterUs)).filter(Number.isFinite);
  const f5us = samples.map((x) => Number(x.f5us)).filter(Number.isFinite);
  const rxDrop = samples.map((x) => Number(x.rxDrop)).filter(Number.isFinite);
  const txFail = samples.map((x) => Number(x.txFail)).filter(Number.isFinite);
  const ef = samples.map((x) => Number(x.ef)).filter(Number.isFinite);
  const arAge = samples.map((x) => Number(x.arAgeMax)).filter(Number.isFinite);
  const coordMax = samples.map((x) => Number(x.coordMax)).filter(Number.isFinite);
  const cmdDrop = samples.map((x) => Number(x.cmdDrop)).filter(Number.isFinite);
  const teleDrop = samples.map((x) => Number(x.teleDrop)).filter(Number.isFinite);
  const ikFail = samples.map((x) => Number(x.ikFail)).filter(Number.isFinite);
  const fref = samples.map((x) => x.fref).filter((x) => x && typeof x === 'object');
  const frefErrT = fref.map((x) => Number(x.errT)).filter(Number.isFinite);
  const frefErrR = fref.map((x) => Number(x.errR)).filter(Number.isFinite);
  const frefVelT = fref.map(frefVelTOf).filter(Number.isFinite);
  const frefVelR = fref.map(frefVelROf).filter(Number.isFinite);

  return {
    file: path.basename(file),
    meta: d.meta,
    durationS: round(durationS, 2),
    commands: {
      follow: followCmds.map((x) => x.cmd),
      vf: vfCmds.map((x) => x.cmd),
      pCount: pCmds.length,
      pfCount: pf.length || lastSummary?.mailbox?.received || lastSummary?.pf?.count || 0,
    },
    pf: {
      hz: pfDurationS > 0 ? round((pf.length - 1) / pfDurationS, 2) : lastSummary?.pf?.hz ?? null,
      dtAvg: pfDt.length ? round(avg(pfDt), 2) : lastSummary?.pf?.dtAvg ?? null,
      dtP95: pfDt.length ? round(percentile(pfDt, 95), 2) : lastSummary?.pf?.dtP95 ?? null,
      dtP99: pfDt.length ? round(percentile(pfDt, 99), 2) : null,
      dtMax: pfDt.length ? round(max(pfDt), 2) : lastSummary?.pf?.dtMax ?? null,
      stepTMax: stepT.length ? round(max(stepT), 3) : lastSummary?.pf?.stepTMax ?? null,
      stepTP95: stepT.length ? round(percentile(stepT, 95), 3) : null,
      stepRMax: stepR.length ? round(max(stepR), 3) : lastSummary?.pf?.stepRMax ?? null,
      stepRP95: stepR.length ? round(percentile(stepR, 95), 3) : null,
    },
    telemetry: {
      samples: d.tele.length || lastSummary?.tele?.count || 0,
      hz: d.tele.length && durationS > 0 ? round((d.tele.length - 1) / durationS, 2) : lastSummary?.tele?.hz ?? null,
      dtAvg: teleDt.length ? round(avg(teleDt), 2) : lastSummary?.tele?.dtAvg ?? null,
      dtP95: teleDt.length ? round(percentile(teleDt, 95), 2) : lastSummary?.tele?.dtP95 ?? null,
      dtP99: teleDt.length ? round(percentile(teleDt, 99), 2) : null,
      dtMax: teleDt.length ? round(max(teleDt), 2) : lastSummary?.tele?.dtMax ?? null,
    },
    mailbox: {
      first: firstSummary?.mailbox || null,
      last: lastSummary?.mailbox || null,
    },
    manual: {
      first: firstSummary?.manual || null,
      last: lastSummary?.manual || null,
      targetEvents: d.manualTargets.length,
      forwardEvents: d.manualForwards.length,
      stopEvents: d.manualStops.map((x) => ({ t: x.t, reason: x.reason })),
      limits: d.manualLimits.map((x) => ({ t: x.t, vmaxT: x.vmaxT, vmaxR: x.vmaxR })),
    },
    fref: {
      samples: fref.length,
      gen: fref.length ? fref[fref.length - 1].gen || null : null,
      errTMax: round(max(frefErrT), 3),
      errTP95: round(percentile(frefErrT, 95), 3),
      errRMax: round(max(frefErrR), 3),
      errRP95: round(percentile(frefErrR, 95), 3),
      velTMax: round(max(frefVelT), 3),
      velRMax: round(max(frefVelR), 3),
      latest: fref.length ? fref[fref.length - 1] : null,
    },
    parse: {
      errors: d.parseErrors.length || lastSummary?.parse?.errors || 0,
      latest: d.parseErrors.length ? d.parseErrors[d.parseErrors.length - 1] : null,
    },
    followState: {
      states: d.states.map((x) => ({ t: x.t, fl: x.fl, profile: x.profile, pos: x.pos, mode: x.mode })),
      statuses: d.statuses.map((x) => ({ t: x.t, status: x.status, fk: x.fk, pose: x.pose, tLimit: x.t, rLimit: x.r })),
    },
    health: {
      okMin: min(ok),
      herrP95: round(percentile(herr, 95), 3),
      herrMax: round(max(herr), 3),
      jitterP95Us: round(percentile(jitter, 95), 1),
      jitterMaxUs: round(max(jitter), 1),
      f5usP95: round(percentile(f5us, 95), 1),
      f5usMax: round(max(f5us), 1),
      efMax: max(ef),
      rxDropMax: max(rxDrop),
      txFailMax: max(txFail),
      arAgeMax: max(arAge),
      coordMaxMax: max(coordMax),
      cmdDropMax: max(cmdDrop),
      teleDropMax: max(teleDrop),
      ikFailMax: max(ikFail),
    },
  };
}

function main() {
  const files = process.argv.slice(2).map((x) => x === 'latest' ? latestFollowDiagFile() : x);
  if (!files.length) { usage(); process.exit(1); }
  const out = files.map(summarize);
  console.log(JSON.stringify(out.length === 1 ? out[0] : out, null, 2));
}

if (require.main === module) main();
else module.exports = { summarize, load };
