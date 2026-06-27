#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { score, load, cropToWindow, buildTargetModel, companionManifest, AXES } = require('./stability_score');

const ROOT = path.resolve(__dirname, '..');
const DATA_DIR = path.join(ROOT, 'sysid', 'data');
const OUT_DIR = path.join(DATA_DIR, 'reports');
const COLORS = ['#c0392b', '#2980b9', '#16a085', '#8e44ad', '#d68910', '#2c3e50'];

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

function round(v, d = 2) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(d) : '-';
}

function files() {
  const programFilter = arg('--program');
  return fs.readdirSync(DATA_DIR)
    .filter((x) => x.endsWith('.jsonl'))
    .filter((x) => !programFilter || x.includes(programFilter))
    .map((x) => path.join(DATA_DIR, x))
    .filter((file) => fs.statSync(file).isFile())
    .sort();
}

function arg(name, fallback = null) {
  const i = process.argv.indexOf(name);
  return i >= 0 ? process.argv[i + 1] : fallback;
}

function range(values) {
  const xs = values.filter(Number.isFinite);
  if (!xs.length) return [-1, 1];
  const lo = Math.min(...xs);
  const hi = Math.max(...xs);
  if (Math.abs(hi - lo) < 1e-9) return [lo - 1, hi + 1];
  const pad = (hi - lo) * 0.1;
  return [lo - pad, hi + pad];
}

function pathData(points, xOf, yOf) {
  let out = '';
  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    if (!Number.isFinite(p.y)) continue;
    out += `${out ? 'L' : 'M'}${xOf(p.t).toFixed(1)},${yOf(p.y).toFixed(1)}`;
  }
  return out;
}

function runData(file) {
  const loaded = load(file);
  const scoped = cropToWindow(loaded.cmds, loaded.tele, loaded.events);
  const result = score(file);
  const targetModel = buildTargetModel(scoped.cmds, scoped.tele, result.targetModel.referenceComp || {});
  const manifest = companionManifest(file);
  const rows = scoped.tele.map((row) => ({
    t: row.t,
    pose: row.pose,
    target: targetModel.referenceTargetAt(row.t),
  })).filter((r) => Array.isArray(r.target));
  return {
    id: path.basename(file, '.jsonl'),
    program: manifest?.program?.name || path.basename(file, '.jsonl'),
    condition: manifest?.condition || null,
    score: result.score,
    commandedAxes: new Set(result.commandedAxes),
    window: result.targetModel.scoringWindow,
    rows,
  };
}

function chart(run) {
  const W = 1120;
  const H = 540;
  const margin = { l: 58, r: 16, t: 34, b: 24 };
  const panelW = (W - margin.l - margin.r - 24) / 2;
  const panelH = (H - margin.t - margin.b - 36) / 3;
  const t0 = run.rows[0]?.t || 0;
  const t1 = run.rows.at(-1)?.t || (t0 + 1);
  const xOfFactory = (col) => (t) => margin.l + col * (panelW + 24) + ((t - t0) / Math.max(1, t1 - t0)) * panelW;
  const parts = [];
  parts.push(`<svg viewBox="0 0 ${W} ${H}" width="${W}" height="${H}" role="img">`);
  parts.push(`<rect width="${W}" height="${H}" fill="#fbfaf8"/>`);
  AXES.forEach((axis, i) => {
    const col = i % 2;
    const row = Math.floor(i / 2);
    const x0 = margin.l + col * (panelW + 24);
    const y0 = margin.t + row * (panelH + 18);
    const actual = run.rows.map((r) => ({ t: r.t, y: r.pose[i] }));
    const target = run.rows.map((r) => ({ t: r.t, y: r.target[i] }));
    const [lo, hi] = range([...actual.map((p) => p.y), ...target.map((p) => p.y)]);
    const xOf = xOfFactory(col);
    const yOf = (v) => y0 + (1 - (v - lo) / Math.max(0.001, hi - lo)) * panelH;
    const color = COLORS[i];
    parts.push(`<rect x="${x0}" y="${y0}" width="${panelW}" height="${panelH}" fill="#fff" stroke="#ddd"/>`);
    for (let g = 1; g < 4; g++) {
      const gy = y0 + panelH * g / 4;
      parts.push(`<line x1="${x0}" y1="${gy}" x2="${x0 + panelW}" y2="${gy}" stroke="#eee"/>`);
    }
    parts.push(`<path d="${pathData(target, xOf, yOf)}" fill="none" stroke="${color}" stroke-width="1.4" stroke-dasharray="5 5" opacity="0.62"/>`);
    parts.push(`<path d="${pathData(actual, xOf, yOf)}" fill="none" stroke="${color}" stroke-width="2.1"/>`);
    parts.push(`<text x="${x0 + 6}" y="${y0 + 16}" font-family="Arial, sans-serif" font-size="13" fill="#222">${esc(axis)}${run.commandedAxes.has(axis) ? ' *' : ''}</text>`);
    parts.push(`<text x="${x0 + panelW - 8}" y="${y0 + 16}" font-family="Arial, sans-serif" font-size="11" fill="#777" text-anchor="end">${round(lo)}..${round(hi)}</text>`);
  });
  parts.push(`<text x="20" y="22" font-family="Arial, sans-serif" font-size="14" font-weight="700" fill="#222">${esc(run.program)} · ${esc(run.id)}</text>`);
  parts.push(`<text x="${W - 20}" y="22" font-family="Arial, sans-serif" font-size="12" fill="#555" text-anchor="end">solid=actual · dashed=reference · score=${round(run.score.stability)}</text>`);
  parts.push('</svg>');
  return parts.join('\n');
}

function conditionLabel(condition) {
  if (!condition) return '';
  const parts = [];
  if (Number(condition.zBias)) parts.push(`zBias ${condition.zBias}`);
  if (Number.isFinite(Number(condition.vmaxT))) parts.push(`vT ${condition.vmaxT}`);
  if (Number.isFinite(Number(condition.vmaxR))) parts.push(`vR ${condition.vmaxR}`);
  return parts.join(' / ');
}

function main() {
  fs.mkdirSync(OUT_DIR, { recursive: true });
  const runs = files().map(runData);
  if (!runs.length) throw new Error('no active JSONL runs found');
  const cards = runs.map((run) => `
    <section class="run">
      <h2>${esc(run.program)}</h2>
      <div class="meta"><code>${esc(run.id)}</code> ${esc(conditionLabel(run.condition))}</div>
      ${chart(run)}
    </section>
  `).join('\n');
  const html = `<!doctype html>
<meta charset="utf-8">
<title>Workspace Waveform Report</title>
<style>
body{margin:24px;background:#f4f1eb;color:#25211d;font:14px Arial,sans-serif}
.run{margin:0 0 28px;padding:18px;background:white;border:1px solid #ddd8cf}
h1{font-size:22px;margin:0 0 18px} h2{font-size:16px;margin:0 0 4px}.meta{color:#756d62;margin-bottom:12px}
svg{max-width:100%;height:auto}
</style>
<h1>Workspace Waveform Report</h1>
${cards}`;
  const out = path.join(OUT_DIR, `workspace-waveforms_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.html`);
  fs.writeFileSync(out, html);
  console.log(path.relative(ROOT, out));
}

if (require.main === module) main();
