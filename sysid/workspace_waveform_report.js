#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { analyze } = require('./workspace_evaluation');

const ROOT = path.resolve(__dirname, '..');
const DATA_DIR = path.join(ROOT, 'sysid', 'data');
const OUT_DIR = path.join(DATA_DIR, 'reports');

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

function arg(name, fallback = null) {
  const i = process.argv.indexOf(name);
  return i >= 0 ? process.argv[i + 1] : fallback;
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

function rel(file) {
  return path.relative(ROOT, file).replaceAll(path.sep, '/');
}

function plotPaths(file) {
  const base = path.basename(file, '.jsonl');
  const motor = path.join(DATA_DIR, 'plots', `${base}_motor6.svg`);
  const plane = path.join(DATA_DIR, 'plots', `${base}_plane6.svg`);
  return {
    motor: fs.existsSync(motor) ? rel(motor) : null,
    plane: fs.existsSync(plane) ? rel(plane) : null,
  };
}

function runCard(file) {
  const ev = analyze(file);
  const plots = plotPaths(file);
  const q = ev.quality;
  const b = ev.fullBadness;
  const h = ev.health;
  return `
    <section class="run ${esc(q.verdict)}">
      <h2>${esc(ev.file)}</h2>
      <div class="meta">
        <span>${esc(q.verdict)}</span>
        <span>max step ${b.motorStepMaxDeg}deg</span>
        <span>p99 ${b.motorStepP99MeanDeg}deg</span>
        <span>cross ${b.poseCrossHpPeak}</span>
        <span>worst ${esc(b.worstMotorStep?.label || '-')}</span>
        <span>rxDrop/s ${h.rxDropPerS}</span>
        <span>ef ${h.efOr}</span>
      </div>
      <div class="failures">${q.failures.map((x) => `<span>${esc(x)}</span>`).join('')}</div>
      <div class="plots">
        ${plots.motor ? `<img src="../${esc(plots.motor)}" alt="motor waveform">` : '<div class="empty">missing motor plot</div>'}
        ${plots.plane ? `<img src="../${esc(plots.plane)}" alt="plane waveform">` : '<div class="empty">missing plane plot</div>'}
      </div>
    </section>
  `;
}

function main() {
  fs.mkdirSync(OUT_DIR, { recursive: true });
  const runs = files();
  if (!runs.length) throw new Error('no active JSONL runs found');
  const html = `<!doctype html>
<meta charset="utf-8">
<title>Workspace Evaluation Waveforms</title>
<style>
body{margin:24px;background:#f4f1eb;color:#25211d;font:14px Arial,sans-serif}
h1{font-size:22px;margin:0 0 18px}.run{margin:0 0 28px;padding:18px;background:white;border:1px solid #ddd8cf}
h2{font-size:16px;margin:0 0 10px}.meta{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:10px}
.meta span{border:1px solid #d9d1c6;padding:4px 8px;border-radius:999px}.reject .meta span:first-child{border-color:#c85b43;color:#c85b43}
.failures{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px;color:#7a2d20}.failures span{background:#f9e7e0;padding:4px 7px;border-radius:4px}
.plots{display:grid;grid-template-columns:1fr;gap:14px}.plots img{width:100%;height:auto;background:#0b0e14}.empty{padding:24px;background:#faf8f3;border:1px dashed #d9d1c6}
@media (min-width:1200px){.plots{grid-template-columns:1fr 1fr}}
</style>
<h1>Workspace Evaluation Waveforms</h1>
${runs.map(runCard).join('\n')}`;
  const out = path.join(OUT_DIR, `workspace-evaluation-waveforms_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.html`);
  fs.writeFileSync(out, html);
  console.log(path.relative(ROOT, out));
}

if (require.main === module) {
  try { main(); }
  catch (err) {
    console.error(`[workspace:waveforms] ${err.message}`);
    process.exit(1);
  }
}
