'use strict';

const fs = require('fs');
const path = require('path');
const { solveFK, resetFK, NEUTRAL_Z } = require('./kin');

const COLORS = ['#ff5a4f', '#ff9f2f', '#ffd51f', '#2ed06e', '#3ca4ff', '#b864e8'];
const BG = '#0b0e14';
const PANEL = '#11151f';
const GRID = '#273043';
const FG = '#e8edf4';
const MUTED = '#8b96a8';

function usage() {
  console.error('Usage: node sysid/plot_motion6.js <record.jsonl> [--out-dir DIR]');
  process.exit(1);
}

function argValue(name, fallback = null) {
  const i = process.argv.indexOf(name);
  return i >= 0 ? process.argv[i + 1] : fallback;
}

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

function loadJsonl(file) {
  const rows = [];
  for (const line of fs.readFileSync(file, 'utf8').split(/\r?\n/)) {
    if (!line.trim()) continue;
    let r;
    try { r = JSON.parse(line); } catch { continue; }
    if (r.dir !== 'in') continue;
    let d = r.d;
    if (typeof d === 'string') {
      try { d = JSON.parse(d); } catch { continue; }
    }
    if (!d || !Array.isArray(d.a) || d.a.length !== 6) continue;
    rows.push({
      t: Number(r.t ?? d.t ?? 0),
      a: d.a.map(Number),
      target: Array.isArray(d.tgt) ? d.tgt.map(Number) : null,
      ok: Number(d.ok ?? 0),
    });
  }
  rows.sort((a, b) => a.t - b.t);
  if (!rows.length) throw new Error('no telemetry rows with six motor angles');
  const t0 = rows[0].t;
  resetFK([0, 0, NEUTRAL_Z, 0, 0, 0]);
  for (const row of rows) {
    row.ts = (row.t - t0) / 1000;
    const fk = solveFK(row.a);
    row.pose = fk.pose.map(Number);
    row.fkOk = fk.converged;
  }
  return rows;
}

function niceRange(min, max) {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [-1, 1];
  if (Math.abs(max - min) < 1e-9) return [min - 1, max + 1];
  const pad = (max - min) * 0.08;
  return [min - pad, max + pad];
}

function fmt(v) {
  if (!Number.isFinite(v)) return '';
  if (Math.abs(v) >= 100) return v.toFixed(0);
  if (Math.abs(v) >= 10) return v.toFixed(1);
  return v.toFixed(2);
}

function seriesStats(rows, key, labels) {
  return labels.map((label, i) => {
    const values = rows.map((r) => r[key][i]).filter(Number.isFinite);
    return {
      label,
      min: Math.min(...values),
      max: Math.max(...values),
      end: values[values.length - 1],
    };
  });
}

function renderChart(rows, key, labels, title, yLabel, outFile) {
  const width = 1280;
  const height = 620;
  const margin = { left: 76, right: 28, top: 92, bottom: 64 };
  const plotW = width - margin.left - margin.right;
  const plotH = height - margin.top - margin.bottom;
  const tMin = rows[0].ts;
  const tMax = rows[rows.length - 1].ts || 1;
  const all = rows.flatMap((r) => r[key]).filter(Number.isFinite);
  const [yMin, yMax] = niceRange(Math.min(...all), Math.max(...all));
  const x = (t) => margin.left + ((t - tMin) / Math.max(0.001, tMax - tMin)) * plotW;
  const y = (v) => margin.top + (1 - (v - yMin) / Math.max(0.001, yMax - yMin)) * plotH;
  const xTicks = 8;
  const yTicks = 6;
  const parts = [];

  parts.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">`);
  parts.push(`<rect width="100%" height="100%" fill="${BG}"/>`);
  parts.push(`<rect x="${margin.left}" y="${margin.top}" width="${plotW}" height="${plotH}" fill="${PANEL}" stroke="${GRID}"/>`);
  parts.push(`<text x="32" y="42" fill="${FG}" font-family="Menlo, monospace" font-size="22" font-weight="700">${esc(title)}</text>`);
  parts.push(`<text x="32" y="68" fill="${MUTED}" font-family="Menlo, monospace" font-size="13">samples=${rows.length} duration=${fmt(tMax - tMin)}s ok6=${fmt(100 * rows.filter((r) => r.ok === 6).length / rows.length)}%</text>`);

  for (let i = 0; i <= xTicks; i++) {
    const tx = tMin + (tMax - tMin) * i / xTicks;
    const px = x(tx);
    parts.push(`<line x1="${px}" y1="${margin.top}" x2="${px}" y2="${margin.top + plotH}" stroke="${GRID}" stroke-width="1"/>`);
    parts.push(`<text x="${px}" y="${height - 30}" fill="${MUTED}" font-family="Menlo, monospace" font-size="12" text-anchor="middle">${fmt(tx)}s</text>`);
  }
  for (let i = 0; i <= yTicks; i++) {
    const vy = yMin + (yMax - yMin) * i / yTicks;
    const py = y(vy);
    parts.push(`<line x1="${margin.left}" y1="${py}" x2="${margin.left + plotW}" y2="${py}" stroke="${GRID}" stroke-width="1"/>`);
    parts.push(`<text x="${margin.left - 10}" y="${py + 4}" fill="${MUTED}" font-family="Menlo, monospace" font-size="12" text-anchor="end">${fmt(vy)}</text>`);
  }
  parts.push(`<text x="${margin.left + plotW / 2}" y="${height - 10}" fill="${FG}" font-family="Menlo, monospace" font-size="13" text-anchor="middle">time</text>`);
  parts.push(`<text x="20" y="${margin.top + plotH / 2}" fill="${FG}" font-family="Menlo, monospace" font-size="13" transform="rotate(-90 20 ${margin.top + plotH / 2})" text-anchor="middle">${esc(yLabel)}</text>`);

  labels.forEach((label, i) => {
    const points = rows.map((r) => `${x(r.ts).toFixed(2)},${y(r[key][i]).toFixed(2)}`).join(' ');
    parts.push(`<polyline points="${points}" fill="none" stroke="${COLORS[i]}" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round"/>`);
  });

  const stats = seriesStats(rows, key, labels);
  const legendX = margin.left + 18;
  const legendY = margin.top + 20;
  stats.forEach((s, i) => {
    const lx = legendX + i * 185;
    parts.push(`<line x1="${lx}" y1="${legendY}" x2="${lx + 22}" y2="${legendY}" stroke="${COLORS[i]}" stroke-width="4" stroke-linecap="round"/>`);
    parts.push(`<text x="${lx + 30}" y="${legendY + 4}" fill="${FG}" font-family="Menlo, monospace" font-size="13">${esc(s.label)}</text>`);
    parts.push(`<text x="${lx + 30}" y="${legendY + 22}" fill="${MUTED}" font-family="Menlo, monospace" font-size="10">end ${fmt(s.end)}</text>`);
  });

  parts.push('</svg>');
  fs.writeFileSync(outFile, parts.join('\n'));
}

function main() {
  const file = process.argv.find((x, i) => i > 1 && !x.startsWith('--') && process.argv[i - 1] !== '--out-dir');
  if (!file) usage();
  const rows = loadJsonl(file);
  const outDir = argValue('--out-dir', path.dirname(file));
  fs.mkdirSync(outDir, { recursive: true });
  const base = path.basename(file, path.extname(file));
  const motorOut = path.join(outDir, `${base}_motor6.svg`);
  const planeOut = path.join(outDir, `${base}_plane6.svg`);
  renderChart(rows, 'a', ['M1', 'M2', 'M3', 'M4', 'M5', 'M6'], 'Motor angles - 6 curves', 'deg', motorOut);
  renderChart(rows, 'pose', ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'], 'Platform pose from FK - 6 curves', 'mm / deg', planeOut);
  console.log(JSON.stringify({ rows: rows.length, motorOut, planeOut }, null, 2));
}

main();
