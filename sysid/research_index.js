#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { summarize: heaveMetrics } = require('./heave_step_metrics');

const ROOT = path.join(__dirname, '..');
const DATA_DIR = path.join(__dirname, 'data');
const PLOTS_DIR = path.join(DATA_DIR, 'plots');
const OUT_JSON = path.join(DATA_DIR, 'research-index.json');
const OUT_MD = path.join(ROOT, 'docs', 'research-dashboard.md');

function rel(file) {
  return path.relative(ROOT, file).replaceAll(path.sep, '/');
}

function mdLink(label, file) {
  return `[${label}](../${rel(file)})`;
}

function readJson(file) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); }
  catch { return null; }
}

function listFiles(dir, pred) {
  const out = [];
  for (const name of fs.readdirSync(dir)) {
    const file = path.join(dir, name);
    const st = fs.statSync(file);
    if (st.isDirectory()) continue;
    if (!pred || pred(file)) out.push(file);
  }
  return out.sort();
}

function firstExisting(files) {
  return files.find((x) => fs.existsSync(x)) || null;
}

function heaveLabel(file) {
  const base = path.basename(file);
  if (base.includes('zbias4p5')) return 'pure-P + zBias 4.5';
  if (base.includes('zbias4')) return 'pure-P + zBias 4.0';
  if (base.includes('pureP')) return 'pure-P';
  if (base.includes('baseline')) return 'baseline mixed';
  return 'Ki=20';
}

function conditionFromName(file) {
  const base = path.basename(file);
  const out = {
    label: heaveLabel(file),
    pid: base.includes('pureP') ? [1024, 0, 0, 0] : [1024, 20, 0, 0],
    zBias: 0,
    clean: base.startsWith('clean_'),
  };
  const m = base.match(/zbias(\d+)(p(\d+))?/);
  if (m) out.zBias = Number(`${m[1]}.${m[3] || 0}`);
  return out;
}

function collectHeave() {
  const jsonls = listFiles(DATA_DIR, (file) => /heave-step_h38_ms5000_.*\.jsonl$/.test(file));
  return jsonls.map((file) => {
    const metrics = heaveMetrics(file);
    const condition = conditionFromName(file);
    const stem = file.replace(/\.jsonl$/, '');
    const motorPlot = firstExisting([
      path.join(PLOTS_DIR, `${path.basename(stem)}_motor6.svg`),
    ]);
    const planePlot = firstExisting([
      path.join(PLOTS_DIR, `${path.basename(stem)}_plane6.svg`),
    ]);
    const summary = readJson(`${stem}.summary.json`);
    const bundle = readJson(`${stem}.bundle.json`);
    return {
      condition,
      file,
      summaryPath: fs.existsSync(`${stem}.summary.json`) ? `${stem}.summary.json` : null,
      bundlePath: fs.existsSync(`${stem}.bundle.json`) ? `${stem}.bundle.json` : null,
      motorPlot,
      planePlot,
      metrics,
      summary,
      bundle,
    };
  });
}

function scoreHeave(run) {
  const e = Math.abs(run.metrics.absolute.finalErrorVsNominal143 ?? run.metrics.absolute.finalErrorVsCommand ?? 99);
  const c = run.metrics.coupling.crossPeak ?? 99;
  const badCan = run.metrics.health?.okFailFrac ? 10 : 0;
  return e * 2 + c + badCan;
}

function collectCanRuns() {
  const files = listFiles(DATA_DIR, (file) => /^freertos_.*\.jsonl$/.test(path.basename(file)));
  return files.map((file) => ({ file, name: path.basename(file) }));
}

function collectBatteryRuns() {
  const files = listFiles(DATA_DIR, (file) => /^battery_.*\.jsonl$/.test(path.basename(file)));
  return files.map((file) => ({ file, name: path.basename(file) }));
}

function collectStabilityRuns() {
  const files = listFiles(DATA_DIR, (file) => file.endsWith('.stability.json'));
  return files.map((file) => {
    const stability = readJson(file);
    if (!stability) return null;
    const bundle = readJson(file.replace(/\.stability\.json$/, '.bundle.json'));
    return {
      file,
      label: path.basename(file).replace(/\.stability\.json$/, ''),
      stability,
      bundle,
      recording: bundle?.recording || stability.path || null,
      planePlot: bundle?.plot?.planeOut || null,
      motorPlot: bundle?.plot?.motorOut || null,
    };
  }).filter(Boolean).sort((a, b) => (b.stability.score?.stability || 0) - (a.stability.score?.stability || 0));
}

function fmt(x, digits = 2, unit = '') {
  return Number.isFinite(x) ? `${x.toFixed(digits)}${unit}` : '-';
}

function writeMarkdown(index) {
  const best = index.currentBest.heave;
  const lines = [];
  lines.push('# Research Dashboard');
  lines.push('');
  lines.push(`Generated: ${index.generatedAt}`);
  lines.push('');
  if (!best) {
    lines.push('## Read This First');
    lines.push('');
    lines.push('- No active experiment runs are currently indexed.');
    lines.push('- Archived runs may exist under `sysid/data/archive/`, but they are intentionally hidden from the active dashboard.');
    lines.push('- Run a new recorded experiment to repopulate this dashboard and the research run panel.');
    lines.push('');
    lines.push('## Data Inventory');
    lines.push('');
    lines.push(`- Heave runs indexed: ${index.heaveRuns.length}`);
    lines.push(`- FreeRTOS/CAN diagnostic files: ${index.canRuns.length}`);
    lines.push(`- Battery/PID disturbance files: ${index.batteryRuns.length}`);
    lines.push(`- Stability-scored runs: ${index.stabilityRuns.length}`);
    lines.push(`- Machine-readable index: ${mdLink('research-index.json', OUT_JSON)}`);
    lines.push('');
    fs.writeFileSync(OUT_MD, lines.join('\n'));
    return;
  }
  lines.push('## Read This First');
  lines.push('');
  lines.push('- Current best heave finding: use a clean inner loop and move static height compensation upward.');
  lines.push(`- Best tested candidate: **${best.condition.label}**; final height error ${fmt(best.metrics.absolute.finalErrorVsNominal143, 2, 'mm')}, cross-axis peak ${fmt(best.metrics.coupling.crossPeak, 2, 'mm')}.`);
  lines.push('- Practical interpretation: joint Ki makes height accurate, but it injects much larger MIMO coupling. A simple Z bias proves the compensation belongs above the joint loop; it is not the final controller.');
  lines.push('- Professional target: replace fixed bias with a calibrated static compensation model, then let pose-space LQI remove the remaining slow error.');
  lines.push('- Still unresolved: MCP2515/500K continues to show CAN error flags and RX drops during motion; treat high-rate control results with caution.');
  lines.push('');
  lines.push('## Best Known Operating Direction');
  lines.push('');
  lines.push('| Layer | Recommendation | Evidence |');
  lines.push('|---|---|---|');
  lines.push(`| Inner motor PID | Prefer pure-P / low Ki for clean dynamics | pure-P cross peak ${fmt(index.reference.pureP?.metrics.coupling.crossPeak, 2, 'mm')} vs Ki=20 ${fmt(index.reference.ki20?.metrics.coupling.crossPeak, 2, 'mm')} |`);
  lines.push(`| Height compensation | Use zBias ~4.5mm only as a local calibration point | final error ${fmt(best.metrics.absolute.finalErrorVsNominal143, 2, 'mm')} at this one operating point |`);
  lines.push('| Professional compensation | Fit a static pose/load compensation map, not one fixed constant | generalizes across height, tilt, payload, and workspace position |');
  lines.push('| Next controller | pose-space feedforward + LQI residual, then LQR/MPC | avoids six independent joint integrators fighting each other |');
  lines.push('| Data plane | Replace/upgrade MCP2515 path before trusting 200-300Hz results | CAN ef/rxDrop remains visible in all motion runs |');
  lines.push('');
  lines.push('## Heave Evidence Table');
  lines.push('');
  lines.push('| Case | PID | zBias | actual final Z | final err vs 143 | cross peak | cross/Z | CAN ef/rxDrop | Data | Plots |');
  lines.push('|---|---:|---:|---:|---:|---:|---:|---:|---|---|');
  for (const run of index.heaveRuns) {
    const m = run.metrics;
    const pid = `[${run.condition.pid.join(',')}]`;
    const data = mdLink('jsonl', run.file);
    const plots = [run.planePlot ? mdLink('plane', run.planePlot) : null, run.motorPlot ? mdLink('motor', run.motorPlot) : null].filter(Boolean).join(' / ') || '-';
    lines.push(`| ${run.condition.label} | ${pid} | ${fmt(run.condition.zBias, 1, 'mm')} | ${fmt(m.absolute.actualFinalZ, 2, 'mm')} | ${fmt(m.absolute.finalErrorVsNominal143, 2, 'mm')} | ${fmt(m.coupling.crossPeak, 2, 'mm')} | ${fmt(100 * m.coupling.crossOverZ, 1, '%')} | ${m.health.canEfOr ?? '-'}/${m.health.rxDropSum ?? '-'} | ${data} | ${plots} |`);
  }
  lines.push('');
  const zRuns = index.stabilityRuns.filter((x) => x.label.startsWith('z_sweep') && x.label.includes('_clean'));
  if (zRuns.length) {
    lines.push('## Z Sweep Stability Table');
    lines.push('');
    lines.push('| Case | Score | tracking | cross-axis | oscillation | quality penalty | CAN rxDrop/s | Data | Plots |');
    lines.push('|---|---:|---:|---:|---:|---:|---:|---|---|');
    for (const run of zRuns) {
      const s = run.stability.score;
      const q = run.stability.quality?.can || {};
      const data = run.recording ? mdLink('jsonl', run.recording) : '-';
      const plots = [run.planePlot ? mdLink('plane', run.planePlot) : null, run.motorPlot ? mdLink('motor', run.motorPlot) : null].filter(Boolean).join(' / ') || '-';
      lines.push(`| ${run.label} | ${fmt(s.stability, 1)} | ${fmt(s.trackingCost, 2)} | ${fmt(s.crossAxisCost, 2)} | ${fmt(s.oscillationCost, 2)} | ${fmt(s.qualityPenalty, 2)} | ${fmt(q.rxDropPerS, 2)} | ${data} | ${plots} |`);
    }
    lines.push('');
  }
  lines.push('## What To Do Next');
  lines.push('');
  lines.push('1. Treat `zBias=4.5mm` as an anchor point, not a final setting.');
  lines.push('2. Repeat clean sweeps for heave +5, +10, +20mm and a few tilted poses to fit `bias = f(pose, load)`.');
  lines.push('3. Keep the fitted compensation as feedforward; do not hide it inside joint Ki.');
  lines.push('4. Add pose-space LQI only for the residual slow error after feedforward.');
  lines.push('5. Keep recorder main-only for sysid; landing/release data belongs in safety logs, not model fitting.');
  lines.push('');
  lines.push('## Data Inventory');
  lines.push('');
  lines.push(`- Heave runs indexed: ${index.heaveRuns.length}`);
  lines.push(`- FreeRTOS/CAN diagnostic files: ${index.canRuns.length}`);
  lines.push(`- Battery/PID disturbance files: ${index.batteryRuns.length}`);
  lines.push(`- Stability-scored runs: ${index.stabilityRuns.length}`);
  lines.push(`- Machine-readable index: ${mdLink('research-index.json', OUT_JSON)}`);
  lines.push('');
  lines.push('## How To Refresh');
  lines.push('');
  lines.push('```bash');
  lines.push('npm run research:index');
  lines.push('```');
  lines.push('');
  fs.writeFileSync(OUT_MD, lines.join('\n'));
}

function main() {
  const heaveRuns = collectHeave().sort((a, b) => {
    const at = a.metrics.meta?.wallClock || a.summary?.meta?.wallClock || '';
    const bt = b.metrics.meta?.wallClock || b.summary?.meta?.wallClock || '';
    return at.localeCompare(bt);
  });
  const cleanHeave = heaveRuns.filter((x) => x.condition.clean);
  const best = [...cleanHeave].sort((a, b) => scoreHeave(a) - scoreHeave(b))[0] || heaveRuns[0] || null;
  const index = {
    generatedAt: new Date().toISOString(),
    currentBest: { heave: best },
    reference: {
      ki20: cleanHeave.find((x) => x.condition.label === 'Ki=20') || null,
      pureP: cleanHeave.find((x) => x.condition.label === 'pure-P') || null,
      purePZBias45: cleanHeave.find((x) => x.condition.label === 'pure-P + zBias 4.5') || null,
    },
    heaveRuns,
    stabilityRuns: collectStabilityRuns(),
    canRuns: collectCanRuns(),
    batteryRuns: collectBatteryRuns(),
  };
  fs.writeFileSync(OUT_JSON, JSON.stringify(index, null, 2));
  writeMarkdown(index);
  console.log(JSON.stringify({
    index: rel(OUT_JSON),
    dashboard: rel(OUT_MD),
    heaveRuns: heaveRuns.length,
    bestHeave: best ? best.condition.label : null,
  }, null, 2));
}

main();
