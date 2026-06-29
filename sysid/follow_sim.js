#!/usr/bin/env node
// Offline numerical simulation of the firmware follow-reference generator.
//
// The follow generator (followStep / followAxisCandidate / applyFollowTight in
// src/main.cpp) is PURE deterministic math: given a target stream it emits a
// jerk/accel/vel-limited + first-order-ease-out trajectory. The motor's internal
// PID consumes that trajectory as its setpoint, so for the reference itself the
// simulation is faithful -- it does not (and offline cannot) model the motor loop,
// CAN timing, or the mechanical plant. Use it to tune the single `tight` knob and
// the ease-out feel without hardware, with real numbers instead of guesses.
//
// Faithful port of src/main.cpp:
//   applyFollowTight()    267-276   (tight -> tau/vmax/accelT/jerkT/K)
//   followAxisCandidate() 291-330   (per-axis vel/accel/jerk limit + ease-out + snap)
//   followStep() dt clamp 335       (dt constrained to [0.001, 0.02] s)
// A single axis is integrated -- that is what determines feel. IK/workspace
// validity is geometry, not filter response, so it is intentionally not modeled.

'use strict';

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

// ---- applyFollowTight (main.cpp:267-276) ----
// brakeScale (default 1, = current firmware) derates the vBrake decel envelope to
// probe the discrete-time overshoot; <1 brakes earlier/softer.
function deriveTight(tight, brakeScale = 1.0, kScale = 1.0, jerkStop = false) {
  tight = clamp(tight, 0, 1);
  const tau = 0.45 - (0.45 - 0.10) * tight;
  return {
    tight, tau, brakeScale, jerkStop,
    vmaxT: 35 + (120 - 35) * tight,
    vmaxR: 25 + (90 - 25) * tight,
    accelT: 0.7 * tau,
    jerkT: 0.35 * tau,
    K: kScale / tau,   // ease-out gain; kScale<1 = gentler approach (probe overshoot)
  };
}

const EPS_T = 0.003, EPS_R = 0.002;  // main.cpp:244-245

// ---- followAxisCandidate (main.cpp:291-330), one axis ----
// epsOv overrides the snap epsilon (FOLLOW_EPS_T/R) so the firmware's 3-micron
// default can be swept against looser, physically-meaningful thresholds offline.
function axisStep(s, target, dt, p, rot, epsOv) {
  const vmax = rot ? p.vmaxR : p.vmaxT;
  const amax = vmax / p.accelT;
  const jmax = amax / p.jerkT;
  const eps = epsOv != null ? epsOv : (rot ? EPS_R : EPS_T);
  const { x, v, a } = s;
  const err = target - x;

  if (Math.abs(err) <= eps && Math.abs(v) <= eps / Math.max(dt, 0.001) && Math.abs(a) <= amax * 0.01) {
    return { x: target, v: 0, a: 0 };
  }
  const dir = err >= 0 ? 1 : -1;
  const vBrake = p.brakeScale * Math.sqrt(Math.max(0, 2 * amax * Math.abs(err)));
  const vApproach = p.K * Math.abs(err);                       // first-order ease-out
  const vDesired = dir * Math.min(vmax, vBrake, vApproach);
  let aDesired = clamp((vDesired - v) / Math.max(dt, 0.001), -amax, amax);
  // jerk-aware stop: cap a braking accel so it can ramp back to 0 before v crosses
  // 0 (|a| <= sqrt(2*jmax*|v|)). Without it, a pins at -amax, the jerk limit can't
  // lift the brake in time, and v is driven past 0 into a reverse overshoot ring.
  if (p.jerkStop && aDesired * v < 0 && Math.abs(v) > 1e-9) {
    const aStop = Math.sqrt(2 * jmax * Math.abs(v));
    if (Math.abs(aDesired) > aStop) aDesired = Math.sign(aDesired) * aStop;
  }
  const da = clamp(aDesired - a, -jmax * dt, jmax * dt);
  const nextA = clamp(a + da, -amax, amax);
  const nextV = clamp(v + nextA * dt, -vmax, vmax);
  let nextX = x + nextV * dt;
  if ((target - x) * (target - nextX) <= 0) return { x: target, v: 0, a: 0 };  // endpoint snap
  return { x: nextX, v: nextV, a: nextA };
}

function dtForHz(hz) { return clamp(1 / hz, 0.001, 0.02); }  // followStep clamp

// ---- step response from 0 -> target ----
// `settleEps` is the position band used to declare "settled" (decoupled from the
// generator's snap eps so the metric stays honest while we sweep the snap eps).
function simStep({ target, tight, hz, rot = false, maxT = 4.0, eps = null, settleEps = 0.05, brakeScale = 1.0, kScale = 1.0, jerkStop = false }) {
  const p = deriveTight(tight, brakeScale, kScale, jerkStop);
  const dt = dtForHz(hz);
  const sgn = Math.sign(target) || 1;
  let s = { x: 0, v: 0, a: 0 };
  const traj = [];
  let settleT = null, riseT = null, peakV = 0, peakA = 0, ring = 0;
  const n = Math.ceil(maxT / dt);
  for (let k = 0; k <= n; k++) {
    const t = k * dt;
    traj.push({ t, x: s.x, v: s.v, a: s.a });
    peakV = Math.max(peakV, Math.abs(s.v));
    peakA = Math.max(peakA, Math.abs(s.a));
    // ring = backward motion (velocity opposite to the approach dir) once past 80% --
    // the signature of overshoot-and-return near the target.
    if (Math.abs(s.x) >= 0.8 * Math.abs(target) && -s.v * sgn > ring) ring = -s.v * sgn;
    if (riseT == null && Math.abs(s.x) >= 0.9 * Math.abs(target)) riseT = t;
    if (Math.abs(target - s.x) <= settleEps && Math.abs(s.v) <= settleEps / dt) {
      if (settleT == null) settleT = t;
    } else { settleT = null; }   // excursion out of band re-arms settle
    s = axisStep(s, target, dt, p, rot, eps);
  }
  return { p, dt, traj, m: { settleT, riseT, peakV, peakA, ring, vmax: rot ? p.vmaxR : p.vmaxT } };
}

// ---- constant-velocity ramp tracking: steady-state position lag ----
function simRampLag({ vTarget, tight, hz, rot = false, maxT = 3.0 }) {
  const p = deriveTight(tight);
  const dt = dtForHz(hz);
  let s = { x: 0, v: 0, a: 0 };
  const n = Math.ceil(maxT / dt);
  let lagTail = 0, cnt = 0;
  for (let k = 0; k <= n; k++) {
    const t = k * dt;
    const target = vTarget * t;
    if (t > maxT * 0.6) { lagTail += (target - s.x); cnt++; }   // steady-state window
    s = axisStep(s, target, dt, p, rot);
  }
  return { lag: cnt ? lagTail / cnt : null, vmax: rot ? p.vmaxR : p.vmaxT, tau: p.tau };
}

// ---- tiny ASCII line plot ----
function asciiPlot(series, target, rows = 14, cols = 70) {
  const T = series[series.length - 1].t;
  const ymax = Math.max(target * 1.05, ...series.map((s) => s.x));
  const ymin = Math.min(0, ...series.map((s) => s.x));
  const grid = Array.from({ length: rows }, () => Array(cols).fill(' '));
  const colT = (t) => Math.min(cols - 1, Math.round((t / T) * (cols - 1)));
  const rowY = (y) => Math.min(rows - 1, Math.max(0, Math.round((1 - (y - ymin) / (ymax - ymin)) * (rows - 1))));
  const tgtRow = rowY(target);
  for (let c = 0; c < cols; c++) grid[tgtRow][c] = '-';        // target line
  for (const s of series) grid[rowY(s.x)][colT(s.t)] = '*';     // trajectory
  const lines = grid.map((r) => r.join(''));
  return `${ymax.toFixed(1).padStart(6)} |${lines[0]}\n` +
    lines.slice(1, -1).map((l) => `       |${l}`).join('\n') +
    `\n${ymin.toFixed(1).padStart(6)} |${lines[lines.length - 1]}\n` +
    `       +${'-'.repeat(cols)}\n        0${' '.repeat(cols - 6)}${T.toFixed(2)}s`;
}

function fmt(v, d = 1) { return v == null ? '  n/a' : v.toFixed(d); }

// ===================== experiments =====================
const TIGHTS = [0.0, 0.25, 0.5, 0.75, 1.0];

function run() {
  console.log('# followStep offline simulation  (faithful port of src/main.cpp)\n');

  // --- 1. tightness sweep on a representative translation step (20 mm heave) ---
  console.log('## 1. Tightness sweep -- 20 mm translation step @ 100 Hz');
  console.log('tight |  vmax | accelT | jerkT |    K  | rise0.9 | settle | peakV (%vmax) | ring(back)');
  console.log('------+-------+--------+-------+-------+---------+--------+---------------+----------');
  for (const tight of TIGHTS) {
    const r = simStep({ target: 20, tight, hz: 100 });
    const pv = (r.m.peakV / r.m.vmax) * 100;
    console.log(
      ` ${tight.toFixed(2)} | ${fmt(r.p.vmaxT, 0).padStart(5)} | ${fmt(r.p.accelT, 3)} | ${fmt(r.p.jerkT, 3)} | ${fmt(r.p.K, 2).padStart(5)} |` +
      ` ${fmt(r.m.riseT, 3).padStart(6)}s | ${fmt(r.m.settleT, 3)}s | ${fmt(r.m.peakV, 1).padStart(5)} (${fmt(pv, 0).padStart(3)}%) | ${fmt(r.m.ring, 1)} mm/s`);
  }

  // --- 2. step size crossover: when does the move become vmax-limited vs ease-out-limited? ---
  console.log('\n## 2. Step-size crossover @ tight=0.5, 100 Hz (peakV vs vmax tells the regime)');
  console.log('  vmax=77.5 mm/s, K=3.64/s -> ease-out caps speed at K*err; vmax caps big moves.');
  console.log('step(mm) | peakV | %vmax | regime          | settle');
  console.log('---------+-------+-------+-----------------+-------');
  for (const step of [2, 5, 10, 20, 30, 50]) {
    const r = simStep({ target: step, tight: 0.5, hz: 100 });
    const pv = (r.m.peakV / r.m.vmax) * 100;
    const regime = pv > 97 ? 'vmax-limited' : 'ease-out-limited';
    console.log(`  ${String(step).padStart(5)}  | ${fmt(r.m.peakV, 1).padStart(5)} | ${fmt(pv, 0).padStart(4)}% | ${regime.padEnd(15)} | ${fmt(r.m.settleT, 3)}s`);
  }

  // --- 3. ease-out proof: velocity collapses linearly with remaining error near target ---
  console.log('\n## 3. Ease-out proof @ tight=0.5 -- reference velocity vs remaining error');
  console.log('  expectation: as err->0, v -> K*err (linear slow-down, no hard stop). K=3.64.');
  console.log('  err(mm) | v(mm/s) | K*err | note');
  console.log('  --------+---------+-------+-----');
  {
    const p = deriveTight(0.5);
    const dt = dtForHz(100);
    let s = { x: 0, v: 0, a: 0 };
    const probes = [20, 10, 5, 2, 1, 0.5, 0.1];
    let pi = 0;
    for (let k = 0; k < 2000 && pi < probes.length; k++) {
      const err = 20 - s.x;
      if (err <= probes[pi]) {
        const Kerr = p.K * err;
        const lim = Math.abs(s.v) <= Kerr * 1.05 ? 'ease-out binding' : '';
        console.log(`  ${fmt(err, 2).padStart(6)}  | ${fmt(s.v, 2).padStart(6)}  | ${fmt(Kerr, 2).padStart(5)} | ${lim}`);
        pi++;
      }
      s = axisStep(s, 20, dt, p, false);
    }
  }

  // --- 4. moving-target tracking lag: how fast can the target move before falling behind? ---
  console.log('\n## 4. Moving-target tracking lag -- constant-velocity ramp @ 100 Hz');
  console.log('  steady-state lag ~ vTarget*tau until vTarget>vmax, then ref saturates (lag grows).');
  for (const tight of [0.25, 0.5, 1.0]) {
    const p = deriveTight(tight);
    console.log(`\n  tight=${tight}  (vmaxT=${p.vmaxT.toFixed(0)} mm/s, tau=${p.tau.toFixed(3)} s):`);
    console.log('    vTarget(mm/s) | lag(mm) | predicted vTarget*tau | status');
    console.log('    --------------+---------+-----------------------+-------');
    for (const vt of [10, 20, 40, 80, 120]) {
      const r = simRampLag({ vTarget: vt, tight, hz: 100 });
      const pred = vt * p.tau;
      const sat = vt > p.vmaxT ? 'SATURATED (lag unbounded)' : 'tracking';
      console.log(`    ${String(vt).padStart(9)}     | ${fmt(r.lag, 1).padStart(6)}  | ${fmt(pred, 1).padStart(11)}           | ${sat}`);
    }
  }

  // --- 5. rate-agnostic check: 100 vs 150 Hz on the same step ---
  console.log('\n## 5. Control-rate sensitivity -- same 20 mm step, tight=0.5, 100 vs 150 Hz');
  console.log('  (if response is ~identical, raising the loop rate is safe for FEEL; it only');
  console.log('   refines integration, the jerk/accel/vel limits are physical not per-tick.)');
  console.log('  hz  | settle | peakV  | peakA   | ring(back)');
  console.log('  ----+--------+--------+---------+----------');
  for (const hz of [100, 150]) {
    const r = simStep({ target: 20, tight: 0.5, hz });
    console.log(`  ${String(hz).padStart(3)} | ${fmt(r.m.settleT, 3)}s | ${fmt(r.m.peakV, 1).padStart(5)} | ${fmt(r.m.peakA, 0).padStart(6)} | ${fmt(r.m.ring, 1)} mm/s`);
  }

  // --- 7. brake-derate sweep: does softer braking kill the overshoot ring + restore monotonicity? ---
  console.log('\n## 7. vBrake derate sweep -- settle(s) & ring(mm/s back) vs tight, 20 mm step @ 100 Hz');
  console.log('  overshoot ring near target comes from discrete-time braking (v just exceeds the');
  console.log('  vBrake=sqrt(2*amax*err) envelope). Derate vBrake -> brake earlier. brakeScale=1.0=firmware.');
  console.log('  scale |  t=0.25      | t=0.50      | t=0.75      | t=1.00      | settle monotonic?');
  console.log('  ------+-------------+-------------+-------------+-------------+------------------');
  for (const bs of [1.0, 0.9, 0.8, 0.7]) {
    const rows = [0.25, 0.5, 0.75, 1.0].map((tg) => simStep({ target: 20, tight: tg, hz: 100, brakeScale: bs }).m);
    let mono = true;
    for (let i = 1; i < rows.length; i++) if (rows[i].settleT != null && rows[i - 1].settleT != null && rows[i].settleT > rows[i - 1].settleT + 1e-6) mono = false;
    const cells = rows.map((m) => `${fmt(m.settleT, 2).padStart(4)}s/${fmt(m.ring, 0).padStart(3)}`).join(' | ');
    console.log(`  ${bs.toFixed(2)}  | ${cells} | ${mono ? 'YES' : 'no'}`);
  }
  console.log('  cell = settle / ring(mm/s backward).  (snap-eps sweep separately showed eps has');
  console.log('   no effect on settle-to-0.05mm -- the 3um tail is compute hygiene, not the cause.)');

  // --- 8. ease-out gain derate: the actual lever for the mid-tight overshoot ring ---
  console.log('\n## 8. Ease-out gain (K) derate -- ring & rise & settle, 20 mm step @ 100 Hz');
  console.log('  K binds the approach (not vBrake); v lags the shrinking K*err target -> overshoot.');
  console.log('  kScale<1 widens the ease-out so v can track it. Watch ring fall vs rise cost.');
  console.log('  kScale | tight |  rise  | settle | ring(back) ');
  console.log('  -------+-------+--------+--------+-----------');
  for (const ks of [1.0, 0.8, 0.6]) {
    for (const tg of [0.5, 0.75]) {
      const r = simStep({ target: 20, tight: tg, hz: 100, kScale: ks });
      console.log(`   ${ks.toFixed(1)}   |  ${tg.toFixed(2)} | ${fmt(r.m.riseT, 3)}s | ${fmt(r.m.settleT, 3)}s | ${fmt(r.m.ring, 1).padStart(5)} mm/s`);
    }
  }

  // --- 9. jerk-aware stop fix ACROSS THE REAL LOOP RATES THE OPERATOR USES (100/400/500 Hz) ---
  // The over-brake ring is a discrete-time artifact: finer dt (higher rate) shrinks it on
  // its own. So the fix only earns its place if the ring is still real at 400-500 Hz, which
  // is where the platform actually runs. Sweep rate x tight to find out.
  console.log('\n## 9. Jerk-aware stop -- ring(mm/s back) OFF/ON across real loop rates, 20 mm step');
  console.log('  (the ring is a discrete artifact; higher rate alone shrinks it. does the fix still matter?)');
  console.log('   hz  | tight | rise OFF/ON | settle OFF/ON |  ring(back) OFF/ON');
  console.log('  -----+-------+-------------+---------------+-------------------');
  for (const hz of [100, 400, 500]) {
    for (const tg of [0.25, 0.5, 0.75, 1.0]) {
      const off = simStep({ target: 20, tight: tg, hz, jerkStop: false }).m;
      const on = simStep({ target: 20, tight: tg, hz, jerkStop: true }).m;
      console.log(`  ${String(hz).padStart(4)} |  ${tg.toFixed(2)} | ${fmt(off.riseT, 3)}/${fmt(on.riseT, 3)} | ${fmt(off.settleT, 3)}/${fmt(on.settleT, 3)} | ${fmt(off.ring, 1).padStart(5)} / ${fmt(on.ring, 1).padStart(5)} mm/s`);
    }
  }

  // --- 6b. settle anomaly probe: final-approach detail (why is settle non-monotonic?) ---
  console.log('\n## 6b. Final-approach probe -- 20 mm step, err & v sampled every 0.1 s');
  console.log('  (binding term: which of vmax / vBrake / K*err caps the commanded speed)');
  for (const tight of [0.5, 0.75]) {
    const p = deriveTight(tight);
    const dt = dtForHz(100);
    const amax = p.vmaxT / p.accelT;
    let s = { x: 0, v: 0, a: 0 };
    console.log(`\n  tight=${tight} (K=${p.K.toFixed(2)}, amax=${amax.toFixed(0)}):`);
    console.log('     t   |  err   |   v    | vmax/vBrake/K*err  | binding');
    let nextSample = 0;
    for (let k = 0; k < 400; k++) {
      const t = k * dt;
      const err = 20 - s.x;
      if (t >= nextSample - 1e-9) {
        const vBrake = Math.sqrt(Math.max(0, 2 * amax * Math.abs(err)));
        const vApp = p.K * Math.abs(err);
        const trio = [p.vmaxT, vBrake, vApp];
        const names = ['vmax', 'vBrake', 'K*err'];
        const bi = trio.indexOf(Math.min(...trio));
        console.log(`   ${t.toFixed(2)}  | ${err.toFixed(3).padStart(6)} | ${s.v.toFixed(2).padStart(6)} | ${p.vmaxT.toFixed(0).padStart(4)}/${vBrake.toFixed(1).padStart(5)}/${vApp.toFixed(1).padStart(5)} | ${names[bi]}`);
        nextSample += 0.1;
        if (err < 0.005) break;
      }
      s = axisStep(s, 20, dt, p, false);
    }
  }

  // --- 6. trajectory shape for the default working point ---
  console.log('\n## 6. Trajectory shape -- 20 mm step @ tight=0.5, 100 Hz (x vs t)');
  const shape = simStep({ target: 20, tight: 0.5, hz: 100, maxT: 1.2 });
  console.log(asciiPlot(shape.traj.filter((_, i) => i % 1 === 0), 20));
}

run();
