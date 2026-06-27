# Pose Compensation Plan

Date: 2026-06-27

## Short Answer

`zBias=4.5mm` is not the final control strategy. It is an experimental anchor
showing that static height compensation should live above the motor joint loop.

The professional path is:

```text
desired pose
  -> static compensation model
  -> jerk-limited trajectory
  -> pose-space LQI / LQR / MPC residual controller
  -> IK
  -> SERVO42D pure-P or low-Ki inner loop
```

## Why Not A Fixed Constant

A fixed height bias only works near one tested operating point:

- heave around +10mm from current home
- platform near level
- current load and support condition
- current battery/power/mechanical setup

It will not automatically generalize to different heights, tilt angles, payloads,
or dynamic motions. Treating it as the final answer would hide a real model
inside a magic number.

## What The Experiment Proved

The clean heave runs showed:

- Joint `Ki=20` reaches target height, but increases cross-axis coupling.
- Pure-P is dynamically cleaner, but sags under static load.
- Pure-P plus command-side Z bias recovers height while keeping coupling low.

So the lesson is not "use a constant forever." The lesson is:

```text
use a clean inner loop, then compensate static load in pose space
```

## Target Compensation Model

Start simple:

```text
z_comp = f(z_target, roll_target, pitch_target, payload_state)
```

First candidate:

```text
z_comp = a0 + a1 * heave + a2 * roll^2 + a3 * pitch^2
```

If data is sparse, use a small lookup table instead:

```text
(heave, roll, pitch) -> z_comp
```

Keep the model visible and versioned. Do not bury it in motor PID.

## Data Needed Next

Use `research_trial --record-main-only` and pure-P:

| Sweep | Purpose |
|---|---|
| heave +5, +10, +20mm | fit height sag curve |
| small roll/pitch holds | see whether tilt changes sag |
| repeat +10mm on different days | check repeatability |
| payload/support variations | separate load-dependent bias |

## Controller Ladder

1. Static feedforward model handles predictable sag.
2. Pose-space LQI handles remaining slow steady-state error.
3. LQR handles coupled local dynamics once the model is reliable.
4. MPC handles constraints and timed dynamic tasks.

This keeps each layer honest: feedforward explains static physics, LQI removes
residual bias, and MPC does constrained planning instead of compensating for an
unmodeled constant.
