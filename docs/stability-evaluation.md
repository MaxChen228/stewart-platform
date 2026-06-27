# Stability Evaluation Method

Date: 2026-06-27

## Goal

Evaluate arbitrary platform behavior with low cognitive load and low risk of
self-deception.

The scorer must work for:

- simple Z moves
- full 6-DOF pose trajectories
- command-side compensation / algorithms
- disturbance injection through `W`
- future LQI/LQR/MPC comparisons

## Core Rule

Never confuse actuator command with task reference.

An algorithm may intentionally send a biased or transformed command to the
firmware. The score must compare measured pose against the intended reference
trajectory, not blindly against the internal command.

Example:

```text
reference target: z = 143
controller command: z = 147.5  (feedforward bias)
measured pose: z = 142.93
```

This is good tracking of the task reference, not a 4.57mm command error.

## Score Components

`sysid/stability_score.js` emits separate components before the final score:

| Component | Meaning |
|---|---|
| `trackingCost` | P95 error on commanded/reference axes |
| `crossAxisCost` | high-pass residual on axes that were not part of the task |
| `oscillationCost` | high-pass residual over all axes |
| `recoveryCost` | disturbance recovery time when `W` events exist |
| `qualityPenalty` | encoder/FK/CAN/data quality penalty |

Final:

```text
stability = 100 - weighted_cost
```

The final score is only a sorting aid. The components are the real diagnosis.

## Oscillation Definition

Oscillation is not "pose moved a lot." A commanded Z sweep is supposed to move Z.

The scorer removes slow motion with a moving average and measures high-pass
residuals. This avoids mislabeling normal upward/downward travel as oscillation.

For a Z-only task, cross-axis stability is mostly:

```text
X, Y, Roll, Pitch, Yaw high-pass residual
```

This catches the wobble seen in the dashboard waveforms.

## Disturbance Evaluation

`W` commands are parsed as disturbance events.

For each event:

1. Estimate baseline pose before the pulse.
2. Measure post-pulse pose magnitude.
3. Record peak response and recovery time.

The same score can compare:

- no disturbance trajectory
- single pulse disturbance
- battery disturbance sequence
- future UI disturbance modes

## Data Quality

CAN and telemetry are part of the score, but normalized by duration where
appropriate. A 40s trial should not be punished merely because it lasted longer
than a 20s trial.

Current quality signals:

- encoder `ok` fraction
- FK failure fraction
- host telemetry gaps
- CAN `EFLG`
- CAN RX drop rate
- CAN TX failure rate

## Current Z Sweep Baseline

Range:

```text
relative z +10mm <-> +62mm
```

Results so far:

| Run | one-way duration | score | cross-axis cost | peak cross residual | note |
|---|---:|---:|---:|---:|---|
| Ki=20 clean | 6s | 52.97 | 2.86 | 10.74 | faster, more cross-axis residual |
| Ki=20 clean slow | 10s | 57.66 | 1.70 | 7.92 | slower improves wobble but does not fix tracking/CAN |

Interpretation:

Slower motion helps, so trajectory excitation is real. It does not solve the
whole problem; large-range Z motion still needs better compensation/control and
a cleaner CAN data plane.

## Usage

```bash
npm run stability:score -- sysid/data/<record>.jsonl --axis z
npm run z:sweep -- --live --i-am-at-rig --low 10 --high 62 --cycles 1 --ms 6000
npm run research:index
```
