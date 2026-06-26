# Telemetry Schema v2 Draft

Date: 2026-06-27

Goal: make every failure class visible from telemetry instead of guessing.

This is a target schema. Existing firmware can keep emitting old fields while new fields are added incrementally.

## Top-Level Fields

| Field | Meaning |
|---|---|
| `t` | Firmware millis timestamp |
| `dt` | Last control-cycle elapsed time in ms |
| `lhz` | Measured control loop frequency over telemetry window |
| `ok` | Count of valid encoder raw values |
| `z` | Encoder calibration loaded/valid |
| `pos` | Position control enabled |
| `cm` | Control mode |
| `fl` | Follow mode enabled |

## Motion State

| Field | Meaning |
|---|---|
| `a[6]` | Current motor angles, deg |
| `r[6]` | Current raw encoder values |
| `hold[6]` | HOLD target motor angles, deg |
| `herr[6]` | HOLD angle error, deg |
| `hmax` | Max HOLD error since HOLD start |
| `pose[6]` | Current FK pose, when available |
| `target[6]` | Current target pose |
| `vel[6]` | Estimated motor angle velocity, deg/s |

## Timing

Target object: `tim`.

| Field | Meaning |
|---|---|
| `tim.dtMin` | Min control dt in telemetry window |
| `tim.dtMax` | Max control dt in telemetry window |
| `tim.dtAvg` | Avg control dt in telemetry window |
| `tim.dtP95` | Approx/pinned p95 if histogram is available |
| `tim.f5us` | Last 6-motor F5 burst duration |
| `tim.f5max` | Max F5 burst duration in telemetry window |
| `tim.canReadUs` | Polling encoder read duration |
| `tim.teleMs` | Telemetry window duration |

## CAN Health

Target object: `can`.

| Field | Meaning |
|---|---|
| `can.backend` | `mcp2515` or `twai` |
| `can.bitrate` | 500000 or 1000000 |
| `can.state` | normal / warning / passive / bus-off |
| `can.ef` | MCP2515 EFLG or backend equivalent |
| `can.tx` | TX error counter or equivalent |
| `can.rx` | RX error counter or equivalent |
| `can.rxDrop` | Backend RX queue drop count |
| `can.txFail` | Backend TX failure count |
| `can.qDepth` | RX queue depth at telemetry time |
| `can.qMax` | Max RX queue depth in telemetry window |
| `can.loadPct` | Estimated bus load from frame counters |

Existing `bus` object remains useful:

| Field | Meaning |
|---|---|
| `bus.f5` | F5 command frames in telemetry window |
| `bus.q` | Encoder query frames in telemetry window |
| `bus.rx` | Encoder auto-return frames received |
| `bus.per[6]` | Per-motor encoder auto-return received |
| `bus.ovr` | RX overflow windows/events |
| `bus.mxd` | Max single drain count |

## Controller Health

Target object: `ctl`.

| Field | Meaning |
|---|---|
| `ctl.mode` | hold / joint / task_pd / lqr / mpc |
| `ctl.sat` | Saturation/limit hit count |
| `ctl.ikFail` | IK invalid count |
| `ctl.fkFail` | FK fail count |
| `ctl.coordMax` | Max absolute F5 coordinate command |
| `ctl.profile` | none / smoothstep / s_curve / follow |

## Why This Matters

The platform has several distinct failure modes:

- CAN bandwidth saturation.
- MCP2515/TWAI RX queue overflow.
- TX retry/error-passive behavior.
- Control loop jitter.
- IK/FK invalidity.
- Trajectory jerk exciting the mechanics.
- Controller instability.
- Mechanical friction/deadband.

Telemetry v2 should make these separable.
