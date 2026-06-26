# MPC Readiness: SERVO42D + Stewart Platform

Date: 2026-06-27

## Short Answer

SERVO42D is sufficient for a first practical outer-loop MPC, but not for
torque-level industrial servo MPC.

Recommended architecture:

```text
state estimator / FK
  -> outer-loop MIMO controller or MPC at 100-300 Hz
  -> IK + command scheduler
  -> SERVO42D F5 absolute coordinate targets
  -> SERVO42D internal 10 kHz position/speed/FOC loops
```

Do not plan on direct torque control. Treat each SERVO42D as a smart position
actuator with adjustable target, speed, acceleration, PID, and encoder feedback.

## Hardware Capabilities That Help MPC

| Need | SERVO42D support | Project status |
|---|---|---|
| High-rate position command | `0xF5` absolute coordinate motion supports real-time target updates | Implemented |
| Position feedback | `0x35` raw accumulated encoder, 14-bit/rev | Implemented |
| Periodic feedback | `0x01` auto-return can stream read-only parameters | Implemented |
| Velocity observation | `0x32` real-time speed | Not yet in telemetry |
| Internal tracking error | `0x39` position angle error | Not yet in telemetry |
| Multi-motor synchronized start | `0x4A` sync flag + broadcast `0x4B` trigger | Not yet used |
| Command response suppression | `0x8C` can disable start/completion replies | Implemented via `C` command |
| 1M CAN bitrate | `0x8A` bitRate `0x03` | Planned, requires controlled migration |
| Fast inner loops | position/speed 10 kHz, torque 20 kHz inside motor | Available but black-box |

## Current Bottlenecks

### 1. MCP2515 is the immediate weak link

MCP2515 has two RX buffers and sits behind SPI. It can work for experiments, but
it is not the long-term interface for 200-300 Hz MIMO control with streamed
feedback.

Observed constraints:

- 500K, MCP2515, auto-return can overflow RX even when high-level telemetry looks usable.
- F5 plus encoder stream at high loop rates pushes TEC/overflow risk.
- TWAI is needed for a deeper hardware RX queue and lower host-side latency.

### 2. 500K can be usable, 1M is the real target

Budget estimates with ACK off:

| Scenario | Bus load |
|---|---:|
| 500K, 200 Hz F5 + 200 Hz encoder | ~63% |
| 1M, 200 Hz F5 + 200 Hz encoder | ~31% |
| 1M, 300 Hz F5 + 300 Hz encoder | ~47% |

So 200 Hz is plausible at 500K only if the host interface is clean. 1M + TWAI is
the better foundation for 200-300 Hz control.

### 3. The actuator is position-mode, not torque-mode

This changes the control design:

- Good: safer, simpler, internal loops are fast.
- Bad: MPC cannot directly command platform force/torque.
- Best first model: command target deltas and predict measured pose response.

Useful model form:

```text
x = [pose_error, pose_velocity, maybe actuator_lag]
u = target_pose_delta or motor_angle_delta
x[k+1] = A x[k] + B u[k]
```

## What To Build First

### Stage 1: Coupling Identification

Measure how each pose-axis command excites every pose-axis response:

```text
command z step/chirp -> observe x,y,z,roll,pitch,yaw
command x step/chirp -> observe x,y,z,roll,pitch,yaw
...
```

Output:

- cross-axis gain matrix
- delay estimate
- dominant oscillation frequency
- damping/ringdown estimate
- whether Ki helps static error but hurts dynamic coupling

### Stage 2: Decoupling Feedforward

Before full MPC, add a simple inverse-coupling compensation:

```text
desired_pose_delta -> learned correction -> IK -> F5
```

This should reduce "Z command creates Y/roll/yaw wobble" without changing the
whole controller.

### Stage 3: LQR/LQI

Once a local linear model is reliable, LQR/LQI is the first real MIMO controller.
LQI is especially relevant if Ki=20 improves final target accuracy but dynamic
oscillation remains.

### Stage 4: MPC

Use MPC when constraints matter:

- motor coordinate limits
- velocity/acceleration limits
- workspace boundaries
- intercept timing for ping-pong

First MPC should run on the host or a companion computer. ESP32 firmware should
remain the deterministic safety and command-scheduling layer until the model and
controller are proven.

## Decision

SERVO42D does not block MPC. The gating items are:

1. TWAI transceiver path verified at 500K.
2. 1M CAN migration verified.
3. Pose/FK telemetry and command-response recordings are clean.
4. A local coupling model exists.
5. A safe fallback remains: HOLD or disable on bad telemetry/model residual.

## Safe Landing Convention

User-provided mechanical support landing flow. Home is configurable in the UI
via "存Home" and is persisted through the server `/api/home` endpoint.

```text
current home relative pose: [0, 0, +28 mm heave, 0, 0, 0]
landing relative pose: [0, 0, +10 mm heave, 0, 0, 0]
```

Operational sequence:

```text
H
P <configured home pose>
P 0 0 NEUTRAL_Z+10 0 0 0
D
```

The platform should then release onto the prepared support fixture. This is now
available as:

```bash
npm run safe:land -- --live
```

Live system-ID scripts should use this landing sequence before release whenever
the support fixture is installed.
