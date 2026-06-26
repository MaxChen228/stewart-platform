# Control Architecture Roadmap

Date: 2026-06-27

## Final Ambition

Make the platform smooth, repeatable, and fast enough for dynamic tasks such as striking a ping-pong ball against a wall.

Loop frequency is necessary but not sufficient. The full stack is:

```text
vision / state estimation
  -> interception planning
  -> trajectory generation
  -> platform controller
  -> IK / motor command scheduler
  -> SERVO42D internal position/FOC loops
```

## Frequency Targets

| Layer | Target |
|---|---:|
| SERVO42D internal FOC/current loops | already kHz-class inside motor |
| Stewart outer control loop | 200 Hz first, 300 Hz stretch |
| Motion command streaming | 200-300 Hz when needed |
| Vision tracking | 120-240 fps |
| High-level planning | 30-120 Hz |

## Controller Ladder

### 1. Baseline HOLD / Pure-P Motor Targeting

Purpose:

- Establish communication health.
- Measure stiffness, noise, drift, and disturbance response.
- Keep a known-simple safe baseline.

Exit criteria:

- HOLD stays bounded.
- CAN errors understood.
- Repeatable step/push response data exists.

### 2. Smooth Trajectory Generation

This should happen before advanced control.

Add:

- S-curve or jerk-limited pose trajectories.
- Velocity/acceleration/jerk limits.
- Motor-space command limit checks.
- Workspace prechecks before motion starts.

Reason:

Many "controller" problems are actually hard target trajectories exciting the mechanics.

### 3. Pose-Space PD + Feedforward

Use FK pose estimate when reliable.

State:

```text
e_pose = target_pose - measured_pose
de_pose = target_velocity - measured_velocity
u = Kp e + Kd de + feedforward
```

Feedforward can include desired velocity/acceleration from the motion profile.

Exit criteria:

- Better push recovery than HOLD.
- Better tracking of smooth pose paths.
- No mode fallback surprises.

### 4. System Identification

Required before LQR/MPC.

Measure:

- Delay from command to encoder response.
- Small step response per pose axis.
- Chirp/sine response.
- Cross-axis coupling.
- Deadband and friction.
- Saturation boundaries.

Produce a local model:

```text
x = [pose_error, pose_velocity]
u = pose_delta or motor_target_delta
x[k+1] = A x[k] + B u[k]
```

### 5. LQR / LQI

Use when a local linear MIMO model exists.

Good for:

- Coupled axes.
- Choosing tradeoffs between position error, velocity, and command effort.
- Stabilizing around a known operating region.

LQI adds integral action for steady-state offsets.

Risks:

- Bad model means bad controller.
- Saturation must be handled outside vanilla LQR.

### 6. MPC

Use when constraints dominate:

- Workspace limits.
- Motor coordinate limits.
- Velocity/acceleration limits.
- Contact timing for ball strikes.

MPC is more appropriate than LQR for "hit this pose at this time with this velocity without violating limits".

Risks:

- More compute.
- More tuning.
- Needs robust model and fallback.

### 7. RL

Do not put RL directly in charge of motor commands.

Useful role:

- High-level strike strategy.
- Selecting intercept pose, normal, and desired contact velocity.
- Adapting to ball spin or imperfect wall bounce.

Safe architecture:

```text
RL/high-level policy -> desired intercept state -> MPC/LQR/PID safety controller -> motors
```

## Ping-Pong Task Decomposition

Minimum stack:

1. Camera detects ball at 120-240 fps.
2. State estimator predicts ball position/velocity/spin proxy.
3. Bounce model predicts wall/platform contact.
4. Planner chooses intercept time, platform pose, and normal velocity.
5. Trajectory generator makes a jerk-limited platform motion.
6. Controller tracks it at 200-300 Hz.
7. Safety layer rejects unreachable or too-aggressive strikes.

## Immediate Engineering Priorities

1. TWAI backend at 500K, then 1M.
2. Telemetry schema v2.
3. Repeatable CAN/control benchmark.
4. Jerk-limited trajectory generator.
5. Step/chirp/push system-ID data.
6. Pose-space PD + feedforward baseline.
7. LQR model fitting only after data quality is good.
