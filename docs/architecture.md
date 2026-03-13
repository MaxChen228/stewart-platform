# Architecture

## Layers

This project now has four distinct layers:

1. `src/main.cpp`
   - ESP32 firmware
   - MCP2515 CAN transport
   - polls motor encoder telemetry
   - executes relative position moves
   - owns hardware-side zeroing via `CALIBRATE`

2. `host/kinematics.py`
   - shared Stewart platform geometry + IK/FK logic
   - computes target actuator angles from `(roll, pitch, yaw, x, y, z)`
   - computes actual platform pose from encoder-derived actuator angles

3. `host/app.py`
   - serial bridge + HTTP server
   - runtime state, calibration state, persistence
   - exposes `/api/state`, `/api/pose`, `/api/geometry`, `/api/command`

4. `web/*`
   - operator UI
   - pose control, geometry editing, simple motor status, 3D visualization

## Shared Truth Model

There are three different data spaces and they must not be conflated:

1. `theory pose`
   - UI input pose
   - `roll, pitch, yaw, x, y, z`

2. `target actuator`
   - output of IK
   - target servo angles / target motor angles

3. `actual actuator`
   - encoder-derived real actuator angles
   - used for actual platform reconstruction

The major lesson from development is that "simulation" is only trustworthy if actual actuator data is treated as its own truth source.

## Calibration vs Home

These are different and must remain different:

1. `calibration`
   - six cranks manually placed vertical-up
   - encoder reference only
   - corresponds to `geometry.calibration_z`
   - this point is not a good operating point

2. `home`
   - stable working pose
   - currently defined near "cranks horizontal"
   - corresponds to `geometry.home_z`

Do not use the calibration pose as the operating home.

## Persistence

Runtime state is persisted in:

- `.runtime_state.json`

This stores:

- geometry
- current pose
- latest calibration snapshot

Reason:

- dashboard/server restarts must not forget calibrated zero offsets
- dashboard restarts previously caused target/actual mismatch because host state fell back to defaults while hardware stayed calibrated

## Current Default Geometry

The project currently assumes:

- `base_radius = 152`
- `base_angle = 18.92`
- `platform_radius = 103`
- `platform_angle = 28.07`
- `lower_leg = 65`
- `upper_leg = 165`
- `stepper_plane_angles = [-90, 90, 30, 210, -210, -30]`
- `motor_signs = [1, -1, 1, -1, 1, -1]`
- `servo_pulses_per_rev = 3200`

These values are good enough for iteration, but not yet verified as a final metrology-grade model of the real machine.
