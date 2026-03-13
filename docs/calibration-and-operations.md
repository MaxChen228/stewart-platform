# Calibration And Operations

## Operator Workflow

The intended workflow is:

1. `Disable All`
   - operator manually trims all six cranks vertical-up

2. `Calibrate`
   - firmware zeros encoder reference at the current physical posture
   - host updates zero offsets and calibration state
   - this defines actuator reference, not a normal working pose

3. `Home`
   - system returns to stable working pose
   - this should be the default operating start point

4. normal motion
   - use pose controls around `home`
   - avoid normal operation near calibration height

## Important Commands

### UI / HTTP commands

- `enable_all`
- `disable_all`
- `stop`
- `calibrate`
- `home`
- `apply_pose`

### Firmware serial commands

- `ENABLE:1`
- `ENABLE:0`
- `STOP`
- `MOVE:a1,a2,a3,a4,a5,a6`
- `CALIBRATE`

## Calibration Semantics

### Hardware-first calibration

Calibration must be hardware-first, not theory-first.

Correct meaning:

- the operator physically places all cranks vertical-up
- that exact physical posture becomes the encoder reference

Incorrect meaning:

- assume a theoretical vertical-up pose from geometry
- force hardware to match that assumption

The project originally mixed these two meanings, which created large mismatches between target and actual states.

## Operations Scripts

### `ops.py`

Operational helper script. Main commands:

- `python3 ops.py status`
- `python3 ops.py watch`
- `python3 ops.py calibrate`
- `python3 ops.py goto-cal`
- `python3 ops.py validate`
- `python3 ops.py recalibrate`

Purpose:

- convert repeated bring-up / validation steps into deterministic workflows
- avoid ad hoc manual API calls

### `dashboard_ctl.py`

Dashboard lifecycle helper:

- `python3 dashboard_ctl.py start`
- `python3 dashboard_ctl.py stop`
- `python3 dashboard_ctl.py restart`
- `python3 dashboard_ctl.py status`

Purpose:

- avoid repeated "port 8080 already in use" failures
- make dashboard lifecycle reproducible

## Validation Strategy

Use these three checks after calibration:

1. `status`
   - confirm `actual` servo angles are near `0`

2. `goto-cal`
   - confirm machine can return to calibration reference

3. `validate`
   - run small Z sweeps such as `-5, -10, -15 mm`
   - verify return-to-target error remains small

## What Requires Reflash

If `src/main.cpp` changes, the ESP32 must be reflashed.

Typical firmware-affecting changes:

- CAN packet format
- direction bit handling
- pulses-per-rev conversion
- telemetry fields
- calibration logic

Host/UI-only changes do not require reflash.
