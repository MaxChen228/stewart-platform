# SERVO42D And Firmware Notes

## Transport

- CAN 2.0A
- standard 11-bit ID
- `500 kbps`
- MCP2515 crystal `8 MHz`

## CRC

CRC must include CAN ID:

`CRC = (CAN_ID + Byte1 + ... + Byte(n-1)) & 0xFF`

This is non-negotiable. Earlier confusion here causes silent failures.

## Important Commands

- `0x30` read encoder carry + current value
- `0xF3` enable / disable
- `0xF6` speed mode
- `0xF7` emergency stop
- `0xFD` relative position by pulses
- `0x92` set zero point exists in protocol but is not yet used as the project’s primary calibration path
- `0x9B` hold current is important for future stability tuning

## Firmware Scan Rate

Current firmware loop behavior:

- `handleSerial()` every loop
- telemetry scan block every `100 ms`

Current effect:

- hardware telemetry rate is about `10 Hz`
- this is a practical upper bound for current UI responsiveness unless push-based transport is added

## Relative Position Mode Lessons

Two critical fixes were required:

1. relative move delta must be referenced from measured encoder angle
   - not from last commanded target
   - otherwise drift accumulates permanently

2. `0xFD` direction bit and pulse scale were initially wrong
   - observed hardware behavior showed inverted direction
   - project now uses `SERVO_PULSES_PER_REV = 3200`

These changes stabilized "move out and return" behavior.

## Encoder And Pulses Are Not The Same Unit

Important distinction:

- encoder telemetry is based on encoder counts / turns
- motion commands use driver pulse units

These cannot be treated as interchangeable scaling domains.

## What The Encoder Actually Means In This Project

The encoder data in this project must be understood in four layers:

1. `raw encoder count / raw angle`
   - this comes from the motor telemetry
   - currently derived from `0x30` carry + single-turn value
   - this is the closest thing to the motor's native accumulated position signal

2. `logical angle`
   - this is what the firmware reports as `deg`
   - it is computed as:
   - `logical angle = raw angle - zeroOffsetDeg`
   - this is not a factory-defined world angle

3. `calibration zero`
   - this is the project-defined reference
   - when `CALIBRATE` is executed, the current physical posture becomes logical zero
   - in other words, zero is assigned by workflow, not discovered automatically by the motor

4. `motor internal zero`
   - the SERVO42D protocol also exposes commands such as `0x92 set zero point`
   - this is a motor-side persistent reference concept
   - the project currently does not use motor internal zero as its primary truth model

Practical consequence:

- the encoder does not magically know "this is mechanically vertical-up"
- it only knows its measured accumulated position
- the project must still define what physical posture should count as logical zero

This is why a machine that has drifted away from its reference can still report a perfectly meaningful encoder value, while no longer being at the intended physical posture.

## Telemetry Fields In Use

Current firmware emits:

- `deg`
- `rawDeg`
- `encoderCount`
- `singleTurnCount`
- `targetDeg`
- `zeroOffsetDeg`
- `zeroSeq`
- `enabled`
- `moving`

These fields are required by the host debug and calibration tools.

## Firmware Constraints

Current firmware is still a motion executor / telemetry bridge.

It does not own Stewart IK.

That is intentional:

- simulation and hardware must share one kinematics source
- putting IK in firmware would duplicate logic and cause drift between target and visualization
