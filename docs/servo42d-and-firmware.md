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
